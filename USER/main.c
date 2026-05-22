#include "stm32f4xx.h"
#include "remote.h"
#include "actuator.h"
#include "vesc_can.h"
#include "line_follow.h"
#include <stdio.h>

volatile uint32_t DEBUG_AliveCount = 0;
volatile uint8_t DEBUG_PA2_Level = 0;
volatile uint32_t DEBUG_BootCFGR = 0;
volatile uint32_t DEBUG_BootCR = 0;
volatile uint32_t DEBUG_BootSystemCoreClock = 0;
volatile uint8_t DEBUG_SysclkSource = 0;
volatile uint8_t DEBUG_HSERdy = 0;
volatile uint8_t DEBUG_PLLRdy = 0;
volatile uint8_t DEBUG_DWT_Alive = 0;
volatile uint32_t DEBUG_DWT_TestDelta = 0;
volatile uint8_t DEBUG_LineFollowState = 0;
volatile uint32_t DEBUG_LineFollowEndUs = 0;

#define USER_KEY_DEBOUNCE_US        30000U
#define LINEFOLLOW_RUN_TIME_US      15000000U

#if defined(__CC_ARM)
#pragma import(__use_no_semihosting)

struct __FILE
{
    int handle;
};

FILE __stdout;

void _sys_exit(int x)
{
    (void)x;
}
#endif

int fputc(int ch, FILE *f)
{
    (void)f;

    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    {
    }
    USART_SendData(USART1, (uint8_t)ch);
    return ch;
}

static void Debug_USART1_Init(uint32_t bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = bound;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStructure);

    USART_Cmd(USART1, ENABLE);
}

static void Debug_CaptureBootState(void)
{
    uint32_t time_start;
    uint32_t time_end;
    volatile uint32_t i;

    SystemCoreClockUpdate();

    DEBUG_BootCR = RCC->CR;
    DEBUG_BootCFGR = RCC->CFGR;
    DEBUG_BootSystemCoreClock = SystemCoreClock;
    DEBUG_SysclkSource = (uint8_t)((RCC->CFGR & RCC_CFGR_SWS) >> 2);
    DEBUG_HSERdy = ((RCC->CR & RCC_CR_HSERDY) != 0U) ? 1U : 0U;
    DEBUG_PLLRdy = ((RCC->CR & RCC_CR_PLLRDY) != 0U) ? 1U : 0U;

    time_start = Remote_GetUs();
    for (i = 0; i < 1000U; i++)
    {
    }
    time_end = Remote_GetUs();
    DEBUG_DWT_TestDelta = time_end - time_start;
    DEBUG_DWT_Alive = (DEBUG_DWT_TestDelta != 0U) ? 1U : 0U;
}

int main(void)
{
    uint32_t last_print_us = 0;
    uint32_t now_us;
    uint32_t user_key_change_us = 0;
    uint8_t user_key_last_raw = 1U;
    uint8_t user_key_stable = 1U;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    Debug_USART1_Init(115200);
    Remote_Init();
    Debug_CaptureBootState();
    Actuator_Init();

    while (1)
    {
        DEBUG_AliveCount++;
        DEBUG_PA2_Level = (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2) != Bit_RESET) ? 1 : 0;
        now_us = Remote_GetUs();
        {
            uint8_t user_key_raw = (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_0) != Bit_RESET) ? 1U : 0U;

            if (user_key_raw != user_key_last_raw)
            {
                user_key_last_raw = user_key_raw;
                user_key_change_us = now_us;
            }

            if (user_key_raw != user_key_stable &&
                (uint32_t)(now_us - user_key_change_us) >= USER_KEY_DEBOUNCE_US)
            {
                user_key_stable = user_key_raw;
                if (user_key_stable == 0U && DEBUG_LineFollowState == 0U)
                {
                    LineFollow_SetEnabled(1U);
                    DEBUG_LineFollowState = 1U;
                    DEBUG_LineFollowEndUs = now_us + LINEFOLLOW_RUN_TIME_US;
                }
            }
        }

        if (DEBUG_LineFollowState != 0U &&
            (uint32_t)(now_us - DEBUG_LineFollowEndUs) < 0x80000000U)
        {
            LineFollow_SetEnabled(0U);
            DEBUG_LineFollowState = 0U;
            DEBUG_LineFollowEndUs = 0U;
        }

        Actuator_UpdateFromRC();
        Actuator_Task();
        VescCan_DebugPoll();

        if ((uint32_t)(now_us - last_print_us) >= 100000U)
        {
            last_print_us = now_us;
            printf("SYS:%u HSE:%u PLL:%u DWT:%u DLT:%u CORE:%u\r\n",
                   DEBUG_SysclkSource,
                   DEBUG_HSERdy,
                   DEBUG_PLLRdy,
                   DEBUG_DWT_Alive,
                   DEBUG_DWT_TestDelta,
                   DEBUG_BootSystemCoreClock);
        }
    }
}
