#include "remote.h"

volatile uint16_t RC_CH[RC_CHANNEL_COUNT] = {0};
volatile uint8_t RC_CH_Updated[RC_CHANNEL_COUNT] = {0};
volatile uint8_t RC_CH_Level[RC_CHANNEL_COUNT] = {0};
volatile uint32_t RC_CH_EdgeCount[RC_CHANNEL_COUNT] = {0};
volatile uint32_t RC_CH_RiseCount[RC_CHANNEL_COUNT] = {0};
volatile uint32_t RC_CH_FallCount[RC_CHANNEL_COUNT] = {0};

volatile uint16_t RC_CH1_Pulse = 0;
volatile uint8_t RC_CH1_Updated = 0;
volatile uint8_t RC_CH1_Level = 0;
volatile uint32_t RC_CH1_EdgeCount = 0;
volatile uint32_t RC_CH1_RiseCount = 0;
volatile uint32_t RC_CH1_FallCount = 0;

static volatile uint32_t rc_rise_us[RC_CHANNEL_COUNT] = {0};
static volatile uint32_t remote_timebase_us_high = 0;

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
    uint8_t port_source;
    uint8_t pin_source;
    uint32_t exti_line;
} RC_InputDef;

static const RC_InputDef rc_inputs[RC_CHANNEL_COUNT] =
{
    {GPIOA, GPIO_Pin_2, EXTI_PortSourceGPIOA, EXTI_PinSource2, EXTI_Line2},
    {GPIOA, GPIO_Pin_3, EXTI_PortSourceGPIOA, EXTI_PinSource3, EXTI_Line3},
    {GPIOA, GPIO_Pin_6, EXTI_PortSourceGPIOA, EXTI_PinSource6, EXTI_Line6},
    {GPIOA, GPIO_Pin_7, EXTI_PortSourceGPIOA, EXTI_PinSource7, EXTI_Line7},
    {GPIOA, GPIO_Pin_8, EXTI_PortSourceGPIOA, EXTI_PinSource8, EXTI_Line8},
    {GPIOC, GPIO_Pin_0, EXTI_PortSourceGPIOC, EXTI_PinSource0, EXTI_Line0},
};

static uint32_t Remote_GetAPB2TimerClock(void)
{
    uint32_t presc_bits;
    uint32_t apb2_div;
    uint32_t pclk2;

    presc_bits = (RCC->CFGR & RCC_CFGR_PPRE2) >> 13;

    if (presc_bits < 4U)
    {
        apb2_div = 1U;
    }
    else
    {
        apb2_div = 1U << ((presc_bits - 3U));
    }

    pclk2 = SystemCoreClock / apb2_div;
    if (apb2_div == 1U)
    {
        return pclk2;
    }

    return pclk2 * 2U;
}

static void Remote_Timebase_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    uint32_t tim_clk;
    uint16_t prescaler;

    SystemCoreClockUpdate();

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);

    tim_clk = Remote_GetAPB2TimerClock();
    prescaler = (uint16_t)((tim_clk / 1000000U) - 1U);

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = prescaler;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 0xFFFFU;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM10, &TIM_TimeBaseStructure);

    TIM_ClearFlag(TIM10, TIM_FLAG_Update);
    TIM_ITConfig(TIM10, TIM_IT_Update, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM10, ENABLE);
}

uint32_t Remote_GetUs(void)
{
    uint32_t high;
    uint16_t cnt;

    high = remote_timebase_us_high;
    cnt = (uint16_t)TIM10->CNT;

    if (((TIM10->SR & TIM_SR_UIF) != 0U) && (cnt < 0x8000U))
    {
        high += 0x10000U;
    }

    return high + (uint32_t)cnt;
}

static void Remote_SyncCH1Debug(void)
{
    RC_CH1_Pulse = RC_CH[0];
    RC_CH1_Updated = RC_CH_Updated[0];
    RC_CH1_Level = RC_CH_Level[0];
    RC_CH1_EdgeCount = RC_CH_EdgeCount[0];
    RC_CH1_RiseCount = RC_CH_RiseCount[0];
    RC_CH1_FallCount = RC_CH_FallCount[0];
}

static void Remote_GPIO_EXTI_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    uint8_t i;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    for (i = 0; i < RC_CHANNEL_COUNT; i++)
    {
        SYSCFG_EXTILineConfig(rc_inputs[i].port_source, rc_inputs[i].pin_source);

        EXTI_InitStructure.EXTI_Line = rc_inputs[i].exti_line;
        EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
        EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
        EXTI_InitStructure.EXTI_LineCmd = ENABLE;
        EXTI_Init(&EXTI_InitStructure);
    }
}

static void Remote_NVIC_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_Init(&NVIC_InitStructure);
}

static void Remote_HandleChannel(uint8_t index)
{
    uint32_t now_us;
    uint32_t pulse_us;

    if (EXTI_GetITStatus(rc_inputs[index].exti_line) == RESET)
    {
        return;
    }

    now_us = Remote_GetUs();
    RC_CH_EdgeCount[index]++;
    RC_CH_Level[index] = (GPIO_ReadInputDataBit(rc_inputs[index].port, rc_inputs[index].pin) != Bit_RESET) ? 1 : 0;

    if (RC_CH_Level[index] != 0)
    {
        RC_CH_RiseCount[index]++;
        rc_rise_us[index] = now_us;
    }
    else
    {
        RC_CH_FallCount[index]++;
        pulse_us = now_us - rc_rise_us[index];

        if ((pulse_us >= 800U) && (pulse_us <= 2200U))
        {
            RC_CH[index] = (uint16_t)pulse_us;
            RC_CH_Updated[index] = 1;
        }
    }

    if (index == 0)
    {
        Remote_SyncCH1Debug();
    }

    EXTI_ClearITPendingBit(rc_inputs[index].exti_line);
}

void Remote_Init(void)
{
    Remote_Timebase_Init();
    Remote_GPIO_EXTI_Init();
    Remote_NVIC_Init();
}

void Remote_CH1_Init(void)
{
    Remote_Init();
}

void Remote_EXTI0_IRQHandler(void)
{
    Remote_HandleChannel(5);
}

void Remote_EXTI2_IRQHandler(void)
{
    Remote_HandleChannel(0);
}

void Remote_EXTI3_IRQHandler(void)
{
    Remote_HandleChannel(1);
}

void Remote_EXTI9_5_IRQHandler(void)
{
    Remote_HandleChannel(2);
    Remote_HandleChannel(3);
    Remote_HandleChannel(4);
}

void Remote_TIM10_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM10, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM10, TIM_IT_Update);
        remote_timebase_us_high += 0x10000U;
    }
}

void Remote_CH1_EXTI2_IRQHandler(void)
{
    Remote_EXTI2_IRQHandler();
}
