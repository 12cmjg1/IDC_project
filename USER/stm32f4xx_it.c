#include "stm32f4xx_it.h"
#include "remote.h"
#include "vesc_can.h"

void NMI_Handler(void) {}

void HardFault_Handler(void)
{
    while (1) {}
}

void MemManage_Handler(void)
{
    while (1) {}
}

void BusFault_Handler(void)
{
    while (1) {}
}

void UsageFault_Handler(void)
{
    while (1) {}
}

//void SVC_Handler(void) {}

void DebugMon_Handler(void) {}

//void PendSV_Handler(void) {}

//void SysTick_Handler(void) {}

void EXTI2_IRQHandler(void)
{
    Remote_EXTI2_IRQHandler();
}

void EXTI0_IRQHandler(void)
{
    Remote_EXTI0_IRQHandler();
}

void EXTI3_IRQHandler(void)
{
    Remote_EXTI3_IRQHandler();
}

void EXTI9_5_IRQHandler(void)
{
    Remote_EXTI9_5_IRQHandler();
}

void CAN1_RX0_IRQHandler(void)
{
    VescCan_RX0_IRQHandler();
}

void TIM1_UP_TIM10_IRQHandler(void)
{
    Remote_TIM10_IRQHandler();
}
