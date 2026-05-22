#ifndef __REMOTE_H
#define __REMOTE_H

#include "stm32f4xx.h"

#define RC_CHANNEL_COUNT 6

extern volatile uint16_t RC_CH[RC_CHANNEL_COUNT];
extern volatile uint8_t RC_CH_Updated[RC_CHANNEL_COUNT];
extern volatile uint8_t RC_CH_Level[RC_CHANNEL_COUNT];
extern volatile uint32_t RC_CH_EdgeCount[RC_CHANNEL_COUNT];
extern volatile uint32_t RC_CH_RiseCount[RC_CHANNEL_COUNT];
extern volatile uint32_t RC_CH_FallCount[RC_CHANNEL_COUNT];

extern volatile uint16_t RC_CH1_Pulse;
extern volatile uint8_t RC_CH1_Updated;
extern volatile uint8_t RC_CH1_Level;
extern volatile uint32_t RC_CH1_EdgeCount;
extern volatile uint32_t RC_CH1_RiseCount;
extern volatile uint32_t RC_CH1_FallCount;

uint32_t Remote_GetUs(void);
void Remote_Init(void);
void Remote_CH1_Init(void);
void Remote_TIM10_IRQHandler(void);
void Remote_EXTI0_IRQHandler(void);
void Remote_EXTI2_IRQHandler(void);
void Remote_EXTI3_IRQHandler(void);
void Remote_EXTI9_5_IRQHandler(void);
void Remote_CH1_EXTI2_IRQHandler(void);

#endif
