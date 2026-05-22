#ifndef __LINE_FOLLOW_H
#define __LINE_FOLLOW_H

#include "stm32f4xx.h"

void LineFollow_Init(void);
void LineFollow_Task(uint16_t speed_scale);
uint8_t LineFollow_IsEnabled(void);
void LineFollow_SetEnabled(uint8_t enabled);

extern volatile uint8_t LineFollow_Enabled;
extern volatile uint16_t LineFollow_Center;
extern volatile uint16_t LineFollow_Threshold;
extern volatile uint16_t LineFollow_Width;
extern volatile int16_t LineFollow_Position;
extern volatile uint8_t LineFollow_Cross;
extern volatile uint8_t LineFollow_Lost;
extern volatile int16_t LineFollow_LeftCmdErpm;
extern volatile int16_t LineFollow_RightCmdErpm;
extern volatile uint32_t LineFollow_FrameCount;
extern volatile uint32_t LineFollow_CrossCount;
extern volatile uint32_t LineFollow_KeyToggleCount;
extern volatile uint8_t LineFollow_Reverse;

#endif
