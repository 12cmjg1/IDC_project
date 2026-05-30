#ifndef __IMU_TEST_H
#define __IMU_TEST_H

#include "stm32f4xx.h"

extern volatile uint8_t ImuTest_Online;
extern volatile uint8_t ImuTest_DeviceId;
extern volatile uint32_t ImuTest_ReadOk;
extern volatile uint32_t ImuTest_ReadFail;
extern volatile int16_t ImuTest_AccelX;
extern volatile int16_t ImuTest_AccelY;
extern volatile int16_t ImuTest_AccelZ;
extern volatile int16_t ImuTest_GyroX;
extern volatile int16_t ImuTest_GyroY;
extern volatile int16_t ImuTest_GyroZ;
extern volatile int16_t ImuTest_PitchX10;

void ImuTest_Init(void);
void ImuTest_Task(uint32_t now_us);

#endif
