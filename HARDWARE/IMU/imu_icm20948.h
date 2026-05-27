#ifndef __IMU_ICM20948_H
#define __IMU_ICM20948_H

#include "stm32f4xx.h"

#define IMU_TYPE_NONE       0U
#define IMU_TYPE_ICM20948   1U

void Imu_Init(void);
void Imu_Task(void);
void Imu_ResetYaw(void);
int32_t Imu_GetYawDeg10(void);
int32_t Imu_GetYawDeltaDeg10(int32_t start_yaw_deg10);

extern volatile uint8_t Imu_Type;
extern volatile uint8_t Imu_Ready;
extern volatile uint8_t Imu_InitFailCode;
extern volatile uint32_t Imu_FrameCount;
extern volatile int16_t Imu_GyroZRaw;
extern volatile int16_t Imu_GyroZOffset;
extern volatile int32_t Imu_YawDeg10;

#endif
