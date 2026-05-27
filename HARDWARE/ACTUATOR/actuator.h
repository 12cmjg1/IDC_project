#ifndef __ACTUATOR_H
#define __ACTUATOR_H

#include "stm32f4xx.h"

void Actuator_Init(void);
void Actuator_OnUserStart(void);
void Actuator_UpdateFromRC(void);
void Actuator_Task(void);

extern volatile int16_t Act_LeftMotor;
extern volatile int16_t Act_RightMotor;
extern volatile int16_t Act_LiftSpeed;
extern volatile int16_t Act_MoveCmd;
extern volatile int16_t Act_TurnCmd;
extern volatile uint16_t Act_GripperPulse;
extern volatile uint16_t Act_SpeedScale;
extern volatile uint8_t Act_GripperMode;
extern volatile int8_t Act_LiftMode;
extern volatile uint16_t Act_LiftPulseCount;
extern volatile uint32_t Act_LiftTargetPulse;
extern volatile uint32_t Act_LiftTriggerCount;
extern volatile uint8_t Act_VescEnable;
extern volatile uint8_t Act_DriveIdle;
extern volatile uint8_t Act_VescSendMode;
extern volatile uint32_t Act_Emm5TxCount;
extern volatile uint32_t Act_Emm5TxOkCount;
extern volatile uint32_t Act_Emm5TxFailCount;
extern volatile uint32_t Act_Emm5TxNoMailboxCount;
extern volatile uint32_t Act_Emm5TxTimeoutCount;
extern volatile uint32_t Act_Emm5LastExtId;
extern volatile uint8_t Act_Emm5LastLen;
extern volatile uint8_t Act_Emm5LastTxStatus;
extern volatile uint8_t Act_Emm5LastCmd;
extern volatile uint8_t Act_Emm5Enabled;
extern volatile uint32_t Act_LiftAutoApplyCount;
extern volatile int8_t Act_LiftTrimState;
extern volatile uint32_t Act_LiftStopCount;

#endif
