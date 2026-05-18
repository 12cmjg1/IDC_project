#ifndef __VESC_CAN_H
#define __VESC_CAN_H

#include "stm32f4xx.h"

#define VESC_CAN_LEFT_ID   1U
#define VESC_CAN_RIGHT_ID  2U

typedef enum
{
    VESC_CAN_PACKET_SET_DUTY = 0,
    VESC_CAN_PACKET_SET_CURRENT = 1,
    VESC_CAN_PACKET_SET_CURRENT_BRAKE = 2,
    VESC_CAN_PACKET_SET_RPM = 3,
    VESC_CAN_PACKET_SET_POS = 4,
    VESC_CAN_PACKET_STATUS = 9
} VescCanPacketId;

void VescCan_Init(void);
void VescCan_SetRpm(uint8_t controller_id, int32_t erpm);
void VescCan_RX0_IRQHandler(void);
void VescCan_DebugPoll(void);

extern volatile uint8_t VescCan_InitOk;
extern volatile uint32_t VescCan_TxCount;
extern volatile uint32_t VescCan_RxCount;
extern volatile uint32_t VescCan_TxOkCount;
extern volatile uint32_t VescCan_TxFailCount;
extern volatile uint32_t VescCan_TxNoMailboxCount;
extern volatile uint8_t VescCan_LastMailbox;
extern volatile uint8_t VescCan_LastTxStatus;
extern volatile uint32_t VescCan_LastTSR;
extern volatile uint32_t VescCan_LastESR;
extern volatile uint32_t VescCan_LastMSR;
extern volatile uint8_t VescCan_LastErrorCode;
extern volatile uint8_t VescCan_BusOff;
extern volatile uint8_t VescCan_ErrorPassive;
extern volatile uint8_t VescCan_ErrorWarning;
extern volatile uint8_t VescCan_RxFifo0Pending;
extern volatile int32_t VescCan_LeftLastErpm;
extern volatile int32_t VescCan_RightLastErpm;
extern volatile uint8_t VescCan_LastStatusId;
extern volatile int32_t VescCan_LeftStatusErpm;
extern volatile int32_t VescCan_RightStatusErpm;
extern volatile int16_t VescCan_LeftStatusCurrent_dA;
extern volatile int16_t VescCan_RightStatusCurrent_dA;
extern volatile int16_t VescCan_LeftStatusDuty_milli;
extern volatile int16_t VescCan_RightStatusDuty_milli;
extern volatile uint8_t VescCan_LeftStatusSeen;
extern volatile uint8_t VescCan_RightStatusSeen;

#endif
