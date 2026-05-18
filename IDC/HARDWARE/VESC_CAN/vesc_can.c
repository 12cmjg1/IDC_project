#include "vesc_can.h"

volatile uint8_t VescCan_InitOk = 0;
volatile uint32_t VescCan_TxCount = 0;
volatile uint32_t VescCan_RxCount = 0;
volatile uint32_t VescCan_TxOkCount = 0;
volatile uint32_t VescCan_TxFailCount = 0;
volatile uint32_t VescCan_TxNoMailboxCount = 0;
volatile uint8_t VescCan_LastMailbox = 0xFF;
volatile uint8_t VescCan_LastTxStatus = 0xFF;
volatile uint32_t VescCan_LastTSR = 0;
volatile uint32_t VescCan_LastESR = 0;
volatile uint32_t VescCan_LastMSR = 0;
volatile uint8_t VescCan_LastErrorCode = 0;
volatile uint8_t VescCan_BusOff = 0;
volatile uint8_t VescCan_ErrorPassive = 0;
volatile uint8_t VescCan_ErrorWarning = 0;
volatile uint8_t VescCan_RxFifo0Pending = 0;
volatile int32_t VescCan_LeftLastErpm = 0;
volatile int32_t VescCan_RightLastErpm = 0;
volatile uint8_t VescCan_LastStatusId = 0;
volatile int32_t VescCan_LeftStatusErpm = 0;
volatile int32_t VescCan_RightStatusErpm = 0;
volatile int16_t VescCan_LeftStatusCurrent_dA = 0;
volatile int16_t VescCan_RightStatusCurrent_dA = 0;
volatile int16_t VescCan_LeftStatusDuty_milli = 0;
volatile int16_t VescCan_RightStatusDuty_milli = 0;
volatile uint8_t VescCan_LeftStatusSeen = 0;
volatile uint8_t VescCan_RightStatusSeen = 0;

static void VescCan_NVIC_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

static void VescCan_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}

static void VescCan_Filter_Init(void)
{
    CAN_FilterInitTypeDef CAN_FilterInitStructure;

    CAN_FilterInitStructure.CAN_FilterNumber = 0;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);
}

static void VescCan_UpdateRegs(void)
{
    VescCan_LastTSR = CAN1->TSR;
    VescCan_LastESR = CAN1->ESR;
    VescCan_LastMSR = CAN1->MSR;
    VescCan_LastErrorCode = (uint8_t)((CAN1->ESR & CAN_ESR_LEC) >> 4);
    VescCan_BusOff = ((CAN1->ESR & CAN_ESR_BOFF) != 0U) ? 1U : 0U;
    VescCan_ErrorPassive = ((CAN1->ESR & CAN_ESR_EPVF) != 0U) ? 1U : 0U;
    VescCan_ErrorWarning = ((CAN1->ESR & CAN_ESR_EWGF) != 0U) ? 1U : 0U;
    VescCan_RxFifo0Pending = (uint8_t)CAN_MessagePending(CAN1, CAN_FIFO0);
}

void VescCan_Init(void)
{
    CAN_InitTypeDef CAN_InitStructure;

    VescCan_GPIO_Init();
    VescCan_NVIC_Init();

    CAN_DeInit(CAN1);
    CAN_StructInit(&CAN_InitStructure);

    CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = ENABLE;
    CAN_InitStructure.CAN_AWUM = ENABLE;
    CAN_InitStructure.CAN_NART = DISABLE;
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = DISABLE;
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
    CAN_InitStructure.CAN_BS1 = CAN_BS1_10tq;
    CAN_InitStructure.CAN_BS2 = CAN_BS2_3tq;
    CAN_InitStructure.CAN_Prescaler = 6;

    VescCan_InitOk = CAN_Init(CAN1, &CAN_InitStructure);
    VescCan_Filter_Init();
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
    VescCan_UpdateRegs();
}

void VescCan_SetRpm(uint8_t controller_id, int32_t erpm)
{
    CanTxMsg tx;
    uint32_t ext_id = ((uint32_t)VESC_CAN_PACKET_SET_RPM << 8) | controller_id;
    uint8_t mailbox;
    uint8_t tx_status;

    tx.StdId = 0;
    tx.ExtId = ext_id;
    tx.IDE = CAN_Id_Extended;
    tx.RTR = CAN_RTR_Data;
    tx.DLC = 8;
    tx.Data[0] = (uint8_t)((erpm >> 24) & 0xFF);
    tx.Data[1] = (uint8_t)((erpm >> 16) & 0xFF);
    tx.Data[2] = (uint8_t)((erpm >> 8) & 0xFF);
    tx.Data[3] = (uint8_t)(erpm & 0xFF);
    tx.Data[4] = 0;
    tx.Data[5] = 0;
    tx.Data[6] = 0;
    tx.Data[7] = 0;

    mailbox = CAN_Transmit(CAN1, &tx);
    VescCan_LastMailbox = mailbox;
    VescCan_TxCount++;
    VescCan_UpdateRegs();

    if (mailbox > 2U)
    {
        VescCan_LastTxStatus = 0xFFU;
        VescCan_TxNoMailboxCount++;
    }
    else
    {
        tx_status = CAN_TransmitStatus(CAN1, mailbox);
        VescCan_LastTxStatus = tx_status;

        if (tx_status == CAN_TxStatus_Ok)
        {
            VescCan_TxOkCount++;
        }
        else if (tx_status == CAN_TxStatus_Failed)
        {
            VescCan_TxFailCount++;
        }
    }

    VescCan_UpdateRegs();

    if (controller_id == VESC_CAN_LEFT_ID)
    {
        VescCan_LeftLastErpm = erpm;
    }
    else if (controller_id == VESC_CAN_RIGHT_ID)
    {
        VescCan_RightLastErpm = erpm;
    }
}

void VescCan_RX0_IRQHandler(void)
{
    CanRxMsg rx;
    uint32_t id;
    uint8_t cmd;
    uint8_t vesc_id;
    int32_t erpm;
    int16_t current;
    int16_t duty;

    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) == RESET)
    {
        return;
    }

    CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
    CAN_Receive(CAN1, CAN_FIFO0, &rx);
    VescCan_RxCount++;
    VescCan_UpdateRegs();

    if (rx.IDE != CAN_Id_Extended)
    {
        return;
    }

    id = rx.ExtId;
    cmd = (uint8_t)((id >> 8) & 0xFFU);
    vesc_id = (uint8_t)(id & 0xFFU);

    if (cmd != (uint8_t)VESC_CAN_PACKET_STATUS || rx.DLC < 8U)
    {
        return;
    }

    erpm = ((int32_t)rx.Data[0] << 24) |
           ((int32_t)rx.Data[1] << 16) |
           ((int32_t)rx.Data[2] << 8)  |
           (int32_t)rx.Data[3];

    current = (int16_t)(((uint16_t)rx.Data[4] << 8) | rx.Data[5]);
    duty = (int16_t)(((uint16_t)rx.Data[6] << 8) | rx.Data[7]);

    VescCan_LastStatusId = vesc_id;

    if (vesc_id == VESC_CAN_LEFT_ID)
    {
        VescCan_LeftStatusErpm = erpm;
        VescCan_LeftStatusCurrent_dA = current;
        VescCan_LeftStatusDuty_milli = duty;
        VescCan_LeftStatusSeen = 1U;
    }
    else if (vesc_id == VESC_CAN_RIGHT_ID)
    {
        VescCan_RightStatusErpm = erpm;
        VescCan_RightStatusCurrent_dA = current;
        VescCan_RightStatusDuty_milli = duty;
        VescCan_RightStatusSeen = 1U;
    }
}

void VescCan_DebugPoll(void)
{
    VescCan_UpdateRegs();
}
