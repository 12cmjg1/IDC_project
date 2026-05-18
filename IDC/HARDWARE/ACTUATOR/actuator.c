#include "actuator.h"
#include "line_follow.h"
#include "remote.h"
#include "vesc_can.h"

#define RC_CENTER_DEADZONE_US   40
#define RC_STOP_DEADZONE_US     80
#define RC_RESTART_DEADZONE_US  120
#define RC_SWITCH_LOW_US        1300U
#define RC_SWITCH_HIGH_US       1700U

#define SERVO_PULSE_MIN_US      500U
#define SERVO_PULSE_MAX_US      2500U
#define SERVO_OPEN_US           1500U
#define SERVO_TENNIS_US         2278U
#define SERVO_GOLF_US           1944U

#define VESC_MAX_ERPM           20000
#define VESC_SEND_PERIOD_US     10000U
#define VESC_LEFT_SIGN          (-1)
#define VESC_RIGHT_SIGN         (1)

#define EMM5_DEFAULT_ADDR       3U
#define EMM5_CHECK_BYTE         0x6BU
#define EMM5_LIFT_SPEED_RPM     300U
#define EMM5_LIFT_ACCEL         50U
#define EMM5_DIR_UP             0U
#define EMM5_DIR_DOWN           1U
#define EMM5_INIT_DELAY_MS      50U

volatile int16_t Act_LeftMotor = 0;
volatile int16_t Act_RightMotor = 0;
volatile int16_t Act_LiftSpeed = 0;
volatile int16_t Act_MoveCmd = 0;
volatile int16_t Act_TurnCmd = 0;
volatile uint16_t Act_GripperPulse = SERVO_OPEN_US;
volatile uint16_t Act_SpeedScale = 0;
volatile uint8_t Act_GripperMode = 1;
volatile int8_t Act_LiftMode = 0;
volatile uint16_t Act_LiftPulseCount = 5000U;
volatile uint32_t Act_LiftTriggerCount = 0;
volatile uint8_t Act_VescEnable = 1;

static uint32_t vesc_last_tx_us = 0;
static uint8_t lift_switch_ready = 0;
static uint8_t lift_last_pos = 1U;
static uint8_t drive_stop_latched = 1U;

static int16_t RC_ToCenteredSigned(uint16_t pulse)
{
    int32_t value;

    if (pulse < 800U || pulse > 2200U)
    {
        return 0;
    }

    value = (int32_t)pulse - 1500;
    if (value > -RC_CENTER_DEADZONE_US && value < RC_CENTER_DEADZONE_US)
    {
        return 0;
    }

    if (value > 500)
    {
        value = 500;
    }
    else if (value < -500)
    {
        value = -500;
    }

    return (int16_t)((value * 1000) / 500);
}

static uint16_t RC_ToServoPulse(uint16_t pulse)
{
    if (pulse < SERVO_PULSE_MIN_US)
    {
        return SERVO_PULSE_MIN_US;
    }
    if (pulse > SERVO_PULSE_MAX_US)
    {
        return SERVO_PULSE_MAX_US;
    }
    return pulse;
}

static uint16_t RC_ToSpeedScale(uint16_t pulse)
{
    uint32_t value;

    if (pulse < 800U || pulse > 2200U)
    {
        return 0;
    }

    if (pulse <= 1000U)
    {
        return 0;
    }

    if (pulse >= 2000U)
    {
        return 1000U;
    }

    value = (uint32_t)(pulse - 1000U);
    return (uint16_t)((value * 1000U) / 1000U);
}

static uint8_t RC_IsNearCenter(uint16_t pulse, uint16_t deadzone_us)
{
    int32_t value;

    if (pulse < 800U || pulse > 2200U)
    {
        return 1U;
    }

    value = (int32_t)pulse - 1500;
    if (value < 0)
    {
        value = -value;
    }

    return (value <= (int32_t)deadzone_us) ? 1U : 0U;
}

static uint8_t RC_ToThreePosition(uint16_t pulse)
{
    if (pulse < 800U || pulse > 2200U)
    {
        return 1U;
    }

    if (pulse <= RC_SWITCH_LOW_US)
    {
        return 0U;
    }

    if (pulse >= RC_SWITCH_HIGH_US)
    {
        return 2U;
    }

    return 1U;
}

static void Actuator_DelayMs(uint32_t ms)
{
    uint32_t start_us = Remote_GetUs();
    uint32_t delay_us = ms * 1000U;

    while ((uint32_t)(Remote_GetUs() - start_us) < delay_us)
    {
    }
}

static void Servo_PWM_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Period = 19999U;
    TIM_TimeBaseStructure.TIM_Prescaler = 83U;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = Act_GripperPulse;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}

static uint8_t Emm5_SendFrame(uint32_t ext_id, const uint8_t *data, uint8_t len)
{
    CanTxMsg tx;
    uint8_t mailbox;
    uint8_t tx_status;
    uint32_t start_us;

    tx.StdId = 0U;
    tx.ExtId = ext_id;
    tx.IDE = CAN_Id_Extended;
    tx.RTR = CAN_RTR_Data;
    tx.DLC = len;
    tx.Data[0] = (len > 0U) ? data[0] : 0U;
    tx.Data[1] = (len > 1U) ? data[1] : 0U;
    tx.Data[2] = (len > 2U) ? data[2] : 0U;
    tx.Data[3] = (len > 3U) ? data[3] : 0U;
    tx.Data[4] = (len > 4U) ? data[4] : 0U;
    tx.Data[5] = (len > 5U) ? data[5] : 0U;
    tx.Data[6] = (len > 6U) ? data[6] : 0U;
    tx.Data[7] = (len > 7U) ? data[7] : 0U;

    mailbox = CAN_Transmit(CAN1, &tx);
    if (mailbox > 2U)
    {
        return 0U;
    }

    start_us = Remote_GetUs();
    do
    {
        tx_status = CAN_TransmitStatus(CAN1, mailbox);
        if (tx_status == CAN_TxStatus_Ok)
        {
            return 1U;
        }
        if (tx_status == CAN_TxStatus_Failed)
        {
            return 0U;
        }
    } while ((uint32_t)(Remote_GetUs() - start_us) < 100000U);

    return 0U;
}

static uint8_t Emm5_SendCmdExt(const uint8_t *cmd, uint8_t len)
{
    uint8_t i = 0U;
    uint8_t pack_num = 0U;
    uint8_t remain;
    uint8_t copy_len;
    uint8_t tx_data[8];
    uint32_t ext_id;

    if (len < 2U)
    {
        return 0U;
    }

    while (i < (uint8_t)(len - 2U))
    {
        tx_data[0] = cmd[1];
        tx_data[1] = 0U;
        tx_data[2] = 0U;
        tx_data[3] = 0U;
        tx_data[4] = 0U;
        tx_data[5] = 0U;
        tx_data[6] = 0U;
        tx_data[7] = 0U;

        ext_id = ((uint32_t)cmd[0] << 8) | pack_num;
        remain = (uint8_t)((len - 2U) - i);
        copy_len = (remain < 7U) ? remain : 7U;

        tx_data[1] = cmd[i + 2U];
        if (copy_len > 1U) { tx_data[2] = cmd[i + 3U]; }
        if (copy_len > 2U) { tx_data[3] = cmd[i + 4U]; }
        if (copy_len > 3U) { tx_data[4] = cmd[i + 5U]; }
        if (copy_len > 4U) { tx_data[5] = cmd[i + 6U]; }
        if (copy_len > 5U) { tx_data[6] = cmd[i + 7U]; }
        if (copy_len > 6U) { tx_data[7] = cmd[i + 8U]; }

        if (Emm5_SendFrame(ext_id, tx_data, (uint8_t)(copy_len + 1U)) == 0U)
        {
            return 0U;
        }

        i = (uint8_t)(i + copy_len);
        pack_num++;
    }

    return 1U;
}

static void Emm5_Enable(uint8_t enable)
{
    uint8_t cmd[6];

    cmd[0] = EMM5_DEFAULT_ADDR;
    cmd[1] = 0xF3U;
    cmd[2] = 0xABU;
    cmd[3] = enable;
    cmd[4] = 0U;
    cmd[5] = EMM5_CHECK_BYTE;
    Emm5_SendCmdExt(cmd, 6U);
}

static void Emm5_ResetZero(void)
{
    uint8_t cmd[4];

    cmd[0] = EMM5_DEFAULT_ADDR;
    cmd[1] = 0x0AU;
    cmd[2] = 0x6DU;
    cmd[3] = EMM5_CHECK_BYTE;
    Emm5_SendCmdExt(cmd, 4U);
}

static void Emm5_PosControl(uint8_t dir, uint32_t pulses, uint8_t absolute_pos)
{
    uint8_t cmd[13];

    cmd[0] = EMM5_DEFAULT_ADDR;
    cmd[1] = 0xFDU;
    cmd[2] = dir;
    cmd[3] = (uint8_t)(EMM5_LIFT_SPEED_RPM >> 8);
    cmd[4] = (uint8_t)(EMM5_LIFT_SPEED_RPM >> 0);
    cmd[5] = (uint8_t)EMM5_LIFT_ACCEL;
    cmd[6] = (uint8_t)(pulses >> 24);
    cmd[7] = (uint8_t)(pulses >> 16);
    cmd[8] = (uint8_t)(pulses >> 8);
    cmd[9] = (uint8_t)(pulses >> 0);
    cmd[10] = absolute_pos;
    cmd[11] = 0U;
    cmd[12] = EMM5_CHECK_BYTE;
    Emm5_SendCmdExt(cmd, 13U);
}

static void Emm5_LiftUp(uint32_t pulses)
{
    Emm5_PosControl(EMM5_DIR_UP, pulses, 0U);
}

static void Emm5_LiftDown(uint32_t pulses)
{
    Emm5_PosControl(EMM5_DIR_DOWN, pulses, 0U);
}

static void Emm5_Init(void)
{
    Actuator_DelayMs(EMM5_INIT_DELAY_MS);
    Emm5_Enable(1U);
    Actuator_DelayMs(EMM5_INIT_DELAY_MS);
    Emm5_ResetZero();
    Actuator_DelayMs(EMM5_INIT_DELAY_MS);
}

void Actuator_Init(void)
{
    VescCan_Init();
    Servo_PWM_Init();
    Emm5_Init();
    LineFollow_Init();
}

void Actuator_UpdateFromRC(void)
{
    int16_t move_cmd = RC_ToCenteredSigned(RC_CH[1]);
    int16_t turn_cmd = (int16_t)(-RC_ToCenteredSigned(RC_CH[0]));
    uint8_t move_near_center = RC_IsNearCenter(RC_CH[1], RC_STOP_DEADZONE_US);
    uint8_t turn_near_center = RC_IsNearCenter(RC_CH[0], RC_STOP_DEADZONE_US);
    uint8_t move_leave_center = RC_IsNearCenter(RC_CH[1], RC_RESTART_DEADZONE_US);
    uint8_t turn_leave_center = RC_IsNearCenter(RC_CH[0], RC_RESTART_DEADZONE_US);
    uint16_t speed_scale = RC_ToSpeedScale(RC_CH[2]);
    uint8_t gripper_pos = RC_ToThreePosition(RC_CH[4]);
    uint8_t lift_pos = RC_ToThreePosition(RC_CH[5]);
    int32_t scaled_move = ((int32_t)move_cmd * speed_scale) / 1000;
    int32_t scaled_turn = ((int32_t)turn_cmd * speed_scale) / 1000;
    int32_t left;
    int32_t right;

    if (move_near_center != 0U && turn_near_center != 0U)
    {
        drive_stop_latched = 1U;
    }
    else if (drive_stop_latched != 0U)
    {
        if (move_leave_center == 0U || turn_leave_center == 0U)
        {
            drive_stop_latched = 0U;
        }
    }

    if (drive_stop_latched != 0U)
    {
        move_cmd = 0;
        turn_cmd = 0;
        left = 0;
        right = 0;
    }
    else if (turn_cmd != 0)
    {
        left = scaled_turn;
        right = -scaled_turn;
    }
    else
    {
        left = scaled_move;
        right = scaled_move;
    }

    if (left > 1000)
    {
        left = 1000;
    }
    else if (left < -1000)
    {
        left = -1000;
    }

    if (right > 1000)
    {
        right = 1000;
    }
    else if (right < -1000)
    {
        right = -1000;
    }

    Act_MoveCmd = move_cmd;
    Act_TurnCmd = turn_cmd;
    Act_SpeedScale = speed_scale;
    Act_GripperMode = gripper_pos;

    LineFollow_Task(speed_scale);

    if (LineFollow_IsEnabled() != 0U)
    {
        Act_LeftMotor = (int16_t)((int32_t)LineFollow_LeftCmdErpm * VESC_LEFT_SIGN);
        Act_RightMotor = (int16_t)((int32_t)LineFollow_RightCmdErpm * VESC_RIGHT_SIGN);
    }
    else
    {
        Act_LeftMotor = (int16_t)((((int32_t)left * VESC_MAX_ERPM) / 1000) * VESC_LEFT_SIGN);
        Act_RightMotor = (int16_t)((((int32_t)right * VESC_MAX_ERPM) / 1000) * VESC_RIGHT_SIGN);
    }

    if (gripper_pos == 2U)
    {
        Act_GripperPulse = SERVO_TENNIS_US;
    }
    else if (gripper_pos == 0U)
    {
        Act_GripperPulse = SERVO_GOLF_US;
    }
    else
    {
        Act_GripperPulse = SERVO_OPEN_US;
    }

    TIM3->CCR1 = RC_ToServoPulse(Act_GripperPulse);

    if (lift_switch_ready == 0U)
    {
        lift_last_pos = lift_pos;
        lift_switch_ready = 1U;
    }
    else if (lift_pos != lift_last_pos)
    {
        lift_last_pos = lift_pos;

        if (lift_pos == 2U)
        {
            Emm5_LiftUp(Act_LiftPulseCount);
            Act_LiftMode = 1;
            Act_LiftSpeed = 1000;
            Act_LiftTriggerCount++;
        }
        else if (lift_pos == 0U)
        {
            Emm5_LiftDown(Act_LiftPulseCount);
            Act_LiftMode = -1;
            Act_LiftSpeed = -1000;
            Act_LiftTriggerCount++;
        }
    }
}

void Actuator_Task(void)
{
    uint32_t now_us = Remote_GetUs();

    if (Act_VescEnable != 0U && (uint32_t)(now_us - vesc_last_tx_us) >= VESC_SEND_PERIOD_US)
    {
        vesc_last_tx_us = now_us;
        VescCan_SetRpm(VESC_CAN_LEFT_ID, Act_LeftMotor);
        VescCan_SetRpm(VESC_CAN_RIGHT_ID, Act_RightMotor);
    }
}
