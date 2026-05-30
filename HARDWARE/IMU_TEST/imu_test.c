#include "imu_test.h"
#include "remote.h"
#include <math.h>

#define IMU_TEST_I2C_PORT        GPIOB
#define IMU_TEST_I2C_RCC         RCC_AHB1Periph_GPIOB
#define IMU_TEST_SCL_PIN         GPIO_Pin_10
#define IMU_TEST_SDA_PIN         GPIO_Pin_11

#define IMU_TEST_ADDR            0xD0U
#define IMU_TEST_WHO_AM_I        0x00U
#define IMU_TEST_REG_BANK_SEL    0x7FU
#define IMU_TEST_USER_CTRL       0x03U
#define IMU_TEST_PWR_MGMT_1      0x06U
#define IMU_TEST_INT_PIN_CFG     0x0FU
#define IMU_TEST_ACCEL_XOUT_H    0x2DU
#define IMU_TEST_GYRO_CONFIG_1   0x01U
#define IMU_TEST_GYRO_SMPLRT_DIV 0x00U
#define IMU_TEST_ACCEL_CONFIG    0x14U
#define IMU_TEST_ACCEL_DIV_2     0x11U
#define IMU_TEST_WHO_AM_I_VALUE  0xEAU

#define IMU_TEST_BANK_0          0x00U
#define IMU_TEST_BANK_2          0x20U
#define IMU_TEST_UPDATE_US       20000U
#define IMU_TEST_ACK_TIMEOUT     5000U

volatile uint8_t ImuTest_Online = 0;
volatile uint8_t ImuTest_DeviceId = 0;
volatile uint32_t ImuTest_ReadOk = 0;
volatile uint32_t ImuTest_ReadFail = 0;
volatile int16_t ImuTest_AccelX = 0;
volatile int16_t ImuTest_AccelY = 0;
volatile int16_t ImuTest_AccelZ = 0;
volatile int16_t ImuTest_GyroX = 0;
volatile int16_t ImuTest_GyroY = 0;
volatile int16_t ImuTest_GyroZ = 0;
volatile int16_t ImuTest_PitchX10 = 0;

static uint32_t imu_last_update_us = 0;

static void ImuTest_Delay(void)
{
    volatile uint16_t i;

    for (i = 0; i < 25U; i++)
    {
    }
}

static void ImuTest_WaitUs(uint32_t us)
{
    uint32_t start_us;

    start_us = Remote_GetUs();
    while ((uint32_t)(Remote_GetUs() - start_us) < us)
    {
    }
}

static void ImuTest_SclHigh(void)
{
    GPIO_SetBits(IMU_TEST_I2C_PORT, IMU_TEST_SCL_PIN);
}

static void ImuTest_SclLow(void)
{
    GPIO_ResetBits(IMU_TEST_I2C_PORT, IMU_TEST_SCL_PIN);
}

static void ImuTest_SdaHigh(void)
{
    GPIO_SetBits(IMU_TEST_I2C_PORT, IMU_TEST_SDA_PIN);
}

static void ImuTest_SdaLow(void)
{
    GPIO_ResetBits(IMU_TEST_I2C_PORT, IMU_TEST_SDA_PIN);
}

static void ImuTest_SdaOut(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = IMU_TEST_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(IMU_TEST_I2C_PORT, &GPIO_InitStructure);
}

static void ImuTest_SdaIn(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = IMU_TEST_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(IMU_TEST_I2C_PORT, &GPIO_InitStructure);
}

static uint8_t ImuTest_ReadSda(void)
{
    return (GPIO_ReadInputDataBit(IMU_TEST_I2C_PORT, IMU_TEST_SDA_PIN) != Bit_RESET) ? 1U : 0U;
}

static void ImuTest_I2cInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(IMU_TEST_I2C_RCC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = IMU_TEST_SCL_PIN | IMU_TEST_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(IMU_TEST_I2C_PORT, &GPIO_InitStructure);

    ImuTest_SclHigh();
    ImuTest_SdaHigh();
}

static void ImuTest_Start(void)
{
    ImuTest_SdaOut();
    ImuTest_SdaHigh();
    ImuTest_SclHigh();
    ImuTest_Delay();
    ImuTest_SdaLow();
    ImuTest_Delay();
    ImuTest_SclLow();
}

static void ImuTest_Stop(void)
{
    ImuTest_SdaOut();
    ImuTest_SclLow();
    ImuTest_SdaLow();
    ImuTest_Delay();
    ImuTest_SclHigh();
    ImuTest_Delay();
    ImuTest_SdaHigh();
    ImuTest_Delay();
}

static uint8_t ImuTest_WaitAck(void)
{
    uint32_t timeout;

    timeout = 0;
    ImuTest_SdaIn();
    ImuTest_SdaHigh();
    ImuTest_Delay();
    ImuTest_SclHigh();
    ImuTest_Delay();

    while (ImuTest_ReadSda() != 0U)
    {
        timeout++;
        if (timeout > IMU_TEST_ACK_TIMEOUT)
        {
            ImuTest_Stop();
            return 0U;
        }
    }

    ImuTest_SclLow();
    return 1U;
}

static void ImuTest_Ack(void)
{
    ImuTest_SclLow();
    ImuTest_SdaOut();
    ImuTest_SdaLow();
    ImuTest_Delay();
    ImuTest_SclHigh();
    ImuTest_Delay();
    ImuTest_SclLow();
}

static void ImuTest_Nack(void)
{
    ImuTest_SclLow();
    ImuTest_SdaOut();
    ImuTest_SdaHigh();
    ImuTest_Delay();
    ImuTest_SclHigh();
    ImuTest_Delay();
    ImuTest_SclLow();
}

static void ImuTest_WriteByte(uint8_t data)
{
    uint8_t i;

    ImuTest_SdaOut();
    ImuTest_SclLow();

    for (i = 0; i < 8U; i++)
    {
        if ((data & 0x80U) != 0U)
        {
            ImuTest_SdaHigh();
        }
        else
        {
            ImuTest_SdaLow();
        }

        data <<= 1;
        ImuTest_Delay();
        ImuTest_SclHigh();
        ImuTest_Delay();
        ImuTest_SclLow();
        ImuTest_Delay();
    }
}

static uint8_t ImuTest_ReadByte(uint8_t ack)
{
    uint8_t i;
    uint8_t data;

    data = 0;
    ImuTest_SdaIn();

    for (i = 0; i < 8U; i++)
    {
        ImuTest_SclLow();
        ImuTest_Delay();
        ImuTest_SclHigh();
        data <<= 1;
        if (ImuTest_ReadSda() != 0U)
        {
            data |= 0x01U;
        }
        ImuTest_Delay();
    }

    if (ack != 0U)
    {
        ImuTest_Ack();
    }
    else
    {
        ImuTest_Nack();
    }

    return data;
}

static uint8_t ImuTest_WriteReg(uint8_t reg, uint8_t data)
{
    ImuTest_Start();
    ImuTest_WriteByte(IMU_TEST_ADDR);
    if (ImuTest_WaitAck() == 0U)
    {
        return 0U;
    }

    ImuTest_WriteByte(reg);
    if (ImuTest_WaitAck() == 0U)
    {
        return 0U;
    }

    ImuTest_WriteByte(data);
    if (ImuTest_WaitAck() == 0U)
    {
        return 0U;
    }

    ImuTest_Stop();
    return 1U;
}

static uint8_t ImuTest_ReadReg(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t i;

    if (data == 0 || len == 0U)
    {
        return 0U;
    }

    ImuTest_Start();
    ImuTest_WriteByte(IMU_TEST_ADDR);
    if (ImuTest_WaitAck() == 0U)
    {
        return 0U;
    }

    ImuTest_WriteByte(reg);
    if (ImuTest_WaitAck() == 0U)
    {
        return 0U;
    }

    ImuTest_Start();
    ImuTest_WriteByte(IMU_TEST_ADDR | 0x01U);
    if (ImuTest_WaitAck() == 0U)
    {
        return 0U;
    }

    for (i = 0; i < len; i++)
    {
        data[i] = ImuTest_ReadByte((i + 1U < len) ? 1U : 0U);
    }

    ImuTest_Stop();
    return 1U;
}

static uint8_t ImuTest_SelectBank(uint8_t bank)
{
    return ImuTest_WriteReg(IMU_TEST_REG_BANK_SEL, bank);
}

static int16_t ImuTest_ToInt16(uint8_t high, uint8_t low)
{
    return (int16_t)(((uint16_t)high << 8) | (uint16_t)low);
}

static int16_t ImuTest_CalcPitchX10(int16_t ax, int16_t ay, int16_t az)
{
    float ax_f;
    float ay_f;
    float az_f;
    float denom;
    float pitch_deg;

    ax_f = (float)ax;
    ay_f = (float)ay;
    az_f = (float)az;
    denom = sqrtf((ay_f * ay_f) + (az_f * az_f));
    pitch_deg = atan2f(-ax_f, denom) * 57.2957795f;

    if (pitch_deg >= 0.0f)
    {
        return (int16_t)((pitch_deg * 10.0f) + 0.5f);
    }

    return (int16_t)((pitch_deg * 10.0f) - 0.5f);
}

void ImuTest_Init(void)
{
    uint8_t id;
    uint8_t ok;
    uint8_t retry;

    ImuTest_I2cInit();
    ImuTest_Online = 0U;
    ImuTest_DeviceId = 0U;

    ok = 0U;
    id = 0U;
    for (retry = 0U; retry < 3U; retry++)
    {
        ok = ImuTest_SelectBank(IMU_TEST_BANK_0);
        ok &= ImuTest_ReadReg(IMU_TEST_WHO_AM_I, &id, 1U);
        ImuTest_DeviceId = id;
        if (ok != 0U && id == IMU_TEST_WHO_AM_I_VALUE)
        {
            break;
        }

        ImuTest_WriteReg(IMU_TEST_PWR_MGMT_1, 0x80U);
        ImuTest_WaitUs(100000U);
    }

    ok &= ImuTest_WriteReg(IMU_TEST_PWR_MGMT_1, 0x80U);
    ImuTest_WaitUs(100000U);
    ok &= ImuTest_SelectBank(IMU_TEST_BANK_0);
    ok &= ImuTest_ReadReg(IMU_TEST_WHO_AM_I, &id, 1U);
    ImuTest_DeviceId = id;
    if (ok == 0U || id != IMU_TEST_WHO_AM_I_VALUE)
    {
        ImuTest_ReadFail++;
        return;
    }

    ok &= ImuTest_WriteReg(IMU_TEST_USER_CTRL, 0x00U);
    ok &= ImuTest_WriteReg(IMU_TEST_PWR_MGMT_1, 0x01U);
    ImuTest_WaitUs(10000U);
    ok &= ImuTest_SelectBank(IMU_TEST_BANK_2);
    ok &= ImuTest_WriteReg(IMU_TEST_GYRO_SMPLRT_DIV, 0x04U);
    ok &= ImuTest_WriteReg(IMU_TEST_GYRO_CONFIG_1, 0x1FU);
    ok &= ImuTest_WriteReg(IMU_TEST_ACCEL_DIV_2, 0x04U);
    ok &= ImuTest_WriteReg(IMU_TEST_ACCEL_CONFIG, 0x29U);
    ok &= ImuTest_SelectBank(IMU_TEST_BANK_0);
    ok &= ImuTest_WriteReg(IMU_TEST_INT_PIN_CFG, 0x02U);

    if (ok == 0U)
    {
        ImuTest_ReadFail++;
        return;
    }

    ImuTest_Online = 1U;
}

void ImuTest_Task(uint32_t now_us)
{
    uint8_t data[12];

    if ((uint32_t)(now_us - imu_last_update_us) < IMU_TEST_UPDATE_US)
    {
        return;
    }
    imu_last_update_us = now_us;

    if (ImuTest_Online == 0U)
    {
        return;
    }

    if (ImuTest_ReadReg(IMU_TEST_ACCEL_XOUT_H, data, 12U) == 0U)
    {
        ImuTest_ReadFail++;
        return;
    }

    ImuTest_AccelX = ImuTest_ToInt16(data[0], data[1]);
    ImuTest_AccelY = ImuTest_ToInt16(data[2], data[3]);
    ImuTest_AccelZ = ImuTest_ToInt16(data[4], data[5]);
    ImuTest_GyroX = ImuTest_ToInt16(data[6], data[7]);
    ImuTest_GyroY = ImuTest_ToInt16(data[8], data[9]);
    ImuTest_GyroZ = ImuTest_ToInt16(data[10], data[11]);
    ImuTest_PitchX10 = ImuTest_CalcPitchX10(ImuTest_AccelX, ImuTest_AccelY, ImuTest_AccelZ);
    ImuTest_ReadOk++;
}
