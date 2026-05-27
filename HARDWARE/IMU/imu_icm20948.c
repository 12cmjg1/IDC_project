#include "imu_icm20948.h"
#include "remote.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include <stdint.h>

#define ICM20948_ADDR             0xD0U
#define ICM20948_WHO_AM_I         0x00U
#define ICM20948_WHO_AM_I_VALUE   0xEAU
#define ICM20948_USER_CTRL        0x03U
#define ICM20948_PWR_MGMT_1       0x06U
#define ICM20948_PWR_MGMT_2       0x07U
#define ICM20948_GYRO_XOUT_H      0x33U
#define ICM20948_REG_BANK_SEL     0x7FU
#define ICM20948_BANK_0           0x00U
#define ICM20948_BANK_2           0x20U
#define ICM20948_GYRO_SMPLRT_DIV  0x00U
#define ICM20948_GYRO_CONFIG_1    0x01U

#define IMU_UPDATE_PERIOD_US      10000U
#define IMU_CALIBRATE_SAMPLES     100U
#define IMU_I2C_TIMEOUT           80U

volatile uint8_t Imu_Type = IMU_TYPE_NONE;
volatile uint8_t Imu_Ready = 0;
volatile uint8_t Imu_InitFailCode = 0;
volatile uint32_t Imu_FrameCount = 0;
volatile int16_t Imu_GyroZRaw = 0;
volatile int16_t Imu_GyroZOffset = 0;
volatile int32_t Imu_YawDeg10 = 0;

static uint32_t imu_last_update_us = 0;

static void Imu_I2cDelay(void)
{
    volatile uint32_t i;

    for (i = 0; i < 60U; i++)
    {
        __NOP();
    }
}

static void Imu_SdaOut(void)
{
    GPIOB->MODER &= ~(3U << (11U * 2U));
    GPIOB->MODER |= (1U << (11U * 2U));
}

static void Imu_SdaIn(void)
{
    GPIOB->MODER &= ~(3U << (11U * 2U));
}

static void Imu_Scl(uint8_t level)
{
    if (level != 0U)
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_10);
    }
    else
    {
        GPIO_ResetBits(GPIOB, GPIO_Pin_10);
    }
}

static void Imu_Sda(uint8_t level)
{
    if (level != 0U)
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_11);
    }
    else
    {
        GPIO_ResetBits(GPIOB, GPIO_Pin_11);
    }
}

static uint8_t Imu_ReadSda(void)
{
    return (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) != Bit_RESET) ? 1U : 0U;
}

static void Imu_DelayMs(uint32_t ms)
{
    uint32_t start_us = Remote_GetUs();
    uint32_t delay_us = ms * 1000U;

    while ((uint32_t)(Remote_GetUs() - start_us) < delay_us)
    {
    }
}

static void Imu_I2cGpioInit(void)
{
    GPIO_InitTypeDef gpio;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_OD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_12;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_OType = GPIO_OType_OD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &gpio);

    Imu_Scl(1U);
    Imu_Sda(1U);
}

static void Imu_I2cStart(void)
{
    Imu_SdaOut();
    Imu_Sda(1U);
    Imu_Scl(1U);
    Imu_I2cDelay();
    Imu_Sda(0U);
    Imu_I2cDelay();
    Imu_Scl(0U);
}

static void Imu_I2cStop(void)
{
    Imu_SdaOut();
    Imu_Scl(0U);
    Imu_Sda(0U);
    Imu_I2cDelay();
    Imu_Scl(1U);
    Imu_I2cDelay();
    Imu_Sda(1U);
    Imu_I2cDelay();
}

static uint8_t Imu_I2cWaitAck(void)
{
    uint32_t timeout = 0U;

    Imu_SdaIn();
    Imu_Sda(1U);
    Imu_I2cDelay();
    Imu_Scl(1U);
    Imu_I2cDelay();

    while (Imu_ReadSda() != 0U)
    {
        timeout++;
        if (timeout > IMU_I2C_TIMEOUT)
        {
            Imu_Scl(0U);
            Imu_I2cStop();
            return 0U;
        }
    }

    Imu_Scl(0U);
    return 1U;
}

static void Imu_I2cAck(uint8_t ack)
{
    Imu_Scl(0U);
    Imu_SdaOut();
    Imu_Sda((ack != 0U) ? 0U : 1U);
    Imu_I2cDelay();
    Imu_Scl(1U);
    Imu_I2cDelay();
    Imu_Scl(0U);
}

static uint8_t Imu_I2cWriteByte(uint8_t data)
{
    uint8_t i;

    Imu_SdaOut();
    Imu_Scl(0U);
    for (i = 0U; i < 8U; i++)
    {
        Imu_Sda((data & 0x80U) ? 1U : 0U);
        data <<= 1;
        Imu_I2cDelay();
        Imu_Scl(1U);
        Imu_I2cDelay();
        Imu_Scl(0U);
        Imu_I2cDelay();
    }

    return Imu_I2cWaitAck();
}

static uint8_t Imu_I2cReadByte(uint8_t ack)
{
    uint8_t i;
    uint8_t data = 0U;

    Imu_SdaIn();
    for (i = 0U; i < 8U; i++)
    {
        Imu_Scl(0U);
        Imu_I2cDelay();
        Imu_Scl(1U);
        data <<= 1;
        if (Imu_ReadSda() != 0U)
        {
            data |= 1U;
        }
        Imu_I2cDelay();
    }
    Imu_Scl(0U);
    Imu_I2cAck(ack);

    return data;
}

static uint8_t Imu_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t ok = 1U;

    Imu_I2cStart();
    ok &= Imu_I2cWriteByte(ICM20948_ADDR | 0U);
    ok &= Imu_I2cWriteByte(reg);
    ok &= Imu_I2cWriteByte(value);
    Imu_I2cStop();

    return ok;
}

static uint8_t Imu_ReadRegs(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t i;
    uint8_t ok = 1U;

    if (data == 0 || len == 0U)
    {
        return 0U;
    }

    Imu_I2cStart();
    ok &= Imu_I2cWriteByte(ICM20948_ADDR | 0U);
    ok &= Imu_I2cWriteByte(reg);
    Imu_I2cStart();
    ok &= Imu_I2cWriteByte(ICM20948_ADDR | 1U);

    if (ok == 0U)
    {
        Imu_I2cStop();
        return 0U;
    }

    for (i = 0U; i < len; i++)
    {
        data[i] = Imu_I2cReadByte((i + 1U < len) ? 1U : 0U);
    }
    Imu_I2cStop();

    return 1U;
}

static uint8_t Imu_ReadReg(uint8_t reg)
{
    uint8_t value = 0U;

    (void)Imu_ReadRegs(reg, &value, 1U);
    return value;
}

static void Imu_SelectBank(uint8_t bank)
{
    (void)Imu_WriteReg(ICM20948_REG_BANK_SEL, bank);
}

static int16_t Imu_ReadGyroZ(void)
{
    uint8_t data[6];

    if (Imu_ReadRegs(ICM20948_GYRO_XOUT_H, data, 6U) == 0U)
    {
        Imu_InitFailCode = 5U;
        return Imu_GyroZRaw;
    }

    return (int16_t)((uint16_t)data[4] << 8 | data[5]);
}

static int32_t Imu_NormalizeYawDeg10(int32_t yaw)
{
    while (yaw > 1800)
    {
        yaw -= 3600;
    }
    while (yaw < -1800)
    {
        yaw += 3600;
    }

    return yaw;
}

void Imu_Init(void)
{
    uint32_t i;
    int32_t sum = 0;

    Imu_Type = IMU_TYPE_NONE;
    Imu_Ready = 0U;
    Imu_InitFailCode = 0U;
    Imu_FrameCount = 0U;
    Imu_GyroZRaw = 0;
    Imu_GyroZOffset = 0;
    Imu_YawDeg10 = 0;

    Imu_I2cGpioInit();
    Imu_DelayMs(10U);

    Imu_SelectBank(ICM20948_BANK_0);
    if (Imu_ReadReg(ICM20948_WHO_AM_I) != ICM20948_WHO_AM_I_VALUE)
    {
        Imu_InitFailCode = 1U;
        return;
    }

    (void)Imu_WriteReg(ICM20948_PWR_MGMT_1, 0x80U);
    Imu_DelayMs(100U);

    Imu_SelectBank(ICM20948_BANK_0);
    (void)Imu_WriteReg(ICM20948_PWR_MGMT_1, 0x01U);
    (void)Imu_WriteReg(ICM20948_PWR_MGMT_2, 0x00U);
    (void)Imu_WriteReg(ICM20948_USER_CTRL, 0x00U);
    Imu_DelayMs(10U);

    Imu_SelectBank(ICM20948_BANK_2);
    (void)Imu_WriteReg(ICM20948_GYRO_SMPLRT_DIV, 0x09U);
    (void)Imu_WriteReg(ICM20948_GYRO_CONFIG_1, 0x13U);
    Imu_SelectBank(ICM20948_BANK_0);
    Imu_DelayMs(20U);

    for (i = 0U; i < IMU_CALIBRATE_SAMPLES; i++)
    {
        sum += Imu_ReadGyroZ();
        Imu_DelayMs(10U);
    }

    Imu_GyroZOffset = (int16_t)(sum / (int32_t)IMU_CALIBRATE_SAMPLES);
    Imu_Type = IMU_TYPE_ICM20948;
    Imu_Ready = 1U;
    imu_last_update_us = Remote_GetUs();
}

void Imu_Task(void)
{
    uint32_t now_us;
    uint32_t dt_us;
    int32_t gyro_corr;
    int64_t delta_deg10;

    if (Imu_Ready == 0U)
    {
        return;
    }

    now_us = Remote_GetUs();
    dt_us = now_us - imu_last_update_us;
    if (dt_us < IMU_UPDATE_PERIOD_US)
    {
        return;
    }

    imu_last_update_us = now_us;
    Imu_GyroZRaw = Imu_ReadGyroZ();
    gyro_corr = (int32_t)Imu_GyroZRaw - (int32_t)Imu_GyroZOffset;

    delta_deg10 = ((int64_t)gyro_corr * (int64_t)dt_us * 100LL) / 655000000LL;
    Imu_YawDeg10 = Imu_NormalizeYawDeg10(Imu_YawDeg10 + (int32_t)delta_deg10);
    Imu_FrameCount++;
}

void Imu_ResetYaw(void)
{
    Imu_YawDeg10 = 0;
    imu_last_update_us = Remote_GetUs();
}

int32_t Imu_GetYawDeg10(void)
{
    return Imu_YawDeg10;
}

int32_t Imu_GetYawDeltaDeg10(int32_t start_yaw_deg10)
{
    return Imu_NormalizeYawDeg10(Imu_YawDeg10 - start_yaw_deg10);
}
