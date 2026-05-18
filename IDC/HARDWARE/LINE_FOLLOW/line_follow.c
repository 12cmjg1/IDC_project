#include "line_follow.h"
#include "remote.h"
#include "misc.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

#define CCD_PIXEL_COUNT                  128U
#define CCD_VALID_LEFT                   5U
#define CCD_VALID_RIGHT                  122U
#define CCD_CENTER_PIXEL                 63
#define LINEFOLLOW_CAPTURE_PERIOD_US     10000U
#define LINEFOLLOW_CROSS_THRESHOLD       30U
#define LINEFOLLOW_LOST_WIDTH_THRESHOLD  4U
#define LINEFOLLOW_BASE_ERPM_MAX         7000
#define LINEFOLLOW_STEER_MAX_ERPM        5000
#define LINEFOLLOW_LOST_STEER_ERPM       3500
#define LINEFOLLOW_KP_NUM                4
#define LINEFOLLOW_KP_DEN                1
#define LINEFOLLOW_KD_NUM                2
#define LINEFOLLOW_KD_DEN                1

volatile uint8_t LineFollow_Enabled = 0;
volatile uint16_t LineFollow_Center = CCD_CENTER_PIXEL;
volatile uint16_t LineFollow_Threshold = 0;
volatile uint16_t LineFollow_Width = 0;
volatile int16_t LineFollow_Position = 0;
volatile uint8_t LineFollow_Cross = 0;
volatile uint8_t LineFollow_Lost = 1;
volatile int16_t LineFollow_LeftCmdErpm = 0;
volatile int16_t LineFollow_RightCmdErpm = 0;
volatile uint32_t LineFollow_FrameCount = 0;
volatile uint32_t LineFollow_CrossCount = 0;
volatile uint32_t LineFollow_KeyToggleCount = 0;
volatile uint8_t LineFollow_Reverse = 0;

static uint16_t ccd_pixels[CCD_PIXEL_COUNT];
static uint16_t ccd_pixels_flipped[CCD_PIXEL_COUNT];
static int16_t last_position = 0;
static uint32_t last_capture_us = 0;
static uint32_t last_cross_us = 0;

static void LineFollow_SmallDelay(void)
{
    volatile uint32_t i;

    for (i = 0; i < 30U; i++)
    {
        __NOP();
    }
}

static uint16_t LineFollow_ReadAdc(void)
{
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1U, ADC_SampleTime_480Cycles);
    ADC_SoftwareStartConv(ADC1);

    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET)
    {
    }

    return ADC_GetConversionValue(ADC1);
}

static void LineFollow_CCD_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOE, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    ADC_CommonStructInit(&ADC_CommonInitStructure);
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
    ADC_CommonInit(&ADC_CommonInitStructure);

    ADC_StructInit(&ADC_InitStructure);
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 1U;
    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_Cmd(ADC1, ENABLE);

    GPIO_ResetBits(GPIOA, GPIO_Pin_5);
    GPIO_ResetBits(GPIOC, GPIO_Pin_4);
}

static void LineFollow_CaptureFrame(void)
{
    uint32_t i;

    GPIO_ResetBits(GPIOA, GPIO_Pin_5);
    GPIO_ResetBits(GPIOC, GPIO_Pin_4);
    LineFollow_SmallDelay();

    GPIO_SetBits(GPIOC, GPIO_Pin_4);
    LineFollow_SmallDelay();

    GPIO_ResetBits(GPIOA, GPIO_Pin_5);
    LineFollow_SmallDelay();

    GPIO_SetBits(GPIOA, GPIO_Pin_5);
    LineFollow_SmallDelay();

    GPIO_ResetBits(GPIOC, GPIO_Pin_4);
    LineFollow_SmallDelay();

    for (i = 0; i < CCD_PIXEL_COUNT; i++)
    {
        GPIO_ResetBits(GPIOA, GPIO_Pin_5);
        LineFollow_SmallDelay();
        ccd_pixels[i] = (uint16_t)(LineFollow_ReadAdc() >> 4);
        GPIO_SetBits(GPIOA, GPIO_Pin_5);
        LineFollow_SmallDelay();
    }
}

static void LineFollow_ProcessFrame(void)
{
    uint16_t max_v = ccd_pixels[CCD_VALID_LEFT];
    uint16_t min_v = ccd_pixels[CCD_VALID_LEFT];
    uint16_t left = CCD_VALID_LEFT;
    uint16_t right = CCD_VALID_RIGHT;
    uint16_t i;
    uint8_t left_found = 0U;
    uint8_t right_found = 0U;
    int16_t position;

    for (i = CCD_VALID_LEFT; i <= CCD_VALID_RIGHT; i++)
    {
        if (ccd_pixels[i] > max_v)
        {
            max_v = ccd_pixels[i];
        }
        if (ccd_pixels[i] < min_v)
        {
            min_v = ccd_pixels[i];
        }
    }

    LineFollow_Threshold = (uint16_t)((max_v + min_v) / 2U);

    for (i = CCD_VALID_LEFT; i < (CCD_VALID_RIGHT - 5U); i++)
    {
        if (ccd_pixels[i] > LineFollow_Threshold &&
            ccd_pixels[i + 1U] > LineFollow_Threshold &&
            ccd_pixels[i + 2U] > LineFollow_Threshold &&
            ccd_pixels[i + 3U] < LineFollow_Threshold &&
            ccd_pixels[i + 4U] < LineFollow_Threshold &&
            ccd_pixels[i + 5U] < LineFollow_Threshold)
        {
            left = (uint16_t)(i + 2U);
            left_found = 1U;
            break;
        }
    }

    for (i = (CCD_VALID_RIGHT - 5U); i > CCD_VALID_LEFT; i--)
    {
        if (ccd_pixels[i] < LineFollow_Threshold &&
            ccd_pixels[i + 1U] < LineFollow_Threshold &&
            ccd_pixels[i + 2U] < LineFollow_Threshold &&
            ccd_pixels[i + 3U] > LineFollow_Threshold &&
            ccd_pixels[i + 4U] > LineFollow_Threshold &&
            ccd_pixels[i + 5U] > LineFollow_Threshold)
        {
            right = (uint16_t)(i + 2U);
            right_found = 1U;
            break;
        }
    }

    if (left_found == 0U || right_found == 0U || right <= left)
    {
        LineFollow_Lost = 1U;
        LineFollow_Cross = 0U;
        LineFollow_Width = 0U;
        LineFollow_Center = CCD_CENTER_PIXEL;
        LineFollow_Position = (last_position >= 0) ? 1000 : -1000;
        return;
    }

    LineFollow_Width = (uint16_t)(right - left);
    LineFollow_Center = (uint16_t)((right + left) / 2U);
    position = (int16_t)(((int32_t)LineFollow_Center - CCD_CENTER_PIXEL) * 1000 / CCD_CENTER_PIXEL);

    if (position > 1000)
    {
        position = 1000;
    }
    else if (position < -1000)
    {
        position = -1000;
    }

    LineFollow_Position = position;
    LineFollow_Lost = (LineFollow_Width < LINEFOLLOW_LOST_WIDTH_THRESHOLD) ? 1U : 0U;
    LineFollow_Cross = (LineFollow_Width > LINEFOLLOW_CROSS_THRESHOLD) ? 1U : 0U;
}

void LineFollow_Init(void)
{
    LineFollow_CCD_Init();
}

uint8_t LineFollow_IsEnabled(void)
{
    return LineFollow_Enabled;
}

void LineFollow_SetEnabled(uint8_t enabled)
{
    LineFollow_Enabled = (enabled != 0U) ? 1U : 0U;
    LineFollow_KeyToggleCount++;

    if (LineFollow_Enabled == 0U)
    {
        LineFollow_LeftCmdErpm = 0;
        LineFollow_RightCmdErpm = 0;
    }
}

void LineFollow_Task(uint16_t speed_scale)
{
    int32_t base_erpm;
    int32_t steer_erpm;
    int32_t diff;
    int32_t left;
    int32_t right;
    uint32_t now_us = Remote_GetUs();

    if ((uint32_t)(now_us - last_capture_us) >= LINEFOLLOW_CAPTURE_PERIOD_US)
    {
        last_capture_us = now_us;
        LineFollow_CaptureFrame();

        /* Reverse 模式下翻转 CCD 像素 (车转180后左右对调) */
        if (LineFollow_Reverse != 0U)
        {
            uint32_t j;
            for (j = 0; j < CCD_PIXEL_COUNT; j++)
            {
                ccd_pixels_flipped[j] = ccd_pixels[CCD_PIXEL_COUNT - 1U - j];
            }
            for (j = 0; j < CCD_PIXEL_COUNT; j++)
            {
                ccd_pixels[j] = ccd_pixels_flipped[j];
            }
        }

        LineFollow_ProcessFrame();
        LineFollow_FrameCount++;

        if (LineFollow_Cross != 0U && (uint32_t)(now_us - last_cross_us) >= 200000U)
        {
            last_cross_us = now_us;
            LineFollow_CrossCount++;
        }
    }

    if (LineFollow_Enabled == 0U || speed_scale == 0U)
    {
        LineFollow_LeftCmdErpm = 0;
        LineFollow_RightCmdErpm = 0;
        return;
    }

    base_erpm = ((int32_t)speed_scale * LINEFOLLOW_BASE_ERPM_MAX) / 1000;

    if (LineFollow_Reverse != 0U)
    {
        base_erpm = -base_erpm;
    }

    if (LineFollow_Lost != 0U)
    {
        steer_erpm = (last_position >= 0) ? LINEFOLLOW_LOST_STEER_ERPM : -LINEFOLLOW_LOST_STEER_ERPM;
    }
    else
    {
        diff = (int32_t)LineFollow_Position - (int32_t)last_position;
        steer_erpm = ((int32_t)LineFollow_Position * LINEFOLLOW_KP_NUM) / LINEFOLLOW_KP_DEN;
        steer_erpm += (diff * LINEFOLLOW_KD_NUM) / LINEFOLLOW_KD_DEN;
    }

    if (steer_erpm > LINEFOLLOW_STEER_MAX_ERPM)
    {
        steer_erpm = LINEFOLLOW_STEER_MAX_ERPM;
    }
    else if (steer_erpm < -LINEFOLLOW_STEER_MAX_ERPM)
    {
        steer_erpm = -LINEFOLLOW_STEER_MAX_ERPM;
    }

    if (LineFollow_Reverse != 0U)
    {
        left = base_erpm - steer_erpm;
        right = base_erpm + steer_erpm;
    }
    else
    {
        left = base_erpm + steer_erpm;
        right = base_erpm - steer_erpm;
    }

    if (left > 12000)
    {
        left = 12000;
    }
    else if (left < -12000)
    {
        left = -12000;
    }

    if (right > 12000)
    {
        right = 12000;
    }
    else if (right < -12000)
    {
        right = -12000;
    }

    LineFollow_LeftCmdErpm = (int16_t)left;
    LineFollow_RightCmdErpm = (int16_t)right;
    last_position = LineFollow_Position;
}
