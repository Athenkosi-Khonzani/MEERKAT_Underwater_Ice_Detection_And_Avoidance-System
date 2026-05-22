#include "temp_sensor.h"

/* Factory calibration values for STM32F446 (RM0390) */
#define TS_CAL1     (*(uint16_t *)0x1FFF7A2C)   /* raw ADC at 30°C  */
#define TS_CAL2     (*(uint16_t *)0x1FFF7A2E)   /* raw ADC at 110°C */

static uint16_t adc_buf[1];   /* DMA target — one sample */

void TempSensor_Init(ADC_HandleTypeDef *hadc)
{
    HAL_ADC_Start_DMA(hadc, (uint32_t *)adc_buf, 1);
}

float TempSensor_ReadC(void)
{
    uint16_t raw = adc_buf[0];

    /* Linear interpolation between factory cal points */
    return ((110.0f - 30.0f) / (float)(TS_CAL2 - TS_CAL1))
           * ((float)raw - (float)TS_CAL1)
           + 30.0f;
}
