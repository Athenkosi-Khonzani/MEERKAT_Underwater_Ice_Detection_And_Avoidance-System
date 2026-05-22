#ifndef TEMP_SENSOR_H
#define TEMP_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void  TempSensor_Init(ADC_HandleTypeDef *hadc);
float TempSensor_ReadC(void);

#ifdef __cplusplus
}
#endif

#endif
