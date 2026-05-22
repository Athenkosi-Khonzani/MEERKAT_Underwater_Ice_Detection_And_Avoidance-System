#ifndef TOF_H
#define TOF_H

#include "stm32f0xx_hal.h" // use F4

// The scanner confirmed the 8-bit address is exactly 0x52
#define TOF_ADDR 0x52
void ToF_SaveCalibration(void);
uint8_t ToF_LoadCalibration(void);
void ToF_EraseCalibration(void);
// Function Prototypes
HAL_StatusTypeDef ToF_Init(I2C_HandleTypeDef *hi2c);
uint16_t ToF_ReadDistance(I2C_HandleTypeDef *hi2c);

// Advanced Calibration Prototypes
uint16_t ToF_GetRawAverage(I2C_HandleTypeDef *hi2c, uint8_t samples);
void ToF_Calibrate_TwoPoint(uint16_t true_dist_1, uint16_t raw_meas_1,
                            uint16_t true_dist_2, uint16_t raw_meas_2);

#endif // TOF_H
