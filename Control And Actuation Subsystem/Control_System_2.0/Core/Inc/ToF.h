#ifndef TOF_H
#define TOF_H

#include "stm32f4xx_hal.h"

// The scanner confirmed the 8-bit address is exactly 0x52
#define TOF_ADDR 0x52

/* Reference distances used during 2-point calibration (mm).
   Adjust to match the physical setup you can repeatably create. */
#define TOF_CAL_NEAR_REF_MM   60U
#define TOF_CAL_FAR_REF_MM    120U
#define TOF_CAL_SAMPLES       50U

// Function Prototypes
HAL_StatusTypeDef ToF_Init(I2C_HandleTypeDef *hi2c);

/* Returns calibrated distance in mm, or 0 if no new reading is ready. */
uint16_t ToF_ReadDistance(I2C_HandleTypeDef *hi2c);

/* Returns the raw, uncalibrated distance — useful during calibration. */
uint16_t ToF_ReadDistanceRaw(I2C_HandleTypeDef *hi2c);

/* Average N raw samples. Blocks ~N * 60 ms. Useful for calibration capture. */
uint16_t ToF_AverageRaw(I2C_HandleTypeDef *hi2c, uint16_t samples);

/* Compute and persist a new 2-point calibration.
   Returns 1 on success, 0 on failure (degenerate inputs or flash error). */
uint8_t ToF_Calibrate(uint16_t measured_near_mm,
                      uint16_t measured_far_mm,
                      uint16_t actual_near_mm,
                      uint16_t actual_far_mm);

/* Clears stored calibration and reverts to identity (slope=1, offset=0). */
uint8_t ToF_ClearCalibration(void);

/* Inspect current calibration. Any output pointer may be NULL. */
void ToF_GetCalibration(float *slope, float *offset_mm, uint8_t *valid);

#endif
