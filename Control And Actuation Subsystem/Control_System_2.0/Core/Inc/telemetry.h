/**
 ******************************************************************************
 * @file    telemetry.h
 * @brief   Formats UAV telemetry as JSON and transmits over UART to ESP32
 ******************************************************************************
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "battery.h"
#include <stdint.h>
#include <stdbool.h>

/* ----- Battery types (mirror your application's definitions) -------------- */

/* ----- Sentinels for "sensor unavailable" --------------------------------- */

#define TELEM_DIST_INVALID   ((uint16_t)0xFFFFu)
#define TELEM_TEMP_INVALID   (-999.0f)

/* ----- API ---------------------------------------------------------------- */

/**
 * @brief  Initialise the telemetry module.
 *         Stores the UART handle used for all subsequent transmissions.
 * @param  huart  Pointer to the HAL UART handle wired to the ESP32.
 */
void Telemetry_Init(UART_HandleTypeDef *huart);

/**
 * @brief  Format and transmit one telemetry packet.
 *         Builds a newline-terminated JSON line and sends it over the UART
 *         previously registered with Telemetry_Init().
 *
 * @param  battery   Pointer to current battery data (must not be NULL).
 * @param  tof_mm    ToF distance in mm. Pass TELEM_DIST_INVALID if unavailable.
 * @param  sonar_mm  Sonar distance in mm. Pass TELEM_DIST_INVALID if unavailable.
 * @param  mcu_temp_c MCU internal temperature in °C.
 *                   Pass TELEM_TEMP_INVALID if unavailable.
 *
 * @retval HAL_OK on success, HAL_ERROR on format/transmit failure.
 */
HAL_StatusTypeDef Telemetry_Send(const BatteryData_t *battery,
                                  uint16_t tof_mm,
                                  uint16_t sonar_mm,
                                  float    mcu_temp_c);

/**
 * @brief  Returns the last sequence number sent. Useful for debug.
 */
uint32_t Telemetry_GetSeq(void);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_H */
