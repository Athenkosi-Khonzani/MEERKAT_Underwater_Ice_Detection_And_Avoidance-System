/**
 ******************************************************************************
 * @file    battery.h
 * @brief   Li-ion Battery SOC and Runtime Estimator
 *
 * Uses INA219 coulomb counting + OCV-based initial SOC estimate.
 * Designed for 18650 Li-ion cells, configurable for other capacities.
 ******************************************************************************
 */

#ifndef BATTERY_H
#define BATTERY_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------
 * Configuration — adjust for your cell / application
 * ------------------------------------------------------------------------- */

/** Rated capacity of the cell in mAh */
#define BAT_CAPACITY_MAH            1000.0f

/** Cutoff voltage (cell is considered empty below this) */
#define BAT_VOLTAGE_EMPTY_V         9.0f

/** Voltage at which OCV lookup table starts (0 % SOC) */
#define BAT_VOLTAGE_FULL_V          12.6f

/** SOC thresholds for status flags */
#define BAT_SOC_OK_THRESHOLD        60.0f   /* BATTERY_OK:       100% – 60% */
#define BAT_SOC_WARNING_THRESHOLD   30.0f   /* BATTERY_WARNING:   60% – 30% */
                                            /* BATTERY_CRITICAL:  30% –  0% */

/** How often to sample the INA219 (ms) */
#define BAT_SAMPLE_PERIOD_MS        500U

/* -------------------------------------------------------------------------
 * Types
 * ------------------------------------------------------------------------- */

/**
 * @brief Battery health / charge status indicator.
 */
typedef enum
{
    BATTERY_OK       = 0,   /**< SOC >= 60 % — green */
    BATTERY_WARNING  = 1,   /**< SOC 30–60 % — amber */
    BATTERY_CRITICAL = 2    /**< SOC < 30 %  — red   */
} BatteryStatus_t;

/**
 * @brief All battery metrics in one struct.
 *
 * Read these fields after calling Battery_Update() each cycle.
 */
typedef struct
{
    float           soc_pct;        /**< State of charge 0.0–100.0 % */
    float           voltage_V;      /**< Terminal voltage in volts */
    float           current_A;      /**< Load current in amps (positive = discharging) */
    float           used_mAh;       /**< Charge consumed since init */
    float           remaining_mAh;  /**< Estimated charge remaining */
    uint32_t        runtime_sec;    /**< Estimated runtime in seconds */
    BatteryStatus_t status;         /**< OK / WARNING / CRITICAL flag */
    bool            valid;          /**< false until first successful read */
} BatteryData_t;

/* -------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

/**
 * @brief  Initialise the battery estimator.
 *
 * Reads OCV from the INA219 to seed the initial SOC estimate, then begins
 * coulomb counting from that point forward.
 *
 * Call once after INA219_Init() has succeeded.
 *
 * @param  hi2c   Pointer to the I2C handle used by the INA219.
 * @retval HAL_OK on success, HAL_ERROR if INA219 cannot be read.
 */
HAL_StatusTypeDef Battery_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief  Update coulomb counter and recalculate all metrics.
 *
 * Call this in your main loop every cycle. Internally rate-limits sampling
 * to BAT_SAMPLE_PERIOD_MS; safe to call more frequently.
 *
 * @retval HAL_OK on success, HAL_ERROR on INA219 read failure.
 */
HAL_StatusTypeDef Battery_Update(void);

/**
 * @brief  Get the latest battery data snapshot.
 *
 * @param  out  Pointer to a BatteryData_t to fill.
 */
void Battery_GetData(BatteryData_t *out);

/**
 * @brief  Convert runtime seconds to HH:MM:SS components.
 *
 * Convenience helper for LCD / UART display.
 *
 * @param  seconds  Total seconds from BatteryData_t.runtime_sec.
 * @param  h        Output hours.
 * @param  m        Output minutes (0–59).
 * @param  s        Output seconds (0–59).
 */
void Battery_RuntimeToHMS(uint32_t seconds, uint32_t *h, uint32_t *m, uint32_t *s);

#endif /* BATTERY_H */
