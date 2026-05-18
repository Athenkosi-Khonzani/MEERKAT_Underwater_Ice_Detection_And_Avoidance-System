/**
 ******************************************************************************
 * @file    battery.h
 * @brief   Li-ion Battery SOC and Runtime Estimator — 3S LiPo Pack
 *
 * Configured for 3 × 3.7 V Li-ion cells in series (3S):
 *   Full charge : 3 × 4.20 V = 12.60 V
 *   Empty       : 3 × 3.00 V =  9.00 V
 *   Capacity    : 950 mAh  (series = same mAh as one cell)
 *
 * HARDWARE NOTE — voltage divider required:
 *   The INA219 bus voltage input is rated to 26 V max.
 *   The 3S pack reaches 12.6 V which is within range, BUT the INA219
 *   internal ADC saturates above ~16 V in the 16 V config mode.
 *   This code uses the 32 V config mode (safe up to 26 V on the V+ pin).
 *
 *   If your INA219 V+ pin sees the full pack voltage directly, you are fine.
 *   If you added a resistor divider (e.g. R1=10k, R2=6.8k → ratio=0.405),
 *   set BAT_VDIV_RATIO below to match your divider so the reading is
 *   scaled back to the true pack voltage.
 *   Set BAT_VDIV_RATIO to 1.0f if no divider is used.
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

/** Number of cells in series */
#define BAT_SERIES_CELLS            3U

/** Rated capacity of the PACK in mAh
 *  Series cells share the same mAh as a single cell — do NOT multiply. */
#define BAT_CAPACITY_MAH            1000.0f

/** Pack voltage when fully charged (3 × 4.20 V) */
#define BAT_VOLTAGE_FULL_V          12.60f

/** Pack cutoff voltage — empty (3 × 3.00 V) */
#define BAT_VOLTAGE_EMPTY_V         8.4f

/** Voltage divider ratio applied to pack voltage before INA219 V+ pin.
 *  true_voltage = ina_reading / BAT_VDIV_RATIO
 *  Example: R1=10k, R2=6.8k → ratio = 6.8/(10+6.8) = 0.4048f
 *  Set to 1.0f if the full pack voltage goes directly to INA219 V+. */
#define BAT_VDIV_RATIO              1.0f

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
