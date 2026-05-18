/**
 ******************************************************************************
 * @file    battery.c
 * @brief   Li-ion Battery SOC and Runtime Estimator — Implementation
 *
 * Algorithm
 * ---------
 *  1. On init, read OCV (no-load voltage) and map it through a lookup table
 *     to get the starting SOC (open-circuit voltage method).
 *  2. Every BAT_SAMPLE_PERIOD_MS, read voltage + current from the INA219.
 *  3. Integrate current over time (coulomb counting) to track charge used.
 *  4. Blend OCV correction into the coulomb-counted SOC slowly, so resting
 *     voltage nudges the estimate without causing sudden jumps under load.
 *  5. Derive estimated runtime from remaining capacity / average current.
 ******************************************************************************
 */

#include "battery.h"

/* -------------------------------------------------------------------------
 * INA219 register / address definitions (mirrors what is in main.c)
 * If you move INA219 code to its own driver file, include that header
 * instead and remove these defines.
 * ------------------------------------------------------------------------- */

#define _INA219_ADDR_7BIT       0x40
#define _INA219_ADDR            (_INA219_ADDR_7BIT << 1)

#define _INA219_REG_CONFIG      0x00
#define _INA219_REG_BUS_VOLT    0x02
#define _INA219_REG_CURRENT     0x04
#define _INA219_REG_CALIBRATION 0x05

/* 32 V range, gain /8 (320 mV shunt), 12-bit, continuous.
 * 32 V mode is required for a 3S pack (up to 12.6 V on V+ pin). */
#define _INA219_CONFIG_VALUE    0x399F
/* 0.1 Ω shunt, 100 µA current LSB → CAL = 4096 */
#define _INA219_CAL_VALUE       4096

/* -------------------------------------------------------------------------
 * OCV → SOC lookup table for a 3S Li-ion pack (3 cells in series)
 *
 * Each voltage point = single-cell OCV × 3.
 * Full = 3 × 4.20 V = 12.60 V  |  Empty = 3 × 3.00 V = 9.00 V
 *
 * Columns: { pack OCV in volts, SOC in percent }
 * Must be sorted descending by voltage.
 * ------------------------------------------------------------------------- */

typedef struct { float voltage; float soc; } OcvPoint_t;

static const OcvPoint_t ocv_table[] =
{
    { 12.60f, 100.0f },   /* 3 × 4.20 V */
    { 12.45f,  95.0f },   /* 3 × 4.15 V */
    { 12.30f,  90.0f },   /* 3 × 4.10 V */
    { 12.15f,  85.0f },   /* 3 × 4.05 V */
    { 12.00f,  80.0f },   /* 3 × 4.00 V */
    { 11.85f,  75.0f },   /* 3 × 3.95 V */
    { 11.70f,  70.0f },   /* 3 × 3.90 V */
    { 11.55f,  65.0f },   /* 3 × 3.85 V */
    { 11.40f,  60.0f },   /* 3 × 3.80 V */
    { 11.25f,  55.0f },   /* 3 × 3.75 V */
    { 11.10f,  50.0f },   /* 3 × 3.70 V */
    { 10.95f,  45.0f },   /* 3 × 3.65 V */
    { 10.80f,  40.0f },   /* 3 × 3.60 V */
    { 10.65f,  35.0f },   /* 3 × 3.55 V */
    { 10.50f,  30.0f },   /* 3 × 3.50 V */
    { 10.35f,  25.0f },   /* 3 × 3.45 V */
    { 10.20f,  20.0f },   /* 3 × 3.40 V */
    {  9.90f,  15.0f },   /* 3 × 3.30 V */
    {  9.60f,  10.0f },   /* 3 × 3.20 V */
    {  9.30f,   5.0f },   /* 3 × 3.10 V */
    {  9.00f,   0.0f },   /* 3 × 3.00 V */
};

#define OCV_TABLE_SIZE  (sizeof(ocv_table) / sizeof(ocv_table[0]))

/* -------------------------------------------------------------------------
 * Runtime state — all private to this file
 * ------------------------------------------------------------------------- */

static I2C_HandleTypeDef *s_hi2c        = NULL;

static float    s_soc_pct               = 0.0f;
static float    s_used_mAh              = 0.0f;
static float    s_voltage_V             = 0.0f;
static float    s_current_A             = 0.0f;

/* Exponential moving average for current — smooths runtime estimate */
static float    s_avg_current_A         = 0.0f;
#define EMA_ALPHA                       0.10f   /* 0 = no update, 1 = instant */

static uint32_t s_last_sample_ms        = 0U;
static bool     s_initialised           = false;
static bool     s_valid                 = false;

/* -------------------------------------------------------------------------
 * Private helpers
 * ------------------------------------------------------------------------- */

static HAL_StatusTypeDef ina_write(uint8_t reg, uint16_t value)
{
    uint8_t buf[3] = { reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF) };
    return HAL_I2C_Master_Transmit(s_hi2c, _INA219_ADDR, buf, 3, HAL_MAX_DELAY);
}

static HAL_StatusTypeDef ina_read(uint8_t reg, uint16_t *out)
{
    uint8_t buf[2];
    if (HAL_I2C_Master_Transmit(s_hi2c, _INA219_ADDR, &reg, 1, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;
    if (HAL_I2C_Master_Receive(s_hi2c, _INA219_ADDR, buf, 2, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;
    *out = ((uint16_t)buf[0] << 8) | buf[1];
    return HAL_OK;
}

static HAL_StatusTypeDef ina_read_voltage(float *v)
{
    uint16_t raw;
    if (ina_read(_INA219_REG_BUS_VOLT, &raw) != HAL_OK) return HAL_ERROR;
    raw >>= 3;                              /* bus voltage LSB = 4 mV */
    float ina_v = (float)raw * 0.004f;

    /* Scale back to true pack voltage if a resistor divider is used.
     * If BAT_VDIV_RATIO = 1.0f this line has no effect. */
    *v = ina_v / BAT_VDIV_RATIO;

    return HAL_OK;
}

static HAL_StatusTypeDef ina_read_current(float *i)
{
    uint16_t raw;
    if (ina_read(_INA219_REG_CURRENT, &raw) != HAL_OK) return HAL_ERROR;
    int16_t signed_raw = (int16_t)raw;  /* current LSB = 100 µA */
    *i = (float)signed_raw * 0.0001f;
    return HAL_OK;
}

/**
 * @brief Linear interpolation through the OCV table.
 * @param  voltage  Open-circuit voltage in volts.
 * @return Estimated SOC in percent (clamped 0–100).
 */
static float ocv_to_soc(float voltage)
{
    /* Above full charge */
    if (voltage >= ocv_table[0].voltage)
        return 100.0f;

    /* Below empty */
    if (voltage <= ocv_table[OCV_TABLE_SIZE - 1].voltage)
        return 0.0f;

    /* Find the two bracketing points and interpolate */
    for (uint8_t i = 0; i < (OCV_TABLE_SIZE - 1U); i++)
    {
        if (voltage <= ocv_table[i].voltage && voltage > ocv_table[i + 1U].voltage)
        {
            float v_hi  = ocv_table[i].voltage;
            float v_lo  = ocv_table[i + 1U].voltage;
            float s_hi  = ocv_table[i].soc;
            float s_lo  = ocv_table[i + 1U].soc;
            float ratio = (voltage - v_lo) / (v_hi - v_lo);
            return s_lo + ratio * (s_hi - s_lo);
        }
    }

    return 0.0f;   /* Should never reach here */
}

/**
 * @brief Clamp a float to [lo, hi].
 */
static float clamp_f(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/**
 * @brief Derive BatteryStatus_t from SOC percentage.
 */
static BatteryStatus_t soc_to_status(float soc)
{
    if (soc >= BAT_SOC_OK_THRESHOLD)      return BATTERY_OK;
    if (soc >= BAT_SOC_WARNING_THRESHOLD) return BATTERY_WARNING;
    return BATTERY_CRITICAL;
}

/* -------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

HAL_StatusTypeDef Battery_Init(I2C_HandleTypeDef *hi2c)
{
    s_hi2c = hi2c;

    /* Configure INA219 */
    HAL_Delay(5U);
    if (ina_write(_INA219_REG_CALIBRATION, _INA219_CAL_VALUE) != HAL_OK) return HAL_ERROR;
    if (ina_write(_INA219_REG_CONFIG,      _INA219_CONFIG_VALUE) != HAL_OK) return HAL_ERROR;

    /* Seed SOC from OCV — battery should be at rest (no load) at this point */
    float v = 0.0f;
    if (ina_read_voltage(&v) != HAL_OK) return HAL_ERROR;

    s_voltage_V     = v;
    s_soc_pct       = ocv_to_soc(v);
    s_used_mAh      = BAT_CAPACITY_MAH * (1.0f - (s_soc_pct / 100.0f));
    s_current_A     = 0.0f;
    s_avg_current_A = 0.0f;

    s_last_sample_ms = HAL_GetTick();
    s_initialised    = true;
    s_valid          = true;

    return HAL_OK;
}

HAL_StatusTypeDef Battery_Update(void)
{
    if (!s_initialised) return HAL_ERROR;

    uint32_t now = HAL_GetTick();
    if ((now - s_last_sample_ms) < BAT_SAMPLE_PERIOD_MS) return HAL_OK;

    uint32_t dt_ms   = now - s_last_sample_ms;
    s_last_sample_ms = now;

    /* ── 1. Read sensors ────────────────────────────────────────────────── */
    float voltage = 0.0f;
    float current = 0.0f;

    if (ina_read_voltage(&voltage) != HAL_OK) { s_valid = false; return HAL_ERROR; }
    if (ina_read_current(&current) != HAL_OK) { s_valid = false; return HAL_ERROR; }

    /* Ensure current is positive for discharging */
    if (current < 0.0f) current = -current;

    s_voltage_V = voltage;
    s_current_A = current;

    /* ── 2. Coulomb counting ────────────────────────────────────────────── */
    float dt_h    = (float)dt_ms / 3600000.0f;
    float delta   = current * dt_h * 1000.0f;   /* mAh consumed this interval */
    s_used_mAh   += delta;
    s_used_mAh    = clamp_f(s_used_mAh, 0.0f, BAT_CAPACITY_MAH);

    /* Coulomb-counted SOC */
    float soc_cc = 100.0f * (1.0f - (s_used_mAh / BAT_CAPACITY_MAH));

    /* ── 3. OCV blend (only when current is very low — near rest) ────────
     * Blend weight: 5 % when current < 200 mA, scales to 0 % at 1500 mA.
     * Thresholds are higher than single-cell because a 3S pack drives larger
     * loads. Adjust to match your application's idle vs load current.
     * ---------------------------------------------------------------------- */
    float ocv_soc   = ocv_to_soc(voltage);
    float i_mA      = current * 1000.0f;
    float blend     = 0.0f;
    if (i_mA < 1500.0f)
    {
        blend = 0.05f * (1.0f - (i_mA / 1500.0f));
    }
    s_soc_pct = clamp_f((1.0f - blend) * soc_cc + blend * ocv_soc, 0.0f, 100.0f);

    /* ── 4. Update average current (EMA) ────────────────────────────────── *
     * Only update EMA when current is meaningful (>1 mA) to avoid noise
     * from zero-load readings corrupting the runtime estimate.
     * ----------------------------------------------------------------------- */
    if (current > 0.001f)
    {
        if (s_avg_current_A < 0.001f)
        {
            s_avg_current_A = current;   /* Seed directly on first real reading */
        }
        else
        {
            s_avg_current_A = EMA_ALPHA * current + (1.0f - EMA_ALPHA) * s_avg_current_A;
        }
    }

    s_valid = true;
    return HAL_OK;
}

void Battery_GetData(BatteryData_t *out)
{
    if (out == NULL) return;

    out->soc_pct       = s_soc_pct;
    out->voltage_V     = s_voltage_V;
    out->current_A     = s_current_A;
    out->used_mAh      = s_used_mAh;
    out->remaining_mAh = clamp_f(BAT_CAPACITY_MAH - s_used_mAh, 0.0f, BAT_CAPACITY_MAH);
    out->status        = soc_to_status(s_soc_pct);
    out->valid         = s_valid;

    /* Runtime estimate — avoid divide-by-zero for tiny currents */
    if (s_avg_current_A > 0.001f)   /* 1 mA minimum — shows estimate as soon as load is seen */
    {
        float remaining_h  = (out->remaining_mAh / 1000.0f) / s_avg_current_A;
        out->runtime_sec   = (uint32_t)(remaining_h * 3600.0f);
    }
    else
    {
        out->runtime_sec = 0xFFFFFFFFU;   /* Sentinel: current too low to estimate */
    }
}

void Battery_RuntimeToHMS(uint32_t seconds, uint32_t *h, uint32_t *m, uint32_t *s)
{
    if (seconds == 0xFFFFFFFFU)
    {
        *h = 99U; *m = 59U; *s = 59U;   /* Display "--:--:--" in caller */
        return;
    }
    *h = seconds / 3600U;
    *m = (seconds % 3600U) / 60U;
    *s = seconds % 60U;
}
