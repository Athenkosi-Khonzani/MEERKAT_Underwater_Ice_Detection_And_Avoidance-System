#include "ToF.h"
#include "FlashStore.h"
#include <math.h>
#include <stdio.h>

// ================= VL53L1X TUNING SETTINGS =================
// The VL53L1X requires these 91 bytes to be written to its memory
// before it will allow the laser to start ranging.
const uint8_t VL53L1X_DEFAULT_CONFIGURATION[] = {
    0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x02, 0x08,
    0x00, 0x08, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00,
    0x00, 0xFF, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x20, 0x0B, 0x00, 0x00, 0x02, 0x0A, 0x21,
    0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xC8,
    0x00, 0x00, 0x38, 0xFF, 0x01, 0x00, 0x08, 0x00,
    0x00, 0x01, 0xCC, 0x0F, 0x01, 0xF1, 0x0D, 0x01,
    0x68, 0x00, 0x80, 0x08, 0xB8, 0x00, 0x00, 0x00,
    0x00, 0x0F, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x0F, 0x0D, 0x0E, 0x0E, 0x00,
    0x00, 0x02, 0xC7, 0xFF, 0x9B, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00
};

// ================= CALIBRATION STATE =================
// Held privately in this translation unit. Loaded from flash on init.
static struct {
    float    slope;
    float    offset_mm;
    uint8_t  valid;
    uint32_t cal_count;
} g_cal = { 1.0f, 0.0f, 0, 0 };

// ================= LOW LEVEL =================

static HAL_StatusTypeDef ToF_WriteReg(I2C_HandleTypeDef *hi2c, uint16_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(hi2c, TOF_ADDR, reg, I2C_MEMADD_SIZE_16BIT, &value, 1, 100);
}

static HAL_StatusTypeDef ToF_ReadReg(I2C_HandleTypeDef *hi2c, uint16_t reg, uint8_t *value)
{
    return HAL_I2C_Mem_Read(hi2c, TOF_ADDR, reg, I2C_MEMADD_SIZE_16BIT, value, 1, 100);
}

// ================= CALIBRATION HELPERS =================

static uint16_t ToF_ApplyCalibration(uint16_t raw_mm)
{
    if (raw_mm == 0) return 0;   /* preserve "no reading" sentinel */

    float corrected = g_cal.slope * (float)raw_mm + g_cal.offset_mm;
    if (corrected < 0.0f)      corrected = 0.0f;
    if (corrected > 65535.0f)  corrected = 65535.0f;
    return (uint16_t)(corrected + 0.5f);
}

static void ToF_LoadCalibrationFromFlash(void)
{
    CalRecord rec;
    if (FlashStore_Load(&rec))
    {
        g_cal.slope     = rec.slope;
        g_cal.offset_mm = rec.offset_mm;
        g_cal.cal_count = rec.cal_count;
        g_cal.valid     = 1;
    }
    else
    {
        g_cal.slope     = 1.0f;
        g_cal.offset_mm = 0.0f;
        g_cal.cal_count = 0;
        g_cal.valid     = 0;
    }
}

// ================= INIT =================

HAL_StatusTypeDef ToF_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t data = 0;
    uint32_t timeout;
    char buf[16];

    // 1. Hardware ID Check
    if (ToF_ReadReg(hi2c, 0x010F, &data) != HAL_OK) return HAL_ERROR;

    if (data != 0xEA) {
        sprintf(buf, "Bad ID: %02X", data);

        return HAL_ERROR;
    }

    // 2. Software Reset
    ToF_WriteReg(hi2c, 0x0000, 0x00);
    HAL_Delay(10);
    ToF_WriteReg(hi2c, 0x0000, 0x01);
    HAL_Delay(10); // Wait for the MCU inside the sensor to wake up

    // 3. Wait for Boot (VL53L1X Firmware Status register is 0x00E5)
    timeout = HAL_GetTick();
    do {
        if (ToF_ReadReg(hi2c, 0x00E5, &data) != HAL_OK) return HAL_ERROR;

        if ((HAL_GetTick() - timeout) > 500) {

            return HAL_TIMEOUT;
        }
    } while ((data & 0x01) == 0); // Loop until Bit 0 goes High

    // 4. Load VL53L1X Tuning Settings (Starts at register 0x002D)
    for (uint16_t i = 0; i < sizeof(VL53L1X_DEFAULT_CONFIGURATION); i++) {
        ToF_WriteReg(hi2c, 0x002D + i, VL53L1X_DEFAULT_CONFIGURATION[i]);
    }

    // 5. Start Autonomous Ranging
    if (ToF_WriteReg(hi2c, 0x0087, 0x40) != HAL_OK) return HAL_ERROR;

    // 6. Load stored calibration (if any) from flash
    ToF_LoadCalibrationFromFlash();

    return HAL_OK;
}

// ================= DISTANCE =================

uint16_t ToF_ReadDistanceRaw(I2C_HandleTypeDef *hi2c)
{
    uint8_t ready = 0;
    uint8_t dist_high = 0;
    uint8_t dist_low = 0;

    // 1. Check if new data is ready (Register 0x0031 bit 0 on VL53L1X)
    if (ToF_ReadReg(hi2c, 0x0031, &ready) != HAL_OK) return 0;
    if (!(ready & 0x01)) return 0; // If bit 0 is low, no new reading yet

    // 2. Read the 16-bit distance in millimeters
    if (ToF_ReadReg(hi2c, 0x0096, &dist_high) != HAL_OK) return 0;
    if (ToF_ReadReg(hi2c, 0x0097, &dist_low) != HAL_OK) return 0;

    // 3. Clear the interrupt (MANDATORY: Tells sensor to take the next reading)
    ToF_WriteReg(hi2c, 0x0086, 0x01);

    return (uint16_t)((dist_high << 8) | dist_low);
}

uint16_t ToF_ReadDistance(I2C_HandleTypeDef *hi2c)
{
    return ToF_ApplyCalibration(ToF_ReadDistanceRaw(hi2c));
}

uint16_t ToF_AverageRaw(I2C_HandleTypeDef *hi2c, uint16_t samples)
{
    if (samples == 0) return 0;

    uint32_t sum = 0;
    uint16_t collected = 0;
    uint32_t attempts  = 0;
    const uint32_t max_attempts = (uint32_t)samples * 10U; /* safety cap */

    while (collected < samples && attempts < max_attempts)
    {
        uint16_t raw = ToF_ReadDistanceRaw(hi2c);
        if (raw > 0)
        {
            sum += raw;
            collected++;
        }
        attempts++;
        HAL_Delay(60);   /* ≥ ranging timing budget */
    }

    if (collected == 0) return 0;
    return (uint16_t)(sum / collected);
}

// ================= CALIBRATION API =================

uint8_t ToF_Calibrate(uint16_t measured_near_mm,
                      uint16_t measured_far_mm,
                      uint16_t actual_near_mm,
                      uint16_t actual_far_mm)
{
    if (measured_far_mm == measured_near_mm) return 0;   /* divide-by-zero */
    if (actual_far_mm   == actual_near_mm)   return 0;

    float mn = (float)measured_near_mm;
    float mf = (float)measured_far_mm;
    float an = (float)actual_near_mm;
    float af = (float)actual_far_mm;

    float slope  = (af - an) / (mf - mn);
    float offset = an - slope * mn;

    /* Reject obviously broken calibrations so flash isn't poisoned. */
    if (slope < 0.5f || slope > 2.0f) return 0;
    if (fabsf(offset) > 500.0f)        return 0;

    CalRecord rec = {
        .magic     = FLASH_STORE_MAGIC,
        .slope     = slope,
        .offset_mm = offset,
        .cal_count = g_cal.cal_count + 1U,
        .crc       = 0      /* filled by FlashStore_Save */
    };

    if (!FlashStore_Save(&rec)) return 0;

    g_cal.slope     = slope;
    g_cal.offset_mm = offset;
    g_cal.cal_count = rec.cal_count;
    g_cal.valid     = 1;
    return 1;
}

uint8_t ToF_ClearCalibration(void)
{
    CalRecord rec = {
        .magic     = FLASH_STORE_MAGIC,
        .slope     = 1.0f,
        .offset_mm = 0.0f,
        .cal_count = g_cal.cal_count + 1U,
        .crc       = 0
    };

    if (!FlashStore_Save(&rec)) return 0;

    g_cal.slope     = 1.0f;
    g_cal.offset_mm = 0.0f;
    g_cal.cal_count = rec.cal_count;
    g_cal.valid     = 0;
    return 1;
}

void ToF_GetCalibration(float *slope, float *offset_mm, uint8_t *valid)
{
    if (slope)     *slope     = g_cal.slope;
    if (offset_mm) *offset_mm = g_cal.offset_mm;
    if (valid)     *valid     = g_cal.valid;
}
