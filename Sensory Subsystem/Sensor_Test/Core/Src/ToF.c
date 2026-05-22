#include "ToF.h"


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

// ================= LOW LEVEL =================

static HAL_StatusTypeDef ToF_WriteReg(I2C_HandleTypeDef *hi2c, uint16_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(hi2c, TOF_ADDR, reg, I2C_MEMADD_SIZE_16BIT, &value, 1, 100);
}

static HAL_StatusTypeDef ToF_ReadReg(I2C_HandleTypeDef *hi2c, uint16_t reg, uint8_t *value)
{
    return HAL_I2C_Mem_Read(hi2c, TOF_ADDR, reg, I2C_MEMADD_SIZE_16BIT, value, 1, 100);
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

    return HAL_OK;
}


// ================= INTERNAL VARIABLES =================
static float ToF_Multiplier = 1.0f;     // The 'm' in y = mx + b
static float ToF_Linear_Offset = 0.0f;  // The 'b' in y = mx + b
static float filtered_distance = 0.0f;  // For the smoothing filter

// Takes multiple readings and averages them to get a clean RAW number
uint16_t ToF_GetRawAverage(I2C_HandleTypeDef *hi2c, uint8_t samples)
{
    uint32_t total = 0;
    uint8_t valid = 0;

    for(int i = 0; i < samples; i++) {
        uint8_t ready = 0, dist_high = 0, dist_low = 0;

        if (ToF_ReadReg(hi2c, 0x0031, &ready) == HAL_OK && (ready & 0x01)) {
            ToF_ReadReg(hi2c, 0x0096, &dist_high);
            ToF_ReadReg(hi2c, 0x0097, &dist_low);
            ToF_WriteReg(hi2c, 0x0086, 0x01); // Clear interrupt

            uint16_t dist = (dist_high << 8) | dist_low;
            if(dist > 0 && dist < 4000) {
                total += dist;
                valid++;
            }
        }
        HAL_Delay(55);
    }

    if (valid == 0) return 0;
    return (uint16_t)(total / valid);
}

// Calculates the slope and offset based on two known points
void ToF_Calibrate_TwoPoint(uint16_t true_dist_1, uint16_t raw_meas_1,
                            uint16_t true_dist_2, uint16_t raw_meas_2)
{
    // Prevent divide-by-zero if sensor gets stuck
    if (raw_meas_2 == raw_meas_1) return;

    // Calculate Multiplier (m) = (Y2 - Y1) / (X2 - X1)
    ToF_Multiplier = (float)(true_dist_2 - true_dist_1) / (float)(raw_meas_2 - raw_meas_1);

    // Calculate Offset (b) = Y1 - (m * X1)
    ToF_Linear_Offset = (float)true_dist_1 - (ToF_Multiplier * (float)raw_meas_1);
}

uint16_t ToF_ReadDistance(I2C_HandleTypeDef *hi2c)
{
    uint8_t ready = 0, dist_high = 0, dist_low = 0;

    if (ToF_ReadReg(hi2c, 0x0031, &ready) != HAL_OK) return 0;
    if (!(ready & 0x01)) return 0;

    ToF_ReadReg(hi2c, 0x0096, &dist_high);
    ToF_ReadReg(hi2c, 0x0097, &dist_low);
    ToF_WriteReg(hi2c, 0x0086, 0x01);

    uint16_t raw_distance = (dist_high << 8) | dist_low;
    if (raw_distance == 0) return 0;

    // 1. APPLY TWO-POINT CALIBRATION (y = mx + b)
    float actual_distance = (raw_distance * ToF_Multiplier) + ToF_Linear_Offset;

    if (actual_distance < 0) actual_distance = 0;

    // 2. APPLY SMOOTHING FILTER
    if (filtered_distance == 0.0f) {
        filtered_distance = actual_distance;
    } else {
        filtered_distance = (0.3f * actual_distance) + (0.7f * filtered_distance);
    }

    return (uint16_t)filtered_distance;
}
// ================= PERMANENT STORAGE (FLASH) =================

/* * WARNING: You must pick an empty page at the very end of your STM32's memory!
 * 0x08007C00 is the last 1KB page for a 32KB flash chip (like STM32F030K6).
 * If you have a 64KB chip, use 0x0800FC00.
 * If you have a 16KB chip, use 0x08003C00.
 */
#define FLASH_CALIB_ADDR 0x08007C00 // comment this out and uncomment the bottom 2 for F4
//#define FLASH_CALIB_ADDR 0x08020000
//#define CALIB_SECTOR     FLASH_SECTOR_5
// Saves the current multiplier and offset permanently to the MCU
void ToF_SaveCalibration(void)
{
    HAL_FLASH_Unlock(); // Unlock the flash memory vault

    // 1. Erase the page before writing (Mandatory for Flash memory)
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError = 0;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = FLASH_CALIB_ADDR;
    EraseInitStruct.NbPages = 1;
    HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);

    // 2. We have to convert our floats into 32-bit integers to write them
    uint32_t m_data = *(uint32_t*)&ToF_Multiplier;
    uint32_t b_data = *(uint32_t*)&ToF_Linear_Offset;

    // 3. Burn them into memory
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_CALIB_ADDR, m_data);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_CALIB_ADDR + 4, b_data);

    HAL_FLASH_Lock(); // Lock the vault
}

// Attempts to load saved calibration. Returns 1 if successful, 0 if it's a brand new board
uint8_t ToF_LoadCalibration(void)
{
    // Read the raw 32-bit data from the flash addresses
    uint32_t m_data = *(__IO uint32_t*)FLASH_CALIB_ADDR;
    uint32_t b_data = *(__IO uint32_t*)(FLASH_CALIB_ADDR + 4);

    // Blank flash memory is always 0xFFFFFFFF. If we see that, it means it's never been calibrated.
    if (m_data == 0xFFFFFFFF || b_data == 0xFFFFFFFF) {
        return 0; // Calibration not found!
    }

    // Convert the data back into our floats
    ToF_Multiplier = *(float*)&m_data;
    ToF_Linear_Offset = *(float*)&b_data;

    return 1; // Calibration loaded successfully!
}

// Completely wipes the calibration page in Flash Memory
void ToF_EraseCalibration(void)
{
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError = 0;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = FLASH_CALIB_ADDR; // The address we defined earlier
    EraseInitStruct.NbPages = 1;

    HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);

    HAL_FLASH_Lock();
}
