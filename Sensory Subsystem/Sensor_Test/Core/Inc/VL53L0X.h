#ifndef VL53L0X_REGISTER_MAP_H
#define VL53L0X_REGISTER_MAP_H

#include "stm32f0xx_hal.h"

// System control registers
#define VL53L0X_SYSRANGE_START                          0x00
#define VL53L0X_SYSTEM_SEQUENCE_CONFIG                  0x01
#define VL53L0X_SYSTEM_INTERMEASUREMENT_PERIOD          0x04
#define VL53L0X_SYSTEM_INTERRUPT_CONFIG_GPIO            0x0A
#define VL53L0X_SYSTEM_INTERRUPT_CLEAR                  0x0B
#define VL53L0X_RESULT_INTERRUPT_STATUS                 0x13
#define VL53L0X_RESULT_RANGE_STATUS                     0x14

// I2C slave device address
#define VL53L0X_I2C_SLAVE_DEVICE_ADDRESS                0x8A
#define VL53L0X_I2C_MODE                                0x88

// Configuration registers
#define VL53L0X_MSRC_CONFIG_CONTROL                     0x60
#define VL53L0X_MSRC_CONFIG_TIMEOUT_MACROP              0x46
#define VL53L0X_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT 0x44
#define VL53L0X_FINAL_RANGE_CONFIG_VALID_PHASE_LOW      0x47
#define VL53L0X_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH     0x48
#define VL53L0X_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI    0x71

// Pre-range registers
#define VL53L0X_PRE_RANGE_CONFIG_VCSEL_PERIOD           0x50
#define VL53L0X_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI      0x51
#define VL53L0X_PRE_RANGE_CONFIG_VALID_PHASE_HIGH       0x56
#define VL53L0X_PRE_RANGE_CONFIG_VALID_PHASE_LOW        0x57

// Final range registers
#define VL53L0X_FINAL_RANGE_CONFIG_VCSEL_PERIOD         0x70

// Global configuration registers
#define VL53L0X_GLOBAL_CONFIG_SPAD_ENABLES_REF_0        0xB0
#define VL53L0X_GLOBAL_CONFIG_REF_EN_START_SELECT       0xB6
#define VL53L0X_GLOBAL_CONFIG_VCSEL_WIDTH               0x32
#define VL53L0X_GPIO_HV_MUX_ACTIVE_HIGH                 0x84
#define VL53L0X_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV       0x89
#define VL53L0X_ALGO_PHASECAL_LIM                       0x30
#define VL53L0X_ALGO_PHASECAL_CONFIG_TIMEOUT            0x30
#define VL53L0X_IDENTIFICATION_MODEL_ID                 0xC0

#endif // VL53L0X_REGISTER_MAP_H

#ifndef VL53L0X_h
#define VL53L0X_h

#include <stdbool.h>

#define TOF_DYNAMIC_FSR
#define ADDRESS_DEFAULT 0x52 // 8-bit default address

typedef enum { VcselPeriodPreRange, VcselPeriodFinalRange } vcselPeriodType;

typedef struct {
  uint32_t Distance;
  uint32_t Status;
  uint16_t Ambient;
  uint16_t Signal;
  uint16_t rawDistance;
  uint16_t spadCnt;
  uint8_t rangeStatus;
  uint32_t timingBudget;
} VL53L0_t;

// Changed to return bool
bool ToF_InitializeSingle(I2C_HandleTypeDef *hi2c, uint16_t signalRate);
void ToF_ReadSingle(VL53L0_t* TOF_result);

bool initVL53L0X(bool io_2v8, I2C_HandleTypeDef *handler);
bool setSignalRateLimit(uint16_t limit_kcps);
uint8_t setMeasurementTimingBudget(uint32_t budget_us);
uint8_t setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks);
void startContinuous(uint32_t period_ms);
void stopContinuous(void);
uint16_t readRangeContinuousMillimeters(VL53L0_t *extraStats);
void setTimeout(uint16_t timeout);

#endif
