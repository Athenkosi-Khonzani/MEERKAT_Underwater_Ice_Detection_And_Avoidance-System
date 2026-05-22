#include "VL53L0X.h"
#include <string.h>

#define I2C_TIMEOUT 100 // Safe 100ms timeout to prevent hanging

//---------------------------------------------------------
// Local variables within this file
//---------------------------------------------------------
uint8_t g_i2cAddr = ADDRESS_DEFAULT;
uint16_t g_ioTimeout = 0;
uint8_t g_isTimeout = 0;
uint32_t g_timeoutStartMs;
uint8_t g_stopVariable;
uint32_t g_measTimBudUs;

I2C_HandleTypeDef VL53L0X_I2C_Handler;
uint8_t msgBuffer[4];

//---------------------------------------------------------
// Utility Macros & Structs
//---------------------------------------------------------
#define startTimeout() (g_timeoutStartMs = HAL_GetTick())
#define checkTimeoutExpired() (g_ioTimeout > 0 && ((uint32_t)HAL_GetTick() - g_timeoutStartMs) > g_ioTimeout)
#define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

typedef struct {
  uint8_t tcc, msrc, dss, pre_range, final_range;
} SequenceStepEnables;

typedef struct {
  uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;
  uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
  uint32_t msrc_dss_tcc_us, pre_range_us, final_range_us;
} SequenceStepTimeouts;

// Private Function Prototypes
bool getSpadInfo(uint8_t *count, bool *type_is_aperture);
void getSequenceStepEnables(SequenceStepEnables * enables);
void getSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts);
bool performSingleRefCalibration(uint8_t vhv_init_byte);
static uint16_t decodeTimeout(uint16_t value);
static uint16_t encodeTimeout(uint16_t timeout_mclks);
static uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);
uint8_t getVcselPulsePeriod(vcselPeriodType type);
uint32_t getMeasurementTimingBudget(void);

//---------------------------------------------------------
// FIXED: I2C communication Functions
//---------------------------------------------------------
void writeReg(uint8_t reg, uint8_t value) {
  msgBuffer[0] = value;
  // Removed manual LSB manipulation. Use the strict HAL size definitions.
  HAL_I2C_Mem_Write(&VL53L0X_I2C_Handler, g_i2cAddr, reg, I2C_MEMADD_SIZE_8BIT, msgBuffer, 1, I2C_TIMEOUT);
}

void writeReg16Bit(uint8_t reg, uint16_t value){
  msgBuffer[0] = (value >> 8) & 0xFF;
  msgBuffer[1] = value & 0xFF;
  HAL_I2C_Mem_Write(&VL53L0X_I2C_Handler, g_i2cAddr, reg, I2C_MEMADD_SIZE_8BIT, msgBuffer, 2, I2C_TIMEOUT);
}

void writeReg32Bit(uint8_t reg, uint32_t value){
  msgBuffer[0] = (value >> 24) & 0xFF;
  msgBuffer[1] = (value >> 16) & 0xFF;
  msgBuffer[2] = (value >> 8) & 0xFF;
  msgBuffer[3] = value & 0xFF;
  HAL_I2C_Mem_Write(&VL53L0X_I2C_Handler, g_i2cAddr, reg, I2C_MEMADD_SIZE_8BIT, msgBuffer, 4, I2C_TIMEOUT);
}

uint8_t readReg(uint8_t reg) {
  msgBuffer[0] = 0; // Clear buffer
  HAL_I2C_Mem_Read(&VL53L0X_I2C_Handler, g_i2cAddr, reg, I2C_MEMADD_SIZE_8BIT, msgBuffer, 1, I2C_TIMEOUT);
  return msgBuffer[0];
}

uint16_t readReg16Bit(uint8_t reg) {
  HAL_I2C_Mem_Read(&VL53L0X_I2C_Handler, g_i2cAddr, reg, I2C_MEMADD_SIZE_8BIT, msgBuffer, 2, I2C_TIMEOUT);
  return (uint16_t)((msgBuffer[0] << 8) | msgBuffer[1]);
}

void writeMulti(uint8_t reg, uint8_t const *src, uint8_t count){
  HAL_I2C_Mem_Write(&VL53L0X_I2C_Handler, g_i2cAddr, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)src, count, I2C_TIMEOUT);
}

void readMulti(uint8_t reg, uint8_t * dst, uint8_t count) {
  HAL_I2C_Mem_Read(&VL53L0X_I2C_Handler, g_i2cAddr, reg, I2C_MEMADD_SIZE_8BIT, dst, count, I2C_TIMEOUT);
}

//---------------------------------------------------------
// Initialization & Control Functions
//---------------------------------------------------------
bool initVL53L0X(bool io_2v8, I2C_HandleTypeDef *handler){
  memcpy(&VL53L0X_I2C_Handler, handler, sizeof(*handler));

  // HARDWARE CHECK: Validate that we are actually talking to a VL53L0X
  // This prevents the MCU from freezing in infinite loops if the wiring is bad.
  uint8_t id = readReg(VL53L0X_IDENTIFICATION_MODEL_ID);
  if (id != 0xEE) {
      return false; // Hardware not found or bad I2C connection!
  }

  if (io_2v8) {
    writeReg(VL53L0X_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, readReg(VL53L0X_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01);
  }

  writeReg(VL53L0X_I2C_MODE, 0x00);
  writeReg(0x80, 0x01); writeReg(0xFF, 0x01); writeReg(0x00, 0x00);
  g_stopVariable = readReg(0x91);
  writeReg(0x00, 0x01); writeReg(0xFF, 0x00); writeReg(0x80, 0x00);

  writeReg(VL53L0X_MSRC_CONFIG_CONTROL, readReg(VL53L0X_MSRC_CONFIG_CONTROL) | 0x12);
  setSignalRateLimit(0.2);
  writeReg(VL53L0X_SYSTEM_SEQUENCE_CONFIG, 0xFF);

  uint8_t spad_count;
  bool spad_type_is_aperture;
  if (!getSpadInfo(&spad_count, &spad_type_is_aperture)) { return false; }

  uint8_t ref_spad_map[6];
  readMulti(VL53L0X_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

  writeReg(0xFF, 0x01); writeReg(0x4F, 0x00); writeReg(0x4E, 0x2C); writeReg(0xFF, 0x00);
  writeReg(VL53L0X_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

  uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0;
  uint8_t spads_enabled = 0;

  for (uint8_t i = 0; i < 48; i++) {
    if (i < first_spad_to_enable || spads_enabled == spad_count) {
      ref_spad_map[i / 8] &= ~(1 << (i % 8));
    } else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1) {
      spads_enabled++;
    }
  }

  writeMulti(VL53L0X_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

  // Load tuning settings
  writeReg(0xFF, 0x01); writeReg(0x00, 0x00); writeReg(0xFF, 0x00); writeReg(0x09, 0x00);
  writeReg(0x10, 0x00); writeReg(0x11, 0x00); writeReg(0x24, 0x01); writeReg(0x25, 0xFF);
  writeReg(0x75, 0x00); writeReg(0xFF, 0x01); writeReg(0x4E, 0x2C); writeReg(0x48, 0x00);
  writeReg(0x30, 0x20); writeReg(0xFF, 0x00); writeReg(0x30, 0x09); writeReg(0x54, 0x00);
  writeReg(0x31, 0x04); writeReg(0x32, 0x03); writeReg(0x40, 0x83); writeReg(0x46, 0x25);
  writeReg(0x60, 0x00); writeReg(0x27, 0x00); writeReg(0x50, 0x06); writeReg(0x51, 0x00);
  writeReg(0x52, 0x96); writeReg(0x56, 0x08); writeReg(0x57, 0x30); writeReg(0x61, 0x00);
  writeReg(0x62, 0x00); writeReg(0x64, 0x00); writeReg(0x65, 0x00); writeReg(0x66, 0xA0);
  writeReg(0xFF, 0x01); writeReg(0x22, 0x32); writeReg(0x47, 0x14); writeReg(0x49, 0xFF);
  writeReg(0x4A, 0x00); writeReg(0xFF, 0x00); writeReg(0x7A, 0x0A); writeReg(0x7B, 0x00);
  writeReg(0x78, 0x21); writeReg(0xFF, 0x01); writeReg(0x23, 0x34); writeReg(0x42, 0x00);
  writeReg(0x44, 0xFF); writeReg(0x45, 0x26); writeReg(0x46, 0x05); writeReg(0x40, 0x40);
  writeReg(0x0E, 0x06); writeReg(0x20, 0x1A); writeReg(0x43, 0x40); writeReg(0xFF, 0x00);
  writeReg(0x34, 0x03); writeReg(0x35, 0x44); writeReg(0xFF, 0x01); writeReg(0x31, 0x04);
  writeReg(0x4B, 0x09); writeReg(0x4C, 0x05); writeReg(0x4D, 0x04); writeReg(0xFF, 0x00);
  writeReg(0x44, 0x00); writeReg(0x45, 0x20); writeReg(0x47, 0x08); writeReg(0x48, 0x28);
  writeReg(0x67, 0x00); writeReg(0x70, 0x04); writeReg(0x71, 0x01); writeReg(0x72, 0xFE);
  writeReg(0x76, 0x00); writeReg(0x77, 0x00); writeReg(0xFF, 0x01); writeReg(0x0D, 0x01);
  writeReg(0xFF, 0x00); writeReg(0x80, 0x01); writeReg(0x01, 0xF8); writeReg(0xFF, 0x01);
  writeReg(0x8E, 0x01); writeReg(0x00, 0x01); writeReg(0xFF, 0x00); writeReg(0x80, 0x00);

  writeReg(VL53L0X_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
  writeReg(VL53L0X_GPIO_HV_MUX_ACTIVE_HIGH, readReg(VL53L0X_GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10);
  writeReg(VL53L0X_SYSTEM_INTERRUPT_CLEAR, 0x01);

  g_measTimBudUs = getMeasurementTimingBudget();
  writeReg(VL53L0X_SYSTEM_SEQUENCE_CONFIG, 0xE8);
  setMeasurementTimingBudget(g_measTimBudUs);

  writeReg(VL53L0X_SYSTEM_SEQUENCE_CONFIG, 0x01);
  if (!performSingleRefCalibration(0x40)) { return false; }

  writeReg(VL53L0X_SYSTEM_SEQUENCE_CONFIG, 0x02);
  if (!performSingleRefCalibration(0x00)) { return false; }
  writeReg(VL53L0X_SYSTEM_SEQUENCE_CONFIG, 0xE8);

  return true;
}

bool ToF_InitializeSingle(I2C_HandleTypeDef *hi2c, uint16_t signalRate) {
  setTimeout(500);

  if (!initVL53L0X(false, hi2c)) { return false; }

  uint8_t PreRange = 18;
  uint8_t FinalRange = 14;

  setSignalRateLimit(signalRate);
  setVcselPulsePeriod(VcselPeriodPreRange, PreRange);
  setVcselPulsePeriod(VcselPeriodFinalRange, FinalRange);
  setMeasurementTimingBudget(50000);
  startContinuous(0);

  return true;
}

void calibrateToF(VL53L0_t* TOF_result , uint16_t distance) {
    uint8_t PreRange = 12;
    uint8_t FinalRange = 8;
    uint32_t timingBudget = 33000;

    if (distance > 500 ) {
        PreRange = 18;
        FinalRange = 14;
        timingBudget = 66000;
    }

    if (TOF_result->timingBudget != timingBudget) {
      TOF_result->timingBudget = timingBudget;
      stopContinuous();
      setMeasurementTimingBudget(timingBudget);
      setVcselPulsePeriod(VcselPeriodPreRange, PreRange);
      setVcselPulsePeriod(VcselPeriodFinalRange, FinalRange);
      startContinuous(0);
    }
}

void ToF_ReadSingle(VL53L0_t* TOF_result) {
  VL53L0_t distanceStr;
  uint16_t distance = readRangeContinuousMillimeters(&distanceStr);

  if (distanceStr.rangeStatus == 11 || distanceStr.rangeStatus == 0) { // RANGECOMPLETE
    if (distance < 8000){
      TOF_result->Distance = distance;
      TOF_result->Status = distanceStr.rangeStatus;
      TOF_result->Ambient = distanceStr.Ambient;
      TOF_result->Signal = distanceStr.Signal;
    }
  }

  #ifdef TOF_DYNAMIC_FSR
    calibrateToF(TOF_result , distance);
  #endif
}

bool setSignalRateLimit(uint16_t limit_kcps) {
    if (limit_kcps > 51199) { return false; }
    uint16_t limit_q9_7 = (limit_kcps << 7) / 1000;
    writeReg16Bit(VL53L0X_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit_q9_7);
    return true;
}

uint8_t setMeasurementTimingBudget(uint32_t budget_us) {
  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead      = 1320;
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;
  uint32_t const MinTimingBudget    = 20000;

  if (budget_us < MinTimingBudget) { return false; }
  uint32_t used_budget_us = StartOverhead + EndOverhead;

  getSequenceStepEnables(&enables);
  getSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc) { used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead); }
  if (enables.dss) { used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead); }
  else if (enables.msrc) { used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead); }
  if (enables.pre_range) { used_budget_us += (timeouts.pre_range_us + PreRangeOverhead); }

  if (enables.final_range) {
    used_budget_us += FinalRangeOverhead;
    if (used_budget_us > budget_us) { return false; }
    uint32_t final_range_timeout_us = budget_us - used_budget_us;
    uint16_t final_range_timeout_mclks = timeoutMicrosecondsToMclks(final_range_timeout_us, timeouts.final_range_vcsel_period_pclks);

    if (enables.pre_range) { final_range_timeout_mclks += timeouts.pre_range_mclks; }
    writeReg16Bit(VL53L0X_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(final_range_timeout_mclks));
    g_measTimBudUs = budget_us;
  }
  return true;
}

uint32_t getMeasurementTimingBudget(void) {
  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead     = 1910;
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  uint32_t budget_us = StartOverhead + EndOverhead;

  getSequenceStepEnables(&enables);
  getSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc) { budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead); }
  if (enables.dss) { budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead); }
  else if (enables.msrc) { budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead); }
  if (enables.pre_range) { budget_us += (timeouts.pre_range_us + PreRangeOverhead); }
  if (enables.final_range) { budget_us += (timeouts.final_range_us + FinalRangeOverhead); }

  g_measTimBudUs = budget_us;
  return budget_us;
}

uint8_t setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks) {
  uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);
  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  getSequenceStepEnables(&enables);
  getSequenceStepTimeouts(&enables, &timeouts);

  if (type == VcselPeriodPreRange) {
    switch (period_pclks) {
      case 12: writeReg(VL53L0X_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18); break;
      case 14: writeReg(VL53L0X_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30); break;
      case 16: writeReg(VL53L0X_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40); break;
      case 18: writeReg(VL53L0X_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50); break;
      default: return false;
    }
    writeReg(VL53L0X_PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
    writeReg(VL53L0X_PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    uint16_t new_pre_range_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);
    writeReg16Bit(VL53L0X_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(new_pre_range_timeout_mclks));
    uint16_t new_msrc_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);
    writeReg(VL53L0X_MSRC_CONFIG_TIMEOUT_MACROP, (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));
  } else if (type == VcselPeriodFinalRange) {
    switch (period_pclks) {
      case 8:
        writeReg(VL53L0X_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10); writeReg(VL53L0X_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(VL53L0X_GLOBAL_CONFIG_VCSEL_WIDTH, 0x02); writeReg(VL53L0X_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
        writeReg(0xFF, 0x01); writeReg(VL53L0X_ALGO_PHASECAL_LIM, 0x30); writeReg(0xFF, 0x00); break;
      case 10:
        writeReg(VL53L0X_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28); writeReg(VL53L0X_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(VL53L0X_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03); writeReg(VL53L0X_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
        writeReg(0xFF, 0x01); writeReg(VL53L0X_ALGO_PHASECAL_LIM, 0x20); writeReg(0xFF, 0x00); break;
      case 12:
        writeReg(VL53L0X_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38); writeReg(VL53L0X_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(VL53L0X_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03); writeReg(VL53L0X_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
        writeReg(0xFF, 0x01); writeReg(VL53L0X_ALGO_PHASECAL_LIM, 0x20); writeReg(0xFF, 0x00); break;
      case 14:
        writeReg(VL53L0X_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48); writeReg(VL53L0X_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(VL53L0X_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03); writeReg(VL53L0X_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
        writeReg(0xFF, 0x01); writeReg(VL53L0X_ALGO_PHASECAL_LIM, 0x20); writeReg(0xFF, 0x00); break;
      default: return false;
    }
    writeReg(VL53L0X_FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);
    uint16_t new_final_range_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);
    if (enables.pre_range) { new_final_range_timeout_mclks += timeouts.pre_range_mclks; }
    writeReg16Bit(VL53L0X_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(new_final_range_timeout_mclks));
  } else { return false; }

  setMeasurementTimingBudget(g_measTimBudUs);
  uint8_t sequence_config = readReg(VL53L0X_SYSTEM_SEQUENCE_CONFIG);
  writeReg(VL53L0X_SYSTEM_SEQUENCE_CONFIG, 0x02);
  performSingleRefCalibration(0x0);
  writeReg(VL53L0X_SYSTEM_SEQUENCE_CONFIG, sequence_config);

  return true;
}

uint8_t getVcselPulsePeriod(vcselPeriodType type) {
  if (type == VcselPeriodPreRange) { return decodeVcselPeriod(readReg(VL53L0X_PRE_RANGE_CONFIG_VCSEL_PERIOD)); }
  else if (type == VcselPeriodFinalRange) { return decodeVcselPeriod(readReg(VL53L0X_FINAL_RANGE_CONFIG_VCSEL_PERIOD)); }
  else { return 255; }
}

void startContinuous(uint32_t period_ms) {
  writeReg(0x80, 0x01); writeReg(0xFF, 0x01); writeReg(0x00, 0x00);
  writeReg(0x91, g_stopVariable); writeReg(0x00, 0x01); writeReg(0xFF, 0x00);
  writeReg(0x80, 0x00);
  writeReg(VL53L0X_SYSRANGE_START, 0x02);
}

void stopContinuous(void) {
  writeReg(VL53L0X_SYSRANGE_START, 0x01);
  writeReg(0xFF, 0x01); writeReg(0x00, 0x00); writeReg(0x91, 0x00);
  writeReg(0x00, 0x01); writeReg(0xFF, 0x00);
}

uint16_t readRangeContinuousMillimeters( VL53L0_t *extraStats) {
  uint8_t tempBuffer[12];
  uint16_t temp = 0;
  startTimeout();

  while ((readReg(VL53L0X_RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
    if (checkTimeoutExpired()) return 65535;
  }

  if (extraStats != NULL) {
    readMulti(0x14, tempBuffer, 12);
    extraStats->rangeStatus = tempBuffer[0x00]>>3;
    extraStats->spadCnt     = (tempBuffer[0x02]<<8) | tempBuffer[0x03];
    extraStats->Signal      = (tempBuffer[0x06]<<8) | tempBuffer[0x07];
    extraStats->Ambient     = (tempBuffer[0x08]<<8) | tempBuffer[0x09];
    temp                    = (tempBuffer[0x0A]<<8) | tempBuffer[0x0B];
    extraStats->rawDistance = temp;
  }

  writeReg(VL53L0X_SYSTEM_INTERRUPT_CLEAR, 0x01);
  return temp;
}

void setTimeout(uint16_t timeout){ g_ioTimeout = timeout; }

// --- Internal Utilities ---
bool getSpadInfo(uint8_t * count, bool * type_is_aperture) {
  uint8_t tmp;
  writeReg(0x80, 0x01); writeReg(0xFF, 0x01); writeReg(0x00, 0x00); writeReg(0xFF, 0x06);
  writeReg(0x83, readReg(0x83) | 0x04); writeReg(0xFF, 0x07); writeReg(0x81, 0x01);
  writeReg(0x80, 0x01); writeReg(0x94, 0x6b); writeReg(0x83, 0x00);
  startTimeout();
  while (readReg(0x83) == 0x00) { if (checkTimeoutExpired()) { return false; } }
  writeReg(0x83, 0x01); tmp = readReg(0x92);
  *count = tmp & 0x7f; *type_is_aperture = (tmp >> 7) & 0x01;
  writeReg(0x81, 0x00); writeReg(0xFF, 0x06); writeReg(0x83, readReg(0x83)  & ~0x04);
  writeReg(0xFF, 0x01); writeReg(0x00, 0x01); writeReg(0xFF, 0x00); writeReg(0x80, 0x00);
  return true;
}

void getSequenceStepEnables(SequenceStepEnables * enables) {
  uint8_t sequence_config = readReg(VL53L0X_SYSTEM_SEQUENCE_CONFIG);
  enables->tcc          = (sequence_config >> 4) & 0x1;
  enables->dss          = (sequence_config >> 3) & 0x1;
  enables->msrc         = (sequence_config >> 2) & 0x1;
  enables->pre_range    = (sequence_config >> 6) & 0x1;
  enables->final_range  = (sequence_config >> 7) & 0x1;
}

void getSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts) {
  timeouts->pre_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodPreRange);
  timeouts->msrc_dss_tcc_mclks = readReg(VL53L0X_MSRC_CONFIG_TIMEOUT_MACROP) + 1;
  timeouts->msrc_dss_tcc_us = timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks, timeouts->pre_range_vcsel_period_pclks);
  timeouts->pre_range_mclks = decodeTimeout(readReg16Bit(VL53L0X_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
  timeouts->pre_range_us = timeoutMclksToMicroseconds(timeouts->pre_range_mclks, timeouts->pre_range_vcsel_period_pclks);
  timeouts->final_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodFinalRange);
  timeouts->final_range_mclks = decodeTimeout(readReg16Bit(VL53L0X_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));
  if (enables->pre_range) { timeouts->final_range_mclks -= timeouts->pre_range_mclks; }
  timeouts->final_range_us = timeoutMclksToMicroseconds(timeouts->final_range_mclks, timeouts->final_range_vcsel_period_pclks);
}

static uint16_t decodeTimeout(uint16_t reg_val) {
  return (uint16_t)((reg_val & 0x00FF) << (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

static uint16_t encodeTimeout(uint16_t timeout_mclks) {
  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;
  if (timeout_mclks > 0) {
    ls_byte = timeout_mclks - 1;
    while ((ls_byte & 0xFFFFFF00) > 0) {
      ls_byte >>= 1;
      ms_byte++;
    }
    return (ms_byte << 8) | (ls_byte & 0xFF);
  } else { return 0; }
}

static uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks) {
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
  return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks) {
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

bool performSingleRefCalibration(uint8_t vhv_init_byte) {
  writeReg(VL53L0X_SYSRANGE_START, 0x01 | vhv_init_byte);
  startTimeout();
  while ((readReg(VL53L0X_RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
    if (checkTimeoutExpired()) { return false; }
  }
  writeReg(VL53L0X_SYSTEM_INTERRUPT_CLEAR, 0x01);
  writeReg(VL53L0X_SYSRANGE_START, 0x00);
  return true;
}
