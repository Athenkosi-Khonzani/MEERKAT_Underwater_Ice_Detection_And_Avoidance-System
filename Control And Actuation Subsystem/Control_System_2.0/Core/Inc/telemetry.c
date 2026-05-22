/**
 ******************************************************************************
 * @file    telemetry.c
 * @brief   UAV telemetry JSON formatter + UART transmitter
 ******************************************************************************
 */

#include "telemetry.h"
#include <stdio.h>
#include <string.h>

/* ----- Module state ------------------------------------------------------- */

#define TELEM_BUF_SIZE   256
#define TELEM_TX_TIMEOUT 100   /* ms */

static UART_HandleTypeDef *s_huart = NULL;
static uint32_t s_seq = 0;
static char     s_buf[TELEM_BUF_SIZE];

/* ----- Public API --------------------------------------------------------- */

void Telemetry_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    s_seq   = 0;
}

uint32_t Telemetry_GetSeq(void)
{
    return s_seq;
}

HAL_StatusTypeDef Telemetry_Send(const BatteryData_t *battery,
                                  uint16_t tof_mm,
                                  uint16_t sonar_mm,
                                  float    mcu_temp_c)
{
    if (s_huart == NULL || battery == NULL) {
        return HAL_ERROR;
    }

    s_seq++;

    /*
     * Compact JSON, single line, newline-terminated.
     * Keep field names short to minimise UART + MQTT bytes.
     *
     * NOTE: requires float printf support. In CubeIDE:
     *   Project -> Properties -> C/C++ Build -> Settings ->
     *   MCU Settings -> "Use float with printf from newlib-nano"
     */
    int n = snprintf(s_buf, TELEM_BUF_SIZE,
        "{"
          "\"seq\":%lu,"
          "\"ts\":%lu,"
          "\"bat\":%.1f,"
          "\"vlt\":%.2f,"
          "\"cur\":%.2f,"
          "\"used\":%.1f,"
          "\"rem\":%.1f,"
          "\"rt\":%lu,"
          "\"bst\":%u,"
          "\"bv\":%u,"
          "\"tmp\":%.1f,"
          "\"tof\":%u,"
          "\"son\":%u"
        "}\n",
        (unsigned long)s_seq,
        (unsigned long)HAL_GetTick(),
        battery->soc_pct,
        battery->voltage_V,
        battery->current_A,
        battery->used_mAh,
        battery->remaining_mAh,
        (unsigned long)battery->runtime_sec,
        (unsigned)battery->status,
        (unsigned)(battery->valid ? 1u : 0u),
        mcu_temp_c,
        (unsigned)tof_mm,
        (unsigned)sonar_mm);

    if (n <= 0 || n >= TELEM_BUF_SIZE) {
        return HAL_ERROR;   /* truncated or formatting failed */
    }

    return HAL_UART_Transmit(s_huart, (uint8_t *)s_buf, (uint16_t)n,
                             TELEM_TX_TIMEOUT);
}
