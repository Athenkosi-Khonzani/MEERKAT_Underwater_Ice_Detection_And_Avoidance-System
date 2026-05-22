#ifndef MISSION_RX_H
#define MISSION_RX_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

extern volatile uint8_t missionInit;   /* set to 1 when "M:1" received */

/**
 * @brief  Arm UART3 RX interrupt to receive mission commands from ESP32.
 *         Call once in main() after MX_USART3_UART_Init().
 */
void MissionRx_Init(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif
