#ifndef SONAR_H
#define SONAR_H

#include <stdint.h>
#include "stm32f4xx_hal.h"   /* change to stm32f4xx_hal.h on the Black Pill */

/* Driver status codes */
typedef enum {
    SONAR_OK = 0,
    SONAR_ERR_TX,            /* failed to send trigger byte           */
    SONAR_ERR_RX,            /* timeout or UART error on reply        */
    SONAR_ERR_HEADER,        /* first byte was not 0xFF               */
    SONAR_ERR_CHECKSUM,      /* checksum mismatch                     */
    SONAR_ERR_OUT_OF_RANGE,  /* sensor reported 0x0000 or 0xFFFF      */
    SONAR_ERR_NULL           /* null pointer passed in                */
} Sonar_Status;

/* Sensor handle — one per physical sensor */
typedef struct {
    UART_HandleTypeDef *huart;   /* UART the sensor is wired to       */
    uint32_t tx_timeout_ms;      /* trigger transmit timeout          */
    uint32_t rx_timeout_ms;      /* reply receive timeout             */
} Sonar_Handle;

/**
 * @brief  Initialise a sonar instance.
 * @param  sonar  Pointer to a Sonar_Handle to populate.
 * @param  huart  UART peripheral the sensor is connected to (already
 *                initialised at 115200 8N1 by CubeMX).
 * @retval SONAR_OK on success, SONAR_ERR_NULL on bad args.
 */
Sonar_Status Sonar_Init(Sonar_Handle *sonar, UART_HandleTypeDef *huart);

/**
 * @brief  Trigger a measurement and return the distance.
 * @param  sonar     Initialised sonar handle.
 * @param  distance_mm  Output: distance in millimetres on success.
 * @retval SONAR_OK on a valid reading, or an error code otherwise.
 *
 * @note   Blocks for up to ~150 ms in the worst case. Leave at least
 *         100 ms between consecutive calls to avoid echo interference.
 */
Sonar_Status Sonar_GetDistance(Sonar_Handle *sonar, uint16_t *distance_mm);

#endif /* SONAR_H */
