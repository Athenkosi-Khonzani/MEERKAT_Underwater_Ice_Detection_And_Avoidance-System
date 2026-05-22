#include "Sonar.h"

#define SONAR_TRIGGER_BYTE   0x55
#define SONAR_FRAME_LEN      4
#define SONAR_HEADER         0xFF

/* Default timeouts — generous enough for the sensor's worst-case echo. */
#define SONAR_DEFAULT_TX_TIMEOUT_MS  50
#define SONAR_DEFAULT_RX_TIMEOUT_MS  100

Sonar_Status Sonar_Init(Sonar_Handle *sonar, UART_HandleTypeDef *huart)
{
    if (sonar == NULL || huart == NULL)
        return SONAR_ERR_NULL;

    sonar->huart          = huart;
    sonar->tx_timeout_ms  = SONAR_DEFAULT_TX_TIMEOUT_MS;
    sonar->rx_timeout_ms  = SONAR_DEFAULT_RX_TIMEOUT_MS;

    return SONAR_OK;
}

Sonar_Status Sonar_GetDistance(Sonar_Handle *sonar, uint16_t *distance_mm)
{
    if (sonar == NULL || sonar->huart == NULL || distance_mm == NULL)
        return SONAR_ERR_NULL;

    uint8_t trigger = SONAR_TRIGGER_BYTE;
    uint8_t buf[SONAR_FRAME_LEN];

    /* 1. Flush any stale bytes from a previous failed read. */
    __HAL_UART_CLEAR_OREFLAG(sonar->huart);

    /* 2. Send trigger byte. */
    if (HAL_UART_Transmit(sonar->huart, &trigger, 1,
                          sonar->tx_timeout_ms) != HAL_OK)
        return SONAR_ERR_TX;

    /* 3. Receive 4-byte reply: [0xFF][Data_H][Data_L][SUM] */
    if (HAL_UART_Receive(sonar->huart, buf, SONAR_FRAME_LEN,
                         sonar->rx_timeout_ms) != HAL_OK)
        return SONAR_ERR_RX;

    /* 4. Validate header. */
    if (buf[0] != SONAR_HEADER)
        return SONAR_ERR_HEADER;

    /* 5. Validate checksum: low byte of (header + data_h + data_l). */
    uint8_t expected = (uint8_t)(buf[0] + buf[1] + buf[2]);
    if (buf[3] != expected)
        return SONAR_ERR_CHECKSUM;

    /* 6. Decode big-endian distance in mm. */
    uint16_t mm = ((uint16_t)buf[1] << 8) | buf[2];
    if (mm == 0x0000 || mm == 0xFFFF)
        return SONAR_ERR_OUT_OF_RANGE;

    *distance_mm = mm;
    return SONAR_OK;
}
