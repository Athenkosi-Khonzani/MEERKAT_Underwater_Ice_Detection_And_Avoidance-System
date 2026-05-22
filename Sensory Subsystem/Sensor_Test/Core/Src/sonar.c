#include "sonar.h"

void Sonar_Init(UART_HandleTypeDef *huart) {
    // Clear out any old garbage data sitting in the UART receiver before we start
    __HAL_UART_FLUSH_DRREGISTER(huart);
}

Sonar_Data_t Sonar_Read(UART_HandleTypeDef *huart) {
    Sonar_Data_t result;
    result.distance_mm = 0;
    result.is_valid = 0;

    // 1. Send the Trigger Pulse
    // The datasheet requires a "low pulse" on the sensor's RX lead[cite: 2648].
    // Sending a dummy byte (0x01) over UART forces the TX line low, perfectly triggering the sensor.
    uint8_t trigger_byte = 0x01;
    if (HAL_UART_Transmit(huart, &trigger_byte, 1, 10) != HAL_OK) {
        return result; // Transmit failed, return invalid
    }

    // 2. Wait for the 4-byte response
    // The sensor takes ~20ms to respond[cite: 2663]. We use a 50ms timeout to be safe.
    uint8_t rx_buffer[4] = {0};
    if (HAL_UART_Receive(huart, rx_buffer, 4, 50) == HAL_OK) {

        // 3. Verify the Frame Header (Must always be 0xFF)
        if (rx_buffer[0] == 0xFF) {

            // 4. Calculate the Checksum: SUM = (Header + Data_H + Data_L) & 0x00FF [cite: 2668]
            uint8_t calculated_sum = (rx_buffer[0] + rx_buffer[1] + rx_buffer[2]) & 0xFF;

            if (rx_buffer[3] == calculated_sum) {
                // 5. Checksum passed! Calculate the actual distance[cite: 2670].
                result.distance_mm = (rx_buffer[1] << 8) | rx_buffer[2];
                result.is_valid = 1; // Mark as successful
            }
        }
    }

    return result;
}
