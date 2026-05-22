#ifndef SONAR_H
#define SONAR_H

#include "stm32f0xx_hal.h"

// Structure to hold our sensor data
typedef struct {
    uint16_t distance_mm;
    uint8_t  is_valid;
} Sonar_Data_t;

// Function Prototypes
void Sonar_Init(UART_HandleTypeDef *huart);
Sonar_Data_t Sonar_Read(UART_HandleTypeDef *huart);

#endif // SONAR_H
