# MEERKAT AUV: Sensory Subsystem Firmware (STM32F411)

## Overview
This repository contains the STM32CubeIDE project for the Sensory Subsystem of the MEERKAT Autonomous Underwater Vehicle (AUV). This subsystem provides critical internal proprioception (Variable Buoyancy System limits) and external exteroception (underwater obstacle avoidance) for the platform.

The firmware successfully integrates two primary sensors:

1. VL53L1X Time-of-Flight (ToF) Sensor: Acts as a continuous, software-defined proximity limit switch to track the linear displacement of the VBS syringe plunger (20mm to 120mm limits).
2. DYP-L08-V2.0 Ultrasonic Sensor: Provides long-range (up to 10m) acoustic obstacle and ice detection in highly turbid Antarctic waters.

## Hardware & Pin Configuration (.ioc mapping)
The project is configured for the STM32F411 microcontroller. The .ioc file contains the following critical pin mappings and peripheral configurations:

### 1. VL53L1X ToF Sensor (I2C1)
- Protocol: I2C (Address: 0x52 - 8-bit shifted)
- SCL: PB6 (or as defined in your .ioc)
- SDA: PB7 (or as defined in your .ioc)

### 2. DYP-L08-V2.0 Ultrasonic Sensor (UART2 / GPIO)
- Protocol: UART (Baud Rate: 115200, 8N1)
- Trigger Pin (Yellow Wire): PA0 (Configured as GPIO Output. MCU pulls this LOW for 1ms to trigger a ping).
- Data RX Pin (White Wire): PA3 (USART2_RX. Receives the 4-byte distance frame).

### 3. Debugging & Failsafe (SWD)
- SWDIO: PA13
- SWCLK: PA14
- Note: The debug pins are explicitly locked open in the firmware (GPIO_MODE_AF_PP, GPIO_AF0_SWJ) to prevent the ST-Link from being locked out during sleep states or clock reconfigurations.

---

## Core File Structure (/Core/Src and /Core/Inc)

### main.c
Handles the primary boot sequence, peripheral initialization, and main execution loop.
- Safe Boot Sequence: On startup, main.c checks the internal Flash memory for a saved calibration profile. If found, it bypasses setup. If missing, it forces the AUV into a calibration state, prompting the user to manually move the syringe to the 20mm and 120mm bounds.
- Hardware Failsafe: Explicitly enables the GPIOA clock and maps PA13/PA14 to the debugger immediately after the system clock configuration to prevent SWD lockouts.

### ToF.c & ToF.h
Contains the drivers, mathematical filters, and memory management for the Variable Buoyancy limit switch.
- Two-Point Linear Calibration: Calculates a dynamic multiplier (m) and offset (b) using ToF_Calibrate_TwoPoint() to mathematically eliminate optical crosstalk caused by the confined syringe housing.
- Sliding Window Median Filter: Buffers distance samples and extracts the median to reject violent optical scatter spikes before they trigger a false motor-stop.
- Non-Volatile Memory Management: Saves the m and b floats permanently into Flash Sector 5 (Address 0x08020000).

### Sonar.c & Sonar.h (or equivalent UART parsing logic)
Handles the acoustic ranging data for collision avoidance.
- Frame Parsing: Reconstructs the physical distance by bit-shifting the High and Low bytes received over UART.
- Checksum Validation: Discards corrupted UART frames to prevent erratic avoidance maneuvers.
- Moving Average Filter: Smooths incoming readings to mitigate false positives caused by localized bubble swarms or water currents.
