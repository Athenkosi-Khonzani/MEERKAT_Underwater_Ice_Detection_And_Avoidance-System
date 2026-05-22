# MEERKAT AUV: Control & Actuation Subsystem Firmware (STM32F446RE)

## Overview
This is the **flight firmware** for the MEERKAT Autonomous Underwater Vehicle (AUV). It runs on an STM32F446RET6 (Nucleo-F446RE) and is responsible for the full mission state machine:

* Receiving the mission-start command from the surface ESP32.
* Driving the **Variable Buoyancy System (VBS)** — two DC motors that push and pull a syringe plunger — to descend and ascend.
* Reading the **VL53L1X ToF** sensor as a software-defined limit switch on the plunger.
* Reading the **DYP-L08-V2.0 ultrasonic** sensor for obstacle / ice detection on the ascent path.
* Reading the **INA219** to track battery State of Charge (SoC) and abort the mission if it drops below threshold.
* Streaming JSON telemetry back to the surface every 500 ms.

The project is built with **STM32CubeIDE** against the STM32F4 HAL.

---

## Hardware & Pin Configuration (.ioc mapping)

The target is `STM32F446RETx`. Pin assignments are taken from `Control_System_2.0.ioc` and `Core/Inc/main.h`.

### 1. VL53L1X ToF Sensor (I²C2) — plunger position
* SCL: **PB10** (I2C2_SCL)
* SDA: **PC12** (I2C2_SDA)
* XSHUT: **PA8** (`XShunt_Pin`, GPIO output — pulsed low/high in `main()` to release the sensor from reset)
* I²C address: `0x52` (8-bit shifted)

### 2. INA219 Battery Monitor (I²C1)
* SCL: **PB8** (I2C1_SCL)
* SDA: **PB7** (I2C1_SDA)
* I²C address: `0x40` (7-bit) → `0x80` (8-bit)

### 3. DYP-L08-V2.0 Ultrasonic Sensor (USART1)
* TX (MCU → sensor trigger): **PA9** (USART1_TX)
* RX (sensor → MCU 4-byte frame): **PA10** (USART1_RX)
* Baud rate: **115 200, 8N1**
* Protocol: MCU writes `0x55` to trigger; sensor responds with `[0xFF][D_H][D_L][SUM]`.

### 4. VBS Motor Drive (TIM3 / TIM4 PWM)
* Motor 1 PWM (push plunger → **descend**): **PA6** (TIM3_CH1) → `TIM3->CCR1`
* Motor 2 PWM (pull plunger → **ascend**): **PB6** (TIM4_CH1) → `TIM4->CCR1`
* Setting `CCR = 1599` drives the motor at full duty; `CCR = 0` brakes / stops.

### 5. Surface ESP32 Link (USART3)
* TX (telemetry out): **PC10** (USART3_TX)
* RX (mission-cmd in): **PC5** (USART3_RX, asynchronous)
* Baud rate: **115 200, 8N1**
* RX interrupt is armed in `MissionRx_Init()` — when `"M:1\n"` arrives, the global `missionInit` flag is set.

### 6. User Interface
* User button (Nucleo B1): **PC13** (`Pushbutton_Pin`, EXTI13, falling edge) — alternative way to start the mission via `HAL_GPIO_EXTI_Callback()`.
* Status LED (Nucleo LD2): **PA5** (`LD2_Pin`) — solid ON during the active mission, used as the calibration prompt LED.

### 7. Debugging (SWD)
* SWDIO: **PA13**, SWCLK: **PA14**, SWO: **PB3** — locked open in CubeMX so the ST-Link cannot be locked out.

### 8. MCU Internal Temperature
* **ADC1** sampling the internal `ADC_CHANNEL_TEMPSENSOR` via DMA — used for the `mcu_temp` field in telemetry.

### 9. Real-Time Clock
* **LSE** crystal on **PC14/PC15** drives the RTC, which timestamps the mission duration.

---

## Mission State Machine (`main.c`)

The flight loop in `main()` implements the following gated flow. Each iteration begins with a fresh RTC snapshot, `UpdateSensors()`, and a non-blocking `Telemetry_Tick()`.

```
                ┌────────────────────────────────┐
                │ Wait for missionInit ("M:1")   │
                └──────────────┬─────────────────┘
                               │
                ┌──────────────▼─────────────────┐
                │ Battery enough?  (SoC > 30 %)  │── no ──► stop_motor(); maybe ascend()
                └──────────────┬─────────────────┘
                               │ yes
                ┌──────────────▼─────────────────┐
                │ Plunger at MIN?                │── no ──► Force_plunger_min(); loop
                └──────────────┬─────────────────┘
                               │ yes
                ┌──────────────▼─────────────────┐
                │ descend()  — TIM3 PWM full     │
                └──────────────┬─────────────────┘
                               │
                ┌──────────────▼─────────────────┐
                │ Submerged dwell loop           │
                │  while !mission_complete       │
                │        && SoC >= 20 %          │
                └──────────────┬─────────────────┘
                               │ timer / battery
                ┌──────────────▼─────────────────┐
                │ ascend()  — TIM4 PWM, blocks   │
                │ if sonar_dist < 1000 mm        │
                └────────────────────────────────┘
```

Key constants in `main.c`:
* `MIN = 65`, `MAX = 100` — VBS plunger limits in mm (linear ToF reading after calibration).
* Sonar safety threshold: ascent will not proceed if `sonar_dist <= 1000 mm`.
* Mission timeout: currently `30 s` (line `if (time_elapsed >= 30)`) — change here to extend dives.
* `TELEM_PERIOD_MS = 500` — telemetry packet rate.

---

## Core File Structure

### `main.c`
The mission state machine described above. Hosts the helpers `stop_motor()`, `Force_plunger_min()`, `descend()`, `ascend()`, and `UpdateSensors()`. It also contains a **one-shot ToF calibration routine** (`RunOneShotCalibration()`) which is normally commented out — see the *First-Flash Calibration* section below.

### `ToF.c` / `ToF.h`  (`Core/Inc/`)
Driver and calibration logic for the VL53L1X.
* **Two-point linear calibration.** Reference points are `TOF_CAL_NEAR_REF_MM = 60` and `TOF_CAL_FAR_REF_MM = 120`. The driver averages `TOF_CAL_SAMPLES = 50` raw samples at each point and computes a slope/offset that maps the sensor's distorted reading inside the syringe housing to true millimetres.
* **Persistent storage.** Slope and offset are saved through `FlashStore_*` to **Flash Sector 7** (`0x08060000`) so calibration survives a power cycle.
* Both raw and calibrated reads are exposed (`ToF_ReadDistanceRaw()` / `ToF_ReadDistance()`).

### `Sonar.c` / `Sonar.h`  (`Core/Inc/`)
DYP-L08-V2.0 driver over UART1.
* `Sonar_GetDistance()` sends the `0x55` trigger byte, then blocks up to 100 ms waiting for the 4-byte reply.
* **Frame validation:** header byte (`0xFF`), checksum (`(uint8_t)(0xFF + D_H + D_L)`), and explicit reject of `0x0000` / `0xFFFF` out-of-range sentinels.
* Returns a structured `Sonar_Status` enum so the caller can distinguish a TX timeout from a bad checksum from a true out-of-range reading.

### `battery.c` / `battery.h`  (`Core/Inc/`)
INA219-based State-of-Charge estimator. Default configuration is for the **3S 1000 mAh Li-ion pack** used in the vehicle (full = 12.6 V, empty = 8.4 V). Combines an **OCV lookup table** (used once at boot for the initial SoC seed) with **coulomb counting** during operation. Exposes a `BatteryData_t` snapshot through `Battery_GetData()` and three status flags (`BATTERY_OK / WARNING / CRITICAL`).

### `telemetry.c` / `telemetry.h`  (`Core/Inc/`)
Builds and transmits a newline-terminated JSON telemetry packet over USART3 (to the surface ESP32). One packet contains: sequence number, SoC %, pack voltage, pack current, ToF mm, sonar mm, MCU temperature. Sentinel values `TELEM_DIST_INVALID` (`0xFFFF`) and `TELEM_TEMP_INVALID` (`-999.0`) flag unavailable sensors.

### `mission_rx.c` / `mission_rx.h`  (`Core/Inc/`)
Receives mission commands from the surface ESP32 on USART3 via interrupt. The only command currently parsed is `"M:1"`, which sets the global `missionInit` flag and unblocks the main loop.

### `temp_sensor.c` / `temp_sensor.h`  (`Core/Inc/`)
Thin wrapper around the STM32's internal temperature sensor on ADC1 (DMA-driven), returning a calibrated temperature in °C.

### `FlashStore.c` / `FlashStore.h`  (`Core/Inc/`)
Two-function persistent-storage helper (`FlashStore_Load`, `FlashStore_Save`) used by `ToF.c` to keep the calibration record across reboots. Storage lives in **Flash Sector 7** (`0x08060000`, 128 KB). The linker script (`STM32F446RETX_FLASH.ld`) must cap application FLASH at `0x60000` to keep code out of that sector — this has already been done.

> **Note:** Driver and configuration sources for the I²C, UART, RTC, ADC, and TIM peripherals were generated by CubeMX and live in `Core/Src/`. Pin assignments are described above and are the source of truth.

---

## First-Flash Calibration (one-shot, then re-flash)

The very first time you flash this firmware onto a fresh board, the ToF needs its two-point calibration captured against the **actual** syringe housing. The procedure lives in `RunOneShotCalibration()` in `main.c`:

1. Uncomment the call to `RunOneShotCalibration(&hi2c2)` in `main()`.
2. Place a flat target exactly **60 mm** from the sensor face.
3. Flash and reset. LD2 stays on for 5 s — get ready.
4. LD2 turns off and the firmware captures 50 samples (~3 s).
5. LD2 comes back on for 5 s — move the target to exactly **120 mm**.
6. LD2 turns off, 50 more samples are captured.
7. **3 quick flashes = success.** The slope/offset are saved to Flash Sector 7. Solid-on LED = failure.
8. Re-comment the `RunOneShotCalibration()` call and re-flash with the production build.

The calibration record is persistent — you do **not** need to repeat this between subsequent flashes of the production firmware unless you erase the chip or change syringe geometry.

---

## Building & Flashing

1. Open **STM32CubeIDE 1.13** or later.
2. `File → Open Projects from File System…` → select the `Control_System_2.0/` folder.
3. `Project → Build All`.
4. Plug in the Nucleo (ST-Link enumerates over USB).
5. `Run → Debug` to flash and start a debug session, or `Run → Run` to flash and detach.

The default build output is `Debug/Control_System_2.0.elf`.

---

## Quick Troubleshooting

| Symptom | Likely cause |
|---|---|
| LED blinks fast forever right after boot | `ToF_Init()` failed — check XShunt (PA8) wiring and I²C2 pull-ups on PB10/PC12. |
| LED solid on, never starts mission | `Battery_Init()` returned `HAL_ERROR` — check INA219 wiring and address (0x40) on I²C1 (PB7/PB8). |
| Vehicle never descends after `"M:1"` is sent | Plunger is already at MAX — `Force_plunger_min()` is running first; give it ~10 s. |
| Telemetry stops mid-mission | UART3 wire on PC10 disconnected, or surface ESP32 reset. |
| Vehicle stays down past mission time | Sonar reading is below 1000 mm — something is above the AUV blocking the ascent. |
| ST-Link suddenly cannot connect | SWD pins (PA13/PA14) somehow got remapped — hold the BOOT0 button on power-up to force the system bootloader, then re-flash. |
