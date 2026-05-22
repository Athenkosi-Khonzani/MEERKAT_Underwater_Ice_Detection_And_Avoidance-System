# MEERKAT AUV: Power Subsystem

## Overview
This subsystem is the **power-distribution and monitoring** layer of the MEERKAT AUV. It takes a 3-cell (3S) Li-ion battery pack and produces the two regulated rails the rest of the vehicle needs (**5 V** and **3.3 V**), while continuously measuring pack voltage and current so the flight firmware can estimate state of charge and abort the mission before the battery dies underwater.

The deliverable is two parts:

1. A **KiCad PCB project** (`PCB/KHNATH002_Power_System/`) implementing the regulators and the INA219 monitor.
2. A **portable C firmware module** (`SOC/`) — `battery.c`/`battery.h` — that runs on the flight MCU and turns INA219 readings into a usable SoC + runtime estimate.

---

## Hardware

### Battery Pack
* **Topology:** 3S Li-ion (three 18650-format cells in series).
* **Capacity:** 1000 mAh nominal (series cells share the pack mAh, they do not multiply).
* **Voltage range:**
  * Full = 3 × 4.20 V = **12.60 V**
  * Empty (cutoff) = 3 × 2.80 V ≈ **8.4 V** (configurable via `BAT_VOLTAGE_EMPTY_V`)

### Regulators (`TPS_Regulators.kicad_sch`)
Two **TPS5430DDAR** synchronous-buck regulators step the pack voltage down to the vehicle's two rails:

| Rail | Regulator | Loads |
|---|---|---|
| **5 V** | TPS5430DDAR | DC motors (via H-bridge driver), DYP-L08 ultrasonic sensor |
| **3.3 V** | TPS5430DDAR | STM32F446 MCU, VL53L1X ToF, INA219, ESP32 link |

The TPS5430 is rated 5.5–36 V in, 3 A continuous, with an internal high-side MOSFET — chosen for the headroom over the 12.6 V pack and for the relatively low BOM cost.

> **Note:** An earlier version of the design used the **MP2338GTL** (see `Regulators.kicad_sch`). That sheet is kept in the repo for history but is **not** part of the current top sheet (`KHNATH002_Power_System.kicad_sch`). The TPS implementation is the one that goes to fab.

### Monitoring (`Monitoring.kicad_sch`)
* **INA219** — high-side I²C current/voltage monitor sitting on the pack output, before the regulators. Configured for the 32 V bus-voltage range (the 16 V range saturates on a freshly charged 3S pack) and the /8 gain (320 mV full-scale on the shunt).
* **Shunt:** 0.1 Ω, giving a 100 µA current LSB after calibration → **CAL register = 4096**.
* **I²C address:** `0x40` (7-bit).

The INA219 talks to the flight MCU over **I²C1 (PB7 / PB8)** — see the Control & Actuation Subsystem README.

---

## PCB Layout

The KiCad project lives under:

```
Power Subsystem/PCB/KHNATH002_Power_System/
├── KHNATH002_Power_System.kicad_pro      # top-level project
├── KHNATH002_Power_System.kicad_sch      # top hierarchical sheet
├── KHNATH002_Power_System.kicad_pcb      # board layout
├── TPS_Regulators.kicad_sch              # 5V + 3V3 buck regulators (active)
├── Monitoring.kicad_sch                  # INA219 pack monitor
├── Regulators.kicad_sch                  # MP2338 regulators (legacy, unused)
└── jlcpcb/                               # fabrication outputs for JLCPCB
```

Open `KHNATH002_Power_System.kicad_pro` in **KiCad 7** or later.

---

## Firmware Module — `SOC/`

`battery.c` / `battery.h` is the on-board **State-of-Charge and runtime estimator**. It is not a stand-alone program — it links into the Control & Actuation Subsystem firmware (see that subsystem's README for how it is wired in).

### Algorithm
1. **Seed.** At init, read the open-circuit pack voltage (OCV) from the INA219 and map it through a lookup table to get the starting SoC. This compensates for the unknown charge state at power-on.
2. **Coulomb count.** Every `BAT_SAMPLE_PERIOD_MS` (default 500 ms), read voltage + current, then integrate current × Δt into a running `used_mAh` figure.
3. **OCV correction.** Resting-voltage measurements are blended into the coulomb-counted SoC slowly so the estimate self-corrects over time without jumping under load.
4. **Runtime.** Remaining mAh divided by recent-average current gives a live `runtime_sec` figure exposed via `Battery_GetData()`.

### Public API
```c
HAL_StatusTypeDef Battery_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef Battery_Update(void);
void              Battery_GetData(BatteryData_t *out);
void              Battery_RuntimeToHMS(uint32_t s, uint32_t *h, uint32_t *m, uint32_t *sec);
```

### `BatteryData_t` snapshot
```c
float   soc_pct;        // 0.0 – 100.0
float   voltage_V;      // pack terminal voltage
float   current_A;      // positive = discharging
float   used_mAh;       // since boot
float   remaining_mAh;
uint32_t runtime_sec;
BatteryStatus_t status; // OK / WARNING / CRITICAL
bool    valid;
```

### Configurable thresholds (in `battery.h`)
| Macro | Default | Meaning |
|---|---|---|
| `BAT_SERIES_CELLS` | 3 | Cells in series — adjust if pack changes. |
| `BAT_CAPACITY_MAH` | 1000 | Rated pack capacity (mAh). |
| `BAT_VOLTAGE_FULL_V` | 12.60 | Full pack voltage. |
| `BAT_VOLTAGE_EMPTY_V` | 8.40 | Cutoff. |
| `BAT_VDIV_RATIO` | 1.0 | Set to your divider ratio if INA219 V+ sees less than full pack voltage. |
| `BAT_SOC_OK_THRESHOLD` | 60 % | Above this → `BATTERY_OK`. |
| `BAT_SOC_WARNING_THRESHOLD` | 30 % | Above this → `BATTERY_WARNING`. Below → `BATTERY_CRITICAL`. |
| `BAT_SAMPLE_PERIOD_MS` | 500 | Coulomb-count integration period. |

The flight firmware uses these directly:
* SoC ≤ **30 %** → motor commands refused; pre-descent gate trips.
* SoC < **20 %** → in-mission abort; vehicle ascends if the path is clear.

---

## Build & Flash (firmware module)
The module is built as part of the Control & Actuation Subsystem firmware. To use it stand-alone in another project:

1. Copy `SOC/battery.c` and `SOC/battery.h` into your project.
2. Include `"battery.h"` in your `main.c`.
3. After your I²C peripheral is initialised, call `Battery_Init(&hi2c1)`.
4. Call `Battery_Update()` in your main loop (it rate-limits itself internally).
5. Read data with `Battery_GetData(&snapshot)`.

The module depends only on the STM32 HAL I²C driver — no other RTOS or library.

---

## Voltage-Divider Note (read this before powering up)

The INA219's bus-voltage input is rated to 26 V max but its 16 V config-register mode saturates above ~16 V. The 3S pack peaks at 12.6 V, which is well inside the 26 V hardware limit — **so this design connects V+ directly to the pack**, and `BAT_VDIV_RATIO` is set to `1.0`.

If you re-spin the board with a divider (e.g. for a 4S pack), update `BAT_VDIV_RATIO` in `battery.h` to the new divider ratio so the firmware un-scales the reading correctly.

---

## Status & Known Issues
* The TPS5430 layout has been verified on the bench but **not yet integrated under load with the full vehicle**.
* `Regulators.kicad_sch` (MP2338 variant) is kept in the repo for history. It is **not** part of the current build.
* The schematic does not yet include explicit reverse-polarity protection on the battery input — a Schottky or P-FET is recommended for the next revision.
