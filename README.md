# MEERKAT — Underwater Ice Detection and Avoidance System

MEERKAT is a small, low-cost Autonomous Underwater Vehicle (AUV) designed to detect and avoid sub-surface ice (frazil ice, brash ice, and submerged keels of icebergs) in turbid polar waters. The platform descends and ascends using a **Variable Buoyancy System (VBS)** — a syringe driven by two DC motors — rather than thrusters, which keeps power consumption low and the acoustic signature small.

This repository contains the full design package: mechanical CAD, the power-electronics PCB, and the embedded firmware that runs the vehicle.

---

## System Architecture

MEERKAT is divided into four independent subsystems. Each lives in its own folder and has its own README with the wiring, build, and flashing details.

```
                      ┌──────────────────────────────┐
                      │      Surface ESP32           │
                      │  (mission start / telemetry) │
                      └──────────────┬───────────────┘
                                     │ UART (115200 8N1)
                                     │ JSON telemetry  /  "M:1" mission cmd
                ┌────────────────────┴────────────────────┐
                │   Control & Actuation Subsystem (F446)  │
                │   Mission state machine, motor PWM,     │
                │   VBS limit logic, battery monitoring   │
                └─┬──────────────┬──────────────┬─────────┘
                  │ I²C          │ UART         │ PWM
                  │              │              │
        ┌─────────▼────────┐ ┌───▼────────┐ ┌──▼───────────────┐
        │  INA219          │ │ DYP-L08    │ │ Dual DC motors   │
        │  (pack V / I)    │ │ Ultrasonic │ │ on VBS syringe   │
        └──────────────────┘ │ (obstacle) │ └──────────────────┘
                             └────────────┘
                  │ I²C
        ┌─────────▼────────────┐
        │  VL53L1X ToF         │
        │  (plunger position)  │
        └──────────────────────┘

        ┌──────────────────────┐    ┌──────────────────────┐
        │  Power Subsystem     │    │  Housing Subsystem   │
        │  3S Li-ion + TPS5430 │    │  Pressure vessel +   │
        │  → 5 V & 3V3 rails   │    │  motor / VBS mounts  │
        └──────────────────────┘    └──────────────────────┘
```

| Subsystem | What it does | Tech |
|---|---|---|
| **Control & Actuation** | Mission state machine; drives the VBS motors; reads sensors; speaks to the surface ESP32. | STM32F446RET6 (Nucleo-F446RE), STM32CubeIDE, HAL |
| **Sensory** | Stand-alone test firmware for the two ranging sensors used on the vehicle. | STM32F051C6T6, STM32CubeIDE, HAL |
| **Power** | 3S Li-ion pack regulated to a 5 V and a 3.3 V rail; INA219 pack monitor; SoC estimator firmware module. | KiCad 7, TPS5430DDAR buck regulators, INA219 |
| **Housing** | Watertight pressure vessel with separate compartments for the battery, two veroboards, and the syringe / motor assembly. | SolidWorks (`.SLDPRT`) |

---

## Mission Concept of Operations
 
1. **Surface idle.** Pack voltage is monitored. The vehicle waits for a `"M:1"` command from the surface ESP32 over UART3.
2. **Pre-descent check.** The plunger is forced to its minimum position (syringe empty of water). If the battery is below the warning threshold, the mission is refused.
3. **Descend.** The motor is driven forward through the H-bridge; the plunger is pushed out to its maximum position, the syringe pulls water in, and the vehicle sinks.
4. **Submerged loop.** The vehicle stays down for a configurable mission time. Throughout, it:
   * streams JSON telemetry to the surface every 500 ms (SoC, ToF, sonar, MCU temperature),
   * watches the upward-looking DYP-L08 ultrasonic for obstacles closer than 1 m,
   * watches the battery and aborts to ascent if SoC < 20 %.
5. **Ascend.** Once the mission timer expires *and* the path above is clear, the motor is driven in reverse through the H-bridge — the plunger is pulled back in, the syringe expels water, and the vehicle rises. If an obstacle is detected above, the ascent waits for it to clear.
The state machine is implemented in `Control And Actuation Subsystem/Control_System_2.0/Core/Src/main.c`.

---

## Repository Layout

```
.
├── Control And Actuation Subsystem/   # STM32CubeIDE project — main flight firmware
├── Sensory Subsystem/                 # STM32CubeIDE project — sensor bring-up firmware
├── Power Subsystem/                   # KiCad PCB project + SoC estimator source
└── Housing Subsystem/                 # SolidWorks part files for the pressure vessel
```

Each folder has its own README that covers wiring, build instructions, and any setup gotchas. Start there for hands-on work.

---

## Getting Started

### Firmware (Control & Sensory)
1. Install **STM32CubeIDE 1.13** or later.
2. `File → Open Projects from File System…` and point it at the `Control_System_2.0` or `Sensor_Test` folder.
3. Build (`Project → Build All`).
4. Connect the ST-Link, then `Run → Debug` to flash.

The Control subsystem expects a **one-shot calibration** the very first time it is flashed onto a new board — the procedure is documented in its README and is gated by an `#ifdef`-style block in `main.c`.

### PCB (Power)
1. Install **KiCad 7** or later.
2. Open `Power Subsystem/PCB/KHNATH002_Power_System/KHNATH002_Power_System.kicad_pro`.
3. The fabrication outputs for JLCPCB are under `jlcpcb/`.

### Mechanical (Housing)
The parts are `.SLDPRT` files for **SolidWorks 2022** or later. They have not yet been packaged into an assembly file — open each part individually.

---

## Status

| Subsystem | Hardware | Firmware | Verified end-to-end |
|---|:---:|:---:|:---:|
| Control & Actuation | ✅ | ✅ | ✅ |
| Sensory | ✅ | ✅ | ✅ |
| Power | ✅ | ✅ | ⚠️ bench-tested only |
| Housing | ✅ | n/a | ⚠️ dry-fit only |

---

## Authors

EEE4113F project, University of Cape Town, Department of Electrical Engineering.

* Athenkosi Khonzani — Power Subsystem 
* Ayanda Madlala — Housing Subsystem 
* Branden Nkhahle — Sensory Subsystem
* Manelisi Ngcobo — Control And Actuation Subsystem

---

## License

Unless otherwise noted, source code is provided as-is for academic use. STM32 HAL drivers are distributed under their original STMicroelectronics license (see `Drivers/` directory in each firmware project).
