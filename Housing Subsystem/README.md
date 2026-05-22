# MEERKAT AUV: Housing Subsystem

## Overview
This subsystem is the **mechanical pressure vessel** for the MEERKAT AUV. Its job is to keep the electronics, the battery, and the Variable Buoyancy System (VBS) syringe dry while the vehicle is submerged, and to provide a rigid mounting point for the single DC motor that drives the VBS plunger.

All parts are designed in **SolidWorks** (version 2022 or later). They are supplied as individual `.SLDPRT` files — there is no master `.SLDASM` assembly in the repo yet.

---

## Parts in this Folder

| File | Purpose |
|---|---|
| `FullBody.SLDPRT` | Outer cylindrical hull of the AUV. Defines the overall envelope and the end-cap interfaces. |
| `Imporved Upper Lid.SLDPRT` | Top end cap. Carries the through-hull port for the **upward-looking ultrasonic sensor** (used during ascent to check the path is clear of ice). *(filename typo preserved for git history.)* |
| `Improved lowe lid.SLDPRT` | Bottom end cap. Mounts the `Circular motor holder` on its inner face and provides the wet-side opening for the VBS syringe. *(filename typo preserved for git history.)* |
| `Circular motor holder.SLDPRT` | Bulkhead bracket fixed to the lower lid; locates and clamps the **single DC motor** that drives the VBS syringe plunger. |
| `Compartment battery.SLDPRT` | Internal tray that holds the 3S Li-ion pack and keeps it isolated from the veroboards. |
| `Compartment 2 veroboard.SLDPRT` | Internal mounting tray for the power-electronics veroboard (regulators + INA219). |
| `Compartment3 veroboard.SLDPRT` | Internal mounting tray for the Control & Actuation veroboard (STM32F446 carrier + sensor breakouts). |

---

## Internal Layout

The hull is divided into three stacked compartments along its axis, with the motor and VBS syringe mounted at the bottom:

```
   ┌──────────────────────────────────────────────────────────┐
   │                  Upper Lid                               │  ← upward-looking ultrasonic port
   ├──────────────────────────────────────────────────────────┤
   │   Compartment 3   — Control veroboard (STM32F446 + I/O)  │
   ├──────────────────────────────────────────────────────────┤
   │   Compartment 2   — Power veroboard (TPS5430 + INA219)   │
   ├──────────────────────────────────────────────────────────┤
   │   Battery Compartment — 3S Li-ion pack                   │
   ├──────────────────────────────────────────────────────────┤
   │                  Lower Lid                               │  ← carries Circular Motor Holder
   │   Circular Motor Holder — 1 × DC motor → VBS syringe     │
   └──────────────────────────────────────────────────────────┘
```

The arrangement keeps the **heaviest component (battery) low** to give the vehicle a stable righting moment, and isolates the **noisy power electronics** in their own compartment so they do not couple into the sensor / MCU veroboard above. Mounting the motor and VBS at the bottom also keeps the wet-side syringe opening submerged for the entire mission.

---

## Sealing Strategy
* End caps mate against the outer body via an **O-ring face seal** on each lid.
* The **upper lid** carries the upward-looking ultrasonic sensor — its housing is sealed against the lid, and a cable gland alongside it passes the sensor's power and signal wires into the dry compartment below.
* The **lower lid** carries the VBS syringe opening (wet side) and the through-bolts that hold the `Circular motor holder` against its dry-side face.
* The motor holder itself is **internal only** — it is not a sealing surface; it just constrains the motor body axially and radially so the plunger reaction force does not deflect it.

---

## Manufacturing Notes
* The lids and internal compartments are intended for **3D printing** (PLA or PETG for prototypes, ABS or nylon for water trials).
* The cylindrical `FullBody` is a **PVC pipe section** that the lids press into — the part file captures the inner diameter against which the lid O-rings seal.
* Allow O-ring squeeze of **15–20 %** when you print final lids; PLA prints loose by ~0.2 mm on diameter.

---

## Opening the Files
1. Open **SolidWorks 2022** or later.
2. `File → Open` and select any `.SLDPRT`.
3. Each part is self-contained and has no external references — they can be opened in any order.

If you build an assembly, recommended mate order:
1. Insert `FullBody` (fix in space).
2. Mate the two lids to its end faces.
3. Mate `Circular motor holder` against the **inner face of the lower lid**.
4. Stack the three compartment trays inside the body, working upward from the battery tray.

---

## Status & Known Issues
* Two part filenames have typos (`Imporved`, `lowe`) — preserved to avoid breaking git history. Renaming them is safe but should be done in a single commit and announced to anyone with the repo open.
* The repo does **not yet contain a top-level assembly file** (`.SLDASM`) or a drawing pack (`.SLDDRW`). These should be added before the design is taken to manufacturing.
* No tolerance analysis has been documented for the O-ring grooves yet — verify against the supplier's recommended squeeze before committing to a print.
* The motor-holder bracket has been dry-fit only; it has not been tested under the full single-motor plunger reaction force in water.
