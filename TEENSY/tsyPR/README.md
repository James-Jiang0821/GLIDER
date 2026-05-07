# P&R Teensy Firmware (`tsyPR`)

Firmware for the Pitch & Roll (P&R) board on the underwater glider.
Runs on a Teensy 4.0 and drives two stepper-actuated axes — a linear
pitch mass and a rotational roll mass — with limit-based homing and a
CAN bridge back to the Raspberry Pi running the
[Glider ROS 2 stack](../../ROS2_SYSTEM/README.md).

A single board handles both axes (`NODE_ID=3`).

## Requirements

- PlatformIO (Arduino framework, `platform = teensy`)
- Hardware:
  - Teensy 4.0
  - 2 × DRV8825 stepper drivers (pitch + roll)
  - Pitch stepper + lead-screw mass slider (0.01 mm/step)
  - Roll stepper + rotational mass (0.045 °/step)
  - 2 × push-switch endstops (pitch fwd/rev limits)
  - Hall-effect sensor (roll zero/homing)
  - Adafruit VL6180X ToF sensor (I2C, pitch position cross-check)
  - TinyBMS over UART (battery telemetry)
  - Leak sensor
  - CAN transceiver on CAN2 (Teensy pins 0/1)

## Build & Flash

From [TEENSY/tsyPR/](.):

```bash
# Build
pio run

# Build + flash over USB
pio run -t upload

# Serial monitor (115200 baud)
pio device monitor
```

Node identity is set by the `-D NODE_ID=...` build flag in
[platformio.ini](platformio.ini). Default is `3` (P&R). Override with:

```bash
pio run -t upload --project-option="build_flags=-D NODE_ID=3"
```

## System overview

The control loop runs:

1. Read step counters + endstops/hall + ToF -> per-axis position
   estimate (steps as primary, ToF/hall as homing reference).
2. `SafetyManager` aggregates leak, driver fault, overcurrent, stall,
   command timeout, and BMS health into `hard_fault` / `soft_fault`
   bits — controllers never decide faults themselves.
3. `PitchController` and `RollController` each run their state machine
   (`UNHOMED -> HOMING -> RUN -> HOLD -> FAULT`), stepping the DRV8825 at
   the configured step rate.
4. CAN frames publish pitch/roll position and status; setpoints and
   mode commands arrive on CAN from the ROS bridge.

## Layout

- [platformio.ini](platformio.ini) — build env, lib deps, `NODE_ID`
- [include/config/](include/config/) — `pin_map.hpp`, `constants.hpp`
  (step geometry, travel limits, safety thresholds, CAN scaling)
- [include/drivers/](include/drivers/) /
  [src/drivers/](src/drivers/) — hardware-facing classes:
  - `stepper_driver` (DRV8825, STEP/DIR/ENA/FLT)
  - `push_switch` (pitch fwd/rev limit endstops)
  - `hall_sensor` (roll homing reference)
  - `tof_sensor` (VL6180X over Wire1)
  - `leak_sensor`, `bms` (TinyBMS UART)
  - `can_bus` (FlexCAN wrapper, 11-bit IDs)
  - `main.cpp` — composition root, control loop
- [include/controllers/](include/controllers/) /
  [src/controllers/](src/controllers/) — control logic:
  - `pitch_controller` (state machine, limit-switch homing,
    soft-zone limits)
  - `roll_controller` (state machine, hall-sensor homing,
    soft-zone limits)
  - `safety_manager` (centralised fault aggregation, latched)
  - `bms_manager` (battery state + timeout policy)
- [include/protocols/](include/protocols/) /
  [src/protocols/](src/protocols/) — `can_messages.hpp`/`.cpp`,
  CAN frame IDs and packing.
- [include/utils/logger.hpp](include/utils/logger.hpp) — serial logging.
- [test/](test/) — bench-test notes for motors, CAN, safety, BMS, etc.

## Key parameters

Defined in [include/config/constants.hpp](include/config/constants.hpp):

- Pitch: 0.01 mm/step, ±56.9 mm physical travel
- Roll: 0.045 °/step, ±90° physical travel
- Overcurrent: 6.0 A hard, 3.0 A stall threshold
- Stall confirm: 2.0 s at ≥25% duty with <0.1 mm motion
- Homing timeout: 60 s
- Leak debounce: 50 ms; driver-fault debounce: 20 ms
- Command timeout: 300 ms (small) / 120 s (large)
- BMS timeout: 5 s (small) / 120 s (large)

## Testing

Bench tests are scripted/manual procedures captured as notes in
[test/](test/), numbered by integration stage (motor only -> motor +
CAN -> safety -> BMS). Run them in order when bringing up a new board.

## Authors

Jasper Yip