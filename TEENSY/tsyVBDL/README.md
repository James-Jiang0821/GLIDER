# VBD Teensy Firmware (`tsyVBDL`)

Firmware for the Variable Buoyancy Device (VBD) board on the underwater
glider. Runs on a Teensy 4.0 and drives a lead-screw piston via a
brushed DC motor, with closed-loop position control and a CAN bridge
back to the Raspberry Pi running the [Glider ROS 2 stack](../../ROS2_SYSTEM/README.md).

Two physical boards share this firmware — `PORT` (NODE_ID=1) and
`STARBOARD` (NODE_ID=2) — selected at build time.

## Requirements

- PlatformIO (Arduino framework, `platform = teensy`)
- Hardware:
  - Teensy 4.0
  - Pololu G2 High-Power 24v13 motor driver
  - Maxon brushed DC motor + 1024 CPR quadrature encoder, 156:1 gearbox
  - 4 mm/rev lead screw, 150 mm stroke piston
  - Adafruit VL6180X ToF sensor (I2C, stroke fusion)
  - Inductive proximity sensor (homing endstop)
  - TinyBMS over UART (battery telemetry)
  - Leak sensor
  - CAN transceiver on CAN2 (Teensy pins 0/1)

## Build & Flash

From [TEENSY/tsyVBDL/](.):

```bash
# Build
pio run

# Build + flash over USB
pio run -t upload

# Serial monitor (115200 baud)
pio device monitor
```

Node identity is set by the `-D NODE_ID=...` build flag in
[platformio.ini](platformio.ini). Default is `1` (LEFT). Override per
board:

```bash
pio run -t upload --project-option="build_flags=-D NODE_ID=2"
```

## System overview

The 1 kHz control loop runs:

1. Read encoder + ToF → `Estimator` produces fused stroke estimate
   (counts as primary, ToF as drift/slip correction).
2. `SafetyManager` aggregates leak, motor fault, overcurrent, stall,
   command timeout, and BMS health into `hard_fault` / `soft_fault`
   bits — controllers never decide faults themselves.
3. `PistonController` runs its state machine
   (`UNHOMED → HOMING → RUN → HOLD → FAULT`), driving `PIDController`
   inside `RUN`.
4. CAN frames at 50 Hz publish position, current, and status; setpoints
   and mode commands arrive on CAN from the ROS bridge.

## Layout

- [platformio.ini](platformio.ini) — build env, lib deps, `NODE_ID`
- [include/config/](include/config/) — `pin_map.hpp`, `constants.hpp`
  (stroke geometry, PID gains, safety thresholds, CAN scaling)
- [include/drivers/](include/drivers/) /
  [src/drivers/](src/drivers/) — hardware-facing classes:
  - `motor_driver` (Pololu G2, PWM/DIR/SLP/FLT, current sense)
  - `encoder` / `enc_alt` (quad encoder, two backends)
  - `tof_sensor` (VL6180X over Wire1)
  - `proximity` (homing endstop)
  - `leak_sensor`, `bms` (TinyBMS UART)
  - `can_bus` (FlexCAN wrapper, 11-bit IDs)
  - `main.cpp` — composition root, control loop
- [include/controllers/](include/controllers/) /
  [src/controllers/](src/controllers/) — control logic:
  - `estimator` (encoder + ToF fusion, slip detection)
  - `pid_controller` (P, I, D with anti-windup + output clamp)
  - `piston_controller` (state machine, homing, soft-zone limits)
  - `safety_manager` (centralised fault aggregation, latched)
  - `bms_manager` (battery state + timeout policy)
- [include/protocols/](include/protocols/) /
  [src/protocols/](src/protocols/) — `can_messages.hpp`/`.cpp`,
  CAN frame IDs and packing (e.g. `STATUS_FAULT 0x050`,
  position in 0.1 mm units).
- [include/utils/logger.hpp](include/utils/logger.hpp) — serial logging.
- [test/](test/) — bench-test notes for motor, ToF, PID, CAN, homing,
  safety, encoder, BMS, leak, prox, etc.

## Key parameters

Defined in [include/config/constants.hpp](include/config/constants.hpp):

- Control loop: 1 kHz, PWM 25 kHz
- Stroke: 0–150 mm physical, 10–145 mm hard limits, 20–135 mm soft zone
- PID: Kp=0.10, Ki=0.0, Kd=0.003
- Position tolerance: 0.1 mm (enter HOLD), 2.0 mm drift restarts PID
- Homing: 40% PWM toward prox, 10% backoff
- Slip detection: 3 mm encoder-vs-ToF disagreement
- Overcurrent: 6.0 A hard, 3.0 A stall threshold
- CAN position scaling: 0.1 mm/LSB

## Testing

Bench tests are scripted/manual
procedures captured as notes in [test/](test/), numbered by integration
stage (motor only → ToF → PID → CAN → homing → safety). Run them in
order when bringing up a new board.

## Authors

Jasper Yip