# Glider System

Top-level repo for the underwater glider. 

ROS 2 stack on the Raspberry Pi, Teensy firmware for the P&R and VBD boards, KiCad designs for the custom electronics, and post-test analysis tooling.

Each subfolder has its own README with full details.

## Subsystems

- **[ROS2_SYSTEM/](ROS2_SYSTEM/README.md)** — ROS 2 Jazzy stack on the Raspberry Pi: sensor drivers, mission state machine, inner/outer-loop controller, safety watchdog, Iridium telemetry, and the CAN bridge to the Teensy boards.
- **[TEENSY/](TEENSY/)** — Teensy 4.0 firmware for the actuator boards.
  - **[tsyPR/](TEENSY/tsyPR/README.md)** — Pitch & Roll firmware (stepper-driven pitch mass + roll mass).
  - **[tsyVBDL/](TEENSY/tsyVBDL/README.md)** — Variable Buoyancy Device firmware (lead-screw piston with closed-loop position control).
- **[PCB/](PCB/README.MD)** — KiCad schematics and layouts for the custom P&R and VBD boards.
- **[BATTERY_BOARD/](BATTERY_BOARD/README.MD)** — KiCad designs for the P&R and VBD battery boards (A/B variants).
- **[ROS2_TESTING/](ROS2_TESTING/README.md)** — Post-test analysis scripts for ROS 2 bag files (`.mcap` to CSV).