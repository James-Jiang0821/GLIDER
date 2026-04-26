# Glider ROS 2 System

ROS 2 Jazzy stack for an underwater glider: sensor drivers, mission
state machine, inner/outer-loop controller, safety watchdog, Iridium
telemetry, and a CAN bridge to the Teensy-based P&R and VBD boards.

## Requirements

- Ubuntu 24.04 + ROS 2 Jazzy
- Python 3.12
- Hardware:
  - Raspberry Pi with I2C and UART enabled
  - Pololu MinIMU-9 v6 (I2C)
  - MAX-M10S GNSS (I2C)
  - Keller pressure sensor (I2C)
  - Ping sonar (UART)
  - Iridium modem (UART)
  - CAN bus to Teensy P&R (pitch/roll) + VBD (variable buoyancy) boards

## Build

The project is cloned into the `src/` of a ROS 2 workspace — e.g.
`~/glider_ws/src/ROS2_SYSTEM/`. Build from the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/glider_ws
colcon build --packages-select glider_msgs
source install/setup.bash
colcon build --packages-select glider_ros
source install/setup.bash
```

## Run

The system is split across three launch files so each runs in its own
terminal with clean, isolated log output:

```bash
# Main system: drivers, adapters, safety, controller
ros2 launch glider_ros all_nodes_launch.py

# Mission state machine — separate terminal
ros2 launch glider_ros state_manager_launch.py

# CAN bridge (Teensy P&R + VBD) — separate terminal
ros2 launch glider_ros can_bridge_launch.py
```

Each fresh terminal needs `source install/setup.bash` once before
`ros2 launch`. Individual nodes can also be launched with
`ros2 run glider_ros <executable>` — see the full executable list in
[setup.py](src/glider_ros/setup.py).

## System overview



The state manager drives two lifecycle nodes (`controller_node`,
`communication_iridium_node`) through `configure` / `activate` /
`deactivate` / `cleanup` transitions as part of mission phasing.

## Layout

- `src/glider_msgs/` — custom messages and actions (`Float64Stamped`,
  `HomeActuators`, `IridiumWindow`).
- `src/glider_ros/glider_ros/driver/` — hardware-facing nodes (I2C,
  UART).
- `src/glider_ros/glider_ros/bridge/` — protocol bridges to external
  microcontrollers (CAN ↔ Teensy P&R / VBD).
- `src/glider_ros/glider_ros/adapter/` — data fusion and formatting
  adapters.
- `src/glider_ros/glider_ros/manager/` — mission state machine.
- `src/glider_ros/glider_ros/controller/` — flight controller (lifecycle).
- `src/glider_ros/glider_ros/safety/` — and watchdog
  monitor.
- `src/glider_ros/launch/` — launch files per deployment scenario.
- `src/glider_ros/config/` — YAML parameter files.
- `src/glider_ros/test/` — pytest unit/integration tests.

For what each node does, please read the module docstring at the top of its
`.py` file.

## Testing

### Unit Testing

To perform unit testing, from the workspace root, run:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m pytest src/glider_ros/test/ -v --cov=glider_ros --cov-report=term-missing --cov-report=html

```

Unit tests cover the adapter, safety, and state manager nodes. HTML
coverage lands in `htmlcov/`.


## Authors

- James Jiang <jamesjiang0821@gmail.com>
- Hussain Khan <hussainniassuh786@gmail.com>

## License

MIT. See `package.xml` / `setup.py`.
