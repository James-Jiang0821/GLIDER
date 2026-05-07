# ROS2 Testing

Post-test analysis scripts for ROS2 bag files recorded on a Raspberry Pi. Each folder corresponds to a test session and contains scripts to extract and process data from `.mcap` bag files into CSV format for analysis.

## Test Sessions

| Folder | Date | Description |
|--------|------|-------------|
| `gnss_test_07-03-2026` | 07 Mar 2026 | GPS fix, velocity and diagnostics |
| `imu_test_07-03-2026` | 07 Mar 2026 | IMU data |
| `imu_test_2_16-03-2026` | 16 Mar 2026 | IMU data (second run) |
| `iridium_node_test_08-03-2026` | 08 Mar 2026 | Iridium modem node |
| `iridium_gnss_signal_test` | — | Iridium signal strength with GNSS |
| `iridium_gnss_test2_21-03-2026` | 21 Mar 2026 | Iridium and GNSS (second run) |
| `iridium_gnss_test4_22-03-2026` | 22 Mar 2026 | Iridium and GNSS (fourth run) |
| `pressure_test_16-03-2026` | 16 Mar 2026 | Fluid pressure and temperature |
| `sonar_test_14-03-2026` | 14 Mar 2026 | Sonar |
| `sonar_water_test_v2-20-04-2026` | 20 Apr 2026 | Sonar in water (v2) |

## Scripts

Each session folder typically contains one or more of these:

- `bag_to_csv.py` — extracts all topics from the bag into CSV files
- `split.py` — splits specific topics into separate CSV files
- `plot.py` / `graph.py` — plots extracted data

## Requirements

```
mcap
mcap-ros2-support
pandas
matplotlib
numpy
```

Install with:

```bash
pip install mcap mcap-ros2-support pandas matplotlib numpy
```

## Usage

Set the `MCAP_FILE` variable in each script to your bag file, then run:

```bash
python bag_to_csv.py
```
