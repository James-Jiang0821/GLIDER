#!/usr/bin/env python3
"""Keller pressure sensor I2C driver that publishes depth (m) on /pressure/depth."""

import math
import struct
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from sensor_msgs.msg import FluidPressure, Temperature
from std_msgs.msg import String
from std_srvs.srv import Trigger
from glider_msgs.msg import Float64Stamped

try:
    from smbus2 import SMBus, i2c_msg
except ImportError:
    SMBus = None
    i2c_msg = None


class KellerLD:
    """Python port of Blue Robotics Arduino Keller LD logic (I2C 0x40, conversion 0xAC, calibration from memory map)."""

    LD_REQUEST = 0xAC
    LD_CUST_ID0 = 0x00
    LD_CUST_ID1 = 0x01
    LD_SCALING0 = 0x12
    LD_SCALING1 = 0x13
    LD_SCALING2 = 0x14
    LD_SCALING3 = 0x15
    LD_SCALING4 = 0x16

    def __init__(self, bus: SMBus, address: int = 0x40, fluid_density: float = 1029.0):
        self.bus = bus
        self.address = address
        self.fluid_density = fluid_density

        self.equipment: Optional[int] = None
        self.place: Optional[int] = None
        self.file: Optional[int] = None

        self.mode: Optional[int] = None
        self.year: Optional[int] = None
        self.month: Optional[int] = None
        self.day: Optional[int] = None

        self.code: Optional[int] = None

        self.P_mode: float = 0.0
        self.P_min: Optional[float] = None
        self.P_max: Optional[float] = None

        self.P_raw: Optional[int] = None
        self.P_bar: Optional[float] = None
        self.T_degc: Optional[float] = None

    def set_fluid_density(self, density: float) -> None:
        self.fluid_density = density

    def _write_byte(self, value: int) -> None:
        self.bus.write_byte(self.address, value)

    def _read_block(self, length: int) -> list[int]:
        return self.bus.read_i2c_block_data(self.address, 0x00, length)

    def read_memory_map(self, mtp_address):

        #write the memory address
        write = i2c_msg.write(self.address, [mtp_address])
        self.bus.i2c_rdwr(write)

        time.sleep(0.001)

        #read 3 bytes
        read = i2c_msg.read(self.address, 3)
        self.bus.i2c_rdwr(read)

        data = list(read)

        status = data[0]
        value = (data[1] << 8) | data[2]

        return value

    @staticmethod
    def _u32_to_float_be(value: int) -> float:
        return struct.unpack(">f", struct.pack(">I", value))[0]

    def init(self) -> None:
        cust_id0 = self.read_memory_map(self.LD_CUST_ID0)
        cust_id1 = self.read_memory_map(self.LD_CUST_ID1)

        self.code = (cust_id1 << 16) | cust_id0
        self.equipment = cust_id0 >> 10
        self.place = cust_id0 & 0b000000111111111
        self.file = cust_id1

        scaling0 = self.read_memory_map(self.LD_SCALING0)

        self.mode = scaling0 & 0b00000011
        self.year = scaling0 >> 11
        self.month = (scaling0 & 0b0000011110000000) >> 7
        self.day = (scaling0 & 0b0000000001111100) >> 2

        #same mode handling as Arduino library
        if self.mode == 0:
            #PR mode vented gauge
            self.P_mode = 1.01325
        elif self.mode == 1:
            #PA mode sealed gauge
            self.P_mode = 1.0
        else:
            #PAA mode absolute
            self.P_mode = 0.0

        scaling12 = (self.read_memory_map(self.LD_SCALING1) << 16) | self.read_memory_map(self.LD_SCALING2)
        scaling34 = (self.read_memory_map(self.LD_SCALING3) << 16) | self.read_memory_map(self.LD_SCALING4)

        self.P_min = self._u32_to_float_be(scaling12)
        self.P_max = self._u32_to_float_be(scaling34)

    def is_initialized(self) -> bool:
        return (
            self.equipment is not None
            and self.equipment != 63
            and self.P_min is not None
            and self.P_max is not None
        )

    def read(self) -> Tuple[float, float, float]:
        """Returns (pressure_pa, temperature_c, depth_m)."""
        if not self.is_initialized():
            raise RuntimeError("Sensor not initialized")

        #send conversion request
        self._write_byte(self.LD_REQUEST)

        #Arduino code uses 9 ms max conversion delay
        time.sleep(0.009)

        data = self._read_block(5)
        if len(data) != 5:
            raise RuntimeError("Failed to read sensor conversion block")

        _status = data[0]
        self.P_raw = (data[1] << 8) | data[2]
        raw_t = (data[3] << 8) | data[4]

        self.P_bar = ((float(self.P_raw) - 16384.0) * (self.P_max - self.P_min) / 32768.0) + self.P_min + self.P_mode
        self.T_degc = (((raw_t >> 4) - 24) * 0.05) - 50.0

        pressure_mbar = self.P_bar * 1000.0
        pressure_pa = pressure_mbar * 100.0
        depth_m = (pressure_pa - 101325.0) / (self.fluid_density * 9.80665)

        return pressure_pa, self.T_degc, depth_m


class KellerPressureNode(Node):
    def __init__(self):
        super().__init__("pressure_keller_node")

        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("i2c_address", 0x40)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("fluid_density", 1029.0)   #seawater
        self.declare_parameter("frame_id", "pressure_link")
        self.declare_parameter("surface_zero_samples", 20)
        self.declare_parameter("depth_offset", 0.0)
        #EMA on depth; smaller α = more smoothing, α=1.0 disables
        self.declare_parameter("depth_filter_alpha", 0.2)
        #reseed filter if gap since last sample exceeds this (e.g. after reconnect)
        self.declare_parameter("depth_filter_reset_gap_s", 1.0)

        self.i2c_bus_num = int(self.get_parameter("i2c_bus").value)
        self.i2c_address = int(self.get_parameter("i2c_address").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.fluid_density = float(self.get_parameter("fluid_density").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.surface_zero_samples = int(self.get_parameter("surface_zero_samples").value)
        self.depth_offset = float(self.get_parameter("depth_offset").value)
        self.depth_filter_alpha = float(self.get_parameter("depth_filter_alpha").value)
        self.depth_filter_reset_gap_s = float(self.get_parameter("depth_filter_reset_gap_s").value)

        self._filt_depth_m: Optional[float] = None
        self._prev_filt_stamp_s: Optional[float] = None
        #reference atmospheric pressure used for depth calculation; overwritten by surface tare
        self.surface_pressure_pa = 101325.0

        self.pressure_pub = self.create_publisher(FluidPressure, "/pressure/raw_pressure", 10)
        self.temperature_pub = self.create_publisher(Temperature, "/pressure/temperature", 10)
        self.depth_pub = self.create_publisher(Float64Stamped, "/pressure/depth", 10)
        self.status_pub = self.create_publisher(String, "/pressure/status", 10)

        #latched so any subscriber (or rosbag) joining late still gets the current calibration value
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.surface_ref_pub = self.create_publisher(Float64Stamped, "/pressure/surface_pressure_ref", latched_qos)
        self._publish_surface_ref()

        self.bus: Optional[SMBus] = None
        self.sensor: Optional[KellerLD] = None

        self._connect_sensor()

        #surface tare only at boot — never on reconnect, in case we're underwater
        if self.sensor is not None and self.surface_zero_samples > 0:
            self._tare_surface_pressure(self.surface_zero_samples)

        self.zero_srv = self.create_service(Trigger, "/pressure/zero", self._handle_zero_request)

        period = 1.0 / self.publish_rate_hz if self.publish_rate_hz > 0.0 else 0.2
        self.timer = self.create_timer(period, self.timer_callback)

    def publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _publish_surface_ref(self) -> None:
        msg = Float64Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.data = self.surface_pressure_pa
        self.surface_ref_pub.publish(msg)

    def _close_bus(self) -> None:
        if self.bus is not None:
            try:
                self.bus.close()
            except Exception:
                pass
        self.bus = None
        self.sensor = None

    def _connect_sensor(self) -> None:
        if SMBus is None:
            err = "python package 'smbus2' is not installed"
            self.get_logger().error(err)
            self.publish_status(err)
            return

        self._close_bus()

        try:
            self.bus = SMBus(self.i2c_bus_num)
            self.sensor = KellerLD(
                bus=self.bus,
                address=self.i2c_address,
                fluid_density=self.fluid_density,
            )
            self.sensor.init()

            if not self.sensor.is_initialized():
                raise RuntimeError("sensor detected but initialization data invalid")

            info = (
                f"Keller sensor initialized on i2c bus {self.i2c_bus_num}, "
                f"address 0x{self.i2c_address:02X}, "
                f"mode={self.sensor.mode}, "
                f"Pmin={self.sensor.P_min:.3f} bar, "
                f"Pmax={self.sensor.P_max:.3f} bar"
            )
            self.get_logger().info(info)
            self.publish_status(info)

        except Exception as exc:
            self._close_bus()
            self.get_logger().warn(f"Failed to initialize pressure sensor: {exc}")
            self.publish_status(f"init_failed: {exc}")

    def _tare_surface_pressure(self, n: int) -> Optional[float]:
        if self.sensor is None:
            return None
        samples = []
        for _ in range(n):
            try:
                pressure_pa, _, _ = self.sensor.read()
                samples.append(pressure_pa)
            except Exception as exc:
                self.get_logger().warn(f"tare sample failed: {exc}")
            time.sleep(0.05)
        if not samples:
            self.publish_status("tare_failed: no valid samples")
            return None
        self.surface_pressure_pa = sum(samples) / len(samples)
        self._publish_surface_ref()
        info = f"surface tare: {self.surface_pressure_pa:.1f} Pa over {len(samples)} samples"
        self.get_logger().info(info)
        self.publish_status(info)
        return self.surface_pressure_pa

    def _handle_zero_request(self, request, response):
        n = self.surface_zero_samples if self.surface_zero_samples > 0 else 20
        result = self._tare_surface_pressure(n)
        if result is None:
            response.success = False
            response.message = "tare failed: sensor unavailable or no samples"
        else:
            response.success = True
            response.message = f"surface_pressure_pa={result:.1f}"
        return response

    def timer_callback(self) -> None:
        if self.sensor is None:
            self._connect_sensor()
            if self.sensor is None:
                return

        try:
            pressure_pa, temp_c, _ = self.sensor.read()
            depth_m = (pressure_pa - self.surface_pressure_pa) / (self.fluid_density * 9.80665) + self.depth_offset

            now = self.get_clock().now().to_msg()
            now_s = now.sec + now.nanosec * 1e-9

            #reseed on first sample or after a long gap
            alpha = self.depth_filter_alpha
            if (
                self._filt_depth_m is None
                or self._prev_filt_stamp_s is None
                or (now_s - self._prev_filt_stamp_s) > self.depth_filter_reset_gap_s
            ):
                self._filt_depth_m = depth_m
            else:
                self._filt_depth_m = alpha * depth_m + (1.0 - alpha) * self._filt_depth_m
            self._prev_filt_stamp_s = now_s

            pressure_msg = FluidPressure()
            pressure_msg.header.stamp = now
            pressure_msg.header.frame_id = self.frame_id
            pressure_msg.fluid_pressure = pressure_pa
            pressure_msg.variance = 0.0

            temp_msg = Temperature()
            temp_msg.header.stamp = now
            temp_msg.header.frame_id = self.frame_id
            temp_msg.temperature = temp_c
            temp_msg.variance = 0.0

            depth_msg = Float64Stamped()
            depth_msg.header.stamp = now
            depth_msg.header.frame_id = self.frame_id
            depth_msg.data = self._filt_depth_m

            self.pressure_pub.publish(pressure_msg)
            self.temperature_pub.publish(temp_msg)
            self.depth_pub.publish(depth_msg)

        except Exception as exc:
            self.get_logger().warn(f"Pressure read failed, will reconnect: {exc}")
            self.publish_status(f"read_failed: {exc}")
            self._close_bus()
            self._filt_depth_m = None
            self._prev_filt_stamp_s = None


def main(args=None):
    rclpy.init(args=args)
    node = KellerPressureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._close_bus()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()