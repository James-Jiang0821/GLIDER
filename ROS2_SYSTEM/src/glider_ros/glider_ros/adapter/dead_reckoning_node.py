#!/usr/bin/env python3
"""Dead reckoning, It Publishes : /dr/heading_*, /dr/speed_mps_*, /dr/local_xy_* and /dr/fix_*."""

import math
from typing import Optional

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, MagneticField, NavSatFix, NavSatStatus
from geometry_msgs.msg import PointStamped
from glider_msgs.msg import Float64Stamped


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)
G = 9.80665


def _stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _meters_per_deg(lat_deg: float) -> tuple[float, float]:
    lat_rad = math.radians(lat_deg)
    sin_lat = math.sin(lat_rad)
    cos_lat = math.cos(lat_rad)
    denom = math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    m_per_deg_lat = math.pi * WGS84_A * (1.0 - WGS84_E2) / (180.0 * denom ** 3)
    m_per_deg_lon = math.pi * WGS84_A * cos_lat / (180.0 * denom)
    return m_per_deg_lat, m_per_deg_lon


class DeadReckoningNode(Node):
    def __init__(self):
        super().__init__("dead_reckoning_node")

        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("depth_rate_tau_s", 1.0)
        self.declare_parameter("speed_pitch_threshold_rad", 0.0872665)
        self.declare_parameter("speed_max_mps", 1.0)
        self.declare_parameter("mag_declination_rad", 0.0)
        self.declare_parameter("accel_bias_x", 0.0)
        self.declare_parameter("imu_topic", "/imu/data_raw")

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.depth_rate_tau_s = float(self.get_parameter("depth_rate_tau_s").value)
        self.speed_pitch_threshold_rad = float(self.get_parameter("speed_pitch_threshold_rad").value)
        self.speed_max_mps = float(self.get_parameter("speed_max_mps").value)
        self.mag_declination_rad = float(self.get_parameter("mag_declination_rad").value)
        self.accel_bias_x = float(self.get_parameter("accel_bias_x").value)
        self.imu_topic = str(self.get_parameter("imu_topic").value)

        self._roll: Optional[float] = None
        self._pitch: Optional[float] = None
        self._mag = None

        self._depth: Optional[float] = None
        self._depth_rate: float = 0.0
        self._last_depth_stamp: Optional[float] = None

        self._speed_geom: float = 0.0

        self._V_x_body: float = 0.0
        self._imu_t_prev: Optional[float] = None

        self._origin_lat_deg: Optional[float] = None
        self._origin_lon_deg: Optional[float] = None
        self._m_per_deg_lat: float = 0.0
        self._m_per_deg_lon: float = 0.0

        self._east_geom_m: float = 0.0
        self._north_geom_m: float = 0.0
        self._east_accel_m: float = 0.0
        self._north_accel_m: float = 0.0
        self._last_integrate_t: Optional[float] = None

        self.create_subscription(Float64Stamped, "/glider/roll_rad", self._on_roll, 20)
        self.create_subscription(Float64Stamped, "/glider/pitch_rad", self._on_pitch, 20)
        self.create_subscription(MagneticField, "/imu/mag", self._on_mag, 20)
        self.create_subscription(Float64Stamped, "/pressure/depth", self._on_depth, 10)
        self.create_subscription(NavSatFix, "/gps/fix", self._on_gps, 10)
        self.create_subscription(Imu, self.imu_topic, self._on_imu, 50)

        self._heading_pub = self.create_publisher(Float64Stamped, "/dr/heading_rad", 10)
        self._heading_deg_pub = self.create_publisher(Float64Stamped, "/dr/heading_deg", 10)

        self._speed_geom_pub = self.create_publisher(Float64Stamped, "/dr/speed_mps_geom", 10)
        self._speed_accel_pub = self.create_publisher(Float64Stamped, "/dr/speed_mps_accel", 10)

        self._local_geom_pub = self.create_publisher(PointStamped, "/dr/local_xy_geom", 10)
        self._local_accel_pub = self.create_publisher(PointStamped, "/dr/local_xy_accel", 10)

        self._fix_geom_pub = self.create_publisher(NavSatFix, "/dr/fix_geom", 10)
        self._fix_accel_pub = self.create_publisher(NavSatFix, "/dr/fix_accel", 10)

        period = 1.0 / self.publish_rate_hz if self.publish_rate_hz > 0.0 else 0.2
        self.timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f"DR observer started, declination={math.degrees(self.mag_declination_rad):+.2f} deg, "
            f"pitch threshold {math.degrees(self.speed_pitch_threshold_rad):.1f} deg, "
            f"accel_bias_x={self.accel_bias_x:+.4f} m/s^2"
        )

    def _on_roll(self, msg: Float64Stamped) -> None:
        self._roll = float(msg.data)

    def _on_pitch(self, msg: Float64Stamped) -> None:
        self._pitch = float(msg.data)

    def _on_mag(self, msg: MagneticField) -> None:
        self._mag = (
            float(msg.magnetic_field.x),
            float(msg.magnetic_field.y),
            float(msg.magnetic_field.z),
        )

    def _on_depth(self, msg: Float64Stamped) -> None:
        t = _stamp_to_sec(msg.header.stamp)
        z = float(msg.data)

        if self._depth is None or self._last_depth_stamp is None:
            self._depth = z
            self._last_depth_stamp = t
            return

        dt = t - self._last_depth_stamp
        if dt <= 0.0:
            return

        raw_rate = (z - self._depth) / dt
        tau = max(self.depth_rate_tau_s, 1e-3)
        alpha = dt / (tau + dt)
        self._depth_rate = (1.0 - alpha) * self._depth_rate + alpha * raw_rate
        self._depth = z
        self._last_depth_stamp = t

    def _on_imu(self, msg: Imu) -> None:
        if self._pitch is None:
            return
        t_now = self.get_clock().now().nanoseconds * 1e-9
        a_x = float(msg.linear_acceleration.x)
        if self._imu_t_prev is not None:
            dt = t_now - self._imu_t_prev
            if dt > 0.0:
                g_body_x = -G * math.sin(self._pitch)
                a_x_lin = a_x - g_body_x - self.accel_bias_x
                self._V_x_body += a_x_lin * dt
        self._imu_t_prev = t_now

    def _on_gps(self, msg: NavSatFix) -> None:
        if msg.status.status == NavSatStatus.STATUS_NO_FIX:
            return

        if self._origin_lat_deg is None:
            self._origin_lat_deg = float(msg.latitude)
            self._origin_lon_deg = float(msg.longitude)
            self._m_per_deg_lat, self._m_per_deg_lon = _meters_per_deg(self._origin_lat_deg)
            self._east_geom_m = 0.0
            self._north_geom_m = 0.0
            self._east_accel_m = 0.0
            self._north_accel_m = 0.0
            self.get_logger().info(
                f"DR origin latched at lat={self._origin_lat_deg:.7f}, "
                f"lon={self._origin_lon_deg:.7f}"
            )

    def _compute_heading_rad(self) -> Optional[float]:
        if self._mag is None or self._roll is None or self._pitch is None:
            return None

        mx, my, mz = self._mag

        cr = math.cos(self._roll)
        sr = math.sin(self._roll)
        cp = math.cos(self._pitch)
        sp = math.sin(self._pitch)

        mx_h = mx * cr + mz * sr
        my_h = mx * sr * sp + my * cp - mz * cr * sp

        heading = math.atan2(-mx_h, my_h) + self.mag_declination_rad
        return math.atan2(math.sin(heading), math.cos(heading))

    def _compute_speed_geom_mps(self) -> float:
        if self._pitch is None:
            return self._speed_geom

        if abs(self._pitch) < self.speed_pitch_threshold_rad:
            return self._speed_geom

        denom = math.tan(self._pitch)
        if abs(denom) < 1e-3:
            return self._speed_geom

        v_h = -self._depth_rate / denom
        if not math.isfinite(v_h):
            return self._speed_geom

        v_h = max(-self.speed_max_mps, min(self.speed_max_mps, v_h))
        self._speed_geom = v_h
        return v_h

    def _compute_speed_accel_mps(self) -> float:
        if self._pitch is None:
            return 0.0
        return self._V_x_body * math.cos(self._pitch)

    def _publish_fix(self, pub, east_m, north_m, frame_id):
        if self._origin_lat_deg is None:
            return
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = frame_id
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = self._origin_lat_deg + north_m / self._m_per_deg_lat
        fix.longitude = self._origin_lon_deg + east_m / self._m_per_deg_lon
        fix.altitude = -(self._depth if self._depth is not None else 0.0)
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        pub.publish(fix)

    def _publish_local(self, pub, east_m, north_m, frame_id):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.point.x = east_m
        msg.point.y = north_m
        msg.point.z = self._depth if self._depth is not None else 0.0
        pub.publish(msg)

    def _tick(self) -> None:
        now = self.get_clock().now().to_msg()
        now_s = _stamp_to_sec(now)

        heading = self._compute_heading_rad()
        speed_geom = self._compute_speed_geom_mps()
        speed_accel = self._compute_speed_accel_mps()

        if heading is not None:
            h_msg = Float64Stamped()
            h_msg.header.stamp = now
            h_msg.data = float(heading)
            self._heading_pub.publish(h_msg)

            h_deg_msg = Float64Stamped()
            h_deg_msg.header.stamp = now
            h_deg_msg.data = math.degrees(heading)
            self._heading_deg_pub.publish(h_deg_msg)

        s_msg = Float64Stamped()
        s_msg.header.stamp = now
        s_msg.data = float(speed_geom)
        self._speed_geom_pub.publish(s_msg)

        s_msg = Float64Stamped()
        s_msg.header.stamp = now
        s_msg.data = float(speed_accel)
        self._speed_accel_pub.publish(s_msg)

        if heading is None or self._origin_lat_deg is None:
            self._last_integrate_t = now_s
            return

        if self._last_integrate_t is not None:
            dt = now_s - self._last_integrate_t 
            if dt > 0.0:
                sin_h = math.sin(heading)
                cos_h = math.cos(heading)
                self._east_geom_m += speed_geom * sin_h * dt
                self._north_geom_m += speed_geom * cos_h * dt
                self._east_accel_m += speed_accel * sin_h * dt
                self._north_accel_m += speed_accel * cos_h * dt
        self._last_integrate_t = now_s

        self._publish_local(self._local_geom_pub, self._east_geom_m, self._north_geom_m, "dr_origin")
        self._publish_local(self._local_accel_pub, self._east_accel_m, self._north_accel_m, "dr_origin")
        self._publish_fix(self._fix_geom_pub, self._east_geom_m, self._north_geom_m, "dr_origin")
        self._publish_fix(self._fix_accel_pub, self._east_accel_m, self._north_accel_m, "dr_origin")


def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
