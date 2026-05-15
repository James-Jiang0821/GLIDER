#!/usr/bin/env python3
"""IMU adapter: computes roll, pitch, and angular rates from MinIMU-9 v6 raw outputs"""

import math

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger
from glider_msgs.msg import Float64Stamped


class ImuAdapterNode(Node):
    def __init__(self):
        super().__init__("imu_adapter_node")

        self.declare_parameter("mount_offset_samples", 250)
        self.declare_parameter("mount_offset_max_var", 1.0e-4)
        self.declare_parameter("publish_during_cal", False)
        self.declare_parameter("comp_alpha", 0.98)
        self.declare_parameter("comp_dt_max_s", 0.2)

        self.mount_offset_samples = int(self.get_parameter("mount_offset_samples").value)
        self.mount_offset_max_var = float(self.get_parameter("mount_offset_max_var").value)
        self.publish_during_cal = bool(self.get_parameter("publish_during_cal").value)
        self.comp_alpha = float(self.get_parameter("comp_alpha").value)
        self.comp_dt_max_s = float(self.get_parameter("comp_dt_max_s").value)

        self.mount_pitch_offset_rad = 0.0
        self.mount_roll_offset_rad = 0.0
        self._cal_buffer = []
        self._mount_calibrated = False

        self._filt_roll = 0.0
        self._filt_pitch = 0.0
        self._prev_stamp_s = None

        self.create_subscription(Imu, "/imu/data_raw", self._on_imu, 20)
        self._zero_srv = self.create_service(Trigger, "/imu/mount_zero", self._handle_mount_zero)

        self._roll_pub = self.create_publisher(Float64Stamped, "/glider/roll_rad", 20)
        self._pitch_pub = self.create_publisher(Float64Stamped, "/glider/pitch_rad", 20)
        self._pitch_rate_pub = self.create_publisher(Float64Stamped, "/glider/pitch_rate_rad_s", 20)
        self._roll_rate_pub = self.create_publisher(Float64Stamped, "/glider/roll_rate_rad_s", 20)

        #degree versions for ros2 topic echo
        self._roll_deg_pub = self.create_publisher(Float64Stamped, "/glider/roll_deg", 20)
        self._pitch_deg_pub = self.create_publisher(Float64Stamped, "/glider/pitch_deg", 20)

        self._start_mount_cal()

    def _start_mount_cal(self):
        self._cal_buffer = []
        self._mount_calibrated = False
        self._prev_stamp_s = None
        self.get_logger().info(
            f"Mount offset cal: collecting {self.mount_offset_samples} stationary samples"
        )

    def _handle_mount_zero(self, request, response):
        self._start_mount_cal()
        response.success = True
        response.message = f"mount cal restarted: {self.mount_offset_samples} samples"
        return response

    def _on_imu(self, msg: Imu) -> None:
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        #roll and pitch from gravity vector
        #IMU mounted flat: X_imu=body-right, Y_imu=body-forward, Z_imu=body-up
        raw_roll = math.atan2(-ax, az)
        raw_pitch = math.atan2(-ay, math.sqrt(ax * ax + az * az))

        if not self._mount_calibrated:
            self._cal_buffer.append([raw_roll, raw_pitch])
            if len(self._cal_buffer) >= self.mount_offset_samples:
                arr = np.array(self._cal_buffer)
                max_var = float(np.max(np.var(arr, axis=0)))
                if max_var > self.mount_offset_max_var:
                    self.get_logger().warn(
                        f"motion during mount cal (max var {max_var:.6f} > {self.mount_offset_max_var:.6f}), restarting"
                    )
                    self._cal_buffer = []
                else:
                    means = np.mean(arr, axis=0)
                    self.mount_roll_offset_rad = float(means[0])
                    self.mount_pitch_offset_rad = float(means[1])
                    self._mount_calibrated = True
                    self._filt_roll = 0.0
                    self._filt_pitch = 0.0
                    self._prev_stamp_s = None
                    self.get_logger().info(
                        f"Mount offsets (rad): roll={self.mount_roll_offset_rad:+.5f}, "
                        f"pitch={self.mount_pitch_offset_rad:+.5f} (max var {max_var:.6f})"
                    )

            if not self.publish_during_cal:
                return

        accel_roll = raw_roll - self.mount_roll_offset_rad
        accel_pitch = raw_pitch - self.mount_pitch_offset_rad

        roll_rate = msg.angular_velocity.y
        pitch_rate = -msg.angular_velocity.x

        stamp = msg.header.stamp
        stamp_s = stamp.sec + stamp.nanosec * 1e-9

        if self._prev_stamp_s is None:
            self._filt_roll = accel_roll
            self._filt_pitch = accel_pitch
        else:
            dt = stamp_s - self._prev_stamp_s
            if dt <= 0.0 or dt > self.comp_dt_max_s:
                self._filt_roll = accel_roll
                self._filt_pitch = accel_pitch
            else:
                a = self.comp_alpha
                self._filt_roll = a * (self._filt_roll + roll_rate * dt) + (1.0 - a) * accel_roll
                self._filt_pitch = a * (self._filt_pitch + pitch_rate * dt) + (1.0 - a) * accel_pitch

        self._prev_stamp_s = stamp_s
        roll = self._filt_roll
        pitch = self._filt_pitch

        outputs = [
            (self._roll_pub, roll),
            (self._pitch_pub, pitch),
            (self._roll_deg_pub, math.degrees(roll)),
            (self._pitch_deg_pub, math.degrees(pitch)),
            (self._pitch_rate_pub, pitch_rate),
            (self._roll_rate_pub, roll_rate),
        ]

        for pub, value in outputs:
            out = Float64Stamped()
            out.header.stamp = stamp
            out.data = value
            pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
