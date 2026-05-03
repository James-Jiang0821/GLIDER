#!/usr/bin/env python3
"""IMU adapter: computes roll, pitch, and angular rates from MinIMU-9 v6 raw outputs"""

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from glider_msgs.msg import Float64Stamped


class ImuAdapterNode(Node):
    def __init__(self):
        super().__init__("imu_adapter_node")

        self.declare_parameter("mount_pitch_offset_rad", 0.0)
        self.declare_parameter("mount_roll_offset_rad", 0.0)
        self.mount_pitch_offset_rad = float(self.get_parameter("mount_pitch_offset_rad").value)
        self.mount_roll_offset_rad = float(self.get_parameter("mount_roll_offset_rad").value)

        self.get_logger().info(
            f"Mount offsets (rad): pitch={self.mount_pitch_offset_rad:+.5f}, roll={self.mount_roll_offset_rad:+.5f}"
        )

        self.create_subscription(Imu, "/imu/data_raw", self._on_imu, 20)


        self._roll_pub = self.create_publisher(Float64Stamped, "/glider/roll_rad", 20)
        self._pitch_pub = self.create_publisher(Float64Stamped, "/glider/pitch_rad", 20)
        self._pitch_rate_pub = self.create_publisher(Float64Stamped, "/glider/pitch_rate_rad_s", 20)
        self._roll_rate_pub = self.create_publisher(Float64Stamped, "/glider/roll_rate_rad_s", 20)

        #degree versions for ros2 topic echo
        self._roll_deg_pub = self.create_publisher(Float64Stamped, "/glider/roll_deg", 20)
        self._pitch_deg_pub = self.create_publisher(Float64Stamped, "/glider/pitch_deg", 20)

    def _on_imu(self, msg: Imu) -> None:
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        #roll and pitch from gravity vector
        #IMU mounted flat: X_imu=body-right, Y_imu=body-forward, Z_imu=body-up
        roll = math.atan2(-ax, az) - self.mount_roll_offset_rad
        pitch = math.atan2(-ay, math.sqrt(ax * ax + az * az)) - self.mount_pitch_offset_rad

        roll_rate = msg.angular_velocity.y
        pitch_rate = -msg.angular_velocity.x

        stamp = msg.header.stamp

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
