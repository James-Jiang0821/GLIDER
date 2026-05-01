#!/usr/bin/env python3
"""IMU adapter: computes roll, pitch, heading, and angular rates from MinIMU-9 v6 raw outputs"""

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, MagneticField
from glider_msgs.msg import Float64Stamped


class ImuAdapterNode(Node):
    def __init__(self):
        super().__init__("imu_adapter_node")

        self.create_subscription(Imu, "/imu/data_raw", self._on_imu, 20)
        self.create_subscription(MagneticField, "/imu/mag", self._on_mag, 20)

        self._roll_pub = self.create_publisher(Float64Stamped, "/glider/roll_rad", 20)
        self._pitch_pub = self.create_publisher(Float64Stamped, "/glider/pitch_rad", 20)
        self._pitch_rate_pub = self.create_publisher(Float64Stamped, "/glider/pitch_rate_rad_s", 20)
        self._roll_rate_pub = self.create_publisher(Float64Stamped, "/glider/roll_rate_rad_s", 20)
        self._heading_pub = self.create_publisher(Float64Stamped, "/glider/heading_deg", 20)

        #degree versions for ros2 topic echo
        self._roll_deg_pub = self.create_publisher(Float64Stamped, "/glider/roll_deg", 20)
        self._pitch_deg_pub = self.create_publisher(Float64Stamped, "/glider/pitch_deg", 20)

        #cached mag sample between IMU ticks
        self._mag = None  #(mx, my, mz) in Tesla

    def _on_mag(self, msg: MagneticField) -> None:
        self._mag = (
            msg.magnetic_field.x,
            msg.magnetic_field.y,
            msg.magnetic_field.z,
        )

    def _on_imu(self, msg: Imu) -> None:
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        #roll and pitch from gravity vector
        #IMU mounted flat: X_imu=body-right, Y_imu=body-forward, Z_imu=body-up
        roll = math.atan2(-ax, az)
        pitch = math.atan2(-ay, math.sqrt(ax * ax + az * az))

        #tilt-compensated heading (only if mag sample exists)
        if self._mag is not None:
            mx, my, mz = self._mag
            cr, sr = math.cos(roll), math.sin(roll)
            cp, sp = math.cos(pitch), math.sin(pitch)
            bx = mx * cp + my * sr * sp + mz * cr * sp
            by = my * cr - mz * sr
            yaw = math.atan2(-by, bx)
            heading_deg = math.degrees(yaw) % 360.0
        else:
            heading_deg = None

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
        if heading_deg is not None:
            outputs.append((self._heading_pub, heading_deg))

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
