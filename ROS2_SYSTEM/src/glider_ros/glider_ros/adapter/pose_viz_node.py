#!/usr/bin/env python3
"""Fuses roll/pitch/heading and dead-reckoned XY+depth into a TF transform and a hull marker for 3D viz."""

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, TransformStamped
from visualization_msgs.msg import Marker
from glider_msgs.msg import Float64Stamped

from tf2_ros import TransformBroadcaster


def _euler_to_quat(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class PoseVizNode(Node):
    def __init__(self):
        super().__init__("pose_viz_node")

        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("parent_frame", "map")
        self.declare_parameter("child_frame", "base_link")
        self.declare_parameter("hull_length_m", 1.5)
        self.declare_parameter("hull_diameter_m", 0.15)
        self.declare_parameter("mesh_resource", "package://glider_ros/meshes/glider.stl")
        #STL authored in metres -> 1.0; STL in millimetres -> 0.001
        self.declare_parameter("mesh_scale", 1.0)

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.parent_frame = str(self.get_parameter("parent_frame").value)
        self.child_frame = str(self.get_parameter("child_frame").value)
        self.hull_length_m = float(self.get_parameter("hull_length_m").value)
        self.hull_diameter_m = float(self.get_parameter("hull_diameter_m").value)
        self.mesh_resource = str(self.get_parameter("mesh_resource").value)
        self.mesh_scale = float(self.get_parameter("mesh_scale").value)

        self._roll: float = 0.0
        self._pitch: float = 0.0
        self._yaw: float = 0.0
        self._east_m: Optional[float] = None
        self._north_m: Optional[float] = None
        self._depth_m: float = 0.0

        self.create_subscription(Float64Stamped, "/glider/roll_rad", self._on_roll, 20)
        self.create_subscription(Float64Stamped, "/glider/pitch_rad", self._on_pitch, 20)
        self.create_subscription(Float64Stamped, "/dr/heading_rad", self._on_heading, 10)
        self.create_subscription(Float64Stamped, "/pressure/depth", self._on_depth, 10)
        self.create_subscription(PointStamped, "/dr/local_xy_geom", self._on_local_xy, 10)

        self._tf_broadcaster = TransformBroadcaster(self)
        self._marker_pub = self.create_publisher(Marker, "/viz/glider_hull", 1)

        period = 1.0 / self.publish_rate_hz if self.publish_rate_hz > 0 else 0.05
        self.timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f"pose_viz_node started: {self.parent_frame} -> {self.child_frame} at {self.publish_rate_hz:.1f} Hz"
        )

    def _on_roll(self, msg: Float64Stamped) -> None:
        self._roll = float(msg.data)

    def _on_pitch(self, msg: Float64Stamped) -> None:
        self._pitch = float(msg.data)

    def _on_heading(self, msg: Float64Stamped) -> None:
        self._yaw = float(msg.data)

    def _on_depth(self, msg: Float64Stamped) -> None:
        self._depth_m = float(msg.data)

    def _on_local_xy(self, msg: PointStamped) -> None:
        self._east_m = float(msg.point.x)
        self._north_m = float(msg.point.y)

    def _tick(self) -> None:
        now = self.get_clock().now().to_msg()

        x = self._east_m if self._east_m is not None else 0.0
        y = self._north_m if self._north_m is not None else 0.0
        z = -self._depth_m

        qx, qy, qz, qw = _euler_to_quat(self._roll, self._pitch, self._yaw)

        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = self.parent_frame
        tf.child_frame_id = self.child_frame
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = z
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(tf)

        hull = Marker()
        hull.header.stamp = now
        hull.header.frame_id = self.child_frame
        hull.ns = "glider"
        hull.id = 0
        hull.action = Marker.ADD
        hull.pose.orientation.w = 1.0
        hull.color.r = 0.9
        hull.color.g = 0.9
        hull.color.b = 0.2
        hull.color.a = 1.0

        if self.mesh_resource:
            hull.type = Marker.MESH_RESOURCE
            hull.mesh_resource = self.mesh_resource
            hull.mesh_use_embedded_materials = False
            hull.scale.x = self.mesh_scale
            hull.scale.y = self.mesh_scale
            hull.scale.z = self.mesh_scale
        else:
            hull.type = Marker.CUBE
            hull.scale.x = self.hull_length_m
            hull.scale.y = self.hull_diameter_m
            hull.scale.z = self.hull_diameter_m

        self._marker_pub.publish(hull)


def main(args=None):
    rclpy.init(args=args)
    node = PoseVizNode()
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
