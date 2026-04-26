#!/usr/bin/env python3
"""Sonar adapter that rejects low-confidence readings and moving-average filters altitude to /glider/range."""

from collections import deque
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import Range
from std_msgs.msg import Float32
from glider_msgs.msg import Float64Stamped


class SonarAdapterNode(Node):
    def __init__(self):
        super().__init__("sonar_adapter_node")

        #parameters — edit values in config/glider_params.yaml; defaults here are fallbacks
        self.declare_parameter("min_confidence", 50.0)  #percent; reject readings below this
        self.declare_parameter("filter_window", 5)
        self.declare_parameter("confidence_max_age_s", 0.5)  #reject stale confidence readings

        self._min_confidence = float(self.get_parameter("min_confidence").value)
        self._confidence_max_age_s = float(self.get_parameter("confidence_max_age_s").value)
        window = int(self.get_parameter("filter_window").value)

        self._range_buf: deque[float] = deque(maxlen=window)
        self._latest_confidence: float = 0.0
        self._confidence_stamp: Optional[Time] = None

        #subscriptions (raw driver output)
        self.create_subscription(Range, "/sonar/range", self._on_range, 10)
        self.create_subscription(Float32, "/sonar/confidence", self._on_confidence, 10)

        #publishers (clean output for the rest of the system)
        self._range_pub = self.create_publisher(Float64Stamped, "/glider/range", 10)

    #callbacks

    def _on_confidence(self, msg: Float32) -> None:
        self._latest_confidence = msg.data
        self._confidence_stamp = self.get_clock().now()

    def _on_range(self, msg: Range) -> None:
        #rejection paths clear the MA buffer to avoid averaging across gaps
        if self._confidence_stamp is None:
            self._range_buf.clear()
            return
        age_s = (self.get_clock().now() - self._confidence_stamp).nanoseconds * 1e-9
        if age_s > self._confidence_max_age_s:
            self._range_buf.clear()
            return

        if self._latest_confidence < self._min_confidence:
            self._range_buf.clear()
            return

        if msg.range < msg.min_range or msg.range > msg.max_range:
            self._range_buf.clear()
            return

        self._range_buf.append(msg.range)

        #preserve the sensor-capture stamp so downstream sees the true sample time
        out = Float64Stamped()
        out.header.stamp = msg.header.stamp
        out.data = sum(self._range_buf) / len(self._range_buf)
        self._range_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SonarAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
