#!/usr/bin/env python3
"""Central watchdog that latches /safety/emergency on hard CAN faults, proximity alerts, and topic staleness."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from glider_msgs.msg import Float64Stamped


WATCHDOG_STALENESS_THRESHOLD_S = 1200.0  #testing value 20 min; production target TBD

WATCHDOG_TOPICS = [
    "/glider/roll_rad",
    "/glider/pitch_rad",
    "/glider/pitch_rate_rad_s",
    "/pressure/depth",
]


class SafetyNode(Node):
    """Monitors CAN faults, sonar proximity, and topic staleness; latches /safety/emergency (republished 2 Hz) with /safety/detail."""

    def __init__(self):
        super().__init__("safety_node")

        #parameters — edit values in config/glider_params.yaml; defaults here are fallbacks
        self.declare_parameter("proximity_threshold_m", 5.0) #this can change the sonar threshold
        self._proximity_threshold_m = float(
            self.get_parameter("proximity_threshold_m").value)

        #require N consecutive confident below-threshold readings before latching (each /glider/range sample is one confident read)
        self.declare_parameter("proximity_consecutive_count", 3)
        self._proximity_consecutive_required = int(
            self.get_parameter("proximity_consecutive_count").value)
        self._proximity_below_count = 0

        self.declare_parameter("staleness_threshold_s", WATCHDOG_STALENESS_THRESHOLD_S)
        self._staleness_threshold_s = float(
            self.get_parameter("staleness_threshold_s").value)

        #latch: once a hard fault fires we stay in emergency
        self._emergency_latched = False
        self._emergency_detail = ""

        #watchdog: track last received time per topic (None = never received)
        self._last_received: dict = {t: None for t in WATCHDOG_TOPICS}
        self._node_start_ns = self.get_clock().now().nanoseconds

        #publishers
        self._emergency_pub = self.create_publisher(Bool, "/safety/emergency", 10)
        self._emergency_detail_pub = self.create_publisher(String, "/safety/detail", 10)

        #fault topic subscriptions from the CAN bridge
        self.create_subscription(
            String, "/bridge/pr/fault", self._cb_pr_fault, 10)
        self.create_subscription(
            String, "/bridge/vbd_left/fault", self._cb_vbd_left_fault, 10)
        self.create_subscription(
            String, "/bridge/vbd_right/fault", self._cb_vbd_right_fault, 10)

        #sonar proximity check
        self.create_subscription(
            Float64Stamped, "/glider/range", self._cb_range, 10)

        #watchdog subscriptions
        for topic in WATCHDOG_TOPICS:
            self.create_subscription(
                Float64Stamped, topic,
                lambda msg, t=topic: self._cb_watchdog(t),
                20)

        #2 Hz republish timer — mirrors the Teensy fault-latch broadcast rate
        self.create_timer(0.5, self._republish_emergency)

        #staleness check timer — runs every 10 s (threshold defaults to WATCHDOG_STALENESS_THRESHOLD_S)
        self.create_timer(10.0, self._check_staleness)

        self.get_logger().info(
            f"Safety node started (proximity threshold: {self._proximity_threshold_m}m, "
            f"staleness threshold: {self._staleness_threshold_s}s)")

    #fault callbacks

    def _cb_pr_fault(self, msg: String):
        self._check_fault("P_R", msg.data)

    def _cb_vbd_left_fault(self, msg: String):
        self._check_fault("VBD_LEFT", msg.data)

    def _cb_vbd_right_fault(self, msg: String):
        self._check_fault("VBD_RIGHT", msg.data)

    def _cb_range(self, msg: Float64Stamped):
        range_m = msg.data
        if range_m < self._proximity_threshold_m:
            self._proximity_below_count += 1
            if self._proximity_below_count >= self._proximity_consecutive_required:
                detail = (
                    f"Proximity alert: obstacle {range_m:.2f}m away "
                    f"for {self._proximity_below_count} consecutive readings "
                    f"(threshold {self._proximity_threshold_m:.1f}m)"
                )
                self._trigger_emergency(detail)
        else:
            self._proximity_below_count = 0

    def _cb_watchdog(self, topic: str):
        self._last_received[topic] = self.get_clock().now().nanoseconds

    def _check_staleness(self):
        if self._emergency_latched:
            return

        now_ns = self.get_clock().now().nanoseconds

        for topic, last_ns in self._last_received.items():
            reference_ns = last_ns if last_ns is not None else self._node_start_ns
            elapsed_s = (now_ns - reference_ns) / 1e9

            if elapsed_s > self._staleness_threshold_s:
                detail = (
                    f"Watchdog: {topic} has not published for "
                    f"{elapsed_s:.0f}s (threshold {self._staleness_threshold_s:.0f}s)"
                )
                self._trigger_emergency(detail)
                return

    #core logic

    def _check_fault(self, source: str, fault_str: str):
        """Trigger emergency if the bridge reports a hard fault."""
        if not fault_str.startswith("HARD:"):
            return

        #extract fault names from "HARD:F1,F2|state:STATE"
        fault_part = fault_str.split("|")[0][len("HARD:"):]
        self._trigger_emergency(f"Hard fault on {source}: {fault_part}")

    def _trigger_emergency(self, detail: str):
        """Latch and publish emergency with the given detail string."""
        if self._emergency_latched:
            return

        self._emergency_latched = True
        self._emergency_detail = detail
        self.get_logger().error(f"SAFETY: {detail} — triggering emergency")

        emergency_msg = Bool()
        emergency_msg.data = True
        self._emergency_pub.publish(emergency_msg)

        detail_msg = String()
        detail_msg.data = detail
        self._emergency_detail_pub.publish(detail_msg)

    def _republish_emergency(self):
        """Republish emergency signal at 2 Hz while latched."""
        if not self._emergency_latched:
            return

        emergency_msg = Bool()
        emergency_msg.data = True
        self._emergency_pub.publish(emergency_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
