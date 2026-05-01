import threading
import time

import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Float64, String
from glider_msgs.msg import Float64Stamped

from glider_ros.mission.mission_node import MissionNode


def _spin_background(*nodes):
    executor = MultiThreadedExecutor()
    for n in nodes:
        executor.add_node(n)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    return executor, thread


def _wait_for(predicate, timeout=2.0, interval=0.02):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return False


@pytest.fixture()
def mission_node(rclpy_session):
    node = MissionNode()
    yield node
    node.destroy_node()


class TestMissionNodeStaticMode:

    def test_static_mode_ignores_force_surface(self, mission_node):
        mission_node.mode = 'static'
        mission_node._operating = False
        msg = Bool()
        msg.data = True
        mission_node._cb_force_surface(msg)
        assert mission_node._operating is False
        assert mission_node.diving is True

    def test_static_mode_param_setpoint_takes_effect(self, mission_node):
        result = mission_node._on_param_change(
            [Parameter('vbd_setpoint', Parameter.Type.DOUBLE, 47.5)])
        assert result.successful is True
        assert mission_node.vbd_setpoint_param == pytest.approx(47.5)

    def test_invalid_mode_rejected(self, mission_node):
        result = mission_node._on_param_change(
            [Parameter('mode', Parameter.Type.STRING, 'banana')])
        assert result.successful is False


class TestMissionNodeMissionMode:

    def test_mission_active_false_to_true_starts_supervisor(self, mission_node):
        mission_node.mode = 'mission'
        mission_node._operating = False
        result = mission_node._on_param_change(
            [Parameter('mission_active', Parameter.Type.BOOL, True)])
        assert result.successful is True
        assert mission_node._operating is True
        assert mission_node.diving is True

    def test_mission_active_true_to_false_stops_supervisor(self, mission_node):
        mission_node.mode = 'mission'
        mission_node._operating = True
        result = mission_node._on_param_change(
            [Parameter('mission_active', Parameter.Type.BOOL, False)])
        assert result.successful is True
        assert mission_node._operating is False

    def test_dive_to_climb_transition_at_depth_lower(self, mission_node):
        mission_node.mode = 'mission'
        mission_node._operating = True
        mission_node.diving = True
        mission_node.depth_lower = 5.0
        mission_node.depth = 5.5
        mission_node._publish_loop()
        assert mission_node.diving is False
        assert mission_node._operating is True

    def test_climb_to_complete_at_depth_upper(self, mission_node):
        mission_node.mode = 'mission'
        mission_node._operating = True
        mission_node.diving = False
        mission_node.depth_upper = 1.0
        mission_node.depth = 0.5
        mission_node._publish_loop()
        assert mission_node._operating is False

    def test_force_surface_in_mission_flips_diving_to_false(self, mission_node):
        mission_node.mode = 'mission'
        mission_node._operating = True
        mission_node.diving = True
        msg = Bool()
        msg.data = True
        mission_node._cb_force_surface(msg)
        assert mission_node.diving is False

    def test_force_surface_when_not_operating_is_ignored(self, mission_node):
        mission_node.mode = 'mission'
        mission_node._operating = False
        mission_node.diving = True
        msg = Bool()
        msg.data = True
        mission_node._cb_force_surface(msg)
        assert mission_node.diving is True

    def test_depth_callback_updates_depth(self, mission_node):
        msg = Float64Stamped()
        msg.data = 7.25
        mission_node._cb_depth(msg)
        assert mission_node.depth == pytest.approx(7.25)


class TestMissionNodePubSub:

    def test_static_mode_publishes_setpoints(self, rclpy_session):
        node = MissionNode()
        node.mode = 'static'
        node.pitch_setpoint_param = 0.123
        node.roll_setpoint_param = -0.05
        node.vbd_setpoint_param = 42.0

        helper = Node('test_mission_static_helper')
        received = {'pitch': [], 'roll': [], 'vbd': [], 'phase': []}
        helper.create_subscription(
            Float64, '/mission/pitch_setpoint',
            lambda m: received['pitch'].append(m.data), 10)
        helper.create_subscription(
            Float64, '/mission/roll_setpoint',
            lambda m: received['roll'].append(m.data), 10)
        helper.create_subscription(
            Float64, '/mission/vbd_setpoint',
            lambda m: received['vbd'].append(m.data), 10)
        helper.create_subscription(
            String, '/controller/phase',
            lambda m: received['phase'].append(m.data), 10)

        executor, thread = _spin_background(node, helper)
        try:
            assert _wait_for(lambda: len(received['pitch']) > 0, timeout=2.0)
            assert _wait_for(lambda: len(received['phase']) > 0, timeout=2.0)
            assert received['pitch'][-1] == pytest.approx(0.123)
            assert received['roll'][-1] == pytest.approx(-0.05)
            assert received['vbd'][-1] == pytest.approx(42.0)
            assert 'OPERATION' in received['phase']
        finally:
            executor.shutdown()
            thread.join(timeout=2.0)
            node.destroy_node()
            helper.destroy_node()

    def test_mission_mode_inactive_publishes_complete(self, rclpy_session):
        node = MissionNode()
        node.mode = 'mission'
        node._operating = False

        helper = Node('test_mission_complete_helper')
        received = []
        helper.create_subscription(
            String, '/controller/phase',
            lambda m: received.append(m.data), 10)

        executor, thread = _spin_background(node, helper)
        try:
            assert _wait_for(lambda: 'COMPLETE' in received, timeout=2.0), \
                'Timed out waiting for COMPLETE on /controller/phase'
        finally:
            executor.shutdown()
            thread.join(timeout=2.0)
            node.destroy_node()
            helper.destroy_node()
