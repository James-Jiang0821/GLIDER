"""
Bench tests for ControllerNode.

Tests are split into eight classes:
    TestPIController: Internal, PI math
    TestFirstOrderFilter: Internal, filter math
    TestControlLoopSteadyState: Internal, zero-error behaviour
    TestControlLoopSignConvention: Internal, error to actuator sign mapping
    TestControlLoopSaturation: Internal, output clamping
    TestControlLoopVbdPassthrough: Internal, VBD setpoint rounding/clamp
    TestControlLoopEnableRollFalse: Internal, roll disable
    TestSafeActuatorsOnDeactivate: Internal, safe-output on deactivate
"""
import math

import pytest

from glider_ros.controller.controller_node import (
    GliderController,
    PIController,
    FirstOrderFilter,
)


#-----------------------------------------------------------------------------
#Capturing publisher stub
#-----------------------------------------------------------------------------

class _CapturePub:
    """Minimal stand-in for an rclpy publisher; records msg.data on publish."""
    def __init__(self):
        self.values = []

    def publish(self, msg):
        self.values.append(msg.data)

    @property
    def last(self):
        return self.values[-1] if self.values else None


def _make_configured_controller():
    """Instantiate, configure, and swap publishers for capture stubs."""
    node = GliderController()
    node.on_configure(None)
    pubs = {
        'pitch_mm': _CapturePub(),
        'roll_deg': _CapturePub(),
        'vbd_left': _CapturePub(),
        'vbd_right': _CapturePub(),
    }
    node.pub_pitch_mm = pubs['pitch_mm']
    node.pub_roll_deg = pubs['roll_deg']
    node.pub_vbd_left = pubs['vbd_left']
    node.pub_vbd_right = pubs['vbd_right']
    #fixed dt so we don't depend on a wall-clock timer
    node.dt = 0.1
    return node, pubs


#-----------------------------------------------------------------------------
#PIController unit tests
#-----------------------------------------------------------------------------

class TestPIController:

    def test_proportional_only_no_integral(self):
        pi = PIController(kp=2.0, ki=0.0, out_min=-10.0, out_max=10.0)
        assert pi.compute(error=1.5, dt=0.1) == pytest.approx(3.0)
        #integral term disabled -> output should not drift between calls
        assert pi.compute(error=1.5, dt=0.1) == pytest.approx(3.0)

    def test_integral_accumulates_when_unsaturated(self):
        pi = PIController(kp=0.0, ki=1.0, out_min=-10.0, out_max=10.0)
        #pure I, error*dt=0.5 each step -> 0.5, 1.0, 1.5
        assert pi.compute(1.0, 0.5) == pytest.approx(0.5)
        assert pi.compute(1.0, 0.5) == pytest.approx(1.0)
        assert pi.compute(1.0, 0.5) == pytest.approx(1.5)

    def test_output_saturates_to_out_max(self):
        pi = PIController(kp=1.0, ki=0.0, out_min=-1.0, out_max=1.0)
        assert pi.compute(error=10.0, dt=0.1) == pytest.approx(1.0)

    def test_output_saturates_to_out_min(self):
        pi = PIController(kp=1.0, ki=0.0, out_min=-1.0, out_max=1.0)
        assert pi.compute(error=-10.0, dt=0.1) == pytest.approx(-1.0)

    def test_anti_windup_freezes_integral_when_pushing_into_saturation(self):
        pi = PIController(kp=1.0, ki=1.0, out_min=-1.0, out_max=1.0)
        for _ in range(5):
            pi.compute(error=10.0, dt=1.0)
        #integral must NOT have grown while clamped at out_max with same-sign error
        assert pi.integral == pytest.approx(0.0)

    def test_anti_windup_releases_when_error_reverses(self):
        pi = PIController(kp=1.0, ki=1.0, out_min=-1.0, out_max=1.0)
        #saturate first
        for _ in range(3):
            pi.compute(error=10.0, dt=1.0)
        assert pi.integral == pytest.approx(0.0)
        #now reverse sign: proposed = -0.5 + 1*(-0.5) = -1.0, exactly at out_min,
        #not strictly less, so integral DOES update
        pi.compute(error=-0.5, dt=1.0)
        assert pi.integral == pytest.approx(-0.5)


#-----------------------------------------------------------------------------
#FirstOrderFilter unit tests
#-----------------------------------------------------------------------------

class TestFirstOrderFilter:

    def test_zero_dt_returns_current_state(self):
        f = FirstOrderFilter(tau=1.0, initial=0.5)
        assert f.update(u=2.0, dt=0.0) == pytest.approx(0.5)

    def test_step_response_moves_toward_input(self):
        f = FirstOrderFilter(tau=1.0, initial=0.0)
        y1 = f.update(u=1.0, dt=0.1)   #alpha=0.1/1.1
        assert 0.0 < y1 < 1.0
        y2 = f.update(u=1.0, dt=0.1)
        assert y1 < y2 < 1.0

    def test_settles_for_long_horizon(self):
        f = FirstOrderFilter(tau=0.1, initial=0.0)
        for _ in range(200):
            f.update(u=1.0, dt=0.1)
        assert f.y == pytest.approx(1.0, abs=1e-6)

    def test_reset(self):
        f = FirstOrderFilter(tau=1.0, initial=0.5)
        f.update(u=2.0, dt=0.5)
        f.reset(0.0)
        assert f.y == 0.0


#-----------------------------------------------------------------------------
#Controller _control_loop: signs, saturation, pass-through
#-----------------------------------------------------------------------------

class TestControlLoopSteadyState:

    def test_zero_error_zero_state_yields_zero_outputs(self, rclpy_session):
        node, pubs = _make_configured_controller()
        try:
            node.theta = 0.0
            node.q = 0.0
            node.phi = 0.0
            node.p = 0.0
            node.pitch_setpoint = 0.0
            node.roll_setpoint = 0.0
            node.vbd_setpoint = 50.0

            node._control_loop()

            assert pubs['pitch_mm'].last == pytest.approx(0.0)
            assert pubs['roll_deg'].last == pytest.approx(0.0)
            assert pubs['vbd_left'].last == 50
            assert pubs['vbd_right'].last == 50
        finally:
            node.destroy_node()


class TestControlLoopSignConvention:
    """
    Pins the I/O sign mapping so we can compare against actuator behaviour
    in the tank. If any of these flip, the tank run will go the wrong way.

    With the YAML defaults and the IMU adapter convention
    (positive theta = nose up, positive phi = right-side down):
        positive pitch error  -> negative pitch_mm  (mass shifts aft)
        positive roll error   -> negative roll_deg

    These tests assert *signs* only, not magnitudes (gain-tuning lives in
    the YAML; magnitude assertions would be brittle).
    """

    def test_positive_pitch_error_drives_negative_pitch_mm(self, rclpy_session):
        node, pubs = _make_configured_controller()
        try:
            node.theta = 0.0
            node.q = 0.0
            node.pitch_setpoint = 0.05  #small enough to avoid q_cmd saturation
            node._control_loop()
            assert pubs['pitch_mm'].last < 0.0
        finally:
            node.destroy_node()

    def test_negative_pitch_error_drives_positive_pitch_mm(self, rclpy_session):
        node, pubs = _make_configured_controller()
        try:
            node.theta = 0.0
            node.q = 0.0
            node.pitch_setpoint = -0.05
            node._control_loop()
            assert pubs['pitch_mm'].last > 0.0
        finally:
            node.destroy_node()

    def test_positive_roll_error_drives_negative_roll_deg(self, rclpy_session):
        node, pubs = _make_configured_controller()
        try:
            node.phi = 0.0
            node.p = 0.0
            node.roll_setpoint = 0.05
            node._control_loop()
            assert pubs['roll_deg'].last < 0.0
        finally:
            node.destroy_node()

    def test_negative_roll_error_drives_positive_roll_deg(self, rclpy_session):
        node, pubs = _make_configured_controller()
        try:
            node.phi = 0.0
            node.p = 0.0
            node.roll_setpoint = -0.05
            node._control_loop()
            assert pubs['roll_deg'].last > 0.0
        finally:
            node.destroy_node()


class TestControlLoopSaturation:

    def test_huge_pitch_error_clamps_to_shift_max_mm(self, rclpy_session):
        node, pubs = _make_configured_controller()
        try:
            #drive q_err extreme by lying about q so inner-loop output saturates
            node.theta = 0.0
            node.pitch_setpoint = -10.0   #huge negative error -> negative q_cmd
            node.q = 10.0                 #q_err very positive -> shift_pi_out -> +max
            #run the filter long enough to settle (shift_cmd_tau=0.01s, dt=0.1)
            for _ in range(20):
                node._control_loop()
            shift_max_mm = node.shift_max_m * 1000.0
            assert pubs['pitch_mm'].last == pytest.approx(shift_max_mm, rel=1e-3)
        finally:
            node.destroy_node()

    def test_huge_pitch_error_clamps_to_shift_min_mm(self, rclpy_session):
        node, pubs = _make_configured_controller()
        try:
            node.theta = 0.0
            node.pitch_setpoint = 10.0
            node.q = -10.0
            for _ in range(20):
                node._control_loop()
            shift_min_mm = node.shift_min_m * 1000.0
            assert pubs['pitch_mm'].last == pytest.approx(shift_min_mm, rel=1e-3)
        finally:
            node.destroy_node()

    def test_roll_command_clamped_to_roll_max_deg(self, rclpy_session):
        node, pubs = _make_configured_controller()
        try:
            node.phi = 0.0
            node.roll_setpoint = -10.0    #huge negative error
            node.p = 10.0
            #roll filter tau=5s by default; cut it short for this test
            node.filt_roll_cmd.tau = 0.01
            for _ in range(50):
                node._control_loop()
            roll_max_deg = math.degrees(node.roll_max_rad)
            assert pubs['roll_deg'].last == pytest.approx(roll_max_deg, rel=1e-3)
        finally:
            node.destroy_node()


class TestControlLoopVbdPassthrough:

    def test_vbd_setpoint_rounded_to_nearest_pct(self, rclpy_session):
        node, pubs = _make_configured_controller()
        try:
            node.vbd_setpoint = 42.7
            node._control_loop()
            assert pubs['vbd_left'].last == 43
            assert pubs['vbd_right'].last == 43
        finally:
            node.destroy_node()

    def test_vbd_setpoint_clamped_low(self, rclpy_session):
        node, pubs = _make_configured_controller()
        try:
            node.vbd_setpoint = -5.0
            node._control_loop()
            assert pubs['vbd_left'].last == 0
            assert pubs['vbd_right'].last == 0
        finally:
            node.destroy_node()

    def test_vbd_setpoint_clamped_high(self, rclpy_session):
        node, pubs = _make_configured_controller()
        try:
            node.vbd_setpoint = 200.0
            node._control_loop()
            assert pubs['vbd_left'].last == 100
            assert pubs['vbd_right'].last == 100
        finally:
            node.destroy_node()

    def test_left_and_right_vbd_match(self, rclpy_session):
        node, pubs = _make_configured_controller()
        try:
            node.vbd_setpoint = 33.0
            node._control_loop()
            assert pubs['vbd_left'].last == pubs['vbd_right'].last
        finally:
            node.destroy_node()


class TestControlLoopEnableRollFalse:

    def test_enable_roll_false_zeros_roll_output(self, rclpy_session):
        node, pubs = _make_configured_controller()
        try:
            node.enable_roll = False
            node.phi = 0.0
            node.p = 0.0
            node.roll_setpoint = 1.0   #large error, but should be ignored
            node._control_loop()
            assert pubs['roll_deg'].last == pytest.approx(0.0)
        finally:
            node.destroy_node()


#-----------------------------------------------------------------------------
#Lifecycle: safe-actuator publish on deactivate
#-----------------------------------------------------------------------------

class TestSafeActuatorsOnDeactivate:
    """
    On deactivate the controller must publish a known safe state:
    zero pitch shift, zero roll, full-buoyancy VBD (100%) so the glider
    surfaces.
    """

    def test_publish_safe_actuators_outputs(self, rclpy_session):
        node, pubs = _make_configured_controller()
        try:
            node._publish_safe_actuators()
            assert pubs['pitch_mm'].values == [0.0]
            assert pubs['roll_deg'].values == [0.0]
            assert pubs['vbd_left'].values == [100]
            assert pubs['vbd_right'].values == [100]
        finally:
            node.destroy_node()
