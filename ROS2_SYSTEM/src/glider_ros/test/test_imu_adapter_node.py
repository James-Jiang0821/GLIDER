"""
Bench tests for ImuAdapterNode.

Assumes IMU mounting: X=body-right, Y=body-forward, Z=body-up.

Tests are split into four classes:
    TestStaticOrientation: Internal, gravity vector to roll/pitch
    TestMountOffsets: Internal, pitch/roll offset subtraction
    TestAngularRateMapping: Internal, gyro axis remap to body rates
    TestStaticGyroSignCoherence: Internal, gyro sign matches orientation convention
"""
import math

import pytest
from sensor_msgs.msg import Imu

from glider_ros.adapter.imu_adapter_node import ImuAdapterNode


G = 9.81


#-----------------------------------------------------------------------------
#Helpers
#-----------------------------------------------------------------------------

class _CapturePub:
    def __init__(self):
        self.values = []

    def publish(self, msg):
        self.values.append(msg.data)

    @property
    def last(self):
        return self.values[-1] if self.values else None


def _make_adapter():
    node = ImuAdapterNode()
    node._mount_calibrated = True
    node.mount_pitch_offset_rad = 0.0
    node.mount_roll_offset_rad = 0.0
    pubs = {
        '_roll_pub': _CapturePub(),
        '_pitch_pub': _CapturePub(),
        '_roll_deg_pub': _CapturePub(),
        '_pitch_deg_pub': _CapturePub(),
        '_pitch_rate_pub': _CapturePub(),
        '_roll_rate_pub': _CapturePub(),
    }
    for name, stub in pubs.items():
        setattr(node, name, stub)
    return node, pubs


def _imu_msg(ax=0.0, ay=0.0, az=G, gx=0.0, gy=0.0, gz=0.0):
    msg = Imu()
    msg.linear_acceleration.x = float(ax)
    msg.linear_acceleration.y = float(ay)
    msg.linear_acceleration.z = float(az)
    msg.angular_velocity.x = float(gx)
    msg.angular_velocity.y = float(gy)
    msg.angular_velocity.z = float(gz)
    return msg


#-----------------------------------------------------------------------------
#Static orientation: gravity vector -> roll, pitch
#-----------------------------------------------------------------------------

class TestStaticOrientation:

    def test_level_yields_zero_roll_and_pitch(self, rclpy_session):
        node, pubs = _make_adapter()
        try:
            node._on_imu(_imu_msg(ax=0.0, ay=0.0, az=G))
            assert pubs['_roll_pub'].last == pytest.approx(0.0, abs=1e-9)
            assert pubs['_pitch_pub'].last == pytest.approx(0.0, abs=1e-9)
        finally:
            node.destroy_node()

    @pytest.mark.parametrize('deg', [10.0, 30.0, 60.0])
    def test_right_side_down_yields_positive_roll(self, rclpy_session, deg):
        """Stationary, body rolled right-side-down by `deg`."""
        node, pubs = _make_adapter()
        try:
            theta = math.radians(deg)
            ax = -G * math.sin(theta)
            az = G * math.cos(theta)
            node._on_imu(_imu_msg(ax=ax, ay=0.0, az=az))
            assert pubs['_roll_pub'].last == pytest.approx(theta, abs=1e-6)
            assert pubs['_pitch_pub'].last == pytest.approx(0.0, abs=1e-6)
            assert pubs['_roll_deg_pub'].last == pytest.approx(deg, abs=1e-4)
        finally:
            node.destroy_node()

    @pytest.mark.parametrize('deg', [10.0, 30.0, 60.0])
    def test_left_side_down_yields_negative_roll(self, rclpy_session, deg):
        node, pubs = _make_adapter()
        try:
            theta = math.radians(deg)
            ax = G * math.sin(theta)
            az = G * math.cos(theta)
            node._on_imu(_imu_msg(ax=ax, ay=0.0, az=az))
            assert pubs['_roll_pub'].last == pytest.approx(-theta, abs=1e-6)
            assert pubs['_pitch_pub'].last == pytest.approx(0.0, abs=1e-6)
        finally:
            node.destroy_node()

    @pytest.mark.parametrize('deg', [10.0, 30.0, 60.0])
    def test_nose_up_yields_positive_pitch(self, rclpy_session, deg):
        """Stationary, body pitched nose-up by `deg`."""
        node, pubs = _make_adapter()
        try:
            theta = math.radians(deg)
            ay = -G * math.sin(theta)
            az = G * math.cos(theta)
            node._on_imu(_imu_msg(ax=0.0, ay=ay, az=az))
            assert pubs['_pitch_pub'].last == pytest.approx(theta, abs=1e-6)
            assert pubs['_roll_pub'].last == pytest.approx(0.0, abs=1e-6)
            assert pubs['_pitch_deg_pub'].last == pytest.approx(deg, abs=1e-4)
        finally:
            node.destroy_node()

    @pytest.mark.parametrize('deg', [10.0, 30.0, 60.0])
    def test_nose_down_yields_negative_pitch(self, rclpy_session, deg):
        node, pubs = _make_adapter()
        try:
            theta = math.radians(deg)
            ay = G * math.sin(theta)
            az = G * math.cos(theta)
            node._on_imu(_imu_msg(ax=0.0, ay=ay, az=az))
            assert pubs['_pitch_pub'].last == pytest.approx(-theta, abs=1e-6)
            assert pubs['_roll_pub'].last == pytest.approx(0.0, abs=1e-6)
        finally:
            node.destroy_node()


#-----------------------------------------------------------------------------
#Mount-offset subtraction
#-----------------------------------------------------------------------------

class TestMountOffsets:
    """
    Mount offsets are the user-tunable knob that compensates for small
    physical misalignments. Big rotations (like 90 deg) cannot be fixed
    with this offset alone -- those need an axis remap in _on_imu.
    """

    def test_pitch_offset_subtracted_from_level_reading(self, rclpy_session):
        node, pubs = _make_adapter()
        try:
            node.mount_pitch_offset_rad = 0.10
            node._on_imu(_imu_msg(ax=0.0, ay=0.0, az=G))
            assert pubs['_pitch_pub'].last == pytest.approx(-0.10, abs=1e-9)
        finally:
            node.destroy_node()

    def test_roll_offset_subtracted_from_level_reading(self, rclpy_session):
        node, pubs = _make_adapter()
        try:
            node.mount_roll_offset_rad = -0.05
            node._on_imu(_imu_msg(ax=0.0, ay=0.0, az=G))
            assert pubs['_roll_pub'].last == pytest.approx(0.05, abs=1e-9)
        finally:
            node.destroy_node()


#-----------------------------------------------------------------------------
#Gyro -> body rates
#-----------------------------------------------------------------------------

class TestAngularRateMapping:
    """
    Per the adapter:  pitch_rate = -gx,  roll_rate = +gy.
    These signs MUST match the static-tilt sign convention so the inner
    rate loop reduces error rather than amplifies it. If you flip gx or gy
    in hardware, the controller will go unstable.
    """

    def test_pitch_rate_is_negative_gx(self, rclpy_session):
        node, pubs = _make_adapter()
        try:
            node._on_imu(_imu_msg(gx=0.5))
            assert pubs['_pitch_rate_pub'].last == pytest.approx(-0.5)
            node._on_imu(_imu_msg(gx=-0.3))
            assert pubs['_pitch_rate_pub'].last == pytest.approx(0.3)
        finally:
            node.destroy_node()

    def test_roll_rate_is_gy(self, rclpy_session):
        node, pubs = _make_adapter()
        try:
            node._on_imu(_imu_msg(gy=0.7))
            assert pubs['_roll_rate_pub'].last == pytest.approx(0.7)
            node._on_imu(_imu_msg(gy=-0.4))
            assert pubs['_roll_rate_pub'].last == pytest.approx(-0.4)
        finally:
            node.destroy_node()

    def test_yaw_rate_does_not_leak_into_roll_or_pitch_rate(self, rclpy_session):
        """gz (yaw) should not affect roll_rate or pitch_rate outputs."""
        node, pubs = _make_adapter()
        try:
            node._on_imu(_imu_msg(gx=0.0, gy=0.0, gz=1.0))
            assert pubs['_pitch_rate_pub'].last == pytest.approx(0.0)
            assert pubs['_roll_rate_pub'].last == pytest.approx(0.0)
        finally:
            node.destroy_node()


#-----------------------------------------------------------------------------
#Sign coherence: static tilt and matching gyro must agree in sign
#-----------------------------------------------------------------------------

class TestStaticGyroSignCoherence:
    """
    If the gyro disagrees with the static reading (e.g. tilting nose-up
    produces NEGATIVE pitch_rate while static pitch shows POSITIVE), the
    inner rate loop will fight the outer angle loop -- a common failure
    mode after re-mounting an IMU. These tests don't simulate motion, but
    they make the convention explicit so a bench rotation test can be
    checked against it.
    """

    def test_nose_up_motion_should_produce_positive_pitch_rate(self, rclpy_session):
        """
        Convention: rotating the body nose-up means the body Y axis (forward)
        rotates toward the body Z axis (up). With X = right, that's a
        rotation about -X by a positive angle, i.e. gx is negative.
        Then pitch_rate = -gx > 0, agreeing with rising pitch angle.
        """
        node, pubs = _make_adapter()
        try:
            #gx negative <=> nose pitching up
            node._on_imu(_imu_msg(gx=-0.2))
            assert pubs['_pitch_rate_pub'].last > 0.0
        finally:
            node.destroy_node()

    def test_right_side_down_motion_should_produce_positive_roll_rate(self, rclpy_session):
        """
        Convention: rolling right-side-down is rotation about +Y (forward)
        with positive sign (right-hand rule: thumb forward, fingers go
        from +Z down toward +X). So gy > 0 corresponds to right-side-down.
        Then roll_rate = +gy > 0, agreeing with rising roll angle.
        """
        node, pubs = _make_adapter()
        try:
            node._on_imu(_imu_msg(gy=0.2))
            assert pubs['_roll_rate_pub'].last > 0.0
        finally:
            node.destroy_node()
