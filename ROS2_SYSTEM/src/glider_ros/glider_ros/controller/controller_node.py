#!/usr/bin/env python3
"""Cascaded PI lifecycle controller. Outer alpha (or theta) loop, inner rate loop, drives shift/roll-mass/VBD."""

import math
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float64, UInt8
from sensor_msgs.msg import Imu
from glider_msgs.msg import Float64Stamped


class PIController:
    def __init__(self, kp, ki, out_min, out_max):
        self.kp = kp
        self.ki = ki
        self.out_min = out_min
        self.out_max = out_max
        self.integral = 0.0

    def reset(self):
        self.integral = 0.0

    def compute(self, error, dt):
        p_term = self.kp * error
        proposed = p_term + self.ki * (self.integral + error * dt)
        output = max(self.out_min, min(self.out_max, proposed))
        if not ((proposed > self.out_max and error > 0) or
                (proposed < self.out_min and error < 0)):
            self.integral += error * dt
        return output

    def compute_backcalc(self, error, dt, Tt):
        unsat = self.kp * error + self.ki * (self.integral + error * dt)
        sat = max(self.out_min, min(self.out_max, unsat))
        if Tt > 1e-6:
            self.integral += (error + (sat - unsat) / Tt) * dt
        else:
            self.integral += error * dt
        return sat


class FirstOrderFilter:
    def __init__(self, tau, initial=0.0):
        self.tau = max(tau, 1e-6)
        self.y = initial

    def update(self, u, dt):
        if dt <= 0:
            return self.y
        alpha = dt / (self.tau + dt)
        self.y += alpha * (u - self.y)
        return self.y

    def reset(self, value=0.0):
        self.y = value


class GliderController(LifecycleNode):

    def __init__(self):
        super().__init__('controller_node')

        # parameters, defaults overridden by glider_params.yaml
        self.declare_parameter('Kp_theta', 1.5)
        self.declare_parameter('Ti_theta', 750.0)
        self.declare_parameter('Td_theta', 20.0)
        self.declare_parameter('Kp_q', 0.3)
        self.declare_parameter('Ti_q', 3.0)
        self.declare_parameter('Tt_q', 0.0)
        self.declare_parameter('Kp_phi', 0.3)
        self.declare_parameter('Ti_phi', 15.0)
        self.declare_parameter('Kp_p', 4.0)
        self.declare_parameter('Ti_p', 20000.0)
        self.declare_parameter('q_cmd_max', math.radians(8))
        self.declare_parameter('p_cmd_max', math.radians(5))
        self.declare_parameter('shift_max_m', 0.0569)
        self.declare_parameter('shift_min_m', -0.0569)
        self.declare_parameter('roll_max_rad', math.pi / 2)
        self.declare_parameter('shift_cmd_tau', 0.01)
        self.declare_parameter('roll_cmd_tau', 5.0)
        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('enable_roll', True)
        self.declare_parameter('accel_bias_x', 0.0)
        self.declare_parameter('depth_rate_tau', 1.0)
        self.declare_parameter('imu_topic', '/imu/data_raw')
        self.declare_parameter('use_alpha_feedback', True)

        # sensor state
        self.theta = 0.0
        self.q = 0.0
        self.phi = 0.0
        self.p = 0.0
        self.depth = 0.0

        # alpha-estimator state
        self.V_x_body = 0.0
        self._imu_t_prev = None
        self._depth_prev = None
        self._depth_t_prev = None

        # mission setpoints
        self.alpha_setpoint = 0.0
        self.roll_setpoint = 0.0
        self.vbd_setpoint = 100.0

        self._sub_roll = None
        self._sub_roll_rate = None
        self._sub_pitch = None
        self._sub_pitch_rate = None
        self._sub_alpha_sp = None
        self._sub_roll_sp = None
        self._sub_vbd_sp = None
        self._sub_imu = None
        self._sub_depth = None
        self.pi_theta = None
        self.pi_q = None
        self.pi_phi = None
        self.pi_p = None
        self.filt_shift_cmd = None
        self.filt_roll_cmd = None
        self.filt_ddot = None
        self.pub_pitch_mm = None
        self.pub_roll_deg = None
        self.pub_vbd_left = None
        self.pub_vbd_right = None

        # set up in on_activate
        self._ctrl_timer = None
        self.dt = 0.1

        self.add_on_set_parameters_callback(self._on_param_change)

    # lifecycle callbacks

    def on_configure(self, state):
        self._load_params()

        #plain PI for outer pitch, outer roll, inner roll. Saturation applied externally.
        self.pi_theta = PIController(
            self.Kp_theta, self.Ki_theta, -float('inf'), float('inf'))
        self.pi_phi = PIController(
            self.Kp_phi, self.Ki_phi, -float('inf'), float('inf'))
        self.pi_p = PIController(
            self.Kp_p, self.Ki_p, -float('inf'), float('inf'))

        #inner pitch saturates internally so back-calc AW can read the limits
        self.pi_q = PIController(
            self.Kp_q, self.Ki_q, self.shift_min_m, self.shift_max_m)

        self.filt_shift_cmd = FirstOrderFilter(self.shift_cmd_tau, 0.0)
        self.filt_roll_cmd = FirstOrderFilter(self.roll_cmd_tau, 0.0)
        self.filt_ddot = FirstOrderFilter(self.depth_rate_tau, 0.0)

        self._sub_roll = self.create_subscription(
            Float64Stamped, '/glider/roll_rad', self._cb_roll, 10)
        self._sub_pitch = self.create_subscription(
            Float64Stamped, '/glider/pitch_rad', self._cb_pitch, 10)
        self._sub_pitch_rate = self.create_subscription(
            Float64Stamped, '/glider/pitch_rate_rad_s', self._cb_pitch_rate, 10)
        self._sub_roll_rate = self.create_subscription(
            Float64Stamped, '/glider/roll_rate_rad_s', self._cb_roll_rate, 10)
        self._sub_alpha_sp = self.create_subscription(
            Float64, '/mission/alpha_setpoint', self._cb_alpha_setpoint, 10)
        self._sub_roll_sp = self.create_subscription(
            Float64, '/mission/roll_setpoint', self._cb_roll_setpoint, 10)
        self._sub_vbd_sp = self.create_subscription(
            Float64, '/mission/vbd_setpoint', self._cb_vbd_setpoint, 10)
        self._sub_imu = self.create_subscription(
            Imu, self.imu_topic, self._cb_imu, 50)
        self._sub_depth = self.create_subscription(
            Float64Stamped, '/pressure/depth', self._cb_depth, 10)

        self.pub_pitch_mm = self.create_publisher(Float64, '/controller/pitch_mm', 10)
        self.pub_roll_deg = self.create_publisher(Float64, '/controller/roll_deg', 10)
        self.pub_vbd_left = self.create_publisher(UInt8, '/controller/vbd_left_pct', 10)
        self.pub_vbd_right = self.create_publisher(UInt8, '/controller/vbd_right_pct', 10)

        self.get_logger().info('Controller configured')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self._load_params()

        rate = self.get_parameter('control_rate_hz').value
        self.dt = 1.0 / rate

        self.pi_theta.reset()
        self.pi_q.reset()
        self.pi_phi.reset()
        self.pi_p.reset()
        self.filt_shift_cmd.reset(0.0)
        self.filt_roll_cmd.reset(0.0)
        self.filt_ddot.reset(0.0)

        self.V_x_body = 0.0
        self._imu_t_prev = None
        self._depth_prev = None
        self._depth_t_prev = None

        self.alpha_setpoint = 0.0
        self.roll_setpoint = 0.0
        self.vbd_setpoint = 100.0

        self._ctrl_timer = self.create_timer(self.dt, self._control_loop)

        self.get_logger().info('Controller activated')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        if self._ctrl_timer is not None:
            self.destroy_timer(self._ctrl_timer)
            self._ctrl_timer = None
        self._publish_safe_actuators()
        self.get_logger().info('Controller deactivated')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        for attr in ('_sub_roll', '_sub_roll_rate', '_sub_pitch', '_sub_pitch_rate',
                     '_sub_alpha_sp', '_sub_roll_sp', '_sub_vbd_sp',
                     '_sub_imu', '_sub_depth'):
            sub = getattr(self, attr, None)
            if sub:
                self.destroy_subscription(sub)
                setattr(self, attr, None)
        for attr in ('pub_pitch_mm', 'pub_roll_deg', 'pub_vbd_left', 'pub_vbd_right'):
            pub = getattr(self, attr, None)
            if pub:
                self.destroy_publisher(pub)
                setattr(self, attr, None)
        self.get_logger().info('Controller cleaned up')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        return TransitionCallbackReturn.SUCCESS

    # load and update parameters

    def _load_params(self):
        p = self.get_parameter
        self.Kp_theta = p('Kp_theta').value
        self.Ki_theta = self.Kp_theta / p('Ti_theta').value
        self.Td_theta = p('Td_theta').value
        self.Kp_q = p('Kp_q').value
        self.Ti_q = p('Ti_q').value
        self.Ki_q = self.Kp_q / self.Ti_q
        self.Tt_q = p('Tt_q').value
        self.Kp_phi = p('Kp_phi').value
        self.Ki_phi = self.Kp_phi / p('Ti_phi').value
        self.Kp_p = p('Kp_p').value
        self.Ki_p = self.Kp_p / p('Ti_p').value
        self.q_cmd_max = p('q_cmd_max').value
        self.p_cmd_max = p('p_cmd_max').value
        self.shift_max_m = p('shift_max_m').value
        self.shift_min_m = p('shift_min_m').value
        self.roll_max_rad = p('roll_max_rad').value
        self.shift_cmd_tau = p('shift_cmd_tau').value
        self.roll_cmd_tau = p('roll_cmd_tau').value
        self.enable_roll = p('enable_roll').value
        self.accel_bias_x = p('accel_bias_x').value
        self.depth_rate_tau = p('depth_rate_tau').value
        self.imu_topic = p('imu_topic').value
        self.use_alpha_feedback = p('use_alpha_feedback').value

    def _on_param_change(self, params):
        for p in params:
            try:
                if p.name == 'Kp_theta':
                    new_kp = float(p.value)
                    self.Kp_theta = new_kp
                    if self.pi_theta is not None:
                        self.pi_theta.kp = new_kp
                        self.pi_theta.ki = new_kp / float(self.get_parameter('Ti_theta').value)
                elif p.name == 'Ti_theta':
                    new_ti = float(p.value)
                    if new_ti == 0.0:
                        return SetParametersResult(
                            successful=False, reason='Ti_theta must be non-zero')
                    if self.pi_theta is not None:
                        self.pi_theta.ki = float(self.get_parameter('Kp_theta').value) / new_ti
                elif p.name == 'Td_theta':
                    self.Td_theta = float(p.value)
                elif p.name == 'Kp_q':
                    new_kp = float(p.value)
                    self.Kp_q = new_kp
                    if self.pi_q is not None:
                        self.pi_q.kp = new_kp
                        self.pi_q.ki = new_kp / float(self.get_parameter('Ti_q').value)
                elif p.name == 'Ti_q':
                    new_ti = float(p.value)
                    if new_ti == 0.0:
                        return SetParametersResult(
                            successful=False, reason='Ti_q must be non-zero')
                    self.Ti_q = new_ti
                    if self.pi_q is not None:
                        self.pi_q.ki = float(self.get_parameter('Kp_q').value) / new_ti
                elif p.name == 'Tt_q':
                    self.Tt_q = float(p.value)
                elif p.name == 'Kp_phi':
                    new_kp = float(p.value)
                    self.Kp_phi = new_kp
                    if self.pi_phi is not None:
                        self.pi_phi.kp = new_kp
                        self.pi_phi.ki = new_kp / float(self.get_parameter('Ti_phi').value)
                elif p.name == 'Ti_phi':
                    new_ti = float(p.value)
                    if new_ti == 0.0:
                        return SetParametersResult(
                            successful=False, reason='Ti_phi must be non-zero')
                    if self.pi_phi is not None:
                        self.pi_phi.ki = float(self.get_parameter('Kp_phi').value) / new_ti
                elif p.name == 'Kp_p':
                    new_kp = float(p.value)
                    self.Kp_p = new_kp
                    if self.pi_p is not None:
                        self.pi_p.kp = new_kp
                        self.pi_p.ki = new_kp / float(self.get_parameter('Ti_p').value)
                elif p.name == 'Ti_p':
                    new_ti = float(p.value)
                    if new_ti == 0.0:
                        return SetParametersResult(
                            successful=False, reason='Ti_p must be non-zero')
                    if self.pi_p is not None:
                        self.pi_p.ki = float(self.get_parameter('Kp_p').value) / new_ti
                elif p.name == 'q_cmd_max':
                    self.q_cmd_max = float(p.value)
                elif p.name == 'p_cmd_max':
                    self.p_cmd_max = float(p.value)
                elif p.name == 'shift_max_m':
                    v = float(p.value); self.shift_max_m = v
                    if self.pi_q is not None:
                        self.pi_q.out_max = v
                elif p.name == 'shift_min_m':
                    v = float(p.value); self.shift_min_m = v
                    if self.pi_q is not None:
                        self.pi_q.out_min = v
                elif p.name == 'roll_max_rad':
                    self.roll_max_rad = float(p.value)
                elif p.name == 'shift_cmd_tau':
                    v = float(p.value); self.shift_cmd_tau = v
                    if self.filt_shift_cmd is not None:
                        self.filt_shift_cmd.tau = max(v, 1e-6)
                elif p.name == 'roll_cmd_tau':
                    v = float(p.value); self.roll_cmd_tau = v
                    if self.filt_roll_cmd is not None:
                        self.filt_roll_cmd.tau = max(v, 1e-6)
                elif p.name == 'depth_rate_tau':
                    v = float(p.value); self.depth_rate_tau = v
                    if self.filt_ddot is not None:
                        self.filt_ddot.tau = max(v, 1e-6)
                elif p.name == 'accel_bias_x':
                    self.accel_bias_x = float(p.value)
                elif p.name == 'use_alpha_feedback':
                    new_mode = bool(p.value)
                    if new_mode != self.use_alpha_feedback and self.pi_theta is not None:
                        self.pi_theta.reset()
                    self.use_alpha_feedback = new_mode
                elif p.name == 'enable_roll':
                    self.enable_roll = bool(p.value)
            except (TypeError, ValueError) as e:
                return SetParametersResult(
                    successful=False, reason=f'bad value for {p.name}: {e}')
        return SetParametersResult(successful=True)

    # sensor and setpoint callbacks

    def _cb_roll(self, msg: Float64Stamped):
        self.phi = msg.data

    def _cb_pitch(self, msg: Float64Stamped):
        self.theta = msg.data

    def _cb_pitch_rate(self, msg: Float64Stamped):
        self.q = msg.data

    def _cb_roll_rate(self, msg: Float64Stamped):
        self.p = msg.data

    def _cb_alpha_setpoint(self, msg: Float64):
        self.alpha_setpoint = msg.data

    def _cb_roll_setpoint(self, msg: Float64):
        self.roll_setpoint = msg.data

    def _cb_vbd_setpoint(self, msg: Float64):
        self.vbd_setpoint = msg.data

    def _cb_imu(self, msg: Imu):
        t_now = self.get_clock().now().nanoseconds * 1e-9
        a_x = msg.linear_acceleration.x
        if self._imu_t_prev is not None:
            dt = t_now - self._imu_t_prev
            if dt > 0:
                g_body_x = -9.81 * math.sin(self.theta)
                a_x_lin = a_x - g_body_x - self.accel_bias_x
                self.V_x_body += a_x_lin * dt
        self._imu_t_prev = t_now

    def _cb_depth(self, msg: Float64Stamped):
        t_now = self.get_clock().now().nanoseconds * 1e-9
        d_now = msg.data
        if self._depth_prev is not None and self._depth_t_prev is not None:
            dt = t_now - self._depth_t_prev
            if dt > 0:
                ddot_raw = (d_now - self._depth_prev) / dt
                self.filt_ddot.update(ddot_raw, dt)
        self._depth_prev = d_now
        self._depth_t_prev = t_now
        self.depth = d_now

    # publish helpers

    def _publish_safe_actuators(self):
        if self.pub_pitch_mm is None:
            return
        m = Float64(); m.data = 0.0;  self.pub_pitch_mm.publish(m)
        m = Float64(); m.data = 0.0;  self.pub_roll_deg.publish(m)
        m = UInt8();   m.data = 100;  self.pub_vbd_left.publish(m)
        m = UInt8();   m.data = 100;  self.pub_vbd_right.publish(m)

    # control loop runs at control_rate_hz when ACTIVE

    def _control_loop(self):
        if self.use_alpha_feedback:
            V_h_est = self.V_x_body * math.cos(self.theta)
            gamma_est = math.atan2(-self.filt_ddot.y, V_h_est)
            alpha_est = self.theta - gamma_est
            e_outer = self.alpha_setpoint - alpha_est
        else:
            e_outer = self.alpha_setpoint - self.theta

        y_pi1 = self.pi_theta.compute(e_outer, self.dt)
        y_outer = y_pi1 - self.Kp_theta * self.Td_theta * self.q
        q_cmd = max(-self.q_cmd_max, min(self.q_cmd_max, y_outer))

        q_err = self.q - q_cmd
        Tt_eff = self.Tt_q if self.Tt_q > 1e-6 else self.Ti_q
        shift_pi_out = self.pi_q.compute_backcalc(q_err, self.dt, Tt_eff)

        shift_smoothed = self.filt_shift_cmd.update(shift_pi_out, self.dt)
        shift_m = max(self.shift_min_m, min(self.shift_max_m, shift_smoothed))

        if self.enable_roll:
            phi_err = self.roll_setpoint - self.phi
            y_pi_phi = self.pi_phi.compute(phi_err, self.dt)
            p_cmd = max(-self.p_cmd_max, min(self.p_cmd_max, y_pi_phi))

            p_err = self.p - p_cmd
            roll_pi_out = self.pi_p.compute(p_err, self.dt)
            roll_pi_sat = max(-self.roll_max_rad, min(self.roll_max_rad, roll_pi_out))

            roll_smoothed = self.filt_roll_cmd.update(roll_pi_sat, self.dt)
            roll_rad = max(-self.roll_max_rad, min(self.roll_max_rad, roll_smoothed))
        else:
            roll_rad = 0.0

        shift_mm = shift_m * 1000.0
        roll_deg = math.degrees(roll_rad)
        vbd_pct = max(0, min(100, int(round(self.vbd_setpoint))))

        m = Float64(); m.data = shift_mm; self.pub_pitch_mm.publish(m)
        m = Float64(); m.data = roll_deg; self.pub_roll_deg.publish(m)
        m = UInt8(); m.data = vbd_pct; self.pub_vbd_left.publish(m)
        m = UInt8(); m.data = vbd_pct; self.pub_vbd_right.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = GliderController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
