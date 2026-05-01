#!/usr/bin/env python3
"""PI control lifecycle node. Tracks pitch, roll and vbd setpoints from mission_node and drives actuators."""

import math
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float64, UInt8
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

        #parameters, defaults overridden by glider_params.yaml
        self.declare_parameter('Kp_theta', 1.5)
        self.declare_parameter('Ti_theta', 750.0)
        self.declare_parameter('Kp_q', 0.02)
        self.declare_parameter('Ti_q', 5.0)
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

        #sensor state
        self.theta = 0.0
        self.q = 0.0
        self.phi = 0.0
        self.p = 0.0

        self.pitch_setpoint = 0.0
        self.roll_setpoint = 0.0
        self.vbd_setpoint = 100.0

        self._sub_roll = None
        self._sub_roll_rate = None
        self._sub_pitch = None
        self._sub_pitch_rate = None
        self._sub_pitch_sp = None
        self._sub_roll_sp = None
        self._sub_vbd_sp = None
        self.pi_theta = None
        self.pi_q = None
        self.pi_phi = None
        self.pi_p = None
        self.filt_shift_cmd = None
        self.filt_roll_cmd = None
        self.pub_pitch_mm = None
        self.pub_roll_deg = None
        self.pub_vbd_left = None
        self.pub_vbd_right = None

        #set up in on_activate
        self._ctrl_timer = None
        self.dt = 0.1

        self.add_on_set_parameters_callback(self._on_param_change)

    #lifecycle callbacks

    def on_configure(self, state):
        self._load_params()

        self.pi_theta = PIController(
            self.Kp_theta, self.Ki_theta, -self.q_cmd_max, self.q_cmd_max)
        self.pi_q = PIController(
            self.Kp_q, self.Ki_q, self.shift_min_m, self.shift_max_m)
        self.pi_phi = PIController(
            self.Kp_phi, self.Ki_phi, -self.p_cmd_max, self.p_cmd_max)
        self.pi_p = PIController(
            self.Kp_p, self.Ki_p, -self.roll_max_rad, self.roll_max_rad)
        self.filt_shift_cmd = FirstOrderFilter(self.shift_cmd_tau, 0.0)
        self.filt_roll_cmd = FirstOrderFilter(self.roll_cmd_tau, 0.0)

        self._sub_roll = self.create_subscription(
            Float64Stamped, '/glider/roll_rad', self._cb_roll, 10)
        self._sub_pitch = self.create_subscription(
            Float64Stamped, '/glider/pitch_rad', self._cb_pitch, 10)
        self._sub_pitch_rate = self.create_subscription(
            Float64Stamped, '/glider/pitch_rate_rad_s', self._cb_pitch_rate, 10)
        self._sub_roll_rate = self.create_subscription(
            Float64Stamped, '/glider/roll_rate_rad_s', self._cb_roll_rate, 10)
        self._sub_pitch_sp = self.create_subscription(
            Float64, '/mission/pitch_setpoint', self._cb_pitch_setpoint, 10)
        self._sub_roll_sp = self.create_subscription(
            Float64, '/mission/roll_setpoint', self._cb_roll_setpoint, 10)
        self._sub_vbd_sp = self.create_subscription(
            Float64, '/mission/vbd_setpoint', self._cb_vbd_setpoint, 10)

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

        self.pitch_setpoint = 0.0
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
                     '_sub_pitch_sp', '_sub_roll_sp', '_sub_vbd_sp'):
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

    #load and update parameters

    def _load_params(self):
        p = self.get_parameter
        self.Kp_theta = p('Kp_theta').value
        self.Ki_theta = self.Kp_theta / p('Ti_theta').value
        self.Kp_q = p('Kp_q').value
        self.Ki_q = self.Kp_q / p('Ti_q').value
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

    def _on_param_change(self, params):
        for p in params:
            try:
                if p.name == 'Kp_theta':
                    new_kp = float(p.value)
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
                elif p.name == 'Kp_q':
                    new_kp = float(p.value)
                    if self.pi_q is not None:
                        self.pi_q.kp = new_kp
                        self.pi_q.ki = new_kp / float(self.get_parameter('Ti_q').value)
                elif p.name == 'Ti_q':
                    new_ti = float(p.value)
                    if new_ti == 0.0:
                        return SetParametersResult(
                            successful=False, reason='Ti_q must be non-zero')
                    if self.pi_q is not None:
                        self.pi_q.ki = float(self.get_parameter('Kp_q').value) / new_ti
                elif p.name == 'Kp_phi':
                    new_kp = float(p.value)
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
                    v = float(p.value); self.q_cmd_max = v
                    if self.pi_theta is not None:
                        self.pi_theta.out_min = -v
                        self.pi_theta.out_max = v
                elif p.name == 'p_cmd_max':
                    v = float(p.value); self.p_cmd_max = v
                    if self.pi_phi is not None:
                        self.pi_phi.out_min = -v
                        self.pi_phi.out_max = v
                elif p.name == 'shift_max_m':
                    v = float(p.value); self.shift_max_m = v
                    if self.pi_q is not None:
                        self.pi_q.out_max = v
                elif p.name == 'shift_min_m':
                    v = float(p.value); self.shift_min_m = v
                    if self.pi_q is not None:
                        self.pi_q.out_min = v
                elif p.name == 'roll_max_rad':
                    v = float(p.value); self.roll_max_rad = v
                    if self.pi_p is not None:
                        self.pi_p.out_min = -v
                        self.pi_p.out_max = v
                elif p.name == 'shift_cmd_tau':
                    v = float(p.value); self.shift_cmd_tau = v
                    if self.filt_shift_cmd is not None:
                        self.filt_shift_cmd.tau = max(v, 1e-6)
                elif p.name == 'roll_cmd_tau':
                    v = float(p.value); self.roll_cmd_tau = v
                    if self.filt_roll_cmd is not None:
                        self.filt_roll_cmd.tau = max(v, 1e-6)
                elif p.name == 'enable_roll':
                    self.enable_roll = bool(p.value)
            except (TypeError, ValueError) as e:
                return SetParametersResult(
                    successful=False, reason=f'bad value for {p.name}: {e}')
        return SetParametersResult(successful=True)

    #sensor and setpoint callbacks

    def _cb_roll(self, msg: Float64Stamped):
        self.phi = msg.data

    def _cb_pitch(self, msg: Float64Stamped):
        self.theta = msg.data

    def _cb_pitch_rate(self, msg: Float64Stamped):
        self.q = msg.data

    def _cb_roll_rate(self, msg: Float64Stamped):
        self.p = msg.data

    def _cb_pitch_setpoint(self, msg: Float64):
        self.pitch_setpoint = msg.data

    def _cb_roll_setpoint(self, msg: Float64):
        self.roll_setpoint = msg.data

    def _cb_vbd_setpoint(self, msg: Float64):
        self.vbd_setpoint = msg.data

    #publish helpers

    def _publish_safe_actuators(self):
        if self.pub_pitch_mm is None:
            return
        m = Float64(); m.data = 0.0;  self.pub_pitch_mm.publish(m)
        m = Float64(); m.data = 0.0;  self.pub_roll_deg.publish(m)
        m = UInt8();   m.data = 100;  self.pub_vbd_left.publish(m)
        m = UInt8();   m.data = 100;  self.pub_vbd_right.publish(m)

    #control loop runs at control_rate_hz when ACTIVE

    def _control_loop(self):
        #outer pitch loop
        alpha_err = self.pitch_setpoint - self.theta
        q_cmd = self.pi_theta.compute(alpha_err, self.dt)

        #inner pitch rate loop
        q_err = self.q - q_cmd
        shift_pi_out = self.pi_q.compute(q_err, self.dt)

        shift_smoothed = self.filt_shift_cmd.update(shift_pi_out, self.dt)
        shift_m = max(self.shift_min_m, min(self.shift_max_m, shift_smoothed))

        if self.enable_roll:
            #outer roll loop
            phi_err = self.roll_setpoint - self.phi
            p_cmd = self.pi_phi.compute(phi_err, self.dt)

            #inner roll rate loop
            p_err = self.p - p_cmd
            roll_pi_out = self.pi_p.compute(p_err, self.dt)

            roll_smoothed = self.filt_roll_cmd.update(roll_pi_out, self.dt)
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
