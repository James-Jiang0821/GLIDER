#!/usr/bin/env python3
"""Publishes pitch, roll and vbd setpoints. Modes: static (from yaml) or mission (depth supervisor)."""

import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Bool, Float64, String
from glider_msgs.msg import Float64Stamped


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


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')

        #parameters, defaults overridden by glider_params.yaml
        self.declare_parameter('mode', 'mission')
        self.declare_parameter('mission_active', False)

        #static-mode setpoints
        self.declare_parameter('pitch_setpoint', 0.0)
        self.declare_parameter('roll_setpoint', 0.0)
        self.declare_parameter('vbd_setpoint', 50.0)

        #mission-mode supervisor parameters
        self.declare_parameter('depth_upper', 1.0)
        self.declare_parameter('depth_lower', 5.0)
        self.declare_parameter('alpha_dive', math.radians(5))
        self.declare_parameter('alpha_rise', math.radians(-5))
        self.declare_parameter('T_alpha_cmd', 2.0)
        self.declare_parameter('T_vbd_cmd', 5.0)
        self.declare_parameter('publish_rate_hz', 10.0)

        self.mode = str(self.get_parameter('mode').value)
        self.pitch_setpoint_param = float(self.get_parameter('pitch_setpoint').value)
        self.roll_setpoint_param = float(self.get_parameter('roll_setpoint').value)
        self.vbd_setpoint_param = float(self.get_parameter('vbd_setpoint').value)
        self.depth_upper = float(self.get_parameter('depth_upper').value)
        self.depth_lower = float(self.get_parameter('depth_lower').value)
        self.alpha_dive = float(self.get_parameter('alpha_dive').value)
        self.alpha_rise = float(self.get_parameter('alpha_rise').value)
        self.T_alpha_cmd = float(self.get_parameter('T_alpha_cmd').value)
        self.T_vbd_cmd = float(self.get_parameter('T_vbd_cmd').value)
        rate = float(self.get_parameter('publish_rate_hz').value)
        self.dt = 1.0 / rate

        #sensor state
        self.depth = 0.0

        #supervisor state
        self._operating = False
        self.diving = True
        self.alpha_ref_raw = self.alpha_dive
        self.vbd_raw_pct = 0.0

        #setpoint smoothing, mission mode only
        self.filt_alpha = FirstOrderFilter(self.T_alpha_cmd, self.alpha_dive)
        self.filt_vbd = FirstOrderFilter(self.T_vbd_cmd, 0.0)

        self._sub_depth = self.create_subscription(
            Float64Stamped, '/pressure/depth', self._cb_depth, 10)
        self._sub_force_surface = self.create_subscription(
            Bool, '/controller/force_surface', self._cb_force_surface, 10)

        self.pub_alpha_sp = self.create_publisher(Float64, '/mission/alpha_setpoint', 10)
        self.pub_roll_sp = self.create_publisher(Float64, '/mission/roll_setpoint', 10)
        self.pub_vbd_sp = self.create_publisher(Float64, '/mission/vbd_setpoint', 10)
        self.pub_phase = self.create_publisher(String, '/controller/phase', 10)

        self.add_on_set_parameters_callback(self._on_param_change)

        #start supervisor if launch set mission_active=true
        if bool(self.get_parameter('mission_active').value) and not self._operating:
            self._reset_supervisor()
            self._operating = True

        self._timer = self.create_timer(self.dt, self._publish_loop)

        self.get_logger().info(f'mission_node started (mode={self.mode})')

    #parameters

    def _on_param_change(self, params):
        for p in params:
            name = p.name
            try:
                if name == 'mode':
                    new_mode = str(p.value)
                    if new_mode not in ('mission', 'static'):
                        return SetParametersResult(
                            successful=False,
                            reason=f"mode must be 'mission' or 'static', got '{new_mode}'")
                    self.mode = new_mode
                elif name == 'mission_active':
                    if bool(p.value) and not self._operating:
                        self._reset_supervisor()
                        self._operating = True
                        self.get_logger().info(
                            f'Mission started, diving to {self.depth_lower}m')
                    elif (not bool(p.value)) and self._operating:
                        self._operating = False
                        self.get_logger().info('Mission stopped externally')
                elif name == 'pitch_setpoint':
                    self.pitch_setpoint_param = float(p.value)
                elif name == 'roll_setpoint':
                    self.roll_setpoint_param = float(p.value)
                elif name == 'vbd_setpoint':
                    self.vbd_setpoint_param = float(p.value)
                elif name == 'depth_upper':
                    self.depth_upper = float(p.value)
                elif name == 'depth_lower':
                    self.depth_lower = float(p.value)
                elif name == 'alpha_dive':
                    self.alpha_dive = float(p.value)
                elif name == 'alpha_rise':
                    self.alpha_rise = float(p.value)
                elif name == 'T_alpha_cmd':
                    self.T_alpha_cmd = float(p.value)
                    self.filt_alpha.tau = max(self.T_alpha_cmd, 1e-6)
                elif name == 'T_vbd_cmd':
                    self.T_vbd_cmd = float(p.value)
                    self.filt_vbd.tau = max(self.T_vbd_cmd, 1e-6)
            except (TypeError, ValueError) as e:
                return SetParametersResult(
                    successful=False, reason=f"bad value for {name}: {e}")
        return SetParametersResult(successful=True)

    def _reset_supervisor(self):
        self.diving = True
        self.alpha_ref_raw = self.alpha_dive
        self.vbd_raw_pct = 0.0
        self.filt_alpha.reset(self.alpha_dive)
        self.filt_vbd.reset(0.0)

    def _self_clear_mission_active(self):
        try:
            self.set_parameters(
                [Parameter('mission_active', Parameter.Type.BOOL, False)])
        except Exception as e:
            self.get_logger().warn(f'Could not self-clear mission_active: {e}')

    #sensor callbacks

    def _cb_depth(self, msg: Float64Stamped):
        self.depth = msg.data

    def _cb_force_surface(self, msg: Bool):
        if self.mode != 'mission':
            return
        if msg.data and self._operating and self.diving:
            self.diving = False
            self.get_logger().warn('EMERGENCY: forced to climb, surfacing')

    #publish helpers

    def _publish_setpoints(self, alpha_rad: float, roll_rad: float, vbd_pct: float):
        m = Float64(); m.data = float(alpha_rad); self.pub_alpha_sp.publish(m)
        m = Float64(); m.data = float(roll_rad);  self.pub_roll_sp.publish(m)
        m = Float64(); m.data = float(vbd_pct);   self.pub_vbd_sp.publish(m)

    def _publish_phase(self, phase_str: str):
        msg = String(); msg.data = phase_str; self.pub_phase.publish(msg)

    def _publish_safe_setpoint(self):
        #neutral pose, full buoyancy
        self._publish_setpoints(0.0, 0.0, 100.0)

    #publish loop

    def _publish_loop(self):
        if self.mode == 'static':
            self._publish_setpoints(
                self.pitch_setpoint_param,
                self.roll_setpoint_param,
                self.vbd_setpoint_param)
            self._publish_phase('OPERATION')
            return

        if not self._operating:
            self._publish_safe_setpoint()
            self._publish_phase('COMPLETE')
            return

        if self.diving and self.depth >= self.depth_lower:
            self.diving = False
            self.get_logger().info('Depth reached, switching to climb')
        elif not self.diving and self.depth <= self.depth_upper:
            self._operating = False
            self._publish_safe_setpoint()
            self._publish_phase('COMPLETE')
            self.get_logger().info('Mission complete, at surface')
            self._self_clear_mission_active()
            return

        if self.diving:
            self.alpha_ref_raw = self.alpha_dive
            self.vbd_raw_pct = 0.0
        else:
            self.alpha_ref_raw = self.alpha_rise
            self.vbd_raw_pct = 100.0

        alpha_filt = self.filt_alpha.update(self.alpha_ref_raw, self.dt)
        vbd_filt = self.filt_vbd.update(self.vbd_raw_pct, self.dt)

        self._publish_setpoints(alpha_filt, 0.0, vbd_filt)
        self._publish_phase('OPERATION')


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
