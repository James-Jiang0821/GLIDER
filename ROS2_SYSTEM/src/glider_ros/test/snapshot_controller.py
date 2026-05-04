#!/usr/bin/env python3
"""
Manual snapshot script for ControllerNode.

Runs hand-crafted scenarios through the controller and prints actuator
outputs after step 1 / 10 / 100 / 500. Not picked up by pytest (no
`test_` prefix). Run from a sourced workspace:
    python3 src/glider_ros/test/snapshot_controller.py
"""
import math
import rclpy

from glider_ros.controller.controller_node import GliderController


SNAPSHOT_STEPS = [1, 10, 100, 500]


class _CapturePub:
    def __init__(self):
        self.values = []

    def publish(self, msg):
        self.values.append(msg.data)


def _build_node(snapshot):
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
    node.dt = 0.1

    node.theta = math.radians(snapshot['theta_deg'])
    node.q = math.radians(snapshot['q_dps'])
    node.phi = math.radians(snapshot['phi_deg'])
    node.p = math.radians(snapshot['p_dps'])
    node.pitch_setpoint = math.radians(snapshot['pitch_sp_deg'])
    node.roll_setpoint = math.radians(snapshot['roll_sp_deg'])
    node.vbd_setpoint = snapshot['vbd_sp_pct']
    if 'enable_roll' in snapshot:
        node.enable_roll = snapshot['enable_roll']
    return node, pubs


def run_scenario(name, snapshot):
    node, pubs = _build_node(snapshot)
    try:
        for _ in range(max(SNAPSHOT_STEPS)):
            node._control_loop()

        print(f'\n=== {name} ===')
        print('inputs:')
        print(f"  theta = {snapshot['theta_deg']:+6.1f} deg   "
              f"q = {snapshot['q_dps']:+6.1f} deg/s")
        print(f"  phi   = {snapshot['phi_deg']:+6.1f} deg   "
              f"p = {snapshot['p_dps']:+6.1f} deg/s")
        print(f"  setpoints: pitch = {snapshot['pitch_sp_deg']:+6.1f} deg, "
              f"roll = {snapshot['roll_sp_deg']:+6.1f} deg, "
              f"vbd = {snapshot['vbd_sp_pct']:5.1f} %")
        if 'enable_roll' in snapshot:
            print(f"  enable_roll = {snapshot['enable_roll']}")

        print(f'outputs (dt = {node.dt} s):')
        print(f"  {'step':>5}  {'pitch_mm':>10}  {'roll_deg':>10}"
              f"  {'vbd_L':>5}  {'vbd_R':>5}")
        for k in SNAPSHOT_STEPS:
            i = k - 1
            print(f"  {k:>5}  {pubs['pitch_mm'].values[i]:>+10.3f}  "
                  f"{pubs['roll_deg'].values[i]:>+10.3f}  "
                  f"{pubs['vbd_left'].values[i]:>5}  "
                  f"{pubs['vbd_right'].values[i]:>5}")
    finally:
        node.destroy_node()


SCENARIOS = [
    ('climb-from-level',
     dict(theta_deg=0.0, q_dps=0.0, phi_deg=0.0, p_dps=0.0,
          pitch_sp_deg=5.0, roll_sp_deg=0.0, vbd_sp_pct=80.0)),

    ('dive-from-level',
     dict(theta_deg=0.0, q_dps=0.0, phi_deg=0.0, p_dps=0.0,
          pitch_sp_deg=-5.0, roll_sp_deg=0.0, vbd_sp_pct=20.0)),

    ('hold-level-with-roll-disturbance',
     dict(theta_deg=0.0, q_dps=0.0, phi_deg=10.0, p_dps=0.0,
          pitch_sp_deg=0.0, roll_sp_deg=0.0, vbd_sp_pct=50.0)),

    ('mid-climb-already-pitched-up',
     dict(theta_deg=3.0, q_dps=1.0, phi_deg=0.0, p_dps=0.0,
          pitch_sp_deg=5.0, roll_sp_deg=0.0, vbd_sp_pct=70.0)),

    ('surface-vbd-only-no-attitude-error',
     dict(theta_deg=0.0, q_dps=0.0, phi_deg=0.0, p_dps=0.0,
          pitch_sp_deg=0.0, roll_sp_deg=0.0, vbd_sp_pct=100.0)),
]


def main():
    rclpy.init()
    try:
        for name, snap in SCENARIOS:
            run_scenario(name, snap)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
