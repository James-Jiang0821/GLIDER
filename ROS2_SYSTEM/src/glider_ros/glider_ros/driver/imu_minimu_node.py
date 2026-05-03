#!/usr/bin/env python3
"""MinIMU-9 v6 I2C driver: LSM6DSO accel+gyro on /imu/data_raw, LIS3MDL mag on /imu/mag."""
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from std_srvs.srv import Trigger

import numpy as np

import board
import busio
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
from adafruit_lsm6ds import Rate, AccelRange, GyroRange
import adafruit_lis3mdl


class MinImuNode(Node):
    """Pololu MinIMU-9 v6 driver (LSM6DSO @0x6B, LIS3MDL @0x1E)."""

    def __init__(self):
        super().__init__("minimu_imu_node")

        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("rate_hz", 50.0)

        self.declare_parameter("lin_acc_cov", 1e-2)
        self.declare_parameter("ang_vel_cov", 1e-2)
        self.declare_parameter("mag_cov", 1e-6)

        self.declare_parameter("gyro_bias_samples", 250)
        self.declare_parameter("gyro_bias_max_var", 0.001)
        self.declare_parameter("gyro_publish_during_cal", False)

        self.frame_id = self.get_parameter("frame_id").value
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.lin_acc_cov = float(self.get_parameter("lin_acc_cov").value)
        self.ang_vel_cov = float(self.get_parameter("ang_vel_cov").value)
        self.mag_cov = float(self.get_parameter("mag_cov").value)

        self.gyro_bias_samples = int(self.get_parameter("gyro_bias_samples").value)
        self.gyro_bias_max_var = float(self.get_parameter("gyro_bias_max_var").value)
        self.gyro_publish_during_cal = bool(self.get_parameter("gyro_publish_during_cal").value)

        self.gyro_bias = np.zeros(3)
        self.gyro_buffer = []
        self.gyro_calibrated = False

        self.imu_pub = self.create_publisher(Imu, "/imu/data_raw", 20)
        self.mag_pub = self.create_publisher(MagneticField, "/imu/mag", 20)

        self.zero_srv = self.create_service(Trigger, "/imu/gyro_zero", self._handle_gyro_zero)

        self.lsm = None
        self.lis = None
        self.good = 0
        self.bad = 0
        self.consecutive_bad = 0
        self.reinit_threshold = 10

        self._init_sensor()
        self._start_gyro_cal()

        period = 1.0 / self.rate_hz if self.rate_hz > 0 else 0.02
        self.timer = self.create_timer(period, self._tick)

    def _init_sensor(self):
        self.get_logger().info("Initialising I2C and MinIMU-9 v6...")

        i2c = busio.I2C(board.SCL, board.SDA)

        #MinIMU-9 v6 puts LSM6DSO at 0x6B (adafruit default is 0x6A); LSM6DSOX driver is register-compatible
        self.lsm = LSM6DSOX(i2c, address=0x6B)
        self.lsm.accelerometer_range = AccelRange.RANGE_4G
        self.lsm.gyro_range = GyroRange.RANGE_500_DPS
        self.lsm.accelerometer_data_rate = Rate.RATE_104_HZ
        self.lsm.gyro_data_rate = Rate.RATE_104_HZ

        self.lis = adafruit_lis3mdl.LIS3MDL(i2c, address=0x1E)

        self.get_logger().info("MinIMU-9 v6 ready")

    def _start_gyro_cal(self):
        self.gyro_buffer = []
        self.gyro_calibrated = False
        self.get_logger().info(
            f"Gyro calibration: collecting {self.gyro_bias_samples} stationary samples — do not move the glider"
        )

    def _handle_gyro_zero(self, request, response):
        self._start_gyro_cal()
        response.success = True
        response.message = f"gyro cal restarted: {self.gyro_bias_samples} samples"
        return response

    def _tick(self):
        try:
            ax, ay, az = self.lsm.acceleration      #m/s^2
            gx, gy, gz = self.lsm.gyro              #rad/s
            mx, my, mz = self.lis.magnetic          #uT

            if not self.gyro_calibrated:
                self.gyro_buffer.append([gx, gy, gz])
                if len(self.gyro_buffer) >= self.gyro_bias_samples:
                    arr = np.array(self.gyro_buffer)
                    max_var = float(np.max(np.var(arr, axis=0)))
                    if max_var > self.gyro_bias_max_var:
                        self.get_logger().warn(
                            f"motion during gyro cal (max axis variance {max_var:.5f} > {self.gyro_bias_max_var:.5f}), restarting"
                        )
                        self.gyro_buffer = []
                    else:
                        self.gyro_bias = np.mean(arr, axis=0)
                        self.gyro_calibrated = True
                        self.get_logger().info(
                            f"Gyro bias: [{self.gyro_bias[0]:+.4f}, {self.gyro_bias[1]:+.4f}, {self.gyro_bias[2]:+.4f}] rad/s "
                            f"(max axis var {max_var:.5f})"
                        )

                if not self.gyro_publish_during_cal:
                    self.consecutive_bad = 0
                    return

            if self.gyro_calibrated:
                gx -= float(self.gyro_bias[0])
                gy -= float(self.gyro_bias[1])
                gz -= float(self.gyro_bias[2])

            stamp = self.get_clock().now().to_msg()

            imu_msg = Imu()
            imu_msg.header.stamp = stamp
            imu_msg.header.frame_id = self.frame_id

            #no onboard fusion - signal "no orientation" per REP-145
            imu_msg.orientation.x = 0.0
            imu_msg.orientation.y = 0.0
            imu_msg.orientation.z = 0.0
            imu_msg.orientation.w = 1.0
            for i in range(9):
                imu_msg.orientation_covariance[i] = 0.0
                imu_msg.angular_velocity_covariance[i] = 0.0
                imu_msg.linear_acceleration_covariance[i] = 0.0
            imu_msg.orientation_covariance[0] = -1.0

            imu_msg.angular_velocity.x = float(gx)
            imu_msg.angular_velocity.y = float(gy)
            imu_msg.angular_velocity.z = float(gz)
            imu_msg.angular_velocity_covariance[0] = self.ang_vel_cov
            imu_msg.angular_velocity_covariance[4] = self.ang_vel_cov
            imu_msg.angular_velocity_covariance[8] = self.ang_vel_cov

            imu_msg.linear_acceleration.x = float(ax)
            imu_msg.linear_acceleration.y = float(ay)
            imu_msg.linear_acceleration.z = float(az)
            imu_msg.linear_acceleration_covariance[0] = self.lin_acc_cov
            imu_msg.linear_acceleration_covariance[4] = self.lin_acc_cov
            imu_msg.linear_acceleration_covariance[8] = self.lin_acc_cov

            self.imu_pub.publish(imu_msg)

            mag_msg = MagneticField()
            mag_msg.header.stamp = stamp
            mag_msg.header.frame_id = self.frame_id
            #uT -> T
            mag_msg.magnetic_field.x = float(mx) * 1e-6
            mag_msg.magnetic_field.y = float(my) * 1e-6
            mag_msg.magnetic_field.z = float(mz) * 1e-6
            for i in range(9):
                mag_msg.magnetic_field_covariance[i] = 0.0
            mag_msg.magnetic_field_covariance[0] = self.mag_cov
            mag_msg.magnetic_field_covariance[4] = self.mag_cov
            mag_msg.magnetic_field_covariance[8] = self.mag_cov

            self.mag_pub.publish(mag_msg)

            self.good += 1
            self.consecutive_bad = 0

            if self.good % 50 == 0:
                self.get_logger().info(
                    f"ACC [{ax:+.2f}, {ay:+.2f}, {az:+.2f}] m/s^2 | "
                    f"GYR [{gx:+.3f}, {gy:+.3f}, {gz:+.3f}] rad/s | "
                    f"MAG [{mx:+.2f}, {my:+.2f}, {mz:+.2f}] uT"
                )

        except Exception as e:
            self.bad += 1
            self.consecutive_bad += 1
            self.get_logger().debug(
                f"IMU read error (total={self.bad}, streak={self.consecutive_bad}): {e}"
            )

            if self.consecutive_bad >= self.reinit_threshold:
                self.get_logger().warn(
                    f"{self.consecutive_bad} consecutive read failures, reinitialising MinIMU-9..."
                )
                self.consecutive_bad = 0
                try:
                    self._init_sensor()
                except Exception as reinit_error:
                    self.get_logger().error(f"Reinit failed: {reinit_error}")
                    time.sleep(1.0)


def main(args=None):
    rclpy.init(args=args)
    node = MinImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
