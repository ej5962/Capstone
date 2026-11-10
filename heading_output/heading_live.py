#!/usr/bin/env python3
"""Calculate gyro and magnetometer headings from calibrated IMU topics."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float64

from heading_output.heading_func import HeadingFuncs, HeadingKalmanFilter


class HeadingLive(Node):
    def __init__(self):
        super().__init__("heading_live")

        self.gyro_heading = 0.0
        self.previous_gyro_time = None
        self.previous_gyro_rate = 0.0

        self.declare_parameter("angle_noise", 0.001)
        self.declare_parameter("bias_noise", 0.003)
        self.declare_parameter("mag_noise", 0.03)
        self.declare_parameter("max_dt", 0.1)
        self.heading_filter = HeadingKalmanFilter(
            angle_noise=float(self.get_parameter("angle_noise").value),
            bias_noise=float(self.get_parameter("bias_noise").value),
            mag_noise=float(self.get_parameter("mag_noise").value),
            max_dt=float(self.get_parameter("max_dt").value),
        )

        self.gyro_heading_pub = self.create_publisher(
            Float64, "/imu/heading/gyro", 10
        )
        self.mag_heading_pub = self.create_publisher(
            Float64, "/imu/heading/mag", 10
        )
        self.kalman_heading_pub = self.create_publisher(
            Float64, "/imu/heading_kalman", 10
        )
        self.gyro_sub = self.create_subscription(
            Imu, "/imu/gyr_cal", self.gyro_callback, 10
        )
        self.mag_sub = self.create_subscription(
            MagneticField, "/imu/mag_cal", self.mag_callback, 10
        )

        self.get_logger().info(
            "Calculating headings from /imu/gyr_cal and /imu/mag_cal; "
            "publishing radians on /imu/heading/gyro, /imu/heading/mag, "
            "and /imu/heading_kalman."
        )

    def gyro_callback(self, msg):
        self.gyro_heading, _, self.previous_gyro_rate = (
            HeadingFuncs.calc_gyro_heading(
                msg,
                self.gyro_heading,
                self.previous_gyro_time,
                self.previous_gyro_rate,
            )
        )
        stamp = msg.header.stamp
        timestamp = stamp.sec + stamp.nanosec * 1e-9
        self.previous_gyro_time = timestamp

        output = Float64()
        output.data = self.gyro_heading
        self.gyro_heading_pub.publish(output)

        kalman_heading = self.heading_filter.predict(
            msg.angular_velocity.z, timestamp
        )
        self.publish_kalman_heading(kalman_heading)

    def mag_callback(self, msg):
        heading = HeadingFuncs.calc_mag_heading(msg)
        output = Float64()
        output.data = heading
        self.mag_heading_pub.publish(output)

        stamp = msg.header.stamp
        timestamp = stamp.sec + stamp.nanosec * 1e-9
        kalman_heading = self.heading_filter.update(heading, timestamp)
        self.publish_kalman_heading(kalman_heading)

    def publish_kalman_heading(self, heading):
        if heading is None:
            return
        output = Float64()
        output.data = heading
        self.kalman_heading_pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = HeadingLive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
