#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class imu_processing_fake(Node):

    def __init__(self):
        super().__init__("imu_publisher")

        # ROS 2 publisher
        self.publisher = self.create_publisher(
            Imu,
            "/imu/data_raw",
            10
        )

        # Publish at 100 Hz
        self.timer = self.create_timer(
            0.01,
            self.publish_imu
        )

        self.get_logger().info(
            "Fake IMU publisher started"
        )

    def publish_imu(self):

        # Fake IMU data
        # Accelerometer: g
        accel = (0.1, 0.2, 1.0)

        # Gyroscope: degrees/second
        gyro = (5.0, 10.0, 15.0)

        # Create ROS IMU message
        msg = Imu()

        # Accelerometer
        # Convert from g to m/s^2
        msg.linear_acceleration.x = accel[0] * 9.80665
        msg.linear_acceleration.y = accel[1] * 9.80665
        msg.linear_acceleration.z = accel[2] * 9.80665

        # Gyroscope
        # Convert from degrees/second to radians/second
        msg.angular_velocity.x = math.radians(gyro[0])
        msg.angular_velocity.y = math.radians(gyro[1])
        msg.angular_velocity.z = math.radians(gyro[2])

        # We are not calculating orientation yet // orientation not provided
        msg.orientation_covariance[0] = -1.0

        # Publish message
        self.publisher.publish(msg)


def main(args=None):

    # Initialise ROS 2
    rclpy.init(args=args)

    # Create node
    node = imu_processing_fake()

    try:
        # Keep node running
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        # Clean up
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
