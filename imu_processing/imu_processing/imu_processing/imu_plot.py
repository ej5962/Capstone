#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

import matplotlib.pyplot as plt
from collections import deque


class imu_plot(Node):

    def __init__(self):
        super().__init__("imu_plotter")

        # Subscribe to the IMU topic
        self.subscription = self.create_subscription(
            Imu,
            "/imu/data_raw",
            self.imu_callback,
            10
        )

        # Store the last 200 measurements
        self.time_data = deque(maxlen=200)

        self.accel_x = deque(maxlen=200)
        self.accel_y = deque(maxlen=200)
        self.accel_z = deque(maxlen=200)

        self.gyro_x = deque(maxlen=200)
        self.gyro_y = deque(maxlen=200)
        self.gyro_z = deque(maxlen=200)

        self.start_time = self.get_clock().now()

        # Create plot
        plt.ion()

        self.fig, self.ax = plt.subplots()

        self.accel_lines = self.ax.plot(
            [], [], label="Accel X"
        )[0]

        self.accel_y_line = self.ax.plot(
            [], [], label="Accel Y"
        )[0]

        self.accel_z_line = self.ax.plot(
            [], [], label="Accel Z"
        )[0]

        self.ax.set_xlabel("Time (seconds)")
        self.ax.set_ylabel("Acceleration (m/s²)")
        self.ax.set_title("Fake IMU Accelerometer")
        self.ax.legend()
        self.ax.grid(True)

        # Update graph at 20 Hz
        self.timer = self.create_timer(
            0.05,
            self.update_plot
        )

    def imu_callback(self, msg):

        # Calculate elapsed time
        current_time = self.get_clock().now()

        elapsed_time = (
            current_time - self.start_time
        ).nanoseconds / 1e9

        # Store time
        self.time_data.append(elapsed_time)

        # Store accelerometer data
        self.accel_x.append(
            msg.linear_acceleration.x
        )

        self.accel_y.append(
            msg.linear_acceleration.y
        )

        self.accel_z.append(
            msg.linear_acceleration.z
        )

    def update_plot(self):

        if len(self.time_data) == 0:
            return

        # Update X axis data
        self.accel_lines.set_data(
            self.time_data,
            self.accel_x
        )

        self.accel_y_line.set_data(
            self.time_data,
            self.accel_y
        )

        self.accel_z_line.set_data(
            self.time_data,
            self.accel_z
        )

        # Automatically adjust axes
        self.ax.relim()
        self.ax.autoscale_view()

        # Redraw graph
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def main(args=None):

    rclpy.init(args=args)

    node = imu_plot()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

