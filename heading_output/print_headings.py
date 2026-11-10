#!/usr/bin/env python3
"""Print the latest gyro, magnetometer, and Kalman headings."""

from collections import deque
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class PrintHeadings(Node):
    def __init__(self):
        super().__init__("print_headings")

        self.declare_parameter("gyro_heading_topic", "/imu/heading/gyro")
        self.declare_parameter("mag_heading_topic", "/imu/heading/mag")
        self.declare_parameter("kalman_heading_topic", "/imu/heading_kalman")
        self.declare_parameter("print_rate_hz", 5.0)
        self.declare_parameter("enable_plot", True)
        print_rate = float(self.get_parameter("print_rate_hz").value)
        if print_rate <= 0.0:
            raise ValueError("print_rate_hz must be greater than zero.")

        self.start_time = time.monotonic()
        self.gyro_heading = None
        self.mag_heading = None
        self.kalman_heading = None
        self.plot_data = {
            "gyro": deque(maxlen=1000),
            "mag": deque(maxlen=1000),
            "kalman": deque(maxlen=1000),
        }

        self.gyro_sub = self.create_subscription(
            Float64,
            self.get_parameter("gyro_heading_topic").value,
            self.gyro_callback,
            10,
        )
        self.mag_sub = self.create_subscription(
            Float64,
            self.get_parameter("mag_heading_topic").value,
            self.mag_callback,
            10,
        )
        self.kalman_sub = self.create_subscription(
            Float64,
            self.get_parameter("kalman_heading_topic").value,
            self.kalman_callback,
            10,
        )
        self.print_timer = self.create_timer(1.0 / print_rate, self.print_headings)
        self.plot_enabled = bool(self.get_parameter("enable_plot").value)
        if self.plot_enabled:
            self.setup_plot()
            self.plot_timer = self.create_timer(0.1, self.update_plot)

    def gyro_callback(self, msg):
        self.gyro_heading = msg.data
        self.add_plot_sample("gyro", msg.data)

    def mag_callback(self, msg):
        self.mag_heading = msg.data
        self.add_plot_sample("mag", msg.data)

    def kalman_callback(self, msg):
        self.kalman_heading = msg.data
        self.add_plot_sample("kalman", msg.data)

    def add_plot_sample(self, series, heading):
        elapsed = time.monotonic() - self.start_time
        self.plot_data[series].append((elapsed, heading))

    def setup_plot(self):
        import matplotlib.pyplot as plt

        self.plt = plt
        self.plt.ion()
        self.figure, self.axes = self.plt.subplots()
        self.lines = {
            "gyro": self.axes.plot([], [], label="Gyro")[0],
            "mag": self.axes.plot([], [], label="Magnetometer")[0],
            "kalman": self.axes.plot([], [], label="Kalman")[0],
        }
        self.axes.set_title("Live IMU Headings")
        self.axes.set_xlabel("Time (s)")
        self.axes.set_ylabel("Heading (rad)")
        self.axes.set_ylim(-math.pi, math.pi)
        self.axes.set_yticks(
            [-math.pi, -math.pi / 2, 0.0, math.pi / 2, math.pi],
            ["-π", "-π/2", "0", "π/2", "π"],
        )
        self.axes.grid(True)
        self.axes.legend()
        self.figure.show()

    def update_plot(self):
        for series, line in self.lines.items():
            samples = self.plot_data[series]
            if samples:
                times, headings = zip(*samples)
                line.set_data(times, headings)

        self.axes.relim()
        self.axes.autoscale_view(scalex=True, scaley=False)
        self.figure.canvas.draw_idle()
        self.plt.pause(0.001)

    def print_headings(self):
        if all(
            heading is None
            for heading in (
                self.gyro_heading,
                self.mag_heading,
                self.kalman_heading,
            )
        ):
            return

        print(
            "gyro_head: {}, mag_head: {}, kalman_head: {}".format(
                self.format_heading(self.gyro_heading),
                self.format_heading(self.mag_heading),
                self.format_heading(self.kalman_heading),
            ),
            flush=True,
        )

    def destroy_node(self):
        if self.plot_enabled:
            self.plt.close(self.figure)
        super().destroy_node()

    @staticmethod
    def format_heading(heading):
        return "waiting" if heading is None else f"{heading:.6f}"


def main(args=None):
    rclpy.init(args=args)
    node = PrintHeadings()
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
