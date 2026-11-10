#!/usr/bin/env python3
"""Print and plot the latest gyro, magnetometer, and Kalman headings.

Headings are shown as compass bearings: 0-360 degrees, clockwise from north.
The published topics stay in radians, counter-clockwise (ROS convention).

To calibrate north: point the rover north, read the printed heading, and pass
it back in, eg. --ros-args -p north_offset_deg:=37.5
"""

from collections import deque
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float64

from imu_processing.imu_plot_func import spin_with_plots


class PrintHeadings(Node):
    def __init__(self):
        super().__init__("print_headings")

        self.declare_parameter("gyro_heading_topic", "/imu/heading/gyro")
        self.declare_parameter("mag_heading_topic", "/imu/heading/mag")
        self.declare_parameter("kalman_heading_topic", "/imu/heading_kalman")
        self.declare_parameter("print_rate_hz", 5.0)
        self.declare_parameter("enable_plot", True)
        self.declare_parameter("plot_window_s", 30.0)
        # Bearing shown when the rover faces north (subtracted so north reads 0)
        self.declare_parameter("north_offset_deg", 0.0)
        # False: topics are counter-clockwise positive (z-axis up, REP 103)
        self.declare_parameter("clockwise_positive", False)
        self.north_offset = float(self.get_parameter("north_offset_deg").value)
        self.direction = 1.0 if self.get_parameter("clockwise_positive").value else -1.0
        self.gyro_offset = None    # Aligns the relative gyro heading to the compass
        print_rate = float(self.get_parameter("print_rate_hz").value)
        if print_rate <= 0.0:
            raise ValueError("print_rate_hz must be greater than zero.")

        self.start_time = time.monotonic()
        self.lock = threading.Lock()    # Callbacks run in the ROS thread, drawing in the main thread
        self.gyro_heading = None
        self.mag_heading = None
        self.kalman_heading = None
        # Trim by time, not count: Kalman publishes twice as often as the others
        self.plot_window = float(self.get_parameter("plot_window_s").value)
        self.plot_data = {"gyro": deque(), "mag": deque(), "kalman": deque()}

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

    def to_compass(self, heading_rad, offset=0.0):
        # Radians (topic convention) -> degrees clockwise from north, [0, 360)
        degrees = self.direction * math.degrees(heading_rad) - self.north_offset + offset
        return degrees % 360.0

    def gyro_callback(self, msg):
        # The gyro heading starts at 0 wherever heading_live started, so line
        # it up with the magnetometer's compass bearing once one is available
        if self.gyro_offset is None:
            if self.mag_heading is None:
                return
            self.gyro_offset = self.mag_heading - self.to_compass(msg.data)
        self.gyro_heading = self.to_compass(msg.data, self.gyro_offset)
        self.add_plot_sample("gyro", self.gyro_heading)

    def mag_callback(self, msg):
        self.mag_heading = self.to_compass(msg.data)
        self.add_plot_sample("mag", self.mag_heading)

    def kalman_callback(self, msg):
        self.kalman_heading = self.to_compass(msg.data)
        self.add_plot_sample("kalman", self.kalman_heading)

    def add_plot_sample(self, series, heading):
        elapsed = time.monotonic() - self.start_time
        with self.lock:
            samples = self.plot_data[series]
            samples.append((elapsed, heading))
            while samples[0][0] < elapsed - self.plot_window:
                samples.popleft()

    def setup_plot(self):
        import matplotlib.pyplot as plt

        self.plt = plt
        self.plt.ion()
        self.figure, self.axes = self.plt.subplots()
        # animated=True keeps the lines out of the full redraw; they're blitted
        # on top of a cached background instead (a full redraw is ~60-80 ms)
        self.lines = {
            "gyro": self.axes.plot([], [], label="Gyro", animated=True)[0],
            "mag": self.axes.plot([], [], label="Magnetometer", animated=True)[0],
            "kalman": self.axes.plot([], [], label="Kalman", animated=True)[0],
        }
        self.axes.set_title("Live IMU Headings")
        self.axes.set_xlabel("Time (s ago)")
        self.axes.set_ylabel("Heading (deg, clockwise from N)")
        # Fixed axes (x is relative to now) so the background never changes
        self.axes.set_xlim(-self.plot_window, 0.0)
        self.axes.set_ylim(0.0, 360.0)
        self.axes.set_yticks(
            [0, 45, 90, 135, 180, 225, 270, 315, 360],
            ["N 0", "NE 45", "E 90", "SE 135", "S 180", "SW 225", "W 270", "NW 315", "N 360"],
        )
        self.axes.grid(True)
        self.axes.legend(loc="upper left")
        self.figure.tight_layout()

        # Re-cache the background whenever the window is drawn (eg. resized)
        self.background = None
        self.figure.canvas.mpl_connect("draw_event", self.cache_background)
        self.figure.show()

    def cache_background(self, _event):
        canvas = self.figure.canvas
        self.background = canvas.copy_from_bbox(self.figure.bbox)

    def update_plot(self):
        now = time.monotonic() - self.start_time
        for series, line in self.lines.items():
            with self.lock:
                samples = list(self.plot_data[series])
            if samples:
                times, headings = self.break_at_wrap(samples)
                line.set_data([t - now for t in times], headings)

        canvas = self.figure.canvas
        if self.background is None:
            canvas.draw()    # First frame: draw everything, which caches the background
            return
        canvas.restore_region(self.background)
        for line in self.lines.values():
            self.axes.draw_artist(line)
        canvas.blit(self.figure.bbox)

    @staticmethod
    def break_at_wrap(samples):
        # Insert a gap where the heading wraps between 360 and 0, so the
        # plot doesn't draw a vertical line across the whole axis
        times, headings = [], []
        for t, heading in samples:
            if headings and abs(heading - headings[-1]) > 180.0:
                times.append(t)
                headings.append(float("nan"))
            times.append(t)
            headings.append(heading)
        return times, headings

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
        return "waiting" if heading is None else f"{heading:6.1f} deg"


def main(args=None):
    node = None
    try:
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        node = PrintHeadings()
        if node.plot_enabled:
            spin_with_plots(node, node.update_plot)
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
