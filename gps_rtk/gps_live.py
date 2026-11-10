#!/usr/bin/env python3
"""Print and plot live GPS fixes from a ROS NavSatFix topic."""

from collections import deque
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class GpsLive(Node):
    def __init__(self):
        super().__init__("gps_live")

        self.declare_parameter("gps_topic", "/gps/fix")
        self.declare_parameter("print_rate_hz", 1.0)
        self.declare_parameter("enable_plot", True)
        self.declare_parameter("plot_history", 2000)
        print_rate = float(self.get_parameter("print_rate_hz").value)
        plot_history = int(self.get_parameter("plot_history").value)
        if print_rate <= 0.0:
            raise ValueError("print_rate_hz must be greater than zero.")
        if plot_history < 2:
            raise ValueError("plot_history must be at least 2.")

        self.plot_enabled = bool(self.get_parameter("enable_plot").value)
        self.first_latitude = None
        self.first_longitude = None
        self.last_print_time = 0.0
        self.invalid_fix_reported = False
        self.east_positions = deque(maxlen=plot_history)
        self.north_positions = deque(maxlen=plot_history)

        self.gps_sub = self.create_subscription(
            NavSatFix,
            self.get_parameter("gps_topic").value,
            self.gps_callback,
            10,
        )
        self.print_period = 1.0 / print_rate
        if self.plot_enabled:
            self.setup_plot()
            self.plot_timer = self.create_timer(0.1, self.update_plot)

        self.get_logger().info(
            f"Reading GPS fixes from "
            f"{self.get_parameter('gps_topic').value}."
        )

    def gps_callback(self, msg):
        if msg.status.status < 0 or not all(
            math.isfinite(value)
            for value in (msg.latitude, msg.longitude)
        ):
            if not self.invalid_fix_reported:
                self.get_logger().warning(
                    "GPS has no valid fix yet; waiting for valid coordinates."
                )
                self.invalid_fix_reported = True
            return

        self.invalid_fix_reported = False
        if self.first_latitude is None:
            self.first_latitude = msg.latitude
            self.first_longitude = msg.longitude

        east, north = self.to_local_coordinates(msg.latitude, msg.longitude)
        self.east_positions.append(east)
        self.north_positions.append(north)

        now = time.monotonic()
        if now - self.last_print_time >= self.print_period:
            altitude = (
                "{:.2f} m".format(msg.altitude)
                if math.isfinite(msg.altitude)
                else "unknown"
            )
            print(
                "latitude: {:.8f}, longitude: {:.8f}, altitude: {}, "
                "fix_status: {}".format(
                    msg.latitude,
                    msg.longitude,
                    altitude,
                    msg.status.status,
                ),
                flush=True,
            )
            self.last_print_time = now

    def to_local_coordinates(self, latitude, longitude):
        reference_lat = math.radians(self.first_latitude)
        earth_radius = 6378137.0
        east = (
            math.radians(longitude - self.first_longitude)
            * earth_radius
            * math.cos(reference_lat)
        )
        north = math.radians(latitude - self.first_latitude) * earth_radius
        return east, north

    def setup_plot(self):
        import matplotlib.pyplot as plt

        self.plt = plt
        self.plt.ion()
        self.figure, self.axes = self.plt.subplots()
        self.path_line, = self.axes.plot([], [], "-", label="GPS path")
        self.current_point, = self.axes.plot([], [], "ro", label="Current fix")
        self.axes.set_title("Live GPS Position")
        self.axes.set_xlabel("East from first fix (m)")
        self.axes.set_ylabel("North from first fix (m)")
        self.axes.grid(True)
        self.axes.legend()
        self.axes.set_aspect("equal", adjustable="datalim")
        self.figure.show()

    def update_plot(self):
        if not self.east_positions:
            return

        self.path_line.set_data(self.east_positions, self.north_positions)
        self.current_point.set_data(
            [self.east_positions[-1]], [self.north_positions[-1]]
        )
        self.axes.relim()
        self.axes.autoscale_view()
        self.figure.canvas.draw_idle()
        self.plt.pause(0.001)

    def destroy_node(self):
        if self.plot_enabled:
            self.plt.close(self.figure)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GpsLive()
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
