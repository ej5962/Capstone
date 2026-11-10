#!/usr/bin/env python3
"""Print and plot live GPS fixes from a ROS NavSatFix topic.

The fix comes from nmea_navsat_driver reading the LG290P over USB serial:
    ros2 run nmea_navsat_driver nmea_serial_driver --ros-args \\
        -p port:=/dev/ttyACM1 -p baud:=460800 -r fix:=/gps/fix

Printed position:
    lat / lon   degrees (WGS84, + = north / east)
    alt         metres above the WGS84 ellipsoid
    east/north  metres from the first valid fix (local flat-earth approximation)
    accuracy    metres, 1 sigma horizontal, from the fix covariance
"""

from collections import deque
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import NavSatFix, NavSatStatus

from imu_processing.imu_plot_func import spin_with_plots

FIX_NAMES = {
    NavSatStatus.STATUS_NO_FIX: "NO FIX",
    NavSatStatus.STATUS_FIX: "GPS",
    NavSatStatus.STATUS_SBAS_FIX: "DGPS/SBAS",
    NavSatStatus.STATUS_GBAS_FIX: "RTK",    # RTK fixed or float - see accuracy
}


class GpsLive(Node):
    def __init__(self):
        super().__init__("gps_live")

        self.declare_parameter("gps_topic", "/gps/fix")
        self.declare_parameter("print_period_s", 2.0)
        self.declare_parameter("enable_plot", True)
        self.declare_parameter("plot_history", 2000)
        print_period = float(self.get_parameter("print_period_s").value)
        plot_history = int(self.get_parameter("plot_history").value)
        if print_period <= 0.0:
            raise ValueError("print_period_s must be greater than zero.")
        if plot_history < 2:
            raise ValueError("plot_history must be at least 2.")

        self.plot_enabled = bool(self.get_parameter("enable_plot").value)
        self.lock = threading.Lock()    # Callbacks run in the ROS thread, drawing in the main thread
        self.first_latitude = None
        self.first_longitude = None
        self.latest_fix = None
        self.latest_status = None
        self.east_positions = deque(maxlen=plot_history)
        self.north_positions = deque(maxlen=plot_history)

        self.gps_sub = self.create_subscription(
            NavSatFix,
            self.get_parameter("gps_topic").value,
            self.gps_callback,
            10,
        )
        self.print_timer = self.create_timer(print_period, self.print_position)
        if self.plot_enabled:
            self.setup_plot()

        self.get_logger().info(
            f"Reading GPS fixes from {self.get_parameter('gps_topic').value}, "
            f"printing every {print_period:g} s."
        )

    def gps_callback(self, msg):
        self.latest_status = msg.status.status
        if msg.status.status < 0 or not all(
            math.isfinite(value)
            for value in (msg.latitude, msg.longitude)
        ):
            return

        if self.first_latitude is None:
            self.first_latitude = msg.latitude
            self.first_longitude = msg.longitude

        east, north = self.to_local_coordinates(msg.latitude, msg.longitude)
        self.latest_fix = (msg, east, north)
        with self.lock:
            self.east_positions.append(east)
            self.north_positions.append(north)

    def print_position(self):
        if self.latest_status is None:
            print(f"waiting for {self.get_parameter('gps_topic').value} "
                  "(is nmea_serial_driver running?)", flush=True)
            return
        if self.latest_fix is None:
            print("NO FIX - waiting for satellites (antenna needs open sky)", flush=True)
            return

        msg, east, north = self.latest_fix
        altitude = f"{msg.altitude:.2f} m" if math.isfinite(msg.altitude) else "unknown"
        # Horizontal 1-sigma error from the east and north variances
        variance = msg.position_covariance[0] + msg.position_covariance[4]
        accuracy = f"{math.sqrt(variance):.3f} m" if variance > 0.0 else "unknown"
        fix_name = FIX_NAMES.get(msg.status.status, str(msg.status.status))
        if self.latest_status < 0:
            fix_name += " (fix lost - showing last valid)"

        print(
            f"lat: {msg.latitude:.8f} deg, lon: {msg.longitude:.8f} deg, "
            f"alt: {altitude} | east: {east:.3f} m, north: {north:.3f} m | "
            f"fix: {fix_name}, accuracy: {accuracy}",
            flush=True,
        )

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
        with self.lock:
            if not self.east_positions:
                return
            east = list(self.east_positions)
            north = list(self.north_positions)

        self.path_line.set_data(east, north)
        self.current_point.set_data([east[-1]], [north[-1]])
        self.axes.relim()
        self.axes.autoscale_view()
        self.figure.canvas.draw_idle()

    def destroy_node(self):
        if self.plot_enabled:
            self.plt.close(self.figure)
        super().destroy_node()


def main(args=None):
    node = None
    try:
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        node = GpsLive()
        if node.plot_enabled:
            # The path autoscales, so each frame is a full redraw: keep it at 2 Hz
            spin_with_plots(node, node.update_plot, rate_hz=2.0)
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
