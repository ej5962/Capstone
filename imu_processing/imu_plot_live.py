#!/usr/bin/env python3
"""Live plots of IMU topics. Runs as its own node so plotting never slows the publishers."""

import threading

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu, MagneticField

# personal library imports
from imu_processing.imu_plot_func import ImuPlotFunc, spin_with_plots

# Calibrated data (default):
#   ros2 run imu_processing imu_plot_live
# Raw data:
#   ros2 run imu_processing imu_plot_live --ros-args -p source:=raw


class ImuPlotLive(Node):

    def __init__(self):
        super().__init__("imu_plot_live")

        self.declare_parameter('source', 'cal')     # 'cal' or 'raw'
        self.declare_parameter('window', 500)       # Samples shown (500 = 5 s at 100 Hz)
        source = self.get_parameter('source').value
        window = self.get_parameter('window').value
        label = "Calibrated" if source == "cal" else "Raw"

        self.plot_accel = ImuPlotFunc(window=window, title="IMU Accelerometer (" + label + ")", ylabel="Acceleration (m/s^2)")
        self.plot_gyro = ImuPlotFunc(window=window, title="IMU Gyroscope (" + label + ")", ylabel="Angular Velocity (rad/s)")
        self.plot_mag = ImuPlotFunc(window=window, title="IMU Magnetometer (" + label + ")", ylabel="Magnetic Field (uT)")
        self.plots = [self.plot_accel, self.plot_gyro, self.plot_mag]

        self.start_time = self.get_clock().now()
        self.lock = threading.Lock()    # Callbacks run in the ROS thread, drawing in the main thread

        self.create_subscription(Imu, "/imu/gyr_" + source, self.imu_callback, 50)
        self.create_subscription(MagneticField, "/imu/mag_" + source, self.mag_callback, 50)

        self.get_logger().info("Plotting /imu/gyr_" + source + " and /imu/mag_" + source)

    def now(self):
        return (self.get_clock().now() - self.start_time).nanoseconds / 1e9

    def imu_callback(self, msg):
        t = self.now()
        a = msg.linear_acceleration
        w = msg.angular_velocity
        with self.lock:
            self.plot_accel.add_sample(t, a.x, a.y, a.z)
            self.plot_gyro.add_sample(t, w.x, w.y, w.z)

    def mag_callback(self, msg):
        m = msg.magnetic_field
        with self.lock:
            self.plot_mag.add_sample(self.now(), m.x * 1e6, m.y * 1e6, m.z * 1e6)    # T -> uT

    def update_plots(self):
        with self.lock:
            for plot in self.plots:
                plot.update()

    def close_plots(self):
        for plot in self.plots:
            plot.close()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ImuPlotLive()
    try:
        spin_with_plots(node, node.update_plots)
    finally:
        node.close_plots()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
