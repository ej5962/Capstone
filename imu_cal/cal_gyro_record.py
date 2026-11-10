#!/usr/bin/env python3

import csv
import os

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu



# SAVES FILES AS 2 CSV
# 1: THE WHOLE DATA SET FOR STATIONARY TESTING - keep IMU still, record 60-90 s, stop with Ctrl+C
# 2: THE AVG BIAS FOUND (only written if at least min_samples were collected)

class cal_gyro_record(Node):

    def __init__(self):
        super().__init__('cal_gyro_stationary')

        # Minimum samples needed before the bias file is overwritten (~30 s at 33 Hz)
        self.declare_parameter('min_samples', 1000)
        self.min_samples = int(self.get_parameter('min_samples').value)

        # Create file to store the reference data
        self.store_imu_stationary_data()

        # Running totals for calculating the average
        self.gyro_x_total = 0.0
        self.gyro_y_total = 0.0
        self.gyro_z_total = 0.0

        self.accel_x_total = 0.0
        self.accel_y_total = 0.0
        self.accel_z_total = 0.0

        # Number of measurements received
        self.sample_count = 0

        # Subscribe to raw (uncalibrated) IMU data
        self.get_gyr_raw = self.create_subscription(Imu, '/imu/gyr_raw', self.gyr_callback, 10)

        self.get_logger().info("Collecting stationary IMU data...")
        self.get_logger().info(f"Saving data to: {self.file_path}")
        self.get_logger().info("Keep the rover still. Press Ctrl+C when finished (aim for 60-90 s).")

    def store_imu_stationary_data(self):

        self.declare_parameter('calibration_dir', os.path.expanduser('~/imu_calibration'))
        self.calibration_path = self.get_parameter('calibration_dir').value
        os.makedirs(self.calibration_path, exist_ok=True)                           # Create the folder if it does not exist

        self.file_path = os.path.join(self.calibration_path, 'gyr_cal_record.csv')  # File to store the WHOLE reference data
        self.file = open(self.file_path, 'w', newline='')                           # Open the CSV file
        self.writer = csv.writer(self.file)                                         # Write the file as CSV

        # Write column headings
        self.writer.writerow([
            'time',
            'gyro_x',
            'gyro_y',
            'gyro_z',
            'accel_x',
            'accel_y',
            'accel_z'
        ])

    def gyr_callback(self, msg):

        # Get timestamp
        time = (
            msg.header.stamp.sec +
            msg.header.stamp.nanosec * 1e-9
        )

        # Get gyroscope data
        gyro_x = msg.angular_velocity.x
        gyro_y = msg.angular_velocity.y
        gyro_z = msg.angular_velocity.z

        # Get accelerometer data
        accel_x = msg.linear_acceleration.x
        accel_y = msg.linear_acceleration.y
        accel_z = msg.linear_acceleration.z

        # Store the data in the CSV file
        self.writer.writerow([
            time,
            gyro_x,
            gyro_y,
            gyro_z,
            accel_x,
            accel_y,
            accel_z
        ])

        # Add values to the running totals
        self.gyro_x_total += gyro_x
        self.gyro_y_total += gyro_y
        self.gyro_z_total += gyro_z

        self.accel_x_total += accel_x
        self.accel_y_total += accel_y
        self.accel_z_total += accel_z

        # Increase sample count
        self.sample_count += 1

        # Make sure data is immediately written to the file
        self.file.flush()

    def calculate_bias(self):

        if self.sample_count == 0:
            self.get_logger().error("No IMU data was received.")
            return

        # Calculate gyro averages
        gyro_bias_x = self.gyro_x_total / self.sample_count
        gyro_bias_y = self.gyro_y_total / self.sample_count
        gyro_bias_z = self.gyro_z_total / self.sample_count

        # Calculate accelerometer averages
        accel_average_x = self.accel_x_total / self.sample_count
        accel_average_y = self.accel_y_total / self.sample_count
        accel_average_z = self.accel_z_total / self.sample_count

        # Print results
        self.get_logger().info("========== CALIBRATION RESULTS ==========")
        self.get_logger().info(f"Samples collected: {self.sample_count}")

        self.get_logger().info(f"Gyro bias X: {gyro_bias_x:.6f} rad/s")
        self.get_logger().info(f"Gyro bias Y: {gyro_bias_y:.6f} rad/s")
        self.get_logger().info(f"Gyro bias Z: {gyro_bias_z:.6f} rad/s")

        self.get_logger().info(f"Accelerometer average X: {accel_average_x:.4f} m/s^2")
        self.get_logger().info(f"Accelerometer average Y: {accel_average_y:.4f} m/s^2")
        self.get_logger().info(f"Accelerometer average Z: {accel_average_z:.4f} m/s^2")

        self.get_logger().info("==========================================")

        # Don't let a short run overwrite a good calibration
        if self.sample_count < self.min_samples:
            self.get_logger().warn(
                f"Only {self.sample_count} samples (minimum {self.min_samples}). "
                "bias_file_gyro.csv was NOT updated."
            )
            return

        # Save calibration values
        bias_file = os.path.join(self.calibration_path, 'bias_file_gyro.csv')      # Create new csv @ same path

        # Write bias into a file
        with open(bias_file, 'w', newline='') as file:                             # Create new file if no file exists

            writer = csv.writer(file)

            writer.writerow([
                "gyro_bias_x",
                "gyro_bias_y",
                "gyro_bias_z",
                "accel_average_x",
                "accel_average_y",
                "accel_average_z"
            ])

            writer.writerow([
                gyro_bias_x,
                gyro_bias_y,
                gyro_bias_z,
                accel_average_x,
                accel_average_y,
                accel_average_z
            ])

        self.get_logger().info("Calibration values saved.")

    def destroy_node(self):

        try:
            self.calculate_bias()                   # Calculate bias before closing
        finally:
            self.file.close()                       # Always close the CSV file
            self.get_logger().info("Reference data saved.")
            super().destroy_node()


def main(args=None):

    # Handle Ctrl+C ourselves so logging still works while results are saved
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    node = cal_gyro_record()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass                                        # Ctrl+C: normal way to stop recording
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
