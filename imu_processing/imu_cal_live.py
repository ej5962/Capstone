#!/usr/bin/env python3
"""Apply saved gyro and magnetometer calibrations to live IMU messages."""

import csv
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField


class ImuCalLive(Node):
    def __init__(self):
        super().__init__("imu_cal")

        self.declare_parameter(
            "calibration_dir", os.path.expanduser("~/imu_calibration")
        )
        self.calibration_path = self.get_parameter("calibration_dir").value
        self.load_gyro_biases()
        self.load_mag_calibration()

        self.gyr_pub = self.create_publisher(Imu, "/imu/gyr_cal", 10)
        self.mag_pub = self.create_publisher(MagneticField, "/imu/mag_cal", 10)
        self.gyr_sub = self.create_subscription(
            Imu, "/imu/gyr_raw", self.gyr_callback_rawdata, 10
        )
        self.mag_sub = self.create_subscription(
            MagneticField, "/imu/mag_raw", self.mag_callback_rawdata, 10
        )

        self.get_logger().info(
            "Publishing calibrated gyro on /imu/gyr_cal and calibrated "
            "magnetometer data on /imu/mag_cal. Accelerometer values remain "
            "unchanged."
        )

    def load_gyro_biases(self):
        bias_file = os.path.join(self.calibration_path, "bias_file_gyro.csv")
        try:
            with open(bias_file, newline="") as file:
                reader = csv.DictReader(file)
                row = next(reader, None)
        except OSError as exc:
            raise RuntimeError(
                f"Unable to read calibration file {bias_file}: {exc}"
            ) from exc

        required_fields = (
            "gyro_bias_x",
            "gyro_bias_y",
            "gyro_bias_z",
        )
        if row is None or any(field not in row for field in required_fields):
            raise ValueError(
                f"{bias_file} must contain a header and values for "
                f"{', '.join(required_fields)}"
            )

        try:
            self.gyro_bias_x = float(row["gyro_bias_x"])
            self.gyro_bias_y = float(row["gyro_bias_y"])
            self.gyro_bias_z = float(row["gyro_bias_z"])
        except (TypeError, ValueError) as exc:
            raise ValueError(f"Invalid gyro bias values in {bias_file}") from exc

    def load_mag_calibration(self):
        calibration_file = os.path.join(
            self.calibration_path, "mag_calibration.csv"
        )
        matrix_fields = tuple(
            f"matrix_{row}{column}"
            for row in range(3)
            for column in range(3)
        )
        required_fields = (
            "offset_x",
            "offset_y",
            "offset_z",
            *matrix_fields,
        )
        try:
            with open(calibration_file, newline="") as file:
                reader = csv.DictReader(file)
                row = next(reader, None)
        except OSError as exc:
            raise RuntimeError(
                f"Unable to read magnetometer calibration file "
                f"{calibration_file}: {exc}"
            ) from exc

        if row is None or any(field not in row for field in required_fields):
            raise ValueError(
                f"{calibration_file} must contain a header and values for "
                f"{', '.join(required_fields)}"
            )

        try:
            self.mag_offset = tuple(
                float(row[f"offset_{axis}"]) for axis in "xyz"
            )
            self.mag_matrix = tuple(
                tuple(
                    float(row[f"matrix_{i}{j}"])
                    for j in range(3)
                )
                for i in range(3)
            )
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f"Invalid magnetometer calibration values in {calibration_file}"
            ) from exc

    def gyr_callback_rawdata(self, msg):
        calibrated = Imu()
        calibrated.header = msg.header
        calibrated.orientation = msg.orientation
        calibrated.orientation_covariance = msg.orientation_covariance
        calibrated.linear_acceleration = msg.linear_acceleration
        calibrated.linear_acceleration_covariance = msg.linear_acceleration_covariance
        calibrated.angular_velocity_covariance = msg.angular_velocity_covariance

        calibrated.angular_velocity.x = msg.angular_velocity.x - self.gyro_bias_x
        calibrated.angular_velocity.y = msg.angular_velocity.y - self.gyro_bias_y
        calibrated.angular_velocity.z = msg.angular_velocity.z - self.gyro_bias_z
        self.gyr_pub.publish(calibrated)

    def mag_callback_rawdata(self, msg):
        # Center the field before applying the fitted soft-iron correction.
        raw = msg.magnetic_field
        centered = (
            raw.x - self.mag_offset[0],
            raw.y - self.mag_offset[1],
            raw.z - self.mag_offset[2],
        )
        calibrated = MagneticField()
        calibrated.header = msg.header
        # Propagate sensor uncertainty through the same linear correction.
        calibrated.magnetic_field_covariance = [
            sum(
                self.mag_matrix[row][i]
                * msg.magnetic_field_covariance[i * 3 + j]
                * self.mag_matrix[column][j]
                for i in range(3)
                for j in range(3)
            )
            for row in range(3)
            for column in range(3)
        ]
        calibrated.magnetic_field.x = sum(
            self.mag_matrix[0][i] * centered[i] for i in range(3)
        )
        calibrated.magnetic_field.y = sum(
            self.mag_matrix[1][i] * centered[i] for i in range(3)
        )
        calibrated.magnetic_field.z = sum(
            self.mag_matrix[2][i] * centered[i] for i in range(3)
        )
        self.mag_pub.publish(calibrated)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ImuCalLive()
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
