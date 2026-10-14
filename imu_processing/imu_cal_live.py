#!/usr/bin/env python3

import math
import csv
import os

import imu_raw_live
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField



class imu_cal_live(Node):

    def __init__(self):
        super().__init__("imu_cal")

        # Subscribe to the raw IMU data topic : gyr/acc + mag
        self.get_gyr_raw = self.create_subscription(Imu, "/imu/gyr_raw", self.gyr_callback_rawdata, 10) 
        self.get_mag_raw = self.create_subscription(MagneticField, "/imu/mag_raw", self.mag_callback_rawdata, 10)

        self.declare_parameter('calibration_dir', os.path.expanduser('~/imu_calibration'))
        self.calibration_path = self.get_parameter('calibration_dir').value                 # Find calibration folder with bias inside

        self.open_bias()        # Open previously recorded bias files

        # Setup the subscriber to live data
        self.start_time = self.get_clock().now()

        # Collect + calibrate raw data


        

    def open_bias(self):
        bias_file = os.path.join(self.calibration_path, 'bias_file_gyro.csv')      # Find the file with bias constants
        with open(bias_file, 'r') as file:
            reader = csv.DictReader(file)

            bias_const = next(reader)

            self.accel_bias_x = float(bias_const['accel_average_x,'])
            self.accel_bias_y = float(bias_const['accel_average_y'])
            self.accel_bias_z = float(bias_const['accel_average_z'])

            self.gyro_bias_x = float(bias_const['gyro_bias_x'])
            self.gyro_bias_y = float(bias_const['gyro_bias_y'])
            self.gyro_bias_z = float(bias_const['gyro_bias_z'])


    def gyr_callback_rawdata(self,msg):  # Runs everytime a msg arrives
        # Time data
        current_time = self.get_clock().now() 
        elapsed_time = ( current_time - self.start_time ).nanoseconds / 1e9
        self.time_data.append(elapsed_time)

        # Store accelerometer data
        self.accel_x.append( msg.linear_acceleration.x )
        self.accel_y.append( msg.linear_acceleration.y )
        self.accel_z.append( msg.linear_acceleration.z )

        # Store gyro data
        self.gyro_x.append(msg.angular_velocity.x)
        self.gyro_y.append(msg.angular_velocity.y)
        self.gyro_z.append(msg.angular_velocity.z)

    def mag_callback_rawdata(self,msg):  # Runs everytime a msg arrives
        # Store magnetometer data
        self.mag_x.append(msg.magnetic_field.x)
        self.mag_y.append(msg.magnetic_field.y)
        self.mag_z.append(msg.magnetic_field.z)

    def calibrate_rawdata (self):
        self.cal_accel_x.append
                
        

def main(args=None):

    # Initialise ROS 2
    rclpy.init(args=args)

    # Create node
    node = imu_raw_live()

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
