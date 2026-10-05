#!/usr/bin/env python3

import math
import csv
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from ament_index_python.packages import get_package_share_directory


#


class imu_calibration_live(Node):

    def __init__(self):
        super().__init__("imu_calibration")

        self.package_path = get_package_share_directory('imu_processing')       # Find imu_processing pkg  
        self.calibration_path = os.path.join(self.package_path, 'calibration')  # Find calibration folder with bias inside

        self.open_bias()        # Open previously recorded bias files

        # Setup the subscriber to live data
        self.start_time = self.get_clock().now()

        # Collect + calibrate raw data


        


    def open_bias(self):
        bias_file = os.path.join(self.calibration_path, bias_file.csv)          # Find the file with bias constants
        with open(bias_file, 'r') as file:
            reader = csv.DictReader(file)

            bias_const = next(reader)

            self.accel_bias_x = float(bias_const['accel_average_x,'])
            self.accel_bias_y = float(bias_const['accel_average_y'])
            self.accel_bias_z = float(bias_const['accel_average_z'])

            self.gyro_bias_x = float(bias_const['gyro_bias_x'])
            self.gyro_bias_y = float(bias_const['gyro_bias_y'])
            self.gyro_bias_z = float(bias_const['gyro_bias_z'])


    def imu_callback_rawdata(self,msg):  # Runs everytime a msg arrives
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

    def calibrate_rawdata (self):
        self.cal_accel_x.append
                
        

def main(args=None):

    # Initialise ROS 2
    rclpy.init(args=args)

    # Create node
    node = imu_calibration_live()

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
