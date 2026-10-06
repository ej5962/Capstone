#!/usr/bin/env python3

# Python imports
import math

#Ros 2 imports
from sensor_msgs.msg import Imu, MagneticField     # Import the IMU and magnetometer message types

# personal library imports
from imu_plot import ImuPlotFunc


class ImuPubPlotFuncs:
    # ------------------------------------------------
    # Helper functions for the IMU node.
    # Not a node by itself - the node inherits these.
    #
    # The node using this must set:
    #   self.imu, self.frame_id, self.print_count,
    #   self.start_time, self.plots
    # ------------------------------------------------

    def updatenprint_plots(self, a, w, m, type = "raw"):
        # Print twice a second (every 50th reading at 100 Hz)
        self.print_count += 1
        if self.print_count >= 50:
            self.print_count = 0
            self.write_msg(a, measurement="Acceleration", type=type, unit="m/s^2")                 # Accelerometer
            self.write_msg(w, measurement="Angular Velocity", type=type, unit="rad/s")             # Gyroscope
            self.write_msg(m, measurement="Magnetic Field", type=type, unit="uT", scale=1e6)       # Magnetometer (T -> uT)

        if self.plots:
            t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

            self.plot_accel.add_sample(t, a.x, a.y, a.z)                        # Accelerometer
            self.plot_gyro.add_sample(t, w.x, w.y, w.z)                         # Gyroscope
            self.plot_mag.add_sample(t, m.x * 1e6, m.y * 1e6, m.z * 1e6)        # Magnetometer (T -> uT)

    def write_msg(self, msg_data, measurement="variable", type="raw", unit="", scale=1.0):
        # Eg : Acceleration XYZ raw: 0.123, -0.045, 9.807 m/s^2
        # scale lets us print in different units to what is published (eg T -> uT)
        x = round(msg_data.x * scale, 3)
        y = round(msg_data.y * scale, 3)
        z = round(msg_data.z * scale, 3)

        data = measurement + " " + "XYZ " + type + ": " + str(x) + ", " + str(y) + ", " + str(z) + " " + unit
        self.get_logger().info(data)

    def read_imu_data(self):
        # Read the ICM-20948
        ax, ay, az, gx, gy, gz = self.imu.read_accelerometer_gyro_data()
        mx, my, mz = self.imu.read_magnetometer_data()

        # Create ROS IMU message
        imu_msg = Imu()
        mag_msg = MagneticField()

        # Set the timestamp (same for both messages)
        stamp = self.get_clock().now().to_msg()
        imu_msg.header.stamp = stamp
        mag_msg.header.stamp = stamp

        # Set the frame ID
        imu_msg.header.frame_id = self.frame_id
        mag_msg.header.frame_id = self.frame_id

        # ------------------------------------------------
        # Accelerometer
        # Library: g - gravity measurement. Multiply by 9.80665 to get acceleration in m/s^2
        # ROS: m/s^2
        # ------------------------------------------------

        imu_msg.linear_acceleration.x = ax * 9.80665
        imu_msg.linear_acceleration.y = ay * 9.80665
        imu_msg.linear_acceleration.z = az * 9.80665

        # ------------------------------------------------
        # Gyroscope
        # Library: degrees/second
        # ROS: radians/second
        # ------------------------------------------------

        imu_msg.angular_velocity.x = math.radians(gx)
        imu_msg.angular_velocity.y = math.radians(gy)
        imu_msg.angular_velocity.z = math.radians(gz)

        # ------------------------------------------------
        # Magnetometer
        # Library: uT - microtesla
        # ROS: T - tesla. Multiply by 1e-6 to convert
        # ------------------------------------------------

        mag_msg.magnetic_field.x = mx * 1e-6   # T
        mag_msg.magnetic_field.y = my * 1e-6   # T
        mag_msg.magnetic_field.z = mz * 1e-6   # T

        return imu_msg, mag_msg

    def create_plot(self, type = "Raw"):
        # Create a plot for the raw data
        self.plot_accel = ImuPlotFunc(title="IMU Accelerometer (" + type + ")", ylabel="Acceleration (m/s^2)")
        self.plot_gyro = ImuPlotFunc(title="IMU Gyroscope (" + type + ")", ylabel="Angular Velocity (rad/s)")
        self.plot_mag = ImuPlotFunc(title="IMU Magnetometer (" + type + ")", ylabel="Magnetic Field (uT)")

        self.plots = [self.plot_accel, self.plot_gyro, self.plot_mag]

        # Create a timer to update the plots at 10 Hz
        self.create_timer(0.1, self.update_plots)

    def update_plots(self):
        # Redraw every plot
        for plot in self.plots:
            plot.update()

    def close_plots(self):
        # Close every plot window
        for plot in self.plots:
            plot.close()