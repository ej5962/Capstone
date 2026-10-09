#!/usr/bin/env python3

# Python imports
import math


class HeadingFuncs:
    
    # def __init__(self):                 # NO SELF AS NOT A ROS 2 NODE
    #     pass

    # Helper function using: ImuPubPlotFuncs
    # required inputs: imu_msg (sensor_msgs/Imu) - from imu_processing_raw.py

    def calc_gyro_heading(imu_msg_cal, prev_heading = 0.0, prev_t=0.0, prev_w=0.0, gyro_bias_z=0.0):
        # prev_t: previous message's header.stamp in seconds (None on first call).
        #         Do NOT pass back the returned t_stamp.
        # Returns (heading, t_stamp, w): heading at the current message,
        #         midpoint time of the integration step, and bias-corrected yaw rate.

        # NOTE: gyro_bias_z is the average bias of w found from stationary testing, and should be subtracted from the gyro z-rate before integration.
    
        stamp = imu_msg_cal.header.stamp                            # Time the reading was taken
        t = stamp.sec + stamp.nanosec * 1e-9                        # Convert ROS 2 timestamp to seconds
        w = imu_msg_cal.angular_velocity.z - gyro_bias_z            # Yaw rate (rad/s), bias removed
        heading = prev_heading                                      # Initialize heading to the previous heading
        dt  =   0.0                                                 # Initialize dt to 0.0

        if prev_t is not None:                                      # Skip the very first message
            dt = t - prev_t                                         # Real time between readings

        if 0.0 < dt < 0.1:                                          # Valid step: integrate
            heading += 0.5 * (w + prev_w) * dt                      # Trapezoidal integration
            heading = math.atan2(math.sin(heading),
                                 math.cos(heading))                 # Wrap to [-pi, pi]
            t_stamp = t - dt / 2                                    # Midpoint of this step
        else:
            t_stamp = t   

        return heading, t_stamp, w

    def calc_mag_heading(imu_msg_cal):
        # Calculate heading from magnetometer data (rad), wrapped to [-pi, pi].
        # NOTE: This is a simple calculation that does not account for tilt. For more accurate heading, use a complementary filter or Kalman filter.

        m = imu_msg_cal.magnetic_field                              # Magnetometer data
        heading = math.atan2(m.y, m.x)                             # Heading (rad)
        heading = math.atan2(math.sin(heading),
                            math.cos(heading))                      # Wrap to [-pi, pi]

        return heading
 
    def calc_acc_roll_pitch(imu_msg_cal):
        # Roll and pitch (rad) from accelerometer data. Gives NO heading.
        # Only valid when the robot is not accelerating (gravity dominates).
        #
        # Assumes the ROS convention (REP 145): z-axis up, so at rest and level
        # linear_acceleration.z reads about +9.81. If roll reads about +/-pi
        # when the IMU is level, the IMU uses z-down: flip the signs of
        # a.y and a.z in the roll line and a.x in the pitch line.
 
        a = imu_msg_cal.linear_acceleration
        roll = math.atan2(a.y, a.z)                                 # Rotation about x
        pitch = math.atan2(-a.x, math.sqrt(a.y**2 + a.z**2))        # Rotation about y

        return roll, pitch



    
