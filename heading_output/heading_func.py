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


class HeadingKalmanFilter:
    """Fuse yaw rate and magnetic heading with a two-state Kalman filter."""

    def __init__(
        self,
        angle_noise=0.001,
        bias_noise=0.003,
        mag_noise=0.03,
        max_dt=0.1,
    ):
        if angle_noise < 0.0 or bias_noise < 0.0 or mag_noise <= 0.0:
            raise ValueError("Noise values must be non-negative; mag_noise > 0.")
        if max_dt <= 0.0:
            raise ValueError("max_dt must be greater than zero.")

        self.angle_noise = angle_noise
        self.bias_noise = bias_noise
        self.mag_noise = mag_noise
        self.max_dt = max_dt
        self.angle = None
        self.bias = 0.0
        self.p00 = mag_noise
        self.p01 = 0.0
        self.p10 = 0.0
        self.p11 = 1.0
        self.last_time = None

    @staticmethod
    def wrap_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def predict(self, gyro_rate, timestamp):
        """Advance the angle state using bias-corrected gyro Z rate."""
        if self.angle is None:
            return None

        if self.last_time is None:
            self.last_time = timestamp
            return self.angle

        dt = timestamp - self.last_time
        self.last_time = timestamp
        if dt <= 0.0 or dt > self.max_dt:
            return self.angle

        rate = gyro_rate - self.bias
        self.angle = self.wrap_angle(self.angle + dt * rate)

        p00 = self.p00
        p01 = self.p01
        self.p00 += dt * (
            dt * self.p11 - p01 - self.p10 + self.angle_noise
        )
        self.p01 -= dt * self.p11
        self.p10 -= dt * self.p11
        self.p11 += self.bias_noise * dt
        return self.angle

    def update(self, mag_heading, timestamp=None):
        """Correct the predicted yaw with a wrapped magnetic measurement."""
        if self.angle is None:
            self.angle = self.wrap_angle(mag_heading)
            self.last_time = timestamp
            return self.angle

        innovation = self.wrap_angle(mag_heading - self.angle)
        innovation_variance = self.p00 + self.mag_noise
        gain_angle = self.p00 / innovation_variance
        gain_bias = self.p10 / innovation_variance

        self.angle = self.wrap_angle(self.angle + gain_angle * innovation)
        self.bias += gain_bias * innovation

        p00 = self.p00
        p01 = self.p01
        self.p00 -= gain_angle * p00
        self.p01 -= gain_angle * p01
        self.p10 -= gain_bias * p00
        self.p11 -= gain_bias * p01
        if timestamp is not None and self.last_time is None:
            self.last_time = timestamp
        return self.angle



    
