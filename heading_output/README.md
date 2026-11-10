# Heading estimation

`heading_live` subscribes to calibrated data from `imu_cal_live`:

- `/imu/gyr_cal` (`sensor_msgs/msg/Imu`): gyro Z angular rate in rad/s.
- `/imu/mag_cal` (`sensor_msgs/msg/MagneticField`): calibrated field in tesla.

It publishes `std_msgs/msg/Float64` headings in radians:

- `/imu/heading/gyro`: trapezoidal integration of calibrated gyro Z.
- `/imu/heading/mag`: `atan2(mag_y, mag_x)` from `HeadingFuncs`.
- `/imu/heading_kalman`: fused yaw estimate for `print_headings`.

## Filter model

The two filter states are yaw angle `theta` and residual gyro bias `b`. Gyro
measurements predict the state using `theta += (gyro_z - b) * dt`. Angle is
wrapped into `[-pi, pi]`. The covariance is propagated with angle process
noise and bias random-walk noise. Each magnetometer heading is then treated
as an angle measurement: its innovation is wrapped before the standard
two-state Kalman correction, preventing errors across the `-pi`/`pi` boundary.
The first magnetometer measurement initializes the angle; gyro prediction
continues after initialization.

`angle_noise`, `bias_noise`, and `mag_noise` are ROS parameters on
`heading_live`. They are process-noise rates for angle and bias, and
magnetometer measurement variance, respectively. `max_dt` rejects nonpositive
or unusually large integration steps (default 0.1 seconds). Tune these values
using logged sensor data; the defaults are starting values, not a sensor
characterization.

The magnetic heading currently uses a horizontal-plane `atan2`; it is not
tilt-compensated and assumes the rover is approximately level. The output is
relative yaw, not an absolute north-referenced heading unless the sensor axes,
magnetic declination, and calibration are configured for that interpretation.

`print_headings` prints the latest three values and opens a live plot by
default. The plot shows the last 30 seconds (`plot_window_s`). Disable it
with `--ros-args -p enable_plot:=false` when running without a graphical
display. Matplotlib is required when plotting is enabled.
