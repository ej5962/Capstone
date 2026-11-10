# Capstone

**Goals:** To assist in automation of device and build a control unit system for a Scout 2.0 Rover with a Latte Panda as CPU.

### Current Setup
Setup includes ROS2 WSL on personal device for virtual testing, with ROS2 installed on lattepanda for testing.

Currently built the imu_processing pkg to be employed in ROS2. Package was made with python.
Imu_processing pkg is now being used to feed into Kalman Filter Pkg - currently in progress.

Currently the ICM20948 IMU is used with the Lattepanda. Note that the IMU is attached to the I2C 1 port pin, powered by 3.3V and CANNOT use the 5V due to current overdraw/overheating.

### IMU calibration

The calibration node consumes `/imu/gyr_raw` and `/imu/mag_raw`. It requires
`~/imu_calibration/bias_file_gyro.csv` and
`~/imu_calibration/mag_calibration.csv`, and publishes calibrated messages on
`/imu/gyr_cal` and `/imu/mag_cal`. The magnetometer calibration uses a
hard- and soft-iron ellipsoid fit.

To create the magnetometer calibration:

1. Run `imu_raw_live` in one terminal.
2. Run `cal_mag_record` in another terminal. Press `s` to start recording,
   slowly rotate the IMU through all orientations in 3D, and press `s` again
   to stop. Press `q` to exit. The recorder saves its CSV under
   `~/mag_recordings`.
3. Run `ros2 run imu_processing calibrate_mag ~/mag_recordings/<recording>.csv`.
   The calibration is saved to `~/imu_calibration/mag_calibration.csv`.
4. Run `imu_cal_live` while `imu_raw_live` is publishing.

The gyro bias file is created by `cal_gyro_record` while the IMU is stationary.
Rebuild and source the ROS workspace after changing or installing this package.

### Live heading output

Run these nodes in separate terminals to calculate and print headings:

```bash
ros2 run imu_processing heading_live
ros2 run imu_processing print_headings
```

`heading_live` uses `/imu/gyr_cal` and `/imu/mag_cal`, then publishes radians
on `/imu/heading/gyro`, `/imu/heading/mag`, and `/imu/heading_kalman`.
`print_headings` prints the latest values in this format:
`gyro_head: ..., mag_head: ..., kalman_head: ...`.

The Kalman heading is `std_msgs/msg/Float64` in radians. Its angle-and-bias
filter integrates calibrated gyro Z rates and corrects drift with wrapped
magnetometer heading measurements. See
[`heading_output/README.md`](./heading_output/README.md) for the filter model
and tuning parameters. The printer's `kalman_heading_topic` parameter can be
overridden if needed.

### Live GPS output

`gps_rtk/gps_live.py` reads `sensor_msgs/msg/NavSatFix` from `/gps/fix`, prints
latitude, longitude, altitude, and fix status, and plots the path in local
east/north meters relative to the first valid fix. Run it with:

```bash
ros2 run imu_processing gps_live
```

Override `gps_topic` if the receiver publishes elsewhere. Disable the plot on a
headless system with `--ros-args -p enable_plot:=false`.
