# Capstone

**Goals:** To assist in automation of device and build a control unit system for a Scout 2.0 Rover with a Latte Panda as CPU.

### Current Setup
Setup includes ROS2 WSL on personal device for virtual testing, with ROS2 installed on lattepanda for testing.

Currently built the imu_processing pkg to be employed in ROS2. Package was made with python.
Imu_processing pkg is now being used to feed into Kalman Filter Pkg - currently in progress.

Currently the ICM20948 IMU is used with the Lattepanda. Note that the IMU is attached to the I2C 1 port pin, powered by 3.3V and CANNOT use the 5V due to current overdraw/overheating.

