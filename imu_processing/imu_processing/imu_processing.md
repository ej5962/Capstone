# IMU Processing file

### Reference Links
**[Heading Estimator](https://przyrbwn.icm.edu.pl/APP/PDF/116/a116z321.pdf)**

**[Current IMU (ICM2094)](https://shop.pimoroni.com/en-au/products/icm20948?variant=27843993960531 )**

# Imu_calibration: 
IMU (ICM2094) takes in the raw acceleration data of *Gyro, Accelerometer, Magnetometer* 

**Equations for Heading: Accelerometer + Gyro:** 

<!-- <img width="746" height="752" alt="image" src="https://github.com/user-attachments/assets/bafb093b-e687-4d5b-a8b7-49d1990b6735" > -->
Using relative heading:

<img width="1474" height="338" alt="image" src="https://github.com/user-attachments/assets/3921ce45-5717-43be-ab83-2dd4d223ddb5" />


**Equations for Heading: Magnetometer** 

*Importing the ICM2094 library: read_magnetometer_data. Read in x,y,z*

<img width="874" height="764" alt="image" src="https://github.com/user-attachments/assets/f334c5cb-b946-45ff-8cde-f3fecc1ae987" />


## Designing my scope

### Constants

### PKGS:

    ImuLiveRawData()  - *Taking live Raw output data*
        accelRawX
        accelRawY
        accelRawZ
        
        gyroRawX
        gyroRawY
        gyroRawZ
        
        magRawX
        magRawY
        magRawZ
  
    imuRawSubscriber() - *Receiving live data*
    imuRawPublisher() - *Publishing live data from sensor input*
        read_magnetometer_data()

    imuPlotLiveData(input data X,Y,Z)

  
  

# Kalman Filter ---
