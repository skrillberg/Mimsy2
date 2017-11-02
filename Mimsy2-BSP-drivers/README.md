# Mimsy 2 BSP and Drivers
This directory contains the board support package (BSP) and drivers for Mimsy 2. The BSP includes firmware that enables the use of inchworm motors, the Mimsy 2's onboard MPU9250 IMU, Mimsy 2's UART-Based serial port, and Mimsy 2's LEDs. 

## accel_mimsy.c
This file contains methods for interfacing with Mimsy 2's onboard MPU9250 IMU. It can currently initialize the accelerometer, gather accel and gyro data into an IMUData struct, and initialize the DMP onboard sensor fusion processor on the IMU. 

### Dependencies
* flash_mimsy.h: requires the IMUData struct from flash_mimsy
* mpu9250/MPU9250_RegisterMap.h
* i2c_mimsy.h: for i2c reads and writes
* gptimer.h: for timer access for timestamping
* hw_gptimer.h: for timer access
* hw_memmap.h: 
* inv_mpu.h:invensense mpu drivers
* inv_mpu_dmp_motion_driver.h:
* invensense.h: random utilities used by the invensense code
* invensense_adv.h: header for MPL binary library
* eMPL_outputs.h: includes data output utilities used by invensense's MPL
* mltypes.h: more random invensense stuff
* mpu.h: more invensense libraries
* log.h: more invensense utilities

### Functions 
* void mimsyIMUInit()

  Initalizes the imu on mimsy. This function enables all of the sensors.


* void mimsySetAccelFsr(int fsr)

  Sets the accellerometers full scale range in g's. Valid values for fsr are 2,4,8,and 16


* void mimsySetGyroFsr(int fsr)

  Sets the gyro's full scale range. Valid values are 250,500,1000, and 2000


* void mimsyIMURead6Dof( IMUData *data)

  Read accel xyz and gyro xyz data from imu. The input to this function is a pointer to a IMUData struct.


* void mimsyDmpBegin()

  Initializes the dmp 
  
## flash_mimsy.c
This file contains functions for using Mimsy's flash memory. Right now it supports saving and reading IMUData structs 

## i2c_mimsy.c
This file contains functions for using i2c to write to and read from the accelerometer.
