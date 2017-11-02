#Mimsy 2 BSP and Drivers
This directory contains the board support package (BSP) and drivers for Mimsy 2. The BSP includes firmware that enables the use of inchworm motors, the Mimsy 2's onboard MPU9250 IMU, Mimsy 2's UART-Based serial port, and Mimsy 2's LEDs. 

##accel_mimsy.c
This file contains methods for interfacing with Mimsy 2's onboard MPU9250 IMU. It can currently initialize the accelerometer, gather accel and gyro data into an IMUData struct, and initialize the DMP onboard sensor fusion processor on the IMU. 

