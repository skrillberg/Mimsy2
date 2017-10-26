#include "i2c_mimsy.h"
#include "mpu9250/MPU9250_RegisterMap.h"
#include "flash_mimsy.h"
#include "gptimer.h"
#include "hw_gptimer.h"
#include "hw_memmap.h"
union IMURaw {
  
  uint16_t words[7];
  uint8_t bytes[14];
  
  
};
void mimsyIMURead6Dof(uint8_t address, IMUData *data){
  uint8_t readbyte;  
  uint8_t *byteptr=&readbyte;

  //Accel X
      i2c_read_register(address,MPU9250_ACCEL_XOUT_H,byteptr);
     (*data).fields.accelX=((uint16_t) readbyte)<<8;
    
    i2c_read_register(address,MPU9250_ACCEL_XOUT_L,byteptr);
      (*data).fields.accelX=((uint16_t) readbyte)| (*data).fields.accelX;
      
        //Accel Y
      i2c_read_register(address,MPU9250_ACCEL_YOUT_H,byteptr);
     (*data).fields.accelY=((uint16_t) readbyte)<<8;
    
    i2c_read_register(address,MPU9250_ACCEL_YOUT_L,byteptr);
      (*data).fields.accelY=((uint16_t) readbyte)| (*data).fields.accelY;
      
        //Accel Z
      i2c_read_register(address,MPU9250_ACCEL_ZOUT_H,byteptr);
     (*data).fields.accelZ=((uint16_t) readbyte)<<8;
    
    i2c_read_register(address,MPU9250_ACCEL_ZOUT_L,byteptr);
      (*data).fields.accelZ=((uint16_t) readbyte)| (*data).fields.accelZ;
      
        //Gyro X
      i2c_read_register(address,MPU9250_GYRO_XOUT_H,byteptr);
     (*data).fields.gyroX=((uint16_t) readbyte)<<8;
    
    i2c_read_register(address,MPU9250_GYRO_XOUT_L,byteptr);
      (*data).fields.gyroX=((uint16_t) readbyte)| (*data).fields.gyroX;
      
        //Gyro Y
      i2c_read_register(address,MPU9250_GYRO_YOUT_H,byteptr);
    (*data).fields.gyroY=((uint16_t) readbyte)<<8;
    
    i2c_read_register(address,MPU9250_GYRO_YOUT_L,byteptr);
      (*data).fields.gyroY=((uint16_t) readbyte)| (*data).fields.gyroY;
      
        //Gyro Z
      i2c_read_register(address,MPU9250_GYRO_ZOUT_H,byteptr);
     (*data).fields.gyroZ=((uint16_t) readbyte)<<8;
    
    i2c_read_register(address,MPU9250_GYRO_ZOUT_L,byteptr);
      (*data).fields.gyroZ=((uint16_t) readbyte)| (*data).fields.gyroZ;
       
      
      (*data).fields.timestamp= TimerValueGet(GPTIMER2_BASE, GPTIMER_A);
}   
     