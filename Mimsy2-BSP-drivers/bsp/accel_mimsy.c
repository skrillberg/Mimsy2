#include "i2c_mimsy.h"
#include "mpu9250/MPU9250_RegisterMap.h"
#include "flash_mimsy.h" //TODO: mive imu_data type to a new mimsy.h file
#include "gptimer.h"
#include "hw_gptimer.h"
#include "hw_memmap.h"
union IMURaw {
  
  uint16_t words[7];
  uint8_t bytes[14];
  
  
};

//TODO: add an imu init function
//mimsy init function; inits board timers, resets imu, wakes imu, sets clock source
//enables sensors

void mimsyIMUInit(){
    board_timer_init();
    uint8_t readbyte;
    
    
         i2c_init();
    uint8_t address;
    address=0x69;
    
     i2c_write_byte(address,MPU9250_PWR_MGMT_1); //reset
     i2c_write_byte(address,0x80);
    
      i2c_write_byte(address,MPU9250_PWR_MGMT_1); //wake
     i2c_write_byte(address,0x00);
    
    uint8_t bytes[2]={MPU9250_PWR_MGMT_1,0x01}  ; 
     i2c_write_bytes(address,bytes,2); //set gyro clock source
   
     //  bytes[0]=0x6A;
  //   bytes[1]=  0x20;
    // i2c_write_bytes(address,bytes); //set reset
     
   /*  bytes[0]=0x6B;
     bytes[1]=  0x80;
     i2c_write_bytes(address,bytes,2); //set reset
*/
     bytes[0]=0x6C;
     bytes[1]=0x03;
        uint8_t *byteptr=&readbyte;
      
        i2c_write_byte(address,MPU9250_PWR_MGMT_2);
     i2c_read_byte(address,byteptr);
     
     i2c_write_byte(address,MPU9250_PWR_MGMT_2); //sens enable
     i2c_write_byte(address,0x00);
     
     i2c_write_byte(address,MPU9250_PWR_MGMT_2);
     i2c_read_byte(address,byteptr);
  
  
}

//TODO: start adding invensense driver-based functions

//reads IMU data from Mimsy's MPU9250 
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
     