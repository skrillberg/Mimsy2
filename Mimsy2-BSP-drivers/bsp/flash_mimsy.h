#define IMU_DATA_STRUCT_SIZE 16
#include <stdint.h>
extern uint32_t * flashWrite(uint32_t *data,uint32_t startPage);


typedef union {
  
  struct IMUData{
  uint16_t accelX;//a X data
  uint16_t accelY;//a X data
  uint16_t accelZ;//a X data
  
  uint16_t gyroX;//gyro X data
  uint16_t gyroY;//gyro Y data
  uint16_t gyroZ;//gyro Z data
  
  uint32_t timestamp;
  
 

} fields;
uint32_t bits[4];
}IMUData;
