#define IMU_DATA_STRUCT_SIZE 16
#include <stdint.h>


typedef union IMUData {
  
  struct {
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

typedef struct IMUDataCard{
    uint32_t page;
    uint32_t startTime;
    uint32_t endTime;
} IMUDataCard;

extern void flashWriteIMU(IMUData data[],uint32_t size, uint32_t startPage, IMUDataCard * card);
extern void flashReadIMU(IMUDataCard card, IMUData *data, uint32_t size);