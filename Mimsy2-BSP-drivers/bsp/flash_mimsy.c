#include "flash_mimsy.h"
#include <stdio.h>
//#include <stdlib.h>
//#include "bsp.h"
//#include "bsp_led.h"
#include "gptimer.h"
#include "sys_ctrl.h"
#include "hw_gptimer.h"
#include "hw_ints.h"
#include "gpio.h"
#include "interrupt.h"
#include "led.h"
#include "hw_memmap.h"
#include "hw_gpio.h"
#include "ioc.h"
#include "inchworm.h"
#include "..\..\cc2538_foundation_firmware_1_0_1_0\driverlib\cc2538\source\flash.h"

#define IMU_DATA_SIZE            16
#define PAGE_SIZE                2048
#define PAGE_TO_ERASE            14
#define PAGE_TO_ERASE_START_ADDR (FLASH_BASE + (PAGE_TO_ERASE * PAGE_SIZE))

//prototypes
void flashWriteIMU(IMUData data[],uint32_t size, uint32_t startPage,IMUDataCard * card){

  uint32_t pageStartAddr=FLASH_BASE + (startPage * PAGE_SIZE);
  int32_t i32Res;

  uint32_t structNum = size;
  uint32_t pageNum= structNum*IMU_DATA_STRUCT_SIZE;





  i32Res = FlashMainPageErase(pageStartAddr);
  for(uint32_t i=0;i<size;i++ ){
      uint32_t* wordified_data=data[i].bits;
      i32Res = FlashMainPageProgram(wordified_data,pageStartAddr+i*IMU_DATA_SIZE,IMU_DATA_SIZE);
       
      
   }
    //flash_contents=FlashGet(PAGE_TO_ERASE_START_ADDR+1);
  
  
  card->page=startPage;
  card->startTime=data[0].fields.timestamp;
  card->endTime=data[size-1].fields.timestamp;
  
  
 
  
  
}  

void flashReadIMU(IMUDataCard card, IMUData * dataArray,uint32_t size){
  uint32_t pageAddr=FLASH_BASE+card.page*PAGE_SIZE;
  
  
  for(uint32_t i=0;i<size;i++){
    for(uint32_t j=0;j<IMU_DATA_SIZE;j++){
    dataArray[i].bits[j]=FlashGet(pageAddr+i*16+j*4);
    }
  }
  
}
