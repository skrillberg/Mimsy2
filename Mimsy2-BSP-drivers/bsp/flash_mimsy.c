#include "flash_mimsy.h"
#include <stdio.h>
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


#define PAGE_SIZE                2048
#define PAGE_TO_ERASE            14
#define PAGE_TO_ERASE_START_ADDR (FLASH_BASE + (PAGE_TO_ERASE * PAGE_SIZE))

//prototypes
uint32_t * flashWriteIMU(IMUData *data,uint32_t startPage){

uint32_t pageStartAddr=FLASH_BASE + (startPage * PAGE_SIZE);
int32_t i32Res;

uint32_t structNum = sizeof(data);
uint32_t pageNum= structNum*IMU_DATA_STRUCT_SIZE;





       i32Res = FlashMainPageErase(PAGE_TO_ERASE_START_ADDR);
for(uint32_t i=0;i<structNum;i++ ){
    uint32_t* wordified_data=data[i].bits;
    i32Res = FlashMainPageProgram(wordified_data,pageStartAddr+i*sizeof(data[i]),sizeof(data[i]));
     
    
 }
    //flash_contents=FlashGet(PAGE_TO_ERASE_START_ADDR+1);
}  
