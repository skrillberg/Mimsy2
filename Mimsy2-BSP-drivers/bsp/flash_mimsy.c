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

/*This function writes a full 2048 KB page-worth of data to flash.
  Parameters:
    IMUData data[]: pointer to array of IMUData structures that are to be written to flash
    uint32_t size: size of data[] in number of IMUData structures
    uint32_t startPage: Flash page where data is to be written
    IMUDataCard *card: pointer to an IMUDataCard where the function will record 
      which page the data was written to and which timestamps on the data are included
 */
void flashWriteIMU(IMUData data[],uint32_t size, uint32_t startPage,IMUDataCard * card){

  uint32_t pageStartAddr=FLASH_BASE + (startPage * PAGE_SIZE); //page base address
  int32_t i32Res;

  uint32_t structNum = size;
  

  i32Res = FlashMainPageErase(pageStartAddr); //erase page so there it can be written to
  
  for(uint32_t i=0;i<size;i++ ){
      uint32_t* wordified_data=data[i].bits; //retrieves the int32 array representation of the IMUData struct
     IntMasterDisable(); //disables interrupts to prevent the write operation from being messed up
      i32Res = FlashMainPageProgram(wordified_data,pageStartAddr+i*IMU_DATA_STRUCT_SIZE,IMU_DATA_STRUCT_SIZE); //write struct to flash
        IntMasterEnable();//renables interrupts
      
   }

   //update card with location information
  card->page=startPage;
  card->startTime=data[0].fields.timestamp;
  card->endTime=data[size-1].fields.timestamp;

}  
/*This function reads a page worth of IMUData from flash. 
  Parameters:
    IMUDataCard card: The IMUDataCard that corresponds to the data that you want to read from flash
    IMUData * dataArray: pointer that points to location of data array that you want the read operation to be written to
    uint32_t size: size of dataArray in number of IMUData structures
*/
void flashReadIMU(IMUDataCard card, IMUData * dataArray,uint32_t size){
  
  uint32_t pageAddr=FLASH_BASE+card.page*PAGE_SIZE;

  for(uint32_t i=0;i<size;i++){
    for(uint32_t j=0;j<IMU_DATA_STRUCT_SIZE/4;j++){
       IntMasterDisable();
    dataArray[i].bits[j]=FlashGet(pageAddr+i*16+j*4);
     IntMasterEnable();
    }
  }
  
}
