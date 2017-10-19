#include "flash_mimsy.h"


#define PAGE_SIZE                2048
#define PAGE_TO_ERASE            14
#define PAGE_TO_ERASE_START_ADDR (FLASH_BASE + (PAGE_TO_ERASE * PAGE_SIZE))

//prototypes
uint32_t * flashWriteIMU(IMUData *data,uint32_t startPage){

uint32_t pageStartAddr=FLASH_BASE + (startPage * PAGE_SIZE);
int32_t i32Res;

uint32_t structNum = sizeof(data);
uint32_t pageNum= structNum*IMU_DATA_STRUCT_SIZE;


uint32_t i;
    

       i32Res = FlashMainPageErase(PAGE_TO_ERASE_START_ADDR);

    
    for(i=0; i<PAGE_SIZE; i+=sizeof(pcStrInRam))
    {
        i32Res = FlashMainPageProgram((uint32_t*) pcStrInRam, 
                                      PAGE_TO_ERASE_START_ADDR+i,
                                      sizeof(pcStrInRam));
     
    }
    flash_contents=FlashGet(PAGE_TO_ERASE_START_ADDR+1);
}  