//*****************************************************************************
//! @file       leds_example.c
//! @brief      Example of board support package led functionality.
//!             This example uses the bspLed functions to blink two LEDs.
//!             See bsp_led.c for more information about the BSP led package.
//!
//! Revised     $Date: 2013-03-14 11:55:47 +0100 (on, 14 mar 2013) $
//! Revision    $Revision: 7035 $
//
//  Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
//
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//    Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//****************************************************************************/


/******************************************************************************
* INCLUDES
*/
#include "mpu9250/MPU9250_RegisterMap.h"
#include <stdio.h>
//#include "uartstdio.h"
//#include "bsp.h"
//#include "bsp_led.h"
//#include "board.c"
#include "gptimer.h"
#include "accel_mimsy.h"
#include "i2c_mimsy.h"
#include "sys_ctrl.h"
#include "hw_gptimer.h"
#include "hw_ints.h"
#include "leds_example.h"
#include "interrupt.h"
#include "led.h"
#include "hw_memmap.h"
#include "hw_gpio.h"
#include "ioc.h"
#include "inchworm.h"
#include "..\..\cc2538_foundation_firmware_1_0_1_0\driverlib\cc2538\source\flash.h"
#include "flash_mimsy.h"
#include "string.h"
#include "usb_firmware_library_headers.h"
#include "usb_cdc.h"
#include "usb_in_buffer.h"
#include "usb_out_buffer.h"
#include "uart.h"
#include "hw_ioc.h"


#include "mpl.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "mltypes.h"
#include "mpu.h"
#include "log.h"
//#include "packet.h"
#include "uart_mimsy.h"

/******************************************************************************
* DEFINES
*/
#define GP4 
#define FREQ_CNT SysCtrlClockGet()/frequency 
#define OVERLAP_MATCH (100-dutycycle)*SysCtrlClockGet()/frequency/100 
#define DEFAULT_MPU_HZ  (20)
#define PAGE_SIZE                2048
#define PAGE_TO_ERASE            14
#define PAGE_TO_ERASE_START_ADDR (FLASH_BASE + (PAGE_TO_ERASE * PAGE_SIZE))


#define FLASH_PAGE_STORAGE_START              14
#define FLASH_PAGES_TOUSE                       100
/******************************************************************************


* LOCAL VARIABLES AND FUNCTIONS
*/
uint32_t frequency=2000; //must be greater than 250  
uint32_t dutycycle=80;
uint32_t timer;
uint32_t timeroffset;
  uint32_t load;
  uint32_t x=5;
  uint32_t y=5;
  uint32_t z=5;
 uint32_t flash_contents; 
  volatile uint32_t gpio_state;
  uint32_t debug;
  uint32_t debug2[4];
  IMUData debug4;
   IMUData imu;
   uint32_t debug3[4];
   IMUData data[128];
   IMUDataCard refCard;
   IMUData flashData[128];
   IMUDataCard * refptr;
   uint8_t readbyte;
   uint32_t bufferCount=0;
   uint32_t currentflashpage=14;
   IMUDataCard cards[100];
   uint32_t pagesWritten=0;
   union IMURaw {
  
  uint16_t words[7];
  uint8_t bytes[14];
  
  
}imuraw;
   USB_EPIN_RINGBUFFER_DATA usbCdcInBufferData;
USB_EPOUT_RINGBUFFER_DATA usbCdcOutBufferData;
static uint8_t pInBuffer[128];
static uint8_t pOutBuffer[128];
static uint8_t pAppBuffer[128];
struct platform_data_s {
    signed char orientation[9];
};

static struct platform_data_s gyro_pdata = {
    .orientation = { 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1}
};
struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};
struct hal_s {
    unsigned char lp_accel_mode;
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned char motion_int_mode;
    unsigned long no_dmp_hz;
    unsigned long next_pedo_ms;
    unsigned long next_temp_ms;
    unsigned long next_compass_ms;
    unsigned int report;
    unsigned short dmp_features;
    struct rx_s rx;
};
static struct hal_s hal = {0};
//*****************************************************************************
//
// Implementations of function that are required by usb framework.
//
//*****************************************************************************
void usbsuspHookEnteringSuspend(bool remoteWakeupAllowed) {
    if (remoteWakeupAllowed) {
    }
}


void usbsuspHookExitingSuspend(void) {
}


   
   
/******************************************************************************
* FUNCTIONS
*/

static void tap_cb(unsigned char direction, unsigned char count)
{

    return;
}

static void android_orient_cb(unsigned char orientation)
{
	switch (orientation) {

	default:
		return;
	}
}


void
Timer1AIntHandler(void)
{
    //
    // Clear the timer interrupt flag.
    //
     TimerIntClear(GPTIMER1_BASE, GPTIMER_CAPA_EVENT );
   // mimsyLedToggle(GPIO_PIN_4);
    gpio_state=GPIOPinRead(GPIO_D_BASE,GPIO_PIN_1);
   // GPIOPinWrite(GPIO_D_BASE,GPIO_PIN_1,~gpio_state& GPIO_PIN_1);


}
void
Timer1BIntHandler(void)
{
    //
    // Clear the timer interrupt flag.
    //
     TimerIntClear(GPTIMER1_BASE, GPTIMER_CAPB_EVENT);
   // mimsyLedToggle(GPIO_PIN_7);
    gpio_state=GPIOPinRead(GPIO_D_BASE,GPIO_PIN_2);
    //GPIOPinWrite(GPIO_D_BASE,GPIO_PIN_2,~gpio_state& GPIO_PIN_2);
    gpio_state=TimerValueGet(GPTIMER1_BASE,GPTIMER_B);

   // TimerEnable(GPTIMER1_BASE,GPTIMER_B);

}

/**************************************************************************//**
* @brief    Main function of example.
******************************************************************************/
void main(void)
{
    volatile uint32_t ui32Loop;
    SysCtrlClockSet(false,false,SYS_CTRL_SYSDIV_32MHZ);
    
        //
    // Set IO clock to the same as system  clock
    //
    SysCtrlIOClockSet(SYS_CTRL_SYSDIV_32MHZ);
   
    
    
    //
    // Init LEDs (turned off)
    //
    IntMasterDisable(); //disable interrupts for initializations

    mimsyLedInit();
    //
    // Turn on LED1 and LED4
    //
    mimsyLedSet(GPIO_PIN_7|GPIO_PIN_4);
    mimsyLedClear(GPIO_PIN_7|GPIO_PIN_4);
    
    //create inchworm motor strucutures
    //AMAYBE TODO: add a contstructor for these motors for  where you only have to put in the mimsy pinout (GP1, GP2, etc...) for this to be generated
    InchwormMotor motor0={
      .GPIObase1=GPIO_D_BASE,  //mapping for pin1 gpio base
      .GPIObase2=GPIO_D_BASE,   //mapping for pin 2 base
      .GPIOpin1=GPIO_PIN_1,     //mapping for pin 1 pin
      .GPIOpin2=GPIO_PIN_2,     //mapping for pin2 pin
      .motorID=0     
    };
    
    InchwormMotor motor1={
      .GPIObase1=GPIO_A_BASE,  //mapping for pin1 gpio base
      .GPIObase2=GPIO_A_BASE,   //mapping for pin 2 base
      .GPIOpin1=GPIO_PIN_2,     //mapping for pin 1 pin
      .GPIOpin2=GPIO_PIN_5,            //mapping for pin2 pin
      .motorID=1
    };
    
    InchwormMotor motorList[2] ={motor0,motor1}; //array of motors to be passed to inchworm setup
    
//create inchworm structure #TODO: change inchworm struct so it contains all 4 inchworm motor pin mappings 
    InchwormSetup inchworm0 = {
      .iwMotors=motorList,
      .numOfMotors=2,
      .motorFrequency=1000,
      .dutyCycle=80,
      .motorID=0,
      .timer=1,
      .phaseTimer=0
    };
    
    //intialize inchworms
    inchwormInit(inchworm0);

   
    // re-enable interrupts

   refptr = &refCard;
   IntMasterEnable();

   //////////////////////////////////////////////////////////////////
mimsyIMUInit();
     ///////////////////////////////////////////////////////////////


     
uint8_t *byteptr=&readbyte;
uint8_t address = 0x69;
      

     //full scale set
              i2c_write_byte(address,0x1C);
     i2c_write_byte(address,0x18);
     
             i2c_write_byte(address,0x1C);
     i2c_read_byte(address,byteptr);
     

//uart init found in uart_mimsy.c
     uartMimsyInit();
     

         i2c_write_byte(address,MPU9250_ACCEL_CONFIG);
     i2c_write_byte(address,0x18);  
     
     uint8_t list[2]={MPU9250_ACCEL_CONFIG,0x18};
     
     //i2c_write_bytes(address,list,2);
     i2c_write_register_8bit(address,MPU9250_ACCEL_CONFIG,0x18);//config full scale range
      i2c_write_register_8bit(address,MPU9250_GYRO_CONFIG,0x18);//config full scale range
     
      
      
    i2c_write_byte(address,MPU9250_ACCEL_CONFIG);
     i2c_read_byte(address,byteptr);  
      mimsyPrintf("Register Value: %x",readbyte);
       uint8_t byte=0x00;
     //  i2c_write_registers(address,MPU9250_ACCEL_CONFIG,1,&byte);
        mpu_read_reg(MPU9250_ACCEL_CONFIG,byteptr);
       
       i2c_write_byte(address,MPU9250_FIFO_EN);
     i2c_read_byte(address,byteptr);  
     mimsyPrintf("Register Value: %x",readbyte);
     
       
bool stopLogging=false;
bool triggered=false;    
    IMUDataCard cards_stable[100];
    
    for(int i=0;i<100;i++){
      (cards_stable[i].page)=FLASH_PAGE_STORAGE_START+i;
    }
    mimsyLedClear(RED_LED);   
    mimsyLedSet(GREEN_LED);
   
    
    
    struct int_param_s intparam; //need an empty struct
    mpu_init(&intparam);

        

    mpu_set_sensors(INV_XYZ_ACCEL|INV_XYZ_GYRO|INV_XYZ_COMPASS); //turn on sensor
    mpu_set_accel_fsr(16); //set fsr for accel
       mimsyPrintf("reg value: %x",readbyte);
     unsigned  short xl[6];
    long debugx;
   
//dmp stuff
   // inv_init_mpl();
  //  inv_enable_quaternion();
  //  inv_enable_9x_sensor_fusion();
    
    //start dmp with 6axis fusion
    mimsyDmpBegin();
    
    short gyro[3];
    short accel[3];
    long quat[4];
    long rot[3];
    long timestamp2;
    unsigned char more;
    short sensors=INV_XYZ_GYRO | INV_WXYZ_QUAT|INV_XYZ_ACCEL;
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    
   
    while(1)
    {
      dmp_read_fifo(gyro, accel, quat,&timestamp2, &sensors, &more);
      mpu_get_accel_reg(xl,&debugx);
      mimsyIMURead6Dof(&debug4);
      data[bufferCount]=debug4;
      bufferCount++;
     // i2c_read_registers(address,MPU9250_FIFO_R_W,14,imuraw.bytes);
     // i2c_read_registers(address,MPU9250_ACCEL_XOUT_H,14,imuraw.bytes);
     
      if(!triggered){
        if((debug4.signedfields.accelY<-3*32768/16 ||debug4.signedfields.accelY>3*32767/16)||(debug4.signedfields.accelX<-3*32768/16 ||debug4.signedfields.accelX>3*32767/16)  ||(debug4.signedfields.accelZ<-3*32768/16 ||debug4.signedfields.accelZ>3*32767/16) ){
          mimsyLedSet(RED_LED);
         // triggered=true;
        //  UARTprintf("\n Accel X: %d, Accel Y: %d, Accel Z: %d ",debug4.signedfields.accelX,debug4.signedfields.accelY,debug4.signedfields.accelZ);
        }
      }

      if(bufferCount==128){
        if(!triggered){
          //UARTprintf("%c[2K",27);
        //  UARTprintf("\n Accel X: %d, Accel Y: %d, Accel Z: %d ",debug4.signedfields.accelX,debug4.signedfields.accelY,debug4.signedfields.accelZ);
        //  UARTprintf(" Gyro X: %d, Gyro Y: %d, Gyro Z: %d ",debug4.signedfields.gyroX,debug4.signedfields.gyroY,debug4.signedfields.gyroZ);
         // UARTprintf(", Timestamp: %x",debug4.fields.timestamp);
          mimsyPrintf("\n Quaternions:%d,%d,%d,%d",quat[0],quat[1],quat[2],quat[3]);
        }
        bufferCount=0;
        
        if(triggered && !stopLogging){
          IMUDataCard *card=&(cards[pagesWritten]);
          flashWriteIMU(data,128,currentflashpage,card);
         
          pagesWritten++;
          currentflashpage++;
        }
      }
      

      
      if(pagesWritten==FLASH_PAGES_TOUSE&&!stopLogging){
        stopLogging=true;
        mimsyLedClear(GREEN_LED);
      } 
      while(stopLogging){
        UARTprintf("\n data starts here:+ \n"); //+ is start condition
        for(int cardindex=0;cardindex<FLASH_PAGES_TOUSE;cardindex++){
          IMUData sendData[128];
          flashReadIMU(cards_stable[cardindex],sendData,128);
        
          //loop through each data point
          for(int dataindex=0;dataindex<128;dataindex++){
            
   
          
              //print csv data to serial
              //format: xl_x,xl_y,xl_z,gyrox,gyroy,gyroz,timestamp
            UARTprintf("%d,%d,%d,%d,%d,%d,%x,%d,%d \n",
                          sendData[dataindex].signedfields.accelX,
                          sendData[dataindex].signedfields.accelY,
                          sendData[dataindex].signedfields.accelZ,
                          sendData[dataindex].signedfields.gyroX,
                          sendData[dataindex].signedfields.gyroY,
                          sendData[dataindex].signedfields.gyroZ,
                          sendData[dataindex].fields.timestamp,
                          cardindex,
                          dataindex);

            
          }
          
        }
        UARTprintf("= \n data ends here\n"); //= is end
          for(ui32Loop=1;ui32Loop<500000;ui32Loop++) {
          }
        }
 

  
      
      
      /*// printf(debug4.fields.accelx,'%d')
       inchwormDriveToPosition(motor0,1000);


//wait
     for(ui32Loop=1;ui32Loop<50000;ui32Loop++) {
    }
    inchwormRelease(motor0);
//enables iws
//    inchwormFreerun(motor0);
 //    inchwormFreerun(motor1);

 //wait   
      
    for(ui32Loop=1;ui32Loop<50000;ui32Loop++) {
    }
    }

*/
}

}
