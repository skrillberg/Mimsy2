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
#include "uartstdio.h"
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

#define EXAMPLE_PIN_UART_RXD            GPIO_PIN_2 
#define EXAMPLE_PIN_UART_TXD            GPIO_PIN_1 
#define EXAMPLE_GPIO_BASE               GPIO_D_BASE
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
  //  GPIOPinTypeGPIOOutput(GPIO_C_BASE,GPIO_PIN_4|GPIO_PIN_7);
    //GPIOPinTypeGPIOOutput(GPIO_D_BASE,GPIO_PIN_1|GPIO_PIN_2);
    //   GPIOPinWrite(GPIO_D_BASE,GPIO_PIN_1|GPIO_PIN_2,GPIO_PIN_1);
  //  gpio_state=GPIO_PIN_1;
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

    //
    // Infinite loop
    //
    //IMUData imu;
    
     
   imu.fields.accelX=1;//a X data
   imu.fields.accelY=2;//a X data
   imu.fields.accelZ=3;//a X data
  
   imu.fields.gyroX=4;//gyro X data
   imu.fields.gyroY=5;//gyro Y data
   imu.fields.gyroZ=6;//gyro Z data
   
   imu.fields.timestamp=1;
   
   debug=sizeof(data);
   for(uint32_t k=0; k<sizeof(data)/16;k++){
     
   data[k].fields.accelX=6*k+1;//a X data
   data[k].fields.accelY=6*k+2;//a X data
   data[k].fields.accelZ=6*k+3;//a X data
  
   data[k].fields.gyroX=6*k+4;//gyro X data
   data[k].fields.gyroY=6*k+5;//gyro Y data
   data[k].fields.gyroZ=6*k+6;//gyro Z data
   
   data[k].fields.timestamp=k;

   }
   refptr = &refCard;
   IntMasterEnable();
  // flashWriteIMU(data,sizeof(data)/16,14,refptr);
       // for(ui32Loop=1;ui32Loop<50000;ui32Loop++) {
  //  }
   //flashReadIMU(refCard,flashData,sizeof(flashData)/16);
   board_timer_init();
    debug=sizeof(imu);
    
    
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
     
     //fifo enable
     
    //          i2c_write_byte(address,MPU9250_USER_CTRL);
    // i2c_write_byte(address,0x40);
     
        // i2c_write_byte(address,MPU9250_FIFO_EN);
   //  i2c_write_byte(address,0x78); //enalbe gyro and accel fifo writes
    


     
     
     readbyte=1;
   
      i2c_write_byte(address,MPU9250_WHO_AM_I);
i2c_read_byte(address,byteptr);
      
     i2c_write_byte(address,0x3B);
     i2c_read_byte(address,byteptr);
     debug4.fields.accelX=((uint16_t) readbyte)<<8;
    
     i2c_write_byte(address,0x3C);
     i2c_read_byte(address,byteptr);
      debug4.fields.accelX=((uint16_t) readbyte)| debug4.fields.accelX;
     
     i2c_write_byte(address,0x3F);
     i2c_read_byte(address,byteptr);
    //  debug4.fields.accelZ[15:8]=readbyte;
          i2c_write_byte(address,0x40);
     i2c_read_byte(address,byteptr);
    //  debug4.fields.accelZ[7:0]=readbyte;
      
              i2c_write_byte(address,0x1C);
     i2c_write_byte(address,0x18);
     
             i2c_write_byte(address,0x1C);
     i2c_read_byte(address,byteptr);
     
          i2c_write_byte(address,0x1C);
     i2c_read_byte(address,byteptr);
     


////////////////////////////////////////////////////////////////uart
       char cThisChar;

    //
    // Set the clocking to run directly from the external crystal/oscillator.
    // (no ext 32k osc, no internal osc)
    //
   // SysCtrlClockSet(false, false, SYS_CTRL_SYSDIV_32MHZ);


    //
    // Enable UART peripheral module
    //
    SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_UART0);

    //
    // Disable UART function
    //
    UARTDisable(UART0_BASE);

    //
    // Disable all UART module interrupts
    //
    UARTIntDisable(UART0_BASE, 0x1FFF);

    //
    // Set IO clock as UART clock source
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Map UART signals to the correct GPIO pins and configure them as
    // hardware controlled.
    //
    IOCPinConfigPeriphOutput(EXAMPLE_GPIO_BASE, EXAMPLE_PIN_UART_TXD, IOC_MUX_OUT_SEL_UART0_TXD);
    GPIOPinTypeUARTOutput(EXAMPLE_GPIO_BASE, EXAMPLE_PIN_UART_TXD); 
    IOCPinConfigPeriphInput(EXAMPLE_GPIO_BASE, EXAMPLE_PIN_UART_RXD, IOC_UARTRXD_UART0);
    GPIOPinTypeUARTInput(EXAMPLE_GPIO_BASE, EXAMPLE_PIN_UART_RXD);
     
    //
    // Configure the UART for 115,200, 8-N-1 operation.
    // This function uses SysCtrlClockGet() to get the system clock
    // frequency.  This could be also be a variable or hard coded value
    // instead of a function call.
    //
   // UARTConfigSetExpClk(UART0_BASE, SysCtrlClockGet(), 115200,
                     //   (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                     //    UART_CONFIG_PAR_NONE));
  //  UARTEnable(UART0_BASE);
    UARTStdioInit(0);
    //
    // Put a character to show start of example.  This will display on the
    // terminal.
    //
      UARTwrite("hello world",11);
    UARTCharPut(UART0_BASE, '!');
         i2c_write_byte(address,MPU9250_ACCEL_CONFIG);
     i2c_write_byte(address,0x18);  
     
     uint8_t list[2]={MPU9250_ACCEL_CONFIG,0x18};
     
     //i2c_write_bytes(address,list,2);
     i2c_write_register_8bit(address,MPU9250_ACCEL_CONFIG,0x18);//config full scale range
      i2c_write_register_8bit(address,MPU9250_GYRO_CONFIG,0x18);//config full scale range
     
      
      
    i2c_write_byte(address,MPU9250_ACCEL_CONFIG);
     i2c_read_byte(address,byteptr);  
       UARTprintf("Register Value: %x",readbyte);
       uint8_t byte=0x00;
     //  i2c_write_registers(address,MPU9250_ACCEL_CONFIG,1,&byte);
        mpu_read_reg(MPU9250_ACCEL_CONFIG,byteptr);
       
       i2c_write_byte(address,MPU9250_FIFO_EN);
     i2c_read_byte(address,byteptr);  
       UARTprintf("Register Value: %x",readbyte);
     
       
bool stopLogging=false;
bool triggered=false;    
    IMUDataCard cards_stable[100];
    
    for(int i=0;i<100;i++){
      (cards_stable[i].page)=FLASH_PAGE_STORAGE_START+i;
    }
    mimsyLedClear(RED_LED);   
    mimsyLedSet(GREEN_LED);
    struct int_param_s intparam;
    mpu_init(&intparam);

        

    mpu_set_sensors(INV_XYZ_ACCEL|INV_XYZ_GYRO);

       UARTprintf("reg value: %x",readbyte);
     unsigned  short xl[6];
    long debugx;
   
//dmp stuff
    inv_init_mpl();
      dmp_load_motion_driver_firmware();
    dmp_set_orientation(
       inv_orientation_matrix_to_scalar(gyro_pdata.orientation));
          dmp_register_tap_cb(tap_cb);
    dmp_register_android_orient_cb(android_orient_cb);
    
        hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
        DMP_FEATURE_GYRO_CAL;
    dmp_enable_feature(hal.dmp_features);
    dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    mpu_set_dmp_state(1);
    hal.dmp_on = 1;
    short gyro;
    short accel;
    long quat;
    long timestamp2;
    unsigned char more;
    short sensors=INV_XYZ_GYRO | INV_WXYZ_QUAT|INV_XYZ_ACCEL;
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    
   
    while(1)
    {
      dmp_read_fifo(&gyro, &accel, &quat,&timestamp2, &sensors, &more);
      mpu_get_accel_reg(xl,&debugx);
      mimsyIMURead6Dof(address,&debug4);
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
          UARTprintf("%c[2K",27);
          UARTprintf("\n Accel X: %d, Accel Y: %d, Accel Z: %d ",debug4.signedfields.accelX,debug4.signedfields.accelY,debug4.signedfields.accelZ);
          UARTprintf(" Gyro X: %d, Gyro Y: %d, Gyro Z: %d ",debug4.signedfields.gyroX,debug4.signedfields.gyroY,debug4.signedfields.gyroZ);
          UARTprintf(", Timestamp: %x",debug4.fields.timestamp);
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
