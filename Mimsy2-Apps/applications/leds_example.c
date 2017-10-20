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
#include <stdio.h>
//#include "bsp.h"
//#include "bsp_led.h"
#include "gptimer.h"
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

/******************************************************************************
* DEFINES
*/
#define GP4 
#define FREQ_CNT SysCtrlClockGet()/frequency 
#define OVERLAP_MATCH (100-dutycycle)*SysCtrlClockGet()/frequency/100 

#define PAGE_SIZE                2048
#define PAGE_TO_ERASE            14
#define PAGE_TO_ERASE_START_ADDR (FLASH_BASE + (PAGE_TO_ERASE * PAGE_SIZE))

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
   IMUData data[3];
   IMUDataCard refCard;
   IMUData flashData[2048/16];
   IMUDataCard * refptr;
/******************************************************************************
* FUNCTIONS
*/
void
Timer1AIntHandler(void)
{
    //
    // Clear the timer interrupt flag.
    //
     TimerIntClear(GPTIMER1_BASE, GPTIMER_CAPA_EVENT );
    mimsyLedToggle(GPIO_PIN_4);
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
    mimsyLedToggle(GPIO_PIN_7);
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

    //
    // Init LEDs (turned off)
    //
    IntMasterDisable(); //disable interrupts for initializations
    GPIOPinTypeGPIOOutput(GPIO_C_BASE,GPIO_PIN_4|GPIO_PIN_7);
    //GPIOPinTypeGPIOOutput(GPIO_D_BASE,GPIO_PIN_1|GPIO_PIN_2);
    //   GPIOPinWrite(GPIO_D_BASE,GPIO_PIN_1|GPIO_PIN_2,GPIO_PIN_1);
    gpio_state=GPIO_PIN_1;
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
  IntMasterEnable();
   
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
   flashWriteIMU(data,sizeof(data)/16,14,refptr);
   flashReadIMU(refCard,flashData,sizeof(flashData)/16);
   
    debug=sizeof(imu);
    
    
   
    for(uint32_t v=0;v<sizeof(imu.bits);v++){
    debug4.bits[v] = imu.bits[v];
    }
    debug=debug4.fields.timestamp;
    
    
    
   
    while(1)
    {
      

       inchwormDriveToPosition(motor0,1000);
//disables iws
      //inchwormRelease(motor0);
       //     inchwormRelease(motor1);

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

    
}
