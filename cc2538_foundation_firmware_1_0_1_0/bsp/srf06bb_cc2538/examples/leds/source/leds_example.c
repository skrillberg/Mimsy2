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
#include "bsp.h"
#include "bsp_led.h"
#include "gptimer.h"
#include "sys_ctrl.h"
#include "hw_gptimer.h"
#include "hw_ints.h"
#include "leds_example.h"
#include "interrupt.h"
#include "C:\Users\kilberg\Documents\Mimsy2\iar\Applications\Mimsy2_BSP_drivers\bsp\led.h" 

/******************************************************************************
* DEFINES
*/


/******************************************************************************
* LOCAL VARIABLES AND FUNCTIONS
*/
  uint32_t timer;
  uint32_t load;
  uint32_t x=5;
  uint32_t y=5;
  uint32_t z=5;
/******************************************************************************
* FUNCTIONS
*/
void
Timer1AIntHandler(void)
{
    //
    // Clear the timer interrupt flag.
    //
     TimerIntClear(GPTIMER1_BASE, GPTIMER_TIMA_TIMEOUT);
    mimsyLedToggle(GPIO_PIN_4);


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
    GPIOPinTypeGPIOOutput(GPIO_C_BASE,GPIO_PIN_4);


    //
    // Turn on LED1 and LED4
    //
    mimsyLedSet(GPIO_PIN_7|GPIO_PIN_4);
    mimsyLedClear(GPIO_PIN_7|GPIO_PIN_4);
    
        //config timer for pwm 
    
    
    SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_GPT1);
    TimerConfigure(GPTIMER1_BASE,GPTIMER_CFG_PERIODIC); //configures timer 0a as periodic
    
  
       
      //interrupts
    TimerIntRegister(GPTIMER1_BASE, GPTIMER_A, Timer1AIntHandler);      
    
    //
    // Enable processor interrupts.
    //
    IntMasterEnable();

    //
    // Configure the Timer0A interrupt for timer timeout.
    //
    TimerIntEnable(GPTIMER1_BASE, GPTIMER_TIMA_TIMEOUT);

    //
    // Enable the Timer0B interrupt on the processor (NVIC).
    //
    IntEnable(INT_TIMER1A);
       
    TimerEnable(GPTIMER1_BASE,GPTIMER_A);
    
    load=TimerLoadGet(GPTIMER1_BASE,GPTIMER_A);
    printf("%d",load);
    //
    // Infinite loop
    //
    while(1)
    {
      
//      timer=TimerValueGet(GPTIMER1_BASE,GPTIMER_A);
//      //printf("%d",timer);
//      load=TimerLoadGet(GPTIMER1_BASE,GPTIMER_A);
//      if(timer<2500000)
//      {
//       mimsyLedSet(GPIO_PIN_4);
//      }else
//      {
//        mimsyLedClear(GPIO_PIN_4);
//      }
      
        //
        // Toggle LED2 and LED3
        //
            //
    // Get current pin values of selected bits
    //
  //  uint32_t ui32Toggle = GPIOPinRead(GPIO_C_BASE, GPIO_PIN_4);

    //
    // Invert selected bits
    //
   // ui32Toggle = (~ui32Toggle) & GPIO_PIN_4;

    //
    // Set GPIO
    //
   // GPIOPinWrite(GPIO_C_BASE, GPIO_PIN_4, ui32Toggle);
        

        //
        // Simple wait
        //
        //for(ui32Loop = 0; ui32Loop < 500000; ui32Loop++)
       // {
        //}
    }

    
}
