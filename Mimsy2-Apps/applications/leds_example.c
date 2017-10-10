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

/******************************************************************************
* DEFINES
*/
#define GP4 

/******************************************************************************
* LOCAL VARIABLES AND FUNCTIONS
*/
  uint32_t timer;
  uint32_t load;
  uint32_t x=5;
  uint32_t y=5;
  uint32_t z=5;
  volatile uint32_t gpio_state;
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
    IntMasterDisable();
    GPIOPinTypeGPIOOutput(GPIO_C_BASE,GPIO_PIN_4|GPIO_PIN_7);
    //GPIOPinTypeGPIOOutput(GPIO_D_BASE,GPIO_PIN_1|GPIO_PIN_2);
    //   GPIOPinWrite(GPIO_D_BASE,GPIO_PIN_1|GPIO_PIN_2,GPIO_PIN_1);
    gpio_state=GPIO_PIN_1;
    //
    // Turn on LED1 and LED4
    //
    mimsyLedSet(GPIO_PIN_7|GPIO_PIN_4);
    mimsyLedClear(GPIO_PIN_7|GPIO_PIN_4);
    
        //config timer for pwm 
    
    
    SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_GPT1);
    TimerConfigure(GPTIMER1_BASE, GPTIMER_CFG_SPLIT_PAIR |GPTIMER_CFG_A_PWM | GPTIMER_CFG_B_PWM); //configures timer 1a as periodic half
    //HWREG(GPTIMER1_BASE + GPTIMER_O_TBMR)= HWREG(GPTIMER1_BASE + GPTIMER_O_TBMR)|64; //configures b to trigger on A timeout
    
    //set output pins for pwm//////////////////////////////
    GPIOPinTypeTimer(GPIO_D_BASE,GPIO_PIN_1|GPIO_PIN_2);
    gpio_state=IOCPadConfigGet(GPIO_D_BASE,GPIO_PIN_1);
    IOCPadConfigSet(GPIO_D_BASE,GPIO_PIN_1|GPIO_PIN_2,IOC_OVERRIDE_OE);
    IOCPinConfigPeriphOutput(GPIO_D_BASE,GPIO_PIN_1,IOC_MUX_OUT_SEL_GPT1_ICP1);
    IOCPinConfigPeriphOutput(GPIO_D_BASE,GPIO_PIN_2,IOC_MUX_OUT_SEL_GPT1_ICP2);

    
    //set pwm polarities to be opposite
    TimerControlLevel(GPTIMER1_BASE,GPTIMER_A,true); 
    TimerControlLevel(GPTIMER1_BASE,GPTIMER_B,false);
    
    //set pwm duty cycles
    TimerMatchSet(GPTIMER1_BASE,GPTIMER_A,25000);
    TimerMatchSet(GPTIMER1_BASE,GPTIMER_B,20000);
       
      //interrupts
    TimerIntClear(GPTIMER1_BASE, GPTIMER_TIMB_TIMEOUT);
    TimerIntClear(GPTIMER1_BASE, GPTIMER_TIMA_TIMEOUT);
    
    TimerIntRegister(GPTIMER1_BASE, GPTIMER_A, Timer1AIntHandler);      
    TimerIntRegister(GPTIMER1_BASE, GPTIMER_B, Timer1BIntHandler);      
    
    //
    // Enable processor interrupts.
    //
    
     TimerLoadSet(GPTIMER1_BASE,GPTIMER_A,50000);
     TimerLoadSet(GPTIMER1_BASE,GPTIMER_B,50000);
     load=TimerLoadGet(GPTIMER1_BASE,GPTIMER_A);
     load=TimerLoadGet(GPTIMER1_BASE,GPTIMER_B);
    //
    // Configure the Timer0A interrupt for timer timeout.
    //
    TimerIntEnable(GPTIMER1_BASE, GPTIMER_CAPA_EVENT| GPTIMER_CAPB_EVENT);
    TimerControlEvent(GPTIMER1_BASE,GPTIMER_BOTH,GPTIMER_EVENT_POS_EDGE);
    //TimerIntEnable(GPTIMER1_BASE, GPTIMER_TIMB_TIMEOUT);


    //
    // Enable the Timer0B interrupt on the processor (NVIC).
    //
    IntEnable(INT_TIMER1A);
    IntEnable(INT_TIMER1B);
    
   
   // TimerLoadSet(GPTIMER1_BASE,GPTIMER_B,500000);
    
    TimerEnable(GPTIMER1_BASE,GPTIMER_BOTH);
   // TimerEnable(GPTIMER1_BASE,GPTIMER_B);
    
    load=TimerLoadGet(GPTIMER1_BASE,GPTIMER_A);
    printf("%d",load);
    IntMasterEnable();
    //
    // Infinite loop
    //
    while(1)
    {
      
      timer=TimerValueGet(GPTIMER1_BASE,GPTIMER_A);
      timer=TimerValueGet(GPTIMER1_BASE,GPTIMER_B);

    }

    
}
