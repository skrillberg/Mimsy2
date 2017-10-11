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

/*GLOBALS
*/
volatile uint32 inchwormTimer=0;
/******************************************************************************
* DEFINES
*/
#define GP4 
#define FREQ_CNT SysCtrlClockGet()/frequency 
#define OVERLAP_MATCH (100-dutycycle)*SysCtrlClockGet()/frequency/100 


/*********************************************************************************
*STRUCTURES
*/


/******************************************************************************
* FUNCTIONS
*/
void
PwmTimerAIntHandler(void)
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
PwmTimerBIntHandler(void)
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


void inchwormInit(struct Inchworms motor){
  uint32 freqCnt=SysCtrlClockGet()/motor.motorFrequency;
  uint32 match=(100-motor.dutyCycle)*SysCtrlClockGet()/motor.motorFrequency/100 ;
  uint32 pwmTimerClkEnable;
  uint32 pwmTimerBase;
  uint32 timerIntA;
  uint32 timerIntB;

  //find timer modules used
  switch(motor.timer){
      
  case 0: 
    pwmTimerClkEnable=SYS_CTRL_PERIPH_GPT0;
    pwmTimerBase=GPTIMER0_BASE;
    timerIntA=TIMER0A;
    timerIntB=TIMER0B;
    break;

  case 1: 
    pwmTimerClkEnable=SYS_CTRL_PERIPH_GPT1;
    pwmTimerBase=GPTIMER1_BASE;
    timerIntA=TIMER1A;
    timerIntB=TIMER1B;
    break;
    
  case 2: 
    pwmTimerClkEnable=SYS_CTRL_PERIPH_GPT2;
    pwmTimerBase=GPTIMER2_BASE;
    timerIntA=TIMER2A;
    timerIntB=TIMER2B;
    break;
    
  case 3: 
    pwmTimerClkEnable=SYS_CTRL_PERIPH_GPT3;
    pwmTimerBase=GPTIMER3_BASE;
    timerIntA=TIMER3A;
    timerIntB=TIMER3B;
    break;    
    
    
  }
  
    inchwormTimer=pwmTimerBase;//updates global value of current inchworm timer so interrupt knows what to do.
  
  
    SysCtrlPeripheralEnable(pwTimerClkEnable); //enables timer module
   
    
    TimerConfigure(pwmTimerBase, GPTIMER_CFG_SPLIT_PAIR |GPTIMER_CFG_A_PWM | GPTIMER_CFG_B_PWM); //configures timers as pwm timers
  
    
    TimerControlWaitOnTrigger( pwmTimerBase,GPTIMER_A,true); //configures 1a as a wait on trigger timer
    TimerConfigure(pwmTimerBase,GPTIMER_CFG_ONE_SHOT); //timer 0b configured as a one shot timer. this will be used to daisy chain start timer 1a 
   
    
    TimerLoadSet(pwmTimerBase,GPTIMER_A,freqCnt); //1a load
    TimerLoadSet(pwmTimerBase,GPTIMER_B,freqCnt); //1b load
  //  load=TimerLoadGet(pwmTimerBase,GPTIMER_A);

    
    //set output pins for pwm//////////////////////////////
    GPIOPinTypeTimer(motor.GPIObase1,motor.GPIOpin1); //enables hw muxing of pin outputs
    GPIOPinTypeTimer(motor.GPIObase2,motor.GPIOpin2); //enables hw muxing of pin outputs

        
    //gpio_state=IOCPadConfigGet(GPIO_D_BASE,GPIO_PIN_1); 
    IOCPadConfigSet(motor.GPIObase1,motor.GPIOpin1,IOC_OVERRIDE_OE|IOC_OVERRIDE_PUE); // enables pins as outputs, necessary for this code to work correctly
        IOCPadConfigSet(motor.GPIObase2,motor.GPIOpin2,IOC_OVERRIDE_OE|IOC_OVERRIDE_PUE); // enables pins as outputs, necessary for this code to work correctly
   
    IOCPinConfigPeriphOutput(motor.GPIObase1,motor.GPIOpin1,IOC_MUX_OUT_SEL_GPT1_ICP1); //maps pwm1 output to pin1
    IOCPinConfigPeriphOutput(motor.GPIObase2,motor.GPIOpin2,IOC_MUX_OUT_SEL_GPT1_ICP2); //maps pwm2 output to pin2

    
    //set pwm polarities 
    TimerControlLevel(pwmTimerBase,GPTIMER_A,true); //active high pwm
    TimerControlLevel(pwmTimerBase,GPTIMER_B,true); //active high pwm
    
    //set pwm duty cycles
    TimerMatchSet(pwmTimerBase,GPTIMER_A,match);
    TimerMatchSet(pwmTimerBase,GPTIMER_B,match);
       
      //interrupts
    TimerIntClear(pwmTimerBase, GPTIMER_TIMB_TIMEOUT);
    TimerIntClear(pwmTimerBase, GPTIMER_TIMA_TIMEOUT);
    
    TimerIntRegister(pwmTimerBase, GPTIMER_A, PwmTimerAIntHandler);       //sets timer a interrupt handler
    TimerIntRegister(pwmTimerBase, GPTIMER_B, PwmTimerBIntHandler);      //sets timer 1b interrupt handler
    
    //
    // Enable processor interrupts.
    //
    

    //
    // enable interrupts for pos edge pwm 
    //
    TimerIntEnable(pwmTimerBase, GPTIMER_CAPA_EVENT| GPTIMER_CAPB_EVENT);
    TimerControlEvent(pwmTimerBase,GPTIMER_BOTH,GPTIMER_EVENT_POS_EDGE);
    //TimerIntEnable(GPTIMER1_BASE, GPTIMER_TIMB_TIMEOUT);


    //
    // Enable the Timer interrupts on the processor (NVIC).
    //
    IntEnable(timerIntA);
    IntEnable(timerIntB);
}

void inchwormDisable(void){
      TimerDisable(GPTIMER1_BASE,GPTIMER_B);

    TimerDisable(GPTIMER1_BASE,GPTIMER_A);
    GPIOPinTypeGPIOOutput(GPIO_D_BASE,GPIO_PIN_1|GPIO_PIN_2);
    GPIOPinWrite(GPIO_D_BASE,GPIO_PIN_1|GPIO_PIN_2,255);
}

void inchwormEnable(void){
      TimerEnable(GPTIMER1_BASE,GPTIMER_B);
    for(ui32Loop=1;ui32Loop<FREQ_CNT/2;ui32Loop++) {
    }
    TimerEnable(GPTIMER1_BASE,GPTIMER_A);
    TimerEnable(GPTIMER0_BASE,GPTIMER_A);

}

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
    
        //config timers for pwm 
    
    
    SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_GPT1); //enables timer 1 module
    SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_GPT0); //enables timer 0 module
    
    TimerConfigure(GPTIMER1_BASE, GPTIMER_CFG_SPLIT_PAIR |GPTIMER_CFG_A_PWM | GPTIMER_CFG_B_PWM); //configures timer 1ab as pwm timers
  
    
    TimerControlWaitOnTrigger( GPTIMER1_BASE,GPTIMER_A,true); //configures 1a as a wait on trigger timer
    TimerConfigure(GPTIMER0_BASE,GPTIMER_CFG_ONE_SHOT); //timer 0b configured as a one shot timer. this will be used to daisy chain start timer 1a 
   
    TimerLoadSet(GPTIMER0_BASE,GPTIMER_A,5000); //loads a timer value into 1b
    TimerLoadSet(GPTIMER1_BASE,GPTIMER_A,FREQ_CNT); //1a load
    TimerLoadSet(GPTIMER1_BASE,GPTIMER_B,FREQ_CNT); //1b load
    load=TimerLoadGet(GPTIMER1_BASE,GPTIMER_A);
    load=TimerLoadGet(GPTIMER0_BASE,GPTIMER_B);
    
    //set output pins for pwm//////////////////////////////
    GPIOPinTypeTimer(GPIO_D_BASE,GPIO_PIN_1|GPIO_PIN_2); //enables hw muxing of pin outputs
    //gpio_state=IOCPadConfigGet(GPIO_D_BASE,GPIO_PIN_1); 
    IOCPadConfigSet(GPIO_D_BASE,GPIO_PIN_1|GPIO_PIN_2,IOC_OVERRIDE_OE|IOC_OVERRIDE_PUE); // enables pins as outputs, necessary for this code to work correctly
   
    IOCPinConfigPeriphOutput(GPIO_D_BASE,GPIO_PIN_1,IOC_MUX_OUT_SEL_GPT1_ICP1); //maps cp1 to gpio1
    IOCPinConfigPeriphOutput(GPIO_D_BASE,GPIO_PIN_2,IOC_MUX_OUT_SEL_GPT1_ICP2); //maps cp2 to gpio2

    
    //set pwm polarities 
    TimerControlLevel(GPTIMER1_BASE,GPTIMER_A,true); //active high pwm
    TimerControlLevel(GPTIMER1_BASE,GPTIMER_B,true); //active high pwm
    
    //set pwm duty cycles
    TimerMatchSet(GPTIMER1_BASE,GPTIMER_A,OVERLAP_MATCH);
    TimerMatchSet(GPTIMER1_BASE,GPTIMER_B,OVERLAP_MATCH);
       
      //interrupts
    TimerIntClear(GPTIMER1_BASE, GPTIMER_TIMB_TIMEOUT);
    TimerIntClear(GPTIMER1_BASE, GPTIMER_TIMA_TIMEOUT);
    
    TimerIntRegister(GPTIMER1_BASE, GPTIMER_A, Timer1AIntHandler);       //sets timer a interrupt handler
    TimerIntRegister(GPTIMER1_BASE, GPTIMER_B, Timer1BIntHandler);      //sets timer 1b interrupt handler
    
    //
    // Enable processor interrupts.
    //
    

    //
    // enable interrupts for pos edge pwm 
    //
    TimerIntEnable(GPTIMER1_BASE, GPTIMER_CAPA_EVENT| GPTIMER_CAPB_EVENT);
    TimerControlEvent(GPTIMER1_BASE,GPTIMER_BOTH,GPTIMER_EVENT_POS_EDGE);
    //TimerIntEnable(GPTIMER1_BASE, GPTIMER_TIMB_TIMEOUT);


    //
    // Enable the Timer interrupts on the processor (NVIC).
    //
    IntEnable(INT_TIMER1A);
    IntEnable(INT_TIMER1B);
    
   
   // TimerLoadSet(GPTIMER1_BASE,GPTIMER_B,500000);
    
    //timer enables
    TimerEnable(GPTIMER1_BASE,GPTIMER_B);
    for(ui32Loop=1;ui32Loop<FREQ_CNT/2;ui32Loop++) {
    }
    TimerEnable(GPTIMER1_BASE,GPTIMER_A);
    TimerEnable(GPTIMER0_BASE,GPTIMER_A);

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
      timeroffset=TimerValueGet(GPTIMER0_BASE,GPTIMER_A);
   //    for(ui32Loop=1;ui32Loop<5000000;ui32Loop++) {
   // }
       
    TimerDisable(GPTIMER1_BASE,GPTIMER_B);

    TimerDisable(GPTIMER1_BASE,GPTIMER_A);
    GPIOPinTypeGPIOOutput(GPIO_D_BASE,GPIO_PIN_1|GPIO_PIN_2);
    GPIOPinWrite(GPIO_D_BASE,GPIO_PIN_1|GPIO_PIN_2,255);
     for(ui32Loop=1;ui32Loop<500000;ui32Loop++) {
    }
    
    TimerLoadSet(GPTIMER1_BASE,GPTIMER_A,FREQ_CNT); //1a load
    TimerLoadSet(GPTIMER1_BASE,GPTIMER_B,FREQ_CNT); //1b load
    GPIOPinTypeTimer(GPIO_D_BASE,GPIO_PIN_1|GPIO_PIN_2); //enables hw muxing of pin outputs
    //gpio_state=IOCPadConfigGet(GPIO_D_BASE,GPIO_PIN_1); 
    IOCPadConfigSet(GPIO_D_BASE,GPIO_PIN_1|GPIO_PIN_2,IOC_OVERRIDE_OE|IOC_OVERRIDE_PUE); // enables pins as outputs, necessary for this code to work correctly
    
    
    TimerEnable(GPTIMER1_BASE,GPTIMER_B);
    for(ui32Loop=1;ui32Loop<FREQ_CNT/3;ui32Loop++) {
    }
    TimerEnable(GPTIMER1_BASE,GPTIMER_A);
   // TimerEnable(GPTIMER0_BASE,GPTIMER_A);
    
    for(ui32Loop=1;ui32Loop<500000;ui32Loop++) {
    }
    }

    
}

