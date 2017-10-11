extern void PwmTimerAIntHandler(void);
extern void PwmTimerBIntHandler(void);
extern void inchwormInit(struct Inchworms motor);
extern void inchwormDisable(void);
extern void inchwormEnable(void);

struct Inchworms{
  uint32 GPIObase1=0;//feed gpio base for pin 1
  uint32 GPIObase2=0; //gpio base for pin 2
  uint8 GPIOpin1=0; //bit packed representation for gpio1
  uint8 GPIOpin2=0;
  uint32 motorFrequency=1000;
  uint32 dutyCycle=80;
  uint32 motorID=0; //id for motor
  uint32 timer=0;
  
};
