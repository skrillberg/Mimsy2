
typedef struct InchwormMotor{
  uint32_t GPIObase1;//feed gpio base for pin 1
  uint32_t GPIObase2; //gpio base for pin 2
  uint8_t GPIOpin1; //bit packed representation for gpio1
  uint8_t GPIOpin2;
  uint8_t motorID;
} InchwormMotor;

typedef struct InchwormSetup{
  InchwormMotor *iwMotors;
  uint32_t numOfMotors;
  uint32_t motorFrequency;
  uint32_t dutyCycle;
  uint32_t motorID; //id for motor
  uint32_t timer;
} InchwormSetup;



extern void PwmTimerAIntHandler(void);
extern void PwmTimerBIntHandler(void);
extern void inchwormInit(struct InchwormSetup motor);
extern void inchwormRelease(InchwormMotor motor);
extern void inchwormFreerun(InchwormMotor motor);