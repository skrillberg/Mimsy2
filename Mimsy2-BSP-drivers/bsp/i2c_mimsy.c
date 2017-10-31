/**
 * Author: Xavier Vilajosana (xvilajosana@eecs.berkeley.edu)
 *         Pere Tuset (peretuset@openmote.com)
 * Date:   July 2013
 * Description:CC2538-specific definition of the "i2c" bsp module.
 */

#include <hw_gpio.h>
#include <hw_i2cm.h>
#include <hw_ioc.h>
#include <hw_memmap.h>
#include <hw_sys_ctrl.h>
#include <hw_types.h>
#include <gptimer.h>
#include <gpio.h>
#include <i2c.h>
#include <ioc.h>
#include <sys_ctrl.h>

//=========================== define ==========================================

#define I2C_PERIPHERAL          ( SYS_CTRL_PERIPH_I2C )
#define I2C_BASE                ( GPIO_D_BASE )
#define I2C_SCL                 ( GPIO_PIN_4 )
#define I2C_SDA                 ( GPIO_PIN_5 )
#define I2C_BAUDRATE            ( 400000 )
#define I2C_MAX_DELAY_US        ( 1000000 )

//=========================== variables =======================================


//=========================== prototypes ======================================

 uint32_t board_timer_get(void);
 bool board_timer_expired(uint32_t future);
bool board_timer_expired(uint32_t future);
//=========================== public ==========================================
/**
 * Timer runs at 32 MHz and is 32-bit wide
 * The timer is divided by 32, whichs gives a 1 microsecond ticks
 */
void board_timer_init(void) {
     SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_GPT2);
  
  // Configure the timer
    TimerConfigure(GPTIMER2_BASE, GPTIMER_CFG_PERIODIC_UP);
    
    // Enable the timer
    TimerEnable(GPTIMER2_BASE, GPTIMER_BOTH);
}

/**
 * Returns the current value of the timer
 * The timer is divided by 32, whichs gives a 1 microsecond ticks
 */
uint32_t board_timer_get(void) {
    uint32_t current;
    
    current = TimerValueGet(GPTIMER2_BASE, GPTIMER_A) >> 5;
    
    return current;
}

/**
 * Returns true if the timer has expired
 * The timer is divided by 32, whichs gives a 1 microsecond ticks
 */
bool board_timer_expired(uint32_t future) {
    uint32_t current;
    int32_t remaining;

    current = TimerValueGet(GPTIMER2_BASE, GPTIMER_A) >> 5;

    remaining = (int32_t) (future - current);
    
    if (remaining > 0) {
        return false;
    } else {
        return true;
    }
}


void delay_ms(uint32_t delay){
  uint32_t current=board_timer_get();
  while(board_timer_get()<current+delay*1000){
    
  }
}
  void get_ms(uint32_t *timestamp){
    *timestamp = board_timer_get()*32/(SysCtrlClockGet()/1000);
  }

void i2c_init(void) {
    bool status;
    
    // Enable peripheral except in deep sleep modes (e.g. LPM1, LPM2, LPM3)
    SysCtrlPeripheralEnable(I2C_PERIPHERAL);
    SysCtrlPeripheralSleepEnable(I2C_PERIPHERAL);
    SysCtrlPeripheralDeepSleepDisable(I2C_PERIPHERAL);

    // Reset peripheral previous to configuring it
    SysCtrlPeripheralReset(I2C_PERIPHERAL);

    // Configure the SCL pin
    GPIOPinTypeI2C(I2C_BASE, I2C_SCL);
    IOCPadConfigSet(I2C_BASE, I2C_SCL,IOC_OVERRIDE_PUE); // enables pins as outputs, necessary for this code to work correctly
 
    IOCPinConfigPeriphInput(I2C_BASE, I2C_SCL, IOC_I2CMSSCL);
    IOCPinConfigPeriphOutput(I2C_BASE, I2C_SCL, IOC_MUX_OUT_SEL_I2C_CMSSCL);

    // Configure the SDA pin
    GPIOPinTypeI2C(I2C_BASE, I2C_SDA);
    IOCPadConfigSet(I2C_BASE, I2C_SDA,IOC_OVERRIDE_PUE); // enables pins as outputs, necessary for this code to work correctly
    IOCPinConfigPeriphInput(I2C_BASE, I2C_SDA, IOC_I2CMSSDA);
    IOCPinConfigPeriphOutput(I2C_BASE, I2C_SDA, IOC_MUX_OUT_SEL_I2C_CMSSDA);

    // Configure the I2C clock
    status = (I2C_BAUDRATE == 400000 ? true : false);
    I2CMasterInitExpClk(SysCtrlClockGet(), status);

    // Enable the I2C module as master
    I2CMasterEnable();
}

bool i2c_read_byte(uint8_t address, uint8_t* byte) {
    uint32_t future = I2C_MAX_DELAY_US;
    
    // Receive operation
    I2CMasterSlaveAddrSet(address, true);

    // Single receive operation
    I2CMasterControl(I2C_MASTER_CMD_SINGLE_RECEIVE);

    // Calculate timeout
    future += board_timer_get();

    // Wait until complete or timeout
    while (I2CMasterBusy()) {
        // Update timeout status and return if expired
        if (board_timer_expired(future)) return false;
    }

    // Read data from I2C
    *byte = I2CMasterDataGet();
    
    // Return status
    return true;
}

uint32_t i2c_read_bytes(uint8_t address, uint8_t* buffer, uint32_t length) {
    uint32_t future = I2C_MAX_DELAY_US;
    
    // Receive operation
    I2CMasterSlaveAddrSet(address, true);

    // Multiple receive operation
    I2CMasterControl(I2C_MASTER_CMD_BURST_RECEIVE_START);

    // Calculate timeout
    future += board_timer_get();

    // Iterate overall all bytes
    while (length) {
        // Wait until complete or timeout
        while (I2CMasterBusy()) {
            // Update timeout status and return if expired
            if (board_timer_expired(future)) return length;
        }
        
        // Read data from I2C
        *buffer++ = I2CMasterDataGet();
        

        // Check if it's the last byte
        if (length == 1) I2CMasterControl(I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
        else             I2CMasterControl(I2C_MASTER_CMD_BURST_RECEIVE_CONT);
        length--;
    }
    
    // Return bytes read
    
    return length;
}

bool i2c_write_byte(uint8_t address, uint8_t byte) {   
    uint32_t future = I2C_MAX_DELAY_US;
    
    // Transmit operation
    I2CMasterSlaveAddrSet(address, false);

    // Write byte to I2C buffer
    I2CMasterDataPut(byte);

    // Single transmit operation
    I2CMasterControl(I2C_MASTER_CMD_SINGLE_SEND);
    
    // Calculate timeout
    future += board_timer_get();

    // Wait until complete or timeout
    while (I2CMasterBusy()) {
        // Check timeout status and return if expired
        if (board_timer_expired(future)) return false;
    }
    
    return true;
}

uint32_t i2c_write_bytes(uint8_t address, uint8_t* buffer, uint32_t length) {
    uint32_t future = I2C_MAX_DELAY_US;
    
    // Transmit operation
    I2CMasterSlaveAddrSet(address, false);

    // Write byte to I2C buffer
    I2CMasterDataPut(*buffer++);
    length--;

    // Multiple transmit operation
    I2CMasterControl(I2C_MASTER_CMD_BURST_SEND_START);

    // Calculate timeout
    future += board_timer_get();

    // Wait until complete or timeout
    while (I2CMasterBusy()) {
        // Check timeout status and return if expired
        if (board_timer_expired(future)) return length;
    }

    // Iterate overall all bytes
    while (length) {
        // Write byte to I2C buffer
        I2CMasterDataPut(*buffer++);

        // Check if it's the last byte
        if (length == 1) I2CMasterControl(I2C_MASTER_CMD_BURST_SEND_FINISH);
        else             I2CMasterControl(I2C_MASTER_CMD_BURST_SEND_CONT);

        // Wait until complete or timeout
        while (I2CMasterBusy()) {
            // Check timeout status and return if expired
            if (board_timer_expired(future)) return length;
        }

        // Update the length
        length--;
    }
    
    // Return bytes written
    return length;
}
int i2c_read_registers(uint8_t slave_addr,
                             uint8_t reg_addr,
                             uint8_t numBytes,
                             uint8_t* spaceToWrite){
                               
           i2c_write_byte(slave_addr,reg_addr);
           //i2c_read_byte(slave_addr,spaceToWrite);
           i2c_read_bytes(slave_addr,spaceToWrite,numBytes);
           delay_ms(1);
           return 0;
}

void i2c_read_register(uint8_t slave_addr,
                             uint8_t reg_addr,
                             
                             uint8_t* spaceToWrite){
                               
           i2c_write_byte(slave_addr,reg_addr);
           i2c_read_byte(slave_addr,spaceToWrite);
          
}

void i2c_write_register_8bit( uint8_t slave_addr,uint8_t reg_addr, uint8_t data){
  uint8_t buffer[2]  ={reg_addr,data};          
        
              i2c_write_bytes(slave_addr,buffer,2);

}

int i2c_write_registers( uint8_t slave_addr,uint8_t reg_addr, uint8_t length,uint8_t *data){
  uint8_t buffer[100] ;
  buffer[0]=reg_addr;
  for(int i = 0; i < length+1; i++){
    buffer[i+1]=*data;
    data++; 
  }
        
              i2c_write_bytes(slave_addr,buffer,length+1);
  delay_ms(1);
  return 0;
}


//=========================== private =========================================

