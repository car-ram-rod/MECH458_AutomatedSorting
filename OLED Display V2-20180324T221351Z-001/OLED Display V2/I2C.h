#ifndef _I2C_H_
#define _I2C_H_

#include <avr/io.h>



#define SCL_CLOCK  100000L

/** defines the data direction (reading from I2C device) in i2c_start(),i2c_rep_start() */
#define I2C_READ    1

/** defines the data direction (writing to I2C device) in i2c_start(),i2c_rep_start() */
#define I2C_WRITE   0


/**
 @brief initialize the I2C master interface. Need to be called only once 
 @return none
 */
extern void i2cInit(void);

/** 
 @brief Issues a start condition and sends address and transfer direction 
  
 @param    addr address and transfer direction of I2C device
 @retval   0   device accessible 
 @retval   1   failed to access device 
 */
extern unsigned char i2cStart(unsigned char addr);

/**
 @brief Send one byte to I2C device
 @param    data  byte to be transfered
 @retval   0 write successful
 @retval   1 write failed
 */
extern unsigned char i2cWrite(unsigned char data);

/** 
 @brief Terminates the data transfer and releases the I2C bus 
 @return none
 */
extern void i2cStop(void);

#endif