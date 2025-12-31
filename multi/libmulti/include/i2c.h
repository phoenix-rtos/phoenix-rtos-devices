#ifndef _LIB_I2C_H_
#define _LIB_I2C_H_

#include <stddef.h>
#include <stdint.h>


int i2c_busWrite(unsigned bus, uint8_t dev_addr, const uint8_t *data, uint32_t len);


int i2c_busRead(unsigned bus, uint8_t dev_addr, uint8_t *data_out, uint32_t len);


int i2c_regWrite(unsigned bus, uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, size_t len);


int i2c_regRead(unsigned bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data_out, size_t len);


#endif
