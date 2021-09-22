#ifndef _PHOENIX_I2C_H
#define _PHOENIX_I2C_H

#include <stdint.h>


/* initialises peripheral, returns 0 on success <0 on error */
extern int i2c_init(unsigned int dev_no);


/* below read/write operations return:
 *  0          -> success
 *  -EBUSY     -> bus is busy (multimaster mode)
 *  -ETIMEDOUT -> timed out waiting for the transfer to succeed
 *  -EIO       -> transfer was not ACKed by the slave device (eg. invalid dev_addr)
 */


/* Performs i2c generic write operation to the given slave device. */
extern int i2c_busWrite(uint8_t dev_addr, const uint8_t *data, uint32_t len);


/* Performs i2c generic read operation from the given slave device. */
extern int i2c_busRead(uint8_t dev_addr, uint8_t *data_out, uint32_t len);


/* Performs i2c regiester read operation from the given slave device */
extern int i2c_regRead(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data_out, uint32_t len);

#endif /* _PHOENIX_I2C_H */
