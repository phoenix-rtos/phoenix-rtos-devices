/**
 * Vybrid-I2C kernel driver - header file
 *
 * Synchronous functions to access an I2C bus.
 * The comfort function i2c_devRead implements the defacto standard I2C operation "read from device".
 *
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * @file
 *
 * @copyright 2014 Phoenix Systems
 *
 * @author Horacio Mijail Anton Quiles <horacio.anton@phoesys.com>
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _VYBRID_I2C_IF_H_
#define _VYBRID_I2C_IF_H_

#include <hal/if.h>
#include <stdbool.h>

/** Get the base address for the given I2C bus instance.
 *
 * @param I2C_instance		must be < I2C_NUM
 * @return					the address, or NULL if a non-unitialized instance is given
 */
addr_t i2c_getBase(u8 I2C_instance);


/** Check if someone answers at the given address */
bool i2c_isDevPresent(addr_t i2c_base_addr, u8 i2c_addr);


/** Read N 8-bit values from I2C device (typical strategy)
 *
 *	This function implements the typical I2C strategy for a "read register" operation: busWrite to a device of a 1-byte command (or register address) + busRead.
 *	Additionally, many slaves implement a multiple-read or register-address-autoincrement mode which allows to get a series of registers on a single transaction.
 *
 * @param i2c_bus_num		number of I2C master bus
 * @param i2c_addr			I2C address of the slave we want to communicate with
 * @param reg				register or command to request to the slave
 * @param buf				buffer in which to store the answer
 * @param n					how many bytes to read into buf
 * @return					0 if OK, or -1 if anything went wrong
 */
int i2c_devRead(unsigned i2c_bus_num, u8 i2c_addr, u8 reg, u8 buf[], u8 n);

/** Write N 8-bit values to I2C device (typical strategy)
 *
 *	This function implements the typical I2C strategy for a "write register" operation: busWrite to a device of a 1-byte command (or register address) + busRead.
 *	Additionally, many slaves implement a multiple-write or register-address-autoincrement mode which allows to get a series of registers on a single transaction.
 *
 * @param i2c_bus_num		number of I2C master bus
 * @param i2c_addr			I2C address of the slave we want to communicate with
 * @param reg				register or command to request to the slave
 * @param buf				buffer in which to store the answer
 * @param len				how many bytes to read into buf
 * @return					0 if OK, or -1 if anything went wrong
 */
int i2c_devWrite(unsigned i2c_bus_num, u8 i2c_addr, u8 reg, u8 buf[], u8 len);

/** Flags to indicate whether the bus access functions should set an optional bus condition*/
enum i2c_setBusCondition {
	ISBC_SET = true,		/**< basic I2C behaviour*/
	ISBC_DONT_SET = false	/**< for command chaining  through REPEATED START */
};


/** Write to the bus as master
 *
 * buf can be NULL (and len can be 0) to just write the address to the bus (useful to prepare a read and for presence testing).
 * Generation of STOP is the normal case, but can be disabled so a REPEATED START could be used afterwards.
 *
 * @param base_addr			base memory address of the master doing the writing
 * @param slave_addr
 * @param buf				buffer to be sent
 * @param len				length of the buffer
 * @param generate_STOP		whether to generate a STOP condition
 * @return					num of bytes that were ACKed by the slave,
 * 							or -1 if no slave answered at that address
 * 							(0 means the slave just ACKed the address)
 */
s16 i2c_busWrite(addr_t base, u8 slave_addr, u8 * buf, u16 len, enum i2c_setBusCondition generate_STOP);


/** Read from the bus as master
 *
 * Generation of START is the normal case, but can be disabled when using a REPEATED START.
 *
 * @param base				base memory address of the master doing the reading
 * @param slave_addr
 * @param buf
 * @param len				requested bytes
 * @param generate_START	whether to generate a START condition
 * @return					num of bytes read; 0 if there was no slave at that address; -1 for errors
 *
 * @note					The I2C protocol forces the slave to send data until the master stops it, so num of bytes read can not be <len unless there is an error.
 */
s16 i2c_busRead(addr_t base, u8 slave_addr, u8 * buf, u16 len, enum i2c_setBusCondition generate_START);


/** Generate a REPEATED START condition*/
void i2c_generateRestart(addr_t base);


/** Generate a STOP condition*/
void i2c_generateStop(addr_t base);


/** Generate a START condition*/
void i2c_generateStart(addr_t base);


/** Initialize I2C devices
 *
 * Init all devices supported by the driver and register them with the OS
 * */
int _i2c_init(void);


#endif
