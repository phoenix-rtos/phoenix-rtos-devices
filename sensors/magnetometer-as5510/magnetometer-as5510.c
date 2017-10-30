/**
 * AS5510 magnetometer driver
 *
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * @file
 *
 * @copyright 2015 Phoenix Systems
 *
 * @author Paweł Krężołek <pawel.krezolek@phoesys.com>
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

/* I2C slave addresses in OmniChip's ADC board: Thermometer = 0x4C, ADCs = 0x48, 0x4A*/
/* In Tytus1: Thermometer orig = 0x48, later changed to 0x4C; Magnetometr = 0x57 or 0x56 ;
 * 				RTC = 0x6f for RTC + 0x57 for EEPROM;
 **/

#include <hal/if.h>
#include <main/if.h>
#include <dev/vybrid-i2c/if.h>
#include <dev/dev.h>
#include <proc/if.h>
#include <dev/sensor/interface.h>
#include "if.h"

#define REG_MAGN_DATA_L 0x0
#define REG_MAGN_DATA_H 0x1


static SensorDevice_t dev;

static inline void printMagReg(u8 reg)
{
	u8 buf[1];
	s16 res;
	res = i2c_devRead(I2C_AS5510_BUS, I2C_AS5510_ADDR, reg, buf, 1);
	assert(res == EOK);
	main_printf(ATTR_INFO, "REGISTER %x: %x\n", reg, buf[0]);
}


/** Read local temperature from thermometer SA56004X
 *
 * @param tempf	the temperature read
 * @return	0 if OK, or -1 if an error happened
 */
int dev_as5510ReadMF(s16 *mf)
{
	s16 res;
	s16 temp;
	u8 buf[1];
	u8 magnetometer_i2c_addr = I2C_AS5510_ADDR;

	proc_mutexLock(&dev.lock);

	/* thermometer's register for high byte of local temp */
	res = i2c_devRead(I2C_AS5510_BUS, magnetometer_i2c_addr, REG_MAGN_DATA_L, buf, 1);
	if(res < 0) {
		proc_mutexUnlock(&dev.lock);
		return -1;
	}
	temp = buf[0];

	/* thermometer's register for low byte of local temp */
	res = i2c_devRead(I2C_AS5510_BUS, magnetometer_i2c_addr, REG_MAGN_DATA_H, buf, 1);
	if(res < 0) {
		proc_mutexUnlock(&dev.lock);
		return -1;
	}

	proc_mutexUnlock(&dev.lock);

	temp |= ((u16)(buf[0] & 0x3)) << 8;
	//1FF is a half of 10 bit range
	temp -= 0x1FF;
	*mf = temp;

	return EOK;
}

int as5510_get(struct SensorDevice_t *device, float *res)
{
	s16 result;
	s16 temp=0;
	u8 buf[1];
	int ret = 0;
	u8 magnetometer_i2c_addr = I2C_AS5510_ADDR;

	if(device == NULL)
		device = &dev;

	proc_mutexLock(&device->lock);

	/* thermometer's register for high byte of local temp */
	result = i2c_devRead(I2C_AS5510_BUS, magnetometer_i2c_addr, REG_MAGN_DATA_L, buf, 1);
	if(result >= 0) {
		temp = buf[0];

		/* thermometer's register for low byte of local temp */
		result = i2c_devRead(I2C_AS5510_BUS, magnetometer_i2c_addr, REG_MAGN_DATA_H, buf, 1);
		if(result < 0)
			ret = -1;
	}
	else
		ret = -1;

	proc_mutexUnlock(&device->lock);

	temp |= ((u16)(buf[0] & 0x3)) << 8;
	//1FF is a half of 10 bit range
	temp -= 0x1FF;
	*res = temp;
	return ret;
}




void _as5510_init(void)
{
	s16 res;
	u8 buf[1];
	int minor = dev_minorAlloc(MAKEDEV(MAJOR_SENSOR, 0));
	memset(&dev, 0x0, sizeof(dev));
	dev.get=as5510_get;

	res = i2c_devRead(I2C_AS5510_BUS, I2C_AS5510_ADDR, 0x0B, buf, 1);
	assert(res == EOK);
	buf[0] &= 0xFC;
	//setting up least sensitive mode - 97 uT/LSB, range +-50mT
	res = i2c_devWrite(I2C_AS5510_BUS, I2C_AS5510_ADDR, 0x0B, buf, 1);
	assert(res == EOK);
	res = i2c_devRead(I2C_AS5510_BUS, I2C_AS5510_ADDR, 0x0B, buf, 1);
	assert(res == EOK);
	assert((buf[0] & 0x03) == 0 );
	
	printMagReg(0x00);
	printMagReg(0x01);
	printMagReg(0x02);
	printMagReg(0x03);
	printMagReg(0x04);
	printMagReg(0x0B);

	proc_mutexCreate(&dev.lock);

	if (subdev_register(MAKEDEV(MAJOR_SENSOR, minor), &dev) < 0) {
		main_printf(ATTR_ERROR, "dev/sensor/as5510: Can't register device for /dev/as5510\n" );
	}
	else
		assert(EOK==dev_mknod(MAKEDEV(MAJOR_SENSOR, minor), "as5510"));
}
