/**
 * SA56004 thermometer driver
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
 * @author Pawel Kolodziej <pawel.kolodziej@phoesys.com>
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

static SensorDevice_t dev;

/** Read local temperature from thermometer SA56004X
 *
 * @param tempf	the temperature read
 * @return	0 if OK, or -1 if an error happened
 */
int dev_sa56004ReadTemp(float *tempf)
{
	s16 res;
	s16 temp;
	u8 buf[1];

	u8 thermometer_i2c_addr = I2C_SA56004_ADDR;

	proc_mutexLock(&dev.lock);

	/* thermometer's register for high byte of local temp */
	res = i2c_devRead(I2C_SA56004_BUS, thermometer_i2c_addr, 0, buf, 1);
	if(res < 0) {
		proc_mutexUnlock(&dev.lock);
		return -1;
	}
	temp = buf[0];
	/* avoiding cast */
	if(temp>127)
		temp = (0-temp) << 3;
	else
		temp = temp << 3;

	/* thermometer's register for low byte of local temp */
	res = i2c_devRead(I2C_SA56004_BUS, thermometer_i2c_addr, 0x22, buf, 1);
	if(res < 0) {
		proc_mutexUnlock(&dev.lock);
		return -1;
	}

	proc_mutexUnlock(&dev.lock);

	temp |= buf[0] >> 5;
	*tempf = temp / 8.0;

	return EOK;
}


int sa56004_get(struct SensorDevice_t *dev, float *res)
{
	assert(dev != NULL);
	proc_mutexLock(&dev->lock);
	int ret=0;
	s16 result;
	s16 temp=0;
	u8 buf[1];

	u8 thermometer_i2c_addr = I2C_SA56004_ADDR;

	/* thermometer's register for high byte of local temp */
	result = i2c_devRead(I2C_SA56004_BUS, thermometer_i2c_addr, 0, buf, 1);
	if(result >= 0) {
		temp = buf[0];
		/* avoiding cast */
		if(temp>127)
			temp = (0-temp) << 3;
		else
			temp = temp << 3;
		/* thermometer's register for low byte of local temp */
		result = i2c_devRead(I2C_SA56004_BUS, thermometer_i2c_addr, 0x22, buf, 1);
		if(result < 0)
			ret = -1;
	}
	else
		ret = -1;

	temp |= buf[0] >> 5;
	*res = temp / 8.0;

	proc_mutexUnlock(&dev->lock);

	return ret;
}


void _sa56004_init(void)
{
	int minor = dev_minorAlloc(MAKEDEV(MAJOR_SENSOR, 0));
	memset(&dev, 0x0, sizeof(dev));
	dev.get=sa56004_get;
	proc_mutexCreate(&dev.lock);

	if (subdev_register(MAKEDEV(MAJOR_SENSOR, minor), &dev) < 0) {
		main_printf(ATTR_ERROR, "dev/sensor/sa56004: Can't register device for /dev/sa56004\n" );
	}
	else
		assert(EOK==dev_mknod(MAKEDEV(MAJOR_SENSOR, minor), "sa56004"));
}
