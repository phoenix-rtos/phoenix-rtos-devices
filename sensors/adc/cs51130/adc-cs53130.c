/**
 * Driver for external ADC: CS53L30
 *
 * Phoenix-RTOS
 * 
 * Operating system kernel
 *
 * @file esai.c
 *
 * @copyright 2014 Phoenix Systems
 *
 * @author Pawel Tryfon<pawel.tryfon@phoesys.com>
 * @author Pawel Kolodziej<pawel.kolodziej@phoesys.com>
 * 
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <hal/if.h>
#include <dev/vybrid-i2c/if.h>
#include <main/if.h>
#include <dev/dev.h>
#include <dev/sensor/interface.h>
 #include "if.h"

#define CS53L30_ADDR1 0x4A
#define CS53L30_ADDR2 0x48

/* Register addresses */
#define POW_RA 0x06
#define MCLK_RA 0x07
#define INT_SR_RA 0x08
#define ASP_CFG_RA 0x0C
#define ASP_CTRL1_RA 0x0D
#define ASP_TDM_TX_CTRL(n) ((0x0E)-1+(n))
#define ASP_TDM_TX_EN(n) ((0x17)+1-(n))
#define ASP_CTRL2_RA 0x18
#define LRCK_CTRL1_RA 0x1B
#define LRCK_CTRL2_RA 0x1C
#define ADC1_CTRL1_RA 0x25
#define ADC1_CTRL2_RA 0x26
#define ADC1_CTRL3_RA 0x27
#define ADC1_NOISE_RA 0x28
#define ADC1A_AFE_RA 0x29
#define ADC1B_AFE_RA 0x2A
#define ADC1A_VOL_RA 0x2B
#define ADC1B_VOL_RA 0x2C
#define ADC2_CTRL1_RA 0x2D
#define ADC2_CTRL2_RA 0x2E
#define ADC2_CTRL3_RA 0x2F
#define ADC2_NOISE_RA 0x30
#define ADC2A_AFE_RA 0x31
#define ADC2B_AFE_RA 0x32
#define ADC2A_VOL_RA 0x33
#define ADC2B_VOL_RA 0x34

#define DEBUG_CS53L30 0

static int myi2c_write(u8 dev,u8 reg,u8 v)
{
	//u8 b[2];
	//b[0] = reg; b[1] = v;
	//return i2c_busWrite(i2c_getBase(0),dev,b,2,ISBC_SET);
	return i2c_devWrite(0, dev, reg, &v, 1);
}


static int myi2c_read(u8 dev,u8 reg,u8* v)
{
	
	 return i2c_devRead(0, dev, reg, v, 1);
	

//	int rc;
	//u8 b;
	//b = reg ; //  | 0x80; /* INCR = 1 */
	//if((rc = i2c_busWrite(i2c_getBase(0),dev,&b,1,ISBC_SET)) < 0)
	//	return rc;
	//return i2c_busRead(i2c_getBase(0),dev,v,1,ISBC_SET);
}

static void _cs53l30_powerDown(u8 CS53L30)
{
	if(myi2c_write(CS53L30,POW_RA,0x50) < 0) {
		main_printf(ATTR_ERROR,"CS53L30 power down failed\n");
		return;
	}
}

static void _cs53l30_powerUp(u8 CS53L30)
{
	u8 v;
	int i;
	if(myi2c_write(CS53L30,POW_RA,0x0) < 0) {
		main_printf(ATTR_ERROR,"CS53L30 power up failed\n");
		return;
	}
	for(i=1; i < 0x36; i++){
		if(myi2c_read(CS53L30, i ,&v) < 0){
			main_printf(ATTR_ERROR,"\nError reading register 0x%x\n", i);
			return;
		} else{
			main_printf(ATTR_INFO,"%2x = %2x  ",i,v);	
		}
	}
	main_printf(ATTR_INFO,"\n");

	//cs53l30_checkConfig();
}

static int checkSleep = 0;
static int cs53l30_checkConfigOne(u8 i2c_addr, int master)
{
	int i;
	int ret=0;
	u8 v;
	u8 expected[]={0x53,  0xa3,  0x00,  0x00,  0xa0,  0x00,  0x00,  0x1c,  0x00,  0xf4,  0x00,  0x1c,  0x10,  0x00,  0x04,  0x08,  0x0c,  0x00,  0x00,  0x00,  0x00,  0xff,  0xff,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x80,  0xaa,  0xaa,  0xa8,  0xec,  0x04,  0x00,  0x00,  0x00,  0x40,  0x40,  0x00,  0x00,  0x04,  0x00,  0x00,  0x00,  0x40,  0x40,  0x00,  0x00};
	if(master)
		expected[MCLK_RA-1] |= 2;
#if DEBUG_CS53L30
	main_printf(ATTR_INFO,"cs53l30 (%x) config: ", i2c_addr);
#endif
	for(i=1; i< 0x35; i++){
		if(myi2c_read(i2c_addr, i ,&v) < 0){
			main_printf(ATTR_ERROR,"\nError reading register 0x%x\n", i);
		} else{
#if DEBUG_CS53L30
			main_printf(ATTR_INFO,"0x%02x,  ",v);
#endif
		}
		if(v!=expected[i-1]) {
			main_printf(ATTR_ERROR,"\nWRONG ADC CONFIGURATION [%x] is 0x%x expected  0x%x at reg 0x%x\n", i2c_addr, v, expected[i-1], i);
			ret = -1;
		}
		proc_threadSleep(checkSleep);
	}
#if DEBUG_CS53L30
	main_printf(ATTR_INFO,";\n");
#endif
	return ret;

}

int cs53l30_checkConfig(void)
{
	int res = 0;
	res = cs53l30_checkConfigOne(CS53L30_ADDR1, 1) || cs53l30_checkConfigOne(CS53L30_ADDR2, 0);
	return res;
}

static void readId(u8 CS53L30)
{
	u8 id[3];

	if( (myi2c_read(CS53L30,0x01,id+0)<0) || (myi2c_read(CS53L30,0x02,id+1) <0) || (myi2c_read(CS53L30,0x03,id+2) <0))
		main_printf(ATTR_INFO, "read error ");
	main_printf(ATTR_INFO, "CS53L30[%x] id: %x %x %x (should be 53 a3 0)\n",CS53L30, id[0], id[1], id[2]);
}

static void _cs53l30_configureOne(u8 CS53L30, int master)
{
	u8 id[3];

	myi2c_read(CS53L30,0x01,id+0);
	myi2c_read(CS53L30,0x02,id+1);
	myi2c_read(CS53L30,0x03,id+2);
	main_printf(ATTR_INFO, "CS53L30[%x] id: %x %x %x (should be 53 a3 0)\n",CS53L30, id[0], id[1], id[2]);
	u8 afe[4];
#if 1
	afe[0] = afe[1] = afe[2] = afe[3] = 0x40;
#else
	if(!master){
		afe[0] = 0x80 | 0x1f;
		afe[1] = 0x80 | 0x1f;
		afe[2] = 0x40;
		afe[3] = 0x40;
	}else{
		afe[0] = 0x40;
		afe[1] = 0x80 | 0x1f;
		afe[2] = 0x40;
		afe[3] = 0x80 | 0x1f;
	}
#endif


	if(myi2c_write(CS53L30,MCLK_RA, (master ? 2 : 0) ) < 0) {
		main_printf(ATTR_ERROR,"CS53L30 setting MCLK failed\n");
		return;
	}

	if(myi2c_write(CS53L30,INT_SR_RA,0x1c) < 0) {
		main_printf(ATTR_ERROR,"CS53L30 setting sample rate failed\n");
		return;
	}
#define ASP_CFG_SCLK_INV (1<<4)
	if(myi2c_write(CS53L30,ASP_CFG_RA,0x1c /* | ASP_CFG_SCLK_INV */ )< 0) { //XXX 48k -> 0x1c  0x18 - 24khz  ; 0x11  = 12kHz
		main_printf(ATTR_ERROR,"CS53L30 setting sample rate failed\n");
		return;
	}
	if(myi2c_write(CS53L30,ASP_TDM_TX_CTRL(1),0x00) < 0) {
		main_printf(ATTR_ERROR,"CS53L30 configuring TDM slots failed\n");
		return;
	}
	if(myi2c_write(CS53L30,ASP_TDM_TX_CTRL(2),0x04) < 0) {
		main_printf(ATTR_ERROR,"CS53L30 configuring TDM slots failed\n");
		return;
	}
	if(myi2c_write(CS53L30,ASP_TDM_TX_CTRL(3),0x08) < 0) {
		main_printf(ATTR_ERROR,"CS53L30 configuring TDM slots failed\n");
		return;
	}
	if(myi2c_write(CS53L30,ASP_TDM_TX_CTRL(4),0x0C) < 0) {
		main_printf(ATTR_ERROR,"CS53L30 configuring TDM slots failed\n");
		return;
	}
	if(myi2c_write(CS53L30,ASP_TDM_TX_EN(1),0xFF) < 0) {
		main_printf(ATTR_ERROR,"CS53L30 enabling TDM slots failed\n");
		return;
	}
	if(myi2c_write(CS53L30,ASP_TDM_TX_EN(2),0xFF) < 0) {
		main_printf(ATTR_ERROR,"CS53L30 enabling TDM slots failed\n");
		return;
	}
	myi2c_write(CS53L30, 0x1f,0x00); // mute - default
	myi2c_write(CS53L30, 0x20,0x80); // mute
	if(myi2c_write(CS53L30,ASP_CTRL2_RA,0x00) < 0) {
		main_printf(ATTR_ERROR,"CS53L30 ASP control failed\n");
		return;
	}
	if(myi2c_write(CS53L30,LRCK_CTRL1_RA,0x0) < 0) {
		main_printf(ATTR_ERROR,"CS53L30 LRCK control failed\n");
		return;
	}
	if(myi2c_write(CS53L30,LRCK_CTRL2_RA,0x00) < 0) {
		main_printf(ATTR_ERROR,"CS53L30 LRCK control failed\n");
		return;
	}
	/* MUTE pins */
	if(myi2c_write(CS53L30,ADC1_CTRL1_RA,0x04) < 0) {
		main_printf(ATTR_ERROR,"CS53L30 setting ADC2 ctrl failed\n");
		return;
	}
	if(myi2c_write(CS53L30,ADC1_CTRL2_RA,0x00) < 0) {
		main_printf(ATTR_ERROR,"CS53L30 setting ADC1 ctrl failed\n");
		return;
	}
	if(myi2c_write(CS53L30,ADC1_CTRL3_RA,0x00) < 0) {
		main_printf(ATTR_ERROR,"CS53L30 setting ADC1 ctrl failed\n");
		return;
	}
	if(myi2c_write(CS53L30,ADC1_NOISE_RA,0x00) < 0) {
		main_printf(ATTR_ERROR,"CS53L30 setting ADC1 noise failed\n");
		return;
	}

	if(myi2c_write(CS53L30,ADC1A_AFE_RA,afe[0]) < 0) {
		main_printf(ATTR_ERROR,"CS53L30 preamp setting failed\n");
		return;
	}
	if(myi2c_write(CS53L30,ADC1B_AFE_RA,afe[1]) < 0) {
		main_printf(ATTR_ERROR,"CS53L30 preamp setting failed\n");
		return;
	}
	if(myi2c_write(CS53L30,ADC1A_VOL_RA,0x00) < 0) {
		main_printf(ATTR_ERROR,"CS53L30 adca vol setting failed\n");
		return;
	}
	if(myi2c_write(CS53L30,ADC1B_VOL_RA,0x00) < 0) {
		main_printf(ATTR_ERROR,"CS53L30 adca vol setting failed\n");
		return;
	}

	if(myi2c_write(CS53L30,ADC2_CTRL1_RA,0x04) < 0) {
		main_printf(ATTR_ERROR,"CS53L30 setting ADC2 ctrl failed\n");
		return;
	}
	if(myi2c_write(CS53L30,ADC2_CTRL2_RA,0x00) < 0) {
		main_printf(ATTR_ERROR,"CS53L30 setting ADC2 ctrl failed\n");
		return;
	}
	if(myi2c_write(CS53L30,ADC2_CTRL3_RA,0x00) < 0) {
		main_printf(ATTR_ERROR,"CS53L30 setting ADC2 ctrl failed\n");
		return;
	}
	if(myi2c_write(CS53L30,ADC1_NOISE_RA,0x00) < 0) {
		main_printf(ATTR_ERROR,"CS53L30 setting ADC1 noise failed\n");
		return;
	}
	if(myi2c_write(CS53L30,ADC2A_AFE_RA,afe[2]) < 0) {
		main_printf(ATTR_ERROR,"CS53L30 preamp setting failed\n");
		return;
	}
	if(myi2c_write(CS53L30,ADC2B_AFE_RA,afe[3]) < 0) {
		main_printf(ATTR_ERROR,"CS53L30 preamp setting failed\n");
		return;
	}
	if(myi2c_write(CS53L30,ADC2A_VOL_RA,0x00) < 0) {
		main_printf(ATTR_ERROR,"CS53L30 adca vol setting failed\n");
		return;
	}
	if(myi2c_write(CS53L30,ADC2B_VOL_RA,0x00) < 0) {
		main_printf(ATTR_ERROR,"CS53L30 adca vol setting failed\n");
		return;
	}

#define ASP_CTRL1_SHIFT_LEFT (1<<4)
	if(myi2c_write(CS53L30,ASP_CTRL1_RA,0x00  | ASP_CTRL1_SHIFT_LEFT ) < 0) {
		main_printf(ATTR_ERROR,"CS53L30 ASP control failed\n");
		return;
	}

	main_printf(ATTR_INFO,"\nCS53L30 at addr %x initialized\n",CS53L30);
}

static SensorDevice_t dev;

int cs53l30_checkConf(struct SensorDevice_t *dev)
{
	int ret = 0;
	assert(dev != NULL);
	proc_mutexLock(&dev->lock);
	ret = cs53l30_checkConfig();
	proc_mutexUnlock(&dev->lock);
	return ret;
}

void _cs53l30_configure(void)
{
	int minor = dev_minorAlloc(MAKEDEV(MAJOR_SENSOR, 0));
	checkSleep = 1;
	readId(CS53L30_ADDR1);
	readId(CS53L30_ADDR2);
	_cs53l30_powerDown(CS53L30_ADDR1);
	_cs53l30_powerDown(CS53L30_ADDR2);
	_cs53l30_configureOne(CS53L30_ADDR1, 1);
	_cs53l30_configureOne(CS53L30_ADDR2, 0);
	_cs53l30_powerUp(CS53L30_ADDR2);
	_cs53l30_powerUp(CS53L30_ADDR1);
	main_printf(ATTR_INFO, "\n\n");
#if 1
	int i=0;
	int sync1=0, sync2=0;
	do{
		u8 status;
		if(i++>1)
			proc_threadSleep(250 * 1000);
		if(!sync1){
			myi2c_read(CS53L30_ADDR1, 0x36 ,&status);
			if(status & 0x20)
				sync1=1;
		}
		if(!sync2){
			myi2c_read(CS53L30_ADDR2, 0x36 ,&status);
			if(status & 0x20)
				sync2=1;
		}

		main_printf(ATTR_INFO, "CL53L30: sync1: %d sync2: %d\n", sync1, sync2);
	}while(! (sync1 && sync2) && i < 3 );
	if( !(sync1 && sync2) )
		main_printf(ATTR_ERROR, "CL53L30: SYNC ERROR! sync1: %d sync2: %d\n", sync1, sync2);
	else
		main_printf(ATTR_INFO, "CL53L30: SYNC ok\n");
#endif
	main_printf(ATTR_INFO, "CL53L30: end of init\n");
	memset(&dev, 0x0, sizeof(dev));
	dev.checkConfig = cs53l30_checkConf;
	proc_mutexCreate(&dev.lock);
	cs53l30_checkConfig();
	checkSleep=1000 * 10;
	if(subdev_register(MAKEDEV(MAJOR_SENSOR, minor), &dev) < 0)
		main_printf(ATTR_ERROR, "dev/sensor/cs53l30: failed to register subdevice");
	else
		assert(EOK==dev_mknod(MAKEDEV(MAJOR_SENSOR, minor), "cs53l30"));
}
