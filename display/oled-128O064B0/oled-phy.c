/*
 * Phoenix-RTOS
 *
 * OLED phy interface
 *
 * Copyright 2020 Phoenix Systems
 * Author: Hubert Buczyński, Marcin Baran
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <errno.h>
#include <unistd.h>
#include <sys/msg.h>
#include <sys/platform.h>
#include <stdio.h>


#include <imxrt-multi.h>
#include <phoenix/arch/imxrt.h>


#define SPI1_PATH         "/dev/spi1"
#define PINS_PATH         "/dev/gpio3"

#define RESET_PIN         0
#define DC_PIN            1
#define PVCC_EN_PIN       16
#define PVDD_EN_PIN	      17

#define OUT_DIR           1

#define STATE_HIGH        1
#define STATE_LOW         0

/* Size of one row of pixels */
#define X_RES 132
/* Size of one column of pixels */
#define Y_RES 64


struct {
	oid_t spiOid;
	oid_t gpioOid;

	uint8_t *txBuff;
	uint8_t rxBuff[X_RES];
} oledphy_common;


static int oledphy_transmitSPI(uint8_t *data, uint8_t size)
{
	if (size > X_RES)
		return -1;

	msg_t msg;
	multi_i_t *idevctl = NULL;
	multi_o_t *odevctl = NULL;

	oledphy_common.txBuff = data;

	msg.type = mtDevCtl;
	msg.i.data = oledphy_common.txBuff;
	msg.i.size = 1;
	msg.o.data = oledphy_common.rxBuff;
	msg.o.size = 0;

	idevctl = (multi_i_t *)msg.i.raw;
	idevctl->id = oledphy_common.spiOid.id;
	idevctl->spi.type = spi_transaction;
	idevctl->spi.transaction.frameSize = size;
	idevctl->spi.transaction.cs = 0;

	odevctl = (multi_o_t *)msg.o.raw;

	if (msgSend(oledphy_common.spiOid.port, &msg) < 0)
		return -1;

	if (odevctl->err < 0)
		return -1;

	return odevctl->err;
}


static int oledphy_gpioSetPin(int gpio, int pin, int state)
{
	msg_t msg;
	multi_i_t *imsg = NULL;
	int res = EOK;

	msg.type = mtDevCtl;
	msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.data = NULL;
	msg.o.size = 0;

	imsg = (multi_i_t *)msg.i.raw;

	imsg->id = gpio;
	imsg->gpio.type = gpio_set_port;
	imsg->gpio.port.val = !!state << pin;
	imsg->gpio.port.mask = 1 << pin;

	if ((res = msgSend(oledphy_common.gpioOid.port, &msg)) < 0)
		return res;

	return res;
}


static int oledphy_gpioSetDir(int gpio, int pin, int dir)
{
	msg_t msg;
	multi_i_t *imsg = NULL;
	int res = EOK;

	msg.type = mtDevCtl;
	msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.data = NULL;
	msg.o.size = 0;

	imsg = (multi_i_t *)msg.i.raw;

	imsg->id = gpio;
	imsg->gpio.type = gpio_set_dir;
	imsg->gpio.dir.val = !!dir << pin;
	imsg->gpio.dir.mask = 1 << pin;

	if ((res = msgSend(oledphy_common.gpioOid.port, &msg)) < 0)
		return res;

	return res;
}


int oledphy_sendCmd(uint8_t cmd)
{
	int res = EOK;

	if ((res = oledphy_gpioSetPin(oledphy_common.gpioOid.id, DC_PIN, STATE_LOW)) < 0)
		return res;

	if ((res = oledphy_transmitSPI(&cmd, 1)) < 0)
		return res;

	return res;
}

int oledphy_sendData(uint8_t data)
{
	int res = EOK;

	if ((res = oledphy_gpioSetPin(oledphy_common.gpioOid.id, DC_PIN, STATE_HIGH)) < 0)
		return res;

	if ((res = oledphy_transmitSPI(&data, 1)) < 0)
		return res;

	return res;
}

int oledphy_sendDataBuf(uint8_t *data, uint8_t size)
{
	int res = EOK;

	if ((res = oledphy_gpioSetPin(oledphy_common.gpioOid.id, DC_PIN, STATE_HIGH)) < 0)
		return res;

	if ((res = oledphy_transmitSPI(data, size)) < 0)
		return res;

	return res;
}


static int oledphy_initSPI(void)
{
	while (lookup(SPI1_PATH, NULL, &oledphy_common.spiOid) < 0)
		usleep(10);

	msg_t msg;
	multi_i_t *idevctl = NULL;
	multi_o_t *odevctl = NULL;

	msg.type = mtDevCtl;
	msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.data = NULL;
	msg.o.size = 0;

	idevctl = (multi_i_t *)msg.i.raw;
	idevctl->id = oledphy_common.spiOid.id;
	idevctl->spi.type = spi_config;
	idevctl->spi.config.cs = 0;
	idevctl->spi.config.endian = spi_msb;
	idevctl->spi.config.mode = spi_mode_0;
	idevctl->spi.config.prescaler = 1;
	idevctl->spi.config.sckDiv = 1;

	odevctl = (multi_o_t *)msg.o.raw;

	if (msgSend(oledphy_common.spiOid.port, &msg) < 0)
		return -1;

	if (odevctl->err < 0)
		return -1;

	return EOK;
}


static int configMux(int mux, int sion, int mode)
{
	platformctl_t pctl;

	pctl.action = pctl_set;
	pctl.type = pctl_iomux;

	pctl.iomux.mux = mux;
	pctl.iomux.sion = sion;
	pctl.iomux.mode = mode;

	return platformctl(&pctl);
}


static int oledphy_initGPIO(void)
{
	int res = EOK;

	while (lookup(PINS_PATH, NULL, &oledphy_common.gpioOid) < 0)
		usleep(10);


	/* Init RESET */
	res = configMux(pctl_mux_gpio_sd_b1_00, 5, 5);
	if ((res = oledphy_gpioSetDir(oledphy_common.gpioOid.id, RESET_PIN, OUT_DIR)) < 0)
		return res;

	if ((res = oledphy_gpioSetPin(oledphy_common.gpioOid.id, RESET_PIN, STATE_HIGH)) < 0)
		return res;


	/* Init PVCC EN */
	res = configMux(pctl_mux_gpio_sd_b0_04, 5, 5);
	if ((res = oledphy_gpioSetDir(oledphy_common.gpioOid.id, PVCC_EN_PIN, OUT_DIR)) < 0)
		return res;

	if ((res = oledphy_gpioSetPin(oledphy_common.gpioOid.id, PVCC_EN_PIN, STATE_HIGH)) < 0)
		return res;


	/* Init PVDD EN */
	res = configMux(pctl_mux_gpio_sd_b0_05, 5, 5);
	if ((res = oledphy_gpioSetDir(oledphy_common.gpioOid.id, PVDD_EN_PIN, OUT_DIR)) < 0)
		return res;

	if ((res = oledphy_gpioSetPin(oledphy_common.gpioOid.id, PVDD_EN_PIN, STATE_HIGH)) < 0)
		return res;


	/* Init D/C Pin */
	res = configMux(pctl_mux_gpio_sd_b1_01, 5, 5);
	if ((res = oledphy_gpioSetDir(oledphy_common.gpioOid.id, DC_PIN, OUT_DIR)) < 0)
		return res;

	if ((res = oledphy_gpioSetPin(oledphy_common.gpioOid.id, DC_PIN, STATE_LOW)) < 0)
		return res;


	return res;
}


int oledphy_setPos(uint8_t x, uint8_t y)
{
	int res = EOK;
	x += 2;

	res |= oledphy_sendCmd(0xb0 + y);
	res |= oledphy_sendCmd(((x & 0xf0) >> 4) | 0x10);
	res |= oledphy_sendCmd(x & 0x0f);

	return res;
}


static int oledphy_initLCD(void)
{
	int res = 0;
	res |= oledphy_sendCmd(0xae);
	res |= oledphy_sendCmd(0xa0); //Segment normal
	res |= oledphy_sendCmd(0xda); //Common pads hardware: alternative
	res |= oledphy_sendCmd(0x12);
	res |= oledphy_sendCmd(0xc0); //Common output scan direction:com0~com63
	res |= oledphy_sendCmd(0xa8); //Multiplex ration mode:63
	res |= oledphy_sendCmd(0x3f);
	res |= oledphy_sendCmd(0xd5);
	//Display divide ratio/osc. freq. mode
	res |= oledphy_sendCmd(0x50);
	res |= oledphy_sendCmd(0x81);
	//Contrast control
	res |= oledphy_sendCmd(0xFF);
	res |= oledphy_sendCmd(0xd9);

	res |= oledphy_sendCmd(0x82);
	res |= oledphy_sendCmd(0xff);
	//Set pre-charge period
	res |= oledphy_sendCmd(0xf1);
	res |= oledphy_sendCmd(0x20); //Set Memory Addressing Mode
	res |= oledphy_sendCmd(0x02); //Page addressing mode
	res |= oledphy_sendCmd(0xdb); //VCOM deselect level mode
	res |= oledphy_sendCmd(0x20);
	//Set Vcomh
	res |= oledphy_sendCmd(0xad); //Master configuration
	res |= oledphy_sendCmd(0x8e); //External VCC supply
	res |= oledphy_sendCmd(0xa4); //Out follows RAM content
	res |= oledphy_sendCmd(0xa6); //Set normal display

	res |= oledphy_sendCmd(0xaf);
	res |= oledphy_sendCmd(0xa4);


	/* clear oled */
	res |= oledphy_sendCmd(0xaf);
	res |= oledphy_sendCmd(0x40);

	int i, j;

	for (i = 0; i < Y_RES / 8; ++i) {
		oledphy_sendCmd(i | 0xb0);
		for (j = 0; j < X_RES; ++j)
			oledphy_sendData(0x0);
	}

	return res;
}

int oledphy_reset(void)
{
	int res = EOK;

	if ((res = oledphy_gpioSetPin(oledphy_common.gpioOid.id, RESET_PIN, STATE_LOW)) < 0)
		return res;

	usleep(10000);

	if ((res = oledphy_gpioSetPin(oledphy_common.gpioOid.id, RESET_PIN, STATE_HIGH)) < 0)
		return res;

	return res;
}

int oledphy_init(void)
{
	int res = EOK;

	/* Initializing communication with multi driver. */
	if ((res = oledphy_initSPI()) != 0) {
		printf("SPI init error: %d\n", res);
		return res;
	}

	if ((res = oledphy_initGPIO()) != 0) {
		printf("GPIO init error: %d\n", res);
		return res;
	}

	if ((res = oledphy_reset()) != 0) {
		printf("RESET oled error: %d\n", res);
		return res;
	}

	oledphy_initLCD();

	return res;
}
