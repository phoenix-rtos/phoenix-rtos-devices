/*
 * Phoenix-RTOS
 *
 * OLED physical interface
 *
 * Copyright 2020 Phoenix Systems
 * Author: Hubert Buczyński
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _OLED_PHY_H_
#define _OLED_PHY_H_


#include <stdint.h>


extern int oledphy_sendCmd(uint8_t cmd);

extern int oledphy_sendData(uint8_t data);

extern int oledphy_sendDataBuf(uint8_t *data, uint8_t size);

extern int oledphy_init(void);

extern int oledphy_setPos(uint8_t x, uint8_t y);

extern int oledphy_reset(void);


#endif
