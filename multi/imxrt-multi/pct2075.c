/*
 * Phoenix-RTOS
 *
 * i.MX RT integrated PCT2075 driver
 *
 * Copyright 2021, 2024 Phoenix Systems
 * Author: Marek Bialowas, Jacek Maksymowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "common.h"
#include "i2c.h"
#if PCT2075


static const uint8_t TEMP_REG_ADDR = 0x00u; /* stored temperature value - 16bit */


/* Get temperature in miliCelsius */
static int pct2075_getTemp(int32_t *tempOut)
{
	uint8_t tempBytes[2];
	int busNum = PCT2075_BUS_NUM;
	if (busNum <= 0) {
		return -EINVAL;
	}

	int ret = multi_i2c_regRead(busNum - 1, PCT2075_DEV_ADDR, TEMP_REG_ADDR, tempBytes, sizeof(tempBytes));
	if (ret < 0) {
		return ret;
	}

	int16_t tempRaw = ((int16_t)tempBytes[0] << 8) | tempBytes[1];
	*tempOut = (tempRaw >> 5) * 125;
	return EOK;
}


void pct2075_handleMsg(msg_t *msg)
{
	switch (msg->type) {
		case mtOpen:
		case mtClose:
			msg->o.err = EOK;
			break;
		case mtWrite:
			msg->o.err = -EINVAL;
			break;
		case mtRead:
			/* don't support partial reads, signal EOF */
			if (msg->i.io.offs > 0) {
				msg->o.err = 0; /* EOF */
			}
			else {
				int32_t temp;
				int ret = pct2075_getTemp(&temp);
				if (ret == EOK) {
					ret = snprintf(msg->o.data, msg->o.size, "%d\n", temp);
					if ((ret > 0) && ((size_t)ret > msg->o.size)) {
						ret = msg->o.size;
					}
				}

				msg->o.err = ret;
			}
			break;
		default:
			msg->o.err = -ENOSYS;
			break;
	}
}

#endif /* PCT2075 */
