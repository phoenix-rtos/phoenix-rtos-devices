/*
 * Phoenix-RTOS
 *
 * OLED device interface
 *
 * Copyright 2018 Phoenix Systems
 * Author: Andrzej Glowinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <sys/stat.h>
#include <sys/file.h>
#include <sys/msg.h>
#include <sys/threads.h>

#include <board_config.h>

#include "oled-phy.h"
#include "oled-graphic.h"
#include "fonts/font.h"
#include "oled-api.h"

#ifndef OLED128O064B0_PRIO
#define OLED128O064B0_PRIO 4
#endif


struct {
	uint32_t port;
} dev_common;


int dev_init(void)
{
	int err;
	msg_t msg;
	oid_t dir;

	if (portCreate(&dev_common.port) != EOK) {
		return -1;
	}

	err = mkdir("/dev", 0);
	if (err < 0 && errno != EEXIST) {
		return -1;
	}

	if (lookup("/dev", NULL, &dir) < 0) {
		return -1;
	}

	msg.type = mtCreate;
	msg.i.create.dir = dir;
	msg.i.create.type = otDev;
	msg.i.create.mode = 0;
	msg.i.create.dev.port = dev_common.port;
	msg.i.create.dev.id = 0;
	msg.i.data = "oled";
	msg.i.size = sizeof("oled");
	msg.o.data = NULL;
	msg.o.size = 0;

	if (msgSend(dir.port, &msg) < 0 || msg.o.create.err != EOK) {
		return - 1;
	}

	return 0;
}

static int dev_write(oled_write_t *cmd)
{
	switch(cmd->type) {
		case oled_write__rect:
			oledgraph_fillRect(cmd->x, cmd->y, cmd->w, cmd->h, cmd->filled);
			break;
		case oled_write__bitmap:
			oledgraph_fillBitmap(cmd->x, cmd->y, cmd->w, cmd->h, cmd->data);
			break;
		case oled_write__text_abs:
			oledgraph_drawStringAbs(cmd->x, cmd->y, cmd->w, cmd->h, font_5x7,
				strlen(cmd->text), cmd->text);
			break;
		case oled_write__text_cont:
			oledgraph_drawStringCont(cmd->x, cmd->y, cmd->w, cmd->h, font_5x7,
				strlen(cmd->text), cmd->text);
			break;
		case oled_write__clear:
			oledgraph_fillRect(cmd->x, cmd->y, cmd->w, cmd->h, 0);
			break;
		case oled_write__draw:
			oledgraph_drawBuffer(cmd->x, cmd->y, cmd->w, cmd->h, 1);
			break;
		default:
			return -EINVAL;
	}

	return EOK;
}

void msg_loop(void)
{
	msg_t msg;
	msg_rid_t rid;

	while (1) {
		if (msgRecv(dev_common.port, &msg, &rid) < 0)
			continue;
		switch (msg.type) {
			case mtOpen:
			case mtClose:
			case mtTruncate:
				msg.o.io.err = EOK;
				break;
			case mtRead:
				if (msg.o.data != NULL && msg.o.size >= sizeof(char)) {
					msg.o.io.err = EOK;
				}
				else {
					msg.o.io.err = -EINVAL;
				}
				break;
			case mtWrite:
				if (msg.i.data != NULL && msg.i.size == sizeof(oled_write_t *)) {
					msg.o.io.err = dev_write((oled_write_t *)msg.i.data);
				}
				else {
					msg.o.io.err = EOK;
				}
				break;
			default:
				msg.o.io.err = -EINVAL;
		}
		msgRespond(dev_common.port, &msg, rid);
	}
}

int main(void)
{
	oid_t root;
	/* Wait for the filesystem */
	while (lookup("/", NULL, &root) < 0)
		usleep(10000);

	/* Wait for the console */
	while (write(1, "", 0) < 0)
		usleep(10000);
	oledphy_init();

	if (dev_init())
		return -EIO;

	/* Rotate by 180 degrees */
	oledphy_sendCmd(0xc0);
	oledphy_sendCmd(0xa0);

	/* Turn on the screen */
	oledphy_sendCmd(0xaf);

	oledgraph_reset(0, 0, 128, 64);

	priority(OLED128O064B0_PRIO);
	msg_loop();

	return 0;
}
