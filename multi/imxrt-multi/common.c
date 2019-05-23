/*
 * Phoenix-RTOS
 *
 * i.MX RT common
 *
 * Copyright 2019 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <sys/platform.h>
#include "common.h"


unsigned int multi_port;


int common_setClock(int dev, unsigned int state)
{
	platformctl_t pctl;

	pctl.action = pctl_set;
	pctl.type = pctl_devclock;
	pctl.devclock.dev = dev;
	pctl.devclock.state = state;

	return platformctl(&pctl);
}


int common_setMux(int mux, char sion, char mode)
{
	platformctl_t pctl;

	pctl.action = pctl_set;
	pctl.type = pctl_iomux;
	pctl.iomux.mux = mux;
	pctl.iomux.sion = sion;
	pctl.iomux.mode = mode;

	return platformctl(&pctl);
}


int common_setPad(int pad, char hys, char pus, char pue, char pke, char ode, char speed, char dse, char sre)
{
	platformctl_t pctl;

	pctl.action = pctl_set;
	pctl.type = pctl_iopad;
	pctl.iopad.pad = pad;
	pctl.iopad.hys = hys;
	pctl.iopad.pus = pus;
	pctl.iopad.pue = pue;
	pctl.iopad.pke = pke;
	pctl.iopad.ode = ode;
	pctl.iopad.speed = speed;
	pctl.iopad.dse = dse;
	pctl.iopad.sre = sre;

	return platformctl(&pctl);
}


int common_setInput(int isel, char daisy)
{
	platformctl_t pctl;

	pctl.action = pctl_set;
	pctl.type = pctl_ioisel;
	pctl.ioisel.isel = isel;
	pctl.ioisel.daisy = daisy;

	return platformctl(&pctl);
}
