/*
 * Phoenix-RTOS
 *
 * i.MX RT1170 definitions
 *
 * Copyright 2019 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _IMXRT1179_H_
#define _IMXRT1179_H_

#include <phoenix/arch/imxrt1170.h>
#include <sys/platform.h>

#define UART_CLK 64000000


#define UART1_BASE ((void *)0x4007c000)
#define UART2_BASE ((void *)NULL)
#define UART3_BASE ((void *)NULL)
#define UART4_BASE ((void *)NULL)
#define UART5_BASE ((void *)NULL)
#define UART6_BASE ((void *)NULL)
#define UART7_BASE ((void *)NULL)
#define UART8_BASE ((void *)NULL)

#define UART1_CLK -1
#define UART2_CLK -1
#define UART3_CLK -1
#define UART4_CLK -1
#define UART5_CLK -1
#define UART6_CLK -1
#define UART7_CLK -1
#define UART8_CLK -1

#define UART1_IRQ 20 + 16
#define UART2_IRQ 21 + 16
#define UART3_IRQ 22 + 16
#define UART4_IRQ 23 + 16
#define UART5_IRQ 24 + 16
#define UART6_IRQ 25 + 16
#define UART7_IRQ 26 + 16
#define UART8_IRQ 27 + 16

#define GPIO1_BASE ((void *)NULL)
#define GPIO2_BASE ((void *)NULL)
#define GPIO3_BASE ((void *)NULL)
#define GPIO4_BASE ((void *)NULL)
#define GPIO5_BASE ((void *)NULL)
#define GPIO6_BASE ((void *)NULL)
#define GPIO7_BASE ((void *)NULL)
#define GPIO8_BASE ((void *)NULL)
#define GPIO9_BASE ((void *)NULL)

#define GPIO1_CLK -1
#define GPIO2_CLK -1
#define GPIO3_CLK -1
#define GPIO4_CLK -1
#define GPIO5_CLK -1

#define UART1_TX_PIN ad_24
#define UART1_RX_PIN ad_25
#define UART1_RTS_PIN ad_27
#define UART1_CTS_PIN ad_26


static inline int common_setClock(int dev, unsigned int state)
{
	platformctl_t pctl;

	pctl.action = pctl_set;
	pctl.type = pctl_devclock;
	pctl.devclock.dev = dev;
	pctl.devclock.state = state;

	return platformctl(&pctl);
}


static inline int common_setMux(int mux, char sion, char mode)
{
	platformctl_t pctl;

	pctl.action = pctl_set;
	pctl.type = pctl_iomux;
	pctl.iomux.mux = mux;
	pctl.iomux.sion = sion;
	pctl.iomux.mode = mode;

	return platformctl(&pctl);
}


static inline int common_setPad(int pad, char hys, char pus, char pue, char pke, char ode, char speed, char dse, char sre)
{
	platformctl_t pctl;

	pctl.action = pctl_set;
	pctl.type = pctl_iopad;
	pctl.iopad.pad = pad;
	pctl.iopad.pus = pus;
	pctl.iopad.pue = pue;
	pctl.iopad.ode = ode;
	pctl.iopad.dse = dse;
	pctl.iopad.sre = sre;
	pctl.iopad.apc = 0;

	return platformctl(&pctl);
}


static inline int common_setInput(int isel, char daisy)
{
	platformctl_t pctl;

	pctl.action = pctl_set;
	pctl.type = pctl_ioisel;
	pctl.ioisel.isel = isel;
	pctl.ioisel.daisy = daisy;

	return platformctl(&pctl);
}


#endif
