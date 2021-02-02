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

#define UART_CLK 24000000

#define UART1_BASE  ((void *)0x4007c000)
#define UART2_BASE  ((void *)0x40080000)
#define UART3_BASE  ((void *)0x40084000)
#define UART4_BASE  ((void *)0x40088000)
#define UART5_BASE  ((void *)0x4008c000)
#define UART6_BASE  ((void *)0x40090000)
#define UART7_BASE  ((void *)0x40094000)
#define UART8_BASE  ((void *)0x40098000)
#define UART9_BASE  ((void *)0x4009c000)
#define UART10_BASE ((void *)0x400a0000)
#define UART11_BASE ((void *)0x40c24000)
#define UART12_BASE ((void *)0x40c28000)

#define UART1_CLK  pctl_clk_lpuart1
#define UART2_CLK  pctl_clk_lpuart2
#define UART3_CLK  pctl_clk_lpuart3
#define UART4_CLK  pctl_clk_lpuart4
#define UART5_CLK  pctl_clk_lpuart5
#define UART6_CLK  pctl_clk_lpuart6
#define UART7_CLK  pctl_clk_lpuart7
#define UART8_CLK  pctl_clk_lpuart8
#define UART9_CLK  pctl_clk_lpuart9
#define UART10_CLK pctl_clk_lpuart10
#define UART11_CLK pctl_clk_lpuart11
#define UART12_CLK pctl_clk_lpuart12

#define UART1_IRQ  lpuart1_irq
#define UART2_IRQ  lpuart2_irq
#define UART3_IRQ  lpuart3_irq
#define UART4_IRQ  lpuart4_irq
#define UART5_IRQ  lpuart5_irq
#define UART6_IRQ  lpuart6_irq
#define UART7_IRQ  lpuart7_irq
#define UART8_IRQ  lpuart8_irq
#define UART9_IRQ  lpuart9_irq
#define UART10_IRQ lpuart10_irq
#define UART11_IRQ lpuart11_irq
#define UART12_IRQ lpuart12_irq

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

#ifndef UART1_TX_PIN
#define UART1_TX_PIN ad_24
//#define UART1_TX_PIN disp_b1_02
//#define UART1_TX_PIN disp_b2_08
#endif
#ifndef UART1_RX_PIN
#define UART1_RX_PIN ad_25
//#define UART1_RX_PIN disp_b1_03
//#define UART1_RX_PIN disp_b2_09
#endif
#define UART1_RTS_PIN ad_27
#define UART1_CTS_PIN ad_26

#define UART2_TX_PIN disp_b2_10
#define UART2_RX_PIN disp_b2_11
#define UART2_RTS_PIN disp_b2_13
#define UART2_CTS_PIN disp_b2_12

#define UART3_TX_PIN ad_30
#define UART3_RX_PIN ad_31
#define UART3_RTS_PIN sd_b2_08
#define UART3_CTS_PIN sd_b2_07

#define UART4_TX_PIN disp_b1_06
#define UART4_RX_PIN disp_b1_04
#define UART4_RTS_PIN disp_b1_07
#define UART4_CTS_PIN disp_b1_05

#define UART5_TX_PIN ad_28
#define UART5_RX_PIN ad_29
#define UART5_RTS_PIN sd_b2_10
#define UART5_CTS_PIN sd_b2_09

#define UART6_TX_PIN emc_b1_40
#define UART6_RX_PIN emc_b1_41
#define UART6_RTS_PIN emc_b2_01
#define UART6_CTS_PIN emc_b2_00

#ifndef UART7_TX_PIN
#define UART7_TX_PIN disp_b2_06
//#define UART7_TX_PIN ad_00
#endif
#ifndef UART7_RX_PIN
#define UART7_RX_PIN disp_b2_07
//#define UART7_RX_PIN ad_01
#endif
#define UART7_RTS_PIN ad_02
#define UART7_CTS_PIN ad_03

#ifndef UART8_TX_PIN
#define UART8_TX_PIN disp_b2_08
//#define UART8_TX_PIN ad_02
#endif
#ifndef UART8_RX_PIN
#define UART8_RX_PIN disp_b2_09
//#define UART8_RX_PIN ad_03
#endif
#define UART8_RTS_PIN ad_05
#define UART8_CTS_PIN ad_04

#define UART9_TX_PIN sd_b2_00
#define UART9_RX_PIN sd_b2_01
#define UART9_RTS_PIN sd_b2_03
#define UART9_CTS_PIN sd_b2_02

#ifndef UART10_TX_PIN
#define UART10_TX_PIN ad_15
//#define UART10_TX_PIN ad_32
#endif
#ifndef UART10_RX_PIN
#define UART10_RX_PIN ad_16
//#define UART10_RX_PIN ad_33
#endif
#define UART10_RTS_PIN ad_35
#define UART10_CTS_PIN ad_34

#ifndef UART11_TX_PIN
#define UART11_TX_PIN lpsr_08
//#define UART11_TX_PIN lpsr_04
#endif
#ifndef UART11_RX_PIN
#define UART11_RX_PIN lpsr_09
//#define UART11_RX_PIN lpsr_05
#endif
#define UART11_RTS_PIN lpsr_11
#define UART11_CTS_PIN lpsr_10

#ifndef UART12_TX_PIN
//#define UART12_TX_PIN lpsr_06
#define UART12_TX_PIN lpsr_00
//#define UART12_TX_PIN lpsr_10
#endif
#ifndef UART12_RX_PIN
//#define UART12_RX_PIN lpsr_07
#define UART12_RX_PIN lpsr_01
//#define UART12_RX_PIN lpsr_11
#endif
#define UART12_RTS_PIN lpsr_04
#define UART12_CTS_PIN lpsr_05


static inline int common_setClock(int clock, int div, int mux, int mfd, int mfn, int state)
{
	platformctl_t pctl;

	pctl.action = pctl_set;
	pctl.type = pctl_devclock;
	pctl.devclock.dev = clock;
	pctl.devclock.div = div;
	pctl.devclock.mfd = mfd;
	pctl.devclock.mfn = mfn;
	pctl.devclock.mux = mux;
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
