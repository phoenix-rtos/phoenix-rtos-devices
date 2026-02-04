/*
 * Phoenix-RTOS
 *
 * i.MX RT1060 definitions
 *
 * Copyright 2019, 2024 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _IMXRT1060_H_
#define _IMXRT1060_H_

#include <phoenix/arch/armv7m/imxrt/10xx/imxrt10xx.h>
#include <phoenix/types.h>
#include <sys/platform.h>

#define UART_CLK 80000000

#define UART1_BASE ((void *)0x40184000)
#define UART2_BASE ((void *)0x40188000)
#define UART3_BASE ((void *)0x4018c000)
#define UART4_BASE ((void *)0x40190000)
#define UART5_BASE ((void *)0x40194000)
#define UART6_BASE ((void *)0x40198000)
#define UART7_BASE ((void *)0x4019c000)
#define UART8_BASE ((void *)0x401a0000)

#define UART9_BASE  ((void *)-1)
#define UART10_BASE ((void *)-1)
#define UART11_BASE ((void *)-1)
#define UART12_BASE ((void *)-1)

#define UART1_CLK pctl_clk_lpuart1
#define UART2_CLK pctl_clk_lpuart2
#define UART3_CLK pctl_clk_lpuart3
#define UART4_CLK pctl_clk_lpuart4
#define UART5_CLK pctl_clk_lpuart5
#define UART6_CLK pctl_clk_lpuart6
#define UART7_CLK pctl_clk_lpuart7
#define UART8_CLK pctl_clk_lpuart8

#define UART9_CLK  -1
#define UART10_CLK -1
#define UART11_CLK -1
#define UART12_CLK -1

#define UART1_IRQ 20 + 16
#define UART2_IRQ 21 + 16
#define UART3_IRQ 22 + 16
#define UART4_IRQ 23 + 16
#define UART5_IRQ 24 + 16
#define UART6_IRQ 25 + 16
#define UART7_IRQ 26 + 16
#define UART8_IRQ 27 + 16

#define UART9_IRQ  -1
#define UART10_IRQ -1
#define UART11_IRQ -1
#define UART12_IRQ -1

#define GPIO_PORTS 9

#define GPIO1_BASE ((void *)0x401b8000)
#define GPIO2_BASE ((void *)0x401bc000)
#define GPIO3_BASE ((void *)0x401c0000)
#define GPIO4_BASE ((void *)0x401c4000)
#define GPIO5_BASE ((void *)0x400c0000)
#define GPIO6_BASE ((void *)0x42000000)
#define GPIO7_BASE ((void *)0x42004000)
#define GPIO8_BASE ((void *)0x42008000)
#define GPIO9_BASE ((void *)0x4200c000)

#define GPIO10_BASE NULL
#define GPIO11_BASE NULL
#define GPIO12_BASE NULL
#define GPIO13_BASE NULL

#define GPIO1_CLK pctl_clk_gpio1
#define GPIO2_CLK pctl_clk_gpio2
#define GPIO3_CLK pctl_clk_gpio3
#define GPIO4_CLK pctl_clk_gpio4
#define GPIO5_CLK pctl_clk_gpio5

#define GPIO6_CLK  -1
#define GPIO7_CLK  -1
#define GPIO8_CLK  -1
#define GPIO9_CLK  -1
#define GPIO10_CLK -1
#define GPIO11_CLK -1
#define GPIO12_CLK -1
#define GPIO13_CLK -1

#define LPSPI1_BASE ((void *)0x40394000)
#define LPSPI2_BASE ((void *)0x40398000)
#define LPSPI3_BASE ((void *)0x4039c000)
#define LPSPI4_BASE ((void *)0x403a0000)

#define LPSPI1_CLK pctl_clk_lpspi1
#define LPSPI2_CLK pctl_clk_lpspi2
#define LPSPI3_CLK pctl_clk_lpspi3
#define LPSPI4_CLK pctl_clk_lpspi4

#define LPSPI1_IRQ 32 + 16
#define LPSPI2_IRQ 33 + 16
#define LPSPI3_IRQ 34 + 16
#define LPSPI4_IRQ 35 + 16

#define I2C1_BASE ((void *)0x403f0000)
#define I2C2_BASE ((void *)0x403f4000)
#define I2C3_BASE ((void *)0x403f8000)
#define I2C4_BASE ((void *)0x403fc000)

#define I2C1_CLK pctl_clk_lpi2c1
#define I2C2_CLK pctl_clk_lpi2c2
#define I2C3_CLK pctl_clk_lpi2c3
#define I2C4_CLK pctl_clk_lpi2c4

#define I2C1_IRQ 28 + 16
#define I2C2_IRQ 29 + 16
#define I2C3_IRQ 30 + 16
#define I2C4_IRQ 31 + 16

#define TRNG_BASE ((void *)0x400cc000)


/* TODO: Remove *_DEFAULT defines together with deprecation warnings in config.h */

#define UART1_TX_PIN_DEFAULT  ad_b0_12
#define UART1_RX_PIN_DEFAULT  ad_b0_13
#define UART1_RTS_PIN_DEFAULT ad_b0_15
#define UART1_CTS_PIN_DEFAULT ad_b0_14

#define UART2_TX_PIN_DEFAULT ad_b1_02
// #define UART2_TX_PIN_DEFAULT sd_b1_11
#define UART2_RX_PIN_DEFAULT ad_b1_03
// #define UART2_RX_PIN_DEFAULT sd_b1_10
#define UART2_RTS_PIN_DEFAULT ad_b1_01
#define UART2_CTS_PIN_DEFAULT ad_b1_00

#define UART3_TX_PIN_DEFAULT emc_13
// #define UART3_TX_PIN_DEFAULT ad_b1_06
// #define UART3_TX_PIN_DEFAULT b0_08
#define UART3_RX_PIN_DEFAULT emc_14
// #define UART3_RX_PIN_DEFAULT ad_b1_07
// #define UART3_RX_PIN_DEFAULT b0_09
#define UART3_RTS_PIN_DEFAULT emc_16
// #define UART3_RTS_PIN_DEFAULT ad_b1_05
#define UART3_CTS_PIN_DEFAULT emc_15
// #define UART3_CTS_PIN_DEFAULT ad_b1_04

#define UART4_TX_PIN_DEFAULT emc_19
// #define UART4_TX_PIN_DEFAULT b1_00
// #define UART4_TX_PIN_DEFAULT sd_b1_00
#define UART4_RX_PIN_DEFAULT emc_20
// #define UART4_RX_PIN_DEFAULT b1_01
// #define UART4_RX_PIN_DEFAULT sd_b1_01
#define UART4_RTS_PIN_DEFAULT emc_18
#define UART4_CTS_PIN_DEFAULT emc_17

#define UART5_TX_PIN_DEFAULT emc_23
// #define UART5_TX_PIN_DEFAULT b1_12
#define UART5_RX_PIN_DEFAULT emc_24
// #define UART5_RX_PIN_DEFAULT b1_13
#define UART5_RTS_PIN_DEFAULT emc_27
#define UART5_CTS_PIN_DEFAULT emc_28

#define UART6_TX_PIN_DEFAULT emc_25
// #define UART6_TX_PIN_DEFAULT ad_b0_12
#define UART6_RX_PIN_DEFAULT emc_26
// #define UART6_RX_PIN_DEFAULT ad_b0_03
#define UART6_RTS_PIN_DEFAULT emc_29
#define UART6_CTS_PIN_DEFAULT emc_30

#define UART7_TX_PIN_DEFAULT emc_31
// #define UART7_TX_PIN_DEFAULT sd_b1_08
#define UART7_RX_PIN_DEFAULT emc_32
// #define UART7_RX_PIN_DEFAULT sd_b1_09
#define UART7_RTS_PIN_DEFAULT sd_b1_07
#define UART7_CTS_PIN_DEFAULT sd_b1_06

#define UART8_TX_PIN_DEFAULT emc_38
// #define UART8_TX_PIN_DEFAULT ad_b1_10
// #define UART8_TX_PIN_DEFAULT sd_b0_04
#define UART8_RX_PIN_DEFAULT emc_39
// #define UART8_RX_PIN_DEFAULT ad_b1_11
// #define UART8_RX_PIN_DEFAULT sd_b0_05
#define UART8_RTS_PIN_DEFAULT sd_b0_03
#define UART8_CTS_PIN_DEFAULT sd_b0_02


#define SPI1_SCK_DEFAULT  sd_b0_00
#define SPI1_SDO_DEFAULT  sd_b0_02
#define SPI1_SDI_DEFAULT  sd_b0_03
#define SPI1_PCS0_DEFAULT sd_b0_01
#define SPI1_PCS1_DEFAULT
#define SPI1_PCS2_DEFAULT
#define SPI1_PCS3_DEFAULT

#define SPI2_SCK_DEFAULT  sd_b1_07
#define SPI2_SDO_DEFAULT  sd_b1_08
#define SPI2_SDI_DEFAULT  sd_b1_09
#define SPI2_PCS0_DEFAULT sd_b1_06
#define SPI2_PCS1_DEFAULT
#define SPI2_PCS2_DEFAULT
#define SPI2_PCS3_DEFAULT

#define SPI3_SCK_DEFAULT  ad_b0_00
#define SPI3_SDO_DEFAULT  ad_b0_01
#define SPI3_SDI_DEFAULT  ad_b0_02
#define SPI3_PCS0_DEFAULT ad_b0_03
#define SPI3_PCS1_DEFAULT
#define SPI3_PCS2_DEFAULT
#define SPI3_PCS3_DEFAULT

#define SPI4_SCK_DEFAULT  b0_03
#define SPI4_SDO_DEFAULT  b0_02
#define SPI4_SDI_DEFAULT  b0_01
#define SPI4_PCS0_DEFAULT b0_00
#define SPI4_PCS1_DEFAULT
#define SPI4_PCS2_DEFAULT
#define SPI4_PCS3_DEFAULT


/* TODO: Take care of I2C (defines + implementation) */

#ifndef I2C1_SCL_PIN
#define I2C1_SCL_PIN ad_b1_00
// #define I2C1_SCL_PIN sd_b1_04
#endif
#ifndef I2C1_SDA_PIN
#define I2C1_SDA_PIN ad_b1_01
// #define I2C1_SDA_PIN sd_b1_05
#endif

#ifndef I2C2_SCL_PIN
#define I2C2_SCL_PIN b0_04
// #define I2C2_SCL_PIN sd_b1_11
#endif
#ifndef I2C2_SDA_PIN
#define I2C2_SDA_PIN b0_05
// #define I2C2_SDA_PIN sd_b1_10
#endif

#ifndef I2C3_SCL_PIN
#define I2C3_SCL_PIN ad_b1_07
// #define I2C3_SCL_PIN emc_22
// #define I2C3_SCL_PIN sd_b0_00
#endif
#ifndef I2C3_SDA_PIN
#define I2C3_SDA_PIN ad_b1_06
// #define I2C3_SDA_PIN emc_21
// #define I2C3_SDA_PIN sd_b0_01
#endif

#ifndef I2C4_SCL_PIN
#define I2C4_SCL_PIN ad_b0_12
// #define I2C4_SCL_PIN emc_12
#endif
#ifndef I2C4_SDA_PIN
#define I2C4_SDA_PIN ad_b0_13
// #define I2C4_SDA_PIN emc_11
#endif


static union {
	unsigned int num;
	handle_t handle;
} gpio_interrupts[] __attribute__((unused)) = {
	{ .num = gpio1_0_15_irq },
	{ .num = gpio1_16_31_irq },
	{ .num = gpio2_0_15_irq },
	{ .num = gpio2_16_31_irq },
	{ .num = gpio3_0_15_irq },
	{ .num = gpio3_16_31_irq },
	{ .num = gpio4_0_15_irq },
	{ .num = gpio4_16_31_irq },
	{ .num = gpio5_0_15_irq },
	{ .num = gpio5_16_31_irq },
};


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
