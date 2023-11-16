/*
 * Phoenix-RTOS
 *
 * GRLIB GPIO driver
 *
 * Copyright 2023 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <errno.h>
#include <sys/types.h>
#include <sys/threads.h>

#include "common.h"
#include "gpio.h"
#include "grlib-multi.h"


#define GPIO_PORT_0 0
#define GPIO_PORT_1 1

#define GPIO_DIR_IN  0
#define GPIO_DIR_OUT 1

#define GPIO_PORT_CNT 2

#define GRGPIO0_BASE ((void *)0x8030C000)
#define GRGPIO1_BASE ((void *)0x8030D000)

/* GPIO registers */

#define GPIO_DATA      0       /* Port data reg : 0x00 */
#define GPIO_OUT       1       /* Output reg : 0x04 */
#define GPIO_DIR       2       /* Port direction reg : 0x08 */
#define GPIO_IMASK     3       /* Interrupt mask reg : 0x0C */
#define GPIO_IPOL      4       /* Interrupt polarity reg : 0x10 */
#define GPIO_IEDGE     5       /* Interrupt edge reg : 0x14 */
#define GPIO_CAP       7       /* Port capability reg : 0x1C */
#define GPIO_IRQMAP(n) (8 + n) /* Interrupt map register n : 0x20 - 0x3C */
#define GPIO_IAVAIL    16      /* Interrupt available reg : 0x40 */
#define GPIO_IFLAG     17      /* Interrupt flag reg : 0x44 */
#define GPIO_IPEN      18      /* Interrupt enable reg : 0x48 */
#define GPIO_PULSE     19      /* Pulse reg : 0x4C */
#define GPIO_IE_LOR    20      /* Interrupt enable logical OR reg : 0x50 */
#define GPIO_PO_LOR    21      /* Port output logical OR reg : 0x54 */
#define GPIO_PD_LOR    22      /* Port direction logical OR reg : 0x58 */
#define GPIO_IM_LOR    23      /* Interrupt mask logical OR reg : 0x5C */
#define GPIO_IE_LAND   24      /* Interrupt enable logical AND reg : 0x60 */
#define GPIO_PO_LAND   25      /* Port output logical AND reg : 0x64 */
#define GPIO_PD_LAND   26      /* Port direction logical AND reg : 0x68 */
#define GPIO_IM_LAND   27      /* Interrupt mask logical AND reg : 0x6C */
#define GPIO_IE_LXOR   28      /* Interrupt enable logical XOR reg : 0x70 */
#define GPIO_PO_LXOR   29      /* Port output logical XOR reg : 0x74 */
#define GPIO_PD_LXOR   30      /* Port direction logical XOR reg : 0x78 */
#define GPIO_IM_LXOR   31      /* Interrupt mask logical XOR reg : 0x7C */
#define GPIO_IE_SC     32      /* Interrupt enable logical set/clear reg : 0x80 - 0x8C */
#define GPIO_PO_SC     36      /* Port output logical set/clear reg : 0x90 - 0x9C */
#define GPIO_PD_SC     40      /* Port direction logical set/clear reg : 0xA0 - 0xAC */
#define GPIO_IM_SC     44      /* Interrupt mask logical set/clear reg : 0xB0 - 0xBC */


static struct {
	volatile uint32_t *grgpio0;
	volatile uint32_t *grgpio1;
} gpio_common;


static inline int gpio_pinToPort(uint8_t pin)
{
	return (pin >> 5);
}


static inline volatile uint32_t *gpio_portToAddr(int port)
{
	switch (port) {
		case GPIO_PORT_0:
			return gpio_common.grgpio0;
		case GPIO_PORT_1:
			return gpio_common.grgpio1;
		default:
			return NULL;
	}
}


static int gpio_setPort(int port, uint32_t mask, uint32_t val)
{
	uint32_t set = val & mask;
	uint32_t clear = (~val) & mask;

	volatile uint32_t *gpio = gpio_portToAddr(port);

	if (gpio == NULL) {
		return -EINVAL;
	}

	*(gpio + GPIO_PO_LAND) = ~clear;
	*(gpio + GPIO_PO_LOR) = set;

	return EOK;
}


static int gpio_getPort(int port, uint32_t *val)
{
	volatile uint32_t *gpio = gpio_portToAddr(port);

	if (gpio == NULL) {
		return -EINVAL;
	}

	*val = *(gpio + GPIO_DATA);

	return EOK;
}


static int gpio_setPortDir(int port, uint32_t mask, uint32_t val)
{
	uint32_t set = val & mask;
	uint32_t clear = (~val) & mask;

	volatile uint32_t *gpio = gpio_portToAddr(port);

	if (gpio == NULL) {
		return -EINVAL;
	}

	*(gpio + GPIO_PD_LAND) = ~clear;
	*(gpio + GPIO_PD_LOR) = set;

	return EOK;
}


static int gpio_getPortDir(int port, uint32_t *val)
{
	volatile uint32_t *gpio = gpio_portToAddr(port);

	if (gpio == NULL) {
		return -EINVAL;
	}

	*val = *(gpio + GPIO_DIR);

	return EOK;
}


static void gpio_handleDevCtl(msg_t *msg, int port)
{
	multi_i_t *idevctl = (multi_i_t *)msg->i.raw;
	multi_o_t *odevctl = (multi_o_t *)msg->o.raw;

	switch (idevctl->gpio.type) {
		case gpio_set_port:
			odevctl->err = gpio_setPort(port, idevctl->gpio.port.mask, idevctl->gpio.port.val);
			break;

		case gpio_get_port:
			odevctl->err = gpio_getPort(port, &odevctl->val);
			break;

		case gpio_set_dir:
			odevctl->err = gpio_setPortDir(port, idevctl->gpio.dir.mask, idevctl->gpio.dir.val);
			break;

		case gpio_get_dir:
			odevctl->err = gpio_getPortDir(port, &odevctl->val);
			break;

		default:
			odevctl->err = -EINVAL;
			break;
	}
}


void gpio_handleMsg(msg_t *msg, int dev)
{
	dev -= id_gpio0;

	if (dev >= GPIO_PORT_CNT) {
		return;
	}

	switch (msg->type) {
		case mtWrite:
		case mtRead:
			msg->o.io.err = EOK;
			break;

		case mtDevCtl:
			gpio_handleDevCtl(msg, dev);
			break;

		default:
			msg->o.io.err = -ENOSYS;
			break;
	}
}


int gpio_init(void)
{
	gpio_common.grgpio0 = GRGPIO0_BASE;
	gpio_common.grgpio1 = GRGPIO1_BASE;

	return 0;
}
