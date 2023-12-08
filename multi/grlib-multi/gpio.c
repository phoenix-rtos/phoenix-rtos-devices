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

#include <board_config.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/platform.h>
#include <sys/types.h>
#include <sys/threads.h>
#include <posix/utils.h>

#include "gpio.h"
#include "grlib-multi.h"

#if defined(__CPU_GR716)
#include <phoenix/arch/gr716.h>
#elif defined(__CPU_GR712RC)
#include <phoenix/arch/gr712rc.h>
#else
#error "Unsupported target"
#endif

#include <phoenix/arch/sparcv8leon3.h>

#define GPIO_PORT_0 0
#define GPIO_PORT_1 1

#define GPIO_DIR_IN  0
#define GPIO_DIR_OUT 1


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
	uint32_t *base;
	uint32_t irq;
} gpio_info[GPIO_PORT_CNT];


static struct {
	volatile uint32_t *vbase;
} gpio_common[GPIO_PORT_CNT];


static int gpio_setPortVal(int port, uint32_t mask, uint32_t val)
{
	if (port >= GPIO_PORT_CNT) {
		return -EINVAL;
	}

	volatile uint32_t *gpio = gpio_common[port].vbase;

	uint32_t set = val & mask;
	uint32_t clear = (~val) & mask;

	*(gpio + GPIO_PO_LAND) = ~clear;
	*(gpio + GPIO_PO_LOR) = set;

	return EOK;
}


static int gpio_getPortVal(int port, uint32_t *val)
{
	if (port >= GPIO_PORT_CNT) {
		return -EINVAL;
	}

	volatile uint32_t *gpio = gpio_common[port].vbase;

	*val = *(gpio + GPIO_DATA);

	return EOK;
}


static int gpio_setPortDir(int port, uint32_t mask, uint32_t val)
{
	if (port >= GPIO_PORT_CNT) {
		return -EINVAL;
	}

	volatile uint32_t *gpio = gpio_common[port].vbase;

	uint32_t set = val & mask;
	uint32_t clear = (~val) & mask;

	*(gpio + GPIO_PD_LAND) = ~clear;
	*(gpio + GPIO_PD_LOR) = set;

	return EOK;
}


static int gpio_getPortDir(int port, uint32_t *val)
{
	if (port >= GPIO_PORT_CNT) {
		return -EINVAL;
	}

	volatile uint32_t *gpio = gpio_common[port].vbase;

	*val = *(gpio + GPIO_DIR);

	return EOK;
}


static void gpio_handleDevCtl(msg_t *msg, int port)
{
	multi_i_t *idevctl = (multi_i_t *)msg->i.raw;
	multi_o_t *odevctl = (multi_o_t *)msg->o.raw;

	switch (idevctl->gpio.type) {
		case gpio_setPort:
			odevctl->err = gpio_setPortVal(port, idevctl->gpio.port.mask, idevctl->gpio.port.val);
			break;

		case gpio_getPort:
			odevctl->err = gpio_getPortVal(port, &odevctl->val);
			break;

		case gpio_setDir:
			odevctl->err = gpio_setPortDir(port, idevctl->gpio.dir.mask, idevctl->gpio.dir.val);
			break;

		case gpio_getDir:
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


int gpio_createDevs(oid_t *oid)
{
	for (unsigned int i = 0; i < GPIO_PORT_CNT; i++) {
		char buf[8];
		if (snprintf(buf, sizeof(buf), "gpio%u", i) >= sizeof(buf)) {
			return -1;
		}

		if (create_dev(oid, buf) < 0) {
			return -1;
		}
	}
	return 0;
}


int gpio_init(void)
{
	for (unsigned int i = 0; i < GPIO_PORT_CNT; i++) {

		unsigned int instance = i;
		ambapp_dev_t dev = { .devId = CORE_ID_GRGPIO };
		platformctl_t pctl = {
			.action = pctl_get,
			.type = pctl_ambapp,
			.ambapp = {
				.dev = &dev,
				.instance = &instance,
			}
		};

		if (platformctl(&pctl) < 0) {
			return -1;
		}

		if (dev.bus != BUS_AMBA_APB) {
			/* GRGPIO should be on APB bus */
			return -1;
		}
		gpio_info[i].base = dev.info.apb.base;
		gpio_info[i].irq = dev.irqn;

		uintptr_t base = ((uintptr_t)gpio_info[i].base & ~(_PAGE_SIZE - 1));
		gpio_common[i].vbase = mmap(NULL, _PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)base);
		if (gpio_common[i].vbase == MAP_FAILED) {
			return -1;
		}

		gpio_common[i].vbase += ((uintptr_t)gpio_info[i].base - base) / sizeof(uintptr_t);
	}

	return 0;
}
