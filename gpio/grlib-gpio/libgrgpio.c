/*
 * Phoenix-RTOS
 *
 * GRLIB GPIO driver
 *
 * Copyright 2026 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <errno.h>
#include <stdio.h>
#include <sys/mman.h>
#include <sys/platform.h>
#include <sys/types.h>
#include <posix/utils.h>

#include <phoenix/gaisler/ambapp.h>

#ifdef __CPU_GR765
#include <phoenix/arch/riscv64/riscv64.h>
#else
#include <phoenix/arch/sparcv8leon/sparcv8leon.h>
#endif

#include "libgrgpio.h"


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


static void grgpio_setPortVal(grgpio_ctx_t *ctx, uint32_t mask, uint32_t val)
{
	volatile uint32_t *gpio = ctx->vbase;

	uint32_t set = val & mask;
	uint32_t clear = (~val) & mask;

	*(gpio + GPIO_PO_LAND) = ~clear;
	*(gpio + GPIO_PO_LOR) = set;
}


static uint32_t grgpio_getPortVal(grgpio_ctx_t *ctx)
{
	volatile uint32_t *gpio = ctx->vbase;

	return *(gpio + GPIO_DATA);
}


static void grgpio_setPortDir(grgpio_ctx_t *ctx, uint32_t mask, uint32_t val)
{
	volatile uint32_t *gpio = ctx->vbase;

	uint32_t set = val & mask;
	uint32_t clear = (~val) & mask;

	*(gpio + GPIO_PD_LAND) = ~clear;
	*(gpio + GPIO_PD_LOR) = set;
}


static uint32_t grgpio_getPortDir(grgpio_ctx_t *ctx)
{
	volatile uint32_t *gpio = ctx->vbase;

	return *(gpio + GPIO_DIR);
}


static void grgpio_handleDevCtl(grgpio_ctx_t *ctx, msg_t *msg)
{
	gpio_i_t *idevctl = (gpio_i_t *)msg->i.raw;
	gpio_o_t *odevctl = (gpio_o_t *)msg->o.raw;

	switch (idevctl->type) {
		case gpio_setPort:
			grgpio_setPortVal(ctx, idevctl->mask, idevctl->val);
			msg->o.err = 0;
			break;

		case gpio_getPort:
			odevctl->val = grgpio_getPortVal(ctx);
			msg->o.err = 0;
			break;

		case gpio_setDir:
			grgpio_setPortDir(ctx, idevctl->mask, idevctl->val);
			msg->o.err = 0;
			break;

		case gpio_getDir:
			odevctl->val = grgpio_getPortDir(ctx);
			msg->o.err = 0;
			break;

		default:
			msg->o.err = -EINVAL;
			break;
	}
}


void grgpio_handleMsg(grgpio_ctx_t *ctx, msg_t *msg)
{
	switch (msg->type) {
		case mtOpen:
		case mtClose:
			msg->o.err = 0;
			break;

		case mtDevCtl:
			grgpio_handleDevCtl(ctx, msg);
			break;

		default:
			msg->o.err = -ENOSYS;
			break;
	}
}


int grgpio_createDev(oid_t *oid, unsigned int portNum)
{
	char buf[8];
	if (snprintf(buf, sizeof(buf), "gpio%u", portNum) >= (int)sizeof(buf)) {
		return -1;
	}

	return create_dev(oid, buf);
}


int grgpio_init(grgpio_ctx_t *ctx, unsigned int instance)
{
	ambapp_dev_t dev = { .devId = CORE_ID_GRGPIO };
	platformctl_t pctl = {
		.action = pctl_get,
		.type = pctl_ambapp,
		.task.ambapp = {
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
	uintptr_t pbase = (uintptr_t)dev.info.apb.base;

	uintptr_t base = (pbase & ~(_PAGE_SIZE - 1));
	ctx->vbase = mmap(NULL, _PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)base);
	if (ctx->vbase == MAP_FAILED) {
		return -1;
	}

	ctx->vbase = (void *)((uintptr_t)ctx->vbase + (pbase - base));

	return 0;
}
