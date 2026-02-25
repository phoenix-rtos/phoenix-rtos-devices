/*
 * Phoenix-RTOS
 *
 * i.MX RT GPIO driver
 *
 * Copyright 2019 Phoenix Systems
 * Author: Aleksander Kaminski, Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <stdint.h>
#include <errno.h>
#include <sys/msg.h>
#include <sys/platform.h>
#include <sys/threads.h>

#include "common.h"
#include "gpio.h"

#include "board_config.h"

#ifndef GPIO_SUPPORT_ASYNC_IRQ_WAIT
#define GPIO_SUPPORT_ASYNC_IRQ_WAIT 0
#endif


#if GPIO_SUPPORT_ASYNC_IRQ_WAIT
#include <string.h>
#include <sys/interrupt.h>

#define GPIO_IRQ_THREAD_PRIO    2
#define GPIO_IRQ_THREAD_STACKSZ 2048

#define GPIO_IRQ_REQ_CACHE_SIZE GPIO_PORTS
#endif /* GPIO_SUPPORT_ASYNC_IRQ_WAIT */


/* clang-format off */

enum { gpio_dr = 0, gpio_gdir, gpio_psr, gpio_icr1, gpio_icr2, gpio_imr,
	gpio_isr, gpio_edge_sel, gpio_dr_set, gpio_dr_clear, gpio_dr_toggle };

/* clang-format on */


#if GPIO_SUPPORT_ASYNC_IRQ_WAIT
typedef struct gpio_async_req {
	struct gpio_async_req *next;
	msg_t msg;
	msg_rid_t rid;
	int port; /* -1: free, != -1: taken */
} gpio_async_req_t;
#endif /* GPIO_SUPPORT_ASYNC_IRQ_WAIT */


static struct {
	volatile uint32_t *base[GPIO_PORTS];
	handle_t lock;

#if GPIO_SUPPORT_ASYNC_IRQ_WAIT
	struct {
		struct {
			gpio_async_req_t *head;
			size_t len;
			handle_t lock;
		} req_list;
		handle_t lock, cond;
		uint8_t stack[GPIO_IRQ_THREAD_STACKSZ];
	} irq;

#endif /* GPIO_SUPPORT_ASYNC_IRQ_WAIT */
} gpio_common;


int gpio_setPort(int port, uint32_t mask, uint32_t val)
{
	unsigned int set, clr, t;

	if ((port <= 0) || (port > GPIO_PORTS)) {
		return -EINVAL;
	}

	port -= 1;
	set = val & mask;
	clr = ~val & mask;

	/* DR_SET & DR_CLEAR registers are not functional */
	mutexLock(gpio_common.lock);
	t = *(gpio_common.base[port] + gpio_dr) & ~clr;
	*(gpio_common.base[port] + gpio_dr) = t | set;
	mutexUnlock(gpio_common.lock);
	return EOK;
}


int gpio_getPort(int port, uint32_t *val)
{
	if ((port <= 0) || (port > GPIO_PORTS)) {
		return -EINVAL;
	}

	port -= 1;
	*val = *(gpio_common.base[port] + gpio_dr);
	return EOK;
}


int gpio_setDir(int port, uint32_t mask, uint32_t val)
{
	unsigned int set, clr, t;

	if ((port <= 0) || (port > GPIO_PORTS)) {
		return -EINVAL;
	}

	port -= 1;
	set = val & mask;
	clr = ~val & mask;

	mutexLock(gpio_common.lock);
	t = *(gpio_common.base[port] + gpio_gdir) & ~clr;
	*(gpio_common.base[port] + gpio_gdir) = t | set;
	mutexUnlock(gpio_common.lock);
	return EOK;
}


int gpio_getDir(int port, uint32_t *val)
{
	if ((port <= 0) || (port > GPIO_PORTS)) {
		return -EINVAL;
	}

	port -= 1;
	*val = *(gpio_common.base[port] + gpio_gdir);
	return EOK;
}


int gpio_setIrqConf(int port, unsigned int pin, int type)
{
	int reg;
	uint32_t t;

	if ((port <= 0) || (port > GPIO_PORTS)) {
		return -EINVAL;
	}
	if (pin > 31) {
		return -EINVAL;
	}
	if ((type < gpio_irq_active_low) || (type > gpio_irq_active_falling)) {
		return -EINVAL;
	}

	port -= 1;
	reg = (pin < 16) ? gpio_icr1 : gpio_icr2;
	pin = (pin & (16 - 1)) * 2;

	mutexLock(gpio_common.lock);
	t = *(gpio_common.base[port] + reg);
	t &= ~(3u << pin);
	t |= (type << pin);
	*(gpio_common.base[port] + reg) = t;
	mutexUnlock(gpio_common.lock);

	return EOK;
}


int gpio_getIrqConf(int port, unsigned int pin, uint32_t *val)
{
	int reg;

	if ((port <= 0) || (port > GPIO_PORTS)) {
		return -EINVAL;
	}
	if (pin > 31) {
		return -EINVAL;
	}

	port -= 1;
	reg = (pin < 16) ? gpio_icr1 : gpio_icr2;
	pin = (pin & (16 - 1)) * 2;

	*val = (*(gpio_common.base[port] + reg) >> pin) & 0x3;

	return EOK;
}


int gpio_waitIrq(msg_t *msg, msg_rid_t rid, int port, uint32_t mask, uint32_t *val)
{
	if ((port <= 0) || (port > GPIO_PORTS)) {
		return -EINVAL;
	}
	if (mask == 0) {
		return -EINVAL;
	}
	port -= 1;

#if GPIO_SUPPORT_ASYNC_IRQ_WAIT

	mutexLock(gpio_common.irq.req_list.lock);
	gpio_async_req_t *req = gpio_common.irq.req_list.head;

	while (req != NULL) {
		if (req->port == -1) {
			break;
		}
		req = req->next;
	}

	if (req == NULL || req->port != -1) {
		req = malloc(sizeof(*req));
		if (req == NULL) {
			return -ENOMEM;
		}
		req->next = gpio_common.irq.req_list.head;
		gpio_common.irq.req_list.head = req;
		gpio_common.irq.req_list.len++;
	}

	memcpy(&req->msg, msg, sizeof(*msg));
	req->rid = rid;
	req->port = port;

	/* clear status & enable requested interrupt(s) */
	mutexLock(gpio_common.lock);
	*(gpio_common.base[port] + gpio_isr) = mask;
	*(gpio_common.base[port] + gpio_imr) |= mask;
	mutexUnlock(gpio_common.lock);

	mutexUnlock(gpio_common.irq.req_list.lock);

	return EWOULDBLOCK;

#else

	uint32_t status = *(gpio_common.base[port] + gpio_isr) & mask;

	mutexLock(gpio_common.lock);
	*(gpio_common.base[port] + gpio_isr) = mask;
	/* no need to enable interrupts - ISR is always asserted
	   when interrupt condition occurs */
	mutexUnlock(gpio_common.lock);

	if (status == 0) {
		/* interrupt did not occur: return -EWOULDBLOCK */
		return -EWOULDBLOCK;
	}

	*val = status;
	return EOK;

#endif /* GPIO_SUPPORT_ASYNC_IRQ_WAIT */
}


static void gpio_handleDevCtl(msg_t *msg, msg_rid_t rid, int port)
{
	multi_i_t *imsg = (multi_i_t *)msg->i.raw;
	multi_o_t *omsg = (multi_o_t *)msg->o.raw;

	msg->o.err = EOK;

	switch (imsg->gpio.type) {
		case gpio_set_port:
			msg->o.err = gpio_setPort(port, imsg->gpio.port.mask, imsg->gpio.port.val);
			break;

		case gpio_get_port:
			msg->o.err = gpio_getPort(port, &omsg->val);
			break;

		case gpio_set_dir:
			msg->o.err = gpio_setDir(port, imsg->gpio.dir.mask, imsg->gpio.dir.val);
			break;

		case gpio_get_dir:
			msg->o.err = gpio_getDir(port, &omsg->val);
			break;

		case gpio_set_irq_conf:
			msg->o.err = gpio_setIrqConf(port, imsg->gpio.irq_conf.pin, imsg->gpio.irq_conf.type);
			break;

		case gpio_get_irq_conf:
			msg->o.err = gpio_getIrqConf(port, imsg->gpio.irq_conf.pin, &omsg->val);
			break;

		case gpio_wait_irq:
			msg->o.err = gpio_waitIrq(msg, rid, port, imsg->gpio.wait_irq.mask, &omsg->val);
			break;

		default:
			msg->o.err = -ENOSYS;
			break;
	}
}


int gpio_handleMsg(msg_t *msg, msg_rid_t rid, int dev)
{
	dev -= id_gpio1 - 1; /* Port number will be verified later */

	switch (msg->type) {
		case mtGetAttr:
		case mtSetAttr:
		case mtOpen:
		case mtClose:
		case mtWrite:
		case mtRead:
			msg->o.err = EOK;
			break;

		case mtDevCtl:
			gpio_handleDevCtl(msg, rid, dev);
			break;

		default:
			msg->o.err = -ENOSYS;
			break;
	}

	return 0;
}


#if GPIO_SUPPORT_ASYNC_IRQ_WAIT
static void _gpio_disableInterrupts(void)
{
	for (int i = 0; i < sizeof(gpio_common.base) / sizeof(gpio_common.base[0]); i++) {
		*(gpio_common.base[i] + gpio_imr) = 0x0;
	}
}


static int gpio_handleIntr(unsigned int n, void *arg)
{
	(void)arg;
	_gpio_disableInterrupts();
	return 0;
}


static void gpio_intrThread(void *arg)
{
	(void)arg;
	gpio_async_req_t *curr;
	gpio_async_req_t *prev;
	gpio_async_req_t *del;
	multi_i_t *imsg;
	multi_o_t *omsg;
	uint32_t status;

	mutexLock(gpio_common.irq.lock);
	for (;;) {
		condWait(gpio_common.irq.cond, gpio_common.irq.lock, 0);

		mutexLock(gpio_common.irq.req_list.lock);
		if (gpio_common.irq.req_list.head == NULL) {
			mutexUnlock(gpio_common.irq.req_list.lock);
			continue;
		}

		curr = gpio_common.irq.req_list.head;
		prev = NULL;

		while (curr != NULL) {
			if (curr->port == -1) {
				prev = curr;
				curr = curr->next;
				continue;
			}

			imsg = (multi_i_t *)curr->msg.i.raw;
			omsg = (multi_o_t *)curr->msg.o.raw;

			status = *(gpio_common.base[curr->port] + gpio_isr) & imsg->gpio.wait_irq.mask;

			if (status == 0) {
				/* re-enable IRQ mask*/
				mutexLock(gpio_common.lock);
				*(gpio_common.base[curr->port] + gpio_imr) |= imsg->gpio.wait_irq.mask;
				mutexUnlock(gpio_common.lock);

				prev = curr;
				curr = curr->next;
				continue;
			}

			/* reset IRQ status */
			mutexLock(gpio_common.lock);
			*(gpio_common.base[curr->port] + gpio_isr) = status;
			mutexUnlock(gpio_common.lock);

			/* respond with bitmask of all IRQs requested that happened */
			omsg->val = status;
			curr->msg.o.err = EOK;
			msgRespond(multi_port, &curr->msg, curr->rid);

			if (gpio_common.irq.req_list.len > GPIO_IRQ_REQ_CACHE_SIZE) {
				del = curr;

				if (prev == NULL) {
					gpio_common.irq.req_list.head = curr->next;
				}
				else {
					prev->next = curr->next;
				}

				curr = curr->next;
				free(del);
				gpio_common.irq.req_list.len--;
			}
			else {
				curr->port = -1;
				prev = curr;
				curr = curr->next;
			}
		}

		mutexUnlock(gpio_common.irq.req_list.lock);
	}
	mutexUnlock(gpio_common.irq.lock);
	endthread();
}


static int gpio_initInterrupts(void)
{
	int i;

	mutexLock(gpio_common.lock);
	_gpio_disableInterrupts();
	mutexUnlock(gpio_common.lock);

	for (i = 0; i < sizeof(gpio_interrupts) / sizeof(gpio_interrupts[0]); i++) {
		if (interrupt(gpio_interrupts[i].num, gpio_handleIntr, NULL, gpio_common.irq.cond, &gpio_interrupts[i].handle) < 0) {
			return -ENOMEM;
		}
	}

	return 0;
}


static int gpio_initAsync(void)
{
	int err;

	if (mutexCreate(&gpio_common.irq.req_list.lock) < 0) {
		return -ENOMEM;
	}
	if (mutexCreate(&gpio_common.irq.lock) < 0) {
		resourceDestroy(gpio_common.irq.req_list.lock);
		return -ENOMEM;
	}
	if (condCreate(&gpio_common.irq.cond) < 0) {
		resourceDestroy(gpio_common.irq.req_list.lock);
		resourceDestroy(gpio_common.irq.lock);
		return -ENOMEM;
	}

	err = gpio_initInterrupts();
	if (err < 0) {
		resourceDestroy(gpio_common.irq.req_list.lock);
		resourceDestroy(gpio_common.irq.lock);
		resourceDestroy(gpio_common.irq.cond);
		return err;
	}

	err = beginthread(gpio_intrThread, GPIO_IRQ_THREAD_PRIO, gpio_common.irq.stack, sizeof(gpio_common.irq.stack), NULL);
	if (err < 0) {
		resourceDestroy(gpio_common.irq.req_list.lock);
		resourceDestroy(gpio_common.irq.lock);
		resourceDestroy(gpio_common.irq.cond);
		return err;
	}

	return 0;
}
#endif /* GPIO_SUPPORT_ASYNC_IRQ_WAIT */


int gpio_init(void)
{
	int i;
	static const void *addresses[] = { GPIO1_BASE, GPIO2_BASE, GPIO3_BASE,
		GPIO4_BASE, GPIO5_BASE, GPIO6_BASE, GPIO7_BASE, GPIO8_BASE, GPIO9_BASE,
		GPIO10_BASE, GPIO11_BASE, GPIO12_BASE, GPIO13_BASE };

#ifndef __CPU_IMXRT117X
	platformctl_t pctl;
	static const int clocks[] = { GPIO1_CLK, GPIO2_CLK, GPIO3_CLK,
		GPIO4_CLK, GPIO5_CLK, GPIO6_CLK, GPIO7_CLK, GPIO8_CLK, GPIO9_CLK,
		GPIO10_CLK, GPIO11_CLK, GPIO12_CLK, GPIO13_CLK };

	pctl.action = pctl_set;
	pctl.type = pctl_devclock;
	pctl.devclock.state = clk_state_run;

	for (i = 0; i < sizeof(clocks) / sizeof(clocks[0]); ++i) {
		if (clocks[i] < 0) {
			continue;
		}

		pctl.devclock.dev = clocks[i];
		if (platformctl(&pctl) < 0) {
			return -1;
		}
	}
#endif

	for (i = 0; i < sizeof(gpio_common.base) / sizeof(gpio_common.base[0]); ++i) {
		gpio_common.base[i] = (void *)addresses[i];
	}

	if (mutexCreate(&gpio_common.lock) < 0) {
		return -1;
	}

#if GPIO_SUPPORT_ASYNC_IRQ_WAIT
	if (gpio_initAsync() < 0) {
		resourceDestroy(gpio_common.lock);
		return -1;
	}
#endif /* GPIO_SUPPORT_ASYNC_IRQ_WAIT */
	return 0;
}
