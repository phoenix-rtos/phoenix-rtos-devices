/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * eSi-GPIO device driver
 *
 * Copyright 2013 Phoenix Systems
 *
 * Author: Jacek Popko
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <hal/if.h>
#include "if.h"
#include <esirisc/device.h>
#include <esirisc/gpio.h>
#include <vm/if.h>
#include <main/if.h>
#include <proc/if.h>

#if 1
#	define DBG_PRINT(...)	while(0)
#else
#	define DBG_PRINT(...)	main_printf(ATTR_DEBUG, __VA_ARGS__)
#endif

typedef struct {
	esi_gpio_t *regs;
	int lines;
	int irq;
	struct {
		gpio_callback_t *callback;
		void *arg;
	} handler[];
} gpio_cfg_t;


static gpio_cfg_t *gpios[4];


void gpio_config(unsigned int bank, unsigned int line_mask, gpio_mode_t mode, gpio_callback_t callback, void * arg)
{
	gpio_cfg_t *cfg = gpios[bank];

	if (cfg->irq >=0)
	{
		int line;
		
		if (mode == INPUT || mode == OUTPUT)
			callback = NULL;
		
		for (line = 0; line < cfg->lines; line++)
			if (line_mask & (1 << line)) {
				cfg->handler[line].arg = arg;
				cfg->handler[line].callback = callback;
			} 
	}
	
	switch(mode) {
		case OUTPUT: 
			cfg->regs->data_direction |= line_mask;
			break;
			
		case INPUT:
			cfg->regs->interrupt_enable &= ~line_mask;
			cfg->regs->data_direction &= ~line_mask;
			break;
			
		case IN_EDGE_FALLING:
			cfg->regs->trigger |= line_mask;	/* 1: edge */
			cfg->regs->sense &= ~line_mask;	/* 0: falling */
			cfg->regs->interrupt_enable |= line_mask;
			cfg->regs->data_direction &= ~line_mask;
			break;
			
		case IN_EDGE_RISING:
			cfg->regs->trigger |= line_mask;	/* 1: edge */
			cfg->regs->sense |= line_mask;	/* 1: rising */
			cfg->regs->interrupt_enable |= line_mask;
			cfg->regs->data_direction &= ~line_mask;
			break;
			
		case IN_LEVEL_LOW:
			cfg->regs->trigger &= ~line_mask;	/* 0: level */
			cfg->regs->sense &= ~line_mask;	/* 0: low */
			cfg->regs->interrupt_enable |= line_mask;
			cfg->regs->data_direction &= ~line_mask;
			break;
			
		case IN_LEVEL_HIGH:
			cfg->regs->trigger &= ~line_mask;	/* 0: level */
			cfg->regs->sense |= line_mask;	/* 1: high */
			cfg->regs->interrupt_enable |= line_mask;
			cfg->regs->data_direction &= ~line_mask;
			break;
			
		default:
			assert(0);
			break;
	}
}


void gpio_setline(unsigned int bank, unsigned int line, bool state)
{
	gpio_cfg_t *cfg = gpios[bank];
	
	if (state > 0)
		cfg->regs->data_out |= 1 << line;
	else
		cfg->regs->data_out &= ~(1 << line);
}


bool gpio_getline(unsigned int bank, unsigned int line)
{
	gpio_cfg_t *cfg = gpios[bank];

	return (cfg->regs->data_in >> line) & 1;
}


void gpio_high(unsigned int bank, unsigned int line_mask)
{
	gpios[bank]->regs->data_out |= line_mask;
}


void gpio_low(unsigned int bank, unsigned int line_mask)
{
	gpios[bank]->regs->data_out &= ~line_mask;
}


static int gpio_isr(unsigned int irq, cpu_context_t *ctx, void *arg)
{
	gpio_cfg_t *gpio_cfg = (gpio_cfg_t *)arg;
	unsigned p, a = 1, i;
	
	p = gpio_cfg->regs->pending;
	for (i = 0; p > 0; i++, p >>= 1, a <<= 1)
		if (p & 1) {
			if (gpio_cfg->handler[i].callback != NULL)
				gpio_cfg->handler[i].callback(i, gpio_cfg->handler[i].arg);
			
			gpio_cfg->regs->ack = a;
		}
	
	//proc_schedule(ctx);
	return 1;
}


static int gpio_init_one(esi_device_info_t *device, gpio_cfg_t **gpio_cfg)
{
	int status, i;
	gpio_cfg_t *cfg;

	if (device->device_id != ESI_DID_ENSILICA_APB_GPIO)
		return ERR_DEV_DETECT;

	if ((cfg = vm_kmalloc(sizeof(gpio_cfg_t) + device->irq >= 0 ? device->config * 2 * sizeof(void *) : 0 )) == NULL) {
		main_printf(ATTR_ERROR, "Out of memory\n");
		return -ENOMEM;
	}

	cfg->lines = device->config;
	cfg->irq = device->irq;
	
	if (cfg->irq >= 0)
		for (i = 0; i < cfg->lines; i++)
			cfg->handler[i].callback = NULL;
	
	status = vm_iomap((addr_t)device->base_address, device->size, PGHD_KERNEL_RW, (void **)&cfg->regs); /*XXX PGHD_DEV_RW*/
	assert(status == EOK);

	if (device->irq >= 0)
		hal_interruptsSetHandler(device->irq, gpio_isr, cfg);

	*gpio_cfg = cfg;
	return EOK;
}


int _gpio_init(void)
{
	esi_device_info_t *device;
	int i = 0, result = 0;

	/* 0 in size field indicates end of device list. */
	for (device = esi_device_get_list(); (device != NULL) && (device->size != 0); device++) {
		if (device->device_id == ESI_DID_ENSILICA_APB_GPIO) {
			main_printf(ATTR_INFO, "dev: eSi-GPIO rev=%d, base=0x%p, %d lines",
				device->revision, device->base_address, device->config);
			if (device->irq >= 0)
				main_printf(ATTR_INFO, " irq=%d\n", device->irq);
			else
				main_printf(ATTR_INFO, "\n");

			assert(i < sizeof(gpios)/sizeof(gpio_cfg_t *));
			
			if (gpio_init_one(device, gpios + i) != EOK) {
				main_printf(ATTR_ERROR, "dev: eSi-GPIO initialization failed\n");
				result++;
			}
			else
				i++;
		}
	}
	return result;
}
