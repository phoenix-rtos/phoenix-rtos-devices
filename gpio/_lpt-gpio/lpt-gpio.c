/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * PC Centronics GPIO driver
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
#include <vm/if.h>
#include <main/if.h>
#include <proc/if.h>

#if 1
#	define DBG_PRINT(...)	while(0)
#else
#	define DBG_PRINT(...)	main_printf(ATTR_DEBUG, __VA_ARGS__)
#endif

typedef struct {
	void *port;
	int irq;
	gpio_callback_t *callback;
	void *arg;
	u8 ctrl, data, stat;
} gpio_cfg_t;

#define PORT_DATA	(cfg->port + 0)
#define PORT_CTRL	(cfg->port + 1)
#define PORT_STAT	(cfg->port + 2)
#define CTRL_INPUT	(1 << 5)
#define CTRL_IRQ	(1 << 4)
#define STAT_ERROR	(1 << 3)
#define STAT_nIRQ	(1 << 2)

/**-----+-----------+-----------+---------
 *	Bit | Direction | Pin(DB25)	| Name
 * -----+-----------+-----------+---------
 *	0	|	in/out	|	2		| DATA0
 *	1	|	in/out	|	3		| DATA1
 *	2	|	in/out	|	4		| DATA2
 *	3	|	in/out	|	5		| DATA3
 *	4	|	in/out	|	6		| DATA4
 *	5	|	in/out	|	7		| DATA5
 *	6	|	in/out	|	8		| DATA6
 *	7	|	in/out	|	9		| DATA7
 *	8	|	in/out	|	1		| STROBE
 *	9	|	in/out	|	14		| AUTOLF
 *	10	|	in/out	|	16		| INIT
 *	11	|	in/out	|	13		| SELECT
 *	12	|	in		|	17		| SELIN
 *	13	|	in		|	12		| PAPEROUT
 *	14	|	in		|	10		| ACK
 *	15	|	in		|	11		| BUSY
 *	16	|	in		|	15		| ERROR
 * -----+-----------+-----------+---------
 */

#define ACK_PIN_NUM	14
#define IO_DATA		0x000ff
#define IO_BIDIR_N	0x00300
#define IO_BIDIR_P	0x00c00
#define IO_SELIN_N	0x01000
#define IO_PAPER	0x02000
#define IO_ACK		(1 << ACK_PIN_NUM)
#define IO_BUSY_N	0x08000
#define IO_ERROR	0x10000

#define set_data(val) hal_outb(PORT_DATA, (cfg->data = (val)))
#define set_ctrl(val) hal_outb(PORT_CTRL, (cfg->ctrl = (val)))

#define get_data() (cfg->data = hal_inb(PORT_CTRL))

static gpio_cfg_t *gpios[4];

void gpio_config(unsigned int bank, unsigned int line_mask, gpio_mode_t mode, gpio_callback_t callback, void * arg)
{
	gpio_cfg_t *cfg = gpios[bank];

	if (mode == OUTPUT) {
		if (line_mask & IO_DATA)
			hal_outb(PORT_CTRL, hal_inb(PORT_CTRL) & ~CTRL_INPUT);
	}
	else if (mode == INPUT) {
		if (line_mask & IO_DATA)
			hal_outb(PORT_CTRL, hal_inb(PORT_CTRL) | CTRL_INPUT);
		if (line_mask & IO_ACK)
			hal_outb(PORT_CTRL, hal_inb(PORT_CTRL) & ~CTRL_IRQ);
	}
	else if (mode == IN_EDGE_RISING){
		if (line_mask == IO_ACK) {
			cfg->callback = callback;
			cfg->arg = arg;
			hal_outb(PORT_CTRL, hal_inb(PORT_CTRL) | CTRL_IRQ);
		}
	} else {
		/*XXX other modes will be just ignored*/
	}
}


/** Clears pending IRQ on given port */
static inline void gpio_clearIRQ(gpio_cfg_t *cfg)
{
	u8 ctrl_reg;

	ctrl_reg = hal_inb(PORT_CTRL);
	hal_outb(PORT_CTRL, ctrl_reg & ~CTRL_IRQ);
	hal_outb(PORT_CTRL, ctrl_reg);
}


void gpio_setline(unsigned int bank, unsigned int line, bool state)
{
	gpio_cfg_t *cfg = gpios[bank];
	u32 mask = 1 << line;

	if (mask & (IO_DATA)) {
		if (state > 0)
			hal_outb(PORT_DATA, (cfg->data |= mask));
		else
			hal_outb(PORT_DATA, (cfg->data &= ~mask));
	}
	else if (mask & IO_BIDIR_P) {
		if (state > 0)
			hal_outb(PORT_CTRL, hal_inb(PORT_CTRL) | (mask >> 8));
		else
			hal_outb(PORT_CTRL, hal_inb(PORT_CTRL) & ~(mask >> 8));
	}
	else if (mask & IO_BIDIR_N) {
		if (state > 0)
			hal_outb(PORT_CTRL, hal_inb(PORT_CTRL) & ~(mask >> 8));
		else
			hal_outb(PORT_CTRL, hal_inb(PORT_CTRL) | (mask >> 8));
	}
}


bool gpio_getline(unsigned int bank, unsigned int line)
{
	gpio_cfg_t *cfg = gpios[bank];
	u32 mask = (1 << line);
	int state = -EINVAL;

	if (mask & (IO_DATA))
			state = (cfg->data = hal_inb(PORT_DATA) & mask);
	else if (mask & IO_BIDIR_P)
			state = hal_inb(PORT_CTRL) & (mask >> 8);
	else if (mask & IO_BIDIR_N)
			state = (hal_inb(PORT_CTRL) & (mask >> 8)) ^ (mask >> 8);
	else if (mask & (IO_PAPER | IO_ACK))
			state = hal_inb(PORT_STAT) & (mask >> 8);
	else if (mask & (IO_SELIN_N | IO_BUSY_N))
			state = (hal_inb(PORT_STAT) & (mask >> 8)) ^ (mask >> 8);
	else if (mask & IO_ERROR)
			state = hal_inb(PORT_STAT) & STAT_ERROR;

	return state;
}


void gpio_high(unsigned int bank, unsigned int line_mask)
{
	gpio_cfg_t *cfg = gpios[bank];

	if (line_mask & IO_DATA)
			hal_outb(PORT_DATA, (cfg->data |= line_mask & IO_DATA));
	if (line_mask & (IO_BIDIR_P | IO_BIDIR_N))
			hal_outb(PORT_CTRL, (hal_inb(PORT_CTRL) | ((line_mask & IO_BIDIR_P ) >> 8)) & ~((line_mask & IO_BIDIR_N ) >> 8));
}


void gpio_low(unsigned int bank, unsigned int line_mask)
{
	gpio_cfg_t *cfg = gpios[bank];

	if (line_mask & IO_DATA)
			hal_outb(PORT_DATA, (cfg->data &= ~(line_mask & IO_DATA)));
	if (line_mask & (IO_BIDIR_P | IO_BIDIR_N))
			hal_outb(PORT_CTRL, (hal_inb(PORT_CTRL) & ~((line_mask & IO_BIDIR_P ) >> 8)) | ((line_mask & IO_BIDIR_N ) >> 8));
}


static int gpio_isr(unsigned int irq, cpu_context_t *ctx, void *arg)
{
	gpio_cfg_t *cfg = (gpio_cfg_t *)arg;

	if (hal_inb(PORT_STAT) & STAT_nIRQ)
		return IHRES_IGNORE;

	if (cfg->callback != NULL)
		cfg->callback(ACK_PIN_NUM, cfg->arg);

	gpio_clearIRQ(cfg);
	return IHRES_HANDLED;
}


static int gpio_init_one(int portnum, int irq, gpio_cfg_t **gpio_cfg)
{
	gpio_cfg_t *cfg;

	if ((cfg = vm_kmalloc(sizeof(gpio_cfg_t))) == NULL) {
		main_printf(ATTR_ERROR, "Out of memory\n");
		return -ENOMEM;
	}

	main_printf(ATTR_INFO, "dev: (lpt-gpio) Detected Centronics interface on 0x%x irq=%d\n", portnum, irq);

	cfg->port = (void *)portnum;
	if (irq >=0) {
		cfg->irq = irq;
		cfg->callback = NULL;
		hal_interruptsSetHandler(irq, gpio_isr, cfg);
	}
	*gpio_cfg = cfg;

	return EOK;
}


int _gpio_init(void)
{
	int i = 0, result = 0;

	if (gpio_init_one(0x378, 7, gpios + i) != EOK) {
			main_printf(ATTR_ERROR, "dev: (lpt-gpio) Initialization failed\n");
			result++;
	}
	else
		i++;

	return result;
}
