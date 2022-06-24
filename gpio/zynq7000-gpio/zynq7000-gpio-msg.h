/*
 * Phoenix-RTOS
 *
 * Zynq-7000 GPIO message interface
 *
 * Copyright 2022 Phoenix Systems
 * Author: Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _ZYNQ7000_GPIO_MSG_H_
#define _ZYNQ7000_GPIO_MSG_H_

#include <stdint.h>

#include <sys/msg.h>
#include <sys/types.h>


enum {
	gpio_devctl_read_pin = 0, /* input: - */
	gpio_devctl_write_pin,    /* input: val */
	gpio_devctl_read_port,    /* input: - */
	gpio_devctl_write_port,   /* input: val, mask */
	gpio_devctl_read_dir,     /* input: - */
	gpio_devctl_write_dir,    /* input: val, mask */
};


typedef union {
	struct {
		unsigned int type; /* Devctl type */
		oid_t oid;         /* Devctl oid (pin/port/dir) */
		uint32_t val;      /* Pin state(s) */
		uint32_t mask;     /* Pin mask */
	} i;

	struct {
		uint32_t val; /* Returned value */
		int err;      /* Error */
	} o;
} __attribute__((packed)) gpio_devctl_t;


/* Returns GPIO pin state (0 - low, 1 - high) */
extern int gpiomsg_readPin(oid_t *pin, uint32_t *val);


/* Sets GPIO pin state (0 - low, 1 - high) */
extern int gpiomsg_writePin(oid_t *pin, uint32_t val);


/* Returns GPIO pins state (0 - low, 1 - high) */
extern int gpiomsg_readPort(oid_t *port, uint32_t *val);


/* Sets GPIO pins state (0 - low, 1 - high) */
extern int gpiomsg_writePort(oid_t *port, uint32_t val, uint32_t mask);


/* Returns GPIO pins direction (0 - input, 1 - output) */
extern int gpiomsg_readDir(oid_t *dir, uint32_t *val);


/* Sets GPIO pins direction (0 - input, 1 - output) */
extern int gpiomsg_writeDir(oid_t *dir, uint32_t val, uint32_t mask);


#endif
