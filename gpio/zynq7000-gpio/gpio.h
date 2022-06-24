/*
 * Phoenix-RTOS
 *
 * Zynq-7000 GPIO controller
 *
 * Copyright 2022 Phoenix Systems
 * Author: Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _ZYNQ7000_GPIO_H_
#define _ZYNQ7000_GPIO_H_

#include <stdint.h>


#define GPIO_BANKS 2  /* Number of GPIO banks */
#define GPIO_PINS  32 /* Max number of pins per bank */


/* GPIO pins configuration */
extern const int gpioPins[GPIO_BANKS][GPIO_PINS];


/* Returns GPIO pin state (0 - low, 1 - high) */
extern int gpio_readPin(unsigned int bank, unsigned int pin, uint32_t *val);


/* Sets GPIO pin state (0 - low, 1 - high) */
extern int gpio_writePin(unsigned int bank, unsigned int pin, uint32_t val);


/* Returns GPIO pins state (0 - low, 1 - high) */
extern int gpio_readPort(unsigned int bank, uint32_t *val);


/* Sets GPIO pins state (0 - low, 1 - high) */
extern int gpio_writePort(unsigned int bank, uint32_t val, uint32_t mask);


/* Returns GPIO pins direction (0 - input, 1 - output) */
extern int gpio_readDir(unsigned int bank, uint32_t *dir);


/* Sets GPIO pins direction (0 - input, 1 - output) */
extern int gpio_writeDir(unsigned int bank, uint32_t dir, uint32_t mask);


/* Initializes GPIO controller */
extern int gpio_init(void);


#endif
