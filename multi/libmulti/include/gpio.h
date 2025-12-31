#ifndef _LIB_GPIO_H_
#define _LIB_GPIO_H_

#include <stdint.h>

// #define GPIO_IRQ_


int gpio_getPort(uint32_t port, uint32_t *val);


int gpio_setPort(uint32_t port, uint32_t mask, uint32_t val);


int gpio_getPin(uint32_t port, uint32_t pin, uint32_t *val);


int gpio_setPin(uint32_t port, uint32_t pin, uint32_t val);


int gpio_getDir(uint32_t port, uint32_t *dir);


int gpio_setDir(uint32_t port, uint32_t pin, uint32_t dir);


int gpio_setIrqConfig(uint32_t port, uint32_t pin, uint32_t flags);


int gpio_getIrqConfig(uint32_t port, uint32_t pin, uint32_t *flags);


/* should error-out if IRQ is not configured for this port:pin. */
int gpio_waitIrq(uint32_t port, uint32_t pin);


#endif /* _LIB_GPIO_H_ */
