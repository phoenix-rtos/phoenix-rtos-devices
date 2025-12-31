#ifndef _LIB_GPIO_H_
#define _LIB_GPIO_H_

#include <stdint.h>


int gpio_getPort(uint32_t port, uint32_t *val);


int gpio_setPort(uint32_t port, uint32_t mask, uint32_t val);


int gpio_getPin(uint32_t port, uint32_t pin, uint32_t *val);


int gpio_setPin(uint32_t port, uint32_t pin, uint32_t val);


int gpio_getDir(uint32_t port, uint32_t *dir);


int gpio_setDir(uint32_t port, uint32_t pin, uint32_t dir);


#endif /* _LIB_GPIO_H_ */
