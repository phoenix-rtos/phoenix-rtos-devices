#ifndef _LIBMULTI_GPIO_H_
#define _LIBMULTI_GPIO_H_

#include "libmulti-driver.h"


int gpio_init(device_ctx_t *ctx);


void gpio_destroy(device_ctx_t *ctx);


int libmulti_gpioGetPort(uint32_t port, uint32_t *val);


int libmulti_gpioSetPort(uint32_t port, uint32_t mask, uint32_t val);


int libmulti_gpioGetDir(uint32_t port, uint32_t *val);


int libmulti_gpioSetDir(uint32_t port, uint32_t mask, uint32_t val);


#endif /* _LIBMULTI_GPIO_H_ */
