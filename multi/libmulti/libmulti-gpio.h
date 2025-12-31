#ifndef _LIBMULTI_GPIO_H_
#define _LIBMULTI_GPIO_H_

#include <sys/types.h>
#include "libmulti.h"


int gpio_init(device_ctx_t *ctx);


void gpio_destroy(device_ctx_t *ctx);


#endif /* _LIBMULTI_GPIO_H_ */
