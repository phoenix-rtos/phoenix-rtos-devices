#ifndef _LIBMULTI_GENERIC_H_
#define _LIBMULTI_GENERIC_H_

#include <sys/msg.h>

#include "libmulti-driver.h"


typedef struct {
	int (*init)(device_ctx_t *ctx);
	void (*destroy)(device_ctx_t *ctx);

	int (*handleMsg)(msg_t *msg);
} generic_driver_t;


int libmulti_registerGeneric(generic_driver_t *device);


#endif /* _LIBMULTI_GPIO_H_ */
