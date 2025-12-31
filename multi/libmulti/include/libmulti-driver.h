#ifndef _LIBMULTI_DRIVER_H_
#define _LIBMULTI_DRIVER_H_

#include <sys/types.h>


typedef struct {
	size_t nperipherals;
	const char *devPathPrefix;

	id_t baseId;
} device_ctx_t;


#endif /* _LIBMULTI_DRIVER_H_ */
