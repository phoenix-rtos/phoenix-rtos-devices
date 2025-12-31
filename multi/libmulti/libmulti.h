#ifndef _LIBMULTI_H_
#define _LIBMULTI_H_

#include <sys/types.h>


typedef struct {
	id_t baseId;
	size_t nperipherals;
	const char *devPathPrefix;
} device_ctx_t;


#endif
