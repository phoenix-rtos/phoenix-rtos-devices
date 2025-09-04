#include "phoenix_iotctl_compatibility.h"

#include <string.h>

int copy_from_user(void *dest, void *src, size_t n)
{
	memcpy(dest, src, n);
    return 0;
}

int copy_to_user(void *dest, void *src, size_t n)
{
	memcpy(dest, src, n);
    return 0;
}