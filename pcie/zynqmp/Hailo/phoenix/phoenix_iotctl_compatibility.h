#include <stdint.h>
#include <sys/types.h>

int copy_from_user(void *dest, void *src, size_t n);
int copy_to_user(void *dest, void *src, size_t n);