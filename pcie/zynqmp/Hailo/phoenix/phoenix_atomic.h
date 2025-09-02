#include <stdatomic.h>

typedef int atomic_t;
#define atomic_inc(ptr)        (void)atomic_fetch_add((volatile _Atomic int *)ptr, 1)
#define atomic_set(ptr, value) atomic_store((volatile _Atomic int *)ptr, value)
#define atomic_read(ptr)       atomic_load((volatile _Atomic int *)ptr)
