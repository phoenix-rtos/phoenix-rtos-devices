#ifndef SAFE_PRINTF
#define SAFE_PRINTF

#ifdef __cplusplus
extern "C" {
#endif

#include <sys/threads.h>

extern handle_t stdio_lock;

void safe_printf(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif