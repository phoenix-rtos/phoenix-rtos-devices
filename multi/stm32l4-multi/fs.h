#ifndef _FS_H_
#define _FS_H_

#include "config.h"

#include <sys/types.h>

#if BUILTIN_DUMMYFS
extern int fs_init(int asRoot, oid_t *rootOut);
#endif


#if BUILTIN_FLASH_SERVER
extern int flashsrv_main(int argc, char *argv[], int allowRoot);
#endif

#endif /* _FS_H_ */
