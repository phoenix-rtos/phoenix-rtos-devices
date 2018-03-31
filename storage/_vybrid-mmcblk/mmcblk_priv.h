#ifndef MMCBLK_PRIV_H
#define MMCBLK_PRIV_H

#include <hal/if.h>
#include <fs/if.h>
#include <dev/if.h>
#include <proc/if.h>
#include <main/if.h>
#include <vm/if.h>
#include <main/std.h>
#include <lib/list.h>
#include <dev/gpio/if.h>
#include <hal/MVF50GS10MK50.h>


#define SDHC_DEBUG 0


#undef LOG
#if SDHC_DEBUG && 1
#define LOG(msg, ...) main_printf(ATTR_FAILURE, "%s(%d): " msg "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)
#else
#define LOG(msg, ...)
#endif

#endif
