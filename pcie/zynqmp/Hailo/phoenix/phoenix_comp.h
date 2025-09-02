#ifndef PHOENIX_COMPATIBILITY
#define PHOENIX_COMPATIBILITY

#include "../../core/pci_utils.h"
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>

#define GFP_KERNEL  0
#define ERESTARTSYS 1

struct curr_thread {
	int tgid;
};

extern struct curr_thread _current;
extern struct curr_thread *current;

typedef uint64_t u64;
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;

typedef uint64_t dma_addr_t;
typedef uint64_t gfp_t;

typedef int s32;

u32 ioread32(u8 *add);
u16 ioread16(u8 *add);
u8 ioread8(u8 *add);

void iowrite32(u32 val, u8 *add);
void iowrite16(u16 val, u8 *add);
void iowrite8(u8 val, u8 *add);


u64 ktime_get_ns(void);
void *dev_get_drvdata(struct device *dev);

typedef clock_t ktime_t;

ktime_t ktime_get(void);
long long ktime_to_ms(ktime_t a);
ktime_t ktime_sub(ktime_t a, ktime_t b);

#define BIT_MASK(nr)     (1u << nr)
#define BIT_WORD(nr)     (nr / (sizeof(unsigned long) * 8))
#define IS_ALIGNED(x, a) (((x) & ((typeof(x))(a) - 1)) == 0)

#define BUG_ON(condition) \
	if (condition) { \
		printf("hailo: %s:%s failed\n", __FILE__, __func__); \
		exit(1); \
	}

#define min(x, y) ({				\
	typeof(x) _min1 = (x);			\
	typeof(y) _min2 = (y);			\
	(void) (&_min1 == &_min2);		\
	_min1 < _min2 ? _min1 : _min2; })

#define msleep(x) usleep(1000 * x)

#define unlikely(expr) expr


#if __STDC_VERSION__ >= 201112L
#define static_assert _Static_assert
#else
#define static_assert ()
#endif

#define BUILD_BUG_ON_MSG(cond, msg) static_assert(!(cond), msg)

/* This attibute is used in Linux for static analysis (Sparse analysis tool) */
/* It enforces no derefrence of pointer (variables marked with this attribute should be pased to ioread/iowrite) */
#define __iomem
#define __user

/* This is not a full-proof implementation of this macro */
/* I assume that it is used properly in Hailo driver */
#define ARRAY_SIZE(x) (sizeof(x) / sizeof(*x))

#define KERNEL_VERSION(a, b, c) 0

#define LINUX_VERSION_CODE 1

#define ERR_PTR(err) ((void *)((long)(err)))
#define PTR_ERR(ptr) ((long)(ptr))
#define IS_ERR(ptr)  ((unsigned long)(ptr) > (unsigned long)(-1000))

#endif
