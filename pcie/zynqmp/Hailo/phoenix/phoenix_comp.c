#include "phoenix_comp.h"
#include <stdio.h>
#include <time.h>

struct curr_thread _current = { 0 };
struct curr_thread *current = &_current;

inline u32 ioread32(u8 *add){
    rmb();
    return *(u32*)add;
}

inline u16 ioread16(u8 *add){
    rmb();
    return *(u16*)add;
}

inline u8 ioread8(u8 *add){
    rmb();
    return *add;
}

inline void iowrite32(u32 val, u8 *add){
    *(u32*)add = val;
    wmb();
}

inline void iowrite16(u16 val, u8 *add){
    *(u16*)add = val;
    wmb();
}

inline void iowrite8(u8 val, u8 *add){
    *(u8*)add = val;
    wmb();
}

u64 ktime_get_ns(void)
{
	struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    return (u64)t.tv_nsec;
}

void *dev_get_drvdata(struct device *dev)
{
	return dev->dirver_data;
}

ktime_t ktime_get(void)
{
	return clock() / (CLOCKS_PER_SEC / 1000);
}

long long ktime_to_ms(ktime_t a)
{
	return a;
}

ktime_t ktime_sub(ktime_t a, ktime_t b){
    return a - b;
}