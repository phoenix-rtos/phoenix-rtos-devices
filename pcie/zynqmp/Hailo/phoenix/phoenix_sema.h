#ifndef PHOENIX_SEMA
#define PHOENIX_SEMA

#include <sys/types.h>
#include <sys/threads.h>

struct semaphore {
	semaphore_t sem;
};

#define __SEMAPHORE_INITIALIZER(name, val) \
	semaphoreCreate((semaphore_t *)&name.sem, val);


void sema_init(struct semaphore *s, int val);

void up(struct semaphore *s);

void down(struct semaphore *s);

int down_interruptible(struct semaphore *s);


#endif