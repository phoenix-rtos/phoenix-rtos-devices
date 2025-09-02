#include "phoenix_sema.h"

void sema_init(struct semaphore *s, int val)
{
    semaphoreCreate(&s->sem, val);
}

void up(struct semaphore *s)
{
	semaphoreUp(&s->sem);
}


void down(struct semaphore *s)
{
	semaphoreDown(&s->sem, 0);
}

int down_interruptible(struct semaphore *s)
{
	semaphoreDown(&s->sem, 0);
	return 0;
}
