/* This file contains implementation of completion mechanism using mutex, and conditional */
#include <unistd.h>
#include <stdlib.h>
#include <sys/threads.h>

#include "phoenix_completion.h"

void init_completion(struct completion *c)
{
	c->done = 0;
	c->do_not_resleep = 0;

	if (mutexCreate(&c->mutex) < 0) {
		printf("completion: Failed to create mutex\n");
		return;
	}

	if (condCreate(&c->cond) < 0) {
		printf("completion: Failed to create conditional\n");
	}
}


void reinit_completion(struct completion *c)
{
	mutexLock(c->mutex);
	c->done = 0;
	c->do_not_resleep = 0;
	mutexUnlock(c->mutex);
}


void wait_for_completion(struct completion *c)
{
	mutexLock(c->mutex);
	while (0 == c->done && c->do_not_resleep != 1) {
		condWait(c->cond, c->mutex, 0);
	}
	mutexUnlock(c->mutex);
}

/* If completion was signaled before timeout return number of jiffies left -> in this case hard coded 1 */
/* If timeout has been reach and completion was not signalled return 0 */
int wait_for_completion_timeout(struct completion *c, unsigned int msecs)
{
	mutexLock(c->mutex);
	while (c->done == 0 && c->do_not_resleep != 1) {
		int ret = condWait(c->cond, c->mutex, 1000 * msecs);
		if (ret < 0 && c->done == 0) {
			mutexUnlock(c->mutex);
			return 0;
		}
	}
	mutexUnlock(c->mutex);
	return 1;
}

int wait_for_completion_interruptible_timeout(struct completion *c, unsigned int msecs)
{
	return wait_for_completion_timeout(c, msecs);
}

void complete_all(struct completion *c)
{
	mutexLock(c->mutex);
	c->done = 1;
	c->do_not_resleep = 1;
	mutexUnlock(c->mutex);
	condBroadcast(c->cond);
}

void complete(struct completion *c)
{
	mutexLock(c->mutex);
	c->done = 1;
	mutexUnlock(c->mutex);
	condBroadcast(c->cond);
}

unsigned long msecs_to_jiffies(const unsigned int n)
{
	return n;
}
