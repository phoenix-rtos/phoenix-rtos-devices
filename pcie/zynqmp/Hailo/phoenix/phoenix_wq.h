#ifndef PHOENIX_WQ
#define PHOENIX_WQ

#include <sys/types.h>
#include <sys/threads.h>
#include <errno.h>

typedef struct {
	handle_t mutex;
	handle_t cond;
} wait_queue_head_t;

#define sleep_on_wq_cond(cond, mutex) \
	mutexLock(mutex); \
	condWait(cond, mutex, 0); \
	mutexUnlock(mutex);


#define wait_event_interruptible(wq, expr) \
	({ \
		while (!expr) { \
			sleep_on_wq_cond(wq.cond, wq.mutex); \
		}; \
		expr ? 0 : -ERESTARTSYS; \
	})

void init_waitqueue_head(wait_queue_head_t *wq);
void wake_up_interruptible_all(wait_queue_head_t *wq);

#endif
