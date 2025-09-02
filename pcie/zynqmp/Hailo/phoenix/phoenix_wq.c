#include "phoenix_wq.h"
#include "phoenix_log.h"


void init_waitqueue_head(wait_queue_head_t *wq){
        if(mutexCreate(&wq->mutex) < 0){
        pr_err("wait queue: Failed to create mutex\n");
        return;
    }

    if(condCreate(&wq->cond) < 0){
        pr_err("wait queue: Failed to create conditional\n");
    }
}

void wake_up_interruptible_all(wait_queue_head_t *wq)
{
    condBroadcast(wq->cond);
}
