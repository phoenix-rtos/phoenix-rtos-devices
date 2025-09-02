#include "phoenix_spinlock.h"

void spin_lock_init(spinlock_t *s)
{
    if(mutexCreate(s) < 0){
        printf("spinlock: Failed to create mutex\n");
    }
}

void spin_lock_irqsave(spinlock_t *s, long dummy)
{
    (void)dummy;
    mutexLock((handle_t)*s);
}

void spin_unlock_irqrestore(spinlock_t *s, long dummy)
{
    (void)dummy;
    mutexUnlock((handle_t)*s);
}
