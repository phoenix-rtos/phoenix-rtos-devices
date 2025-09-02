#include <sys/types.h>
#include <sys/threads.h>

typedef handle_t spinlock_t;

void spin_lock_init(spinlock_t *s);
void spin_lock_irqsave(spinlock_t *s, long dummy);
void spin_unlock_irqrestore(spinlock_t *s, long dummy);