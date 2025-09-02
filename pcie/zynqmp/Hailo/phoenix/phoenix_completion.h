#ifndef PHOENIX_COMPLETION
#define PHOENIX_COMPLETION

#include <sys/types.h>
#include <sys/threads.h>

struct completion{
    unsigned int done;
    /* This will make sure that everyone will wake up and if needed do not get back to sleeping 
    in other contexts */
    unsigned int do_not_resleep;
    handle_t mutex;
    handle_t cond;
};

void init_completion(struct completion *c);
void reinit_completion(struct completion *c);
void wait_for_completion(struct completion *c);
int wait_for_completion_timeout(struct completion *c, unsigned int msecs);
int wait_for_completion_interruptible_timeout(struct completion *c, unsigned int msecs);
void complete(struct completion *c);
void complete_all(struct completion *c);
unsigned long msecs_to_jiffies(const unsigned int n);

#endif