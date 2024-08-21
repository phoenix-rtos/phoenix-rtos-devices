#ifndef _EVENT_QUEUE_H_
#define _EVENT_QUEUE_H_

#include <sys/types.h>

typedef struct _event_queue_t event_queue_t;

int event_queue_init(event_queue_t **res, unsigned int sz);

void event_queue_put(event_queue_t *eq, unsigned char event, unsigned int min_size, unsigned int flags);

int event_queue_get(event_queue_t *eq, void *dest, unsigned int size, unsigned int flags);

unsigned int event_queue_count(event_queue_t *eq);

void event_queue_destroy(event_queue_t *eq);

#endif
