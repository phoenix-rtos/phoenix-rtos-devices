#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <sys/threads.h>

#include <fcntl.h>

#include "event_queue.h"

#define min(a, b) ({ \
	__typeof__(a) _a = (a); \
	__typeof__(b) _b = (b); \
	_a > _b ? _b : _a; \
})


struct _event_queue_t {
	unsigned char *buf;
	unsigned int count;
	unsigned int r;
	unsigned int w;
	unsigned int sz;
	handle_t mutex;
	handle_t waitq;
};


int event_queue_init(event_queue_t **res, unsigned int sz)
{
	if (sz == 0)
		return EINVAL;

	int err;
	event_queue_t *eq = malloc(sizeof(event_queue_t));
	if (eq == NULL) {
		return -1;
	}

	memset(eq, 0, sizeof(event_queue_t));

	eq->buf = malloc(sizeof(unsigned char) * sz);
	if (eq->buf == NULL) {
		return -1;
	}

	eq->sz = sz;

	if ((err = mutexCreate(&eq->mutex)) < 0 || (err = condCreate(&eq->waitq)) < 0) {
		free(eq->buf);
		free(eq);
		return err;
	}

	*res = eq;

	return EOK;
}


void event_queue_put(event_queue_t *eq, unsigned char event, unsigned int min_size, unsigned int flags)
{
	mutexLock(eq->mutex);
	if (eq->count < eq->sz) {
		eq->buf[eq->w] = event;
		eq->w = (eq->w + 1) % eq->sz;
		eq->count++;
		if (eq->count >= min_size)
			condSignal(eq->waitq);
	}  // else: discard (or handle blocking variant... TODO)
	mutexUnlock(eq->mutex);
}


int event_queue_get(event_queue_t *eq, void *dest, unsigned int size, unsigned int flags)
{
	int err;

	mutexLock(eq->mutex);
	do {
		if (flags & O_NONBLOCK && eq->count < size) {
			err = -EWOULDBLOCK;
			break;
		}

		while (eq->count < size)
			condWait(eq->waitq, eq->mutex, 0);

		unsigned int bytes;

		if (eq->w < eq->r)
			memcpy(dest, eq->buf + eq->r, bytes = min(eq->count, eq->w - eq->r));
		else {
			memcpy(dest, eq->buf + eq->r, bytes = min(size, eq->sz - eq->r));

			if (bytes < size) {
				size -= bytes;
				memcpy(dest + bytes, eq->buf, min(size, eq->w));
				bytes += min(size, eq->w);
			}
		}

		eq->r = (eq->r + bytes) % eq->sz;
		eq->count -= bytes;

		err = bytes;
	} while (0);

	mutexUnlock(eq->mutex);

	return err;
}

unsigned int event_queue_count(event_queue_t *eq)
{
	unsigned int res;
	mutexLock(eq->mutex);
	res = eq->count;
	mutexUnlock(eq->mutex);
	return res;
}


void event_queue_destroy(event_queue_t *eq)
{
	resourceDestroy(eq->mutex);
	resourceDestroy(eq->waitq);
	free(eq->buf);
	free(eq);
}
