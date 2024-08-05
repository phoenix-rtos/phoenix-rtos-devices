/*
 * Phoenix-RTOS
 *
 * Simple circular buffer for enqueueing PS/2 events
 *
 * Copyright 2024 Phoenix Systems
 * Author: Adam Greloch
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/threads.h>
#include <sys/minmax.h>

#include "event_queue.h"


int event_queue_init(event_queue_t *eq)
{
	int err;

	if (eq == NULL) {
		return -EINVAL;
	}

	(void)memset(eq, 0, sizeof(event_queue_t));

	err = mutexCreate(&eq->mutex);
	if (err < 0) {
		return err;
	}

	err = condCreate(&eq->waitq);
	if (err < 0) {
		resourceDestroy(eq->mutex);
		return err;
	}

	return EOK;
}


int event_queue_put(event_queue_t *eq, unsigned char event, unsigned int notify_cnt)
{
	if (eq == NULL) {
		return -EINVAL;
	}

	int bytes = 0;

	mutexLock(eq->mutex);
	if (eq->cnt < TTYPC_EQBUF_SIZE) {
		eq->buf[eq->w] = event;
		eq->w = (eq->w + 1u) % TTYPC_EQBUF_SIZE;
		eq->cnt++;
		if (eq->cnt >= notify_cnt) {
			condSignal(eq->waitq);
		}
		bytes = 1;
	}
	mutexUnlock(eq->mutex);

	return bytes;
}


static int _event_queue_get(event_queue_t *eq, void *dest, unsigned int size, unsigned int minsize, unsigned int flags)
{
	int err;
	unsigned int bytes;

	if ((flags & O_NONBLOCK) != 0 && eq->cnt < minsize) {
		return -EWOULDBLOCK;
	}

	while (eq->cnt < minsize) {
		condWait(eq->waitq, eq->mutex, 0);
	}

	if (size > eq->cnt) {
		size = eq->cnt;
	}

	if (eq->w > eq->r) {
		bytes = size;
	}
	else {
		bytes = min(size, TTYPC_EQBUF_SIZE - eq->r);
	}

	(void)memcpy(dest, eq->buf + eq->r, bytes);

	if (bytes < size) {
		size -= bytes;
		(void)memcpy((char *)dest + bytes, eq->buf, size);
		bytes += size;
	}

	eq->r = (eq->r + bytes) % TTYPC_EQBUF_SIZE;
	eq->cnt -= bytes;

	err = bytes;

	return err;
}


int event_queue_get(event_queue_t *eq, char *dest, unsigned int size, unsigned int minsize, unsigned int flags)
{
	int err;

	if (eq == NULL || dest == NULL || size < minsize) {
		return -EINVAL;
	}

	if (size == 0u) {
		return 0;
	}

	mutexLock(eq->mutex);
	err = _event_queue_get(eq, dest, size, minsize, flags);
	mutexUnlock(eq->mutex);

	return err;
}


int event_queue_get_mouse(event_queue_t *meq, char *dest, unsigned int size, unsigned int flags)
{
	int res, total = 0;

	if (meq == NULL || dest == NULL) {
		return -EINVAL;
	}

	if (size < 3) {
		/*
		 * Pretend there's nothing to read. We don't want someone to read partial mouse
		 * packets - this would mess up byte ordering and confuse recipients.
		 */
		return 0;
	}

	mutexLock(meq->mutex);

	if (size > meq->cnt) {
		size = meq->cnt;
	}

	/* read only multiples of 3 */
	size = size - (size % 3);

	while (total < size) {
		/* find first package byte */
		do {
			/* 3rd bit should be set in the first byte of mouse package */
			res = _event_queue_get(meq, dest + total, 1, 1, flags);
			if (res < 1) {
				mutexUnlock(meq->mutex);
				return res;
			}
		} while ((dest[0] & (1 << 3)) == 0);

		/* found the first byte, get the other two */
		res = _event_queue_get(meq, dest + total + 1, 2, 2, flags);
		if (res != 2) {
			break;
		}

		total += 3;
	};

	mutexUnlock(meq->mutex);

	return total;
}


int event_queue_count(const event_queue_t *eq)
{
	int res;

	if (eq == NULL) {
		return -EINVAL;
	}

	mutexLock(eq->mutex);
	res = eq->cnt;
	mutexUnlock(eq->mutex);

	return res;
}


void event_queue_destroy(const event_queue_t *eq)
{
	if (eq == NULL) {
		return;
	}

	resourceDestroy(eq->waitq);
	resourceDestroy(eq->mutex);
}
