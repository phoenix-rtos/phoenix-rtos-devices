#ifndef _LIBTTY_FIFO_H
#define _LIBTTY_FIFO_H

typedef struct fifo_s fifo_t;

struct fifo_s {
	unsigned int head;
	unsigned int tail;
	unsigned int size_mask;
	uint8_t data[];
};


/* NOTE: size must be a power of 2 ! */
static inline void fifo_init(fifo_t *f, unsigned int size)
{
	f->head = 0;
	f->tail = 0;
	f->size_mask = size - 1;
}

static inline void fifo_remove_all(fifo_t *f)
{
	f->head = f->tail = 0;
}

static inline void fifo_remove_all_but_one(fifo_t *f)
{
	if (f->head != f->tail)
		f->head = (f->tail + 1) & f->size_mask;
}

static inline unsigned int fifo_is_full(fifo_t *f)
{
	return ((f->head + 1) & f->size_mask) == f->tail;
}


static inline unsigned int fifo_is_empty(fifo_t *f)
{
	return (f->head == f->tail);
}

static inline unsigned int fifo_count(fifo_t *f)
{
	return (f->head - f->tail) & f->size_mask;
}

static inline unsigned int fifo_freespace(fifo_t *f)
{
	return (f->tail - f->head - 1) & f->size_mask;
}


static inline void fifo_push(fifo_t *f, uint8_t byte)
{
	f->data[f->head] = byte;
	f->head = (f->head + 1) & f->size_mask;
}


static inline uint8_t fifo_pop_back(fifo_t *f)
{
	uint8_t ret = f->data[f->tail];
	f->tail = (f->tail + 1) & f->size_mask;

	return ret;
}


static inline uint8_t fifo_pop_front(fifo_t *f)
{
	unsigned int new_head = (f->head - 1) & f->size_mask;
	uint8_t ret = f->data[new_head];
	f->head = new_head;

	return ret;
}

static inline uint8_t fifo_peek_front(fifo_t *f)
{
	unsigned int new_head = (f->head - 1) & f->size_mask;
	uint8_t ret = f->data[new_head];
	return ret;
}

static inline int fifo_has_char(fifo_t *f, char byte)
{
	unsigned int tail = f->tail;

	if (fifo_is_empty(f))
		return 0;

	while (tail != f->head) {
		if (f->data[tail] == (uint8_t)byte)
			return 1;

		tail = (tail + 1) & f->size_mask;
	}

	return 0;
}

#endif /* _LIBTTY_FIFO_H */
