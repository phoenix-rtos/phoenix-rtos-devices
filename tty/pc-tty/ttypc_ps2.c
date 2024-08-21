#include <errno.h>
#include <unistd.h>

#include <sys/io.h>

#include "ttypc_vt.h"
#include "event_queue.h"

int ttypc_ps2_waitstatus(ttypc_t *ttypc, unsigned char bit, unsigned char state)
{
	unsigned int i;

	for (i = 0; i < 0xffff; i++) {
		if (!(inb((void *)((uintptr_t)ttypc->kbd + 4)) & ((1 << bit) ^ (state << bit))))
			return EOK;
		usleep(10);
	}

	return -ETIMEDOUT;
}


static int _ttypc_ps2_write_to_port(ttypc_t *ttypc, void *port, unsigned char byte)
{
	int err;

	/* Wait for input buffer to be empty */
	if ((err = ttypc_ps2_waitstatus(ttypc, 1, 0)) < 0)
		return err;

	outb(port, byte);

	return EOK;
}


int ttypc_ps2_write(ttypc_t *ttypc, unsigned char byte)
{
	return _ttypc_ps2_write_to_port(ttypc, (void *)ttypc->kbd, byte);
}


int ttypc_ps2_read(ttypc_t *ttypc, unsigned char byte)
{
	int err;

	/* Wait for output buffer not to be empty */
	if ((err = ttypc_ps2_waitstatus(ttypc, 0, 1)) < 0)
		return err;

	return inb((void *)((uintptr_t)ttypc->kbd));
}


int ttypc_ps2_write_ctrl(ttypc_t *ttypc, unsigned char byte)
{
	return _ttypc_ps2_write_to_port(ttypc, (void *)ttypc->kbd + 4, byte);
}


unsigned char ttypc_ps2_read_ctrl(ttypc_t *ttypc)
{
	return inb((void *)((uintptr_t)ttypc->kbd + 4));
}
