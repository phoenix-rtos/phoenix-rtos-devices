#include <errno.h>
#include <paths.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <libklog.h>

#include <sys/debug.h>
#include <sys/file.h>
#include <sys/msg.h>
#include <sys/interrupt.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/threads.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <posix/utils.h>

typedef struct
{
	union {
		struct {
			uint32_t head; /* Head contains CAN packet mode and IDs */
			uint32_t stat;

			uint8_t payload[8]; /* Payload */
		} frame;

		uint8_t payload[16];
	};
} grlibCan_msg_t;

typedef struct {
	enum { can_config = 0,
		can_getStatus,
		can_writeSync,
		can_readSync,
		can_writeAsync,
		can_readAsync } type;
} grlibCan_devCtrl_t;

int main(int argc, char **argv)
{
	char *canDevice = "/dev/can0";
	oid_t canDev;

	while (lookup(canDevice, NULL, &canDev) < 0) {
		printf("Device does not exist\n");
		return EXIT_FAILURE;
	}

	/* Opening CAN device */
	msg_t msg = { 0 };
	msg.type = mtOpen;
	msg.oid.id = 0;

	printf("Sending message to open CAN device\n");

	if (msgSend(canDev.port, &msg) == 0) {
		if (msg.o.err < 0) {
			printf("Failed to acquire device\n");
			return EXIT_FAILURE;
		}
	}

	printf("Managed to open CAN device\n");

	grlibCan_msg_t frames[10];

	frames->frame.head = (1 << 18);
	frames->frame.stat = 0x10u << 24u;
	frames->frame.payload[0] = 0x0F;

	for (int i = 1; i < 10; i++) {
		memcpy((void *)&frames[i], (void *)frames, sizeof(grlibCan_msg_t));
		frames[i].frame.payload[0] = 0x0F + i;
	}

	msg.type = mtDevCtl;
	msg.i.size = 10;
	msg.i.data = frames;

	grlibCan_devCtrl_t *p = (grlibCan_devCtrl_t *)msg.i.raw;
	p->type = can_writeSync;

	if (msgSend(canDev.port, &msg) < 0) {
		printf("Failed to send message");
	}

	printf("Device send: %d\n", msg.o.err);

	grlibCan_msg_t ret[10];
	msg.type = mtDevCtl;
	msg.o.size = 10;
	msg.o.data = (void *)ret;

	p = (grlibCan_devCtrl_t *)msg.i.raw;
	p->type = can_readAsync;

	usleep(10000);

	if (msgSend(canDev.port, &msg) < 0) {
		printf("Failed to send message");
	}

	printf("Received from device %d\n", msg.o.err);

	return EXIT_SUCCESS;
}
