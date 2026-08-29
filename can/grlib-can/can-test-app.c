#include <errno.h>
#include <paths.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

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

#include "grlib-can-if.h"

int main(int argc, char **argv)
{
	char *canDevice = "/dev/can0";
	oid_t canDev;

	if (lookup(canDevice, NULL, &canDev) < 0) {
		printf("Device does not exist\n");
		return EXIT_FAILURE;
	}

	printf("Successfully checked CAN device oid\n");

	printf("Can device open: %d\n", grlibCan_open(canDev));

	uint32_t status;
	grlibCan_getStatus(canDev, &status);
	printf("Can device status: %x\n", status);

	grlibCan_config_t config;

	grlibCan_getConfig(canDev, &config);
	printf("Can device baud rate - nom: %d, data: %d\n", config.nomBdRate, config.dataBdRate);

	grlibCan_msg_t frames[10];

	frames->frame.head = (1 << 18);
	frames->frame.stat = 0x10u << 24;
	frames->frame.payload[0] = 0x0F;

	for (int i = 1; i < 10; i++) {
		memcpy((void *)&frames[i], (void *)frames, sizeof(grlibCan_msg_t));
		frames[i].frame.payload[0] = 0x0F + i;
	}

	int ret = grlibCan_Send(canDev, frames, 10, true);
	printf("Can device managed to send %d frames : block\n", ret);

	grlibCan_msg_t buf[10];

	ret = grlibCan_Recv(canDev, buf, 10, true);
	printf("Can device managed to receive %d frames : block\n", ret);

	ret = grlibCan_Send(canDev, frames, 10, false);
	printf("Can device managed to send %d frames : non-block\n", ret);

	ret = grlibCan_Recv(canDev, buf, 10, false);
	printf("Can device managed to receive %d frames : non-block\n", ret);

	printf("Sending frame of extended length\n");
	frames->frame.stat = 0xF0 << 24 | (1 << 2);
	ret = grlibCan_Send(canDev, (void *)frames, 5, true);
	printf("Ret: %d\n", ret);

	printf("Trying to read in SYNC mode to buffer which is to small\n");
	ret = grlibCan_Recv(canDev, buf, 1, true);
	printf("Ret: %d\n", ret);

	printf("Trying to read in SYNC mode to buffer of appropriate size\n");
	ret = grlibCan_Recv(canDev, buf, 5, true);
	printf("Ret: %d\n", ret);

	printf("Sending frame of extended length\n");
	frames->frame.stat = 0xF0 << 24 | (1 << 2);
	ret = grlibCan_Send(canDev, (void *)frames, 5, true);
	printf("Ret: %d\n", ret);

	printf("Trying to read in ASYNC mode to buffer which is to small\n");
	ret = grlibCan_Recv(canDev, buf, 1, false);
	printf("Ret: %d\n", ret);

	printf("Trying to read in ASYNC mode to buffer of appropriate size\n");
	ret = grlibCan_Recv(canDev, buf, 5, false);
	printf("Ret: %d\n", ret);

	printf("Closing device\n");
	grlibCan_close(canDev);

	ret = grlibCan_Send(canDev, frames, 10, true);
	printf("Trying to send with device closed: %d\n", ret);

	return EXIT_SUCCESS;
}
