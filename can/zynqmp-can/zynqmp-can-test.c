/*
 * Phoenix-RTOS
 *
 * ZynqMP CAN driver test application
 *
 * Copyright 2025 Phoenix Systems
 * Author: Dariusz Sabala
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <termios.h>
#include <stddef.h>
#include <sys/types.h>
#include <sys/threads.h>
#include <sys/msg.h>
#include <signal.h>
#include <unistd.h>
#include <sys/time.h>

#include <zynqmp-can-if.h>

/* Structure to store statistics for received frames */
typedef struct {
	uint32_t receivedFramesCount; /**< Number of received frames */
	zynqmp_canFrame lastFrame;    /**< Last received frame */
} frameStats;

/* Application data */
static struct {
	uint8_t stack[_PAGE_SIZE] __attribute__((aligned(16)));
	int run;                                   /**< flag to control the main loop */
	handle_t mutex_stats_access;               /**< mutex for accessing frame statistics */
	frameStats frame_stats[ZYNQMP_CAN_ID_MAX]; /**< Array to store statistics for each CAN ID */
} common;

/**
 * Waits for a command input for a specified number of seconds.
 */
static int zynqmpCanTest_waitCmd(unsigned int secs)
{
	struct timeval left;
	fd_set fds;

	left.tv_sec = (time_t)secs;
	left.tv_usec = 0;
	fflush(stdout);
	FD_ZERO(&fds);
	FD_SET(STDIN_FILENO, &fds);
	select(STDIN_FILENO + 1, &fds, NULL, NULL, &left);

	if (FD_ISSET(STDIN_FILENO, &fds)) {
		return getchar();
	}

	return 0;
}

/**
 * Decodes and prints the contents of a CAN frame based on its ID.
 */
static void zynqmpCanTest_decodeAndPrintFrame(zynqmp_canFrame *frame)
{
	switch (frame->id) {
		/* Placeholder for decoding logic */
		default: {
			/* Unknown frame content */
			printf("\n");
		} break;
	}
}

/**
 * Thread function to print CAN frame statistics periodically.
 * Displays the received frame count and content for each CAN ID.
 */
static void zynqmpCanTest_statsPrintThread(void *arg)
{
	unsigned int delay = 1;

	/* Clear terminal */
	printf("\033[2J");

	/* Disable cursor */
	printf("\033[?25l");

	while (common.run) {
		/* Move cursor to the beginning */
		printf("\033[f");
		printf("\033[K");
		printf("\033[2J");

		mutexLock(common.mutex_stats_access);

		printf("   ID | Count | Frame content \n");
		for (uint32_t i = 0; i < ZYNQMP_CAN_ID_MAX; i++) {
			if (common.frame_stats[i].receivedFramesCount != 0) {
				printf("0x%3X |  %4i | ", i, common.frame_stats[i].receivedFramesCount);
				zynqmpCanTest_decodeAndPrintFrame(&common.frame_stats[i].lastFrame);
			}
			common.frame_stats[i].receivedFramesCount = 0;
		}

		mutexUnlock(common.mutex_stats_access);

		int cmd = zynqmpCanTest_waitCmd(delay);
		if (cmd == 'q') {
			common.run = 0;
		}
	}

	endthread();
}

/**
 * Sends two example CAN frames.
 */
static int zynqmpCanTest_SendTwoFrames(oid_t *oid)
{
	static const zynqmp_canFrame frames[] = {
		{ .id = 0x100,
			.len = 2,
			.payload.bytes = { 0x21, 0x37 } },
		{ .id = 0x200,
			.len = 8,
			.payload.bytes = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88 } },
	};

	/* Perform non-blocking send */
	uint32_t framesSent = 0;
	int ret = zynqmp_canSend(oid, frames, (sizeof(frames) / sizeof(frames[0])), false, &framesSent);

	if (ret == 0) {
		printf("zynqmp-can-test: Successfully sent %u CAN frames\n", framesSent);
		return EXIT_SUCCESS;
	}
	else {
		printf("zynqmp-can-test: Failed to send CAN frames, error %i\n", ret);
		return EXIT_FAILURE;
	}
}

/**
 * Sends CAN frames with blocking enabled.
 */
static int zynqmpCanTest_SendBlocking(oid_t *oid)
{
	/* Generate example data: "enumerated" frames */
	zynqmp_canFrame frames[64] = { 0 };
	for (uint32_t i = 0; i < (sizeof(frames) / sizeof(frames[0])); i++) {
		frames[i].id = 0x100;
		frames[i].len = 1;
		frames[i].payload.bytes[0] = i;
	}

	/* Send first 32 frames without problems and wait */
	uint32_t framesSent = 0;
	int ret = zynqmp_canSend(oid, frames, 32, true, &framesSent);
	if (ret == 0) {
		printf("zynqmp-can-test: Successfully sent %u CAN frames\n", framesSent);
	}
	else {
		printf("zynqmp-can-test: Failed to send CAN frames, error %i\n", ret);
		return EXIT_FAILURE;
	}

	/* Send frames again using the whole buffer; it should block but succeed */
	ret = zynqmp_canSend(oid, frames, (sizeof(frames) / sizeof(frames[0])), true, &framesSent);
	if (ret == 0) {
		printf("zynqmp-can-test: Successfully sent %u CAN frames\n", framesSent);
		return EXIT_SUCCESS;
	}
	else {
		printf("zynqmp-can-test: Failed to send CAN frames, error %i\n", ret);
		return EXIT_FAILURE;
	}
}

/**
 * Sends CAN frames without blocking.
 */
static int zynqmpCanTest_SendNonBlocking(oid_t *oid)
{
	/* Generate example data: "enumerated" frames */
	zynqmp_canFrame frames[64] = { 0 };
	for (uint32_t i = 0; i < (sizeof(frames) / sizeof(frames[0])); i++) {
		frames[i].id = 0x100;
		frames[i].len = 1;
		frames[i].payload.bytes[0] = i;
	}

	/* Send first 32 frames without problems and without waiting */
	uint32_t framesSent = 0;
	int ret = zynqmp_canSend(oid, frames, 32, false, &framesSent);
	if (ret == 0) {
		printf("zynqmp-can-test: Successfully sent %u CAN frames\n", framesSent);
	}
	else {
		printf("zynqmp-can-test: Failed to send CAN frames, error %i\n", ret);
		return EXIT_FAILURE;
	}

	/* Send frames again using the whole buffer; it should fail to send all frames */
	ret = zynqmp_canSend(oid, frames, (sizeof(frames) / sizeof(frames[0])), false, &framesSent);
	if (ret == -EAGAIN) {
		printf("zynqmp-can-test: As expected, failed to send all frames, sent only %u frames\n", framesSent);
		return EXIT_SUCCESS;
	}
	else {
		printf("zynqmp-can-test: Unexpected test result, error code %i, sent %u frames\n", ret, framesSent);
		return EXIT_FAILURE;
	}
}

/**
 * Receives CAN frames from the RX FIFO without blocking
 */
static int zynqmpCanTest_RecvNoBlock(oid_t *oid)
{
	/* Read all frames from RX FIFO */
	zynqmp_canFrame buf[64] = { 0 };
	uint32_t recvFrames = 0;
	int ret = zynqmp_canRecv(oid, buf, (sizeof(buf) / sizeof(buf[0])), false, 0, &recvFrames);
	if (ret == 0) {
		printf("zynqmp-can-test: Successfully read %u CAN frames\n", recvFrames);
		for (uint32_t i = 0; ((i < (sizeof(buf) / sizeof(buf[0]))) && (i < recvFrames)); i++) {
			printf("CAN frame ID: 0x%x DLC: %u Data: ", buf[i].id, buf[i].len);
			for (int n = 0; n < buf[i].len; n++) {
				printf("0x%02X ", buf[i].payload.bytes[n]);
			}
			printf("\n");
		}
		return EXIT_SUCCESS;
	}
	else {
		printf("zynqmp-can-test: Failed to receive frames, error %i\n", ret);
		return EXIT_FAILURE;
	}
}

/**
 * Receives CAN frames from the RX FIFO, block until there are frames
 */
static int zynqmpCanTest_RecvBlock(oid_t *oid)
{
	/* Read all frames from RX FIFO */
	zynqmp_canFrame buf[64] = { 0 };
	uint32_t recvFrames = 0;
	uint32_t timeUs = 0; /* Wait indefinitely */
	int ret = zynqmp_canRecv(oid, buf, (sizeof(buf) / sizeof(buf[0])), true, timeUs, &recvFrames);
	if (ret == 0) {
		printf("zynqmp-can-test: Successfully read %u CAN frames\n", recvFrames);
		for (uint32_t i = 0; ((i < (sizeof(buf) / sizeof(buf[0]))) && (i < recvFrames)); i++) {
			printf("CAN frame ID: 0x%x DLC: %u Data: ", buf[i].id, buf[i].len);
			for (int n = 0; n < buf[i].len; n++) {
				printf("0x%02X ", buf[i].payload.bytes[n]);
			}
			printf("\n");
		}
		return EXIT_SUCCESS;
	}
	else {
		printf("zynqmp-can-test: Failed to receive frames, error %i\n", ret);
		return EXIT_FAILURE;
	}
}

/**
 * Receives CAN frames from the RX FIFO, block with timeout until there are frames
 */
static int zynqmpCanTest_RecvBlockTimeout(oid_t *oid)
{
	/* Read all frames from RX FIFO */
	zynqmp_canFrame buf[64] = { 0 };
	uint32_t recvFrames = 0;
	uint32_t timeUs = 5 * 1000 * 1000; /* Wait a few seconds */
	int ret = zynqmp_canRecv(oid, buf, (sizeof(buf) / sizeof(buf[0])), true, timeUs, &recvFrames);
	if (ret == 0) {
		printf("zynqmp-can-test: Successfully read %u CAN frames\n", recvFrames);
		for (uint32_t i = 0; ((i < (sizeof(buf) / sizeof(buf[0]))) && (i < recvFrames)); i++) {
			printf("CAN frame ID: 0x%x DLC: %u Data: ", buf[i].id, buf[i].len);
			for (int n = 0; n < buf[i].len; n++) {
				printf("0x%02X ", buf[i].payload.bytes[n]);
			}
			printf("\n");
		}
		return EXIT_SUCCESS;
	}
	else if (ret == -ETIME) {
		printf("zynqmp-can-test: Received zero frames, timeout\n");
		return EXIT_SUCCESS;
	}
	else {
		printf("zynqmp-can-test: Failed to receive frames, error %i\n", ret);
		return EXIT_FAILURE;
	}
}

/**
 * Restore saved termios state
 */
static void zynqmpCanTest_RestoreTermios(struct termios *cfg)
{
	tcsetattr(STDIN_FILENO, TCSANOW, cfg);
}

/**
 * Receives CAN frames in a loop with a timeout.
 * Displays received frame statistics in real time.
 */
static int zynqmpCanTest_RecvLoop(oid_t *oid)
{
	common.run = 1;

	/* Save previous termios state */
	struct termios termiosCfg;
	if (tcgetattr(STDIN_FILENO, &termiosCfg) < 0) {
		fprintf(stderr, "cantest: tcgetattr() failed\n");
		return EXIT_FAILURE;
	}

	/* Modify termios state */
	struct termios termiosCfgNew;
	memcpy(&termiosCfgNew, &termiosCfg, sizeof(struct termios));
	termiosCfgNew.c_lflag &= ~ICANON;
	termiosCfgNew.c_lflag &= ~ECHO;
	termiosCfgNew.c_cc[VMIN] = 1;
	if (tcsetattr(STDIN_FILENO, TCSANOW, &termiosCfgNew) < 0) {
		fprintf(stderr, "cantest: failed to set termios mode\n");
		return EXIT_FAILURE;
	}

	if (mutexCreate(&common.mutex_stats_access) != 0) {
		fprintf(stderr, "cantest: Failed to create mutex\n");
		zynqmpCanTest_RestoreTermios(&termiosCfg);
		return EXIT_FAILURE;
	}

	/* Allocate buffer for received frames */
	zynqmp_canFrame buf[64] = { 0 };
	uint32_t recv_frames = 0;

	/* Start thread to print results each second */
	handle_t thread_hdl;
	beginthread(zynqmpCanTest_statsPrintThread, 4, common.stack, sizeof(common.stack), &thread_hdl);

	/* Receive frames in a loop with timeout so the loop can be exited */
	while (common.run) {
		uint32_t timeoutUs = 50 * 1000;
		int ret = zynqmp_canRecv(oid, buf, (sizeof(buf) / sizeof(buf[0])), true, timeoutUs, &recv_frames);
		if (ret == -ETIME) {
			/* Timeout, ignore and keep trying to read frames (if run is 1) */
		}
		else if (ret == 0) {
			if (recv_frames == 0) {
				printf("zynqmp-can-test: Failed, received 0 frames\n");
			}
			for (uint32_t i = 0; ((i < (sizeof(buf) / sizeof(buf[0]))) && (i < recv_frames)); i++) {
				/* Access statistics data displayed by another thread */
				mutexLock(common.mutex_stats_access);
				/* Update statistics for all possible frames */
				if (buf[i].id > ZYNQMP_CAN_ID_MAX) {
					printf("zynqmp-can-test: Failed, CAN ID too large\n");
				}
				else {
					common.frame_stats[buf[i].id].receivedFramesCount++;
					memcpy(&common.frame_stats[buf[i].id].lastFrame, &buf[i], sizeof(zynqmp_canFrame));
				}
				mutexUnlock(common.mutex_stats_access);
			}
		}
		else {
			printf("zynqmp-can-test: Failed to receive frames, error %i\n", ret);
			zynqmpCanTest_RestoreTermios(&termiosCfg);
			return EXIT_FAILURE;
		}
	}

	zynqmpCanTest_RestoreTermios(&termiosCfg);
	return EXIT_SUCCESS;
}

/**
 * Main function to execute CAN test operations.
 */
int main(int argc, char **argv)
{
	/* Lookup hardcoded CAN device */
	char *canDevice = "/dev/can1";
	oid_t oid;
	if (lookup(canDevice, NULL, &oid) < 0) {
		printf("cantest: Cannot lookup device /dev/can1\n");
		return EXIT_FAILURE;
	}

	/* Check if the correct number of arguments was passed */
	if (argc != 2) {
		printf("usage: zynqmp-can-test <operation>\n");
		return EXIT_FAILURE;
	}

	/* Perform action based on command */
	if (strcmp("send", argv[1]) == 0) {
		return zynqmpCanTest_SendTwoFrames(&oid);
	}
	if (strcmp("sendblock", argv[1]) == 0) {
		return zynqmpCanTest_SendBlocking(&oid);
	}
	if (strcmp("sendnoblock", argv[1]) == 0) {
		return zynqmpCanTest_SendNonBlocking(&oid);
	}
	else if (strcmp("recvnoblock", argv[1]) == 0) {
		return zynqmpCanTest_RecvNoBlock(&oid);
	}
	else if (strcmp("recvblock", argv[1]) == 0) {
		return zynqmpCanTest_RecvBlock(&oid);
	}
	else if (strcmp("recvblockwithtimeout", argv[1]) == 0) {
		return zynqmpCanTest_RecvBlockTimeout(&oid);
	}
	else if (strcmp("recvloop", argv[1]) == 0) {
		return zynqmpCanTest_RecvLoop(&oid);
	}
	else {
		printf("Wrong command\n");
		return EXIT_FAILURE;
	}
}
