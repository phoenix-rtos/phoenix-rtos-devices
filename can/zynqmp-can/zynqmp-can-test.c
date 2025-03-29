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
#include <termios.h>
#include <stddef.h>
#include <sys/types.h>
#include <phoenix/msg.h>
#include <sys/threads.h>
#include <sys/msg.h>
#include <signal.h>
#include <unistd.h>

#include <zynqmp-can-if.h>

/* Macro to calculate the length of an array */
#define ARRAY_LENGTH(array) (sizeof((array)) / sizeof((array)[0]))

/* Structure to store statistics for received frames */
typedef struct {
	uint32_t receivedFramesCount; /**< Number of received frames */
	zynqmp_canFrame lastFrame;    /**< Last received frame */
} frameStats;

/* Mutex for accessing frame statistics */
static handle_t mutex_stats_access;

/* Array to store statistics for each CAN ID */
static frameStats frame_stats[ZYNQMP_CAN_ID_MAX] = { 0 };

/* Flag to control the main loop */
static int run = 1;

/**
 * Waits for a command input for a specified number of seconds.
 */
static int waitcmd(unsigned int secs)
{
	struct timeval left;
	fd_set fds;

	left.tv_sec = (time_t)secs;
	left.tv_usec = 0;
	fflush(stdout);
	FD_ZERO(&fds);
	FD_SET(STDIN_FILENO, &fds);
	select(STDIN_FILENO + 1, &fds, NULL, NULL, &left);

	if (FD_ISSET(STDIN_FILENO, &fds))
		return getchar();

	return 0;
}

/**
 * Decodes and prints the contents of a CAN frame based on its ID.
 */
static void decode_and_print_frame(zynqmp_canFrame *frame)
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
static void cantest_thread(void *arg)
{
	unsigned int delay = 1;

	/* Clear terminal */
	printf("\033[2J");

	/* Disable cursor */
	printf("\033[?25l");

	while (run) {
		/* Move cursor to the beginning */
		printf("\033[f");
		printf("\033[K");
		printf("\033[2J");

		mutexLock(mutex_stats_access);

		printf("   ID | Count | Frame content \n");
		for (uint32_t i = 0; i < ZYNQMP_CAN_ID_MAX; i++) {
			if (frame_stats[i].receivedFramesCount != 0) {
				printf("0x%3X |  %4i | ", i, frame_stats[i].receivedFramesCount);
				decode_and_print_frame(&frame_stats[i].lastFrame);
			}
			frame_stats[i].receivedFramesCount = 0;
		}

		mutexUnlock(mutex_stats_access);

		int cmd = waitcmd(delay);
		if (cmd == 'q') {
			run = 0;
		}
	}

	endthread();
}

/**
 * Sends two example CAN frames.
 */
static int send_two_frames(oid_t *oid)
{
	zynqmp_canFrame frames[] = {
		{ .id = 0x100,
			.len = 2,
			.payload.bytes = { 0x21, 0x37 } },
		{ .id = 0x200,
			.len = 8,
			.payload.bytes = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88 } },
	};

	/* Perform non-blocking send */
	uint32_t sendFrames = 0;
	int ret = zynqmp_can_send(oid, frames, ARRAY_LENGTH(frames), false, &sendFrames);

	if (EOK == ret) {
		printf("zynqmp-can-test: Successfully sent %u CAN frames\n", sendFrames);
	}
	else {
		printf("zynqmp-can-test: Failed to send CAN frames, error %i\n", ret);
	}

	return ret;
}

/**
 * Sends CAN frames with blocking enabled.
 */
static int send_with_blocking(oid_t *oid)
{
	/* Generate example data: "enumerated" frames */
	static zynqmp_canFrame frames[64] = { 0 };
	for (uint32_t i = 0; i < ARRAY_LENGTH(frames); i++) {
		frames[i].id = 0x100;
		frames[i].len = 1;
		frames[i].payload.bytes[0] = i;
	}

	/* Send first 32 frames without problems and wait */
	uint32_t sendFrames = 0;
	int ret = zynqmp_can_send(oid, frames, 32, true, &sendFrames);
	if (EOK == ret) {
		printf("zynqmp-can-test: Successfully sent %u CAN frames\n", sendFrames);
	}
	else {
		printf("zynqmp-can-test: Failed to send CAN frames, error %i\n", ret);
		return ret;
	}

	/* Send frames again using the whole buffer; it should block but succeed */
	ret = zynqmp_can_send(oid, frames, ARRAY_LENGTH(frames), true, &sendFrames);
	if (EOK == ret) {
		printf("zynqmp-can-test: Successfully sent %u CAN frames\n", sendFrames);
	}
	else {
		printf("zynqmp-can-test: Failed to send CAN frames, error %i\n", ret);
	}

	return ret;
}

/**
 * Sends CAN frames without blocking.
 */
static int send_without_blocking(oid_t *oid)
{
	/* Generate example data: "enumerated" frames */
	static zynqmp_canFrame frames[64] = { 0 };
	for (uint32_t i = 0; i < ARRAY_LENGTH(frames); i++) {
		frames[i].id = 0x100;
		frames[i].len = 1;
		frames[i].payload.bytes[0] = i;
	}

	/* Send first 32 frames without problems and without waiting */
	uint32_t sendFrames = 0;
	int ret = zynqmp_can_send(oid, frames, 32, false, &sendFrames);
	if (EOK == ret) {
		printf("zynqmp-can-test: Successfully sent %u CAN frames\n", sendFrames);
	}
	else {
		printf("zynqmp-can-test: Failed to send CAN frames, error %i\n", ret);
		return ret;
	}

	/* Send frames again using the whole buffer; it should fail to send all frames */
	ret = zynqmp_can_send(oid, frames, ARRAY_LENGTH(frames), false, &sendFrames);
	if (ret == -ENOMEM) {
		printf("zynqmp-can-test: As expected, failed to send all frames, sent only %u frames\n", sendFrames);
	}
	else {
		printf("zynqmp-can-test: Unexpected test result, error code %i, sent %u frames\n", ret, sendFrames);
	}

	return ret;
}

/**
 * Receives CAN frames from the RX FIFO once.
 */
static int recv_one_time(oid_t *oid)
{
	/* Read all frames from RX FIFO */
	zynqmp_canFrame buf[64] = { 0 };
	uint32_t recvFrames = 0;
	time_t time = 0; /* Wait indefinitely */
	int ret = zynqmp_can_recv(oid, buf, ARRAY_LENGTH(buf), time, &recvFrames);
	if (ret == 0) {
		printf("zynqmp-can-test: Successfully read %u CAN frames\n", recvFrames);
		for (uint32_t i = 0; ((i < ARRAY_LENGTH(buf)) && (i < recvFrames)); i++) {
			printf("CAN frame ID: 0x%x DLC: %u Data: ", buf[i].id, buf[i].len);
			for (int n = 0; n < buf[i].len; n++) {
				printf("0x%02X ", buf[i].payload.bytes[n]);
			}
			printf("\n");
		}
	}
	else {
		printf("zynqmp-can-test: Failed to receive frames, error %i\n", ret);
	}

	return ret;
}

/**
 * Receives CAN frames from the RX FIFO with a timeout.
 */
static int recv_one_time_with_timeout(oid_t *oid)
{
	/* Read all frames from RX FIFO */
	zynqmp_canFrame buf[64] = { 0 };
	uint32_t recvFrames = 0;
	time_t time = 5 * 1000 * 1000; /* Wait a few seconds */
	int ret = zynqmp_can_recv(oid, buf, ARRAY_LENGTH(buf), time, &recvFrames);
	if (ret == 0) {
		printf("zynqmp-can-test: Successfully read %u CAN frames\n", recvFrames);
		for (uint32_t i = 0; ((i < ARRAY_LENGTH(buf)) && (i < recvFrames)); i++) {
			printf("CAN frame ID: 0x%x DLC: %u Data: ", buf[i].id, buf[i].len);
			for (int n = 0; n < buf[i].len; n++) {
				printf("0x%02X ", buf[i].payload.bytes[n]);
			}
			printf("\n");
		}
	}
	else if (ret == -ETIME) {
		printf("zynqmp-can-test: Received zero frames, timeout\n");
	}
	else {
		printf("zynqmp-can-test: Failed to receive frames, error %i\n", ret);
	}

	return ret;
}

/**
 * Enables or disables canonical mode and echo for the terminal.
 */
static void switchmode(int canon)
{
	struct termios state;

	tcgetattr(STDIN_FILENO, &state);
	if (canon) {
		state.c_lflag |= ICANON;
		state.c_lflag |= ECHO;
	}
	else {
		state.c_lflag &= ~ICANON;
		state.c_lflag &= ~ECHO;
		state.c_cc[VMIN] = 1;
	}
	tcsetattr(STDIN_FILENO, TCSANOW, &state);
}

/**
 * Receives CAN frames in a loop with a timeout.
 * Displays received frame statistics in real time.
 */
static int recv_in_loop(oid_t *oid)
{
	switchmode(0);

	if (mutexCreate(&mutex_stats_access) != 0) {
		printf("cantest: Failed to create mutex\n");
		return -1;
	}

	/* Allocate buffer for received frames */
	static zynqmp_canFrame buf[64] = { 0 };
	uint32_t recv_frames = 0;

	/* Start thread to print results each second */
	handle_t thread_hdl;
	uint8_t stack[4096];
	beginthread(cantest_thread, 4, stack, sizeof(stack), &thread_hdl);

	/* Receive frames in a loop with timeout so the loop can be exited */
	while (run) {
		time_t timeoutUs = 50 * 1000;
		int ret = zynqmp_can_recv(oid, buf, ARRAY_LENGTH(buf), timeoutUs, &recv_frames);
		if (ret == -ETIME) {
			/* Timeout, ignore and keep trying to read frames (if run is 1) */
		}
		else if (ret == EOK) {
			if (recv_frames == 0) {
				printf("zynqmp-can-test: Failed, received 0 frames\n");
			}
			for (uint32_t i = 0; ((i < ARRAY_LENGTH(buf)) && (i < recv_frames)); i++) {
				/* Access statistics data displayed by another thread */
				mutexLock(mutex_stats_access);
				/* Update statistics for all possible frames */
				if (buf[i].id > ZYNQMP_CAN_ID_MAX) {
					printf("zynqmp-can-test: Failed, CAN ID too large\n");
				}
				else {
					frame_stats[buf[i].id].receivedFramesCount++;
					memcpy(&frame_stats[buf[i].id].lastFrame, &buf[i], sizeof(zynqmp_canFrame));
				}
				mutexUnlock(mutex_stats_access);
			}
		}
		else {
			printf("zynqmp-can-test: Failed to receive frames, error %i\n", ret);
			return ret;
		}
	}

	switchmode(1);

	return 0;
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
		return -1;
	}

	/* Check if the correct number of arguments was passed */
	if (argc != 2) {
		printf("usage: zynqmp-can-test <operation>\n");
		return -1;
	}

	/* Perform action based on command */
	if (0 == strcmp("send", argv[1])) {
		return send_two_frames(&oid);
	}
	if (0 == strcmp("sendblock", argv[1])) {
		return send_with_blocking(&oid);
	}
	if (0 == strcmp("sendnoblock", argv[1])) {
		return send_without_blocking(&oid);
	}
	else if (0 == strcmp("recv", argv[1])) {
		return recv_one_time(&oid);
	}
	else if (0 == strcmp("recvwithtimeout", argv[1])) {
		return recv_one_time_with_timeout(&oid);
	}
	else if (0 == strcmp("recvloop", argv[1])) {
		return recv_in_loop(&oid);
	}
	else {
		printf("Wrong command\n");
		return -1;
	}
}
