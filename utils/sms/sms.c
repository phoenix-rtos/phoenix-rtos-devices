/*
 * Phoenix-RTOS
 *
 * Tool for sending sms
 *
 * sms.c
 *
 * Copyright 2019 Phoenix Systems
 * Author: Jan Sikorski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>

#define DEBUG(...)
// #define DEBUG(...) printf(__VA_ARGS__)

#define WRITE_MAX_RETRIES 3
#define READ_RETRY_TIME_MS 10

enum {
	AT_RESULT_OK,
	AT_RESULT_CONNECT,
	AT_RESULT_RING,
	AT_RESULT_NO_CARRIER,
	AT_RESULT_ERROR,
	AT_RESULT_NO_ANSWER,
	AT_RESULT_PROMPT
};


static const char* at_result_codes[] = { "OK", "CONNECT", "RING", "NO CARRIER", "ERROR", "NO ANSWER", "> ", NULL };
static char readbuf[0x10000];
static char writebuf[0x1000];
static int fd;


void usage(char *name)
{
	fprintf(stderr, "usage:\n"
		"\t%s show - display messages\n"
		"\t%s send number text - send message containing text to number\n"
		"\t%s delete index - delete message stored at given index\n", name, name, name);
	exit(EXIT_FAILURE);
}


static int serial_write(const char *data, unsigned len)
{
	int off = 0;
	int retries = 0;
	int res, to_write;

	while (off < len) {
		to_write = len - off;
		res = write(fd, data + off, to_write);

		if (res < 0) {
			if (errno == EINTR) {
				usleep(5 * 1000);
				continue;
			}

			if (errno == EWOULDBLOCK) {
				if (retries >= WRITE_MAX_RETRIES)
					break;

				retries += 1;
				usleep(5 * 1000);
				continue;
			}

			DEBUG("write fail\n");
			return -1;
		}

		off += res;
		retries = 0;
	}

	return off;
}


static int at_check_result(int off)
{
	int res = 0;
	char *result;

	while (at_result_codes[res]) {
		if ((result = strstr(readbuf + off, at_result_codes[res])) != NULL) {
			*result = '\0';
			return res;
		}
		res += 1;
	}

	return -1;
}


static int at_send_cmd(const char *cmd, int timeout_ms)
{
	int max_len = sizeof(readbuf) - 1;
	int off = 0, len, res;

	DEBUG("AT send: [%s]\n", cmd);
	serial_write(cmd, strlen(cmd));

	while (off < max_len) {
		len = read(fd, readbuf + off, max_len - off);

		if (len < 0) {
			if (errno == EINTR)
				continue;

			if (errno == EWOULDBLOCK) {
				usleep(READ_RETRY_TIME_MS * 1000);

				if (timeout_ms >= 0) {
					timeout_ms -= READ_RETRY_TIME_MS;

					if (timeout_ms <= 0) {
						DEBUG("AT timeout\n");
						return -1;
					}
				}

				continue;
			}

			break;
		} else if (len == 0) {
			break;
		}

		DEBUG("AT recv: [%s]\n", readbuf);

		readbuf[off + len + 1] = '\0';

		res = at_check_result(off);
		off += len;

		if (res >= 0)
			return res;
	}

	DEBUG("response too big\n");
	return -1;
}


int main(int argc, char **argv)
{
	int index, err;
	const char *acm_path;

	if (argc < 2)
		usage(argv[0]);

	if ((acm_path = getenv("SMS_TTYACM_PATH")) == NULL)
		acm_path = "/dev/ttyacm1";

	if ((fd = open(acm_path, O_RDWR | O_NONBLOCK)) < 0) {
		fprintf(stderr, "error opening %s\n", acm_path);
		exit(EXIT_FAILURE);
	}

	if ((err = at_send_cmd("ATE=0\r\n", 300)) != AT_RESULT_OK) {
		fprintf(stderr, "error disabling echo: %s\n", at_result_codes[err]);
		exit(EXIT_FAILURE);
	}

	if ((err = at_send_cmd("AT+CMGF=1\r\n", 300)) != AT_RESULT_OK) {
		fprintf(stderr, "error changing PDU mode: %s\n", at_result_codes[err]);
		exit(EXIT_FAILURE);
	}

	if (!strcmp(argv[1], "show")) {
		if ((err = at_send_cmd("AT+CMGL=ALL\r\n", 20000) != AT_RESULT_OK)) {
			fprintf(stderr, "error reading messages: %s\n", at_result_codes[err]);
			exit(EXIT_FAILURE);
		}

		puts(readbuf);
	}
	else if (!strcmp(argv[1], "send")) {
		if (argc != 4)
			usage(argv[0]);

		snprintf(writebuf, sizeof(writebuf), "AT+CMGW=%s\r\n", argv[2]);

		if ((err = at_send_cmd(writebuf, 300)) != AT_RESULT_PROMPT) {
			fprintf(stderr, "error getting prompt: %s\n", at_result_codes[err]);
			exit(EXIT_FAILURE);
		}

		snprintf(writebuf, sizeof(writebuf), "%s\032", argv[3]);

		if ((err = at_send_cmd(writebuf, 500)) != AT_RESULT_OK) {
			fprintf(stderr, "error writing: %s\n", at_result_codes[err]);
			exit(EXIT_FAILURE);
		}

		sscanf(readbuf, "\n+CMGW: %d", &index);
		snprintf(writebuf, sizeof(writebuf), "AT+CMSS=%d\r\n", index);

		if ((err = at_send_cmd(writebuf, 10000)) != AT_RESULT_OK) {
			fprintf(stderr, "error sending: %s\n", at_result_codes[err]);
			exit(EXIT_FAILURE);
		}
	}
	else if (!strcmp(argv[1], "delete")) {
		if (argc != 3)
			usage(argv[0]);

		snprintf(writebuf, sizeof(writebuf), "AT+CMGD=%s\r\n", argv[2]);

		if ((err = at_send_cmd(writebuf, 500)) != AT_RESULT_OK) {
			fprintf(stderr, "error deleting: %s\n", at_result_codes[err]);
			exit(EXIT_FAILURE);
		}
	}
	else {
		usage(argv[0]);
	}

	return EXIT_SUCCESS;
}
