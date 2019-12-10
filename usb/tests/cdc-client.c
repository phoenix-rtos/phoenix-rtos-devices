/*
 * Phoenix-RTOS
 *
 * cdc client - Example of using: USB Communication Device Class
 *
 * Copyright 2019 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <sys/platform.h>
#include <phoenix/arch/imxrt.h>

#include <cdc.h>

#define LOG(str, ...) do { if (1) fprintf(stderr, "cdc-client: " str "\n", ##__VA_ARGS__); } while (0)
#define LOG_ERROR(str, ...) do { fprintf(stderr, __FILE__  ":%d error: " str "\n", __LINE__, ##__VA_ARGS__); } while (0)

#define BUFF_SIZE 0x400

struct {
	char buff[BUFF_SIZE];
} client_common;


static void cdc_enabelCache(unsigned char enable)
{
	platformctl_t pctl;

	pctl.action = pctl_set;
	pctl.type = pctl_devcache;
	pctl.devcache.state = !!enable;

	platformctl(&pctl);
}


static void cdc_fillBuff(void)
{
	int i, hd;

	for (i = 0, hd = 64; i < hd; ++i)
		client_common.buff[i] = 'f';

	for (i = hd; i < BUFF_SIZE; ++i)
		client_common.buff[i] = 'a';

	client_common.buff[0] = '\n';
}


int main(int argc, char **argv)
{
	cdc_line_coding_t lineCoding;

	cdc_enabelCache(0);

	if (cdc_init()) {
		LOG_ERROR("couldn't initialize CDC transport.");
		return -1;
	}

	LOG("initialized.");

	/* SET Line Coding Request */
	cdc_recv((char *)client_common.buff, 0x20);
	memcpy((void *)&lineCoding, (void *)client_common.buff, sizeof(cdc_line_coding_t));
	LOG("basic attributes: \nSpeed: %d [bps] \tStop bits: %d \tParity: %d \tData length: %d", lineCoding.line_speed, lineCoding.stop_bits, lineCoding.parity, lineCoding.len);

	/* SET Control Line State */
	cdc_recv((char *)client_common.buff, 0x20);

	/* SET Line Coding Request - this packet is received only if the ttyACM is opened with additional attributes */
	cdc_recv((char *)client_common.buff, 0x20);
	memcpy((void *)&lineCoding, (void *)client_common.buff, sizeof(cdc_line_coding_t));
	LOG("new attributes: \nSpeed: %d [bps] \tStop bits: %d \tParity: %d \tData length: %d", lineCoding.line_speed, lineCoding.stop_bits, lineCoding.parity, lineCoding.len);

	cdc_fillBuff();

	/* Send example data
	 * CDC Header is ommited. Due to this fact, the packet's protocol is unknown. */
	cdc_send((char *)client_common.buff, BUFF_SIZE);
	cdc_send((char *)client_common.buff, BUFF_SIZE);
	cdc_send((char *)client_common.buff, BUFF_SIZE);
	cdc_send((char *)client_common.buff, BUFF_SIZE);

	LOG("Transaction completed.");

	while (1)
	{};

	return EOK;
}
