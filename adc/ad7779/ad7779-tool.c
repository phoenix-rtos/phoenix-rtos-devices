/*
 * Phoenix-RTOS
 *
 * AD7779 driver tool
 *
 * Copyright 2023 Phoenix Systems
 * Author: Marek Bialowas
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <assert.h>
#include <errno.h>
#include <signal.h>
#include <sys/msg.h>
#include <sys/mman.h>
#include <sys/stat.h>

#include <adc-api-ad7779.h>

#define DEV_PATH "/dev/ad7779"

/** ADC configuration for DTR 3-phase measurement **/
#define ACTIVE_CHANNELS 0b00111111
#define TOTAL_CHANNELS  8
#define PHASE_CNT       3

#define V_PGA 2
#define I_PGA 8

const static int adc_gain[] = { I_PGA, V_PGA, I_PGA, V_PGA, I_PGA, V_PGA, 1, 1 };

static struct {
	struct {
		size_t size;
		size_t nr;
		volatile uint32_t *ptr[2];
	} config;

	int verbose;
	volatile sig_atomic_t running;
} tool_common;


#define log_verbose(fmt, ...) \
	do { \
		if (tool_common.verbose > 0) \
			printf(fmt, ##__VA_ARGS__); \
	} while (0)


static int adc_get(uint32_t port, adc_dev_ctl_type_t type, adc_dev_ctl_t *ioctl_out)
{
	msg_t msg = {
		.type = mtDevCtl,
	};

	adc_dev_ctl_t ioctl = {
		.type = type,
	};

	memcpy(msg.o.raw, &ioctl, sizeof(adc_dev_ctl_t));
	int res = msgSend(port, &msg);
	if ((res < 0) || (msg.o.io.err != 0)) {
		printf("failed to process req type %u sample rate %d, %d\n", type, res, msg.o.io.err);
		return -1;
	}

	memcpy(ioctl_out, msg.o.raw, sizeof(adc_dev_ctl_t));
	return 0;
}


static int adc_getChConfig(uint32_t port, uint32_t channel, adc_dev_ctl_t *ioctl_out)
{
	msg_t msg = {
		.type = mtDevCtl,
	};

	adc_dev_ctl_t ioctl = {
		.type = adc_dev_ctl__get_channel_config,
		.ch_config = { .channel = channel }
	};

	memcpy(msg.o.raw, &ioctl, sizeof(adc_dev_ctl_t));
	int res = msgSend(port, &msg);
	if ((res < 0) || (msg.o.io.err != 0)) {
		printf("failed to process req type %u sample rate %d, %d\n", ioctl.type, res, msg.o.io.err);
		return -1;
	}

	memcpy(ioctl_out, msg.o.raw, sizeof(adc_dev_ctl_t));
	return 0;
}


static int adc_getConfig(uint32_t port, bool configDump)
{
	adc_dev_ctl_t ioctl = { 0 };

	if (adc_get(port, adc_dev_ctl__get_config, &ioctl) < 0) {
		return 1;
	}
	if (configDump) {
		unsigned int ch_cnt = ioctl.config.channels;
		printf("config: sampling rate: %u, enabled ch: 0x%02x, ch_cnt=%u\n", ioctl.config.sampling_rate, ioctl.config.enabled_ch, ch_cnt);

		for (unsigned int ch = 0; ch < ch_cnt; ++ch) {
			if (adc_getChConfig(port, ch, &ioctl) < 0) {
				return 1;
			}
			printf("  [%u] gain=%u, mode=%s\n", ch, ioctl.ch_config.gain, ((ioctl.ch_config.meter_rx_mode != 0) ? "RX" : "REF"));
		}


		printf("ADC status registers: \n");
		if (adc_get(port, adc_dev_ctl__status, &ioctl) < 0) {
			return 1;
		}
		for (int i = 0; i < sizeof(ioctl.status.ch_status); i++) {
			printf("  CH%d_ERR_REG: 0x%02x\n", i, ioctl.status.ch_status[i]);
		}

		for (int i = 0; i < sizeof(ioctl.status.dsp_status); i++) {
			printf("  CH%d_%d_SAT_ERR: 0x%02x\n", 2 * i, 2 * i + 1, ioctl.status.dsp_status[i]);
		}

		for (int i = 0; i < sizeof(ioctl.status.general_err); i++) {
			printf("  GEN_ERR_REG_%d: 0x%02x\n", i, ioctl.status.general_err[i]);
		}

		for (int i = 0; i < sizeof(ioctl.status.status); i++) {
			printf("  STATUS_REG_%d: 0x%02x\n", i, ioctl.status.status[i]);
		}
	}

	if (adc_get(port, adc_dev_ctl__get_buffers, &ioctl) < 0) {
		return 1;
	}

	if (configDump) {
		printf("buffers: addr=%p, len=%u, num=%u\n", (void *)ioctl.buffers.paddr, ioctl.buffers.size, ioctl.buffers.num);
	}

	assert((ioctl.buffers.size % _PAGE_SIZE) == 0);
	assert(ioctl.buffers.num == 2); /* WARN: not tested with other values, might be changed later */

	tool_common.config.size = ioctl.buffers.size / ioctl.buffers.num; /* size in bytes of a single buffer */
	tool_common.config.nr = ioctl.buffers.num;
	volatile uint32_t *base = mmap(NULL, ioctl.buffers.size, PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, ioctl.buffers.paddr);

	if (base == MAP_FAILED) {
		printf("failed to mmap buffers\n");
		return 1;
	}

	for (unsigned i = 0; i < tool_common.config.nr; ++i) {
		tool_common.config.ptr[i] = (uint32_t *)((uint8_t *)base + i * tool_common.config.size);
	}

	return 0;
}


static int adc_reset(uint32_t port)
{

	msg_t msg = { 0 };
	adc_dev_ctl_t ioctl = { 0 };

	msg.type = mtDevCtl;
	ioctl.type = adc_dev_ctl__disable;
	memcpy(msg.o.raw, &ioctl, sizeof(adc_dev_ctl_t));

	log_verbose("disable\n");
	if ((msgSend(port, &msg) < 0) || (msg.o.io.err != 0)) {
		printf("sample: failed to disable adc device: %d\n", msg.o.io.err);
		return 1;
	}

	msg.type = mtDevCtl;
	ioctl.type = adc_dev_ctl__reset;
	ioctl.mux = 0;
	memcpy(msg.o.raw, &ioctl, sizeof(adc_dev_ctl_t));

	log_verbose("reset\n");
	if ((msgSend(port, &msg) < 0) || (msg.o.io.err != 0)) {
		printf("sample: failed to reset adc device: %d\n", msg.o.io.err);
		return 1;
	}

	msg.type = mtDevCtl;
	ioctl.type = adc_dev_ctl__set_config;
	ioctl.config.sampling_rate = 8000;
	ioctl.config.enabled_ch = ACTIVE_CHANNELS;
	memcpy(msg.o.raw, &ioctl, sizeof(adc_dev_ctl_t));

	log_verbose("set_config\n");
	if ((msgSend(port, &msg) < 0) || (msg.o.io.err != 0)) {
		printf("sample: failed to set sample rate: %d\n", msg.o.io.err);
		return 1;
	}

	/* set channel gains */
	for (int i = 0; i < PHASE_CNT * 2; i++) {
		msg.type = mtDevCtl;
		ioctl.type = adc_dev_ctl__set_channel_gain;

		ioctl.gain.channel = i;

		if (((1 << i) & ACTIVE_CHANNELS) == 0) {
			continue;
		}

		ioctl.gain.val = adc_gain[i];
		log_verbose("set_channel_gain ch=%u\n", i);
		memcpy(msg.o.raw, &ioctl, sizeof(adc_dev_ctl_t));
		if ((msgSend(port, &msg) < 0) || (msg.o.io.err != 0)) {
			printf("sample: failed to set channel %d gain: %d\n", i, msg.o.io.err);
			return 1;
		}
	}

	msg.type = mtDevCtl;
	ioctl.type = adc_dev_ctl__enable;
	memcpy(msg.o.raw, &ioctl, sizeof(adc_dev_ctl_t));

	log_verbose("enable\n");
	if ((msgSend(port, &msg) < 0) || (msg.o.io.err != 0)) {
		printf("sample: failed to enable adc: %d\n", msg.o.io.err);
		return 1;
	}

	return 0;
}


static uint32_t read_intr(uint32_t port)
{
	msg_t msg = { 0 };
	uint32_t intr = 0;

	msg.type = mtRead;
	msg.o.size = sizeof(uint32_t);
	msg.o.data = &intr;

	if ((msgSend(port, &msg) < 0) || (msg.o.io.err != 0)) {
		printf("MSG failed to read adc intr: %s", strerror(msg.o.io.err));
	}

	return intr;
}


static void signal_handler(int signal)
{
	tool_common.running = 0;
}


static inline uint8_t raw2channel(uint32_t raw)
{
	return (raw & 0x70000000) >> 28;
}


static inline int32_t raw2value(uint32_t raw)
{
	int32_t value = raw & 0x00FFFFFFu;
	if (value >= 1 << 23) {
		value -= 1 << 24;
	}
	return value;
}


static inline void print_us(const char *type, uint64_t us)
{
	printf("time %s: %llu.%03llu ms\n", type, (us) / 1000, (us) % 1000);
}


static int adc_read(uint32_t port, uint32_t readIntCnt, int streamFd, bool dumpSampleRate, bool dumpAverage)
{
	uint32_t first_intr = 0;
	uint32_t last_intr = 0;

	int64_t sum[TOTAL_CHANNELS] = { 0 };
	uint32_t sumcnt = 0;
	int retCode = 0;


	tool_common.running = 1;

	struct timespec start, end;
	for (int i = 1; i <= readIntCnt; ++i) {
		uint32_t intr = read_intr(port);

		if (first_intr == 0) {
			clock_gettime(CLOCK_MONOTONIC_RAW, &start);
			first_intr = intr;
		}

		if ((last_intr != 0) && (last_intr + 1 != intr)) {
			printf("missed intr! %u -> %u\n", last_intr, intr);
			retCode = 1; /* signal with exit code that data might not be reliable */
		}

		last_intr = intr;

		/* support infinite running until ^C */
		if (tool_common.running == 0) {
			break;
		}

		/* dumping & averaging */
		if ((streamFd >= 0) || dumpAverage) {
			volatile uint32_t *samplebuf = tool_common.config.ptr[(intr - 1) % tool_common.config.nr];

			const int frames = (tool_common.config.size / sizeof(uint32_t)) / TOTAL_CHANNELS;
			int32_t writebuf[4096 / sizeof(uint32_t)];
			int32_t *writeptr = writebuf;
			const int32_t *writebufEnd = writebuf + (sizeof(writebuf) / sizeof(writebuf[0]));

			for (int i = 0; i < frames * TOTAL_CHANNELS; i++) {
				bool lastIteration = (i == (frames * TOTAL_CHANNELS - 1));
				uint32_t raw = *samplebuf++;
				int32_t value = raw2value(raw);

				sum[i % TOTAL_CHANNELS] += value;

				if (streamFd >= 0) {
					*writeptr++ = value;

					if ((writeptr == writebufEnd) || lastIteration) {
						size_t writeSz = (uint8_t *)writeptr - (uint8_t *)writebuf;
						if (write(streamFd, &writebuf, writeSz) != writeSz) {
							printf("stream write error: %s\n", strerror(errno));
							retCode = 1; /* signal with exit code that data might not be reliable */
						}

						writeptr = writebuf;
					}
				}
			}

			sumcnt += frames;
		}
	}

	clock_gettime(CLOCK_MONOTONIC_RAW, &end);

	if (dumpAverage) {
		if (tool_common.verbose == 0) {
			printf("AVG: ");
		}
		else {
			printf("AVG[%u]:\n", sumcnt);
		}
		for (int i = 0; i < TOTAL_CHANNELS; ++i) {
			const float valMax = (uint32_t)(1 << 23) - 1;
			const float voltMax = 1.65f / adc_gain[i];

			float avg = (float)sum[i] / sumcnt;
			float ratio = avg / valMax;
			float voltageOffs = ratio * voltMax;
			if (tool_common.verbose > 0) {
				printf("  [%u] avg: %16.3f, proportional: %10.3f %% ==> voltage offset: %6.3f / %6.3f V\n", i, avg, ratio * 100, voltageOffs, voltMax);
			}
			else {
				printf("%.2f%%, ", ratio * 100);
			}
		}

		if (tool_common.verbose == 0) {
			printf("\n");
		}
	}

	if (dumpSampleRate) {
		uint64_t us = (end.tv_sec * 1000000 + end.tv_nsec / 1000) - (start.tv_sec * 1000000 + start.tv_nsec / 1000);
		uint64_t us_per_buffer = us / (last_intr - first_intr);
		uint64_t us_per_sample = us_per_buffer / (tool_common.config.size / TOTAL_CHANNELS / sizeof(uint32_t));

		if (tool_common.verbose > 0) {
			printf("read operations: %u\n", readIntCnt);
			printf("ints: %u - %u  + 1 = %u\n", last_intr, first_intr, last_intr - first_intr + 1);
			print_us("total", us);
			print_us("per buffer", us_per_buffer);
			print_us("per sample", us_per_sample);
		}

		printf("freq = %6.3f\n", (float)1000000 / us_per_sample);
	}

	return retCode;
}


static void print_usage(const char *progname)
{
	printf("Usage: %s [OPTIONS]\n", progname);
	puts("\t-h             This help message");
	puts("\t-v             Increase verbosity");
	puts("\t-r             Reset ADC");
	puts("\t-c             Show current configuration");
	puts("\t-s             Estimate current samplerate");
	puts("\t-a             Estimate average signal per channel");
	puts("\t-i [int_cnt]   Limit reading samples to [int_cnt] interrupts (default: infinite until ^C)");
	puts("\t-d [file.pcm]  Dump samples to file");
}


int main(int argc, char **argv)
{
	oid_t oid;
	uint32_t readIntCnt = UINT32_MAX;
	char *str;
	bool do_adcReset = false;
	bool do_configDump = false;
	bool do_sampleRateEstimate = false;
	bool do_sampleAverage = false;
	int ret = 0;
	const char *dumpFname = NULL;

	while (true) {
		int c = getopt(argc, argv, "acd:hi:rsv");
		if (c == -1) {
			break;
		}

		switch (c) {
			case 'a':
				do_sampleAverage = true;
				break;

			case 'v':
				tool_common.verbose += 1;
				break;

			case 'h':
				print_usage(argv[0]);
				return 0;

			case 'r':
				do_adcReset = true;
				break;

			case 's':
				do_sampleRateEstimate = true;
				break;

			case 'c':
				do_configDump = true;
				break;

			case 'i':
				readIntCnt = strtoul(optarg, &str, 10);
				if ((str == optarg) || (*str != '\0')) {
					printf("invalid -i value\n");
					return 1;
				}
				break;

			case 'd':
				dumpFname = optarg;
				break;

			default:
				print_usage(argv[0]);
				return 1;
		}
	}

	if (lookup(DEV_PATH, NULL, &oid) < 0) {
		printf("adc device: " DEV_PATH " not found\n");
		return 2;
	}

	bool needs_adcRead = (do_sampleRateEstimate || do_sampleAverage);
	int streamFd = -1;
	if (dumpFname != NULL) {
		needs_adcRead = 1;
		streamFd = open(dumpFname, O_WRONLY | O_CREAT | O_TRUNC, DEFFILEMODE);
		if (streamFd < 0) {
			perror("failed to open stream file");
			return 2;
		}
	}

	signal(SIGINT, signal_handler);

	if (do_adcReset) {
		ret |= adc_reset(oid.port);
	}

	if (needs_adcRead || do_configDump) {
		ret |= adc_getConfig(oid.port, do_configDump);
	}
	if (needs_adcRead) {
		ret |= adc_read(oid.port, readIntCnt, streamFd, do_sampleRateEstimate, do_sampleAverage);
	}

	if (tool_common.config.ptr[0] != NULL) {
		munmap((void *)tool_common.config.ptr[0], tool_common.config.size * tool_common.config.nr);
	}

	if (streamFd >= 0) {
		close(streamFd);
	}

	return ret;
}
