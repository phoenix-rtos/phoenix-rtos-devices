/*
 * Phoenix-RTOS
 *
 * IMX6ULL battery driver.
 *
 * Reads battery voltage
 *
 * Copyright 2023 Phoenix Systems
 * Author: Ziemowit Leszczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/minmax.h>

#include <sys/msg.h>
#include <sys/mman.h>
#include <sys/platform.h>
#include <posix/utils.h>

#include <phoenix/arch/imx6ull.h>

#include <imx6ull-gpio.h>

#define LOG_TAG       "imx6ull-vbat: "
#define log(fmt, ...) printf(LOG_TAG fmt "\n", ##__VA_ARGS__)

#define DEV_VBAT_ID   0
#define DEV_VBAT_NAME "vbat"

#define VBAT_GPIO_NUM 1
#define VBAT_GPIO_PIN 4
#define VBAT_GPIO_MUX pctl_mux_gpio1_04
#define VBAT_GPIO_PAD pctl_pad_gpio1_04

#define VBAT_ON_GPIO_NUM 5
#define VBAT_ON_GPIO_PIN 9

#define ADC_BASE_ADDR 0x2198000
#define ADC_CLK       pctl_clk_adc1
#define ADC_CHANNEL   4
#define ADC_VREF_MV   3500
#define ADC_MAX_VALUE 4095

/* clang-format off */
enum { hc0 = 0,	hs = 2,	r0,	cfg = 5, gc, gs, cv, ofs, cal };
/* clang-format on */

static struct {
	volatile uint32_t *adc_base;
	int gpio_port;
	uint32_t port;
	bool busy;
	char buf[16];
	int len;
} common;


static int gpio_set_pin(int fd, unsigned int pin, bool state)
{
	gpiodata_t gpiodata;

	gpiodata.w.val = (state ? 1U : 0U) << pin;
	gpiodata.w.mask = 1 << pin;

	if (write(fd, &gpiodata, sizeof(gpiodata)) < sizeof(gpiodata)) {
		return -1;
	}

	return 0;
}


static int gpio_set_dir(int fd, unsigned int pin, bool output)
{
	gpiodata_t gpiodata;

	gpiodata.w.val = (output ? 1U : 0U) << pin;
	gpiodata.w.mask = 1 << pin;

	if (write(fd, &gpiodata, sizeof(gpiodata)) < sizeof(gpiodata)) {
		return -1;
	}

	return 0;
}


static int gpio_init(void)
{
	platformctl_t pctl;
	char buf[32];
	int fd;

	/* Configure VBAT pin as GPIO */
	pctl.action = pctl_set;
	pctl.type = pctl_iomux;
	pctl.iomux.sion = 0;
	pctl.iomux.mux = VBAT_GPIO_MUX;
	pctl.iomux.mode = 5;
	platformctl(&pctl);

	/* Configure VBAT pad */
	pctl.action = pctl_set;
	pctl.type = pctl_iopad;
	pctl.iopad.pad = VBAT_GPIO_PAD;
	pctl.iopad.hys = 0;
	pctl.iopad.pus = 0;
	pctl.iopad.pue = 0;
	pctl.iopad.pke = 0;
	pctl.iopad.ode = 0;
	pctl.iopad.speed = 2;
	pctl.iopad.dse = 0;
	pctl.iopad.sre = 0;
	platformctl(&pctl);

	snprintf(buf, sizeof(buf), "/dev/gpio%u/dir", VBAT_GPIO_NUM);
	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		log("failed to open %s", buf);
		return -1;
	}

	/* Configure VBAT as input */
	gpio_set_dir(fd, VBAT_GPIO_PIN, false);

	close(fd);

	snprintf(buf, sizeof(buf), "/dev/gpio%u/dir", VBAT_ON_GPIO_NUM);
	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		log("failed to open %s", buf);
		return -1;
	}

	/* Configure VBAT_ON as output */
	gpio_set_dir(fd, VBAT_ON_GPIO_PIN, true);

	close(fd);

	snprintf(buf, sizeof(buf), "/dev/gpio%u/port", VBAT_ON_GPIO_NUM);
	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		log("failed to open %s", buf);
		return -1;
	}

	/* Set VBAT_ON to 0 */
	gpio_set_pin(fd, VBAT_ON_GPIO_PIN, false);

	common.gpio_port = fd;

	return 0;
}


static void gpio_deinit(void)
{
	close(common.gpio_port);
}


static int adc_init(void)
{
	platformctl_t pctl;

	common.adc_base = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, ADC_BASE_ADDR);
	if (common.adc_base == MAP_FAILED) {
		log("ADC mmap failed");
		return -1;
	}

	/* Enable ADC clock */
	pctl.action = pctl_set;
	pctl.type = pctl_devclock;
	pctl.devclock.dev = ADC_CLK;
	pctl.devclock.state = 3;
	platformctl(&pctl);

	/* Sample time duration is 20 ADC clocks, long sample time, 12-bit conversion mode */
	*(common.adc_base + cfg) = (2U << 8) | (1U << 4) | (2U << 2);

	/* Start calibration */
	*(common.adc_base + gc) |= 1U << 7;

	/* Wait until calibration is completed */
	while ((*(common.adc_base + gc) & (1U << 7)) != 0) {
		usleep(100);
	}

	/* Check if calibration was successful */
	if ((*(common.adc_base + gs) & (1U << 1)) != 0) {
		log("ADC calibration failed");
		munmap((void *)common.adc_base, 0x1000);
		return -1;
	}

	return 0;
}


static void adc_deinit(void)
{
	platformctl_t pctl;

	munmap((void *)common.adc_base, 0x1000);

	/* Disable ADC clock */
	pctl.action = pctl_set;
	pctl.type = pctl_devclock;
	pctl.devclock.dev = ADC_CLK;
	pctl.devclock.state = 0;
	platformctl(&pctl);
}


static uint16_t adc_read(void)
{
	uint16_t value;

	/* Initiate conversion */
	*(common.adc_base + hc0) = ADC_CHANNEL;

	/* Wait until conversion is completed */
	while ((*(common.adc_base + hs) & 1U) == 0)
		;

	value = *(common.adc_base + r0);

	return value;
}


static unsigned int getVoltage(void)
{
	uint16_t value;

	/* Set VBAT_ON to 1 */
	gpio_set_pin(common.gpio_port, VBAT_ON_GPIO_PIN, true);

	usleep(10000);

	value = adc_read();

	/* Set VBAT_ON to 0 */
	gpio_set_pin(common.gpio_port, VBAT_ON_GPIO_PIN, false);

	return (ADC_VREF_MV * value) / ADC_MAX_VALUE;
}


static int dev_init(void)
{
	int res;
	oid_t dev;

	res = portCreate(&common.port);
	if (res < 0) {
		log("could not create port");
		return -1;
	}

	dev.id = DEV_VBAT_ID;
	dev.port = common.port;

	res = create_dev(&dev, DEV_VBAT_NAME);
	if (res < 0) {
		log("could not create device");
		portDestroy(common.port);
		return -1;
	}

	return 0;
}


static void dev_deinit(void)
{
	portDestroy(common.port);
	unlink("/dev/vbat");
}


static int dev_open(id_t id)
{
	int len;

	if (id != DEV_VBAT_ID) {
		return -ENOENT;
	}

	if (common.busy) {
		return -EBUSY;
	}

	len = snprintf(common.buf, sizeof(common.buf), "%u\n", getVoltage());
	if (len >= sizeof(common.buf)) {
		return -EFBIG;
	}

	common.busy = true;
	common.len = len;

	return EOK;
}


static int dev_close(id_t id)
{
	if (id != DEV_VBAT_ID) {
		return -ENOENT;
	}

	if (!common.busy) {
		return -EBADF;
	}

	common.busy = false;

	return EOK;
}


static int dev_read(id_t id, void *data, size_t size, offs_t offs)
{
	int len;

	if (id != DEV_VBAT_ID) {
		return -ENOENT;
	}

	if (!common.busy) {
		return -EBADF;
	}

	if (offs > common.len) {
		return -ERANGE;
	}

	len = min(size, common.len - offs);
	memcpy(data, common.buf + offs, len);

	return len;
}


static void msgLoop(uint32_t port)
{
	msg_t msg;
	msg_rid_t rid;
	bool running = true;

	while (running) {
		int err = msgRecv(common.port, &msg, &rid);
		if (err < 0) {
			if (err == -EINTR) {
				continue;
			}
			else {
				/* EINVAL (invalid/closed port) or ENOMEM - fatal error */
				break;
			}
		}

		switch (msg.type) {
			case mtOpen:
				msg.o.io.err = dev_open(msg.i.io.oid.id);
				break;

			case mtClose:
				msg.o.io.err = dev_close(msg.i.io.oid.id);
				break;

			case mtRead:
				msg.o.io.err = dev_read(msg.i.io.oid.id, msg.o.data, msg.o.size, msg.i.io.offs);
				break;

			case mtUnlink:
				/* Somebody is deleting our device file - exit */
				msg.o.io.err = EOK;
				running = false;
				break;

			default:
				msg.o.io.err = -EINVAL;
				break;
		}

		msgRespond(port, &msg, rid);
	}
}


static void print_usage(const char *progname)
{
	printf("Usage: %s [OPTIONS]\n", progname);
	printf("\t-r read voltage and exit\n");
	printf("\t-h print help\n");
}


int main(int argc, char **argv)
{
	int res;
	oid_t root;
	int read_and_exit = 0;

	for (;;) {
		int opt = getopt(argc, argv, "rh");
		if (opt == -1) {
			break;
		}

		switch (opt) {
			case 'r':
				read_and_exit = 1;
				break;

			case 'h':
				print_usage(argv[0]);
				return 0;

			default:
				print_usage(argv[0]);
				return 1;
		}
	}

	if (read_and_exit == 0) {
		/* Wait for the filesystem */
		while (lookup("/", NULL, &root) < 0) {
			usleep(10000);
		}
	}

	res = gpio_init();
	if (res < 0) {
		return 2;
	}

	res = adc_init();
	if (res < 0) {
		gpio_deinit();
		return 3;
	}

	if (read_and_exit == 0) {
		res = dev_init();
		if (res < 0) {
			adc_deinit();
			gpio_deinit();
			return 4;
		}

		msgLoop(common.port);

		log("Exiting!");
	}
	else {
		printf("%u\n", getVoltage());
	}

	dev_deinit();
	adc_deinit();
	gpio_deinit();

	return 0;
}
