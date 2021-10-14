#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/platform.h>

#include <phoenix/arch/imx6ull.h>

#include <imx6ull-gpio.h>

#include "../ad7779.h"

typedef struct {
	int port;
	int dir;
} gpio_dev_t;

struct {
	gpio_dev_t reset;
	gpio_dev_t start;
} gpio;

static int open_gpio(gpio_dev_t *gdev, int nr)
{
	unsigned tries = 100;
	char port[16];
	int res;

	if (nr > 5 && nr < 1)
		return -1;

	res = snprintf(port, sizeof(port), "/dev/gpio%d/port", nr);
	if (res < 0)
		return res;

	while ((gdev->port = open(port, O_RDWR)) < 0) {
		usleep(100 * 1000);
		if (--tries == 0)
			return -1;
	}

	res = snprintf(port, sizeof(port), "/dev/gpio%d/dir", nr);
	if (res < 0)
		return res;

	if ((gdev->dir = open(port, O_RDWR) < 0))
		return -1;

	return 0;
}

static int set_pin(int fd, int pin, int state)
{
	gpiodata_t gpiodata;

	state = !!state;
	gpiodata.w.val = state << pin;
	gpiodata.w.mask = 1 << pin;

	if (write(fd, &gpiodata, sizeof(gpiodata)) < sizeof(gpiodata))
		return -1;

	return 0;
}

int ad7779_gpio(ad7779_gpio_t type, int state)
{
	switch (type) {
		case reset:
			return set_pin(gpio.reset.port, 2, state);
		case start:
			return set_pin(gpio.start.port, 13, state);
		case hardreset:
		default:
			break;
	}

	return 0;
}

int ad7779_gpio_init(void)
{
	int res;
	platformctl_t pctl = { 0 };

	/* Configure RESET pin as GPIO */
	pctl.action = pctl_set;
	pctl.type = pctl_iomux;
	pctl.iomux.mux = pctl_mux_tamper2;
	pctl.iomux.sion = 0;
	pctl.iomux.mode = 5;
	platformctl(&pctl);

	pctl.action = pctl_set;
	pctl.type = pctl_iopad;
	pctl.iopad.pad = pctl_pad_tamper2;
	pctl.iopad.hys = 0;
	pctl.iopad.pus = 0;
	pctl.iopad.pue = 0;
	pctl.iopad.pke = 1;
	pctl.iopad.ode = 0;
	pctl.iopad.speed = 0b10;
	pctl.iopad.dse = 1;
	pctl.iopad.sre = 0;
	platformctl(&pctl);

	/* Configure START pin as GPIO */
	pctl.action = pctl_set;
	pctl.type = pctl_iomux;
	pctl.iomux.mux = pctl_mux_lcd_d8;
	pctl.iomux.sion = 0;
	pctl.iomux.mode = 5;
	platformctl(&pctl);

	pctl.action = pctl_set;
	pctl.type = pctl_iopad;
	pctl.iopad.pad = pctl_pad_lcd_d8;
	pctl.iopad.hys = 0;
	pctl.iopad.pus = 0;
	pctl.iopad.pue = 0;
	pctl.iopad.pke = 1;
	pctl.iopad.ode = 0;
	pctl.iopad.speed = 0b10;
	pctl.iopad.dse = 1;
	pctl.iopad.sre = 0;
	platformctl(&pctl);


	/* Init RESET and START ports */
	if ((res = open_gpio(&gpio.reset, 5)) < 0)
		return res;

	if ((res = open_gpio(&gpio.start, 3)) < 0)
		return res;

	/* Set RESET as output */
	if ((res = set_pin(gpio.reset.port, 2, 1)))
		return res;

	if ((res = set_pin(gpio.reset.dir, 2, 1)))
		return res;

	/* Set START as output */
	if ((res = set_pin(gpio.reset.port, 13, 1)))
		return res;

	if ((res = set_pin(gpio.reset.dir, 13, 1)))
		return res;

	return 0;
}