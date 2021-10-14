#include <unistd.h>
#include <sys/msg.h>
#include <sys/platform.h>
#include <phoenix/arch/imxrt.h>

#include <imxrt-multi.h>

#include "../ad7779.h"

/* Pin config:
 * /START -> GPIO_B0_04 (ALT5 GPIO2_IO04)
 * /RESET -> GPIO_B0_05 (ALT5 GPIO2_IO05)
 * /DRDY -> GPIO_B0_14 (ALT3 SAI1_RX_SYNC)
 * DCLK -> GPIO_B0_15 (ALT3 SAI1_RX_BCLK)
 * DOUT3 -> GPIO_B0_12 (ALT3 SAI1_RX_DATA03)
 * DOUT2 -> GPIO_B0_11 (ALT3 SAI1_RX_DATA02)
 * DOUT1 -> GPIO_B0_10 (ALT3 SAI1_RX_DATA01)
 * DOUT0 -> GPIO_B1_00 (ALT3 SAI1_RX_DATA00)
 * /CS -> GPIO_B0_00 (managed by Multidrv)
 * SCLK -> GPIO_B0_03 (managed by Multidrv)
 * SDO -> GPIO_B0_01 (managed by Multidrv)
 * SDI -> GPIO_B0_02 (managed by Multidrv)
 * CLK_SEL -> GPIO_B0_06 (ALT5 GPIO2_IO06)
 */

static struct {
	oid_t multidrv;
} gpio_common;


static void gpio_setPin(int gpio, int pin, int state)
{
	msg_t msg;
	multi_i_t *imsg;

	msg.type = mtDevCtl;
	msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.data = NULL;
	msg.o.size = 0;

	imsg = (multi_i_t *)msg.i.raw;

	imsg->id = gpio;
	imsg->gpio.type = gpio_set_port;
	imsg->gpio.port.val = !!state << pin;
	imsg->gpio.port.mask = 1 << pin;

	msgSend(gpio_common.multidrv.port, &msg);
}


static void gpio_setDir(int gpio, int pin, int dir)
{
	msg_t msg;
	multi_i_t *imsg;

	msg.type = mtDevCtl;
	msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.data = NULL;
	msg.o.size = 0;

	imsg = (multi_i_t *)msg.i.raw;

	imsg->id = gpio;
	imsg->gpio.type = gpio_set_dir;
	imsg->gpio.dir.val = !!dir << pin;
	imsg->gpio.dir.mask = 1 << pin;

	msgSend(gpio_common.multidrv.port, &msg);
}


int ad7779_gpio(ad7779_gpio_t gpio, int state)
{
	switch (gpio) {
		case start:
			gpio_setPin(id_gpio2, 4, state);
			break;
		case reset:
			gpio_setPin(id_gpio2, 5, state);
			break;
		case hardreset:
			gpio_setPin(id_gpio2, 7, state);
			break;
	}

	return 0;
}


int ad7779_gpio_init(void)
{
	platformctl_t pctl = { 0 };

	if (gpio_common.multidrv.port == 0) {
		while (lookup("/dev/gpio1", NULL, &gpio_common.multidrv) < 0)
			usleep(100 * 1000);
	}

	pctl.action = pctl_set;
	pctl.type = pctl_iomux;

	pctl.iomux.sion = 0;
	pctl.iomux.mode = 5;

	/* Configure as GPIOs */

	/* /START */
	pctl.iomux.mux = pctl_mux_gpio_b0_04;
	platformctl(&pctl);
	/* /RESET */
	pctl.iomux.mux = pctl_mux_gpio_b0_05;
	platformctl(&pctl);
	/* /CLK_SEL */
	pctl.iomux.mux = pctl_mux_gpio_b0_06;
	platformctl(&pctl);
	/* Hardware reset */
	pctl.iomux.mux = pctl_mux_gpio_b0_07;
	platformctl(&pctl);

	pctl.type = pctl_iopad;
	pctl.iopad.hys = 0;
	pctl.iopad.pus = 0x3;
	pctl.iopad.pue = 0;
	pctl.iopad.pke = 1;
	pctl.iopad.ode = 0;
	pctl.iopad.speed = 2;
	pctl.iopad.dse = 1;
	pctl.iopad.sre = 0;

	/* /START */
	pctl.iopad.pad = pctl_pad_gpio_b0_04;
	platformctl(&pctl);
	/* /RESET */
	pctl.iopad.pad = pctl_pad_gpio_b0_05;
	platformctl(&pctl);
	/* /CLK_SEL */
	pctl.iopad.pad = pctl_pad_gpio_b0_06;
	platformctl(&pctl);
	/* Hardware reset */
	pctl.iopad.pad = pctl_pad_gpio_b0_07;
	platformctl(&pctl);

	/* Set states */
	gpio_setDir(id_gpio2, 4, 1);
	gpio_setDir(id_gpio2, 5, 1);
	gpio_setDir(id_gpio2, 6, 1);
	gpio_setDir(id_gpio2, 7, 1);

	gpio_setPin(id_gpio2, 4, 1);
	gpio_setPin(id_gpio2, 5, 1);
	gpio_setPin(id_gpio2, 6, 1);
	gpio_setPin(id_gpio2, 7, 0);

	return 0;
}
