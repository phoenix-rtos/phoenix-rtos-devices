#include <errno.h>
#include <sys/msg.h>

#include "gpio.h"
#include "libmulti-internal.h"


static struct {
	uint32_t msgport;
	id_t baseId;
} common;


static inline id_t libmulti_gpioPortToId(uint32_t port)
{
	return common.baseId + port - 1;
}


static int libmulti_gpioSendMsg(gpio_op_t op, uint32_t port, uint32_t mask, uint32_t *val)
{
	int err;
	msg_t msg = {
		.type = mtDevCtl,
		.oid = { .port = common.msgport, .id = libmulti_gpioPortToId(port) },
	};
	multi_msg_t *multi = (multi_msg_t *)&msg.i.raw;

	multi->gpio.op = op;
	multi->gpio.port = port;
	multi->gpio.mask = mask;
	multi->gpio.val = *val;

	err = msgSend(common.msgport, &msg);
	if (err < 0) {
		return err;
	}
	if (msg.o.err < 0) {
		return msg.o.err;
	}

	if (op == gpio_opGetPort || op == gpio_opGetDir) {
		*val = multi->gpio.val;
	}

	return 0;
}


int gpio_getPort(uint32_t port, uint32_t *val)
{
	return libmulti_gpioSendMsg(gpio_opGetPort, port, 0xffffffffU, val);
}


int gpio_setPort(uint32_t port, uint32_t mask, uint32_t val)
{
	return libmulti_gpioSendMsg(gpio_opSetPort, port, mask, &val);
}


int gpio_getPin(uint32_t port, uint32_t pin, uint32_t *val)
{
	if (pin > 31) {
		return -EINVAL;
	}

	int ret = libmulti_gpioSendMsg(gpio_opGetPort, port, pin, val);
	if (ret < 0) {
		return ret;
	}

	*val = ((*val & pin) != 0) ? 1 : 0;

	return EOK;
}


int gpio_setPin(uint32_t port, uint32_t pin, uint32_t val)
{
	if (pin > 31) {
		return -EINVAL;
	}

	return libmulti_gpioSendMsg(gpio_opSetPort, port, (1U << pin), &val);
}


int gpio_getDir(uint32_t port, uint32_t *dir)
{
	return libmulti_gpioSendMsg(gpio_opGetDir, port, 0xffffffffU, dir);
}


int gpio_setDir(uint32_t port, uint32_t pin, uint32_t dir)
{
	return libmulti_gpioSendMsg(gpio_opSetDir, port, pin, &dir);
}


void gpio_msgInit(uint32_t msgport, id_t baseId)
{
	common.msgport = msgport;
	common.baseId = baseId;
}
