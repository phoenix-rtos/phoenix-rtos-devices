#include <errno.h>
#include <stdbool.h>
#include <string.h>
#include <sys/minmax.h>
#include <sys/msg.h>
#include <sys/list.h>
#include <sys/threads.h>
#include <posix/utils.h>

#include "libmulti.h"
#include "libmulti-internal.h"
#include "libmulti-generic.h"

#include <board_config.h>


static struct {
	libmulti_dev_t *gpio, *i2c, *spi, *generic;

	// TODO: multiple message threads defined in config.h/board_config.h (need multiple stacks)
	uint32_t msgport;
	uint8_t stack[1024];
} common;


static inline bool libmulti_idIsDev(const libmulti_dev_t *dev, id_t id)
{
	return id >= dev->baseId && id < dev->baseId + dev->nperipherals;
}


int libmulti_registerDevice(libmulti_dev_t *dev)
{
	switch (dev->devtype) {
		case devtype_gpio:
			if (common.gpio != NULL) {
				return -1;
			}
			common.gpio = dev;
			break;

		case devtype_i2c:
			if (common.i2c != NULL) {
				return -1;
			}
			common.i2c = dev;
			break;

		case devtype_spi:
			if (common.spi != NULL) {
				return -1;
			}
			common.spi = dev;
			break;

		default:
			// TODO: do we need to traverse the whole list to search if dev was not added before?
			for (libmulti_dev_t *generic = common.generic; generic != NULL; generic = generic->next) {
				if (dev == generic) {
					return -1;
				}
			}
			LIST_ADD(&common.generic, dev);
			break;
	}

	return 0;
}


static inline int libmulti_id2devtype(id_t id)
{
	if (common.gpio != NULL && libmulti_idIsDev(common.gpio, id)) {
		return devtype_gpio;
	}

	if (common.i2c != NULL && libmulti_idIsDev(common.i2c, id)) {
		return devtype_i2c;
	}

	if (common.spi != NULL && libmulti_idIsDev(common.spi, id)) {
		return devtype_spi;
	}

	/* assume every other id is generic, the caller must check if this is true */
	return devtype_generic;
}


// TODO: device locking?
// TODO: implement shell read/write interface (libmulti_handleGpioTextMode(msg_t *msg)?)
static int libmulti_handleGpio(multi_msg_t *multi)
{
	switch (multi->gpio.op) {
		case gpio_opGetPort:
			return common.gpio->gpio.getPort(multi->gpio.port, &multi->gpio.val);

		case gpio_opSetPort:
			return common.gpio->gpio.setPort(multi->gpio.port, multi->gpio.mask, multi->gpio.val);

		case gpio_opGetDir:
			return common.gpio->gpio.getDir(multi->gpio.port, &multi->gpio.val);

		case gpio_opSetDir:
			return common.gpio->gpio.setDir(multi->gpio.port, multi->gpio.mask, multi->gpio.val);

		default:
			return -ENOSYS;
	}
}


// TODO: shell interface (handle read/write)
static void libmulti_msgthread(void *arg)
{
	msg_t msg;
	msg_rid_t rid;
	multi_msg_t *multi;
	int err;

	for (;;) {
		if (msgRecv(common.msgport, &msg, &rid) < 0) {
			break;
		}

		multi = (void *)msg.i.raw;

		// TODO: implement device-specific read/write
		// TODO: could be implemented in libmulti-<devtype>.c
		// TODO: the plan:
		// TODO: - if type == mtDevCtl: handle as multi_msg_t
		// TODO: - else: handle as shell interface (open/close return EOK (TODO: device locking?), custom read/write)
		// TODO: maybe differentiate msg type in libmulti_handle<Devtype>?
		switch (libmulti_id2devtype(msg.oid.id)) {
			case devtype_gpio:
				msg.o.err = libmulti_handleGpio(multi);
				break;

			case devtype_i2c:
				// msg.o.err = libmulti_handleI2c(multi);
				break;

			case devtype_spi:
				// msg.o.err = libmulti_handleSpi(multi);
				break;

			case devtype_generic:
				msg.o.err = -ENOSYS;
				for (libmulti_dev_t *generic = common.generic; generic != NULL; generic = generic->next) {
					if (msg.oid.id >= generic->baseId && msg.oid.id < generic->baseId + generic->nperipherals) {
						msg.o.err = generic->generic.handleMsg(&msg);
						break;
					}
				}
				break;

			default:
				msg.o.err = -ENOSYS;
				break;
		}

		memcpy(msg.o.raw, multi, sizeof(*multi));
		msgRespond(common.msgport, &msg, rid);
	}
}


static int libmulti_createDevs(libmulti_dev_t *ctx)
{
	char path[32];
	oid_t oid = { .port = common.msgport };
	id_t id = ctx->baseId;

	for (int i = 0; i < ctx->nperipherals; i++) {
		oid.id = id;
		id++;
		// TODO: error handling
		sprintf(path, "%s%d", ctx->devPathPrefix, i);
		create_dev(&oid, path);
	}

	return 0;
}


void libmulti_destroy(void)
{
#if LIBMULTI_HAS_GPIO_DRV
	gpio_destroy(&common.gpio);
#endif
#if LIBMULTI_HAS_I2C_DRV
	i2c_destroy(&common.i2c);
#endif
#if LIBMULTI_HAS_SPI_DRV
	spi_destroy(&common.spi);
#endif
#if LIBMULTI_N_GENERIC_DRVS > 0
	for (size_t i = 0; i < LIBMULTI_N_GENERIC_DRVS; i++) {
		if (common.generic[i].driver != NULL) {
			common.generic[i].driver->destroy(&common.generic[i].ctx);
		}
	}
#endif
}


// TODO: GPIO driver and other drivers provided in the main driver
// TODO: handle errors
int libmulti_init(void)
{
	id_t currentId = 0;
	oid_t oid = { 0 };

	portCreate(&oid.port);
	common.msgport = oid.port;

	if (common.gpio != NULL) {
		if (gpio_init(&common.gpio) < 0) {
			return -1;
		}
		common.gpio.baseId = currentId;

		gpio_msgInit(common.msgport, common.gpio.baseId);

		// TODO: error handling
		libmulti_createDevs(&common.gpio);
		currentId += common.gpio.nperipherals;
	}

	if (i2c_init(&common.i2c) < 0) {
		return -1;
	}
	common.i2c.baseId = currentId;

	// TODO: implement i2c_msgInit
	i2c_msgInit(common.msgport, common.i2c.baseId);

	// TODO: error handling
	libmulti_createDevs(&common.i2c);
	currentId += common.i2c.nperipherals;
#endif

#if LIBMULTI_HAS_SPI_DRV
	if (spi_init(&common.spi) < 0) {
		return -1;
	}
	common.spi.baseId = currentId;

	// TODO: implement spi_msgInit
	spi_msgInit(common.msgport, common.spi.baseId);

	// TODO: error handling
	libmulti_createDevs(&common.spi);
	currentId += common.spi.nperipherals;
#endif

#if LIBMULTI_N_GENERIC_DRVS > 0
	for (size_t i = 0; i < LIBMULTI_N_GENERIC_DRVS; i++) {
		if (common.generic[i].driver->init(&common.generic[i].ctx) < 0) {
			return -1;
		}

		// TODO: generic_msgInit?
		generic_msgInit(common.msgport, currentId);

		libmulti_createDevs(&common.generic[i].ctx);
		currentId += common.generic[i].ctx.nperipherals;
	}
#endif

	// TODO: begin more threads (LIBMULTI_N_MSGTHREADS, configurable from board_config.h)
	// TODO: priority and stack size configurable from board_config.h
	// TODO: add error handling
	beginthread(libmulti_msgthread, 3, common.stack, sizeof(common.stack), NULL);

	return 0;
}
