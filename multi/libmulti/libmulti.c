#include <errno.h>
#include <string.h>
#include <sys/minmax.h>
#include <sys/msg.h>
#include <sys/threads.h>
#include <posix/utils.h>

#include "libmulti.h"
#include "libmulti-internal.h"

#include "libmulti-gpio.h"
#include "libmulti-i2c.h"
#include "libmulti-spi.h"
#include "libmulti-generic.h"

#include "include/gpio.h"

#include "config.h"

// TODO: disable all drivers on default
#ifndef LIBMULTI_HAS_GPIO_DRV
#define LIBMULTI_HAS_GPIO_DRV 0
#elif LIBMULTI_HAS_GPIO_DRV != 0 && LIBMULTI_HAS_GPIO_DRV != 1
#error "LIBMULTI_HAS_GPIO_DRV must be either 0 or 1"
#endif

#ifndef LIBMULTI_HAS_I2C_DRV
#define LIBMULTI_HAS_I2C_DRV 0
#elif LIBMULTI_HAS_I2C_DRV != 0 && LIBMULTI_HAS_I2C_DRV != 1
#error "LIBMULTI_HAS_I2C_DRV must be either 0 or 1"
#endif

#ifndef LIBMULTI_HAS_SPI_DRV
#define LIBMULTI_HAS_SPI_DRV 0
#elif LIBMULTI_HAS_SPI_DRV != 0 && LIBMULTI_HAS_SPI_DRV != 1
#error "LIBMULTI_HAS_SPI_DRV must be either 0 or 1"
#endif

#ifndef LIBMULTI_N_GENERIC_DRVS
#define LIBMULTI_N_GENERIC_DRVS 0
#elif LIBMULTI_N_GENERIC_DRVS < 0
#error "LIBMULTI_N_GENERIC_DRVS must be non-negative"
#endif

#define N_DRVS (LIBMULTI_HAS_GPIO_DRV + \
		LIBMULTI_HAS_I2C_DRV + \
		LIBMULTI_HAS_SPI_DRV + \
		LIBMULTI_N_GENERIC_DRVS)

#if N_DRVS == 0
#error "At least one driver must be enabled in libmulti"
#endif


static struct {
#if LIBMULTI_HAS_GPIO_DRV
	device_ctx_t gpio;
#endif
#if LIBMULTI_HAS_I2C_DRV
	device_ctx_t i2c;
#endif
#if LIBMULTI_HAS_SPI_DRV
	device_ctx_t spi;
#endif
#if LIBMULTI_N_GENERIC_DRVS > 0
	struct {
		device_ctx_t ctx;
		const generic_driver_t *driver;
	} generic[LIBMULTI_N_GENERIC_DRVS];
#endif

	uint32_t msgport;

	// TODO: multiple message threads defined in config.h/board_config.h (need multiple stacks)
	uint8_t stack[1024];
} common;


#if LIBMULTI_N_GENERIC_DRVS > 0
int libmulti_registerGeneric(generic_driver_t *ctx)
{
	for (size_t i = 0; i < LIBMULTI_N_GENERIC_DRVS; i++) {
		if (common.generic[i].driver == NULL) {
			common.generic[i].driver = ctx;
			return 0;
		}
	}
	return -1;
}
#endif


/* caller could assume this never fails as  */
static inline int libmulti_id2devtype(id_t id)
{
#if LIBMULTI_HAS_GPIO_DRV
	if (id < common.gpio.baseId + common.gpio.nperipherals) {
		return devtype_gpio;
	}
#endif
#if LIBMULTI_HAS_I2C_DRV
	if (id < common.i2c.baseId + common.i2c.nperipherals) {
		return devtype_i2c;
	}
#endif
#if LIBMULTI_HAS_SPI_DRV
	if (id < common.spi.baseId + common.spi.nperipherals) {
		return devtype_spi;
	}
#endif
#if LIBMULTI_N_GENERIC_DRVS > 0
	for (size_t i = 0; i < LIBMULTI_N_GENERIC_DRVS; i++) {
		if (id < common.generic[i].ctx.baseId + common.generic[i].ctx.nperipherals) {
			return devtype_generic;
		}
	}
#endif

	/* should not happen */
	return -1;
}


// TODO: shell interface (handle read/write)
static void libmulti_msgthread(void *arg)
{
	msg_t msg;
	msg_rid_t rid;
	multi_msg_t *multi;

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
#if LIBMULTI_HAS_GPIO_DRV
			case devtype_gpio:
				msg.o.err = libmulti_handleGpio(multi);
				break;
#endif

#if LIBMULTI_HAS_I2C_DRV
			case devtype_i2c:
				msg.o.err = libmulti_handleI2c(multi);
				break;
#endif

#if LIBMULTI_HAS_SPI_DRV
			case devtype_spi:
				msg.o.err = libmulti_handleSpi(multi);
				break;
#endif

#if LIBMULTI_N_GENERIC_DRVS > 0
			case devtype_generic:
				/* should not happen, as libmulti_id2devtype */
				if (msg.oid.id < common.generic[0].ctx.baseId) {
					msg.o.err = -ENOSYS;
					break;
				}
				for (size_t i = 0; i < LIBMULTI_N_GENERIC_DRVS; i++) {
					if (msg.oid.id < common.generic[i].ctx.baseId + common.generic[i].ctx.nperipherals) {
						msg.o.err = common.generic[i].driver->handleMsg(&msg);
						break;
					}
				}
				break;
#endif

			default:
				msg.o.err = -ENOSYS;
				break;
		}

		memcpy(msg.o.raw, multi, sizeof(*multi));
		msgRespond(common.msgport, &msg, rid);
	}
}


static int libmulti_createDevs(device_ctx_t *ctx)
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


static void libmulti_destroy(void)
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

#if LIBMULTI_HAS_GPIO_DRV
	if (gpio_init(&common.gpio) < 0) {
		return -1;
	}
	common.gpio.baseId = currentId;

	// TODO: error handling
	libmulti_createDevs(&common.gpio);
	currentId += common.gpio.nperipherals;
#endif

#if LIBMULTI_HAS_I2C_DRV
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

	// TODO: begin threads (LIBMULTI_N_MSGTHREADS, configurable from board_config.h)
	// TODO: priority and stack size configurable from board_config.h
	// TODO: error handling
	beginthread(libmulti_msgthread, 3, common.stack, sizeof(common.stack), NULL);

	return 0;
}
