#ifndef _LIBMULTI_H_
#define _LIBMULTI_H_

#include <stddef.h>
#include <stdint.h>
#include <sys/msg.h>


/* clang-format off */
typedef enum { devtype_gpio, devtype_i2c, devtype_spi, devtype_generic } devtype_t;
/* clang-format on */


typedef struct libmulti_dev {
	size_t nperipherals;
	const char *devPathPrefix;

	devtype_t devtype;
	union {
		struct {
			int (*getPort)(uint32_t port, uint32_t *val);
			int (*setPort)(uint32_t port, uint32_t mask, uint32_t val);
			int (*getDir)(uint32_t port, uint32_t *val);
			int (*setDir)(uint32_t port, uint32_t mask, uint32_t val);
		} gpio;

		struct {
			int (*busRead)(uint32_t devaddr, const void *data, size_t len);
			int (*busWrite)(uint32_t devaddr, void *data, size_t len);
			int (*regRead)(uint32_t devaddr, uint32_t regaddr, const void *data, size_t len);
			int (*regWrite)(uint32_t devaddr, uint32_t regaddr, void *data, size_t len);
		} i2c;

		struct {
			int (*configure)(uint32_t spi, uint32_t bdiv, uint32_t prescaler, uint32_t endian, uint32_t mode, uint8_t cs);
			int (*transaction)(uint32_t spi, const uint8_t *txBuff, uint8_t *rxBuff, size_t len, uint8_t cs);
		} spi;

		struct {
			int (*handleMsg)(msg_t *msg);
		} generic;
	};

	int (*init)(void);
	void (*destroy)(void);

	struct libmulti_dev *prev, *next;
	id_t baseId;
} libmulti_dev_t;


int libmulti_deviceRegister(const libmulti_dev_t *dev);


#endif
