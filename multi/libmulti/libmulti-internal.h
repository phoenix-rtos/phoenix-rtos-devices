#include <stddef.h>
#include <stdint.h>
#include <sys/msg.h>

#include "libmulti.h"


/* clang-format off */
typedef enum { gpio_opGetPort, gpio_opSetPort, gpio_opGetDir, gpio_opSetDir } gpio_op_t;


typedef enum { i2c_opBusWrite, i2c_opBusRead, i2c_opRegWrite, i2c_opRegRead } i2c_op_t;


typedef enum { spi_opConfigure, spi_opTransaction } spi_op_t;
/* clang-format on */


typedef union {
	struct {
		gpio_op_t op;
		uint32_t port, mask, val;
	} gpio;

	struct {
		i2c_op_t op;
		uint32_t devaddr;
		uint32_t regaddr;
		union {
			const void *dataWrite; /* points to msg.i.data */
			void *dataRead;        /* points to msg.o.data */
		};
		size_t len;
	} i2c;

	struct {
		uint32_t spi;
		uint8_t cs;
		union {
			struct {
				uint32_t bdiv;
				uint32_t prescaler;
				uint32_t endian;
				uint32_t mode;
			} config;

			struct {
				const uint8_t *txBuff; /* points to msg.i.data */
				uint8_t *rxBuff;       /* points to msg.o.data */
				size_t len;
			} transaction;
		};
	} spi;
} multi_msg_t;
