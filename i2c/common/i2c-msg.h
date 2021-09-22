/* i2c internal message API, to be used only by the implementers */
#ifndef _PHOENIX_I2C_MSG_H
#define _PHOENIX_I2C_MSG_H

#include <stdint.h>

enum {
	i2c_devctl_bus_write, /* input params: dev_addr, *data, len */
	i2c_devctl_bus_read,  /* input params: dev_addr, *data, len */
	i2c_devctl_reg_read   /* input params: dev_addr, reg_addr, *data, len */
};

typedef struct {
	union {
		struct {
			unsigned int type;
			uint8_t dev_addr;
			uint8_t reg_addr;
		} i;
		struct {
			int err;
		} o;
	};
} __attribute__((packed)) i2c_devctl_t;

#endif /* _PHOENIX_I2C_MSG_H */
