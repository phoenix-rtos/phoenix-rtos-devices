#include <unistd.h>
#include <sys/msg.h>

#include <phoenix/arch/imxrt.h>
#include <imxrt-multi.h>

#include "../ad7779.h"

static oid_t spi_driver;


int spi_exchange(uint8_t *buff, uint8_t len)
{
	int res;
	msg_t msg = { 0 };
	multi_i_t *imsg = (multi_i_t *)msg.i.raw;
	multi_o_t *omsg = (multi_o_t *)msg.o.raw;

	msg.type = mtDevCtl;
	msg.i.data = buff;
	msg.i.size = len;
	msg.o.data = buff;
	msg.o.size = len;

	imsg->id = id_spi4;
	imsg->spi.type = spi_transaction;
	imsg->spi.transaction.cs = 0;
	imsg->spi.transaction.frameSize = len;

	if ((res = msgSend(spi_driver.port, &msg)) < 0)
		return res;

	return omsg->err;
}


int spi_init(void)
{
	int res;
	msg_t msg;
	multi_i_t *imsg = (multi_i_t *)msg.i.raw;
	multi_o_t *omsg = (multi_o_t *)msg.o.raw;

	if (spi_driver.port == 0) {
		while (lookup("/dev/spi4", NULL, &spi_driver) < 0)
			usleep(100 * 1000);
	}

	msg.type = mtDevCtl;
	msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.data = NULL;
	msg.o.size = 0;

	imsg->id = id_spi4;
	imsg->spi.type = spi_config;
	imsg->spi.config.cs = 0;
	imsg->spi.config.mode = spi_mode_0;
	imsg->spi.config.endian = spi_msb;
	imsg->spi.config.sckDiv = 4;
	imsg->spi.config.prescaler = 3;

	if ((res = msgSend(spi_driver.port, &msg)) < 0)
		return res;

	return omsg->err;
}
