#include <unistd.h>
#include <sys/msg.h>

#include <imx6ull-ecspi.h>

#include "../ad7779.h"


int spi_exchange(uint8_t *buff, uint8_t len)
{
	return ecspi_exchangeBusy(ecspi1, buff, buff, len);
}


int spi_init(void)
{
	int res;

	/* Initialize ecspi1 with channel 0 */
	if ((res = ecspi_init(ecspi1, 1)) < 0) {
		log_error("failed to initialize ecspi");
		return res;
	}

	if ((res = ecspi_setClockDiv(ecspi1, 0xF, 0x4))) {
		log_error("failed to divide ecspi clock");
		return res;
	}

	return 0;
}
