#include <dev/mmcblk/mmcblk_sdio.h>
#include <dev/mmcblk/mmcblk_priv.h>

int mmcblk_sdio_init(MmcblkCard_t *card) {
	assert(!"Not implemented");
	return 0;
}

void mmcblk_sdio_deinit(MmcblkCard_t *card) {
	assert(!"Not implemented");
}

int mmcblk_sdio_inserted(MmcblkCard_t *card) {
	assert(!"Not implemented");
	return 0;
}

int mmcblk_sdio_switchHighSpeed(MmcblkCard_t *card) {
	assert(!"Not implemented");
	return 0;
}

int mmcblk_sdio_write(MmcblkCard_t *card, offs_t offs, char *buff, unsigned int len) {
	assert(!"Not implemented");
	return 0;
}

int mmcblk_sdio_read(MmcblkCard_t *card, offs_t offs, char *buff, unsigned int len) {
	assert(!"Not implemented");
	return 0;
}
