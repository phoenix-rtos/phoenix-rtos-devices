#include <dev/mmcblk/mmcblk_mmc.h>
#include <dev/mmcblk/mmcblk_priv.h>

int mmcblk_mmc_init(MmcblkCard_t *card) {
	assert(!"Not implemented");
	return 0;
}

void mmcblk_mmc_deinit(MmcblkCard_t *card) {
	assert(!"Not implemented");
}

int mmcblk_mmc_inserted(MmcblkCard_t *card) {
	assert(!"Not implemented");
	return 0;
}

int mmcblk_mmc_switchHighSpeed(MmcblkCard_t *card) {
	assert(!"Not implemented");
	return 0;
}

int mmcblk_mmc_write(MmcblkCard_t *card, offs_t offs, char *buff, unsigned int len) {
	assert(!"Not implemented");
	return 0;
}

int mmcblk_mmc_read(MmcblkCard_t *card, offs_t offs, char *buff, unsigned int len) {
	assert(!"Not implemented");
	return 0;
}
