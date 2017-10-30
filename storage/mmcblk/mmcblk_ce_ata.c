#include <dev/mmcblk/mmcblk_ce_ata.h>
#include <dev/mmcblk/mmcblk_priv.h>

int mmcblk_ce_ata_init(MmcblkCard_t *card) {
	assert(!"Not implemented");
	return 0;
}

void mmcblk_ce_ata_deinit(MmcblkCard_t *card) {
	assert(!"Not implemented");
}

int mmcblk_ce_ata_inserted(MmcblkCard_t *card) {
	assert(!"Not implemented");
	return 0;
}

int mmcblk_ce_ata_switchHighSpeed(MmcblkCard_t *card) {
	assert(!"Not implemented");
	return 0;
}

int mmcblk_ce_ata_write(MmcblkCard_t *card, offs_t offs, char *buff, unsigned int len) {
	assert(!"Not implemented");
	return 0;
}

int mmcblk_ce_ata_read(MmcblkCard_t *card, offs_t offs, char *buff, unsigned int len) {
	assert(!"Not implemented");
	return 0;
}
