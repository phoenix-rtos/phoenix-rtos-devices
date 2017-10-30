#ifndef MMCBLK_MMC_H
#define MMCBLK_MMC_H

#include <dev/mmcblk/mmcblk.h>

int mmcblk_mmc_init(MmcblkCard_t *card);
void mmcblk_mmc_deinit(MmcblkCard_t *card);
int mmcblk_mmc_inserted(MmcblkCard_t *card);
int mmcblk_mmc_switchHighSpeed(MmcblkCard_t *card);

int mmcblk_mmc_write(MmcblkCard_t *card, offs_t offs, char *buff, unsigned int len);
int mmcblk_mmc_read(MmcblkCard_t *card, offs_t offs, char *buff, unsigned int len);



#endif
