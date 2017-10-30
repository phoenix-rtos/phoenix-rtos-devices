#ifndef MMCBLK_SDHIO_H
#define MMCBLK_SDHIO_H

#include <dev/mmcblk/mmcblk.h>

int mmcblk_sdio_init(MmcblkCard_t *card);
void mmcblk_sdio_deinit(MmcblkCard_t *card);
int mmcblk_sdio_inserted(MmcblkCard_t *card);
int mmcblk_sdio_switchHighSpeed(MmcblkCard_t *card);

int mmcblk_sdio_write(MmcblkCard_t *card, offs_t offs, char *buff, unsigned int len);
int mmcblk_sdio_read(MmcblkCard_t *card, offs_t offs, char *buff, unsigned int len);

#endif
