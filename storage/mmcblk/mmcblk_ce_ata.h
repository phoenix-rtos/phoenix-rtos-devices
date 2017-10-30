#ifndef MMCBLK_CE_ATA_H
#define MMCBLK_CE_ATA_H


#include <dev/mmcblk/mmcblk.h>


int mmcblk_ce_ata_init(MmcblkCard_t *card);
void mmcblk_ce_ata_deinit(MmcblkCard_t *card);
int mmcblk_ce_ata_inserted(MmcblkCard_t *card);
int mmcblk_ce_ata_switchHighSpeed(MmcblkCard_t *card);

int mmcblk_ce_ata_write(MmcblkCard_t *card, offs_t offs, char *buff, unsigned int len);
int mmcblk_ce_ata_read(MmcblkCard_t *card, offs_t offs, char *buff, unsigned int len);


#endif
