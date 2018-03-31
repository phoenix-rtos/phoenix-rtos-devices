#ifndef MMCBLK_SDHC_H
#define MMCBLK_SDHC_H

#include <dev/mmcblk/mmcblk.h>

#define MMCBLK_SDHC_MAX_BAUDRATE 33000000UL

extern void mmcblk_sdhc_init(void *cardPtr);
extern void mmcblk_sdhc_reset(void *cardPtr);
extern int mmcblk_sdhc_sendCommand(void *cardPtr, u32 cmd, u32 cmd_arg, s32 block_num, u16 block_size, void *admaDT);
extern MmcblkResponse_t mmcblk_sdhc_waitForResponse(void *cardPtr, int cmd);
extern int mmcblk_sdhc_transferWait(void *cardPtr);
extern int mmcblk_sdhc_waitBusy(void *cardPtr);
extern void *mmcblk_sdhc_setupDMA(void *cardPtr, void *bufptr, s32 bufsize, FreePtr **fp, char *htBuf);
extern void mmcblk_sdhc_freeDMA(FreePtr *fp);
extern int mmcblk_sdhc_switchHighSpeed(void *cardPtr, u32 baudrate);
extern int mmcblk_sdhc_setupBaudRate(void *cardPtr, u32 baudrate);
extern int mmcblk_sdhc_setupBusWidth(void *cardPtr, MmcblkBusWidth_t width);
extern int mmcblk_sdhc_setupEndian(void *cardPtr, MmcblkEndian_t endian);

#endif
