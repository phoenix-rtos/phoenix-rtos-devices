/**
 * Enhanced Serial Audio Interface (ESAI) implementation for Vybrid
 *
 * Phoenix-RTOS
 * 
 * Operating system kernel
 *
 * @file esai.c
 *
 * @copyright 2014 Phoenix Systems
 *
 * @author Pawel Tryfon<pawel.tryfon@phoesys.com>
 * 
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _DEV_VYBRID_ESAI_H_
#define _DEV_VYBRID_ESAI_H_

int _esai_init(void);

void esai_readBufStart(void);

int esai_readBuf(int samples, s32 **buf);
int esai_available(void);

#endif
