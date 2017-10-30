/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * CT69000 video driver
 *
 * Initialization derived from XFree86 driver. Other code derived from IMMOS graph library
 *
 * Copyright 2012 Phoenix Systems
 * Copyright 2006, 2008 Pawel Pisarczyk
 * Copyright 2001-2003 IMMOS (R. Jurkiewicz, P. Pisarczyk)
 * Authors: Rafal Jurkiewicz, Pawel Pisarczyk
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _GRAPH_ASILIANT_H_
#define _GRAPH_ASILIANT_H_


typedef struct {
    u8 msr;
    u8 fcr;
    u8 xr02;
    u8 xr03;
    u8 xr33;
    u8 xr54;
    u8 fr03;
    int clock;
    int fpclock;
} chips_clock_t;


typedef struct {
	u8 xr[0xff];
	u8 cr[0x80];
	u8 fr[0x80];
	u8 mr[0x80];

	u8 sr[5];
	u8 gr[9];
	u8 vcr[25];
	u8 ar[21];

  chips_clock_t clock;
} chips_regs_t;


typedef struct {
    u8 xr02;
    u8 xr03;
    u8 xr14;
    u8 xr15;
    u8 vgaIOBaseFlag;
} chips_hack_t;


typedef struct _chips_t {
  void *fbaddr;
  void *fbbase;
  unsigned int fbsize;

/*  chips_clock_t	savedclock; */
  u8 clocktype;
  
  u8 crtclk[4];
  int crtclkidx;
  u8 fpclk[4];
  int fpclkidx;
  u8 fpclkmod;

 chips_regs_t savedregs;
/*  chips_regs_t savedregs2;
  vga_regs_t vgaregs; */
} chips_t;


extern int asiliant_init(screen_t *screen);


#endif
