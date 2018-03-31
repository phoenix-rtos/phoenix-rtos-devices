/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * Graph device driver for Asilliant CT69000
 *
 * Copyright 2012 Phoenix Systems
 * Copyright 2006, 2008 Pawel Pisarczyk
 * Copyright 2001-2003 IMMOS (R. Jurkiewicz, P. Pisarczyk)
 * Authors: Rafal Jurkiewicz, Pawel Pisarczyk
 *
 * Initialization derived from XFree86 driver
 * Other code derived from IMMOS graph library
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <config.h>
#include <hal/if.h>
#include <main/if.h>
#include <vm/if.h>
#include <dev/if.h>
#include <graph/if.h>

#include <dev/chips/asiliant.h>


typedef struct _color_t {
	unsigned int length;
	unsigned int offset;
} color_t;


typedef struct _mode_t {
	unsigned int xres;
	unsigned int yres;
	unsigned int xres_virtual;
	unsigned int yres_virtual;
	u8 bits_per_pixel;

	color_t red;
	color_t green;
	color_t blue;
  
	unsigned int pixclock;
  
	unsigned int left_margin;
	unsigned int right_margin;
	unsigned int upper_margin;
	unsigned int lower_margin;

	unsigned int hsync_len;
	unsigned int vsync_len;
} mode_t;




/*(II) I810(0): Modeline "640x480"   31.50  640 656 720 840  480 481 484 500 -hsync -vsync
(II) I810(0): Modeline "640x480"   25.20  640 656 752 800  480 490 492 525 -hsync -vsync
(II) I810(0): Modeline "720x400"   28.32  720 738 846 900  400 412 414 449 -hsync +vsync
(II) I810(0): Modeline "1024x768"   65.00  1024 1048 1184 1344  768 771 777 806 -hsync -vsync
(II) I810(0): Modeline "800x600"   49.50  800 816 896 1056  600 601 604 625 +hsync +vsync
(II) I810(0): Modeline "640x480"   35.00  640 664 728 816  480 483 487 507 -hsync +vsync
(II) I810(0): Modeline "800x600"   56.75  800 848 928 1056  600 603 607 633 -hsync +vsync
(II) I810(0): Modeline "800x600"   56.25  800 832 896 1048  600 601 604 631 +hsync +vsync
(II) I810(0): Modeline "1024x768"   65.00  1024 1048 1184 1344  768 771 777 806 -hsync -vsync*/

mode_t modes[] = {
	{ 640, 480, 640, 480, 8, { 0, 0 }, { 0, 0 }, { 0, 0 }, 39721, 40, 24, 32, 11, 96, 2 },
	{ 640, 480, 640, 480, 16, { 0, 5 }, { 0, 5 }, { 0, 6 }, 39721, 40, 24, 32, 11, 96, 2 },
	{ 800, 600, 800, 600, 16, { 0, 5 }, { 0, 5 }, { 0, 6 }, 27778, 64, 24, 22, 1, 72, 2 },
	{ 1024, 768, 1024, 768, 16, { 0, 5 }, { 0, 5 }, { 0, 5 }, 15384, 168, 8, 29, 3, 144, 4 },
	{ 1152, 864, 1152, 864, 16, { 0, 5 }, { 0, 5 }, { 0, 5 }, 12004, 200, 64, 32, 16, 80, 4 } 
};


/* Built in clock of the 69030 */
const unsigned Fref = 14318180;


struct chips_init_reg {
	unsigned char addr;
	unsigned char data;
	u8 old;
};


#define N_ELTS(x)	(sizeof(x) / sizeof(x[0]))


static struct chips_init_reg chips_init_sr[] =
{
	{0x00, 0x03},		/* Reset register */
	{0x01, 0x01},		/* Clocking mode */
	{0x02, 0x0f},		/* Plane mask */
	{0x04, 0x0e}		/* Memory mode */
};

static struct chips_init_reg chips_init_gr[] =
{
	{0x03, 0x00},		/* Data rotate */
	{0x05, 0x00},		/* Graphics mode */
	{0x06, 0x01},		/* Miscellaneous */
	{0x08, 0x00}		/* Bit mask */
};

__attribute__((unused))
static struct chips_init_reg chips_init_ar[] =
{
	{0x10, 0x01},		/* Mode control */
	{0x11, 0x00},		/* Overscan */
	{0x12, 0x0f},		/* Memory plane enable */
	{0x13, 0x00}		/* Horizontal pixel panning */
};

static struct chips_init_reg chips_init_cr[] =
{
	{0x0c, 0x00},		/* Start address high */
	{0x0d, 0x00},		/* Start address low */
	{0x40, 0x00},		/* Extended Start Address */
	{0x41, 0x00},		/* Extended Start Address */
	{0x14, 0x00},		/* Underline location */
	{0x17, 0xe3},		/* CRT mode control */
	{0x70, 0x00}		/* Interlace control */
};

static struct chips_init_reg chips_init_fr[] =
{
	{0x01, 0x02},
	{0x03, 0x08},
	{0x08, 0xcc},
	{0x0a, 0x08},
	{0x18, 0x00},
	{0x1e, 0x80},
	{0x40, 0x83},
	{0x41, 0x00},
	{0x48, 0x13},
	{0x4d, 0x60},
	{0x4e, 0x0f},

	{0x0b, 0x01},

	{0x21, 0x51},
	{0x22, 0x1d},
	{0x23, 0x5f},
	{0x20, 0x4f},
	{0x34, 0x00},
	{0x24, 0x51},
	{0x25, 0x00},
	{0x27, 0x0b},
	{0x26, 0x00},
	{0x37, 0x80},
	{0x33, 0x0b},
	{0x35, 0x11},
	{0x36, 0x02},
	{0x31, 0xea},
	{0x32, 0x0c},
	{0x30, 0xdf},
	{0x10, 0x0c},
	{0x11, 0xe0},
	{0x12, 0x50},
	{0x13, 0x00},
	{0x16, 0x03},
	{0x17, 0xbd},
	{0x1a, 0x00},
};


static struct chips_init_reg chips_init_xr[] =
{
	{0xce, 0x00},	/* set default memory clock */
	{0xcc, 200 },	/* MCLK ratio M */
	{0xcd, 18  },	/* MCLK ratio N */
	{0xce, 0x90},	/* MCLK divisor = 2 */
	{0xc4, 209 },
	{0xc5, 118 },
	{0xc7, 32 },
	{0xcf, 0x06},

	{0x09, 0x01},		/* IO Control - CRT controller extensions */

	{0x0a, 0x02},		/* Frame buffer mapping */
	{0x0b, 0x01},		/* PCI burst write */
	{0x40, 0x03},		/* Memory access control */

	{0x80, 0x82},		/* Pixel pipeline configuration 0 */
	{0x81, 0x12},		/* Pixel pipeline configuration 1 */
	{0x82, 0x08},		/* Pixel pipeline configuration 2 */

	{0xd0, 0x0f},
	{0xd1, 0x01},
};


static void usleep(int t)
{
	unsigned int i;	
	for (i = 0; i < 1000000000; i++);
}


static u8 _asiliant_getmsr(chips_t *chips)
{
	return hal_inb((void *)0x3cc);
}


static void _asiliant_setmsr(chips_t *chips, u8 reg)
{
	hal_outb((void *)0x3c2, reg);
}


static u8 _asiliant_getxr(chips_t *chips, u8 reg)
{
	hal_outb((void *)0x3d6, reg);
	return hal_inb((void *)0x3d7);
}


static void _asiliant_setxr(chips_t *chips, u8 reg, u8 data)
{
	hal_outb((void *)0x3d6, reg);
	hal_outb((void *)0x3d7, data);
}


static u8 _asiliant_getfr(chips_t *chips, u8 reg)
{
	hal_outb((void *)0x3d0, reg);
	return hal_inb((void *)0x3d1);
}


static void _asiliant_setfr(chips_t *chips, u8 reg, u8 data)
{
	hal_outb((void *)0x3d0, reg);
	hal_outb((void *)0x3d1, data);
}


static u8 _asiliant_getcr(chips_t *chips, u8 reg)
{
	hal_outb((void *)0x3d4, reg);
	return hal_inb((void *)0x3d5);
}


static void _asiliant_setcr(chips_t *chips, u8 reg, u8 data)
{
	hal_outb((void *)0x3d4, reg);
	hal_outb((void *)0x3d5, data);
}


static u8 _asiliant_getgr(chips_t *chips, u8 reg)
{
	hal_outb((void *)0x3ce, reg);
	return hal_inb((void *)0x3cf);
}


static void _asiliant_setgr(chips_t *chips, u8 reg, u8 data)
{
	hal_outb((void *)0x3ce, reg);
	hal_outb((void *)0x3cf, data);
}


static u8 _asiliant_getsr(chips_t *chips, u8 reg)
{
	hal_outb((void *)0x3c4, reg);
	return hal_inb((void *)0x3c5);
}


static void _asiliant_setsr(chips_t *chips, u8 reg, u8 data)
{
	hal_outb((void *)0x3c4, reg);
	hal_outb((void *)0x3c5, data);
}


__attribute__((unused))
static u8 _asiliant_getar(chips_t *chips, u8 reg)
{
	hal_inb((void *)0x3da);
	hal_outb((void *)0x3c0, reg);
	return hal_inb((void *)0x3c1);
}


__attribute__((unused))
static void _asiliant_setar(chips_t *chips, u8 reg, u8 data)
{
	hal_inb((void *)0x3da);
	
	hal_outb((void *)0x3c0, reg);
	hal_outb((void *)0x3c0, data);
}


__attribute__((unused))
static void _asiliant_clockProbe(chips_t *chips)
{
	unsigned int i;
	unsigned int N,M,PSN,P,VCO_D;
	int offset;
	unsigned int probed[3];
	u8 tmp;

	/* Probe the dot clocks */
	for (i = 0; i < 3; i++) {
		offset = i * 4;
		tmp = _asiliant_getxr(chips, 0xc2 + offset);
		M = (_asiliant_getxr(chips, 0xc0 + offset) | (tmp & 0x03)) + 2;
		N = (_asiliant_getxr(chips, 0xc1 + offset) | ((tmp >> 4) & 0x03)) + 2;
		
		tmp = _asiliant_getxr(chips, 0xc3 + offset);
		PSN = 1;
		VCO_D = ((tmp & 0x04) ? 1 : 4);
		P = ((tmp & 0x70) >> 4);

		probed[i] = VCO_D * Fref / N;
		probed[i] = probed[i] * M / (PSN * (1 << P));
		probed[i] = probed[i] / 1000;

		main_printf(ATTR_DEBUG, "dev: (asiliant) clock[%d]=%d\n", i, probed[i]);
	}
	return;
}


/* Calculate ratios for dot clocks without using a single long long value */
static void _asiliant_clockCalc(screen_t *screen, unsigned int pixclock, u8 clk[] /*dclk2_m, u8 *dclk2_n, u8 *dclk2_div*/)
{
	unsigned Ftarget = 1000000 * (1000000 / pixclock);
	unsigned n;
	unsigned best_error = 0xffffffff;
	unsigned best_m = 0xffffffff, best_n = 0xffffffff;
	unsigned ratio, remainder;
	unsigned char divisor = 0;

	/* Calculate frequency required. This is hard enough. */
	ratio = 1000000 / pixclock;
	remainder = 1000000 % pixclock;
	Ftarget = 1000000 * ratio + (1000000 * remainder) / pixclock;

	while (Ftarget < 100000000) {
		divisor += 0x10;
		Ftarget <<= 1;
	}

	ratio = Ftarget / Fref;
	remainder = Ftarget % Fref;

	/* This expresses the constraint that 150kHz <= Fref/n <= 5Mhz, together with 3 <= n <= 257 */
	for (n = 3; n <= 257; n++) {
		unsigned m = n * ratio + (n * remainder) / Fref;

		/* 3 <= m <= 257 */
		if (m >= 3 && m <= 257) {
			unsigned new_error = ((Ftarget * n) - (Fref * m)) >= 0 ? ((Ftarget * n) - (Fref * m)) : ((Fref * m) - (Ftarget * n));
			if (new_error < best_error) {
				best_n = n;
				best_m = m;
				best_error = new_error;
			}
		}
		
		/* But if VLD = 4, then 4m <= 1028 */
		else if (m <= 1028) {
			/* remember there are still only 8-bits of precision in m, so avoid over-optimistic error calculations */
			unsigned new_error = ((Ftarget * n) - (Fref * (m & ~3))) >= 0 ? ((Ftarget * n) - (Fref * (m & ~3))) : ((Fref * (m & ~3)) - (Ftarget * n));
			if (new_error < best_error) {
				best_n = n;
				best_m = m;
				best_error = new_error;
			}
		}
	}
	if (best_m > 257)
		best_m >>= 2;	/* divide m by 4, and leave VCO loop divide at 4 */
	else
		divisor |= 4;	/* or set VCO loop divide to 1 */

	clk[0] = best_m - 2;
	clk[1] = best_n - 2;
	clk[2] = divisor;
	
	return;
}


static void _asiliant_clockSave(screen_t *screen, chips_clock_t *clock)
{
	u8 tmp;
//	vga_t *vga = screen->vga;
	chips_t *chips = (chips_t *)screen->priv;

	/* (MOD) */
	clock->msr = (_asiliant_getmsr(chips) & 0xfe);
	clock->fr03 = _asiliant_getfr(chips, 0x03);

	if (!clock->clock) {
		tmp = chips->crtclkidx << 2;
		chips->crtclk[0] = _asiliant_getxr(chips, 0xc0 + tmp);
		chips->crtclk[1] = _asiliant_getxr(chips, 0xc1 + tmp);
		chips->crtclk[2] = _asiliant_getxr(chips, 0xc2 + tmp);
		chips->crtclk[3] = _asiliant_getxr(chips, 0xc3 + tmp);

		tmp = chips->fpclkidx << 2;
		chips->fpclk[0] = _asiliant_getxr(chips, 0xc0 + tmp);
		chips->fpclk[1] = _asiliant_getxr(chips, 0xc1 + tmp);
		chips->fpclk[2] = _asiliant_getxr(chips, 0xc2 + tmp);
		chips->fpclk[3] = _asiliant_getxr(chips, 0xc3 + tmp);
	}
	return;
}


static void _asiliant_clockLoad(screen_t *screen, chips_clock_t *clock)
{
//	vga_t *vga = screen->vga;
	chips_t *chips = (chips_t *)screen->priv;

	volatile unsigned char tmp, tmpmsr;
	volatile u8 tmpf03;
	unsigned char vclk[3];       
	
	tmpmsr = _asiliant_getmsr(chips);

	/* save alternate clock select reg.  */
	tmpf03 = _asiliant_getfr(chips, 0x03);

	/* select fixed clock 0  before tampering with VCLK select (MOD) */

	_asiliant_setmsr(chips, (tmpmsr & ~0x0d) | 1 /* chips->SuspendHack.vgaIOBaseFlag */);
	_asiliant_setfr(chips, 0x03, (tmpf03 & ~0x0c) | 0x04);

	if (!clock->clock) {      /* Hack to load saved console clock  */
		tmp = chips->crtclkidx << 2;

		_asiliant_setxr(chips, 0xc0 + tmp, (chips->crtclk[0] & 0xff));
		_asiliant_setxr(chips, 0xc1 + tmp, (chips->crtclk[1] & 0xff));
		_asiliant_setxr(chips, 0xc2 + tmp, (chips->crtclk[2] & 0xff));
		_asiliant_setxr(chips, 0xc3 + tmp, (chips->crtclk[3] & 0xff));

		if (chips->fpclkmod) {
			usleep(10000); /* let VCO stabilize */
			tmp = chips->fpclkidx << 2;
			_asiliant_setxr(chips, 0xc0 + tmp, (chips->fpclk[0] & 0xff));
			_asiliant_setxr(chips, 0xc1 + tmp, (chips->fpclk[1] & 0xff));
			_asiliant_setxr(chips, 0xc2 + tmp, (chips->fpclk[2] & 0xff));
			_asiliant_setxr(chips, 0xc3 + tmp, (chips->fpclk[3] & 0xff));
		}
	}
	else {
		/*
		 * Don't use the extra 2 bits in the M, N registers available
		 * on the HiQV, so write zero to 0xCA 
		 */
		_asiliant_clockCalc(screen, clock->clock, vclk);
		tmp = chips->crtclkidx << 2;
		_asiliant_setxr(chips, 0xc0 + tmp, (vclk[1] & 0xff));
		_asiliant_setxr(chips, 0xc1 + tmp, (vclk[2] & 0xff));
		_asiliant_setxr(chips, 0xc2 + tmp, 0x0);
		_asiliant_setxr(chips, 0xc3 + tmp, (vclk[0] & 0xff));
		
		if (chips->fpclkidx) {
			/* let VCO stabilize */
			usleep(10000);
			_asiliant_clockCalc(screen, clock->fpclock, vclk);
			tmp = chips->fpclkidx << 2;
			_asiliant_setxr(chips, 0xc0 + tmp, (vclk[1] & 0xff));
			_asiliant_setxr(chips, 0xc1 + tmp, (vclk[2] & 0xff));
			_asiliant_setxr(chips, 0xc2 + tmp, 0x0);
			_asiliant_setxr(chips, 0xc3 + tmp, (vclk[0] & 0xff));
			chips->fpclkmod = 1;
		}
	}
	
	/* Let VCO stabilise */
	usleep(10000);
	_asiliant_setfr(chips, 0x03, ((tmpf03 & ~0x0c) | (clock->fr03 & 0x0c)));

//	vga->writemsc(vga, (clock->msr & 0xfe) | chips->SuspendHack.vgaIOBaseFlag);
	_asiliant_setmsr(chips, (clock->msr & 0xfe) | 1);
}


void _asiliant_fixresume(screen_t *screen)
{
	chips_t *chips = (void *)screen->priv;
//	vga_t *vga = chips->vga;
	u8 tmp;

	tmp = _asiliant_getmsr(chips);

//	vga->writemsc(vga, (tmp & 0xfe) | chips->hack.vgaIOBaseFlag);
	_asiliant_setmsr(chips, (tmp & 0xfe) | 1 /* chips->hack.vgaIOBaseFlag*/);

	tmp = _asiliant_getcr(chips, 0x11);      /* (MOD) */
	_asiliant_setcr(chips, 0x11, tmp & 0x7f);
}


static void _asiliant_save(screen_t *screen, /* vga_regs_t *vgaregs, */chips_regs_t *chipsregs)
{
	chips_t *chips = (chips_t *)screen->priv;
//	vga_t vga = chips->vga;
	int i;
	u8 tmp;

	/* set registers into memory space */	
	_asiliant_setxr(chips, 0x0e, 0x00);

	_asiliant_fixresume(screen);

	tmp = _asiliant_getxr(chips, 0x02);
	_asiliant_setxr(chips, 0x02, tmp & ~0x18);

	/* get generic registers */
//	vga_save(scrn, vgaregs, VGA_SR_ALL);
for (i = 0; i < 25; i++)
	chipsregs->vcr[i] = _asiliant_getcr(chips, i);

for (i = 0; i < 9; i++)
	chipsregs->gr[i] = _asiliant_getgr(chips, i);

for (i = 1; i < 5; i++)
	chipsregs->sr[i] = _asiliant_getsr(chips, i);

//for (i = 0; i < 21; i++)
//	chipsregs->ar[i] = _asiliant_getar(chips, i);

	/* save clock */
	_asiliant_clockSave(screen, &chipsregs->clock);

	/* save extended registers */
	for (i = 0; i < 0xff; i++) {
#ifdef SAR04
		/* Save SAR04 multimedia register correctly */
		if (i == 0x4f)
			_asiliant_setxr(chips, 0x4e, 0x04);
#endif
		chipsregs->xr[i] = _asiliant_getxr(chips, i);
	}

	for (i = 0; i < 0x80; i++)
		chipsregs->fr[i] = _asiliant_getfr(chips, i);

//	for (i = 0; i < 0x80; i++)
//		_asiliant_mr[i] = _asiliant_getmr(chips, i);

	/* Save CR0-CR40 even though we don't use them, so they can be printed (MOD) */
	for (i = 0x0; i < 0x80; i++)
		chipsregs->cr[i] = _asiliant_getcr(chips, i);
}


static void _asiliant_restoreExtendedRegs(screen_t *screen, chips_regs_t *regs)
{
	chips_t *chips = (chips_t *)screen->priv;
//	vga_t *vga = chips->vga;
	int i;
	unsigned char tmp;

	/* set extended regs */
	for (i = 0; i < 0x43; i++) {
		if (_asiliant_getxr(chips, i) != regs->xr[i])
			_asiliant_setxr(chips, i, regs->xr[i]);
	}

#if 0
	/* Set SAR04 multimedia register correctly */
	if ((chips->flags & chipsOverlay8plus16) || (chips->flags & chipsVideoSupport)) {
#ifdef SAR04
		_asiliant_setxr(chips, 0x4e, 0x04);
		if (_asiliant_getxr(chips, 0x4f) != regs->xR[0x4f])
			_asiliant_setxr(chips, 0x4f, regs->xr[0x4f]);
#endif
	}
#endif

	/* Don't touch reserved memory control registers */
	for (i = 0x50; i < 0xbf; i++) {
		if ((_asiliant_getxr(chips, i)) != regs->xr[i])
			_asiliant_setxr(chips, i, regs->xr[i]);
	}
		
	/* Don't touch VCLK regs, but fix up MClk */
	
	/* set mem clock - select Fixed MClk before */
	tmp = _asiliant_getxr(chips, 0xce);
	_asiliant_setxr(chips, 0xce, tmp & 0x7f);

	if ((_asiliant_getxr(chips, 0xcc)) != regs->xr[0xcc])
		_asiliant_setxr(chips, 0xcc, regs->xr[0xcc]);	
	if ((_asiliant_getxr(chips, 0xcd)) != regs->xr[0xcd])
		_asiliant_setxr(chips, 0xcd, regs->xr[0xcd]);
	if ((_asiliant_getxr(chips, 0xce)) != regs->xr[0xce])
		_asiliant_setxr(chips, 0xce, regs->xr[0xce]);

	/* set flat panel regs. */
	for (i = 0xd0; i < 0xff; i++) {
		if ((_asiliant_getxr(chips, i)) != regs->xr[i])
			_asiliant_setxr(chips, i, regs->xr[i]);
	}

	for (i = 0; i < 0x80; i++) {
		if (i == 0x03) {
			tmp = _asiliant_getfr(chips, 0x03);
			_asiliant_setfr(chips, 0x03, ((regs->fr[0x03] & 0xc3) | (tmp & ~0xc3)));
			continue;
		}

		/* !! set stretching but disable compensation   */
		if ( (i == 0x40) || (i==0x48)) {
			_asiliant_setfr(chips, i, regs->fr[i] & 0xfe);
			continue ;
		}

		if (_asiliant_getfr(chips, i) != regs->fr[i])
			_asiliant_setfr(chips, i, regs->fr[i]);
	}

	/* set the multimedia regs */
/*
	for (i = 0x02; i < 0x80; i++) {
		if ((i == 0x43) || (i == 0x44))
			continue;
		if ((_asiliant_getmr(chips, i)) != regs->mr[i])
			_asiliant_setmr(chips, i, regs->mr[i]);
	} */
	
	/* set extended crtc regs (MOD) */
	for (i = 0x30; i < 0x80; i++) {
		if (_asiliant_getcr(chips, i) != regs->cr[i])
			_asiliant_setcr(chips, i, regs->cr[i]);
	}
}


static void _asiliant_restore(screen_t *screen, /* vga_regs_t *vgaregs, */ chips_regs_t *chipsregs, int fonts)
{
	chips_t *chips = (chips_t *)screen->priv;
	u8 tmp = 0;

	/* set registers so that we can program the controller */
	_asiliant_setxr(chips, 0x0e, 0x00);
	
	_asiliant_fixresume(screen);

	/*
	 * Wait for vsync if sequencer is running - stop sequencer.
	 * Only do if sync reset is ignored. Dual pipeline capable 
	 * chips have pipeline forced off here, so we don't care. 
	 */

	/* Disable sequencer (MOD) */
	while ((hal_inb((void *)0x3da) & 0x08) == 0x08);
	while ((hal_inb((void *)0x3da) & 0x08) == 0x00);

	_asiliant_setsr(chips, 7, 0);

	/* set the clock */
	_asiliant_clockLoad(screen, &chipsregs->clock);

//	vgaregs->mscout = hal_inb((void *)0x3cc);
//hal_inb((void *)0x3cc);
	/* set extended regs */
	_asiliant_restoreExtendedRegs(screen, chipsregs);

	/* 
	 * Enabling writing to the colourmap causes 69030's to lock. 
	 * Anyone care to explain to me why ????
	 */
//	vga_load(screen, vgaregs, VGA_SR_MODE | VGA_SR_CMAP | (restoreFonts ? VGA_SR_FONTS : 0));

_asiliant_setcr(chips, 17, _asiliant_getcr(chips, 17) & ~0x80);

unsigned int i;
for (i = 0; i < 25; i++)
	_asiliant_setcr(chips, i, chipsregs->vcr[i]);

for (i = 0; i < 9; i++)
	_asiliant_setgr(chips, i, chipsregs->gr[i]);

for (i = 1; i < 5; i++)
	_asiliant_setsr(chips, i, chipsregs->sr[i]);

//for (i = 0; i < 21; i++)
//	_asiliant_setar(chips, i, chipsregs->ar[i]);

	/* perform a synchronous reset */
	_asiliant_setsr(chips, 0x00, 0x01);
	usleep(10000);
	_asiliant_setsr(chips, 0x00, 0x03);

	/* Flag valid start address, if using CRT extensions */
	if ((chipsregs->xr[0x09] & 0x1) == 0x1) {
		tmp = _asiliant_getcr(chips, 0x40);
		_asiliant_setcr(chips, 0x40, tmp | 0x80);
	}

	/* Fix resume again here, as Nozomi seems to need it */
	_asiliant_fixresume(screen);
	/*vgaHWProtect(pScrn, FALSE);*/
}


static void _asiliant_modeSet(screen_t *screen, mode_t *mode)
{
	chips_t *chips = (chips_t *)screen->priv;
	unsigned int i;
	unsigned hd = mode->xres / 8;
	unsigned hs = (mode->xres + mode->right_margin) / 8;
	unsigned he = (mode->xres + mode->right_margin + mode->hsync_len) / 8;
	unsigned ht = (mode->left_margin + mode->xres + mode->right_margin + mode->hsync_len) / 8;
	unsigned vd = mode->yres;
	unsigned vs = mode->yres + mode->lower_margin;
	unsigned ve = mode->yres + mode->lower_margin + mode->vsync_len;
	unsigned vt = mode->upper_margin + mode->yres + mode->lower_margin + mode->vsync_len;
	unsigned wd = (mode->xres_virtual * ((mode->bits_per_pixel + 7) / 8)) / 8;	
	u8 clk[4];

	/* Initialize units */
	for (i = 0; i < N_ELTS(chips_init_xr); ++i)
		_asiliant_setxr(chips, chips_init_xr[i].addr, chips_init_xr[i].data);

	_asiliant_setxr(chips, 0x81, 0x16);
	_asiliant_setxr(chips, 0x82, 0x08);
	_asiliant_setxr(chips, 0x20, 0x00);
	
	for (i = 0; i < N_ELTS(chips_init_sr); ++i)
		_asiliant_setsr(chips, chips_init_sr[i].addr, chips_init_sr[i].data);

	for (i = 0; i < N_ELTS(chips_init_gr); ++i)
		_asiliant_setgr(chips, chips_init_gr[i].addr, chips_init_gr[i].data);

//	for (i = 0; i < N_ELTS(chips_init_ar); ++i)
//		_asiliant_setar(chips, chips_init_ar[i].addr, chips_init_ar[i].data);

	/* Enable video output in attribute index register */
	//hal_outb((void *)0x3c0, 0x20);

	for (i = 0; i < N_ELTS(chips_init_cr); ++i)
		_asiliant_setcr(chips, chips_init_cr[i].addr, chips_init_cr[i].data);

	for (i = 0; i < N_ELTS(chips_init_fr); ++i)
		_asiliant_setfr(chips, chips_init_fr[i].addr, chips_init_fr[i].data);

	/* Set color depth */
	if (mode->bits_per_pixel == 24) {
		_asiliant_setxr(chips, 0x81, 0x16);	/* 24 bit packed color mode */
		_asiliant_setxr(chips, 0x82, 0x00);	/* Disable palettes */
		_asiliant_setxr(chips, 0x20, 0x20);	/* 24 bit blitter mode */
	}
	else if (mode->bits_per_pixel == 16) {
		if (mode->red.offset == 11)
			_asiliant_setxr(chips, 0x81, 0x15);	/* 16 bit color mode */
		else
			_asiliant_setxr(chips, 0x81, 0x14);	/* 15 bit color mode */
		_asiliant_setxr(chips, 0x82, 0x00);	/* Disable palettes */
		_asiliant_setxr(chips, 0x20, 0x10);	/* 16 bit blitter mode */
	}
	else if (mode->bits_per_pixel == 8) {
		_asiliant_setxr(chips, 0x0a, 0x02);	/* Linear */

_asiliant_setxr(chips, 0x81, 0x10);	/* 8 bit color mode */

		_asiliant_setxr(chips, 0x82, 0x00);	/* Graphics gamma enable */
		_asiliant_setxr(chips, 0x20, 0x00);	/* 8 bit blitter mode */
	}

	/* p->fix.line_length = mode->xres * (mode->bits_per_pixel >> 3); */
	/* p->fix.visual = (mode->bits_per_pixel == 8) ? FB_VISUAL_PSEUDOCOLOR : FB_VISUAL_TRUECOLOR; */

	/* Set pixel clock */
	_asiliant_clockCalc(screen, mode->pixclock, clk);
	_asiliant_setxr(chips, 0xc4, clk[0]);
	_asiliant_setxr(chips, 0xc5, clk[1]);
	_asiliant_setxr(chips, 0xc7, clk[2]);

	/* Set screen timing */
	if ((mode->xres == 640) && (mode->yres == 480) && (mode->pixclock == 39722))
		_asiliant_setfr(chips, 0x01, 0x02);  /* LCD */
	else
		_asiliant_setfr(chips, 0x01, 0x01);  /* CRT */

	_asiliant_setcr(chips, 0x11, (ve - 1) & 0x0f);
	_asiliant_setcr(chips, 0x00, (ht - 5) & 0xff);
	_asiliant_setcr(chips, 0x01, hd - 1);
	_asiliant_setcr(chips, 0x02, hd);
	_asiliant_setcr(chips, 0x03, ((ht - 1) & 0x1f) | 0x80);
	_asiliant_setcr(chips, 0x04, hs);
	_asiliant_setcr(chips, 0x05, (((ht - 1) & 0x20) << 2) | (he & 0x1f));
	_asiliant_setcr(chips, 0x3c, (ht - 1) & 0xc0);
	_asiliant_setcr(chips, 0x06, (vt - 2) & 0xff);
	_asiliant_setcr(chips, 0x30, (vt - 2) >> 8);
	_asiliant_setcr(chips, 0x07, 0x00);
	_asiliant_setcr(chips, 0x08, 0x00);
	_asiliant_setcr(chips, 0x09, 0x00);
	_asiliant_setcr(chips, 0x10, (vs - 1) & 0xff);
	_asiliant_setcr(chips, 0x32, ((vs - 1) >> 8) & 0xf);
	_asiliant_setcr(chips, 0x11, ((ve - 1) & 0x0f) | 0x80);
	_asiliant_setcr(chips, 0x12, (vd - 1) & 0xff);
	_asiliant_setcr(chips, 0x31, ((vd - 1) & 0xf00) >> 8);
	_asiliant_setcr(chips, 0x13, wd & 0xff);
	_asiliant_setcr(chips, 0x41, (wd & 0xf00) >> 8);
	_asiliant_setcr(chips, 0x15, (vs - 1) & 0xff);
	_asiliant_setcr(chips, 0x33, ((vs - 1) >> 8) & 0xf);
	_asiliant_setcr(chips, 0x38, ((ht - 5) & 0x100) >> 8);
	_asiliant_setcr(chips, 0x16, (vt - 1) & 0xff);
	_asiliant_setcr(chips, 0x18, 0x00);
	
	screen->width = mode->xres;
	screen->height = mode->yres;
	screen->bpp = mode->bits_per_pixel;
}


static void asiliant_test(screen_t *screen)
{
	int k, xb, xe, yb, ye;
	image_t image;

	image.width = screen->width;
	image.height = screen->height;
	image.bpp = 2;
	image.data = screen->fb;
  
	for (k = 0; k < 1024 * 128; k++) {
		xb = main_random(screen->width);
		xe = main_random(screen->width);
		yb = main_random(screen->height - 200);
		ye = main_random(screen->height - 200);
		graph_line(&image, xb, 200 + yb, xe - xb, ye - yb, 3, main_random(65536));
	}
	return;
}	


static int _asiliant_devinit(pci_device_t *dev, screen_t *screen)
{
	void *vram;
	chips_t *chips;
  
	if (vm_iomap(dev->resources[0].base, dev->resources[0].limit, PGHD_WRITE | PGHD_PRESENT, &vram) < 0)
		return -ENOMEM;	

	main_printf(ATTR_DEBUG, "asiliant: screen mapped at %p, limit=%x\n", vram, dev->resources[0].limit);
	
	if ((chips = (chips_t *)vm_kmalloc(sizeof(chips_t))) == NULL) {
		main_printf(ATTR_ERROR, "asiliant: Can't allocate memory for CT69000 driver!\n");
		return -ENOMEM;
	}
	hal_memset(chips, 0, sizeof(chips_t));
	chips->crtclkidx = 1;
	chips->fpclkidx = 1;

	chips->fbbase = vram;
	chips->fbsize = dev->resources[0].limit;

	screen->priv = (void *)chips;
	screen->fb = chips->fbbase;

	_asiliant_save(screen, &chips->savedregs);

	_asiliant_modeSet(screen, &modes[2]);
	asiliant_test(screen);
	_asiliant_restore(screen, &chips->savedregs, 0);


	_asiliant_save(screen, &chips->savedregs);
	_asiliant_modeSet(screen, &modes[2]);
	_asiliant_restore(screen, &chips->savedregs, 0);

	main_printf(ATTR_DEBUG, "return to text mode\n");

	return 0;
}


int asiliant_init(screen_t *screen)
{
	pci_device_t *dev;
	unsigned int i;  
	static const pci_id_t ids[] = {
		{ 0x102c, 0x00c0, PCI_ANY, PCI_ANY, PCI_ANY },
		{ 0, 0, 0, 0 }
	};

	/* Find all asiliant devices */
	for (i = 0; ids[i].vendor != 0; i++) {
		dev_pciAlloc(&ids[i], &dev);

		if (dev) {
			main_printf(ATTR_INFO, "graph: Asiliant graphics card found\n");
			_asiliant_devinit(dev, screen);
			break;
		}
	}
	return EOK;
}
