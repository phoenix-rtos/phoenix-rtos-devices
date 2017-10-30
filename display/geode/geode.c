/*
 * Graph library for DPMI32
 *
 * LX Geode driver
 *
 * Copyright 2009 Phoenix Systems
 */

#include <stdlib.h>
#include <stdio.h>
#include <dos.h>

#include <kernel.h>

#include "graph.h"


#define VENDOR 0x1022                  // AMD
#define DEVICE 0x2081                  // Geode LX

#define CURSORSZ 0x1000


#define DC3_IRQFILTCTL    0x94
#define DC3_IRQ           0xc8

#define GP3_CMD_TOP    0x00000050
#define GP3_CMD_BOTTOM 0x00000054
#define GP3_CMD_READ   0x00000058
#define GP3_CMD_WRITE  0x0000005c
#define GP3_BASE_OFFSET  0x4c

#define DC3_UNLOCK    0
#define DC3_GCFG      4
#define DC3_DCFG      8
#define DC3_PALADDR   0x70
#define DC3_PALDATA   0x74


/* Graphic modes table */
static unsigned int modes[] = {
  GRAPH_640x480x8,   0x04101, 640,  480,  1,
  GRAPH_800x600x8,   0x04103, 800,  600,  1,
  GRAPH_1024x768x8,  0x04105, 1024, 768,  1,
  GRAPH_1280x1024x8, 0x04107, 1280, 1024, 1,
  GRAPH_640x480x16,  0x04111, 640,  480,  2,
  GRAPH_800x600x16,  0x04114, 800,  600,  2,
  GRAPH_1024x768x16, 0x04117, 1024, 768,  2,
  0
};


struct {
  u8 irq;
  u32 fb;
  u32 gpbase;
  u16 vgsel;               // video generator
  u16 gpsel;               // graphic processor
  u32 cmdoffs;             // command buffer offset
  u16 prev_mode;           // previous graphic mode

  unsigned char bg;
  unsigned char fg;
} geode;


/* Function schedules and executes new task */
extern int graph_schedule();


int geode_isr(u32 n, void *d)
{
  graph_t *graph = (graph_t *)d;

  u32 irqenable = low_getfar(geode.vgsel, DC3_IRQ);

  if (!(irqenable & 0x10000))
    return IRQ_IGNORE;

  graph->vsync++;

  /* Clear interrupt source */
  u32 unlock = low_getfar(geode.vgsel, 0);
  low_setfar(geode.vgsel, 0, 0x4758);
  low_setfar(geode.vgsel, DC3_IRQ, irqenable);
  low_setfar(geode.vgsel, 0, unlock);

  asm sti;
  graph_schedule();

  return IRQ_HANDLED;
}


int geode_open(graph_t *graph, unsigned char irq)
{
  asm {
    mov eax, 4f03h
    int 10h                            // get VESA mode
  }
  if (_AX != 0x4f)
    return GRAPH_ERR_DEVICE;

  geode.prev_mode = _BX;

  if (irq < 16)
    geode.irq = irq;                   // use forced irq
  else
    irq = geode.irq;                   // use PCI interrupt

  if (irq > 15)
    return GRAPH_ERR_DEVICE;           // no interrupt assigned

  if (irq && irq_install(irq, geode_isr, graph))
    return GRAPH_ERR_DPMI;

  if (irq)
    low_maskirq(irq, 0);

  u32 unlock = low_getfar(geode.vgsel, 0);
  low_setfar(geode.vgsel, 0, 0x4758);

  u32 irqline = low_getfar(geode.vgsel, DC3_IRQFILTCTL);
  low_setfar(geode.vgsel, DC3_IRQFILTCTL, irqline & ~0x7ff0000);

  u32 irqenable = low_getfar(geode.vgsel, DC3_IRQ);
  low_setfar(geode.vgsel, DC3_IRQ, irqenable & ~0x1 | 0x30000);

  low_setfar(geode.vgsel, 0, unlock);

  return GRAPH_SUCCESS;
}


int geode_close(graph_t *graph)
{
  irq_uninstall(geode.irq, geode_isr, NULL);

  if (geode.prev_mode) {
    _EBX = geode.prev_mode;            // previous graphic mode
    asm {
      mov eax, 4f02h
      int 10h                          // set VESA mode
    }
    if (_AX != 0x4f)
      return GRAPH_ERR_DEVICE;
  }

  return GRAPH_SUCCESS;
}


int geode_mode(graph_t *graph, int mode, char freq)
{
  int index;

  for (index = 0; modes[index] != mode; index += 5)
    if (!modes[index])
      return GRAPH_ERR_ARG;

  graph->width = modes[index + 2];
  graph->height = modes[index + 3];
  graph->depth = modes[index + 4];

  // set screen parameters
  _EBX = modes[index + 1];
  asm {
    mov eax, 4f02h
    int 10h                            // set VESA mode
  }
  if (_AX != 0x4f)
    return GRAPH_ERR_DEVICE;

  return GRAPH_SUCCESS;
}

#define GP3_BLT_STATUS 0x44

#define GP3_BS_BLT_BUSY 0x1
#define GP3_BS_BLT_PENDING 0x4
#define GP3_BS_HALF_EMPTY 0x8
#define GP3_BS_CB_EMPTY 0x10


int geode_isbusy(graph_t *graph)
{
  u32 status = low_getfar(geode.gpsel, GP3_BLT_STATUS);

  if ((status & GP3_BS_BLT_BUSY) || !(status & GP3_BS_CB_EMPTY))
    return -1;

  return 0;
}


int geode_trigger(graph_t *graph)
{
  // Verify interrupt and clear interrupt source
  u32 flags;
  asm {
    pushfd
    pop [flags]
    cli
  }

  u32 irqenable = low_getfar(geode.vgsel, DC3_IRQ);
  u32 unlock = low_getfar(geode.vgsel, 0);
  low_setfar(geode.vgsel, 0, 0x4758);
  low_setfar(geode.vgsel, DC3_IRQ, irqenable);
  low_setfar(geode.vgsel, 0, unlock);

  if (irqenable & 0x10000)
    ++graph->vsync;

  asm {
    push [flags]
    popfd
  }

  if (geode_isbusy(graph))
    return GRAPH_ERR_BUSY;

  return graph_schedule();
}


#define GP3_BLT_WRAP       0x80000000
#define GP3_BLT_TYPE       0x00000000
#define GP3_BLT_HAZARD     0x10000000
#define GP3_BLT_RASTER     0x00000001
#define GP3_BLT_DSTOFFS    0x00000002
#define GP3_BLT_SRCOFFS    0x00000004
#define GP3_BLT_STRIDE     0x00000008
#define GP3_BLT_WIDHI      0x00000010
#define GP3_BLT_SRCFG      0x00000020
#define GP3_BLT_SRCBG      0x00000040
#define GP3_BLT_PATCLR0    0x00000080
#define GP3_BLT_PATCLR1    0x00000100
#define GP3_BLT_PATDTA0    0x00000200
#define GP3_BLT_PATDTA1    0x00000400
#define GP3_BLT_CH3OFFS    0x00000800
#define GP3_BLT_CH3STRIDE  0x00001000
#define GP3_BLT_CH3WIDHI   0x00002000
#define GP3_BLT_BASEOFFS   0x00004000
#define GP3_BLT_MODE       0x00008000

#define GP3_BLT_RASTER_OFFS      0x00000004
#define GP3_BLT_DSTOFFS_OFFS     0x00000008
#define GP3_BLT_SRCOFFS_OFFS     0x0000000c
#define GP3_BLT_STRIDE_OFFS      0x00000010
#define GP3_BLT_WIDHI_OFFS       0x00000014
#define GP3_BLT_SRCFG_OFFS       0x00000018
#define GP3_BLT_SRCBG_OFFS       0x0000001c
#define GP3_BLT_PATCLR0_OFFS     0x00000020
#define GP3_BLT_PATCLR1_OFFS     0x00000024
#define GP3_BLT_PATDTA0_OFFS     0x00000028
#define GP3_BLT_PATDTA1_OFFS     0x0000002c
#define GP3_BLT_CH3OFFS_OFFS     0x00000030
#define GP3_BLT_CH3STRIDE_OFFS   0x00000034
#define GP3_BLT_CH3WIDHI_OFFS    0x00000038
#define GP3_BLT_BASEOFFS_OFFS    0x0000003c
#define GP3_BLT_MODE_OFFS        0x00000040


int geode_rect(graph_t *graph, char *arg)
{
  unsigned int x = *(unsigned int *)arg;
  unsigned int y = *(unsigned int *)(arg + sizeof(int));
  unsigned int dx = *(unsigned int *)(arg + 2 * sizeof(int));
  unsigned int dy = *(unsigned int *)(arg + 3 * sizeof(int));
  unsigned int color = *(unsigned int *)(arg + 4 * sizeof(int));

  u32 dstoffset = (y * graph->width + x) * graph->depth;
  u32 base = (geode.gpbase & 0x3fffff) | ((geode.fb & 0xff000000) + (dstoffset & 0xffc00000));
  u32 hdr = GP3_BLT_TYPE | GP3_BLT_RASTER | GP3_BLT_DSTOFFS | GP3_BLT_STRIDE |
    GP3_BLT_WIDHI | GP3_BLT_SRCFG | GP3_BLT_BASEOFFS | GP3_BLT_MODE;

  u32 rm = 0xcc;
  if (graph->depth == 2)
    rm |= 0x60000000;
  else if (graph->depth == 4)
    rm |= 0x80000000;

  low_setfar(graph->fbsel, geode.cmdoffs, hdr);
  low_setfar(graph->fbsel, geode.cmdoffs + GP3_BLT_RASTER_OFFS, rm);
  low_setfar(graph->fbsel, geode.cmdoffs + GP3_BLT_DSTOFFS_OFFS, dstoffset & 0x3fffff);
  low_setfar(graph->fbsel, geode.cmdoffs + GP3_BLT_STRIDE_OFFS, graph->depth * graph->width);
  low_setfar(graph->fbsel, geode.cmdoffs + GP3_BLT_WIDHI_OFFS, (dx << 16) | dy);
  low_setfar(graph->fbsel, geode.cmdoffs + GP3_BLT_SRCFG_OFFS, color);
  low_setfar(graph->fbsel, geode.cmdoffs + GP3_BLT_BASEOFFS_OFFS, base);
  low_setfar(graph->fbsel, geode.cmdoffs + GP3_BLT_MODE_OFFS, 0x4);

  low_setfar(geode.gpsel, GP3_CMD_READ, 0);
  low_setfar(geode.gpsel, GP3_CMD_WRITE, 68);

  return GRAPH_SUCCESS;
}


int geode_move(graph_t *graph, char *arg)
{
  unsigned int x = *(unsigned int *)arg;
  unsigned int y = *(unsigned int *)(arg + sizeof(int));
  unsigned int dx = *(unsigned int *)(arg + 2 * sizeof(int));
  unsigned int dy = *(unsigned int *)(arg + 3 * sizeof(int));
  int mx = *(int *)(arg + 4 * sizeof(int));
  int my = *(int *)(arg + 5 * sizeof(int));

  u32 srcoffset = (y * graph->width + x) * graph->depth;
  u32 dstoffset = srcoffset + (my * graph->width + mx) * graph->depth;

  u32 bltmode = 0x5;
  if (mx > 0) {
    srcoffset += dx * graph->depth - 1;
    dstoffset += dx * graph->depth - 1;
    bltmode |= 0x200;
  }
  if (my > 0) {
    srcoffset += (dy - 1) * graph->width * graph->depth;
    dstoffset += (dy - 1) * graph->width * graph->depth;
    bltmode |= 0x100;
  }

  u32 base = ((geode.fb & 0xff000000) + (dstoffset & 0xffc00000)) |
    ((geode.fb & 0xff000000) >> 10 + ((srcoffset & 0xffc00000) >> 10));

  u32 hdr = GP3_BLT_TYPE | GP3_BLT_RASTER | GP3_BLT_SRCOFFS | GP3_BLT_DSTOFFS |
    GP3_BLT_STRIDE | GP3_BLT_WIDHI | GP3_BLT_BASEOFFS | GP3_BLT_MODE;

  u32 rm = 0xcc;
  if (graph->depth == 2)
    rm |= 0x60000000;
  else if (graph->depth == 4)
    rm |= 0x80000000;

  low_setfar(graph->fbsel, geode.cmdoffs, hdr);
  low_setfar(graph->fbsel, geode.cmdoffs + GP3_BLT_RASTER_OFFS, rm);
  low_setfar(graph->fbsel, geode.cmdoffs + GP3_BLT_SRCOFFS_OFFS, srcoffset & 0x3fffff);
  low_setfar(graph->fbsel, geode.cmdoffs + GP3_BLT_DSTOFFS_OFFS, dstoffset & 0x3fffff);
  low_setfar(graph->fbsel, geode.cmdoffs + GP3_BLT_STRIDE_OFFS, (graph->depth * graph->width << 16) | graph->depth * graph->width);
  low_setfar(graph->fbsel, geode.cmdoffs + GP3_BLT_WIDHI_OFFS, (dx << 16) | dy);
  low_setfar(graph->fbsel, geode.cmdoffs + GP3_BLT_BASEOFFS_OFFS, base);
  low_setfar(graph->fbsel, geode.cmdoffs + GP3_BLT_MODE_OFFS, bltmode);

  low_setfar(geode.gpsel, GP3_CMD_READ, 0);
  low_setfar(geode.gpsel, GP3_CMD_WRITE, 68);

  return GRAPH_SUCCESS;
}


int geode_cursorcol(char *buf, unsigned char color)
{
  color &= 1;

  u32 unlock = low_getfar(geode.vgsel, DC3_UNLOCK);

  low_setfar(geode.vgsel, DC3_UNLOCK, 0x4758);
  low_setfar(geode.vgsel, DC3_PALADDR, 256 + color);

  u32 c = ((u32)(u8)*(buf++) << 16) | ((u32)(u8)*(buf++) << 8) | (u8)*buf;
  low_setfar(geode.vgsel, DC3_PALDATA, c);
  low_setfar(geode.vgsel, DC3_UNLOCK, unlock);

  return GRAPH_SUCCESS;
}


int geode_colorset(graph_t *graph, char *colors, unsigned char first, unsigned char last)
{
  int i;

  u32 unlock = low_getfar(geode.vgsel, DC3_UNLOCK);

  low_setfar(geode.vgsel, DC3_UNLOCK, 0x4758);
  low_setfar(geode.vgsel, DC3_PALADDR, first);

  for (i = first; i <= last; i++) {
    u32 c = ((u32)(u8)*(colors++) << 16) | ((u32)(u8)*(colors++) << 8) | (u8)*(colors++);
    low_setfar(geode.vgsel, DC3_PALDATA, c);
  }
  low_setfar(geode.vgsel, DC3_UNLOCK, unlock);

  return GRAPH_SUCCESS;
}


int geode_colorget(graph_t *graph, char *colors, unsigned char first, unsigned char last)
{
  int i;

  u32 unlock = low_getfar(geode.vgsel, DC3_UNLOCK);

  low_setfar(geode.vgsel, DC3_UNLOCK, 0x4758);
  low_setfar(geode.vgsel, DC3_PALADDR, first);

  for (i = first; i <= last; i++) {
    u32 c = low_getfar(geode.vgsel, DC3_PALDATA);
    *(colors++) = (c >> 16);
    *(colors++) = (c >> 8);
    *(colors++) = c;
  }
  low_setfar(geode.vgsel, DC3_UNLOCK, unlock);

  return GRAPH_SUCCESS;
}


int geode_cursorset(graph_t *graph, char *and, char *xor, unsigned char bg, unsigned char fg)
{
  int n, a;
  char colors[3];

  geode.bg = bg;
  geode.fg = fg;

  geode_colorget(graph, colors, bg, bg);
  geode_cursorcol(colors, 0);
  geode_colorget(graph, colors, fg, fg);
  geode_cursorcol(colors, 1);

  u32 offs = graph->memsz - graph->cursorsz;

  for (n = 0; n < 64; ++n) {
    a = (u32)(u8)and[n * 8] << 24 | (u32)(u8)and[n * 8 + 1] << 16 | (u32)(u8)and[n * 8 + 2] << 8 | (u32)(u8)and[n * 8 + 3];
    low_setfar(graph->fbsel, offs + n * 16 + 4, a);
    a = (u32)(u8)and[n * 8 + 4] << 24 | (u32)(u8)and[n * 8 + 5] << 16 | (u32)(u8)and[n * 8 + 6] << 8 | (u32)(u8)and[n * 8 + 7];
    low_setfar(graph->fbsel, offs + n * 16, a);

    a = (u32)(u8)xor[n * 8] << 24 | (u32)(u8)xor[n * 8 + 1] << 16 | (u32)(u8)xor[n * 8 + 2] << 8 | (u32)(u8)xor[n * 8 + 3];
    low_setfar(graph->fbsel, offs + n * 16 + 12, a);
    a = (u32)(u8)xor[n * 8 + 4] << 24 | (u32)(u8)xor[n * 8 + 5] << 16 | (u32)(u8)xor[n * 8 + 6] << 8 | (u32)(u8)xor[n * 8 + 7];
    low_setfar(graph->fbsel, offs + n * 16 + 8, a);
  }

  return GRAPH_SUCCESS;
}


int geode_cursorpos(graph_t *graph, unsigned int x, unsigned int y)
{
  u32 unlock = low_getfar(geode.vgsel, DC3_UNLOCK);

  low_setfar(geode.vgsel, DC3_UNLOCK, 0x4758);
  low_setfar(geode.vgsel, 0x60, x);
  low_setfar(geode.vgsel, 0x64, y);
  low_setfar(geode.vgsel, DC3_UNLOCK, unlock);

  return GRAPH_SUCCESS;
}


int geode_cursorshow(graph_t *graph)
{
  /* Enable hardware cursor */
  u32 unlock = low_getfar(geode.vgsel, DC3_UNLOCK);
  u32 gcfg = low_getfar(geode.vgsel, 4);

  u32 offs = graph->memsz - graph->cursorsz;

  low_setfar(geode.vgsel, DC3_UNLOCK, 0x4758);
  low_setfar(geode.vgsel, 0x18, offs);
  low_setfar(geode.vgsel, DC3_GCFG, gcfg | 2);
  low_setfar(geode.vgsel, DC3_UNLOCK, unlock);

  return GRAPH_SUCCESS;
}


int geode_cursorhide(graph_t *graph)
{
  /* Disable hardware cursor */
  u32 unlock = low_getfar(geode.vgsel, DC3_UNLOCK);
  u32 gcfg = low_getfar(geode.vgsel, 4);

  low_setfar(geode.vgsel, DC3_UNLOCK, 0x4758);
  low_setfar(geode.vgsel, DC3_GCFG, gcfg & ~2);
  low_setfar(geode.vgsel, DC3_UNLOCK, unlock);

  return GRAPH_SUCCESS;
}


static void geode_msr_read(u32 reg, u32 dev, u32 *v)
{
  u32 addr = (dev | reg);
  asm {
    mov dx, 0xac1c
    mov eax, 0xfc530007
    out dx, eax
    add dl, 2
    mov ecx, [addr]
    in ax, dx
  }
  v[0] = _EAX;
  v[1] = _EDX;

  return;
}


static void geode_msr_write(u32 reg, u32 dev, u32 *v)
{
  u32 addr = (dev | reg);
  u32 h = v[1], l = v[0];

  asm {
    mov dx, 0xac1c
    mov eax, 0xfc530007
    out dx, eax
    add dl, 2
    mov ecx, [addr]
    mov ebx, [h]
    mov eax, [l]
    mov esi, 0
    mov edi, 0
    out dx, ax
  }
  return;
}


#define MSR_GEODELINK_CAP   0x2000
#define MSR_GLIU_CAP        0x0086

#define MSR_ADDRESS_VAIL    0x00000000
#define MSR_ADDRESS_GLIU0   0x10000000
#define MSR_ADDRESS_GLIU1   0x40000000
#define MSR_ADDRESS_GLIU2   0x51010000


void geode_enumgeodelink()
{
  u32 v[2];
  unsigned int i;

  geode_msr_read(MSR_GLIU_CAP, MSR_ADDRESS_GLIU0, v);
  printf("MSR0: %p%p\n", v[1], v[0]);

  geode_msr_read(MSR_GLIU_CAP, MSR_ADDRESS_GLIU1, v);
  printf("MSR1: %p%p\n", v[1], v[0]);

  geode_msr_read(MSR_GLIU_CAP, MSR_ADDRESS_GLIU2, v);
  printf("MSR2: %p%p\n", v[1], v[0]);

u32 gpaddr = 0;

  for (i = 0; i < 8; i++) {
    geode_msr_read(MSR_GEODELINK_CAP, (i << 29), v);
    printf("supergliu0[%d]: %p%p\n", i, v[1], v[0]);

    if (((v[0] >> 12) & 0xff) == 0x3d)
      gpaddr = (i << 29);
  }

geode_msr_read(0x2001, gpaddr, v);
printf("gpaddr=%p, cmdbase=%p%p\n", gpaddr, v[1], v[0]);

//  for (i = 0; i < 8; i++) {
//    geode_msr_read(MSR_GEODELINK_CAP, (0x21 << 29) + (i << 26), v);
//    printf("supergliu1[%d]: %p%p\n", i, v[1], v[0]);
//  }

  for (i = 0; i < 8; i++) {
    geode_msr_read(MSR_GEODELINK_CAP,
      (0x21 << 29) + (0x41 << 26) + (0x21 << 23) + (i << 20), v);
    printf("supergliu2[%d]: %p%p\n", i, v[1], v[0]);
  }

}


int geode_setcmdbuff(u32 addr, u32 start, u32 stop)
{
  u32 v[2];

  /* (MOD) */
  geode_msr_read(0x2001, 0xa0000000, v);
  v[0] &= 0xf000ffff;
  v[0] |= (addr >> 4) & 0x0fff0000;

  /* (MOD) */
  geode_msr_write(0x2001, 0xa0000000, v);

  low_setfar(geode.gpsel, GP3_CMD_TOP, start);
  low_setfar(geode.gpsel, GP3_CMD_BOTTOM, stop);
  low_setfar(geode.gpsel, GP3_CMD_READ, start);

  return 0;
}


int geode_init(graph_t *graph)
{
  pci_id_t id = { VENDOR, DEVICE, 0 };
  pci_dev_t pdev;

  if (!pci_find(&id, 0, PCI_FIND_DEVICE, &pdev))
    return GRAPH_ABSENT;

  geode.fb = pdev.base[0] & ~0xf;
  u32 gp = pdev.base[1] & ~0xf;
  u32 vg = pdev.base[2] & ~0xf;

  /* Get video memory size */
  low_outw(0xac1c, 0xfc53);
  low_outw(0xac1c, 0x200);
  int memsz = (int)(low_inw(0xac1e) & 0xfe) << 20;

  geode.irq = pdev.irq;
  geode.cmdoffs = memsz - 0x200000;

  graph->fbsel = low_dpmi_ldtalloc(1);
  low_dpmi_copydescr(low_getds(), graph->fbsel);
  low_dpmi_setbase(graph->fbsel, low_dpmi_phmap(geode.fb, memsz));
  low_dpmi_setlimit(graph->fbsel, memsz - 1);

  geode.gpsel = low_dpmi_ldtalloc(1);
  low_dpmi_copydescr(low_getds(), geode.gpsel);
  low_dpmi_setbase(geode.gpsel, low_dpmi_phmap(gp, 0x4000));
  low_dpmi_setlimit(geode.gpsel, 0x3fff);

  geode.vgsel = low_dpmi_ldtalloc(1);
  low_dpmi_copydescr(low_getds(), geode.vgsel);
  low_dpmi_setbase(geode.vgsel, low_dpmi_phmap(vg, 0x4000));
  low_dpmi_setlimit(geode.vgsel, 0x3fff);

  geode_setcmdbuff(geode.fb + geode.cmdoffs, 0, 0x200000);

  geode.gpbase = geode.fb >> 24;
  geode.gpbase = (geode.gpbase << 24) | (geode.gpbase << 14) | (geode.gpbase << 4);
  low_setfar(geode.gpsel, GP3_BASE_OFFSET, geode.gpbase);

  graph->open = geode_open;
  graph->close = geode_close;
  graph->mode = geode_mode;
  graph->isbusy = geode_isbusy;
  graph->trigger = geode_trigger;
  graph->rect = geode_rect;
  graph->move = geode_move;
  graph->colorset = geode_colorset;
  graph->colorget = geode_colorget;
  graph->cursorset = geode_cursorset;
  graph->cursorpos = geode_cursorpos;
  graph->cursorshow = geode_cursorshow;
  graph->cursorhide = geode_cursorhide;

  graph->memsz = geode.cmdoffs;
  graph->cursorsz = CURSORSZ;

  return GRAPH_SUCCESS;
}

