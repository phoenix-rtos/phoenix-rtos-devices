/*
 * Graph library for DPMI32
 *
 * S3 Savage4 driver
 *
 * Copyright 2007 IMMOS
 */

#include <stdlib.h>
#include <stdio.h>
#include <dos.h>

#include <kernel.h>

#include "graph.h"


#define VENDOR 0x5333                  // S3
#define DEVICE 0x8d01                  // Savage4

#define CMDSZ 0x100000
#define MEMSZ 0x400000
#define CURSORSZ 0x1000


/* Registers */
enum {
  PlaneWmask = 0x8128,
  PlaneRmask = 0x812c,

  LutIdx = 0x83c8,
  LutData = 0x83c9,

  CrtIdx = 0x83d4,
  CrtData = 0x83d5,

  SubsystemCtl = 0x8504,
    VsyncInt = 1,
    GebusyInt = 1 << 1,
    BfifoFullInt = 1 << 2,
    BfifoEmptyInt = 1 << 3,
    CfifoFullInt = 1 << 4,
    CfifoEmptyInt = 1 << 5,
    BciInt = 1 << 6,
    LpbInt = 1 << 7,
    CbHiInt = 1 << 16,
    CbLoInt = 1 << 17,

    VsyncEna = 1 << 8,
    GebusyEna = 1 << 9,
    BfifoFullEna = 1 << 10,
    BfifoEmptyEna = 1 << 11,
    CfifoFullEna = 1 << 12,
    CfifoEmptyEna = 1 << 13,
    BciEna = 1 << 14,
    CbHiEna = 1 << 24,
    CbLoEna = 1 << 25,

  BciFifo = 0x10000,

  CobPtr = 0x48c18,
    CobEna = 1 << 2,
    CobBciEna = 1 << 3,

  AltStatus0 = 0x48c60,
    CbeMask = 0x1ffff,
    Ge2Idle = 1 << 23
};


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
  u16 cmdsel;              // command buffer selector
  u16 prev_mode;           // previous graphic mode
} savage4;


/* Function schedules and executes new task */
extern int graph_schedule();


static u8 savage4_getcrt(u8 reg)
{
  low_setfar8(savage4.cmdsel, CrtIdx, reg);
  return low_getfar8(savage4.cmdsel, CrtData);
}


static void savage4_setcrt(u8 reg, u8 v)
{
  low_setfar8(savage4.cmdsel, CrtIdx, reg);
  low_setfar8(savage4.cmdsel, CrtData, v);
}


int savage4_isr(u32 n, void *d)
{
  graph_t *graph = (graph_t *)d;

  graph->vsync++;

  /* Clear interrupt source */
  low_setfar(savage4.cmdsel, SubsystemCtl, VsyncEna | VsyncInt);
  return IRQ_IGNORE;
}


int savage4_open(graph_t *graph, unsigned char irq)
{
  asm {
    mov eax, 4f03h
    int 10h                            // get VESA mode
  }
  if (_AX != 0x4f)
    return GRAPH_ERR_DEVICE;

  savage4.prev_mode = _BX;

  if (irq < 16)
    savage4.irq = irq;                 // use forced irq
  else
    irq = savage4.irq;                 // use PCI interrupt

  if (irq > 15)
    return GRAPH_ERR_DEVICE;           // no interrupt assigned

  /* Set planes */
  low_setfar(savage4.cmdsel, PlaneWmask, -1);
  low_setfar(savage4.cmdsel, PlaneRmask, -1);

  /* Enable BCI */
  low_setfar(savage4.cmdsel, CobPtr, low_getfar(savage4.cmdsel, CobPtr) | CobBciEna /*| 2*/);

  /* Enable interrupt sources */
  low_setfar(savage4.cmdsel, SubsystemCtl, VsyncEna | VsyncInt);

  if (irq && irq_install(irq, savage4_isr, graph))
    return GRAPH_ERR_DPMI;

  if(irq)
    low_maskirq(irq, 0);

  return GRAPH_SUCCESS;
}


int savage4_close(graph_t *graph)
{
  irq_uninstall(savage4.irq, savage4_isr, NULL);

  if (savage4.prev_mode) {
    _EBX = savage4.prev_mode;          // previous graphic mode
    asm {
      mov eax, 4f02h
      int 10h                          // set VESA mode
    }
    if (_AX != 0x4f)
      return GRAPH_ERR_DEVICE;
  }

  return GRAPH_SUCCESS;
}


int savage4_mode(graph_t *graph, int mode, char freq)
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

  /* Set GBD0 - bitmap offset */
  low_setfar(savage4.cmdsel, BciFifo, 0x960100e0);
  low_setfar(savage4.cmdsel, BciFifo + 4, 0);

  /* Set GBD1 - stride and depth */
  low_setfar(savage4.cmdsel, BciFifo + 8, 0x960100e1);
  low_setfar(savage4.cmdsel, BciFifo + 12, 0x10000009 | graph->width | (graph->depth << 19));

  return GRAPH_SUCCESS;
}


int savage4_isbusy(graph_t *graph)
{
  return (low_getfar(savage4.cmdsel, AltStatus0) & (CbeMask | Ge2Idle)) != Ge2Idle;
}


int savage4_trigger(graph_t *graph)
{
  u32 v;

  // Verify interrupt and clear interrupt source
  v = low_getfar(savage4.cmdsel, SubsystemCtl);
  low_setfar(savage4.cmdsel, SubsystemCtl, v | VsyncEna);

  if (v & 1)
    ++graph->vsync;

  if (savage4_isbusy(graph))
    return GRAPH_ERR_BUSY;

  return graph_schedule();
}


int savage4_rect(graph_t *graph, char *arg)
{
  unsigned int x = *(unsigned int *)arg;
  unsigned int y = *(unsigned int *)(arg + sizeof(int));
  unsigned int dx = *(unsigned int *)(arg + 2 * sizeof(int));
  unsigned int dy = *(unsigned int *)(arg + 3 * sizeof(int));
  unsigned int color = *(unsigned int *)(arg + 4 * sizeof(int));

  low_setfar(savage4.cmdsel, BciFifo, 0x4bcc8000 | 2);
  low_setfar(savage4.cmdsel, BciFifo + 4, color);
  low_setfar(savage4.cmdsel, BciFifo + 8, (y << 16) | x);
  low_setfar(savage4.cmdsel, BciFifo + 12, (dy << 16) | dx);

  return GRAPH_SUCCESS;
}


int savage4_move(graph_t *graph, char *arg)
{
  u32 cmd = 0x48cc0020;
  unsigned int x = *(unsigned int *)arg;
  unsigned int y = *(unsigned int *)(arg + sizeof(int));
  unsigned int dx = *(unsigned int *)(arg + 2 * sizeof(int));
  unsigned int dy = *(unsigned int *)(arg + 3 * sizeof(int));
  int mx = *(int *)(arg + 4 * sizeof(int));
  int my = *(int *)(arg + 5 * sizeof(int));
  
  if (mx <= 0)
    cmd |= 0x01000000;
  else
    x += dx - 1;

  if (my <= 0)
    cmd |= 0x02000000;
  else
    y += dy - 1;

  low_setfar(savage4.cmdsel, BciFifo, cmd);
  low_setfar(savage4.cmdsel, BciFifo + 4, (y << 16) | x);
  low_setfar(savage4.cmdsel, BciFifo + 8, ((y + my) << 16) | (x + mx));
  low_setfar(savage4.cmdsel, BciFifo + 12, (dy << 16) | dx);

  return GRAPH_SUCCESS;
}


int savage4_colorset(graph_t *graph, char *colors, unsigned char first, unsigned char last)
{
  int i;

  low_setfar8(savage4.cmdsel, LutIdx, first);

  for (i = 0; i <= (last - first); i++) {
    low_setfar8(savage4.cmdsel, LutData, (u8)colors[i * 3] >> 2);
    low_setfar8(savage4.cmdsel, LutData, (u8)colors[i * 3 + 1] >> 2);
    low_setfar8(savage4.cmdsel, LutData, (u8)colors[i * 3 + 2] >> 2);
  }

  return GRAPH_SUCCESS;
}


int savage4_colorget(graph_t *graph, char *colors, unsigned char first, unsigned char last)
{
  int i;

  low_setfar8(savage4.cmdsel, LutIdx - 1, first);

  for (i = 0; i <= (last - first); i++) {
    colors[i * 3] = low_getfar8(savage4.cmdsel, LutData) << 2;
    colors[i * 3 + 1] = low_getfar8(savage4.cmdsel, LutData) << 2;
    colors[i * 3 + 2] = low_getfar8(savage4.cmdsel, LutData) << 2;
  }

  return GRAPH_SUCCESS;
}


int savage4_cursorset(graph_t *graph, char *and, char *xor, unsigned char bg, unsigned char fg)
{
  int n;
  u32 offs;

  /* Reset color stack and set background */
  savage4_getcrt(0x45);
  savage4_setcrt(0x4b, bg);

  // Reset color stack and set foreground
  savage4_getcrt(0x45);
  savage4_setcrt(0x4a, fg);

  offs = graph->memsz - graph->cursorsz;

  for (n = 0; n < 256; ++n) {
    low_setfar16(graph->fbsel, offs + n * 4, ((short *)and)[n]);
    low_setfar16(graph->fbsel, offs + n * 4 + 2, ((short *)xor)[n]);
  }

  /* Set cursor buffer */
  savage4_setcrt(0x4c, offs >> 18);
  savage4_setcrt(0x4d, (offs >> 10) & 0xff);

  /* Set xoffs and yoffs */
  savage4_setcrt(0x4e, 0);
  savage4_setcrt(0x4f, 0);

  low_getfar(savage4.cmdsel, AltStatus0);

  return GRAPH_SUCCESS;
}


int savage4_cursorpos(graph_t *graph, unsigned int x, unsigned int y)
{
  savage4_setcrt(0x46, x >> 8);
  savage4_setcrt(0x47, x & 0xff);

  savage4_setcrt(0x49, y & 0xff);
  savage4_setcrt(0x48, y >> 8);

  return GRAPH_SUCCESS;
}


int savage4_cursorshow(graph_t *graph)
{
  /* Enable hardware cursor */
  savage4_setcrt(0x45, savage4_getcrt(0x45) | 1);

  return GRAPH_SUCCESS;
}


int savage4_cursorhide(graph_t *graph)
{
  /* Disable hardware cursor */
  savage4_setcrt(0x45, savage4_getcrt(0x45) & ~1);

  return GRAPH_SUCCESS;
}


int savage4_init(graph_t *graph)
{
  u32 cmd, fb;
  pci_id_t id = { VENDOR, DEVICE, 0 };
  pci_dev_t pdev;

  if (!pci_find(&id, 0, PCI_FIND_DEVICE, &pdev))
    return GRAPH_ABSENT;

  cmd = pdev.base[0] & ~0xf;
  fb = pdev.base[1] & ~0xf;

  savage4.irq = pdev.irq;

  savage4.cmdsel = low_dpmi_ldtalloc(1);
  low_dpmi_copydescr(low_getds(), savage4.cmdsel);
  low_dpmi_setbase(savage4.cmdsel, low_dpmi_phmap(cmd, CMDSZ));
  low_dpmi_setlimit(savage4.cmdsel, CMDSZ - 1);

  graph->fbsel = low_dpmi_ldtalloc(1);
  low_dpmi_copydescr(low_getds(), graph->fbsel);
  low_dpmi_setbase(graph->fbsel, low_dpmi_phmap(fb, MEMSZ));
  low_dpmi_setlimit(graph->fbsel, MEMSZ - 1);

  graph->open = savage4_open;
  graph->close = savage4_close;
  graph->mode = savage4_mode;
  graph->isbusy = savage4_isbusy;
  graph->trigger = savage4_trigger;
  graph->rect = savage4_rect;
  graph->move = savage4_move;
  graph->colorset = savage4_colorset;
  graph->colorget = savage4_colorget;
  graph->cursorset = savage4_cursorset;
  graph->cursorpos = savage4_cursorpos;
  graph->cursorshow = savage4_cursorshow;
  graph->cursorhide = savage4_cursorhide;

  graph->memsz = MEMSZ;
  graph->cursorsz = CURSORSZ;

  return GRAPH_SUCCESS;
}

