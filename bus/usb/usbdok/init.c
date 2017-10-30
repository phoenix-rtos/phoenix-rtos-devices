/*
 * USB Disk On Key driver for MS-DOS
 * (c) IMMOS, 2003, 2005, 2010
 *
 * Driver initialization
 */

#include "low.h"
#include "io.h"
#include "usb.h"
#include "umass.h"

#define PROMPT "USB Disk On Key dirver, (c) 2003, 2005, 2010 IMMOS"

void main(void)
{
  char buff[10240];
  int res = 0;
  u16 k;

  io_printf(PROMPT" (DEBUG)\n");

  if (usb_init() < 0) {
    io_printf("\nCan't initialize USB stack!");
    return;
  }

  for (k = 0; ;) {

    if ((res = umass_read(0, get_addr(buff), k, 10)) >= 0)
      io_printf("\r%d", k++);
    else
      io_printf("Read error, res=%d!\n", res);

    if (inb(0x60) == 0x81)
      break;
  }
  usb_done();

  return;
}


int init(void)
{
  unsigned int i;

  io_printf(PROMPT"\n");

  if (usb_init() < 0) {
    io_printf("\nCan't initialize USB stack!\n");
    return -1;
  }
  return 0;
}

