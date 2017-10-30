/*
 * USB Disk On Key driver for MS-DOS
 * (c) IMMOS, 2003, 2005
 *
 * USB mass storage driver implementation
 */

#include "io.h"
#include "usb.h"
#include "umass.h"


#define MAX_SECS          32
#define MAX_USB_RETRIES   2
#define MAX_SCSI_RETRIES  4

#define htonl(l)  ((l >> 24) | (l << 24) | ((l >> 8) & 0x0ff00) | ((l << 8) & 0x0ff0000))
#define htons(s)  ((s >> 8) | (s << 8))


struct {
  urb_t      *urbs[2];
  u32        part_offset[2];
  u32        part_size[2];
  bulk_cbw_t cbw;
  bulk_csw_t csw;
  u32        cbulk[2];
  u8         mbr[UMASS_SECSZ * 2];
  u8         buff[UMASS_SECSZ * 2];
  ctlreq_t   req;
} umass;


int umass_reset(urb_t *urb)
{
  int res;

  urb->toggle = 0;
  umass.req.bmRequestType = 0x21;
  umass.req.bRequest = 0xff;
  umass.req.wValue = 0;
  umass.req.wIndex = 1;
  umass.req.wLength = 0;
  if ((res = usb_transmit(urb, pidSETUP, get_addr((u8 *)&umass.req), sizeof(ctlreq_t))) < 0)
    return res;
  if ((res = usb_transmit(urb, pidIN, NULL, 0)) < 0)
    return res;

  return 0;
}


int bulk_transport(int port, u8 *cmd, u16 clen, u32 data, u16 dlen, int dir)
{
  int res, retry;
  u16 st;

  if (port & ~1)
    return ERR_USB_PORT;
  if (clen > 16)
    return ERR_UMASS_IO;

  umass.cbw.sig = CBW_SIG;
  umass.cbw.tag = umass.cbulk[port]++;
  umass.cbw.dlen = dlen;
  umass.cbw.flags = dir;
  umass.cbw.lun = 0;
  umass.cbw.clen = clen;
  io_memcpy(umass.cbw.cmd, cmd, clen);

  for (retry = 0;;retry++) {
    if (retry == MAX_USB_RETRIES)
      return res;

#ifdef _TRACE
    io_printf("bulk_transport: port=%d CSB\n", port);
#endif

    if ((res = usb_transmit(&(umass.urbs[port])[2], pidOUT, get_addr((u8 *)&umass.cbw), sizeof(bulk_cbw_t))) < 0) {
      if (res == ERR_USB_STALL)
        res = usb_bulk_reset(&(umass.urbs[port])[0], &(umass.urbs[port])[2]);
      if (res >= 0)
        continue;

#ifdef _TRACE
      io_printf("bulk_transport: port=%d CSB ERROR, res=%d\n", port, res);
#endif
      return res;
    }

    if (dlen > 0) {
#ifdef _TRACE
      io_printf("bulk_transport: port=%d DATA\n", port);
#endif

      if (dir == BULK_READ) {
        if ((res = usb_transmit(&(umass.urbs[port])[1], pidIN, data, dlen)) < 0) {
          if (res == ERR_USB_STALL)
            res = usb_bulk_reset(&(umass.urbs[port])[0], &(umass.urbs[port])[1]);
          if (res >= 0)
            continue;
#ifdef _TRACE
          io_printf("bulk_transport: port=%d DATA ERROR, res=%d\n", port, res);
#endif
          return res;
        }
      }
      else {
        if ((res = usb_transmit(&(umass.urbs[port])[2], pidOUT, data, dlen)) < 0) {
          if (res == ERR_USB_STALL)
            res = usb_bulk_reset(&(umass.urbs[port])[0], &(umass.urbs[port])[2]);
          if (res >= 0)
            continue;
#ifdef _TRACE
          io_printf("bulk_transport: port=%d DATA ERROR, res=%d\n", port, res);
#endif
          return res;
        }
      }
    }

    /* Read CSW */
#ifdef _TRACE
    io_printf("bulk_transport: port=%d CSW\n", port);
#endif
    if ((res = usb_transmit(&(umass.urbs[port])[1], pidIN, get_addr((u8 *)&umass.csw), sizeof(bulk_csw_t))) < 0) {
      if (res == ERR_USB_STALL) {
#ifdef _TRACE
        io_printf("bulk_transport: port=%d endpoint stalled\n", port);
#endif
        res = usb_bulk_reset(&(umass.urbs[port])[0], &(umass.urbs[port])[1]);
        if (res >= 0)
          res = usb_transmit(&(umass.urbs[port])[1], pidIN, get_addr((u8 *)&umass.csw), sizeof(bulk_csw_t));
      }
      if (res >= 0)
        continue;
#ifdef _TRACE
      io_printf("bulk_transport: port=%d CSW ERROR\n", port);
#endif
      return res;
    }

    if (umass.csw.sig != (u32)CSW_SIG) {
#ifdef _TRACE
      io_printf("bulk_transport: port=%d CSW SIG ERROR\n", port);
#endif
      return res;
    }
    break;
  }

  if (umass.csw.status || umass.csw.dr) {
#ifdef _TRACE
    io_printf("bulk_transport: port=%d CSW STATUS ERROR, status=%d, dr=%d\n", port, umass.csw.status, umass.csw.dr);
#endif
    return ERR_UMASS_IO;
  }

  return 0;
}


int umass_chk(int port)
{
  int n, res;
  static u8 scsi_test[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };  
  static u8 scsi_mbr[16] = { 0x28, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0 };

#ifdef _TRACE
  io_printf("umass_chk: port=%d\n", port);
#endif

  if (!usb_check_hub(port)) {
    umass.part_size[port] = 0;
    return ERR_UMASS_IO;
  }

  /* Test if disk is ready */
  if (umass.part_size[port])
    return 0;

  for (n = 0; n < MAX_SCSI_RETRIES; n++) {
    res = bulk_transport(port, scsi_test, 16, NULL, 0, BULK_READ);
    if (res == 0)
      break;
    usb_sleep(500);
  }
  if (res < 0)
    return res;

  if ((res = bulk_transport(port, scsi_mbr, 16, get_addr((u8 *)&umass.mbr), UMASS_SECSZ * 2, BULK_READ)) < 0)
    return res;

  if (*(u16 *)(umass.mbr + 0x1fe) == 0xaa55) {

    if (*(u8 *)(umass.mbr + 0x1be) == 0x80) {
      umass.part_offset[port] = *(u32 *)(umass.mbr + 0x1c6);
      umass.part_size[port] = *(u32 *)(umass.mbr + 0x1ca);
    }
    else {
      umass.part_offset[port] = 0;
      umass.part_size[port] = *(u16 *)(umass.mbr + 0x13);
      if (!umass.part_size[port])
        umass.part_size[port] = *(u32 *)(umass.mbr + 0x20);
    }

#ifdef _TRACE
  io_printf("umass_chk: port=%d, device checked\n", port);
#endif

    return 0;
  }

  return ERR_UMASS_IO;
}


int umass_read(int port, u32 buffer, u32 lbn, u16 lbc)
{
  int res;
  u16 len;
  static u8 scsi_read[16] = { 0x28, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

#ifdef _TRACE
  io_printf("umass_read: port=%d, lbn=%d, lbc=%d\n", port, lbn, lbc);
#endif
  if ((res = umass_chk(port)) < 0)
    return res;

  if ((lbn + lbc) > umass.part_size[port]) {
#ifdef _TRACE
    io_printf("umass_read: umass.part_size[%d]=%d\n", port, umass.part_size[port]);
#endif
    return ERR_UMASS_IO;
  }

  while (lbc) {
    len = (lbc > MAX_SECS) ? MAX_SECS : lbc;
    *(u32 *)(scsi_read + 2) = htonl(lbn + umass.part_offset[port]);

    if (lbc > 1) {
      *(u16 *)(scsi_read + 7) = htons(len);
      if((res = bulk_transport(port, scsi_read, 16, buffer, UMASS_SECSZ * len, BULK_READ)) < 0)
        return res;
    } else {
      *(u16 *)(scsi_read + 7) = htons(2);
      if((res = bulk_transport(port, scsi_read, 16, get_addr((u8 *)&umass.buff), UMASS_SECSZ * 2, BULK_READ)) < 0)
        return res;
      memcpy2addr(buffer, (u8 *)&umass.buff, UMASS_SECSZ);
    }

    lbn += MAX_SECS;
    lbc -= len;
    buffer += UMASS_SECSZ * len;
  }
  return 0;
}


int umass_write(int port, u32 buffer, u32 lbn, u16 lbc)
{
  int res;
  u16 len;
  static u8 scsi_write[16] = { 0x2a, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

#ifdef _TRACE
  io_printf("umass_write: port=%d\n", port);
#endif
  if ((res = umass_chk(port)) < 0)
    return res;

  if ((lbn + lbc) > umass.part_size[port])
    return ERR_UMASS_IO;

  while(lbc) {
    len = (lbc > MAX_SECS) ? MAX_SECS : lbc;
    *(u32 *)(scsi_write + 2) = htonl(lbn + umass.part_offset[port]);
    *(u16 *)(scsi_write + 7) = htons(len);
    if((res = bulk_transport(port, scsi_write, 16, buffer, UMASS_SECSZ * len, BULK_WRITE)) < 0)
      return res;
    lbn += MAX_SECS;
    lbc -= len;
    buffer += UMASS_SECSZ * len;
  }
  return 0;
}


int umass_capacity(int port, u32 *bn)
{
  int res;

#ifdef _TRACE
  io_printf("umass_capacity: port=%d\n", port);
#endif
  *bn = umass.part_size[port];
	return 0;
}


/* Function configures newly inserted device */
int umass_insertion(int port, urb_t *urbs)
{
  int res;

#ifdef _DEBUG
  io_printf("USB mass storage attached\n");
  io_printf("  port=%d address=%d, endpoints=[%d:%d] [%d:%d], [%d:%d]\n", port, urbs[0].dev,
         urbs[0].ep, urbs[0].mtu, urbs[1].ep, urbs[1].mtu, urbs[2].ep, urbs[2].mtu);
#endif

  umass.urbs[port] = urbs;
  umass.cbulk[port] = 0;
  urbs[1].toggle = 0;
  urbs[2].toggle = 0;

  return 0;
}


int umass_state(int port)
{
#ifdef _TRACE
  io_printf("umass_state: port=%d\n", port);
#endif
  return (umass_chk(port) >= 0);
}
