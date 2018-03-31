/*
 * USB Disk On Key driver for MS-DOS
 * (c) IMMOS, 2003, 2005
 *
 * USB mass storage driver
 */

#ifndef _UMASS_H_
#define _UMASS_H_


#define UMASS_SECSZ  512

#define BULK_WRITE 0
#define BULK_READ  0x80

#define STATE_READY    0
#define STATE_REMOVED -1
#define STATE_CHANGED -2
#define STATE_ERROR   -4
#define STATE_HALT    -5

#define CBW_SIG 0x43425355
#define CSW_SIG 0x53425355


typedef struct _bulk_cbw_t {
  u32 sig;
  u32 tag;
  u32 dlen;
  u8  flags;
  u8  lun;
  u8  clen;
  u8  cmd[16];
} bulk_cbw_t;


typedef struct _bulk_csw_t {
  u32 sig;
  u32 tag;
  u32 dr;
  u8  status;
} bulk_csw_t;


/* Function reads lbc number of sector starting from block lbn */
extern int umass_read(int port, u32 buffer, u32 lbn, u16 lbc);


/* Function writes lbc number of sector starting from block lbn */
extern int umass_write(int port, u32 buffer, u32 lbn, u16 lbc);


/* Function returns disk capacity */
extern int umass_capacity(int port, u32 *bn);


/* Function configures newly inserted disk */
extern int umass_insertion(int port, urb_t *urbs);


#endif
