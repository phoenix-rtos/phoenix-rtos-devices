/*
 * USB Disk On Key driver for MS-DOS
 * (c) IMMOS, 2003
 *
 * Simple stdio routines
 */

#ifndef _IO_H_
#define _IO_H_


#define NULL    0


/* Function converts integer to string representation */
extern char *io_itoa(unsigned int i, char *buff);


/* Function converts integer to hex string represenation */
extern char *io_itoah(unsigned int i, char *buff);


extern unsigned int io_ahtoi(char *s);


extern int io_strlen(char *s);


extern int io_strncmp(char *s1, char *s2, unsigned int count);


extern void io_memcpy(char *dest, char *src, unsigned int l);


extern void io_putc(char c);


extern void io_puts(char *s);


extern void io_printf(char *fmt, ...);


#endif
