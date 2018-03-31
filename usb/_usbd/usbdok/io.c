/*
 * USB Disk On Key driver for MS-DOS
 * (c) IMMOS, 2003
 *
 * Simple stdio routines implementation
 */

#include <stdarg.h>

int io_strlen(char *s);


/* Function converts integer to string representation */
char *io_itoa(unsigned int i, char *buff)
{
  char *digits = "0123456789";
  int l = 0;
  int div = 10000;
  int offs = 0;
  int was = 0;

  div = 10000;
  while (div) {
    offs = i / div;
    if (offs)
      was = 1;

    if (was) {
      *(buff + l) = *(char *)(digits + offs);
      l++;
    }
    i -= offs * div;
    div /= 10; 
  }
  if (!l)
    *(buff + l++) = *(char *)(digits + offs);

  *(buff + l) = 0;
  return buff;
}


/* Function converts integer to hex string represenation */
char *io_itoah(unsigned int i, char *buff)
{
  char *digitsh = "0123456789abcdef";
  unsigned int l = 0;
  unsigned int offs = 0;
  int was = 0;
  unsigned int k;
  unsigned int v;

  v = i;

  for (k = 0; k < 4; k++) {
    offs = ((v >> (12 - 4 * k)) & 0x000f);
    if (offs)
      was = 1;
    if (was) {
      *(buff + l) = *(char *)(digitsh + offs);
      l++;
    }
  }
  if (!l)
    *(buff + l++) = *(char *)(digitsh + offs);
  *(buff + l) = 0;
  return buff;
}


unsigned int io_ahtoi(char *s)
{
  char *digitsh = "0123456789abcdef";
  char *p;
  char was = 0;
  unsigned int v = 0;
  unsigned char pow = 0;
  unsigned int i;
  int k;

  for (k = io_strlen(s) - 1; k >= 0; k--) {
    p = (char *)(s + k);
    if ((!was) && ((*p == ' ') || (*p == '\t')))
      continue;
    was = 0;
    for (i = 0; i < 16; i++)
      if (digitsh[i] == *(char *)p) {
        was = 1;
        break;
      }
    if (!was)
      return 0;

    if (pow > 3)
      return v;
    v += (i << (pow * 4));
    pow++;
  }
  return v;
}


int io_strlen(char *s)
{
  char *p;
  unsigned int k = 0;

  for (p = s; *p; p++)
    k++;
  return k;
}


int io_strncmp(char *s1, char *s2, unsigned int count)
{
  char *p;
  unsigned int k = 0;

  for (p = s1; *p; p++) {
    if ((*p != *(s2 + k)) && (k < count)) 
      return -1;
    k++;
  }
  if (k < count)
    return -1;
  return 0;
}


void io_memcpy(char *dest, char *src, unsigned int l)
{
  asm {
    mov cx, l
    mov ax, dest
    mov di, ax
    mov bx, src
    mov si, bx
    rep movsb
  }
  return;
}


void io_putc(char c)
{
  asm {
    xor   bh, bh
    mov   al, c         
    mov   cx, 01h
    mov   ah, 0eh
    int   10h
  }
  return;
}


void io_puts(char *s)
{
  char *p;

  for (p = s; *p; p++)
    io_putc(*p);
  return;
}


void io_printf(char *fmt, ...)
{
  va_list ap;
  char *p;
  char buff[10];

  va_start(ap, fmt);

  for (p = fmt; *p; p++) {
    if (*p != '%') {
      if (*p == '\n')
        io_putc('\r');
      io_putc(*p);
      continue;
    }

    switch (*++p) {
    case 'd':
      io_puts(io_itoa(va_arg(ap, int), buff));
      break;
    case 'x':
      io_puts(io_itoah(va_arg(ap, int), buff));
      break;
    case 's':
      io_puts(va_arg(ap, char *));
      break;
    case 'c':
      io_putc(va_arg(ap, char));
      break;
    case 'p':
      io_putc('%');
      break;
    }
  }
  va_end(ap);
  return;
}
