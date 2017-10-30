/*
 * USB Disk On Key driver for MS-DOS
 * (c) IMMOS, 2003, 2005
 *
 * Low level routines
 */

#ifndef _LOW_H_
#define _LOW_H_


typedef unsigned char u8;
typedef unsigned int u16;
typedef unsigned long u32;


static u8 inb(u16 port)
{
  asm {
    mov dx, port
    in al, dx
  }
  return _AL;
}

static void outb(u16 port, u8 val)
{
  asm {
    mov dx, port
    mov al, val
    out dx, al
  }
}

static u16 inw(u16 port)
{
  asm {
    mov dx, port
    in ax, dx
  }
  return _AX;
}

static void outw(u16 port, u16 val)
{
  asm {
    mov dx, port
    mov ax, val
    out dx, ax
  }
}

static u32 inl(u16 port)
{
  asm {
    mov dx, port
    in  eax, dx
  }
  return _EAX;
}

static void outl(u16 port, u32 val)
{
  asm {
    mov dx, port
    mov eax, val
    out dx, eax
  }
}


static u32 get_addr(void *p)
{
  u32 addr;

  asm {
    xor eax,  eax
    mov ax,   ds
    shl eax,  4
    mov addr, eax
  }
  addr += (u16)p;
  return addr;
}


static void *get_offs(u32 addr)
{
  u32 s;

  asm {
    xor eax, eax
    mov ax, ds
    shl eax, 4
    mov s, eax
  }
  return (void *)(addr - s);
}


static void memcpy2addr(u32 dst, void *src, u16 len)
{
  asm {
    push es
    mov eax, dst
    mov di, ax
    and di, 0xf
    shr eax, 4
    mov es, ax
    mov si, src
    mov cx, len
    rep
    movsb
    pop es
  }
}


static void low_intack(void)
{
  asm {
    mov al, 0bh
    out (20h), al
    in  al, (20h)
    and al, 07h
    cmp al, 04h
    mov al, 20h
    jnz __low_intack1
    out (0a0h), al
  }
__low_intack1:
  asm {
    out (20h), al
  }
  return;
}


/* Function registers interrupt service routine */
extern void low_register_irq(u8 irq, void *isr);

extern void low_isr_get(u8 irq, u16 *segment, u16 *offset);

extern void low_isr_set(u8 irq, u16 *segment, u16 *offset);


#endif
