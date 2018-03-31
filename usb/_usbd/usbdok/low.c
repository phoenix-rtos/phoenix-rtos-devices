/*
 * USB Disk On Key driver for MS-DOS
 * (c) IMMOS, 2003
 *
 * Low level routines implementation
 */

#include "low.h"

extern u16 _isr_func;
extern void *_isr_stub;


/* Function enables interrupt in interrupt controler */
void low_enable_irq(u8 irq)
{
  u8 mask8259;

  if (irq < 8) {
    mask8259 = inb(0x21);
    outb(0x21, mask8259 & (~(1 << irq)));
  }
  else {
    mask8259 = inb(0xa1);
    outb(0xa1, mask8259 & (~(1 << (irq - 8))));
  }        
  return;
}


/* Function disables interrupt in interrupt controler */
void low_disable_irq(u8 irq)
{
  u8 mask8259;

  if (irq < 8) {
    mask8259 = inb(0x21);
    outb(0x21, mask8259 | (1 << irq));
  }
  else {
    mask8259 = inb(0xa1);
    outb(0xa1, mask8259 | (1 << (irq - 8)));
  }
  return;
}


/* Function registers interrupt service routine */
void low_register_irq(u8 irq, void *isr)
{
  low_disable_irq(irq);
  asm {
    mov ax, isr
    mov cs:[_isr_func], ax
    mov ax, cs
    mov ds, ax
    mov dx, offset _isr_stub
    mov ah, 25h
    mov al, irq
    add al, 8
    cmp al, 16
    jc  low_regisr1
    add al, 60h
  }
low_regisr1:
  asm {
    int 21h
    mov ax, es
    mov ds, ax
  }
  low_enable_irq(irq);
}


void low_isr_get(u8 irq, u16 *segment, u16 *offset)
{
  u16 s, o;
  irq += (irq < 8) ? 0x8 : 0x68;

  asm {
    mov ah, 35h
    mov al, irq
    int 21h
    mov o, bx
    mov ax, es
    mov s, ax
    mov ax, ds
    mov es, ax
  }

  *segment = s;
  *offset = o;
}


void low_isr_set(u8 irq, u16 *segment, u16 *offset)
{
  u16 s = *segment, o = *offset;
  irq += (irq < 8) ? 0x8 : 0x68;

  asm {
    mov ax, s
    mov ds, ax
    mov dx, o
    mov ah, 25h
    mov al, irq
    int 21h
    mov ax, es
    mov ds, ax
  }
}

