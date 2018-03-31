#
# Makefile for USB Disk On Key driver for MS-DOS
#
# (c) IMMOS, 2002-2003
#

CC=bcc
AS=tasm32
LD=tlink

CFLAGS = -c -IC:\BORLAND\INCLUDE;C:\Borland\INCLUDE;. -3 -mt -B
#-D_DEBUG -D_TRACE
ASFLAGS = /ml
LDFLAGS = /s

OBJS = usbdok.obj init.obj io.obj pci.obj low.obj isr.obj uhci.obj ohci.obj usb.obj umass.obj

usbdok.exe: $(OBJS)
  $(LD) $(LDFLAGS) $(OBJS)

usbdok.obj: usbdok.asm
  $(AS) $(ASFLAGS) usbdok.asm

init.obj: init.c
  $(CC) $(CFLAGS) init.c

io.obj: io.c
  $(CC) $(CFLAGS) io.c

pci.obj: pci.c
  $(CC) $(CFLAGS) pci.c

low.obj: low.c
  $(CC) $(CFLAGS) low.c

isr.obj: isr.asm
  $(AS) $(ASFLAGS) isr.asm

uhci.obj: uhci.c
  $(CC) $(CFLAGS) uhci.c

ohci.obj: ohci.c
  $(CC) $(CFLAGS) ohci.c

usb.obj: usb.c
  $(CC) $(CFLAGS) usb.c

umass.obj: umass.c
  $(CC) $(CFLAGS) umass.c

clean:
  del *.obj
