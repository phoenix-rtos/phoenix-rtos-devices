#
# Phoenix-RTOS
#
# ttypc - VT220 terminal emulator based on VGA and 101-key US keyboard
#
# Copyright 2017 Phoenix Systems
# Author: Pawel Pisarczyk
#

CC ?= i386-pc-phoenix-gcc
CFLAGS = -m32 -c -Wall -O2 -g -I. -I../../../libphoenix/ -DARCH=\"arch/ia32/arch.h\"

LD ?= i386-pc-phoenix-gcc
LDFLAGS = -m elf_i386 -nostdlib
#-z max-page-size=0x10000 - nostartfiles

SRCS = ttypc.c ttypc_virt.c ttypc_vtf.c ttypc_vga.c ttypc_kbd.c
OBJS = $(SRCS:.c=.o)
GCCLIB := $(shell $(CC) $(CFLAGS) -print-libgcc-file-name)
BIN = pc-tty

all: pc-tty

.c.o:
	$(CC) $(CFLAGS) $<

pc-tty: $(OBJS)
	$(LD) $(LDFLAGS) -o $(BIN) $(OBJS) ../../../libphoenix/libphoenix.a $(GCCLIB)

clean:
	rm -f *.o *~ core
