#
# Phoenix-RTOS
#
# pc-uart - PC serial port driver
#
# Copyright 2017 Phoenix Systems
# Author: Pawel Pisarczyk
#
CROSS ?= i386-pc-phoenix-

CC = $(CROSS)gcc
CFLAGS = -m32 -fstack-protector -c -Wall -O2 -g\
	 -I. -I../../../../libphoenix/ -I../../../../phoenix-rtos-kernel/src\
	 -DARCH=\"arch/ia32/arch.h\"\
	 -DHAL=\"hal//ia32//hal.h\"


LD = $(CROSS)gcc
LDFLAGS = -m32 -nostdlib -nostartfiles -static

SRCS = pc-pci.c
OBJS = $(SRCS:.c=.o)
GCCLIB := $(shell $(CC) $(CFLAGS) -print-libgcc-file-name)
BIN = pc-pci

all: pc-pci test_pci

.c.o:
	$(CC) $(CFLAGS) $<

pc-pci: $(OBJS)
	$(LD) $(LDFLAGS) -o $(BIN) $(OBJS) ../../../../libphoenix/libphoenix.a $(GCCLIB)

clean:
	rm -f *.o *~ core
