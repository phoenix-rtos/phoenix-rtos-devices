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
CFLAGS = -m32 -fno-pie -c -Wall -O2 -g\
	 -I. -I../../../../../libphoenix/\
	 -DARCH=\"arch/ia32/arch.h\"\
	 -DHAL=\"hal//ia32//hal.h\"


LD = $(CROSS)gcc
LDFLAGS = -m32 -nostdlib -nostartfiles

SRCS = test.c
OBJS = $(SRCS:.c=.o)
GCCLIB := $(shell $(CC) $(CFLAGS) -print-libgcc-file-name)
BIN = test-pci

all: test_pci

.c.o:
	$(CC) $(CFLAGS) $<

test_pci: $(OBJS)
	$(LD) $(LDFLAGS) -o $(BIN) $(OBJS) ../../../../../libphoenix/libphoenix.a $(GCCLIB)

clean:
	rm -f *.o *~ core
