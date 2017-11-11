#
# Phoenix-RTOS
#
# pc-uart - PC serial port driver
#
# Copyright 2017 Phoenix Systems
# Author: Pawel Pisarczyk
#

CC = i386-pc-phoenix-gcc
CFLAGS = -c -Wall -O2 -g -I. -I../../../libphoenix/ -DARCH=\"arch/ia32/arch.h\"

LD = i386-pc-phoenix-gcc
LDFLAGS = -nostartfiles -nostdlib

SRCS = pc-uart.c
OBJS = $(SRCS:.c=.o)
GCCLIB := $(shell $(CC) $(CFLAGS) -print-libgcc-file-name)
BIN = pc-uart

all: pc-uart

.c.o:
	$(CC) $(CFLAGS) $<

ttypc: $(OBJS)
	$(LD) $(LDFLAGS) -o $(BIN) $(OBJS) ../../../libphoenix/libphoenix.a $(GCCLIB)

clean:
	rm -f *.o *~ core
