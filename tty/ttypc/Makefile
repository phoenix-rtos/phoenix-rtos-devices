#
# Phoenix-RTOS
#
# ttypc - VT220 terminal emulator based on VGA and 101-key US keyboard
#
# Copyright 2017 Phoenix Systems
# Author: Pawel Pisarczyk
#

CC = i386-pc-phoenix-gcc
CFLAGS = -c -Wall -O2 -g -I. -I../../../libphoenix/

LD = i386-pc-phoenix-gcc
LDFLAGS = -nostartfiles -nostdlib

SRCS = ttypc.c
OBJS = $(SRCS:.c=.o)
BIN = ttypc

all: ttypc

.c.o:
	$(CC) $(CFLAGS) $<

$(OBJS):

meterfs: $(OBJS)
	$(LD) $(LDFLAGS) -o $(BIN) $(OBJS) ../../../libphoenix/libphoenix.a

clean:
	rm -f *.o *~ core
