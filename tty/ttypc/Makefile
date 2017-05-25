#
# ttypc driver
# Copyright 2017 Pawel Pisarczyk
#

CC = i386-pc-phoenix-gcc
CFLAGS = -c -Wall -I . -O2 -g

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
	$(LD) $(LDFLAGS) -o $(BIN) $(OBJS)

clean:
	rm -f *.o *~ core
