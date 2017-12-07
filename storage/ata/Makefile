#
# Phoenix-RTOS
#
# descriptions
#
# Copyright 2017 Phoenix Systems
# Author: Pawel Pisarczyk
#

CC = gcc
CFLAGS = -m32 -c -Wall -O2 -g -I. -I../../../libphoenix/ -DARCH=\"arch/ia32/arch.h\"\
			-DHAL=\"hal//ia32//hal.h\" \
			-I../../../phoenix-rtos-kernel/src/ -I../../bus/pci/pc-pci/

LD = gcc
LDFLAGS = -m32 -nostdlib -nostartfiles -static
#-z max-page-size=0x10000

SRCS = pc-ata.c
OBJS = $(SRCS:.c=.o)
GCCLIB := $(shell $(CC) $(CFLAGS) -print-libgcc-file-name)
BIN = pc-ata

all: pc-ata

.c.o:
	$(CC) $(CFLAGS) $<

pc-ata: $(OBJS)
	$(LD) $(LDFLAGS) -o $(BIN) $(OBJS) ../../../libphoenix/libphoenix.a $(GCCLIB)

clean:
	rm -f *.o *~ core
