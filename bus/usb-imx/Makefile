#
# Phoenix-RTOS
#
# dummyfs - dummy filesystem driver
#
# Copyright 2018 Phoenix Systems
# Author: Pawel Pisarczyk
#

CC = arm-phoenix-gcc
CFLAGS = -mthumb -c -Wall -Os -nostartfiles -nostdlib -mno-unaligned-access -mcpu=cortex-a7 -mtune=cortex-a7 -mfloat-abi=hard -fomit-frame-pointer -ffreestanding -I. -I../../../libphoenix/ -I../../../phoenix-rtos-kernel/src/ -DARCH=\"arch/arm-imx/arch.h\"

LD = arm-phoenix-gcc
LDFLAGS = -nostdlib -z max-page-size=0x1000

STRIP = arm-phoenix-strip

SRCS = usb.c
OBJS = $(SRCS:.c=.o)

LIBPHOENIX = ../../../libphoenix/libphoenix.a
GCCLIB := $(shell $(CC) $(CFLAGS) -print-libgcc-file-name)
BIN = usb-imx

all: usb-imx usb-imx.debug

.c.o:
	$(CC) $(CFLAGS) $<

usb-imx: $(OBJS) $(LIBPHOENIX)
	$(LD) $(LDFLAGS) -o $(BIN) $(OBJS) $(LIBPHOENIX) $(GCCLIB)
	$(STRIP) $(BIN)

usb-imx.debug: $(OBJS) $(LIBPHOENIX)
	$(LD) $(LDFLAGS) -o $(BIN).debug $(OBJS) $(LIBPHOENIX) $(GCCLIB)

clean:
	rm -f *.o *~ core
