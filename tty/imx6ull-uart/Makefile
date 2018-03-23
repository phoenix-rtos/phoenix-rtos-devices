#
# Phoenix-RTOS
#
# uart  - i.MX6ULL uart driver
#
# Copyright 2018 Phoenix Systems
# Author: Kamil Amanowicz
#

CC = arm-phoenix-gcc
CFLAGS = -mthumb -c -Wall -Os -nostartfiles -nostdlib -mno-unaligned-access -mcpu=cortex-a7 -mtune=cortex-a7 -mfloat-abi=hard -fomit-frame-pointer -ffreestanding -I. -I../../../libphoenix/ -I../../../phoenix-rtos-kernel/src/ -DARCH=\"arch/arm-imx/arch.h\"

LD = arm-phoenix-gcc
LDFLAGS = -nostdlib -z max-page-size=0x1000

STRIP = arm-phoenix-strip

SRCS = uartdrv.c
OBJS = $(SRCS:.c=.o)

LIBPHOENIX = ../../../libphoenix/libphoenix.a
GCCLIB := $(shell $(CC) $(CFLAGS) -print-libgcc-file-name)
BIN = imx-uart

all: imx-uart imx-uart.debug

.c.o:
	$(CC) $(CFLAGS) $<

imx-uart: $(OBJS) $(LIBPHOENIX)
	$(LD) $(LDFLAGS) -o $(BIN) $(OBJS) $(LIBPHOENIX) $(GCCLIB)
	$(STRIP) $(BIN)

imx-uart.debug: $(OBJS) $(LIBPHOENIX)
	$(LD) $(LDFLAGS) -o $(BIN).debug $(OBJS) $(LIBPHOENIX) $(GCCLIB)

clean:
	rm -f *.o *~ core
