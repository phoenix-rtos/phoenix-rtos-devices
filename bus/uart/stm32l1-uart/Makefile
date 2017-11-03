#
# Makefile for libphoenix
#
# Copyright 2017 Phoenix Systems
#
# %LICENSE%
#

SIL ?= @

SRCDIR=$(CURDIR)
LIBPHOENIX ?= ../../../../libphoenix

CROSS ?= arm-phoenix-

CC = $(CROSS)gcc
CFLAGS += -O0 -Wall -Wstrict-prototypes -g -I$(SRCDIR) -I../../gpio/stm32l1-gpio -I$(LIBPHOENIX) -nostartfiles -nostdlib\
	-mcpu=cortex-m3 -mthumb \
	-fomit-frame-pointer -ffreestanding\
	-fpic -fpie -msingle-pic-base -mno-pic-data-is-text-relative\
	-DARCH=\"arch/armv7/arch.h\" -DNOMMU

AR = $(CROSS)ar
ARFLAGS = -r


all: uartdrv.a

.c.o:
	$(SIL)(printf "CC  %-24s  \n" "$<"; $(CC) -c $(CFLAGS) $<)

.o.a:
	$(SIL)$(AR) $(ARFLAGS) -o $@ $<

clean:
	$(SIL)rm -f *.o *.a

.PHONY: clean
# DO NOT DELETE
