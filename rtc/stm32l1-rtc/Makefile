#
# Makefile for libphoenix
#
# Copyright 2017 Phoenix Systems
#
# %LICENSE%
#

SIL ?= @

SRCDIR=$(CURDIR)
LIBPHOENIX ?= ../../../libphoenix
LIB = $(LIBPHOENIX)/libphoenix.a

CROSS ?= arm-phoenix-

CC = $(CROSS)gcc
CFLAGS += -O2 -Wall -Wstrict-prototypes -g -I$(SRCDIR) -I$(LIBPHOENIX) -nostartfiles -nostdlib\
	-mcpu=cortex-m3 -mthumb \
	-fomit-frame-pointer -ffreestanding\
	-fpic -fpie -msingle-pic-base -mno-pic-data-is-text-relative\
	-DARCH=\"arch/armv7/arch.h\" -DNOMMU

LD = $(CROSS)ld
LDFLAGS = -nostdlib -e _start --section-start .init=8000000 --section-start .text=0 -Tbss=20000000 -z max-page-size=0x10
GCCLIB := $(shell $(CC) $(CFLAGS) -print-libgcc-file-name)


all: rtcdrv.elf

%.elf: %.o
	$(SIL)(printf "LD  %-24s  \n" "$<"; $(LD) $(LDFLAGS) -o $@ $< $(LIB) $(GCCLIB))

.c.o:
	$(SIL)(printf "CC  %-24s  \n" "$<"; $(CC) -c $(CFLAGS) $<)

clean:
	$(SIL)rm -f *.o *.elf

.PHONY: clean
# DO NOT DELETE
