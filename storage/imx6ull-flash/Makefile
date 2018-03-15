#
# Makefile for IMX6ULL flash driver
#
# Copyright 2017 Phoenix Systems
#
# %LICENSE%
#

SIL ?= @

CROSS ?= arm-phoenix-
SUBDIRS = arch/arm-imx $(SUBSYSTEMS)

MKDEP = $(CROSS)gcc -MM
MKDEPFLAGS = $(CFLAGS)

CC = $(CROSS)gcc

SRCDIR=$(CURDIR)
LIBPHOENIX=../../../libphoenix
LIB=$(LIBPHOENIX)/libphoenix.a
KERNELHEADERS=../../../phoenix-rtos-kernel/include

CFLAGS += -Os -Wall -Wstrict-prototypes -g -I$(SRCDIR) -I$(LIBPHOENIX) -I$(KERNELHEADERS) -nostartfiles -nostdlib\
	-mcpu=cortex-a7 -mtune=cortex-a7 -mfloat-abi=hard -mthumb\
	-fomit-frame-pointer -ffreestanding -mno-unaligned-access\
	-DVERSION=\"$(VERSION)\" -DARCH=\"arch/arm-imx/arch.h\"

AR = $(CROSS)ar
ARFLAGS = -r

LD = $(CROSS)ld
LDFLAGS = -nostdlib -z max-page-size=0x1000
GCCLIB := $(shell $(CC) $(CFLAGS) -print-libgcc-file-name)

OBJCOPY = $(CROSS)objcopy
OBJDUMP = $(CROSS)objdump
STRIP = $(CROSS)strip

OBJECTS = flashdrv.o

all: flashdrv.elf

flashdrv.elf: $(OBJECTS)
	$(SIL)(printf "LD  %-24s  \n" "$(OBJECTS)"; $(LD) $(LDFLAGS) -o $@_unstripped $(OBJECTS) $(LIB) $(GCCLIB))
	$(SIL)($(STRIP) $@_unstripped -o $@)

.c.o:
	$(SIL)(printf "CC  %-24s  \n" "$<"; $(CC) -c $(CFLAGS) $<)

clean:
	$(SIL)rm -f *.o *.elf

.PHONY: clean
# DO NOT DELETE
