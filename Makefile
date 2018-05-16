#
# Makefile for phoenix-rtos-devices
#
# Copyright 2018 Phoenix Systems
#
# %LICENSE%
#

SIL ?= @
MAKEFLAGS += --no-print-directory

TARGET ?= ia32-qemu
#TARGET ?= armv7-stm32-tiramisu
#TARGET ?= arm-imx

VERSION = 0.2
SRCDIR := $(CURDIR)

include Makefile.ia32
include Makefile.armv7
include Makefile.arm


export SIL TARGET CC CFLAGS MKDEP MKDEPFLAGS AR ARFLAGS LD LDFLAGS LDLIBS OBJDUMP STRIP


all: drivers


drivers:
	@for i in $(SUBDIRS); do\
		d=`pwd`;\
		echo "\033[1;32mCOMPILE $$i\033[0m";\
		if ! cd $$i; then\
			exit 1;\
		fi;\
		if ! make; then\
			exit 1;\
		fi;\
		cd $$d;\
	done;


depend:
	@for i in $(SUBDIRS) test; do\
		d=`pwd`;\
		echo "DEPEND $$i";\
		if ! cd $$i; then\
			exit 1;\
		fi;\
		if ! make -s depend; then\
			exit 1;\
		fi;\
		cd $$d;\
	done;


clean:
	@rm -f core *.o $(LIB)
	@for i in $(SUBDIRS); do\
		d=`pwd`;\
		echo "CLEAN $$i";\
		if ! cd $$i; then\
			exit 1;\
		fi;\
		if ! make clean; then\
			exit 1;\
		fi;\
		cd $$d;\
	done;


.PHONY: clean
# DO NOT DELETE
