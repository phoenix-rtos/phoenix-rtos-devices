#
# Makefile for Phoenix-RTOS imx6ull-nandtool
#
# Copyright 2018 Phoenix Systems
#

SRCS = nandtool.c ../imx6ull-flash/flashdrv.c
OBJS = $(SRCS:.c=.o)
BIN = nandtool

CFLAGS += -DLIBFLASHDRV -I. -I../../../libphoenix -I../../../phoenix-rtos-kernel/include

ifeq ($(CONFIG_NANDTOOL_TEST), 1)
	CFLAGS += -DCONFIG_NANDTOOL_TEST
	SRCS += test.c
endif

all: check $(BIN)


check:
	@(if test -z "$(TARGET)"; then\
		echo "Error: Only main Makefile can be used for compilation!";\
		exit 1;\
	fi;)


.c.o:
	@(printf " CC    %-24s  \n" "$<"; $(CC) -c $(CFLAGS) $<)

-include .depend


$(BIN): $(OBJS)
	@echo " LD    $@"
	$(SIL)$(LD) $(LDFLAGS) -o $(BIN) $(notdir $(OBJS)) ../../../libphoenix/libphoenix.a $(GCCLIB)
	@echo " STRIP $@"
	$(SIL)$(STRIP) $(BIN) 2>/dev/null

depend: check
	$(SIL)$(MKDEP) $(MKDEPFLAGS) $(SRCS) >.depend


clean: check
	$(SIL)rm -f core *.o $(ARCH)


.PHONY: clean
# DO NOT DELETE
