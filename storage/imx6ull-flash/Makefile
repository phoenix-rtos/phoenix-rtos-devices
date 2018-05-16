#
# Makefile for Phoenix-RTOS imx6ull-uart driver
#
# Copyright 2018 Phoenix Systems
#

SRCS = flashdrv.c
OBJS = $(SRCS:.c=.o)
BIN = flashdrv

CFLAGS += -I. -I../../../libphoenix -I../../../phoenix-rtos-kernel/include


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
	$(SIL)$(LD) $(LDFLAGS) -o $(BIN) $(OBJS) ../../../libphoenix/libphoenix.a $(GCCLIB)
	@echo " STRIP $@"
	$(SIL)$(STRIP) $(BIN) 2>/dev/null
	$(SIL)rm -f core *.o $(ARCH)


depend: check
	$(SIL)$(MKDEP) $(MKDEPFLAGS) $(SRCS) >.depend


clean: check
	$(SIL)rm -f core *.o $(ARCH)


.PHONY: clean
# DO NOT DELETE
