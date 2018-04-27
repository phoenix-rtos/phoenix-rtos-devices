#
# Makefile for Phoenix-RTOS imx6ull-uart driver
#
# Copyright 2018 Phoenix Systems
#

SRCS = imx6ull-gpio.c
OBJS = $(SRCS:.c=.o)
BIN = imx6ull-gpio

CFLAGS += -I. -I../../../libphoenix


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
	@cp $(BIN) $(BIN)-unstriped
	@echo " STRIP $@"
	$(SIL)$(STRIP) $(BIN) 2>/dev/null


depend: check
	$(SIL)$(MKDEP) $(MKDEPFLAGS) $(SRCS) >.depend


clean: check
	$(SIL)rm -f core *.o $(ARCH)


.PHONY: clean
# DO NOT DELETE
