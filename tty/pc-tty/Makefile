#
# Makefile for Phoenix-RTOS pc-tty drivers
#
# Copyright 2017, 2018 Phoenix Systems
# Copyright 2001, 2005-2006 Pawel Pisarczyk
#

SRCS = ttypc.c ttypc_virt.c ttypc_vtf.c ttypc_vga.c ttypc_kbd.c
OBJS = $(SRCS:.c=.o)
BIN = pc-tty

CFLAGS += -I. -I../../../libphoenix/ -DARCH=\"arch/ia32/arch.h\"


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
	$(SIL)$(LD) -o $(BIN) $(OBJS) ../../../libphoenix/libphoenix.a $(GCCLIB)
	@echo " STRIP $@"
	$(SIL)$(STRIP) $(BIN) 2>/dev/null


depend: check
	$(SIL)$(MKDEP) $(MKDEPFLAGS) $(SRCS) >.depend


clean: check
	$(SIL)rm -f core *.o $(ARCH)


.PHONY: clean
# DO NOT DELETE
