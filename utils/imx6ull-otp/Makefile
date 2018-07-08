#
# Makefile for Phoenix-RTOS imx6ull-nandtool
#
# Copyright 2018 Phoenix Systems
#

SRCS = fuser.c
OBJS = $(SRCS:.c=.o)
LIBS := fuser.o
BIN = fuser

CFLAGS += -DLIBFLASHDRV

all: check $(BIN)


check:
	@(if test -z "$(TARGET)"; then\
		echo "Error: Only main Makefile can be used for compilation!";\
		exit 1;\
	fi;)


.c.o:
	@(printf " CC    %-24s  \n" "$<"; $(CC) -c $(CFLAGS) "$<" -o "$@")

-include .depend

$(BIN): $(OBJS) $(LIBS)
	@echo " LD    $@"
	$(SIL)$(LD) $(LDFLAGS) -o $(BIN)_unstripped $^ $(LDLIBS)
	@echo " STRIP $@"
	$(SIL)$(STRIP) $(BIN)_unstripped -o $(BIN) 2>/dev/null

depend: check
	$(SIL)$(MKDEP) $(MKDEPFLAGS) $(SRCS) >.depend


clean: check
	$(SIL)rm -f core *.o $(ARCH)


.PHONY: clean
# DO NOT DELETE
