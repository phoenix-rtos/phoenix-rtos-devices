#
# Makefile for Phoenix-RTOS imx6ull-nandtool
#
# Copyright 2018 Phoenix Systems
#

SRCS = nandtool.c
OBJS = $(SRCS:.c=.o)
LIBS := flashdrv.o
BIN = nandtool

CFLAGS += -DLIBFLASHDRV

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
	@(printf " CC    %-24s  \n" "$<"; $(CC) -c $(CFLAGS) "$<" -o "$@")

-include .depend


#FIXME: move this code to static library, do not compile twice
flashdrv.o: ../imx6ull-flash/flashdrv.c
	@(printf " CC    %-24s  \n" "$<"; $(CC) -c $(CFLAGS) "$<" -o "$@")

$(BIN): $(OBJS) $(LIBS)
	@echo " LD    $@"
	$(SIL)$(LD) $(LDFLAGS) -o $(BIN) $^ $(LDLIBS)
	@echo " STRIP $@"
	$(SIL)$(STRIP) $(BIN) 2>/dev/null

depend: check
	$(SIL)$(MKDEP) $(MKDEPFLAGS) $(SRCS) >.depend


clean: check
	$(SIL)rm -f core *.o $(ARCH)


.PHONY: clean
# DO NOT DELETE
