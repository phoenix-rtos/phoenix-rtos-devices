#
# Makefile for Phoenix-RTOS imx6ull-uart driver
#
# Copyright 2018 Phoenix Systems
#

DRV_SRCS = sdma-driver.c
DRV_OBJS = $(DRV_SRCS:.c=.o)
DRV = sdma-driver

LIB_SRCS = sdma-lib.c
LIB_OBJS = $(LIB_SRCS:.c=.o)
LIB = libsdma.a

all: check $(DRV) $(LIB)


check:
	@(if test -z "$(TARGET)"; then\
		echo "Error: Only main Makefile can be used for compilation!";\
		exit 1;\
	fi;)


.c.o:
	@(printf " CC    %-24s  \n" "$<"; $(CC) -c $(CFLAGS) $<)

-include .depend


$(LIB): $(LIB_OBJS)
	@echo " AR    $@"
	$(AR) $(ARFLAGS) -o $(LIB) $(LIB_OBJS)


$(DRV): $(DRV_OBJS)
	@echo " LD    $@"
	$(SIL)$(LD) $(LDFLAGS) -o $(DRV) $(DRV_OBJS) $(LDLIBS)
	@echo " STRIP $@"
	$(SIL)$(STRIP) $(DRV) 2>/dev/null


depend: check
	$(SIL)$(MKDEP) $(MKDEPFLAGS) $(DRV_SRCS) $(LIB_SRCS) >.depend


clean: check
	$(SIL)rm -f core *.o $(ARCH)


.PHONY: clean
# DO NOT DELETE
