#
# Makefile for Phoenix-RTOS imx6ull sdma lib
#
# Copyright 2018 Phoenix Systems
#

SRCS = sdma-lib.c
OBJS = $(SRCS:.c=.o)
LIB = libsdma.a

all: check $(LIB)


check:
	@(if test -z "$(TARGET)"; then\
		echo "Error: Only main Makefile can be used for compilation!";\
		exit 1;\
	fi;)


.c.o:
	@(printf " CC    %-24s  \n" "$<"; $(CC) -c $(CFLAGS) $<)

-include .depend


$(LIB): $(OBJS)
	@echo " AR    $@"
	$(AR) $(ARFLAGS) -o $(LIB) $(OBJS)

depend: check
	$(SIL)$(MKDEP) $(MKDEPFLAGS) $(SRCS) >.depend


clean: check
	$(SIL)rm -f core *.o $(ARCH)


.PHONY: clean
# DO NOT DELETE
