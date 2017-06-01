#
# Makefile for Phoenix-RTOS kernel (dev/serial/uart16550)
#
# Copyright 2015 Phoenix Systems
#

SRCS = uart16550.c

OBJS = $(SRCS:.c=.o)


all: check $(ARCH)


check:
	@(if test -z "$(ARCH)"; then\
		echo "Error: Only main Makefile can be used for compilation!";\
		exit 1;\
	fi;)


.c.o:
	@(echo "  CC $<"; $(CC) -c $(CFLAGS) $<)


-include .depend


$(ARCH): $(OBJS)
	$(SIL)$(AR) $(ARFLAGS) $(ARCH) $(OBJS) 2>/dev/null


depend: check
	$(SIL)$(MKDEP) $(MKDEPFLAGS) $(SRCS) >.depend


clean: check
	@rm -f core *.o $(ARCH)


.PHONY: clean
# DO NOT DELETE