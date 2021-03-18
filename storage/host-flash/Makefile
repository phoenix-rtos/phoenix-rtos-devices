#
# Makefile for flash emulator
#
# Copyright 2021 Phoenix Systems
#
# %LICENSE%
#

MODULE = host-flash
LIBNAME = $(MODULE).a
SRCS = host-flash.c host-flashsrv.c
OBJS = $(addprefix $(PREFIX_O)storage/$(MODULE)/, $(SRCS:.c=.o))

$(PREFIX_A)$(LIBNAME): $(OBJS)
	$(ARCH)

$(PREFIX_H)host-flashsrv.h: storage/host-flash/host-flashsrv.h
	$(HEADER)

all: $(PREFIX_A)$(LIBNAME) $(PREFIX_H)host-flashsrv.h
