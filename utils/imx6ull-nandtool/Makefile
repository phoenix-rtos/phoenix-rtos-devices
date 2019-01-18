#
# Makefile for Phoenix-RTOS imx6ull-nandtool
#
# Copyright 2018, 2019 Phoenix Systems
#

$(PREFIX_PROG)imx6ull-nandtool: $(addprefix $(PREFIX_O)utils/imx6ull-nandtool/, nandtool.o bch.o bcb.o) $(PREFIX_A)libflashdrv.a
	$(LINK)
	
all: $(PREFIX_PROG_STRIPPED)imx6ull-nandtool


#ifeq ($(CONFIG_NANDTOOL_TEST), 1)
#CFLAGS += -DCONFIG_NANDTOOL_TEST
#PROGS_SRCS := test.c
#endif
