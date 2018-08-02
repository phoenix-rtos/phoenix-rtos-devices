#
# Makefile for Phoenix-RTOS imx6ull-nandtool
#
# Copyright 2018 Phoenix Systems
#

PROGS_SRCS := nandtool.c bch.c bcb.c
PROGS = nandtool
PROGS_LIBS := flashdrv.a

ifeq ($(CONFIG_NANDTOOL_TEST), 1)
CFLAGS += -DCONFIG_NANDTOOL_TEST
PROGS_SRCS := test.c
endif

# include after all dependencies are set
include $(TOPDIR)/Makefile.rules
