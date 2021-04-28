#
# Makefile for Phoenix-RTOS imx6ull-nandtool
#
# Copyright 2018, 2019 Phoenix Systems
#

NAME := imx6ull-nandtool
LOCAL_SRCS := nandtool.c bch.c bcb.c
DEP_LIBS := libflashdrv-imx6ull
DEPS := imx6ull-flash

include $(binary.mk)

#ifeq ($(CONFIG_NANDTOOL_TEST), 1)
#CFLAGS += -DCONFIG_NANDTOOL_TEST
#PROGS_SRCS := test.c
#endif
