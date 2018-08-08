#
# Makefile for Phoenix-RTOS imx6ull-uart driver
#
# Copyright 2018 Phoenix Systems
#

# build and install into staging static lib
LIB := libflashdrv.a
LIB_SRCS := flashdrv.c
INSTALL_HDRS := flashdrv.h

# build test program and link against lib
PROGS := flashdrv_test
PROGS_LIBS := libflashdrv.a

# include after all dependencies are set
include $(TOPDIR)/Makefile.rules
