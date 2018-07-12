#
# Makefile for Phoenix-RTOS imx6ull-uart driver
#
# Copyright 2018 Phoenix Systems
#

# build and install into staging static lib
LIB := flashdrv.a
LIB_SRCS := flashdrv.c
LIB_HDRS := flashdrv.h

# build test program and link against lib
PROGS := flashdrv_test
PROGS_LIBS := flashdrv.a

# include after all dependencies are set
include $(TOPDIR)/Makefile.rules
