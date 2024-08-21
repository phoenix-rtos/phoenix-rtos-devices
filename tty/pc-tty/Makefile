#
# Makefile for Phoenix-RTOS pc-tty driver
#
# Copyright 2017, 2018, 2019 Phoenix Systems
# Copyright 2001, 2005-2006 Pawel Pisarczyk


NAME := pc-tty
SRCS := $(wildcard tty/pc-tty/*.c)
DEP_LIBS := libtty libklog

ifeq ($(HAS_MOUSE), n)
	SRCS := $(filter-out tty/pc-tty/ttypc_mouse.c, $(SRCS))
endif

include $(binary.mk)
