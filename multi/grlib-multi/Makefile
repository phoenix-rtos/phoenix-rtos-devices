#
# Makefile for Phoenix-RTOS grlib-multi
#
# Copyright 2023 Phoenix Systems
#
# %LICENSE%
#

NAME := grlib-multi
LOCAL_SRCS := adc.c gpio.c spacewire.c spi.c uart.c grlib-multi.c
LOCAL_HEADERS := grlib-multi.h
DEP_LIBS := libtty libklog libpseudodev libgrdmac2

# FIXME: adapt code to array-bounds checker 
LOCAL_CFLAGS := -Wno-array-bounds

include $(binary.mk)
