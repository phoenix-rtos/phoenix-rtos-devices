#
# Makefile for Phoenix-RTOS zynq-uart driver
#
# Copyright 2021 Phoenix Systems
#

NAME := zynq-uart
ifneq (, $(findstring virt, $(TARGET_PROJECT)))
  LOCAL_SRCS := zynq-uart-virt.c
else
  LOCAL_SRCS := zynq-uart.c
endif
DEP_LIBS := libtty libklog
include $(binary.mk)
