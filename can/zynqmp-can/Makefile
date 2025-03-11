#
# Makefile for Phoenix-RTOS Zynq7000-uart driver
#
# Copyright 2025 Phoenix Systems
#

# ZYNQ MP CAN driver interface library
NAME := libzynqmp-can-if
LOCAL_SRCS := zynqmp-can-if.c
LOCAL_HEADERS := zynqmp-can-if.h
include $(static-lib.mk)

# ZYNQ MP CAN driver server
NAME := zynqmp-can
LOCAL_SRCS := zynqmp-can.c
DEP_LIBS := libklog libzynqmp-can-if
include $(binary.mk)
