#
# Makefile for Phoenix-RTOS Zynq7000-uart driver
#
# Copyright 2025 Phoenix Systems
#

# ZynqMP CAN driver interface library
NAME := libzynqmp-can-if
LOCAL_SRCS := zynqmp-can-if.c
LOCAL_HEADERS := zynqmp-can-if.h
include $(static-lib.mk)

# ZynqMP CAN driver server
NAME := zynqmp-can
LOCAL_SRCS := zynqmp-can.c
DEP_LIBS := libzynqmp-can-if
include $(binary.mk)

# ZynqMP CAN manual test application
NAME := zynqmp-can-test
LOCAL_SRCS := zynqmp-can-test.c
DEP_LIBS := libzynqmp-can-if
include $(binary.mk)
