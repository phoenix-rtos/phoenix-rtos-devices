#
# Makefile for Phoenix-RTOS Zynq PCI Express driver
#
# Copyright 2025 Phoenix Systems
#

NAME := pcie
LOCAL_SRCS := pcie.c
include $(binary.mk)

NAME := fsbl
LOCAL_SRCS := fsbl.c
include $(binary.mk)

NAME := fsbl2
LOCAL_SRCS := fsbl2.c fsbl2_integration.c
LOCAL_HEADERS := fsbl2.h fsbl2_integration.h
DEP_LIBS := i2c-common libi2c-msg
include $(binary.mk)

NAME := status
LOCAL_SRCS := status.c
include $(binary.mk)

NAME := si5345
LOCAL_SRCS := si5345.c
DEP_LIBS := i2c-common libi2c-msg
include $(binary.mk)
