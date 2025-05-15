#
# Makefile for Phoenix-RTOS Zynq PCI Express driver
#
# Copyright 2025 Phoenix Systems
#

NAME := pcie
LOCAL_SRCS := pcie.c
include $(binary.mk)

NAME := si5345
LOCAL_SRCS := si5345.c
DEP_LIBS := i2c-common libi2c-msg
include $(binary.mk)
