#
# Makefile for Phoenix-RTOS GRLIB I2C MST driver
#
# Copyright 2026 Phoenix Systems
#

NAME := libgri2cmst
LOCAL_SRCS := libgri2cmst.c
DEPS := i2c-common
# no install headers as we're implementing generic interface only
include $(static-lib.mk)


NAME := gri2cmst
LOCAL_SRCS := gri2cmst.c
DEP_LIBS := libgri2cmst
DEPS := i2c-common
include $(binary.mk)
