#
# Makefile for Phoenix-RTOS zynq-i2c driver
#
# Copyright 2022 Phoenix Systems
#

NAME := libzynq-i2c
LOCAL_SRCS := libzynq-i2c.c
DEPS := i2c-common
# no install headers as we're implementing generic interface only
include $(static-lib.mk)


NAME := zynq-i2c
LOCAL_SRCS := zynq-i2c.c
DEP_LIBS := libzynq-i2c
DEPS := i2c-common
include $(binary.mk)
