#
# Makefile for Phoenix-RTOS zynq7000-i2c driver
#
# Copyright 2022 Phoenix Systems
#

NAME := libzynq7000-i2c
LOCAL_SRCS := libzynq7000-i2c.c
DEPS := i2c-common
# no install headers as we're implementing generic interface only
include $(static-lib.mk)


NAME := zynq7000-i2c
LOCAL_SRCS := zynq7000-i2c.c
DEP_LIBS := libzynq7000-i2c
DEPS := i2c-common
include $(binary.mk)
