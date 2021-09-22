#
# Makefile for Phoenix-RTOS imx6ull-i2c driver
#
# Copyright 2021 Phoenix Systems
#

NAME := libimx6ull-i2c
LOCAL_SRCS := libimx6ull-i2c.c
DEPS := i2c-common
# no install headers as we're implementing generic interface only
include $(static-lib.mk)


NAME := imx6ull-i2c
LOCAL_SRCS := imx6ull-i2c.c
DEP_LIBS := libimx6ull-i2c
DEPS := i2c-common
include $(binary.mk)
