#
# Makefile for Phoenix-RTOS zynq-i2c driver
#
# Copyright 2025 Phoenix Systems
#

NAME := libaxi-i2c
LOCAL_SRCS := libaxi-i2c.c
DEPS := i2c-common
# no install headers as we're implementing generic interface only
include $(static-lib.mk)


NAME := axi-i2c
LOCAL_SRCS := axi-i2c.c axi-i2c.h
# DEP_LIBS := libaxi-i2c
DEPS := i2c-common
include $(static-lib.mk)

NAME := axi-i2c-test
LOCAL_SRCS := axi-test.c
DEP_LIBS := libaxi-i2c axi-i2c
DEPS := i2c-common
include $(binary.mk)

NAME := imx219
LOCAL_SRCS := imx219pq.c imx219pq.h
DEP_LIBS := libaxi-i2c axi-i2c
DEPS := i2c-common
include $(binary.mk)

NAME := i2cdetect
LOCAL_SRCS := i2cdetect.c
DEP_LIBS := libaxi-i2c axi-i2c
DEPS := i2c-common
include $(binary.mk)
