#
# Makefile for Phoenix-RTOS zynq-spi
#
# Copyright 2025 Phoenix Systems
#

# Zynq7000 / ZynqMP SPI library
NAME := libzynq-spi
LOCAL_SRCS := libzynq-spi.c
DEPS := spi-common
include $(static-lib.mk)

# Zynq7000 / ZynqMP SPI server
NAME := zynq-spi
LOCAL_SRCS := zynq-spi.c
DEP_LIBS := libzynq-spi
include $(binary.mk)
