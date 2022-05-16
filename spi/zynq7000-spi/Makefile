#
# Makefile for Phoenix-RTOS zynq7000-spi
#
# Copyright 2022 Phoenix Systems
#

# zynq7000 SPI library
NAME := libzynq7000-spi
LOCAL_SRCS := libzynq7000-spi.c
DEPS := spi-common
include $(static-lib.mk)

# zynq7000 SPI server
NAME := zynq7000-spi
LOCAL_SRCS := zynq7000-spi.c
DEP_LIBS := libzynq7000-spi
include $(binary.mk)
