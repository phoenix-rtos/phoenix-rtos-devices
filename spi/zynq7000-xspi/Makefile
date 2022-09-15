#
# Makefile for Phoenix-RTOS zynq7000-xspi
#
# Copyright 2022 Phoenix Systems
#

# zynq7000 AXI SPI library
NAME := libzynq7000-xspi
LOCAL_SRCS := libzynq7000-xspi.c
DEPS := spi-common
include $(static-lib.mk)

# zynq7000 AXI SPI server
NAME := zynq7000-xspi
LOCAL_SRCS := zynq7000-xspi.c
DEP_LIBS := libzynq7000-xspi
include $(binary.mk)
