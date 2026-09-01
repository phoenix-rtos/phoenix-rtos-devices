#
# Makefile for Phoenix-RTOS tda4vm-spi
#
# Copyright 2026 Phoenix Systems
#

# SPI library
NAME := libspi-tda4vm
LOCAL_SRCS := spi-tda4vm.c
LOCAL_HEADERS := spi-tda4vm.h
include $(static-lib.mk)

# SPI demo app
NAME := mcspidemo
LOCAL_SRCS := spidemo.c
DEP_LIBS := libspi-tda4vm
include $(binary.mk)