#
# Makefile for Phoenix-RTOS SPI API
#
# Copyright 2022 Phoenix Systems
#

# SPI API
NAME := spi-common
LOCAL_HEADERS := spi.h spi-msg.h
include $(static-lib.mk)

# SPI message API
NAME := libspi-msg
LOCAL_SRCS := libspi-msg.c
DEPS := spi-common
include $(static-lib.mk)
