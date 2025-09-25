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
ifneq ($(findstring stm32,$(TARGET_SUBFAMILY)),)
  LOCAL_SRCS := libspi-msg-stm32.c
else
  LOCAL_SRCS := libspi-msg.c
endif

NAME := libspi-msg
DEPS := spi-common
include $(static-lib.mk)
