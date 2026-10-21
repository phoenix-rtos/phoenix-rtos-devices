#
# Makefile for Phoenix-RTOS stm32-multi
#
# Copyright 2022 Phoenix Systems
#

LOCAL_PATH := $(call my-dir)

NAME := libstm32l4-multi
DEPS := libtty

LOCAL_SRCS = libi2c.c libuart.c
ifeq ($(TARGET_SUBFAMILY),stm32l4x6)
  LOCAL_SRCS += libhash.c libaes.c libdma.c libspi_l4.c
else ifeq ($(TARGET_SUBFAMILY),stm32n6)
  LOCAL_SRCS += libhash.c libspi_n6.c
  DEPS += libstm32-xpdma
else ifeq ($(TARGET_SUBFAMILY),stm32u3)
  LOCAL_SRCS += libspi_n6.c
  DEPS += libstm32-xpdma
endif

include $(static-lib.mk)
