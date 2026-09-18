#
# Makefile for Phoenix-RTOS stm32-multi
#
# Copyright 2022 Phoenix Systems
#

LOCAL_PATH := $(call my-dir)

NAME := libstm32l4-multi

LOCAL_SRCS = libi2c.c libuart.c
ifeq ($(TARGET_SUBFAMILY),stm32l4x6)
  LOCAL_SRCS += libhash.c libaes.c libdma.c libspi_l4.c
else ifeq ($(TARGET_SUBFAMILY),stm32n6)
  LOCAL_SRCS += libhash.c libgpdma.c libspi_n6.c
else ifeq ($(TARGET_SUBFAMILY),stm32u3)
  LOCAL_SRCS += libgpdma.c libspi_n6.c
else 
endif

DEPS := libtty

include $(static-lib.mk)
