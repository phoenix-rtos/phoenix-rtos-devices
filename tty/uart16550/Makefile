#
# Makefile for Phoenix-RTOS pc-uart driver
#
# Copyright 2017, 2018 Phoenix Systems
# Copyright 2001, 2005-2006 Pawel Pisarczyk
#

NAME := uart16550
LOCAL_SRCS := uart16550.c

ifeq ($(TARGET_FAMILY),riscv64)
  LOCAL_SRCS += uarthw-riscv.c
else ifeq ($(TARGET_FAMILY),ia32)
  LOCAL_SRCS += uarthw-pc.c
else ifeq ($(TARGET_SUBFAMILY),zynq7000)
  LOCAL_SRCS += uarthw-zynq7000.c
endif

DEP_LIBS := libtty libklog
include $(binary.mk)
