#
# Makefile for Phoenix-RTOS tda4vm MCU CAN
#
# Copyright 2026 Phoenix Systems
#

# CAN library
NAME := libcan-tda4vm
LOCAL_SRCS := can-tda4vm.c phy.c
LOCAL_HEADERS := can-tda4vm.h
include $(static-lib.mk)

# CAN demo app
NAME := mcandemo
LOCAL_SRCS := candemo.c
DEP_LIBS := libcan-tda4vm libgpio-tda4vm
include $(binary.mk)