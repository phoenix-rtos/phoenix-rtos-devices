#
# Makefile for Phoenix-RTOS GRLIB SPI CTRL
#
# Copyright 2026 Phoenix Systems
#

NAME := libgrspictrl
LOCAL_SRCS := libgrspictrl.c
DEPS := libspi-mst
include $(static-lib.mk)

NAME := grspictrl
LOCAL_SRCS := grspictrl.c
DEP_LIBS := libgrspictrl libspi-mst libspi-mst-msg
include $(binary.mk)
