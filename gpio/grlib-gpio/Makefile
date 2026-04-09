#
# Makefile for Phoenix-RTOS GRLIB GPIO
#
# Copyright 2026 Phoenix Systems
#
# SPDX-License-Identifier: BSD-3-Clause
#

NAME := libgrgpio
LOCAL_HEADERS := libgrgpio.h
LOCAL_SRCS := libgrgpio.c

include $(static-lib.mk)


NAME := libgrgpio-msg
LOCAL_HEADERS := grgpio-msg.h
LOCAL_SRCS := libgrgpio-msg.c

include $(static-lib.mk)


NAME := grgpio
LOCAL_SRCS := grgpiosrv.c
DEP_LIBS := libgrgpio libgrgpio-msg

include $(binary.mk)
