#
# Makefile for Phoenix-RTOS grlib-can
#
# Copyright 2023 Phoenix Systems
#
# %LICENSE%
#

NAME := grlib-can
LOCAL_SRCS := grlib-can.c
LOCAL_HEADERS := grlib-can.h

LOCAL_CFLAGS = -Wno-error -fno-omit-frame-pointer
include $(binary.mk)


NAME := can-test-app
LOCAL_SRCS := can-test-app.c
LOCAL_CFLAGS = -Wno-error
include $(binary.mk)
