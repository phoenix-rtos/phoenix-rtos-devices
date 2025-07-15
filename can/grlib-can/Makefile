#
# Makefile for Phoenix-RTOS grlib-can
#
# Copyright 2023 Phoenix Systems
#
# %LICENSE%
#
NAME := grlib-can-core
LOCAL_SRCS := grlib-can-core.c
LOCAL_HEADERS := grlib-can-core.h grlib-can-shared.h
include $(static-lib.mk)


NAME := grlib-can-driver
LOCAL_SRCS := grlib-can-driver.c
DEP_LIBS := grlib-can-core
LOCAL_CFLAGS = -Wno-error
include $(binary.mk)

NAME := grlib-can-if
LOCAL_SRCS := grlib-can-if.c
LOCAL_HEADERS := grlib-can-if.h grlib-can-shared.h
include $(static-lib.mk)

NAME := can-test-app
LOCAL_SRCS := can-test-app.c
DEP_LIBS := grlib-can-if
LOCAL_CFLAGS = -Wno-error
include $(binary.mk)
