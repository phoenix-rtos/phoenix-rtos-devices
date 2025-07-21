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

include $(binary.mk)

NAME := grlib-can-if
LOCAL_SRCS := grlib-can-if.c
LOCAL_HEADERS := grlib-can-if.h

include $(static-lib.mk)

NAME := can-test-app
LOCAL_SRCS := can-test-app.c
DEP_LIBS := grlib-can-if

include $(binary.mk)

NAME := can-sender-stress
LOCAL_SRCS := grlib-can-sender-stress-test.c
DEP_LIBS := grlib-can-core

include $(binary.mk)

NAME := can-receiver-stress
LOCAL_SRCS := grlib-can-receiver-stress-test.c
DEP_LIBS := grlib-can-core

include $(binary.mk)