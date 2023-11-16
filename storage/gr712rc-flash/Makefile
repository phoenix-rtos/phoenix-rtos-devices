#
# Makefile for Phoenix-RTOS gr712rc-flash driver
#
# Copyright 2023 Phoenix Systems
#

NAME := libflashdrv-gr712rc
LOCAL_SRCS := flashdrv.c flash.c

include $(static-lib.mk)

NAME := gr712rc-flash
LOCAL_SRCS := flashsrv.c
DEP_LIBS := libflashdrv-gr712rc
LIBS := libjffs2 libstorage libmtd libptable

include $(binary.mk)
