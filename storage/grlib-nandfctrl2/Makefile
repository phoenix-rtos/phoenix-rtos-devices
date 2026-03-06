#
# Makefile for Phoenix-RTOS grlib-nandfctrl2-flash driver
#
# Copyright 2026 Phoenix Systems
#

NAME := libnandfctrl2-flashdrv
LOCAL_SRCS := nandfctrl2-flashdrv.c nandfctrl2-onfi.c
LOCAL_HEADERS := nandfctrl2-flashdrv.h nandfctrl2-onfi.h

include $(static-lib.mk)

NAME := grlib-nandfctrl2-flash
LOCAL_SRCS := nandfctrl2-flashsrv.c nandfctrl2-flashdev.c
LOCAL_HEADERS := nandfctrl2-flashsrv.h nandfctrl2-flashdev.h
DEP_LIBS := libnandfctrl2-flashdrv
LIBS := libjffs2 libstorage libmtd libptable

include $(binary.mk)
