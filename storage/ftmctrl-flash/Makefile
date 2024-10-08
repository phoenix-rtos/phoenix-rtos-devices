#
# Makefile for Phoenix-RTOS ftmctrl-flash driver
#
# Copyright 2023, 2024 Phoenix Systems
#

NAME := libflashdrv-ftmctrl
LOCAL_SRCS := amd-flash.c flashdrv.c flash.c intel-flash.c

include $(static-lib.mk)

NAME := ftmctrl-flash
LOCAL_SRCS := flashsrv.c
DEP_LIBS := libflashdrv-ftmctrl
LIBS := libjffs2 libstorage libmtd libptable libcache

include $(binary.mk)
