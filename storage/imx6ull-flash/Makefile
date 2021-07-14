#
# Makefile for Phoenix-RTOS imx6ull-flash driver
#
# Copyright 2018, 2019 Phoenix Systems
#

NAME := libflashdrv-imx6ull
LOCAL_SRCS := flashdrv.c
LOCAL_HEADERS := imx6ull-flashdrv.h

include $(static-lib.mk)

NAME := imx6ull-flash
LOCAL_SRCS := flashsrv.c
LOCAL_HEADERS := imx6ull-flashsrv.h
DEP_LIBS := libflashdrv-imx6ull
LIBS := libjffs2

include $(binary.mk)
