#
# Makefile for Phoenix-RTOS gr716-flash driver
#
# Copyright 2023 Phoenix Systems
#

NAME := libflashdrv-gr716
LOCAL_SRCS := flashdrv.c spimctrl.c nor/nor.c

include $(static-lib.mk)

NAME := gr716-flash
LOCAL_SRCS := flashsrv.c
LOCAL_HEADERS := gr716-flashsrv.h
DEP_LIBS := libflashdrv-gr716
LIBS := libmeterfs libptable

include $(binary.mk)
