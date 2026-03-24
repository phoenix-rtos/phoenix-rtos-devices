#
# Makefile for Phoenix-RTOS grlib-nandfctrl2 driver
#
# Copyright 2026 Phoenix Systems
#

NAME := grlib-nandfctrl2
LOCAL_SRCS := flashsrv.c flashdev.c flashdrv.c onfi-4.c
LOCAL_HEADERS := nandfctrl2-flashsrv.h
LIBS := libjffs2 libstorage libmtd libptable

include $(binary.mk)

# Low-level NAND flash test program
NAME := test-grlib-nandfctrl2
LOCAL_SRCS := grlib-nandfctrl2-test.c flashdev.c flashdrv.c onfi-4.c
LOCAL_HEADERS := nandfctrl2-flashdrv.h
LIBS := libstorage

include $(binary.mk)
