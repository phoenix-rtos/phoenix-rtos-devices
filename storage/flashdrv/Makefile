#
# Makefile for Phoenix-RTOS ftmctrl-flash driver
#
# Copyright 2023-2025 Phoenix Systems
#

NAME := flashdrv
LOCAL_SRCS := common.c flashsrv.c
LIBS := libstorage libmtd libptable libcache
ifneq ($(filter $(TARGET_SUBFAMILY), grfpga gr712rc gr740),)
	DEP_LIBS := libftmctrl libspimctrl
	LOCAL_CFLAGS += -DFLASHSRV_ENABLE_JFFS2=1
	LIBS += libjffs2
endif

include $(binary.mk)
