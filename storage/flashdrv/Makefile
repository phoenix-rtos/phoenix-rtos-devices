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
else ifneq ($(filter $(TARGET_SUBFAMILY), stm32n6),)
# Set stack size - on ARMv8-M targets the default stack size given by the kernel may be too small
# when filesystem support is added
	LOCAL_LDFLAGS := -z stack-size=2048 -z noexecstack
	LOCAL_CFLAGS += -DFLASHSRV_STORAGE_STACK_SIZE=2048
	DEP_LIBS := libstm32_extflash
	LOCAL_CFLAGS += -DFLASHSRV_ENABLE_LITTLEFS=1
	LIBS += liblfs
endif

include $(binary.mk)
