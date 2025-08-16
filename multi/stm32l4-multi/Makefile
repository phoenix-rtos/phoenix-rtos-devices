#
# Makefile for Phoenix-RTOS stm32-multi
#
# Copyright 2018, 2020 Phoenix Systems
#

LOCAL_PATH := $(call my-dir)

NAME := stm32l4-multi
SRCS := $(wildcard $(LOCAL_PATH)*.c)
LOCAL_HEADERS := stm32l4-multi.h
DEP_LIBS := libstm32l4-multi libtty libklog
LIBS := libdummyfs libklog libposixsrv libmeterfs

# FIXME: adapt code to array-bounds checker 
LOCAL_CFLAGS := -Wno-array-bounds

include $(binary.mk)
