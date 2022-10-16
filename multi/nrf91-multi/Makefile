#
# Makefile for Phoenix-RTOS stm32-multi
#
# Copyright 2018, 2020 Phoenix Systems
#

LOCAL_PATH := $(call my-dir)

NAME := nrf91-multi
SRCS := $(wildcard $(LOCAL_PATH)*.c)
LOCAL_HEADERS := nrf91-multi.h
DEP_LIBS := libtty
LIBS := libdummyfs
# CFLAGS += -O0 -ggdb3 #DEBUG CFLAGS

include $(binary.mk)
