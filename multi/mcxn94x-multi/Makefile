#
# Makefile for Phoenix-RTOS mcxn94x-multi
#
# Copyright 2024 Phoenix Systems
#

NAME := mcxn94x-multi

LOCAL_PATH := $(call my-dir)

LOCAL_SRCS = mcxn94x-multi.c uart.c
DEP_LIBS := libtty libklog libpseudodev
LIBS := libdummyfs libklog libpseudodev libposixsrv
LOCAL_HEADERS := mcxn94x-multi.h

include $(binary.mk)
