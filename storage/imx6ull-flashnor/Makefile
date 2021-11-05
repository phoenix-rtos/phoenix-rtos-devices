#
# Makefile for Phoenix-RTOS imx6ull-flashnor driver
#
# Copyright 2021 Phoenix Systems
#

NAME := imx6ull-flashnor
LOCAL_PATH = $(call my-dir)
SRCS := $(wildcard $(LOCAL_PATH)*.c)
DEP_LIBS := libimx6ull-ecspi
LIBS := libmeterfs
LOCAL_INSTALL_PATH := /sbin

include $(binary.mk)
