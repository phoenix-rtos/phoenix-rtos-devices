#
# Makefile for Phoenix-RTOS imx6ull-flashnor driver
#
# Copyright 2021 Phoenix Systems
#

NAME := imx6ull-flashnor
LOCAL_SRCS := flashnor.c flashnor-ecspi.c storage.c
LOCAL_HEADERS := flashnor-ecspi.h storage.h
DEP_LIBS := libimx6ull-ecspi
LIBS := libmeterfs
LOCAL_INSTALL_PATH := /sbin

include $(binary.mk)

# NOR flash test program
NAME := flashnor-test
LOCAL_SRCS := flashnor-test.c flashnor-ecspi.c storage.c
DEP_LIBS := libimx6ull-ecspi
LIBS := libmeterfs

include $(binary.mk)
