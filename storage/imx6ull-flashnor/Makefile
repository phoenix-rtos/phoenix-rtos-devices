#
# Makefile for Phoenix-RTOS imx6ull-flashnor driver
#
# Copyright 2021 Phoenix Systems
#

NAME := imx6ull-flashnor
LOCAL_SRCS := flashnor.c flashnor-ecspi.c flashnor-drv.c
LOCAL_HEADERS := flashnor-ecspi.h flashnor-drv.h
DEP_LIBS := libimx6ull-ecspi
LIBS := libmeterfs-storage libmeterfs libstorage
LOCAL_INSTALL_PATH := /sbin

include $(binary.mk)

# NOR flash test program
NAME := flashnor-test
LOCAL_SRCS := flashnor-test.c flashnor-ecspi.c
DEP_LIBS := libimx6ull-ecspi

include $(binary.mk)
