#
# Makefile for Phoenix-RTOS imx6ull-flashnor driver
#
# Copyright 2021 Phoenix Systems
#


NAME := libflashnordrv-imx6ull
LOCAL_SRCS := flashnor-ecspi.c flashnor-qspi.c flashnor-drv.c nor.c
LOCAL_HEADERS := imx6ull-flashnor-ecspi.h imx6ull-flashnor-qspi.h imx6ull-flashnor-drv.h imx6ull-nor.h
DEPS := libimx6ull-ecspi libimx6ull-qspi

include $(static-lib.mk)

NAME := imx6ull-flashnor
LOCAL_SRCS := flashnor-srv.c
DEP_LIBS := libflashnordrv-imx6ull libimx6ull-ecspi libimx6ull-qspi
LIBS := libmeterfs-storage libmeterfs libstorage libjffs2 libmtd
LOCAL_INSTALL_PATH := /sbin

include $(binary.mk)

# NOR flash test program
NAME := flashnor-test
LOCAL_SRCS := flashnor-test.c
DEP_LIBS := libflashnordrv-imx6ull libimx6ull-ecspi libimx6ull-qspi

include $(binary.mk)
