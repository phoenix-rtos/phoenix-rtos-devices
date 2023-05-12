#
# Makefile for Phoenix-RTOS imx6ull-flashnor driver
#
# Copyright 2021 Phoenix Systems
#

NAME := imx6ull-flashnor
LOCAL_SRCS := flashnor.c flashnor-ecspi.c flashnor-qspi.c flashnor-drv.c nor/nor.c
LOCAL_HEADERS := flashnor-ecspi.h flashnor-qspi.h flashnor-drv.h nor/nor.h
DEP_LIBS := libimx6ull-ecspi libimx6ull-qspi
LIBS := libmeterfs-storage libmeterfs libstorage libjffs2 libmtd
LOCAL_INSTALL_PATH := /sbin

include $(binary.mk)

# NOR flash test program
NAME := flashnor-test
LOCAL_SRCS := flashnor-test.c
LIBS :=

include $(binary.mk)
