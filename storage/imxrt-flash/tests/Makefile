#
# Phoenix-RTOS
#
#  i.MX RT storage tests
#
# Copyright 2020 Phoenix Systems
# Author: Hubert Buczynski
#

NAME := flash-tests
LOCAL_SRCS := tests.c flashsrv_mfs_tests.c flashsrv_raw_tests.c flashdrv_tests.c
DEPS := imxrt-flash
DEP_LIBS := libflashdrv-imxrt
LIBS := libptable libmeterfs

include $(binary.mk)

