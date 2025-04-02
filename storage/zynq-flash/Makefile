#
# Phoenix-RTOS
#
# nor flash driver
#
# Copyright 2021 Phoenix Systems
# Author: Hubert Buczynski, Malgorzata Wrobel
#

NAME := libflashdrv-zynq
LOCAL_SRCS := qspi.c flashcfg.c flashdrv.c
LOCAL_HEADERS := flashdrv.h flashcfg.h

include $(static-lib.mk)

NAME := zynq-flash
LOCAL_SRCS := zynq-flash.c
DEP_LIBS := libflashdrv-zynq
LIBS := libstorage libjffs2 libmtd libcache

include $(binary.mk)
