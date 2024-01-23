#
# Phoenix-RTOS
#
# SD card driver
#
# Copyright 2023 Phoenix Systems
# Author: Jacek Maksymowicz
#

NAME := libsdcard-zynq
LOCAL_SRCS := sdcard.c zynq7000-sdio.c
LOCAL_HEADERS := sdcard.h

include $(static-lib.mk)

NAME := zynq7000-sdcard
LOCAL_SRCS := sdstorage_dev.c sdstorage_srv.c
DEP_LIBS := libsdcard-zynq
LIBS := libstorage libcache libmbr libmtd libjffs2 libext2

include $(binary.mk)
