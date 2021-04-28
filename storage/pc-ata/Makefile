#
# Makefile for Phoenix-RTOS pc-ata driver
#
# Copyright 2018, 2019 Phoenix Systems
#
NAME := libata
LOCAL_SRCS := ata.c
LOCAL_HEADERS: ata.h

include $(static-lib.mk)

NAME := pc-ata
LOCAL_SRCS := atasrv.c mbr.c
DEP_LIBS := libata
LIBS := libext2

include $(binary.mk)
