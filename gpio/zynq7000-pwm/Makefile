#
# Makefile for Phoenix-RTOS zynq7000-pwm driver
#
# Copyright 2022 Phoenix Systems
#

NAME := zynq7000-pwm
LOCAL_SRCS := pwm.c

include $(binary.mk)

NAME := libzynq7000pwm
LOCAL_SRCS := zynq7000-pwm-msg.c
LOCAL_HEADERS := zynq7000-pwm-msg.h

include $(static-lib.mk)
