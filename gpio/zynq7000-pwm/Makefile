#
# Makefile for Phoenix-RTOS zynq7000-gpio driver
#
# Copyright 2022 Phoenix Systems
#

NAME := zynq7000-pwm
LOCAL_SRCS := pwm.c

include $(binary.mk)


# PWM message API
NAME := libpwmmsg
LOCAL_SRCS := pwm-msg.c
LOCAL_HEADERS := pwm-msg.h
include $(static-lib.mk)
