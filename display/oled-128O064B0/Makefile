#
# Makefile for BeSmart oled driver
#
# Copyright 2020 Phoenix Systems
# Author: Hubert Buczynski, Marcin Baran
#

OLED_CORE := oled-phy.c
OLED_GRAPHIC := oled-graphic.c fonts/font.c fonts/5x7.c

NAME := oled-128O064B0
LOCAL_SRCS := oled-dev.c $(OLED_GRAPHIC) $(OLED_CORE)
LOCAL_HEADERS := oled-api.h
DEPS := imxrt-multi
include $(binary.mk)

NAME := oled-test
LOCAL_SRCS := oled-test.c $(OLED_CORE)
include $(binary.mk)
