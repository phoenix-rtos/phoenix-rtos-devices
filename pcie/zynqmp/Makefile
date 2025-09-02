#
# Makefile for Phoenix-RTOS Zynq PCI Express driver
#
# Copyright 2025 Phoenix Systems
#

LOCAL_PATH := $(call my-dir)

NAME := pcie-core
#LOCAL_SRCS := core/pci_utils.c core/pci_core.c core/pci_link.c
SRCS := $(wildcard $(LOCAL_PATH)core/*.c)
LOCAL_CFLAGS := -Wno-error
include $(static-lib.mk)


NAME := hailo-common
SRCS := $(wildcard $(LOCAL_PATH)Hailo/common/*.c)
SRCS += $(wildcard $(LOCAL_PATH)Hailo/phoenix/*.c)
LOCAL_SRCS := Hailo/src/pcie.c Hailo/vdma/vdma.c Hailo/src/irq.c Hailo/src/nnc.c
#SRCS += $(wildcard $(LOCAL_PATH)Hailo/src/*.c)
LOCAL_CFLAGS := -Wno-error
include $(static-lib.mk)


NAME := pcie
LOCAL_SRCS := pcie.c
LOCAL_CFLAGS := -Wno-error
DEP_LIBS := pcie-core hailo-common
include $(binary.mk)