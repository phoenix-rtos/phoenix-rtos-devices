#
# Makefile for Phoenix-RTOS Zynq PCI Express driver
#
# Copyright 2025 Phoenix Systems
#

LOCAL_PATH := $(call my-dir)

NAME := pcie-core
# LOCAL_SRCS := $(wildcard $(LOCAL_PATH)pci_core/*.c)
LOCAL_SRCS := pci_core/pci_core.c pci_core/pci_link.c pci_core/pci_utils.c
# $(error $(LOCAL_SRCS))
# LOCAL_SRCS := $(wildcard $(LOCAL_PATH)core/*.c)
LOCAL_HEADERS := pci_core/pci.h pci_core/pci_core.h pci_core/pci_link.h pci_core/pci_utils.h
LOCAL_CFLAGS := -Wno-error
include $(static-lib.mk)

NAME := pcie
LOCAL_SRCS := pcie.c
LOCAL_CFLAGS := -Wno-error
DEP_LIBS := pcie-core
include $(binary.mk)