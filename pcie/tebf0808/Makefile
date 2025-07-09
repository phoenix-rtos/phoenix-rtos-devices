#
# Makefile for Phoenix-RTOS
#
# Copyright 2025 Phoenix Systems
#

# TEBF0808 evaluation board specific initialisation code for PCI Express interface
ifeq ($(PCI_EXPRESS_INIT_TEBF0808_PHY),y)
  NAME := tebf0808-pcie-refclk
  LOCAL_SRCS := tebf0808-pcie-refclk.c
  LOCAL_HEADERS := tebf0808-pcie-refclk.h
  include $(static-lib.mk)

  NAME := tebf0808-ps-gtr-phy
  LOCAL_SRCS := tebf0808-ps-gtr-phy.c
  LOCAL_HEADERS := tebf0808-ps-gtr-phy.h
  include $(static-lib.mk)
endif
