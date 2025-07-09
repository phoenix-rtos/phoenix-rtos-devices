#
# Makefile for Phoenix-RTOS
#
# Copyright 2025 Phoenix Systems
#

# PCI Express server
NAME := pcie
LOCAL_SRCS := pcie.c
LOCAL_HEADERS := pcie.h

ifeq ($(PCI_EXPRESS_INIT_TEBF0808_PHY),y)
  CFLAGS += -DPCI_EXPRESS_INIT_TEBF0808_PHY
  DEP_LIBS := tebf0808-pcie-refclk tebf0808-ps-gtr-phy i2c-common libi2c-msg
endif

ifeq ($(PCI_EXPRESS_XILINX_NWL),y)
  CFLAGS += -DPCI_EXPRESS_XILINX_NWL
  DEP_LIBS += pcie-xilinx-nwl
endif

ifeq ($(PCI_EXPRESS_XILINX_AXI),y)
  CFLAGS += -DPCI_EXPRESS_XILINX_AXI
  DEP_LIBS += pcie-xilinx-axi
endif

include $(binary.mk)
