# PCI Express Driver

This driver is split into several modules:
- `/sbin/pcie` - platform-agnostic server
- Zynq UltraScale+ platform drivers:
  - `pcie-xilinx-nwl` - controls the PCI Express peripheral in the Processing Subsystem (PS)
  - `pcie-xilinx-axi` - controls the PCI Express peripheral instantiated in Programmable Logic (PL)
- board-specific support:
  - `tebf0808` – provides initialisation code for the TEBF0808 baseboard, setting up the reference clock and PHY.

## Configuration

Configuration is driven by:
1. Environment variables stored in the `build.project` file
2. Pre-processor macros defined in `board_config.h` file
