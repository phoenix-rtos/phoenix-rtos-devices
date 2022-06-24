# zynq7000-xgpio
This hardware driver server provides access to Zynq7000 AXI GPIO IP core. This driver requires the AXI GPIO IP core to be loaded to the PL.

## Usage
`zynq7000-xgpio [OPTIONS]`
Options:
- `-b base_addr` - base address of AXI GPIO IP (required)
- `-p priority`  - server thread priority
- `-h`           - help

## Interface
Server creates a directory in /dev for each GPIO channel (i.e. `/dev/xgpio0`, `/dev/xgpio1`). Each directory contains 3 types of files:
- port - provides read/write access to the whole 32 bit-wide GPIO port
- dir  - allows GPIO port configuration (0: input, 1: output)
- pinx - access to the individual GPIO pins

Example:
- setting GPIO channel 1 pin 15 to logic one: `echo 1 > /dev/xgpio0/pin15`
- getting value of GPIO channel 2 (whole port): `cat /dev/xgpio1/port`
