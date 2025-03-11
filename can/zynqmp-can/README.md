# zynqmp-can
The `zynqmp-can` is a hardware driver that provides access to the ZynqMP CAN (Processor Subsystem) peripheral.

## Usage
`zynqmp-can [options]`

### Options:
- `-n id`       - Specifies the peripheral number (ZynqMP has two peripherals: CAN0 and CAN1).
- `-b baudrate` - Sets the baud rate in [kbps].
- `-h`          - Displays the help menu.

### Example:
The command `zynqmp-can -n 1 -b 125` initializes CAN1 with a baud rate of 125000 baud/s.

## Interface
The server creates device files `/dev/can0` or `/dev/can1`. However, due to the relatively complex nature of CAN frames, these files cannot be accessed using standard open/close/read/write calls. Instead, a static library called `libzynqmp-can-if` is provided, which includes dedicated functions for interface users.

## Manual Test Application
A test application named `zynqmp-can-test` is available to test the API functionality of this driver.
