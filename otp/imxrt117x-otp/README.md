This tool blows OTP fuses of iMX RT117x MCU.

Usage from psh:

	sysexec [map] imxrt117x-otp [-r | -w value] -f fuse

Where:

	map - memory map for data (created by plo, dtcm or ocram2 for example)
	-r  - read fuse value
	-w  - write value to fuse
	-f  - select fuse from fusemap (0x800 to 0x18F0)

Example - burning of BT_FUSE_SEL fuse:

	sysexec dtcm imxrt117x-otp -f 0x960 - w 0x10

to verify that the fuse has been burned successfuly:

	sysexec dtcm imxrt117x-otp -f 0x960 -r

You should get the value 0x00000010.
