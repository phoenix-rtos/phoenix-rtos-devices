# imx6ull-uart


This server provides TTY functionality for i.MX 6ULL SoC's UARTS.

Usage:

    imx6ull-uart [mode] [device] [speed] [parity] [use_rts_cts] 
    
No args for default settings (cooked, uart1, B115200, 8N1).
    
- mode: 0 - raw, 1 - cooked
- device: 1 to 8
- speed: baud_rate
- parity: 0 - none, 1 - odd, 2 - even
- use_rts_cts: 0 - no hardware flow control, 1 - use hardware flow control

Server creates special file in the <i>/dev</i> directory - <i>/dev/uartx</i>, where x is number of an UART device.
