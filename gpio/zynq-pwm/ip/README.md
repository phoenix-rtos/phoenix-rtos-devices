# PWM8X IP core
This IP core provides 8 PWM channels. AXI Lite interface is used.

## Functional description
32-bit forward counter is incremented on each AXI clock cycle. When counter reaches `reload` value (register #9) it value is reset to zero. Each PWM channel has `compare value` register. PWM output is high, when `compare value` is greater then current `counter` value, otherwise output is set low. Output is synchronized to AXI clock, so it's glitch free. Each `compare value` register is buffered - it's new value is taken into account only on `counter` reset.

## Registers description
All registers are 32-bit wide, only 32-bit access is supported
- base + 0x00: PWM channel 0 compare value
- base + 0x04: PWM channel 1 compare value
- base + 0x08: PWM channel 2 compare value
- base + 0x0A: PWM channel 3 compare value
- base + 0x0C: PWM channel 4 compare value
- base + 0x10: PWM channel 5 compare value
- base + 0x14: PWM channel 6 compare value
- base + 0x18: PWM channel 7 compare value
- base + 0x1C: PWM channel 8 compare value
- base + 0x20: PWM counter reload value
