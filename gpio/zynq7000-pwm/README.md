# zynq7000-pwm
This hardware driver server provides access to Zynq7000 PWM8X IP core. This driver require the PWM8X IP core to be loaded to the PL.

## Usage
`zynq7000-pwm [OPTIONS]`
Options:
- `-b base_addr` - base address of PWM8X IP (required)
- `-r reload`    - reload value 0-4294967295, optional, default 100000
- `-p priority   - server thread priority
- `-h`           - help

## Interface
Server creates a file in /dev for each PWM channel (e.g. <i>/dev/pwm0</i>, <i>/dev/pwm1</i> ...). Manipulation of PWM duty cycle is performed via writes and reads to/from this files.
Example:
- setting 100/100000 duty cycle on channel #0: `echo 100 > /dev/pwm0`
- getting current duty cycle of channel #0: `cat /dev/pwm0`
