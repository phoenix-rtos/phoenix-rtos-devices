# imx6ull-gpio
This hardware driver server provides access to i.MX 6ULL GPIOs.

Interface
Server creates folder in /dev for each gpio port (e.g. <i>/dev/gpio1</i>, <i>/dev/gpio2</i> ...). Each folder contains two files: <i>port</i> and <i>dir</i>. Manipulation of GPIO port is performed via writes and read to/from this files using below binary structure:

    typedef union {
        unsigned int val;
        struct {
            unsigned int val;
            unsigned int mask;
        } __attribute__((packed)) w;
    } gpiodata_t;

## dir file
This file is used to select pin's direction, i.e. if it is input or output pin. For example, to set pin 9 of gpio2 as an output:

open file <i>/dev/gpio2/dir</i>:

    fid = open("/dev/gpio2/dir", O_WRONLY);

prepare data:

    gpiodata_t data;

    data.w.val = 1 << 9; /* Pin 9 as output */
    data.w.mask = 1 << 9; /* Allow status of pin 9 to change */

write:

    write(fid, &data, sizeof(data));

## port file
This file is used to output logical state to port or to read external stimuli, according to direction set for each pin.

### Write example (set pin 9 of gpio2 to high):

open file /dev/gpio2/port:

    fid = open("/dev/gpio2/port", O_WRONLY);

prepare data:

    gpiodata_t data

    data.w.val = 1 << 9; /* Logical high on pin 9 */
    data.w.mask = 1 << 9; /* Allow state of pin 9 to change */

write:

    write(fid, &data, sizeof(data));

### Read example (read state of gpio2):

open file <i>/dev/gpio2/port</i>:

    fid = open("/dev/gpio2/port", O_RDONLY);
    
read data:

    gpiodata_t data;

    read(fid, &data, sizeof(data));
    /* State of gpio port 2 is in data.val */

## Note

Input/output multiplexers and physical pad control is performed independently by kernel's platformctl interface and should be performed by user prior to usage of GPIO driver.


## Text write mode

The `dir` and `port` files support also text write mode for easier usage from shell scripts.

The syntax is `[+/-][pin]`, eg `+28`, `-7`, where:
 - `+` means _value high_ for `port` and _output_ for `dir`
 - `-` means _value low_ for `port` and _input_ for `dir`
 - `pin` is the numeric pin value in <0, 31> range

This interface allows changing only one pin value / direction at a time (state of other pins won't be changed).

Please note that the byte length of the text message can't be divisible by 4 to avoid confusing it with the binary interface.


### Example

```bash
# set direction to output
echo -n "+9" > /dev/gpio2/dir
# set output value to LOW
echo -n "-9" > /dev/gpio2/port
```
