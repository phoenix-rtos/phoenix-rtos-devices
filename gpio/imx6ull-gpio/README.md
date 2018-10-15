# imx6ull-gpio
This hardware driver server provides access to i.MX 6ULL GPIOs.

Interface
Server creates folder in /dev for each gpio port (e.g. <i>/dev/gpio1</i>, /dev/gpio2 ...). Each folder contains two files: port and dir. Manipulation of GPIO port is performed via writes and read to/from this files using below binary structure:

    typedef union {
        unsigned int val;
        struct {
            unsigned int val;
            unsigned int mask;
        } __attribute__((packed)) w;
    } gpiodata_t;

## dir file
This file is used to select pin's direction, i.e. if it is input or output pin. For example, to set pin 9 of gpio2 as an output:

open file /dev/gpio2/dir:

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

open file /dev/gpio2/port:

    fid = open("/dev/gpio2/port", O_RDONLY);
    
read data:

    gpiodata_t data;

    read(fid, &data, sizeof(data));
    /* State of gpio port 2 is in data.val */

## Note

Input/output multiplexers and physical pad control is performed independently by kernel's platformctl interface and should be performed by user prior to usage of GPIO driver.
