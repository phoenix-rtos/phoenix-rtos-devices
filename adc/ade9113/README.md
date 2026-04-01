ADC driver continuously reading samples from 4 chained [ADE9113](https://www.analog.com/en/products/ade9113.html)

This code was written for specific HW configuration. Contents of `main.c` are included here to serve as an example /
base that can be adapted to work with other platforms.

SPI / clock chain
-----------------
Current driver (`main.c`) implementation expects 4 ADE9113 chips chained together in following way:
```
A0_XTALIN -> IMX6_PWM3_OUT
A0_MOSI -> IMX6_MOSI

A1_XTALIN <- A0_CLKOUT
A1_MOSI <- A0_MISO

A2_XTALIN <- A0_CLKOUT
A2_MOSI <- A1_MISO

A3_XTALIN <- A0_CLKOUT
A3_MOSI <- A2_MISO
A3_MISO -> IMX6_MISO
A3_DREADY -> IMX6_GPIO3_IO03 (any DREADY output can be used here with exception of A0_DREADY as it is used as clock
output)
```

Sampling frequency
------------------
Using PWM3 output for clocking ADC's introduces small frequency error. While using 16.5 Mhz clock fits in ADE9113
recommended range it results in sampling rate increase of `16.5 / 16.384 ~= 1.007`. Default sampling rate is 8 ksps
(~8.057 ksps with frequency correction). Different rates can be selected by changing datapath with `-s [0-7]` argument.

Output stream
-------------
Samples can be read from "/dev/ade9113" device. After open (internal) read offset is always aligned to next sample
start. Each sample contains up to 12 values encoded as little-endian two's complement 24 bit signed integer. Number
of values depends on what channels were enabled with `-i <ch_bitmask>` option passed to driver. By default all channels
are enabled resulting in sample size `3 * 12 == 36` bytes.

Each (blocking) read from opened device will write full amount of bytes requested. Reader should regularly read data to
avoid missing any samples. If data is not consumed fast enough read will return `-EPIPE`. To resume reading app needs to
close and open device again. Output from multiple successful reads after open is guaranteed to contain conitnous stream
of samples starting from time around when open was issued.

Channel values in sample use following order:
`A0_I, A0_V1, A0_V2, A1_I, A1_V1, A1_V2, A2_I, A2_V1, A2_V2, A3_I, A3_V1, A3_V2`

Dumping raw samples
-------------------
Standard unix command line tools can be used for saving sample stream:
```bash
# dump 500 kb of sample data to file
dd if=/dev/ade9113 of=/var/tmp/samples.pcm bs=1k count=500

# serve sample stream on TCP port 9113
nc -ll -p 9113 -e cat /dev/ade9113 > /dev/null
```

Importing samples to audacity
----------------------------
Audio tools are useful for analyzing analog samples. [Audacity](https://www.audacityteam.org) is a good open source tool
for that.

Select "Import -> Raw Data"

Import setup:

* Encoding: Signed 24-bit PCM
* Byte order: little-endian
* Channels: 12 (or less if `-i` flag is used)
* Start offset: 0
* Sample rate: 8057 Hz
