This catalog contains code for the MIPI driver for the IMX219 camera - code in early development.

i2cdetect.c - small program to see which devices respond over the I2C bus.

Main function is in axi-test.c, it currently configures the camera and sees, whether the MIPI-RX subcore sees the end of frame.

libaxi-i2c.c contains the functions for configuring the camera over I2C.

Other files contain mainly macros for different subcores.
