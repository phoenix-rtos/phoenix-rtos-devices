# Adding new device guide

Below are steps that need to be taken when adding new device. They describe how to add device of following parameters:
 - Name: `newimu123`
 - Family: `newimuxxx`
 - i2c device address: `0x34`
 - whoami register address: `0x0F`
 - whoami register value: `0x12`
 - data length: 6 bytes (3 times `uint16_t`)

## 1. Adding device to list
1. create new header file called `newimuxxx.h` and copy standard content of such file
    1. skip if new device belongs to one of supported families
    2. add macro `NEWIMUXXX_DATA_ALL_SIZE` to match the amount of bytes that need to be allocated to store all measurements from device. If, for example `newimu123` offers only accelerometer reading in 3 axis, each as `uint16_t` then this macro should be set to 3 * sizeof(uint16_t) which gives `6` in total.
    3. create enum of supported devices from new family
        ```c
        enum newimuxxx_family { newimu123 };
        ```
2. in _communications.h_ add `newimuxxx` to the `dev_families` enum
    ```c
    enum dev_families { ... , newimuxxx,  unknown };
    ```
3. in _communications.c_:
    1. include header of `newimuxxx.h`
	2. add new entry into `devicelist`. As we are dealing with imu type sensor such entry will look like:
    ```c
    { .type = type_imu, .family = newimuxxx, .model = newimu123, .name = "NEWIMU123", .devAddr = 0x34, .whoamiAddr = 0x0F, .whoamiVal = 0x12 },
    ```
4. create new source file called `newimuxxx.c` and add dummy calls of functions from header `newimuxxx.h`:
    ```c
    #include <stdio.h>

    #include "communication.h"
    #include "newimuxxx.h"

    int newimuxxx_init(const imu_dev_t *dev) {return 0;}
    int newimuxxx_getAllData(const imu_dev_t *dev, float *buffer, uint8_t buflen) {return 0;}
    ```
5. Connect the sensor to the board/devkit/microcontroller and launch the imu driver. If everything is set up correctly the following should appear on the screen:
    ``` bash
    (psh)% sysexec imx6ull-imu
    Cannot initialize: LSM9DS1_IMU1
    (psh)% 
    ```
    which **is a good sign** meaning that our sensor has been found and recognized by the driver. 

## 2. Device initialization

This step should bring the sensor up to the running condition. All necessary registers should set up here such as: scales, frequencies of sampling, resets etc. This requires writing **family specific** implementation of function: 
```c
int newimuxxx_init(const imu_dev_t *dev)
```
It should distinguish between sensors from its family based on information stored in structure pointed by `dev` pointer. After implementation is ready this function must be added into `communications.c` into switch case in:
```c
int initDevice(const imu_dev_t *dev)
```

## 3. Data acquisition

This step should acquire all data that is offered by sensor. This requires writing **family specific** implementation of function:
```c
int newimuxxx_getAllData(const imu_dev_t *dev, float *buffer, uint8_t buflen)
```
It should distinguish between sensors from its family based on information stored in structure pointed by `dev` pointer. After implementation is ready this function must be added into `communications.c` into switch case in:
```c
int getAllData(const imu_dev_t *dev, float *buffer, uint8_t buflen)
```

# Hints

 - Following functions may be used to communicate with sensors:
    ```c
    /* Writes one byte into register 'reg_addr' of 'dev_addr' i2c device */
    int i2c_regWrite(uint8_t dev_addr, uint8_t reg_addr, uint8_t val)
    
    /* Performs i2c regiester read operation from the given slave device */
    int i2c_regRead(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data_out, uint32_t len)
    ```
 - If sensor can be recognized under couple of addresses (e.g different physical connections specifies the address) then copies of such device should be made in `devicelist`.
