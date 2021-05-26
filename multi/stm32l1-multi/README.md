# stm32l1-multi

This server acculumates all devices drivers of STM32L152 microcontrollers familly. It provides support for:

- ADC,
- EEPROM and program flash read/write,
- GPIO,
- I2C,
- LCD controller,
- RTC,
- SPI,
- UART.

## config.h

Configuration file config.h allows to enable/disable some of the elements of multi-driver.

    #define UART1 1 /* 1 enables UART, 0 disables */
    #define UART2 1
    #define UART3 0
    #define UART4 1
    #define UART5 0

    #define UART_CONSOLE 4 /* Selects which UART is used by stdout */

    #define SPI1 1 /* 1 enables SPI, 0 disables */
    #define SPI2 0
    #define SPI3 0

    #define LCD 1 /* 1 enables LCD controller driver, 0 disables */

## Interface

Server accepts 3 types of messages

    mtRead
This message type returns data from UART_CONSOLE UART. This read blocks until specified number of bytes is available.

    mtWrite
This message type writes data to UART_CONSOLE UART.

    mtDevCtl

## Input parameters
mtDevCtl message expects below structure to be serialized in message i.raw field:

    typedef struct {
        int type;
    
        union {
            int adc_channel;
            rtctimestamp_t rtc_timestamp;
            lcdmsg_t lcd_msg;
            i2cmsg_t i2c_msg;
            uartget_t uart_get;
            uartset_t uart_set;
            uartdef_t uart_def;
            gpiodef_t gpio_def;
            gpioget_t gpio_get;
            gpioset_t gpio_set;
            spirw_t spi_rw;
            spidef_t spi_def;
            unsigned int flash_addr;
        };
    } __attribute__((packed)) multi_i_t;   

where

### type

is one from:

    enum { adc_get = 0, rtc_get, rtc_set, lcd_get, lcd_set, i2c_get, i2c_set,
        gpio_def, gpio_get, gpio_set, uart_def, uart_get, uart_set, flash_get,
        flash_set, spi_get, spi_set, spi_def };

### adc_channel

Selects from which channel analog value should be fetched.

### rtc_timestamp

Structure of below format:

    typedef struct {
	    unsigned int year;
	    unsigned char month;
	    unsigned char day;
	    unsigned char wday;

	    unsigned char hours;
	    unsigned char minutes;
	    unsigned char seconds;
    } __attribute__((packed)) rtctimestamp_t;

It is used to set new time to real time clock.

### lcd_msg

Structure of below format:

    typedef struct {
	    char str[10];
	    char str_small[2];
	    unsigned int sym_mask;
	    int on;
    } __attribute__((packed)) lcdmsg_t;

where

- str - alphanumeric string to be displated   
- str_small - small 2 digit string to be displayed
- sym_mask - which symbols should be active
- on - if LCD should be active

### i2c_msg

Structure of below format:

    typedef struct {
	    char addr;
	    char reg;
    } __attribute__((packed)) i2cmsg_t;

where

- addr - address of I2C device on the bus
- reg - address of I2C device's register

Data to write is send in the msg.i.data field. Buffer for reading is passed in msg.o.data field.

### uart_get

Structure of below format:

    typedef struct {
	    int uart;
	    int mode;
	    unsigned int timeout;
    } __attribute__((packed)) uartget_t;

Is used for reading from UART other than UART_CONSOLE, also for reading with timeout.

- uart - from pool of enum { usart1 = 0, usart2, usart3, uart4, uart5 };
- mode - from pool of enum { uart_mnormal = 0, uart_mnblock };

where

- uart_mnormal - blocking read
- uart_mnblock - non blocking read

timeout - timeout in micro seconds. If no data is present during timeout period, read will end and return any data it managed to get. If user want to wait for data indefinitely, timeout should be set to 0.

### uart_set

Structure of below format:

	typedef struct {
		int uart;
	} __attribute__((packed)) uartset_t;

Is used for writing to UART other than UART_CONSOLE.

uart - from pool of enum { usart1 = 0, usart2, usart3, uart4, uart5 };

### uart_def

Structure of below format:

	typedef struct {
		int uart;
		unsigned int baud;
		char enable;
		char bits;
		char parity;
	} __attribute__((packed)) uartdef_t;

Is used to configure UART.

- uart - from pool of enum { usart1 = 0, usart2, usart3, uart4, uart5 };
- baud - desired baudrate
- enable - when set to 0 UART is turned off
- bits - number of bits per frame (7, 8 or 9)
- parity - form pool of enum { uart_parnone = 0, uart_pareven, uart_parodd };

where

- uart_parnone - no parity
- uart_pareven - even parity
- uart_parodd - odd parity

### gpio_def

Structure of below format:

	typedef struct {
		int port;
		char pin;
		char mode;
		char af;
		char otype;
		char ospeed;
		char pupd;
	} __attribute__((packed)) gpiodef_t;

Is used to configure paramters of GPIO pin.

- port - from pool of enum { gpioa = 0, gpiob, gpioc, gpiod, gpioe, gpiof, gpiog, gpioh };
- pin - from 0 to 15, selects pin for configuration
- mode, af, otype, ospeed, pupd - paramters of pin, as described in MCU documentation

### gpio_get

Structure of below format:

	typedef struct {
		int port;
	} __attribute__((packed)) gpioget_t;

Is used for selection from which port state should be read.

- port - from pool of enum { gpioa = 0, gpiob, gpioc, gpiod, gpioe, gpiof, gpiog, gpioh };

### gpio_set

Structure of below format:

	typedef struct {
		int port;
		unsigned int mask;
		unsigned int state;
	} __attribute__((packed)) gpioset_t;

Is used for setting output pin(s) state.

- port - from pool of enum { gpioa = 0, gpiob, gpioc, gpiod, gpioe, gpiof, gpiog, gpioh };
- mask - bit mask which defines which pins state should change
- state - new state of port, masked by mask field

### spi_rw

Structure of below format:

	typedef struct {
		int spi;
		unsigned int addr;
		unsigned int flags;
		char cmd;
	} spirw_t;

Is used for read/write transactions with SPI device.

- spi - from pool of enum { spi1 = 0, spi2, spi3 };
- cmd - command to SPI device, first byte always written during transaction
- addr - address to be written after cmd during transaction
- flags - bit mask of enum { spi_cmd, spi_dummy, spi_addrlsb } and/or (addrsz << SPI_ADDRSHIFT)


where

- addrsz - this many bytes of addr will be transmitted
- spi_cmd - if set, cmd will be transmitted
- spi_dummy - if set one dummy transaction will preceed read/write
- spi_addrlsb - if set, addr will be sent least significant byte first

### spi_def

Structure of below format:

	typedef struct {
		int spi;
		int enable;
		char mode;
		char bdiv;
	} spidef_t;

Is used to configure paramters of transmission of SPI device.

- spi - from pool of enum { spi1 = 0, spi2, spi3 };
- mode - 0 to 3, configures polarity and active edge of SPI clock
- bdiv - selects SPI speed
- enable - if set to 0 SPI is disabled

### Output parameters

mtDevCtl message returns below structure serialized in message o.raw field:

	typedef struct {
		int err;

		union {
			unsigned short adc_val;
			rtctimestamp_t rtc_timestamp;
			lcdmsg_t lcd_msg;
			unsigned int gpio_get;
		};
	} __attribute__((packed)) multi_o_t;

### err

Specifies error from errno.h pool.

### adc_val

Returns 12 bit converstion result, aligned to MSB.

### rtc_timestamp

Structure of below format:

	typedef struct {
		unsigned int year;
		unsigned char month;
		unsigned char day;
		unsigned char wday;

		unsigned char hours;
		unsigned char minutes;
		unsigned char seconds;
	} __attribute__((packed)) rtctimestamp_t;

Returns state of the RTC.

### lcd_msg

Structure of below format:

	typedef struct {
		char str[10];
		char str_small[2];
		unsigned int sym_mask;
		int on;
	} __attribute__((packed)) lcdmsg_t;

Returns state of the LCD.

### gpio_get

Returns state of GPIO read using gpio_get request.
