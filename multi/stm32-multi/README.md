# stm32-multi

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

    str - alphanumeric string to be displated   
    str_small - small 2 digit string to be displayed
    sym_mask - which symbols should be active
    on - if LCD should be active

### i2c_msg

Structure of below format:

    typedef struct {
	    char addr;
	    char reg;
    } __attribute__((packed)) i2cmsg_t;

where

    addr - address of I2C device on the bus
    reg - address of I2C device's register

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
