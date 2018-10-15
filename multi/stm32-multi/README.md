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

is one from

    enum { adc_get = 0, rtc_get, rtc_set, lcd_get, lcd_set, i2c_get, i2c_set,
        gpio_def, gpio_get, gpio_set, uart_def, uart_get, uart_set, flash_get,
        flash_set, spi_get, spi_set, spi_def };
