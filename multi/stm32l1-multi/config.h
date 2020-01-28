/*
 * Phoenix-RTOS
 *
 * STM32L1 multi driver config.
 *
 * Copyright 2018 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * %LICENSE%
 */


#ifndef _CONFIG_H_
#define _CONFIG_H_

#ifndef UART1
#define UART1 1
#endif

#ifndef UART2
#define UART2 1
#endif

#ifndef UART3
#define UART3 0
#endif

#ifndef UART4
#define UART4 1
#endif

#ifndef UART5
#define UART5 0
#endif

#ifndef UART_CONSOLE
#define UART_CONSOLE 4
#endif

#ifndef SPI1
#define SPI1 1
#endif

#ifndef SPI2
#define SPI2 0
#endif

#ifndef SPI3
#define SPI3 0
#endif

#ifndef LCD
#define LCD 1
#endif

#ifndef FLASH_PROGRAM_1_ADDR
#define FLASH_PROGRAM_1_ADDR 0x08000000
#endif

#ifndef FLASH_PROGRAM_2_ADDR
#define FLASH_PROGRAM_2_ADDR 0x08040000
#endif

#ifndef FLASH_EEPROM_1_ADDR
#define FLASH_EEPROM_1_ADDR 0x08080000
#endif

#ifndef FLASH_EEPROM_2_ADDR
#define FLASH_EEPROM_2_ADDR 0x08082000
#endif

#endif
