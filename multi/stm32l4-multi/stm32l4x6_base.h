/*
 * Phoenix-RTOS
 *
 * Multidrv-lib: Peripheral base address definitions for STM32L4x6
 *
 * Copyright 2025 Phoenix Systems
 * Author: Jacek Maksymowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _STM32L4X6_BASE_H_
#define _STM32L4X6_BASE_H_

#define ADC_BASE    ((void *)0x50040000)
#define AES_BASE    ((void *)0x50060000)
#define DMA1_BASE   ((void *)0x40020000)
#define DMA2_BASE   ((void *)0x40020400)
#define EXTI_BASE   ((void *)0x40010400)
#define FLASH_BASE  ((void *)0x40022000)
#define GPIOA_BASE  ((void *)0x48000000)
#define GPIOB_BASE  ((void *)0x48000400)
#define GPIOC_BASE  ((void *)0x48000800)
#define GPIOD_BASE  ((void *)0x48000c00)
#define GPIOE_BASE  ((void *)0x48001000)
#define GPIOF_BASE  ((void *)0x48001400)
#define GPIOG_BASE  ((void *)0x48001800)
#define GPIOH_BASE  ((void *)0x48001c00)
#define GPIOI_BASE  ((void *)0x48002000)
#define HASH_BASE   ((void *)0x50060400)
#define I2C1_BASE   ((void *)0x40005400)
#define I2C2_BASE   ((void *)0x40005800)
#define I2C3_BASE   ((void *)0x40005c00)
#define I2C4_BASE   ((void *)0x40008400)
#define PWR_BASE    ((void *)0x40007000)
#define RCC_BASE    ((void *)0x40021000)
#define RNG_BASE    ((void *)0x50060800)
#define RTC_BASE    ((void *)0x40002800)
#define SPI1_BASE   ((void *)0x40013000)
#define SPI2_BASE   ((void *)0x40003800)
#define SPI3_BASE   ((void *)0x40003c00)
#define SYSCFG_BASE ((void *)0x40010000)
#define USART1_BASE ((void *)0x40013800)
#define USART2_BASE ((void *)0x40004400)
#define USART3_BASE ((void *)0x40004800)
#define UART4_BASE  ((void *)0x40004c00)
#define UART5_BASE  ((void *)0x40005000)

#endif /* _STM32L4X6_BASE_H_ */
