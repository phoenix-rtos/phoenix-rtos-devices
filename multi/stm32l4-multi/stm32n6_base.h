/*
 * Phoenix-RTOS
 *
 * Multidrv-lib: Peripheral base address definitions for STM32N6
 *
 * Copyright 2025 Phoenix Systems
 * Author: Jacek Maksymowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _STM32N6_BASE_H_
#define _STM32N6_BASE_H_

#define ADC_BASE ((void *)0x50022000)

#define EXTI_BASE ((void *)0x56025000)

#define GPIOA_BASE ((void *)0x56020000)
#define GPIOB_BASE ((void *)0x56020400)
#define GPIOC_BASE ((void *)0x56020800)
#define GPIOD_BASE ((void *)0x56020c00)
#define GPIOE_BASE ((void *)0x56021000)
#define GPIOF_BASE ((void *)0x56021400)
#define GPIOG_BASE ((void *)0x56021800)
#define GPIOH_BASE ((void *)0x56021c00)
#define GPION_BASE ((void *)0x56023400)
#define GPIOO_BASE ((void *)0x56023800)
#define GPIOP_BASE ((void *)0x56023c00)
#define GPIOQ_BASE ((void *)0x56024000)

#define HASH_BASE ((void *)0x54020400)

#define I2C1_BASE ((void *)0x50005400)
#define I2C2_BASE ((void *)0x50005800)
#define I2C3_BASE ((void *)0x50005c00)
#define I2C4_BASE ((void *)0x56001c00)

#define PWR_BASE ((void *)0x56024800)
#define RCC_BASE ((void *)0x56028000)

#define RNG_BASE ((void *)0x54020000)

#define RTC_BASE ((void *)0x56004000)

#define SPI1_BASE ((void *)0x52003000)
#define SPI2_BASE ((void *)0x50003800)
#define SPI3_BASE ((void *)0x50003c00)
#define SPI4_BASE ((void *)0x52003400)
#define SPI5_BASE ((void *)0x52005000)
#define SPI6_BASE ((void *)0x56001400)

#define USART1_BASE  ((void *)0x52001000)
#define USART2_BASE  ((void *)0x50004400)
#define USART3_BASE  ((void *)0x50004800)
#define UART4_BASE   ((void *)0x50004c00)
#define UART5_BASE   ((void *)0x50005000)
#define USART6_BASE  ((void *)0x52001400)
#define UART7_BASE   ((void *)0x50007800)
#define UART8_BASE   ((void *)0x50007c00)
#define UART9_BASE   ((void *)0x52001800)
#define USART10_BASE ((void *)0x52001c00)

#endif /* _STM32N6_BASE_H_ */
