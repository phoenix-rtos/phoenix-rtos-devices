/*
 * Phoenix-RTOS
 *
 * Multidrv-lib: Peripheral base address definitions for STM32H5
 *
 * Copyright 2025, 2026 Phoenix Systems
 * Author: Jacek Maksymowicz, Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _STM32H5_BASE_H_
#define _STM32H5_BASE_H_

#define ADC_BASE ((void *)0x52028000)

#define EXTI_BASE ((void *)0x54022000)

#define GPDMA_BASE ((void *)0x50020000)
#define HPDMA_BASE ((void *)0x50021000)

#define GPIOA_BASE ((void *)0x52020000)
#define GPIOB_BASE ((void *)0x52020400)
#define GPIOC_BASE ((void *)0x52020800)
#define GPIOD_BASE ((void *)0x52020C00)
#define GPIOE_BASE ((void *)0x52021000)
#define GPIOF_BASE ((void *)0x52021400)
#define GPIOG_BASE ((void *)0x52021800)
#define GPIOH_BASE ((void *)0x52021C00)
#define GPIOI_BASE ((void *)0x52022000)

#define HASH_BASE ((void *)0x520C0400)

#define I2C1_BASE ((void *)0x50005400)
#define I2C2_BASE ((void *)0x50005800)
#define I2C3_BASE ((void *)0x54002800)
#define I2C4_BASE ((void *)0x54002C00)

#define PWR_BASE ((void *)0x54020800)
#define RCC_BASE ((void *)0x54020C00)

#define RNG_BASE ((void *)0x520C0800)

#define RTC_BASE ((void *)0x54007800)

#define SPI1_BASE ((void *)0x50013000)
#define SPI2_BASE ((void *)0x50003800)
#define SPI3_BASE ((void *)0x50003C00)
#define SPI4_BASE ((void *)0x50014C00)
#define SPI5_BASE ((void *)0x54002000)
#define SPI6_BASE ((void *)0x50015000)

#define USART1_BASE  ((void *)0x50013800)
#define USART2_BASE  ((void *)0x50004400)
#define USART3_BASE  ((void *)0x50004800)
#define UART4_BASE   ((void *)0x50004C00)
#define UART5_BASE   ((void *)0x50005000)
#define USART6_BASE  ((void *)0x50006400)
#define UART7_BASE   ((void *)0x50007800)
#define UART8_BASE   ((void *)0x50007C00)
#define UART9_BASE   ((void *)0x50008000)
#define USART10_BASE ((void *)0x50006800)
#define USART11_BASE ((void *)0x50006C00)
#define UART12_BASE  ((void *)0x50008400)

#define TIM1_BASE  ((void *)0x50012C00)
#define TIM2_BASE  ((void *)0x50000000)
#define TIM3_BASE  ((void *)0x50000400)
#define TIM4_BASE  ((void *)0x50000800)
#define TIM5_BASE  ((void *)0x50000C00)
#define TIM6_BASE  ((void *)0x50001000)
#define TIM7_BASE  ((void *)0x50001400)
#define TIM8_BASE  ((void *)0x50013400)

/* No TIM9-11 */

#define TIM12_BASE ((void *)0x50001800)
#define TIM13_BASE ((void *)0x50001C00)
#define TIM14_BASE ((void *)0x50002000)
#define TIM15_BASE ((void *)0x50014000)

/* No TIM16 */

#define TIM17_BASE ((void *)0x50014800)
#define TIM18_BASE ((void *)0x50014400)


#endif /* _STM32H5_BASE_H_ */
