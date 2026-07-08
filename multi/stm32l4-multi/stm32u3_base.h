/*
 * Phoenix-RTOS
 *
 * Multidrv-lib: Peripheral base address definitions for STM32U3
 *
 * Copyright 2026 Apator Metrix
 * Author: Mateusz Karcz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _STM32U3_BASE_H_
#define _STM32U3_BASE_H_

#define EXTI_BASE ((void *)0x50032000U)

#define GPDMA_BASE ((void *)0x50020000U)

#define USART1_BASE ((void *)0x50013800U)
#define USART2_BASE ((void *)0x50004400U)
#define USART3_BASE ((void *)0x50004800U)
#define UART4_BASE  ((void *)0x50004c00U)
#define UART5_BASE  ((void *)0x50005000U)

#define PWR_BASE ((void *)0x50030800U)
#define RCC_BASE ((void *)0x50030c00U)
#define RNG_BASE ((void *)0x520c0800U)
#define RTC_BASE ((void *)0x50007800U)

#define GPIOA_BASE ((void *)0x52020000U)
#define GPIOB_BASE ((void *)0x52020400U)
#define GPIOC_BASE ((void *)0x52020800U)
#define GPIOD_BASE ((void *)0x52020c00U)
#define GPIOE_BASE ((void *)0x52021000U)
#define GPIOF_BASE ((void *)0x52021400U)
#define GPIOG_BASE ((void *)0x52021800U)
#define GPIOH_BASE ((void *)0x52021c00U)

#endif /* _STM32U3_BASE_H_ */
