/*
 * Phoenix-RTOS
 *
 * UART 16550 device driver
 *
 * Hardware abstraction layer (TDA4VM)
 *
 * Copyright 2025 Phoenix Systems
 * Author: Aleksander Kaminski, Jacek Maksymowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/platform.h>
#include <board_config.h>
#include <phoenix/arch/armv7r/tda4vm/tda4vm.h>
#include <phoenix/arch/armv7r/tda4vm/tda4vm_pins.h>

#include "uart16550.h"

/* UART extension registers */
#define REG_EFR  2 /* Enhanced feature register */
#define REG_MDR1 8 /* Mode definition register 1 */
#define REG_MDR2 9 /* Mode definition register 2 */


#define MCU_UART0_BASE_ADDR ((void *)0x40a00000)
#define MCU_UART0_IRQ       30

#define MAIN_UART0_BASE_ADDR ((void *)0x02800000)
#define MAIN_UART1_BASE_ADDR ((void *)0x02810000)
#define MAIN_UART2_BASE_ADDR ((void *)0x02820000)
#define MAIN_UART3_BASE_ADDR ((void *)0x02830000)
#define MAIN_UART4_BASE_ADDR ((void *)0x02840000)
#define MAIN_UART5_BASE_ADDR ((void *)0x02850000)
#define MAIN_UART6_BASE_ADDR ((void *)0x02860000)
#define MAIN_UART7_BASE_ADDR ((void *)0x02870000)
#define MAIN_UART8_BASE_ADDR ((void *)0x02880000)
#define MAIN_UART9_BASE_ADDR ((void *)0x02890000)

#ifndef MAIN_UART0_IRQ
#define MAIN_UART0_IRQ -1
#endif

#ifndef MAIN_UART1_IRQ
#define MAIN_UART1_IRQ -1
#endif

#ifndef MAIN_UART2_IRQ
#define MAIN_UART2_IRQ -1
#endif

#ifndef MAIN_UART3_IRQ
#define MAIN_UART3_IRQ -1
#endif

#ifndef MAIN_UART4_IRQ
#define MAIN_UART4_IRQ -1
#endif

#ifndef MAIN_UART5_IRQ
#define MAIN_UART5_IRQ -1
#endif

#ifndef MAIN_UART6_IRQ
#define MAIN_UART6_IRQ -1
#endif

#ifndef MAIN_UART7_IRQ
#define MAIN_UART7_IRQ -1
#endif

#ifndef MAIN_UART8_IRQ
#define MAIN_UART8_IRQ -1
#endif

#ifndef MAIN_UART9_IRQ
#define MAIN_UART9_IRQ -1
#endif


#define MAX_PINS_PER_UART 6
typedef struct {
	volatile uint32_t *base;
	struct {
		int16_t clksel; /* Value of < 0 means this UART has no CLKSEL */
		uint8_t clksel_val;
	};
	int16_t clkdiv;
	uint8_t divisor;
	uint8_t pll;
	uint8_t hsdiv;
	int irq;
	struct {
		int16_t pin; /* Value of < 0 signals end of list */
		uint8_t muxSetting;
		uint8_t isTx;
	} pins[MAX_PINS_PER_UART];
	struct {
		uint32_t tx;
		uint32_t rx;
	} pins_selected;
} tda4vm_uart_info_t;


static const tda4vm_uart_info_t uarthw_info[] = {
	{
		.base = MCU_UART0_BASE_ADDR,
		.clksel = clksel_mcu_usart,
		.clksel_val = 0, /* CLKSEL set to MCU_PLL1_HSDIV3_CLKOUT */
		.clkdiv = -1,
		.divisor = 1,
		.pll = clk_mcu_per_pll1,
		.hsdiv = 3,
		.irq = MCU_UART0_IRQ,
		.pins = {
			{ pin_mcu_ospi1_d2, 4, 1 },
			{ pin_wkup_gpio0_10, 2, 1 },
			{ pin_wkup_gpio0_12, 0, 1 },
			{ pin_mcu_ospi1_d1, 4, 0 },
			{ pin_wkup_gpio0_11, 2, 0 },
			{ pin_wkup_gpio0_13, 0, 0 },
		},
		.pins_selected = {
			.tx = UART0_TX,
			.rx = UART0_RX,
		},
	},
};


typedef struct {
	volatile uint32_t *base;
	unsigned int irq;
} uarthw_ctx_t;


uint8_t uarthw_read(void *hwctx, unsigned int reg)
{
	volatile uint32_t *p = (((uarthw_ctx_t *)hwctx)->base + reg);

	return ((uint8_t)(*p));
}


void uarthw_write(void *hwctx, unsigned int reg, uint8_t val)
{
	volatile uint32_t *p = (((uarthw_ctx_t *)hwctx)->base + reg);

	*p = val;
}


char *uarthw_dump(void *hwctx, char *s, size_t sz)
{
	snprintf(s, sz, "base=0x%p irq=%u", (volatile void *)(((uarthw_ctx_t *)hwctx)->base), ((uarthw_ctx_t *)hwctx)->irq);
	return s;
}


unsigned int uarthw_irq(void *hwctx)
{
	return ((uarthw_ctx_t *)hwctx)->irq;
}


static int uarthw_setPin(const tda4vm_uart_info_t *info, uint32_t pin)
{
	platformctl_t pctl;
	pctl.action = pctl_set;
	pctl.type = pctl_pinconfig;

	for (size_t i = 0; i < MAX_PINS_PER_UART; i++) {
		if (info->pins[i].pin < 0) {
			return -EINVAL;
		}

		if (info->pins[i].pin == pin) {
			pctl.pin_config.pin_num = pin;
			pctl.pin_config.debounce_idx = 0;
			pctl.pin_config.mux = info->pins[i].muxSetting;
			if (info->pins[i].isTx) {
				pctl.pin_config.flags = TDA4VM_GPIO_PULL_DISABLE;
			}
			else {
				pctl.pin_config.flags = TDA4VM_GPIO_RX_EN | TDA4VM_GPIO_PULL_DISABLE;
			}

			return platformctl(&pctl);
		}
	}

	return -EINVAL;
}


int uarthw_init(unsigned int uartn, void *hwctx, size_t hwctxsz, unsigned int *fclk)
{
	void *base;
	platformctl_t pctl;
	int ret;
	if (hwctxsz < sizeof(uarthw_ctx_t)) {
		return -EINVAL;
	}

	if ((uartn >= sizeof(uarthw_info) / sizeof(uarthw_info[0]))) {
		return -ENODEV;
	}

	const tda4vm_uart_info_t *info = &uarthw_info[uartn];
	if ((info->base == 0) || (info->irq < 0)) {
		return -EINVAL;
	}

	if (info->clksel >= 0) {
		pctl.action = pctl_set;
		pctl.type = pctl_clksel;
		pctl.clksel_clkdiv.sel = info->clksel;
		pctl.clksel_clkdiv.val = info->clksel_val;
		if (platformctl(&pctl) < 0) {
			return -EIO;
		}
	}

	if (info->clkdiv >= 0) {
		pctl.action = pctl_set;
		pctl.type = pctl_clkdiv;
		pctl.clksel_clkdiv.sel = info->clkdiv;
		pctl.clksel_clkdiv.val = info->divisor;
		if (platformctl(&pctl) < 0) {
			return -EIO;
		}
	}

	pctl.action = pctl_get;
	pctl.type = pctl_frequency;
	pctl.frequency.pll_num = info->pll;
	pctl.frequency.hsdiv = info->hsdiv;

	if (platformctl(&pctl) < 0) {
		return -EIO;
	}

	if ((pctl.frequency.val == 0) || (pctl.frequency.val > UINT32_MAX)) {
		return -EIO;
	}

	*fclk = (unsigned int)pctl.frequency.val / info->divisor;
	ret = uarthw_setPin(info, info->pins_selected.tx);
	if (ret < 0) {
		return ret;
	}

	ret = uarthw_setPin(info, info->pins_selected.rx);
	if (ret < 0) {
		return ret;
	}

	base = mmap(NULL, _PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (addr_t)info->base);
	if (base == MAP_FAILED) {
		return -ENOMEM;
	}

	((uarthw_ctx_t *)hwctx)->base = base;
	((uarthw_ctx_t *)hwctx)->irq = info->irq;

	/* Put into UART x16 mode */
	uarthw_write(hwctx, REG_MDR1, 0x0);

	/* Enable enhanced functions */
	uarthw_write(hwctx, REG_LCR, 0xbf);
	uarthw_write(hwctx, REG_EFR, 1 << 4);
	uarthw_write(hwctx, REG_LCR, 0x0);

	return EOK;
}
