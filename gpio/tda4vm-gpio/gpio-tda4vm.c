#include <sys/platform.h>
#include <sys/threads.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <limits.h>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <stdbool.h>
#include <stdatomic.h>

#include <phoenix/arch/armv7r/tda4vm/tda4vm.h>
#include <phoenix/arch/armv7r/tda4vm/tda4vm_pins.h>
#include "gpio-tda4vm.h"


#define GPIO_BANK_PINCOUNT     16U
#define GPIO_REGISTER_PINCOUNT 32U
#define GPIO_REG_STRIDE        10U

#define GPIO_PIN_TX_MODE 0U
#define GPIO_PIN_RX_MODE 1U

/* CTRLMMR_WKUP_PADCONFIGx bitfields */
#define CTRLMMR_WKUP_PADCONFIG_MUXMODE     0U
#define CTRLMMR_WKUP_PADCONFIG_VGPIO       4U
#define CTRLMMR_WKUP_PADCONFIG_ST_EN       14U
#define CTRLMMR_WKUP_PADCONFIG_FORCE_DS_EN 15U
#define CTRLMMR_WKUP_PADCONFIG_PULLUDEN    16U
#define CTRLMMR_WKUP_PADCONFIG_PULLTYPESEL 17U
#define CTRLMMR_WKUP_PADCONFIG_RXACTIVE    18U
#define CTRLMMR_WKUP_PADCONFIG_DRVSTR      19U
#define CTRLMMR_WKUP_PADCONFIG_TXDIS       21U

#define GPIO0_BASE      (0x600000UL)
#define GPIO1_BASE      (0x601000UL)
#define GPIO2_BASE      (0x610000UL)
#define GPIO3_BASE      (0x611000UL)
#define GPIO4_BASE      (0x620000UL)
#define GPIO5_BASE      (0x621000UL)
#define GPIO6_BASE      (0x630000UL)
#define GPIO7_BASE      (0x631000UL)
#define WKUP_GPIO1_BASE (0x42100000UL)
#define WKUP_GPIO0_BASE (0x42110000UL)

#define WKUP_GPIO_PIN_MIN 0U
#define WKUP_GPIO_PIN_MAX 83U

#define CTRLMMR_WKUP_PADCONFIG0 (0x4301C000)

#define GROUP_WKUP_GPIO0 0U
#define GROUP_GPIO0      1U
#define GROUP_GPIO1      2U
#define GPIO_GROUP_CNT   3U

#define GPIO_IDLE  0U
#define GPIO_INIT  1U
#define GPIO_READY 2U


struct gpio_group {
	handle_t lock;
	uint8_t initialised;
	uint8_t refCnt;
};


struct gpio_module {
	volatile uint32_t *base;
	volatile uint32_t *padcfg;
	uint8_t initialised;
	uint8_t busy;
	struct gpio_group *group;
};


enum {
	pid = 0,
	pcr,
	binten,
	dir01 = 4,
	out_data01,
	set_data01,
	clr_data01,
	in_data01,
	set_ris_trig01,
	clr_ris_trig01,
	set_fal_trig01,
	clr_fal_trig01,
	intstat01,
};


const addr_t paddr[] = { GPIO0_BASE, GPIO1_BASE, GPIO2_BASE, GPIO3_BASE, GPIO4_BASE,
	GPIO5_BASE, GPIO6_BASE, GPIO7_BASE, WKUP_GPIO0_BASE, WKUP_GPIO1_BASE };


const uint8_t groupMember[] = { GROUP_GPIO0, GROUP_GPIO1, GROUP_GPIO0, GROUP_GPIO1, GROUP_GPIO0,
	GROUP_GPIO1, GROUP_GPIO0, GROUP_GPIO1, GROUP_WKUP_GPIO0, GROUP_WKUP_GPIO0 };


static struct {
	handle_t gpioLock;
	volatile _Atomic uint8_t gpioReady;
	struct gpio_group groups[GPIO_GROUP_CNT];
	struct gpio_module modules[GPIO_MODULE_CNT];
} common;


static struct gpio_pinMapping {
	int gpioReg;      /* GPIO number in GPIO Register Set */
	int padconfigReg; /* GPIO number in CTRLMMR_WKUP_PADCONFIG registers */
} gpioMapping[] = {
	{ .gpioReg = 0, .padconfigReg = pin_wkup_gpio0_0 },
	{ .gpioReg = 54, .padconfigReg = pin_mcu_spi0_d1 },
	{ .gpioReg = 60, .padconfigReg = pin_mcu_i3c0_scl },
	{ .gpioReg = 61, .padconfigReg = pin_mcu_i3c0_sda },
};


static int gpio_globalInit(void)
{
	int ret;
	uint8_t expected = GPIO_IDLE;
	bool swap;

	swap = atomic_compare_exchange_strong(&common.gpioReady, &expected, GPIO_INIT);
	if (swap == true) {
		ret = mutexCreate(&common.gpioLock);
		if (ret == EOK) {
			atomic_store(&common.gpioReady, GPIO_READY);
		}
		else {
			atomic_store(&common.gpioReady, GPIO_IDLE);
		}
		return ret;
	}

	switch (expected) {
		case GPIO_READY:
			return EOK;

		case GPIO_INIT:
			return -EBUSY;

		default:
			return -EINVAL;
	}
}


static int gpio_checkArgs(int gpioModule, int gpioPin, struct gpio_pinMapping **mapping)
{
	int cnt;

	if (gpioModule < WKUP_GPIO0_IDX || gpioModule > WKUP_GPIO1_IDX) {
		return -EINVAL;
	}
	if (gpioPin < WKUP_GPIO_PIN_MIN || gpioPin > WKUP_GPIO_PIN_MAX) {
		return -EINVAL;
	}
	/* check if mapping in gpioMapping[] exists */
	for (cnt = 0; cnt < (sizeof(gpioMapping) / sizeof(struct gpio_pinMapping)); cnt++) {
		if (gpioMapping[cnt].gpioReg == gpioPin) {
			*mapping = &gpioMapping[cnt];
			break;
		}
	}
	if (*mapping == NULL) {
		return -ENXIO;
	}

	return EOK;
}


/* CTRLMMR_WKUP_PADCONFIG registers are using GPIO enumeration acc. to TDA4VM datasheet SPRSP36L p.136 */
static void gpio_setPinMode(int gpioModule, int gpioPin, uint8_t state)
{
	uint8_t vgpio = (gpioModule == WKUP_GPIO0_IDX) ? 0U : 1U;
	struct gpio_module *module = &common.modules[gpioModule];
	uint32_t reg;

	reg = *(module->padcfg + gpioPin);

	/* set muxmode */
	reg &= ~(0xFUL);
	reg |= 7UL;

	/* vgpio */
	reg &= ~(3UL << CTRLMMR_WKUP_PADCONFIG_VGPIO);
	reg |= (vgpio << CTRLMMR_WKUP_PADCONFIG_VGPIO);

	/* pull disable */
	reg |= (1UL << CTRLMMR_WKUP_PADCONFIG_PULLUDEN);

	if (state == GPIO_PIN_TX_MODE) {
		reg &= ~(1UL << CTRLMMR_WKUP_PADCONFIG_RXACTIVE);
		reg &= ~(1UL << CTRLMMR_WKUP_PADCONFIG_TXDIS);
	}
	else if (state == GPIO_PIN_RX_MODE) {
		reg |= (1UL << CTRLMMR_WKUP_PADCONFIG_RXACTIVE);
		reg |= (1UL << CTRLMMR_WKUP_PADCONFIG_TXDIS);
	}

	*(module->padcfg + gpioPin) = reg;
}


/* GPIO Registers are using regular GPIO enumeration (ex. WKUP_GPIO1_78 - Module 1, Pin 78) */
static void gpio_setPinState(int gpioModule, int gpioPin, int pinState)
{
	int bankNum, bankPin;
	uint32_t reg;
	struct gpio_module *module = &common.modules[gpioModule];

	/* compute bank and pin number */
	bankNum = gpioPin / GPIO_REGISTER_PINCOUNT;
	bankPin = gpioPin % GPIO_REGISTER_PINCOUNT;

	/* set output direction */
	*(module->base + dir01 + (bankNum * GPIO_REG_STRIDE)) &= ~(1UL << bankPin);

	reg = *(module->base + out_data01 + (bankNum * GPIO_REG_STRIDE));
	if (pinState == GPIO_PIN_STATE_HIGH) {
		reg |= (1UL << bankPin);
	}
	else if (pinState == GPIO_PIN_STATE_LOW) {
		reg &= ~(1UL << bankPin);
	}
	*(module->base + out_data01 + (bankNum * GPIO_REG_STRIDE)) = reg;
}


static uint8_t gpio_readPinState(int gpioModule, int gpioPin)
{
	int bankNum, bankPin;
	uint32_t reg;
	struct gpio_module *module = &common.modules[gpioModule];

	/* compute bank and pin number */
	bankNum = gpioPin / GPIO_REGISTER_PINCOUNT;
	bankPin = gpioPin % GPIO_REGISTER_PINCOUNT;

	reg = *(module->base + in_data01 + (bankNum * GPIO_REG_STRIDE));

	return (uint8_t)((reg >> bankPin) & 1U);
}


int gpio_writePin(int gpioModule, int gpioPin, uint8_t pinState)
{
	struct gpio_module *module;
	struct gpio_pinMapping *mapping = NULL;
	int ret;

	ret = gpio_checkArgs(gpioModule, gpioPin, &mapping);
	if (ret != EOK) {
		return ret;
	}
	module = &common.modules[gpioModule];

	ret = gpio_globalInit();
	if (ret != EOK) {
		fprintf(stderr, "[GPIO] Module %d not initialized\n", gpioModule);
		return ret;
	}

	mutexLock(common.gpioLock);
	if (module->initialised == 0U) {
		mutexUnlock(common.gpioLock);
		return -ENODEV;
	}
	mutexLock(module->group->lock);
	mutexUnlock(common.gpioLock);

	gpio_setPinMode(gpioModule, mapping->padconfigReg, GPIO_PIN_TX_MODE);

	gpio_setPinState(gpioModule, mapping->gpioReg, pinState);

	mutexUnlock(module->group->lock);

	return EOK;
}


int gpio_readPin(int gpioModule, int gpioPin)
{
	struct gpio_module *module;
	struct gpio_pinMapping *mapping = NULL;
	int ret;

	ret = gpio_checkArgs(gpioModule, gpioPin, &mapping);
	if (ret != EOK) {
		return ret;
	}
	module = &common.modules[gpioModule];

	ret = gpio_globalInit();
	if (ret != EOK) {
		fprintf(stderr, "[GPIO] Module %d not initialized\n", gpioModule);
		return ret;
	}

	mutexLock(common.gpioLock);
	if (module->initialised == 0U) {
		mutexUnlock(common.gpioLock);
		return -ENODEV;
	}
	mutexLock(module->group->lock);
	mutexUnlock(common.gpioLock);

	gpio_setPinMode(gpioModule, mapping->padconfigReg, GPIO_PIN_RX_MODE);

	ret = (int)gpio_readPinState(gpioModule, mapping->gpioReg);

	mutexUnlock(module->group->lock);

	return ret;
}


int gpio_initModule(int num)
{
	int ret;
	platformctl_t pctl;
	struct gpio_module *module;

	/* initialize global lock */
	ret = gpio_globalInit();
	if (ret != EOK) {
		return ret;
	}

	/* TODO: add support for MAIN GPIO */
	if (num < WKUP_GPIO0_IDX || num > WKUP_GPIO1_IDX) {
		return -EINVAL;
	}
	module = &common.modules[num];

	mutexLock(common.gpioLock);
	if (module->initialised == 1U) {
		mutexUnlock(common.gpioLock);
		return EOK;
	}

	module->base = mmap(NULL, PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)paddr[num]);
	if (module->base == MAP_FAILED) {
		mutexUnlock(common.gpioLock);
		return -ENOMEM;
	}

	module->padcfg = mmap(NULL, PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)CTRLMMR_WKUP_PADCONFIG0);
	if (module->padcfg == MAP_FAILED) {
		(void)munmap((void *)module->base, PAGE_SIZE);
		mutexUnlock(common.gpioLock);
		return -ENOMEM;
	}

	module->group = &common.groups[groupMember[num]];

	pctl.action = pctl_set;
	pctl.type = pctl_clksel;
	pctl.clksel_clkdiv.sel = clksel_wkup_gpio;
	pctl.clksel_clkdiv.val = 0; /* MCU_SYSCLK0/6 */
	ret = platformctl(&pctl);
	if (ret < 0) {
		(void)munmap((void *)module->base, PAGE_SIZE);
		(void)munmap((void *)module->padcfg, PAGE_SIZE);
		mutexUnlock(common.gpioLock);
		return -EIO;
	}

	/* every module is protected by its group's lock */
	if (module->group->initialised == 0U) {
		ret = mutexCreate(&module->group->lock);
		if (ret < 0) {
			(void)munmap((void *)module->base, PAGE_SIZE);
			(void)munmap((void *)module->padcfg, PAGE_SIZE);
			mutexUnlock(common.gpioLock);
			return ret;
		}
		module->group->initialised = 1U;
	}
	module->group->refCnt += 1;
	module->initialised = 1U;

	mutexUnlock(common.gpioLock);

	return EOK;
}


int gpio_deinitModule(int num)
{
	int ret;
	struct gpio_module *module;
	struct gpio_group *group;

	if (num < WKUP_GPIO0_IDX || num > WKUP_GPIO1_IDX) {
		return -EINVAL;
	}

	module = &common.modules[num];
	group = module->group;

	ret = gpio_globalInit();
	if (ret != EOK) {
		return ret;
	}

	mutexLock(common.gpioLock);
	if (module->initialised == 0U) {
		mutexUnlock(common.gpioLock);
		return EOK;
	}

	mutexLock(group->lock);
	module->initialised = 0U;
	module->group->refCnt -= 1U;

	(void)munmap((void *)module->base, PAGE_SIZE);
	(void)munmap((void *)module->padcfg, PAGE_SIZE);
	mutexUnlock(group->lock);
	if (module->group->refCnt == 0U) {
		module->group->initialised = 0U;
		(void)resourceDestroy(group->lock);
	}
	mutexUnlock(common.gpioLock);

	return EOK;
}
