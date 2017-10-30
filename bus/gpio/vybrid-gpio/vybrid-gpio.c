/**
 * Vybrid-GPIO kernel driver
 *
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * @file
 *
 * @copyright 2014 Phoenix Systems
 *
 * @author Horacio Mijail Anton Quiles <horacio.anton@phoesys.com>
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <hal/if.h>
#include <dev/gpio/if.h>
#include <vm/if.h>
#include <main/if.h>
#include <proc/if.h>
#include <hal/MVF50GS10MK50.h>
#include <dev/if.h>
#include <include/gpio.h>
#include <include/errno.h>
#include <lib/list.h>

#ifdef DEBUG_BUILD
	#define DEBUG_BOOL 1
	#define debug_printf(...)	main_printf(ATTR_DEBUG, __VA_ARGS__)
	#define DEBUG_FUNC
	#pragma message "*** DEBUG BUILD ***"
#else
	#define DEBUG_BOOL 0
	/* safe NOP macro*/
	#define debug_printf(...)	do {} while (0)
	#define DEBUG_FUNC			__attribute__((warning("A debug function is still being used!")))
#endif

struct GPIO_cfg_raw {	/*raw config data*/
	addr_t iomuxc;
	addr_t port;
	addr_t gpio;
	u32 irq;
	u32	pins_num;
};
static const struct GPIO_cfg_raw GPIO_cfg_raw_banks[] = {
		/*defined here instead of the BSP because this is all embedded and unchanging*/
		/*IOMUXC is the same for all pins, unlike PORTs and GPIOs*/
		{IOMUXC_BASE, PORT0_BASE, GPIO0_BASE, PORT0_IRQn,	32},
		{IOMUXC_BASE, PORT1_BASE, GPIO1_BASE, PORT1_IRQn,	32},
		{IOMUXC_BASE, PORT2_BASE, GPIO2_BASE, PORT2_IRQn,	32},
		{IOMUXC_BASE, PORT3_BASE, GPIO3_BASE, PORT3_IRQn,	32},
		{IOMUXC_BASE, PORT4_BASE, GPIO4_BASE, PORT4_IRQn,	7},
};


struct handler {
	gpio_callback_t *callback;
	void *arg;
};

struct GPIO_bank{
	IOMUXC_Type *iomuxc;	/* for electrical control */
	PORT_Type	*port;		/* for interrupt functionalities */
	GPIO_Type	*gpio;		/* for digital I/O */
	u32 irq;
	u32 lines;				/* no. of lines available in this bank*/
	struct handler handler[];	/*each line can have an ISR*/
};


static struct GPIO_bank **gpio_banks; /* array of pointers to GPIO_banks*/

static bool *isGPIO = NULL;
static u8 maxPinNum = 0;
static spinlock_t gpio_lock;

static int vybrid_gpio_ioctl(file_t* file, unsigned int cmd, unsigned long arg);

void gpio_resolve(GPIO_pinNumber_t pinNumber, unsigned int * bank, unsigned int * line)
{
	*bank = pinNumber / 32;
	*line = pinNumber % 32;
}


void gpio_config(unsigned int bank, unsigned int line_mask, gpio_mode_t mode, gpio_callback_t callback, void * isr_arg)
{
	struct GPIO_bank *cfg = gpio_banks[bank];
	u8 line;
	u8 trig = 0;
	
	if (mode == INPUT || mode == OUTPUT)
		callback = NULL;

	for (line = 0; line < cfg->lines; line++)
		if (line_mask & (1u << line)) {
			cfg->handler[line].arg = isr_arg;
			cfg->handler[line].callback = callback;
		}

	switch(mode) {
		case OUTPUT: 
			for(line = 0; line < cfg->lines; line++){
				if(line_mask & (1u << line)) {
					cfg->iomuxc->RGPIO[bank*32+line] |= IOMUXC_RGPIO_IBE_MASK;	/* enable input*/
					cfg->iomuxc->RGPIO[bank*32+line] |= IOMUXC_RGPIO_OBE_MASK;	/* enable output*/
					cfg->port->PCR[line] = 0;									/* disable ISR*/
				}
			}
			break;
			
		case INPUT:
			for(line = 0; line < cfg->lines; line++){
				if(line_mask & (1u << line)) {
					cfg->iomuxc->RGPIO[bank*32+line] &= ~IOMUXC_RGPIO_OBE_MASK;	/* disable output*/
					cfg->iomuxc->RGPIO[bank*32+line] |= IOMUXC_RGPIO_IBE_MASK;	/* enable input*/
					cfg->port->PCR[line] = 0;									/* disable ISR*/
				}
			}
			break;
			
		case IN_EDGE_FALLING:
		case IN_EDGE_RISING:
		case IN_EDGE_BOTH:
		case IN_LEVEL_LOW:
		case IN_LEVEL_HIGH:
			switch(mode){
				case IN_EDGE_FALLING:
					trig = (u8)0b1010;
					break;
				case IN_EDGE_RISING:
					trig = (u8)0b1001;
					break;
				case IN_EDGE_BOTH:
					trig = (u8)0b1011;
					break;
				case IN_LEVEL_LOW:
					trig = (u8)0b1000;
					break;
				case IN_LEVEL_HIGH:
					trig = (u8)0b1100;
					break;
				default:
					assert(0); /*unknown mode*/
			}
			for(line = 0; line < cfg->lines; line++){
				if(line_mask & (1u << line)) {
					cfg->iomuxc->RGPIO[bank*32+line] &= ~IOMUXC_RGPIO_OBE_MASK;	/* disable output*/
					cfg->iomuxc->RGPIO[bank*32+line] |= IOMUXC_RGPIO_IBE_MASK;	/* enable input*/
					cfg->port->PCR[line] = PORT_PCR_IRQC(trig);					/* enable ISR*/
				}
			}
			break;

		default:
			assert(0);
	}
}


void gpio_setline(unsigned int bank, unsigned int line, bool state)
{
	struct GPIO_bank *cfg = gpio_banks[bank];
	
	if (state)
		cfg->gpio->PSOR = 1u << line;
	else
		cfg->gpio->PCOR = 1u << line;
}


bool gpio_getline(unsigned int bank, unsigned int line)
{
	return (gpio_banks[bank]->gpio->PDIR >> line) & 1;
}


void gpio_high(unsigned int bank, unsigned int line_mask)
{
	gpio_banks[bank]->gpio->PSOR = line_mask;
}


void gpio_low(unsigned int bank, unsigned int line_mask)
{
	gpio_banks[bank]->gpio->PCOR = line_mask;
}


void gpio_toggle(unsigned int bank, unsigned int line_mask)
{
	gpio_banks[bank]->gpio->PTOR = line_mask;
}


/* The IRQ are bank-global, so on IRQ we have to search for the lines that have callbacks configured and call them*/
static int gpio_isr(u32 irq, cpu_context_t *ctx, void *arg)
{
	struct GPIO_bank *gpio_cfg = (struct GPIO_bank *)arg;
	typeof(gpio_cfg->port->ISFR) pending = gpio_cfg->port->ISFR;	/*bitmap of lines with pending IRQs in this bank */
	typeof(pending) a = 1, line;
	
	for(line=0; line<gpio_cfg->lines; line++, a<<=1){
		if(pending & a){
			if(gpio_cfg->handler[line].callback != NULL)
				gpio_cfg->handler[line].callback(line, gpio_cfg->handler[line].arg);
			gpio_cfg->port->ISFR = a;	/*clear interrupt flag*/
		}
	}

	return IHRES_HANDLED;
}


/** Prepare bank infrastructure from raw config info*/
static int gpio_init_one(const struct GPIO_cfg_raw *raw_cfg, struct GPIO_bank **gpio_cfg)
{
	int status, i;
	struct GPIO_bank *cfg;

	if ((cfg = vm_kmalloc(sizeof(struct GPIO_bank) + raw_cfg->pins_num * sizeof(struct handler))) == NULL) {
		main_printf(ATTR_ERROR, "Out of memory\n");
		return -ENOMEM;
	}

	cfg->lines = raw_cfg->pins_num;
	cfg->irq = raw_cfg->irq;
	
	if (cfg->irq >= 0)
		for (i = 0; i < cfg->lines; i++)
			cfg->handler[i].callback = NULL;
	
	status = vm_iomap(raw_cfg->port, sizeof(PORT_Type), PGHD_DEV_RW, (void **)&cfg->port);
	assert(status == EOK);
	/*IOMUX iomapped for all banks in the general init because it is common for all*/
	/*GPIO iomapped for all banks in the general init because its granularity is too small
	 * for vm_iomap*/

	hal_interruptsSetHandler(cfg->irq, gpio_isr, cfg);

	*gpio_cfg = cfg;
	return EOK;
}


static const u8 GPIO_banks_num = sizeof(GPIO_cfg_raw_banks)/sizeof(typeof(GPIO_cfg_raw_banks[0]));

/*
 * For Vybrid, the GPIO functionality involves 3 modules:
 * GPIO: sets or reads the state of a pad
 * IOMUX: connects the outside-world pin with the GPIO module pads, and sets the electrical characteristics
 * PORT: interrupt functionalities
 *
 * IOMUX needs clock settings (in BSP) and memory mapping, to change pin I/O direction at runtime
 * GPIO needs only memory mapping
 * PORT needs clock settings (in BSP) and memory mapping.
 */
//XXX might be interesting to make a pre-VM and post-VM initialization, so GPIO would be easy to use for debugging (or early initialization of devices) at any moment
int _gpio_init(void)
{
	static const file_ops_t vybrid_gpio_ops = {
		.ioctl = vybrid_gpio_ioctl
	};
	u8 banks_num = sizeof(GPIO_cfg_raw_banks)/sizeof(struct GPIO_cfg_raw);
	u8 i;
	s32 result = 0;
	s32 status;
	void * iomuxc_virt, * gpio_virt;
	u8 GPIO_pins[] = {GPIO_TABLE(EXPAND_AS_LIST_OF_PINS)};
	u8 pinsNum = 0;

	proc_spinlockCreate(&gpio_lock, "gpio_lock");
	/*bulk iomapping for later hand-adjustment*/
	status = vm_iomap(GPIO_cfg_raw_banks[0].iomuxc, sizeof(IOMUXC_Type), PGHD_DEV_RW, (void **)&iomuxc_virt);
	assert(status == EOK);
	status = vm_iomap(GPIO_cfg_raw_banks[0].gpio, sizeof(GPIO_Type), PGHD_DEV_RW, (void **)&gpio_virt);
	assert(status == EOK);

	gpio_banks = vm_kmalloc(GPIO_banks_num * sizeof(struct GPIO_bank *)); /*allocate the array of pointers*/
	assert(gpio_banks != NULL);

	for(i=0;i<banks_num;i++){
		if(gpio_init_one(&GPIO_cfg_raw_banks[i], &gpio_banks[i]) == EOK){
			/*all banks have the same iomuxc*/
			gpio_banks[i]->iomuxc = iomuxc_virt;
			/*set virtual address for GPIO, adjusted by the offset of the bank from the bank0 address*/
			gpio_banks[i]->gpio = (GPIO_Type *)((addr_t)gpio_virt + (GPIO_cfg_raw_banks[i].gpio - GPIO_cfg_raw_banks[0].gpio));
			main_printf(ATTR_INFO, "dev: GPIO base=0x%x, PORT base=0x%x, %d lines, irq=%d\n",
					GPIO_cfg_raw_banks[i].gpio,
					GPIO_cfg_raw_banks[i].port,
					GPIO_cfg_raw_banks[i].pins_num,
					GPIO_cfg_raw_banks[i].irq);

			/*set 32KHz clock source for digital filters*/
			gpio_banks[i]->port->DFCR = 1;
		} else {
			main_printf(ATTR_ERROR, "dev: vybrid-GPIO%d initialization failed\n", i);
			result++;
		}
	}

	if (dev_register(MAKEDEV(MAJOR_GPIO, 0), &vybrid_gpio_ops) < 0) {
		main_printf(ATTR_ERROR, "Vybrid-gpio: Can't register device for /dev/gpio!\n" );
	}
	assert(EOK==dev_mknod(MAKEDEV(MAJOR_GPIO, 0), "gpio"));
	pinsNum = sizeof(GPIO_pins) / sizeof(GPIO_pins[0]);
	if (pinsNum == 0)
		return result;
	for (i = 0; i < pinsNum; i++)
		if (GPIO_pins[i] > maxPinNum)
			maxPinNum = GPIO_pins[i];
	if ((isGPIO = (bool *)vm_kmalloc(sizeof(bool) * (maxPinNum + 1))) == NULL)
		return -ENOMEM;
	for (i = 0; i <= maxPinNum + 1; i++)
		*(isGPIO + i) = 0;
	for (i = 0; i < pinsNum; i++)
		*(isGPIO + GPIO_pins[i]) = 1;
	return result;
}


void gpio_disableFilters(unsigned int bank)
{
	gpio_banks[bank]->port->DFER = 0;
}


void gpio_setFilter(unsigned int bank, unsigned int line, bool state)
{
	u32 mask = 1u << line;
	if(state)
		gpio_banks[bank]->port->DFER |= mask;
	else
		gpio_banks[bank]->port->DFER &= ~mask;
}


bool gpio_setFilterLength(unsigned int bank, unsigned char length)
{
	if(gpio_banks[bank]->port->DFER != 0)
		return false;

	if(length > 31)		/* only 6 bits long on Vybrid */
		length = 31;
	gpio_banks[bank]->port->DFWR = length;
	return true;
}


void gpio_configPN(GPIO_pinNumber_t pinNumber, gpio_mode_t mode, gpio_callback_t callback, void * arg)
{
	u32 bank, line;
	gpio_resolve(pinNumber, &bank, &line);
	gpio_config(bank, 1u<<line, mode, callback, arg);
}


void gpio_setLinePN(GPIO_pinNumber_t pinNumber, bool state)
{
	u32 bank, line;
	gpio_resolve(pinNumber, &bank, &line);
	gpio_setline(bank, line, state);
}


bool gpio_getLinePN(GPIO_pinNumber_t pinNumber)
{
	u32 bank, line;
	bool state;
	gpio_resolve(pinNumber, &bank, &line);
	state = gpio_getline(bank, line);
	return state;
}


void gpio_setFilterPN(GPIO_pinNumber_t pinNumber, bool state)
{
	u32 bank, line;
	gpio_resolve(pinNumber, &bank, &line);
	gpio_setFilter(bank, line, state);
}


void gpio_setLineParamsPN(GPIO_pinNumber_t pinNumber, u32 params)
{
	/*all banks use the same iomuxc anyway*/
	gpio_banks[0]->iomuxc->RGPIO[pinNumber] = params;
}


extern void gpio_setLineParams(unsigned int bank, unsigned int line, unsigned int params)
{
	u8 rgpio = bank*32+line;
	gpio_banks[bank]->iomuxc->RGPIO[rgpio] = params;
}


void gpio_toggleLinePN(GPIO_pinNumber_t pinNumber)
{
	u32 bank, line;
	gpio_resolve(pinNumber, &bank, &line);
	gpio_toggle(bank, 1<<line);
}

static void gpio_toggleLinePN_adapter(void * arg)
{
	GPIO_pinNumber_t bn = (GPIO_pinNumber_t) arg; //XXX too hackish?
	gpio_toggleLinePN(bn);
}

void gpio_togglePulsePN(GPIO_pinNumber_t pinNumber, unsigned int us, timer_t * timer)
{
	gpio_toggleLinePN(pinNumber);
	if(timer == NULL){
		proc_threadSleep(us);
		gpio_toggleLinePN(pinNumber);
	} else {
		assert(sizeof(pinNumber)<=sizeof(void *));
		timesys_timerAdd(timer, us, gpio_toggleLinePN_adapter, (void *) pinNumber);
	}
}
typedef struct GpioMonitored_t{
	int oldState;
	unsigned num;
	LIST_ENTRY(GpioMonitored_t) other;
} GpioMonitored_t;

typedef struct {
	LIST_HEAD(GpioMonitored_t) mon;
	mutex_t lock;
} GpioData_t;

int vybrid_gpio_ioctl(file_t *file, unsigned int cmd, unsigned long arg) {
	assert(file != NULL);
	vnode_t *vnode = file->vnode;
	assert(vnode->type == vnodeDevice);
	process_t *proc = proc_current()->process;
	assert(proc != NULL);

	if (MAJOR(vnode->dev) != MAJOR_GPIO)
		return -EINVAL;
	if (cmd != GPIO_STATUS && (arg > maxPinNum || *(isGPIO + arg) == 0))
		return -EINVAL;
	if (cmd == GPIO_STATUS && (*(unsigned long *)arg > maxPinNum || *(isGPIO + *(unsigned long *)arg) == 0))
		return -EINVAL;
	proc_spinlockSet(&gpio_lock);
	switch (cmd) {
	case (GPIO_CONFIG_INPUT):
		gpio_configPN(arg, INPUT, NULL, NULL);
		break;
	case (GPIO_CONFIG_OUTPUT):
		gpio_configPN(arg, OUTPUT, NULL, NULL);
		break;
	case (GPIO_HIGH):
		gpio_setLinePN(arg, 0);
		break;
	case (GPIO_LOW):
		gpio_setLinePN(arg, 1);
		break;
	case (GPIO_TOGGLE):
		gpio_toggleLinePN(arg);
		break;
	case (GPIO_STATUS):
		*(unsigned long *)arg = gpio_getLinePN(*(unsigned long *)arg);
		break;
	default:
		proc_spinlockClear(&gpio_lock, sopNop);
		return -EINVAL;
	}
	proc_spinlockClear(&gpio_lock, sopNop);
	return EOK;
}
