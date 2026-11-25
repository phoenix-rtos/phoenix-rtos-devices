#ifndef _GPIO_TDA4VM_
#define _GPIO_TDA4VM_

/* GPIO module ID */
#define GPIO0_IDX       0U
#define GPIO1_IDX       1U
#define GPIO2_IDX       2U
#define GPIO3_IDX       3U
#define GPIO4_IDX       4U
#define GPIO5_IDX       5U
#define GPIO6_IDX       6U
#define GPIO7_IDX       7U
#define WKUP_GPIO0_IDX  8U
#define WKUP_GPIO1_IDX  9U
#define GPIO_MODULE_CNT 10U

/* Pin state */
#define GPIO_PIN_STATE_LOW  0U
#define GPIO_PIN_STATE_HIGH 1U


int gpio_initModule(int module);


int gpio_deinitModule(int module);

/*
 * @param module: GPIO module index
 * @param pin:    GPIO pin number acc. TDA4VM TRM section 12.1.2
 * @param state:  GPIO_PIN_STATE_LOW, GPIO_PIN_STATE_HIGH
 */
int gpio_writePin(int module, int pin, uint8_t state);


int gpio_readPin(int module, int pin);

#endif
