#ifndef __LINUX_SECO_GPIO_EXPANDER_H
#define __LINUX_SEC0_GPIO_EXPANDER_H

/* platform data for the Seco CPLD GPIO x8 I/O expander drive */

struct seco_gpio_exp_platform_data {
	/* number of the first GPIO */
	unsigned       gpio_base;
	/* label given to the GPIOs */
	const char     *label;
	/* initial direction of the GPIOs */
	uint8_t        direction_init;
	
	int            irq;
	unsigned int   flag_irq;
};

#endif  /* __LINUX_SEC0_GPIO_EXPANDER_H */
