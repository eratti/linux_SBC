#ifndef __LINUX_SECO_W83627DHG_GPIO_H
#define __LINUX_SECO_W83627DHG_GPIO_H


struct seco_w83627dhg_gpio_platform_data {
	/* number of the first GPIO */
	unsigned       gpio_base;
	/* label given to the GPIOs */
	const char     *label;
	/* initial direction of the GPIOs */
	uint8_t        direction_init;
	/* function list supported from w83627 driver */
	/* read operation */
	int            (*superio_read)(unsigned long addr, int *data);
	/* write operation */
	int            (*superio_write)(unsigned long addr, int value);
};


#endif  /* __LINUX_SECO_W83627DHG_GPIO_H */

