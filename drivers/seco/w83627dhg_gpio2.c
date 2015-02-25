
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/seco_w83627dhg_gpio.h>


#define GPIO2_REG_MUX_EN1     0x29  // EN for GPIOs 2.0, 2.1
#define GPIO2_MUX_EN_B1_1     0xFB  // in AND
#define GPIO2_MUX_EN_B1_2     0x02  // in OR
#define GPIO2_REG_MUX_EN2     0x2A  // EN for GPIOs 2.4, 2.5, 2.6, 2.7
#define GPIO2_MUX_EN_B2_1     0x01  // in OR

#define GPIO2_REG_EN          0x30
#define GPIO2_REG_DIR         0xE3
#define GPIO2_REG_BUFFER      0xE4
#define GPIO2_REG_INV_LOGIC   0xE5
#define GPIO2_REG_STATUS      0xE6

#define NR_GPIO    8

#define GPIO1      0x01
#define GPIO2      0x02
#define GPIO3      0x04
#define GPIO4      0x08
#define GPIO5      0x0F
#define GPIO6      0x20
#define GPIO7      0x40
#define GPIO8      0x80
#define GPIO_ALL   0xFF

#define GPIO2_MASK_EN          0x01

#define GPIO2_EN          0x01
#define GPIO2_DIR_IN      0x01
#define GPIO2_INV_LOGIC   0x01


#define DIR_OUT  0x0
#define DIR_IN   0x1


#define WBGPIO_INFO(fmt, arg...) printk(KERN_INFO "W83627DHG_GPIO2: " fmt "\n" , ## arg)
#define WBGPIO_ERR(fmt, arg...)  printk(KERN_ERR "%s: " fmt "\n" , __func__ , ## arg)
#define WBGPIO_DBG(fmt, arg...)  pr_debug("%s: " fmt "\n" , __func__ , ## arg) 

struct w83627dhg_gpio_chip {
	struct gpio_chip   gpio_chip;
	/* ghost register of the buffer register: to avoid expencive reading in set value functions*/
	uint8_t	           reg_out;

	unsigned int       mask;
	uint8_t            dir_input;
	uint8_t            dir_output;

	struct mutex       lock;

	/* read operation */
	int                (*superio_read)(unsigned long addr, int *data);
	/* write operation */
	int                (*superio_write)(unsigned long addr, int value);

};


static int w83627dhg_gpio_get_value (struct gpio_chip *gc, unsigned offset) {
	struct w83627dhg_gpio_chip *chip;
	int reg_value;
	int ret = 0;
	uint8_t mask = 1u << offset;

	chip = container_of (gc, struct w83627dhg_gpio_chip, gpio_chip);
	ret = chip->superio_read (GPIO2_REG_BUFFER, &reg_value);
	if (ret < 0) {
		pr_err ("%s, operation failed!\n", __func__);
		return 0;
	}

	return (int)((reg_value & mask) >> offset);
}


static void w83627dhg_gpio_set_value (struct gpio_chip *gc, unsigned offset, int value) {
	struct w83627dhg_gpio_chip *chip;
	uint8_t reg_out;
	uint8_t mask = 1u << offset;
	int ret = 0;

	chip = container_of (gc, struct w83627dhg_gpio_chip, gpio_chip);

	mutex_lock (&chip->lock);

	reg_out = (value) ? chip->reg_out | mask : chip->reg_out & ~mask;

	ret = chip->superio_write (GPIO2_REG_BUFFER, reg_out);
	if (ret < 0) {
		pr_err ("%s, operation failed!\n", __func__);
		goto out;
	}

	chip->reg_out = reg_out;

out:
	mutex_unlock (&chip->lock);
}


static uint8_t seco_gpio_exp_direction_reg (struct w83627dhg_gpio_chip *chip) {
	uint8_t reg = 0x0;
	uint8_t mask = 0x0;
	int port;
	for (port = 0 ; port < NR_GPIO ; port++) {
		mask = 1u << port;
		if (chip->dir_output & mask) {   // direction as output
			reg &= ~mask;
		} else if (chip->dir_input & mask) {   // direction as input
			reg |= mask;
		}
	}
	return reg;
}


/*
 ** /sys/devices/platform/secoGPIO
 **   /directions
 **        Read Only. Shows the current directions of all the GPIOs
 **   /values
 **        Read Only. Shows the current values of all the GPIOs.
 **        (for the GPIOs setted as output, the shown values will are the setted output values;
 **        for the GPIOs setted as input, the shown values will are the readed input values).
 **/


static ssize_t gpio_directions_show (struct device *dev,
		struct device_attribute *attr, char *buf) {
	ssize_t status;
	// three character, plus blank space, every gpio
	struct seco_w83627dhg_gpio_platform_data *pdata;

	char *leg =  " g1  g2  g3  g4  g5  g6  g7";
	char *stat = "                           ";
	int reg, i;
	int mask = 0x1;
	int ret;
	pdata = dev->platform_data;
	ret = pdata->superio_read (GPIO2_REG_DIR, &reg);
	if (ret < 0) {
		status = sprintf (buf, "Error in reading directions\n");
	}
	for (i = 0 ; i < NR_GPIO ; i++) {
		if (!(reg & mask)) {
			stat[4*i+2] = 'O';
		} else {
			stat[4*i+2] = 'I';
		}
		reg >>= 1;
	}

	status = sprintf (buf, "%s\n%s\n", leg, stat);

	return status;

}

static DEVICE_ATTR (directions, 0444, gpio_directions_show, NULL);


static ssize_t gpio_values_show (struct device *dev,
		struct device_attribute *attr, char *buf) {
	// three character, plus blank space, every gpio
	struct seco_w83627dhg_gpio_platform_data *pdata;
	ssize_t status;

	char *leg =  " g1  g2  g3  g4  g5  g6  g7";
	char *stat = "                           ";
	int reg, i;
	int mask = 0x1;
	int ret;
	pdata = dev->platform_data;
	ret = pdata->superio_read (GPIO2_REG_BUFFER, &reg);
	if (ret < 0) {
		status = sprintf (buf, "Error in reading values\n");
	}
	for (i = 0 ; i < NR_GPIO ; i++) {
		if (!(reg & mask)) {
			stat[4*i+2] = '0';
		} else {
			stat[4*i+2] = '1';
		}
		reg >>= 1;
	}

	status = sprintf (buf, "%s\n%s\n", leg, stat);

	return status;

}

static DEVICE_ATTR (values, 0444, gpio_values_show, NULL);


static int w83627dhg_gpio_direction_input (struct gpio_chip *gc, unsigned offset) {
	struct w83627dhg_gpio_chip *chip;
	uint8_t reg_value;

	chip = container_of (gc, struct w83627dhg_gpio_chip, gpio_chip);

	mutex_lock (&chip->lock);

	chip->dir_output &= ~(GPIO2_DIR_IN << offset);
	chip->dir_input |= GPIO2_DIR_IN << offset;

	reg_value = seco_gpio_exp_direction_reg (chip);

	chip->superio_write (GPIO2_REG_DIR, (int)reg_value);

	mutex_unlock (&chip->lock);

	return 0;
}


static int w83627dhg_gpio_direction_output (struct gpio_chip *gc, unsigned offset, int value) {
	struct w83627dhg_gpio_chip *chip;
	uint8_t reg_value;

	chip = container_of (gc, struct w83627dhg_gpio_chip, gpio_chip);

	mutex_lock (&chip->lock);

	chip->dir_output |= GPIO2_DIR_IN << offset;
	chip->dir_input &= ~(GPIO2_DIR_IN << offset);

	reg_value = seco_gpio_exp_direction_reg (chip);

	chip->superio_write (GPIO2_REG_DIR, (int)reg_value);

	mutex_unlock (&chip->lock);

	// set the output
	w83627dhg_gpio_set_value (gc, offset, value);

	return 0;
}


static int seco_w83627dgh_gpio_setup (struct platform_device *device, struct seco_w83627dhg_gpio_platform_data *pdata, struct w83627dhg_gpio_chip *chip) {
	int port = 0;
	int reg;
	struct gpio_chip *gc = &chip->gpio_chip;
	uint8_t dir_init = pdata->direction_init;

	for (port = 0 ; port < NR_GPIO ; port++, dir_init >>= 1) {
		uint8_t mask = 1 << port;
		// 1 for input, 0 for output
		switch (dir_init & 0x1) {
			case DIR_OUT:
				chip->dir_output |= mask;
				chip->dir_input &= ~mask;
				break;
			case DIR_IN:
				chip->dir_output &= ~mask;
				chip->dir_input |= mask;
				break;
			default:
				// default as input
				chip->dir_output &= ~mask;
				chip->dir_input |= mask;
		}
	}

	/* set and init the gpio hardware registers */
		// set the pin muxing
        chip->superio_read (GPIO2_REG_MUX_EN1, &reg);
	reg &= GPIO2_MUX_EN_B1_1;
	reg |= GPIO2_MUX_EN_B1_2;
        chip->superio_write (GPIO2_REG_MUX_EN1, reg);	
	chip->superio_read (GPIO2_REG_MUX_EN2, &reg);
	reg |= GPIO2_MUX_EN_B2_1;
        chip->superio_write (GPIO2_REG_MUX_EN2, reg);	

		// enable GPIO2
	
	chip->superio_read (GPIO2_REG_EN, &reg);
	chip->superio_write (GPIO2_REG_EN, reg | GPIO2_MASK_EN);
		// set all GPIO's direction
	chip->superio_write (GPIO2_REG_DIR, 0xFF & seco_gpio_exp_direction_reg(chip));
		// set all GPIOs as not inverted
	chip->superio_write (GPIO2_REG_INV_LOGIC, (~(0xFF * GPIO2_INV_LOGIC)) & GPIO_ALL);

	// set the gpiolib interface
	gc->direction_input = w83627dhg_gpio_direction_input;
	gc->direction_output = w83627dhg_gpio_direction_output;

	gc->set = w83627dhg_gpio_set_value;
	gc->get = w83627dhg_gpio_get_value;

	gc->can_sleep = 1;

	gc->base = pdata->gpio_base;
	gc->ngpio = NR_GPIO;
	gc->label = pdata->label;
	gc->dev = &device->dev;
	gc->owner = THIS_MODULE;

	return port;
}


static int __devinit seco_w83627dhg_gpio_probe (struct platform_device *pdev) {
	struct w83627dhg_gpio_chip *chip;
	struct seco_w83627dhg_gpio_platform_data *pdata;
	int ret = 0;

	WBGPIO_INFO ("SECO W83627 GPIO2 Expander star probe");

	pdata = pdev->dev.platform_data;
	if (pdev == NULL) {
		dev_err (&pdev->dev, "no platform data!\n");                                             
		return -EINVAL;                                                                          
	}

	chip = kzalloc (sizeof(struct w83627dhg_gpio_chip), GFP_KERNEL); 
	if (chip == NULL) {
		return -EINVAL;                                                                          
	}

	if (pdata->superio_read == NULL || pdata->superio_write == NULL) {
		WBGPIO_ERR ("Error in probing: missing data");
		ret = -EIO;
		goto probe_failed;
	}

	chip->superio_read = pdata->superio_read;
	chip->superio_write = pdata->superio_write;

	ret = seco_w83627dgh_gpio_setup (pdev, pdata, chip);


	if (ret != NR_GPIO) {
		WBGPIO_ERR ("Error in probing: gpio error");
		ret = -EINVAL;
		goto probe_failed;
	}

	mutex_init (&chip->lock);

	ret = device_create_file (&pdev->dev, &dev_attr_directions);
	ret = device_create_file (&pdev->dev, &dev_attr_values);

	ret = gpiochip_add (&chip->gpio_chip);
	if (ret < 0) {
		WBGPIO_ERR ("Error in probing: gpio error");
		ret = -EINVAL;
		goto probe_failed;
	}

	WBGPIO_INFO ("W83627 GPIO2 Expander probed!");

	return 0;

probe_failed:
	kfree (chip);
	return ret;
}


static int __devexit seco_w83627dhg_gpio_remove (struct platform_device *pdev) {
	return 0;
}


static struct platform_driver seco_w83627dhg_gpio_driver = {
	.driver = {
		.name = "secoW83627DHG_gpio2",
		.owner = THIS_MODULE,
	},
	.probe = seco_w83627dhg_gpio_probe,
	.remove = __devexit_p(seco_w83627dhg_gpio_remove),
};


static int __init seco_w83627dhg_gpio_init (void) {
	return platform_driver_register (&seco_w83627dhg_gpio_driver);
}
subsys_initcall(seco_w83627dhg_gpio_init);


static void __exit seco_w83627dhg_gpio_exit (void) {
	        platform_driver_unregister(&seco_w83627dhg_gpio_driver);
}
module_exit(seco_w83627dhg_gpio_exit);


MODULE_AUTHOR("DC SECO");
MODULE_DESCRIPTION("SECO W83627DHG_P GPIO");
MODULE_LICENSE("GPL");

