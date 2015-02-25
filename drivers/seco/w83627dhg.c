#include <linux/module.h>
#include <linux/init.h> 
#include <linux/slab.h>
#include <linux/string.h>    
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h> 
#include <linux/seco_lpc.h> 
#include <linux/seco_gpio_expander.h>
#include <linux/delay.h>

#include <linux/serial_8250.h>
#include <mach/imx-uart.h>    // provvisorio, vedere come splittarlo nel file di piattaforma

#include <linux/seco_w83627dhg.h>
#include <linux/seco_w83627dhg_gpio.h>


#define WB_INFO(fmt, arg...) printk(KERN_INFO "W83627DHG-P: " fmt "\n" , ## arg)
#define WB_ERR(fmt, arg...)  printk(KERN_ERR "%s: " fmt "\n" , __func__ , ## arg)
#define WB_DBG(fmt, arg...)  pr_debug("%s: " fmt "\n" , __func__ , ## arg)

/*
 *     w83627ehf - Driver for the hardware monitoring functionality of
 *                     the Winbond W83627EHF Super-I/O chip
 *                         w83627dhg    9      5       4       3      0xa020 0xc1    0x5ca3
 *                             w83627dhg-p  9      5       4       3      0xb070 0xc1    0x5ca3
 *                             */
/*
 *  * Super-I/O constants and functions
 *  */


#define W83627DHG_UARTA       0x02 	/* UART A */
#define W83627DHG_UARTB       0x03 	/* UART B IR */
#define W83627EHF_LD_HWM      0x0b
#define W83667HG_LD_VID       0x0d    /* Logical device = ?? */

#define SIO_REG_LDSEL         0x07	/* Logical device select */
#define SIO_REG_DEVID         0x20	/* Device ID (2 bytes) */
#define SIO_REG_GLOBALOPT     0x24	/* Global Options */
#define SIO_REG_EN_VRM10      0x2C	/* GPIO3, GPIO4 selection */
#define SIO_REG_ENABLE        0x30	/* Logical device enable */
#define SIO_REG_ADDR          0x60	/* Logical device address (2 bytes) */
#define SIO_REG_IRQ           0x70	/* Logical device irq*/
#define SIO_REG_VID_CTRL      0xF0	/* VID control */
#define SIO_REG_VID_DATA      0xF1	/* VID data */

#define CFG_REG_UART_A_CLK    0xF0 	/* CFG REGISTER OF THE UART A CLOCK */

#define CFG_REG_UART_B_CLK    0xF0 	/* CFG REGISTER OF THE UART B CLOCK */

#define WDT_CLOCK_1846200     (0x00)  /* 1.8462MHz clock source (24MHz/13) */ 
#define WDT_CLOCK_2000000     (0x01)  /* 2MHz clock source (24MHz/12) */
#define WDT_CLOCK_2400000     (0x02)  /* 24MHz clock source (24MHz/1) */

#define SIO_W83627EHF_ID      0x8850
#define SIO_W83627EHG_ID      0x8860
#define SIO_W83627DHG_ID      0xa020
#define SIO_W83627DHG_P_ID    0xb070
#define SIO_W83667HG_ID       0xa510
#define SIO_ID_MASK           0xFFF0

#define CFG_REG_IDX           (0x2E << 1)
#define CFG_REG_DATA          (0x2F << 1)


#define SLOT_UART_A           4
#define SLOT_UART_B           3

#define BASE_CHIP_ID          0xB070

#define CR_SOFT_RESET         0x02
#define CR_LOGICAL_DEV        0x07
#define CR_CHIP_ID_LO         0x20
#define CR_CHIP_ID_HI         0x21
#define CR_DEV_PWR_DOWN       0x22
#define CR_IPD                0x23
#define CR_GLOBAL_OPT1        0x24
#define CR_INTERFACE_TRI_EN   0x25
#define CR_GLOBAL_OPT2        0x26


#define REG_EFER              0x2E    // Extended Function Enable/Index Register
#define REG_EFDR              0x2F    // Extended Function Data Register


#define ENTER_EXT_MODE_CODE   0x87
#define EXIT_EXT_MODE_CODE    0xAA
#define SW_RESET_CODE         0x01


#define WINBOND_UART_CLOCK    24000000

#define SUPERIO_UART3_BASE    0x03F8
#define SUPERIO_UART4_BASE    0x02F8



struct w83627_device {
	int                       num_devices;
	struct platform_device    **pdevices;
	enum W83627DHG_P_DEVICE   *devices_type;
	struct lpc_device         *lpc_dev;
};


struct w83627_device *w83627;


////////////////////////////////////////////////////////////////
//                       BASIC FUNCTIONS                      //
////////////////////////////////////////////////////////////////

static inline int superio_outb (uint16_t addr, uint16_t value) {
	lpc_writew (REG_EFER, addr);
	lpc_writew (REG_EFDR, value);
	return 0;
}


static inline int superio_inb (uint16_t addr, uint16_t *data) {
	lpc_writew (REG_EFER, addr);
	lpc_readw (REG_EFDR, data);
	return 0;
}


int superio_readw (unsigned long addr, int offset) {
	uint16_t data;
	lpc_readw (((addr & 0xFFFF) >> 1) + offset, &data);
	return (int)data;
}


void superio_writew (unsigned long addr, int offset, int value) {
	lpc_writew (((addr & 0xFFFF) >> 1) + (unsigned long)offset, (int)value);
}


static inline void superio_enter_ext_mode (void) {
	lpc_writew (REG_EFER, ENTER_EXT_MODE_CODE);
	lpc_writew (REG_EFER, ENTER_EXT_MODE_CODE);
}


static inline void superio_exit_ext_mode (void) {
	lpc_writew (REG_EFER, EXIT_EXT_MODE_CODE);
}


static inline void superio_sw_reset (void) {
	lpc_writew (REG_EFER, CR_SOFT_RESET);
	lpc_writew (REG_EFDR, SW_RESET_CODE);
}


static int logicalDeviceValidate (enum W83627DHG_P_DEVICE  dev) {
	int isValid = 0;
	switch (dev) {
		case FDC:
		case PARALLEL_PORT:
		case UARTA:
		case UARTB:
		case KEYBOARD_CRTL:
		case SPI:
		case GPIO6:
		case WDTO_PLED:
		case GPIO2:
		case GPIO3:
		case GPIO4:
		case GPIO5:
		case ACPI:
		case HWMON:
		case PECI_SST:
			isValid = 1;
			break;
		default:
			isValid = 0;
	}
	return isValid;
}


static inline int superio_select (enum W83627DHG_P_DEVICE id) {
	int ret = -1;
	if (logicalDeviceValidate (id)) {
		superio_outb (CR_LOGICAL_DEV, (uint16_t)id);
		ret = (int)id;
	} else {
		ret = -1;
	}
	return ret;
}	



////////////////////////////////////////////////////////////////
//                        HWMON FUNCTIONS                     //
////////////////////////////////////////////////////////////////

static void hwmon_init (void) {
	uint16_t value;
	// set bit 0|1 CR2C to 11 to support pin78-85 to UARTB
	superio_inb (SIO_REG_EN_VRM10, &value);
	value |= 0x03;
	superio_outb (SIO_REG_EN_VRM10, value);
	// SELECT THE DEVICE HARDWARE MONITOR
	superio_outb (SIO_REG_LDSEL, W83627EHF_LD_HWM);
	// SET ADDRESS OF THE HARDWARE MONITOR
	superio_outb (SIO_REG_ADDR, 0x02);
	superio_outb (SIO_REG_ADDR + 1, 0x90);
	// ACTIVATE THE HARDWARE MONITOR
	superio_outb (SIO_REG_ENABLE, 0x01);
}



////////////////////////////////////////////////////////////////
//                        GPIO FUNCTIONS                      //
////////////////////////////////////////////////////////////////

#define GPIO_DEVICE 9

static int superio_gpio_read (unsigned long addr, int *data) {
	superio_enter_ext_mode ();
	superio_select (GPIO_DEVICE);
	superio_inb (addr, (uint16_t *)data);
	superio_exit_ext_mode ();
	return 0;
}


static int superio_gpio_write (unsigned long addr, int value) {
	superio_enter_ext_mode ();
	superio_select (GPIO_DEVICE);
	superio_outb (addr, value);
	superio_exit_ext_mode ();
	return 0;
}


static struct seco_w83627dhg_gpio_platform_data w83627dhg_gpio2_platform_data = {
	.label            = "w83627_gpio2",
	.direction_init   = 0xFF,
	.superio_read     = superio_gpio_read,
	.superio_write    = superio_gpio_write,
};


static struct platform_device w83627dhg_gpio2_device = {
	.name = "secoW83627DHG_gpio2",
	.id   = -1,
	.dev = {
		.platform_data = &w83627dhg_gpio2_platform_data,
	},
};


static struct seco_w83627dhg_gpio_platform_data w83627dhg_gpio3_platform_data = {
	.label            = "w83627_gpio3",
	.direction_init   = 0xFF,
	.superio_read     = superio_gpio_read,
	.superio_write    = superio_gpio_write,
};


static struct platform_device w83627dhg_gpio3_device = {
	.name = "secoW83627DHG_gpio3",
	.id   = -1,
	.dev = {
		.platform_data = &w83627dhg_gpio3_platform_data,
	},
};


////////////////////////////////////////////////////////////////
//                        UART FUNCTIONS                      //
////////////////////////////////////////////////////////////////

static void uart_a_irq_enable (void) {
	irq_slot_set (SLOT_UART_A, 1);
}


static void uart_a_irq_disable (void) {
	irq_slot_set (SLOT_UART_A, 0);
}


static void uart_b_irq_enable (void) {
	irq_slot_set (SLOT_UART_B, 1);
}


static void uart_b_irq_disable (void) {
	irq_slot_set (SLOT_UART_B, 0);
}


static void irq_after_handler (int irq, void *dev_id) {
	int value;
	struct plat_serial8250_port *pdata = ((struct device *)dev_id)->platform_data;
	if (pdata != NULL) {
		value = superio_readw (pdata[0].mapbase, 0x4);
		value |= 0x08;
		superio_writew (pdata[0].mapbase, 0x4, value);
		udelay (10);
		value &= 0xF7;
		superio_writew (pdata[0].mapbase, 0x4, value);
	}
}


static struct plat_serial8250_port w83627dhg_uart_a_platform_data[] = {
	{
		.mapbase               = (SUPERIO_UART3_BASE << 1),
		.flags                 = UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,// | UPF_IOREMAP,
		.iotype                = UPIO_LPC,
		.regshift              = 0,
		.uartclk               = WINBOND_UART_CLOCK,
		.irq_management        = NULL,	// initialized by driver 8250
		.irq_after_management  = NULL, //irq_after_handler,
		.irq_data              = NULL,	// initialized by driver 8250
		.irq_enable            = uart_a_irq_enable,
		.irq_disable           = uart_a_irq_disable,
	}, 
	{
		.flags = 0,
	},
};


static struct plat_serial8250_port w83627dhg_uart_b_platform_data[] = {
	{
		.mapbase               = (SUPERIO_UART4_BASE << 1),
		.flags                 = UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,// | UPF_IOREMAP,
		.iotype                = UPIO_LPC,
		.regshift              = 0,
		.uartclk               = WINBOND_UART_CLOCK,
		.irq_management        = NULL,	// initialized by driver 8250
		.irq_after_management  = NULL,  //irq_after_handler,
		.irq_data              = NULL,	// initialized by driver 8250
		.irq_enable            = uart_b_irq_enable,
		.irq_disable           = uart_b_irq_disable,


	},
	{
		.flags = 0,
	},
};


static struct platform_device w83627dhg_uart_a_device = {
	.name = "serial8250",
	.id   = 0,
	.dev = {
		.platform_data = &w83627dhg_uart_a_platform_data,
	},
};


static struct platform_device w83627dhg_uart_b_device = {
	.name = "serial8250",
	.id   = 1,
	.dev = {
		.platform_data = &w83627dhg_uart_b_platform_data,
	},
};


static void uart_a_init (void) {
	// SELECT THE DEVICE UART A
	superio_outb (SIO_REG_LDSEL, W83627DHG_UARTA);
	// SET ADDRESS OF THE PORT UART A
	superio_outb (SIO_REG_ADDR, ((SUPERIO_UART3_BASE & 0xFF00) >> 8));
	superio_outb (SIO_REG_ADDR + 1, (SUPERIO_UART3_BASE & 0x00FF));
        // SET CLOCK OF THE UART A 
	superio_outb (CFG_REG_UART_A_CLK, WDT_CLOCK_2400000);
        // ACTIVATE UART A
	superio_outb (SIO_REG_ENABLE, 0x01);
}


static void uart_b_init (void) {
	// SELECT THE DEVICE UART B
	superio_outb (SIO_REG_LDSEL, W83627DHG_UARTB);
	// SET ADDRESS OF THE PORT UART B
	superio_outb (SIO_REG_ADDR, ((SUPERIO_UART4_BASE & 0xFF00) >> 8));
	superio_outb (SIO_REG_ADDR + 1, (SUPERIO_UART4_BASE & 0x00FF));
	// SET CLOCK OF THE UART B 
	superio_outb (CFG_REG_UART_B_CLK, WDT_CLOCK_2400000);
        // ACTIVATE UART B
	superio_outb (SIO_REG_ENABLE, 0x01);
}


static uint16_t getChipID (void) {
	uint16_t tmp = 0x0000;
	uint16_t rev = 0x0000;

	udelay (10);
	superio_inb (CR_CHIP_ID_LO, &tmp);
	superio_inb (CR_CHIP_ID_HI, &rev);

	rev |= (tmp << 8);

	return rev;
}


static int chipIDvalidate (uint16_t id) {
	return (id & 0xFFF0) == BASE_CHIP_ID ? 1 : 0;
}


static int __devinit seco_w83627dhg_probe (struct platform_device *pdev) {
	struct seco_w83627dhg_platform_data *pdata;
	struct lpc_device *lpc_dev;
	int i;
	enum W83627DHG_P_DEVICE *dev_list;
	int ret = 0;
	uint16_t rev = 0;

	WB_INFO ("Probe starting...");

	pdata = pdev->dev.platform_data;
	if (pdata == NULL) {
		WB_ERR ("no platform_data!");
		return -EINVAL;
	}

	w83627 = kzalloc (sizeof(struct w83627_device), GFP_KERNEL);
	if (w83627 == NULL) {
		WB_ERR ("no device data!");
		return -EINVAL;
	}

	superio_sw_reset ();
	udelay (10);	
	
	superio_enter_ext_mode ();

	rev = getChipID ();
	
	if (chipIDvalidate (rev)) {
		dev_info (&pdev->dev, "Winbond chip ID: %x\n", rev);
	} else {
		dev_err (&pdev->dev, "SuperIO invalid chip ID: %x\n", rev);
		goto probe_failed;
	}

	hwmon_init ();

	if (pdata->num_devices < 1) {
		WB_INFO ("No devices associated to Winbond");
		goto dev_err_out;
	} else {
		w83627->num_devices = pdata->num_devices;
		w83627->pdevices = kzalloc(sizeof(struct platform_device *) * pdata->num_devices, GFP_KERNEL);
		w83627->lpc_dev = kzalloc(sizeof(struct lpc_device) * pdata->num_devices, GFP_KERNEL);
		if  (w83627->pdevices == NULL || w83627->lpc_dev == NULL) {
			WB_ERR ("Error in devices allocation");
		} else {
			for (i = 0, dev_list = pdata->devices ; i < pdata->num_devices ; i++, dev_list++) {
				switch (*dev_list) {
					case UARTA:
						WB_INFO ("UART_A device selected");
						w83627dhg_uart_a_platform_data[0].mapbase += lpc_getMemBase();
						w83627dhg_uart_a_platform_data[0].irq = lpc_getIRQ(0);
						w83627dhg_uart_a_platform_data[0].irqflags = lpc_getIRQ_flags(0) | IRQF_LPCSTYLE;
						uart_a_init ();

						w83627->pdevices[i] = &w83627dhg_uart_a_device;
						w83627->lpc_dev[i].lpc_slot = 4;
						w83627->lpc_dev[i].dev = &w83627->pdevices[i]->dev;

						w83627->lpc_dev[i].handler = &w83627dhg_uart_a_platform_data[0].irq_management;
						w83627->lpc_dev[i].after_handler = irq_after_handler;
						w83627->lpc_dev[i].irq_dev_id = w83627dhg_uart_a_platform_data[0].irq_data;
						w83627->lpc_dev[i].maskable = LPC_IRQ_MASKABLE;

						break;
					case UARTB:
						WB_INFO ("UART_B device selected");
						w83627dhg_uart_b_platform_data[0].mapbase += lpc_getMemBase();
						w83627dhg_uart_b_platform_data[0].irq = lpc_getIRQ(0);
						w83627dhg_uart_b_platform_data[0].irqflags = lpc_getIRQ_flags(0) | IRQF_LPCSTYLE;
						uart_b_init ();

						w83627->pdevices[i] = &w83627dhg_uart_b_device;
						w83627->lpc_dev[i].lpc_slot = 3;
						w83627->lpc_dev[i].dev = &w83627->pdevices[i]->dev;

						w83627->lpc_dev[i].handler = &w83627dhg_uart_b_platform_data[0].irq_management;
						w83627->lpc_dev[i].after_handler = irq_after_handler;
						w83627->lpc_dev[i].irq_dev_id = w83627dhg_uart_b_platform_data[0].irq_data;
						w83627->lpc_dev[i].maskable = LPC_IRQ_MASKABLE;

						break;
					case GPIO2:
						WB_INFO ("GPIO2 device selected");
						w83627dhg_gpio2_platform_data.gpio_base = pdata->gpio2_base;

						w83627->pdevices[i] = &w83627dhg_gpio2_device;
						// slop lpc don't used in the GPIO case 
						w83627->lpc_dev[i].lpc_slot = -1;
									
						break;
					case GPIO3:
						WB_INFO ("GPIO3 device selected");
						w83627dhg_gpio3_platform_data.gpio_base = pdata->gpio3_base;

						w83627->pdevices[i] = &w83627dhg_gpio3_device;
						// slop lpc don't used in the GPIO case 
						w83627->lpc_dev[i].lpc_slot = -1;
									
						break;
					default:
						WB_ERR ("Device not supported by SuperIO");
				}
			}
		}
	}
	superio_exit_ext_mode ();

	platform_add_devices (w83627->pdevices, w83627->num_devices);

	lpc_dev = w83627->lpc_dev;
	for (i = 0 ; i < w83627->num_devices ; i++, lpc_dev++) {
		if (lpc_dev->lpc_slot >= 0)
			lpc_add_device (lpc_dev);
	}
	

	return 0;
dev_err_out:
	kfree (w83627->lpc_dev);
	kfree (w83627->pdevices);
probe_failed:
	kfree (w83627);
	return ret;
}


static int __devexit seco_w83627dhg_remove (struct platform_device *pdev) {

	platform_device_unregister (pdev); 
	
	kfree (w83627);
	kfree (pdev);

	return 0;
}


static struct platform_driver seco_w83627dhg_driver = {
	.driver = {
		.name = "secoW83627DHG",
		.owner = THIS_MODULE,
	},
	.probe = seco_w83627dhg_probe,
	.remove = __devexit_p(seco_w83627dhg_remove),
};

MODULE_ALIAS("platform:seco_w836270dhg");


static int __init seco_w83627dhg_init (void) {
	return platform_driver_register (&seco_w83627dhg_driver);
}
subsys_initcall (seco_w83627dhg_init);


static void __exit seco_w83627dhg_exit (void) {
	platform_driver_unregister(&seco_w83627dhg_driver);
}
module_exit (seco_w83627dhg_exit);


MODULE_AUTHOR("DC SECO");
MODULE_DESCRIPTION("SECO winbond w83627dhg");
MODULE_LICENSE("GPL");
