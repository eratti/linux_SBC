#include <linux/module.h>
#include <linux/init.h> 
#include <linux/slab.h>
#include <linux/string.h>    
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h> 
#include <linux/seco_lpc.h> 
#include <linux/delay.h>

#include <linux/serial_8250.h>
#include <mach/imx-uart.h>    // provvisorio, vedere come splittarlo nel file di piattaforma

#include <linux/seco_xr28v382.h>


#define XR_INFO(fmt, arg...) printk(KERN_INFO "XR28V382: " fmt "\n" , ## arg)
#define XR_ERR(fmt, arg...)  printk(KERN_ERR "%s: " fmt "\n" , __func__ , ## arg)
#define XR_DBG(fmt, arg...)  pr_debug("%s: " fmt "\n" , __func__ , ## arg)


#define XR_DEBUG 0


/*  GLOBAL REGISTERs  */
#define REG_GBL_SREST                      0x02     /*  Software Reset  */
#define REG_GBL_LDN                        0x07     /*  Logic Device Number Register  */
#define REG_GBL_DEV_ID_M                   0x20     /*  Device ID MSB Register  */
#define REG_GBL_DEV_ID_L                   0x21     /*  Device ID LSB Register  */
#define REG_GBL_VID_M                      0x23     /*  Vendor ID MSB Register  */
#define REG_GBL_VID_L                      0x24     /*  Vendor ID LSB Register  */
#define REG_GBL_CLKSEL                     0x25     /*  Clock Select Register */
#define REG_GBL_WDT                        0x26     /*  Watchdog Timer Control Register  */
#define REG_GBL_PSEL                       0x27     /*  Port Select Register  */


/*  UART REGISTERs  */
#define REG_UART_EN                        0x30     /*  UART Enable Register  */
#define REG_UART_BADDR_H                   0x60     /*  Base Address High Register  */
#define REG_UART_BADDR_L                   0x61     /*  Base Address Low Register  */
#define REG_UART_IRQ_CH_SEL                0x70     /*  IRQ Channel Select Register  */
#define REG_UART_ENH_MULTIFUN              0xF0     /*  Enhanced Multifunction Register  */
#define REG_UART_IRC                       0xF1     /*  IR Control Register  */
#define REG_UART_S_ADDR_MODE               0xF4     /*  9-bit Mode Slave Address Register  */
#define REG_UART_S_ADDR_MODE_MASK          0xF5     /*  9-bit Mode Slave Address Mask Regiser  */
#define REG_UART_FIFO_MOD_SEL              0xF6     /*  FIFO Mode Select Register  */


/*  WDT REGISTERs  */
#define REG_WDT_EN                         0x30     /*  Watchdog Enable Register  */
#define REG_WDT_BADDR_H                    0x60     /*  Base Address High Regiser  */
#define REG_WDT_BADDR_L                    0x61     /*  Base Address Low Register  */
#define REG_WDT_IRQ_CH_SEL                 0x70     /*  IRQ Channel Select Register  */
#define REG_WDT_T_STATUS_CTRL              0xF0     /*  Timer Status And Control Register  */
#define REG_WDT_TMR_COUNT                  0xF1     /*  Timer Count Numner Register  */



/*  MASKs  */
#define MASK_SRST                          0x01     /*  Software Reset  */
#define MASK_LDN                           0xFF     /*  Logic Device Number Register  */
#define MASK_DEV_ID                        0xFF     /*  Device ID Registers  */
#define MASK_VID                           0xFF     /*  Vendor ID Registers  */
#define MASK_CLKSEL                        0x01     /*  Clock Select Register */
#define MASK_WDT_ASSERT                    0x01     /*  Watchdog Timer - Assert a low pulse from WDTOUT# pin  */
#define MASK_WDT_RST_TIME                  0x02     /*  Watchdog Timer - Reset timer  */
#define MASK_PSEL_SEL_CONF_KEY             0x03     /*  Port Select Register - Select configuration entry key  */
#define MASK_PSEL_SEL_CONF_PORT            0x10     /*  Port Select Register - Select configuration port  */
#define MASK_UART_EN                       0x01     /*  UART Enable Register - Enable/Disable  */
#define MASK_UART_BADDR                    0xFF     /*  Base Address Register */
#define MASK_UART_IRQ_SEL                  0x0F     /*  IRQ Channel Select Register - Select IRQ channel  */
#define MASK_UART_IRQ_EN                   0x10     /*  IRQ Channel Select Register - Enable/Disable  */
#define MASK_UART_IRQ_SHARING_MOD          0x60     /*  IRQ Channel Select Register - Sharing Mode  */
#define MASK_UART_ENH_CLK_FREQ             0x03     /*  Enhanced Multifunction Register - Internal Clock frequency  */
#define MASK_UART_ENH_IR_TX_DELAY          0x04     /*  Enhanced Multifunction Register - IR mode TX Delay  */
#define MASK_UART_ENH_IR_RX_DELAY          0x08     /*  Enhanced Multifunction Register - IR mode RX Delay  */
#define MASK_UART_ENH_EN_AUTO_485          0x10     /*  Enhanced Multifunction Register - Enable/Disable Auto RS-485 Half-Duplex Control mode  */
#define MASK_UART_ENH_INVERT_POL_485       0x20     /*  Enhanced Multifunction Register - Invert the RTS#/RS485 signal polarity for RS485 Half-Duplex mode  */
#define MASK_UART_ENH_AUTO_ADDR_DETECT     0x40     /*  Enhanced Multifunction Register - Auto Address Detection  */
#define MASK_UART_ENH_ENABLE_9BIT_MOD      0x80     /*  Enhanced Multifunction Register - Enable/Disable the 9-bit mode  */
#define MASK_UART_IRC_IRRXA_INVERT         0x01     /*  IR Control Register - IR mode IRRXA# invert  */
#define MASK_UART_IRC_IRTXA_INVERT         0x02     /*  IR Control Register - IR mode IRTXA# invert  */
#define MASK_UART_IRC_HALF_DUPLEX          0x04     /*  IR Control Register - IR mode Half-Duplex */
#define MASK_UART_IRC_EN                   0x18     /*  IR Control Register - IR mode Enable  */
#define MASK_UART_FIFO_MOD_SEL_SIZE        0x02     /*  FIFO Mode Select Register - FIFO size for TX/RX  */
#define MASK_UART_FIFO_MOD_SEL_RX_TRG_L    0x20     /*  FIFO Mode Select Register - RX trigger level  */
#define MASK_UART_FIFO_MOD_SEL_TX_TRG_L    0x80     /*  FIFO Mode Select Register - TX trigger level  */
#define MASK_WDT_EN                        0x01     /*  Timer Count Numner Register - WDT Enable/Disable  */
#define MASK_WDT_BADDR                     0xFF     /*  Timer Count Numner Register - Base Address High/Low Regisers  */
#define MASK_WDT_IRQ_CH_SEL_WDT            0x0F     /*  IRQ Channel Select Register - Select the IRQ channel for watchdog timer  */
#define MASK_WDT_IRQ_CH_SEL_EN             0x10     /*  IRQ Channel Select Register - Enable/Disable the watchdog time IRQ  */
#define MASK_WDT_T_STATUS_TIME_OUT_EVN     0x01     /*  Timer Status And Control Register - Time Out Event  */
#define MASK_WDT_T_STATUS_WDT_INTERVAL     0x06     /*  Timer Status And Control Register - WDT Interval  */
#define MASK_WDT_TMR_COUNT                 0xFF     /*  Timer Count Numner Register  */




#define DEVICE_SEL_UARTA                   0x00
#define DEVICE_SEL_UARTB                   0x01
#define DEVICE_SEL_WDT                     0x08

#define DFL_DEV_ID_M                       0x03
#define DFL_DEV_ID_L                       0x82

#define DFL_VEN_ID_M                       0x13
#define DFL_VEN_ID_L                       0xA8

#define CLK_SEL_24MHZ                      0x00
#define CLK_SEL_48MHZ                      0x01

#define PORT_ENTRY_KEY_77                  0x00
#define PORT_ENTRY_KEY_A0                  0x01
#define PORT_ENTRY_KEY_87                  0x02
#define PORT_ENTRY_KEY_67                  0x03

#define SEL_CONF_PORT_0                    0x00     /*  The configuration port is 0x2E/0x2F  */
#define SEL_CONF_PORT_1                    0x01     /*  The configuration port is 0x4E/0x4F  */

#define UART_DISABLE                       0x00
#define UART_ENABLE                        0x01

#define IRQ_SHARING_DISPLAY                0x00
#define IRQ_SHARING_ENABLE                 0x01

#define FIFO_TXRX_SIZE_16                  0x00
#define FIFO_TXRX_SIZE_32                  0x01
#define FIFO_TXRX_SIZE_64                  0x02
#define FIFO_TXRX_SIZE_128                 0x03

#define RX_TRIGGER_LEVEL_X1                0x00
#define RX_TRIGGER_LEVEL_X2                0x01
#define RX_TRIGGER_LEVEL_X4                0x02
#define RX_TRIGGER_LEVEL_X8                0x03

#define TX_HOLDING_NO_DELAY                0x00
#define TX_HOLDING_DELAY_1TX               0x01

#define WDT_DISABLE                        0x00
#define WDT_ENABLE                         0x01

#define WDT_IRQ_DISABLE                    0x00
#define WDT_IRQ_ENABLE                     0x01

#define WDT_INTERVAL_10MSEC                0x00
#define WDT_INTERVAL_1SEC                  0x01
#define WDT_INTERVAL_1MIN                  0x02


#define INDEX_PORT                         0x2E
#define DATA_PORT                          0x2F

#define ENTER_EXT_MODE_CODE                0x67
#define EXIT_EXT_MODE_CODE                 0xAA


#define XR28V382_UART_CLOCK                1843200  /* frequency of 24MHz with prescaler */ 
#define SUPERIO_UARTA_BASE                 0x3F8
#define SUPERIO_UARTB_BASE                 0x2F8



/*  UART INTERNAL REGISTERs  */
#define UART_REG_RHR_THR                   0x00
#define UART_REG_IER                       0x01
#define UART_REG_ISR                       0x02
#define UART_REG_FCR                       0x02
#define UART_REG_LCR                       0x03
#define UART_REG_MCR                       0x04
#define UART_REG_LSR                       0x05
#define UART_REG_MSR                       0x06
#define UART_REG_SPR                       0x07


#define UART_IER_RHR_INT_EN                0x01
#define UART_IER_THR_INT_EN                0x02
#define UART_IER_RLS_INT_EN                0x04
#define UART_IER_MODEM_INT_EN              0x08

#define UART_ISR_INT_STATUS                0x01
#define UART_ISR_INT_SOURCE_STATUS         0x06
#define UART_ISR_INT_FIFO_EN_STATUS        0XC0

#define UART_FCR_FIFO_EN                   0x01
#define UART_FCR_RX_FIFO_RESET             0x02
#define UART_FCR_TX_FIFO_RESET             0x03
#define UART_FCR_RX_FIFO_TRIGGER_SEL       0xC0

#define UART_LCR_WORD_LENGTH               0x03
#define UART_LCR_STOP_BIT LENGTH           0x04
#define UART_LCR_PARITY_EN                 0x08
#define UART_LCR_PARITY_TYPE               0x10
#define UART_LCR_PARITY_SELECT             0x20
#define UART_LCR_TX_BREAK_EN               0x04
#define UART_LCR_BD_DIV_EN                 0x08

#define UART_MCR_DTR_OUTPUT                0x01
#define UART_MCR_RTS_OUTPUT                0x02
#define UART_MCR_EN_INT_SERIEQ             0x04
#define UART_MCR_INT_LOOPBACK_EN           0x08

#define UART_LSR_RX_DATA_READY             0x01
#define UART_LSR_RX_OVERRUN_FLAG           0x02
#define UART_LSR_RX_DATA_PARITY_ERR        0x04
#define UART_LSR_RX_DATA_FRAMMING_ERR      0x08
#define UART_LSR_RX_BREAK                  0x10
#define UART_LSR_TX_HOLD_REG_EMPTY         0x20
#define UART_LSR_THR_TSR_EMPTY             0x40
#define UART_LSR_RX_FIFO_DATA_ERR          0x80

#define UART_MSR_DELTA_CTS_INPUT           0x01
#define UART_MSR_DELTA_DSR_INPUT           0x02
#define UART_MSR_DELTA_IR_INPUT            0x04
#define UART_MSR_DELTA_CD_INPUT            0x08
#define UART_MSR_CTS_INPUT                 0x10
#define UART_MSR_DSR_INPUT                 0x20
#define UART_MSR_RI_INPUT                  0x40
#define UART_MSR_CD_INPUT_STATUS           0x80


#define SLOT_UART_A                        3
#define SLOT_UART_B                        4
 



struct xr28v382_device {
	int                       num_devices;
	struct platform_device    **pdevices;
	enum W83627DHG_P_DEVICE   *devices_type;
	struct lpc_device         *lpc_dev;
};


struct xr28v382_device *xr28v382;



////////////////////////////////////////////////////////////////
//                       BASIC FUNCTIONS                      //
////////////////////////////////////////////////////////////////

static inline int superio_outb (uint16_t addr, uint16_t value) {
	lpc_writew (INDEX_PORT, addr);
	lpc_writew (DATA_PORT, value);
	return 0;
}


static inline int superio_inb (uint16_t addr, uint16_t *data) {
	lpc_writew (INDEX_PORT, addr);
	lpc_readw  (DATA_PORT, data);
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


static inline void superio_sw_reset (void) {
	uint16_t data;
	superio_inb (REG_GBL_SREST, &data);
	superio_outb (REG_GBL_SREST, data | MASK_SRST);
}


static inline void superio_enter_ext_mode (void) {
	lpc_writew (INDEX_PORT, ENTER_EXT_MODE_CODE);
	lpc_writew (INDEX_PORT, ENTER_EXT_MODE_CODE);
}


static inline void superio_exit_ext_mode (void) {
	lpc_writew (INDEX_PORT, EXIT_EXT_MODE_CODE);
}


static uint16_t getID (uint8_t id_h, uint8_t id_l) {
	uint16_t tmp = 0x0000;
	uint16_t rev = 0x0000;

	superio_inb (id_h, &tmp);
	superio_inb (id_l, &rev);
	
	rev |= (tmp << 8);

	return rev;
}


static inline uint16_t getDeviceID (void) {
	return getID (REG_GBL_DEV_ID_M, REG_GBL_DEV_ID_L);
}


static inline uint16_t getVendorID (void) {
	return getID (REG_GBL_VID_M, REG_GBL_VID_L);
}


static inline int chipIDvalidate (uint16_t dev_id, uint16_t ven_id) {
	return dev_id == ((DFL_DEV_ID_M << 8) | DFL_DEV_ID_L) &&
		ven_id == ((DFL_VEN_ID_M << 8) | DFL_VEN_ID_L) ? 1 : 0;
}


static int logicalDeviceValidate (enum XR28V382_DEVICE  dev) {
	int isValid = 0;
	switch (dev) {
		case UARTA:
		case UARTB:
		case WDT:
			isValid = 1;
			break;
		default:
			isValid = 0;
	}
	return isValid;
}


static inline int superio_select (enum XR28V382_DEVICE id) {
	int ret = -1;
	if (logicalDeviceValidate (id)) {
		superio_outb (REG_GBL_LDN, (uint16_t)id);
		ret = (int)id;
	} else {
		ret = -1;
	}
	return ret;
}


//
//////////////////////////////////////////////////////////////////
////                        HWMON FUNCTIONS                     //
//////////////////////////////////////////////////////////////////
//
//static void hwmon_init (void) {
//	uint16_t value;
//	// set bit 0|1 CR2C to 11 to support pin78-85 to UARTB
//	superio_inb (SIO_REG_EN_VRM10, &value);
//	value |= 0x03;
//	superio_outb (SIO_REG_EN_VRM10, value);
//	// SELECT THE DEVICE HARDWARE MONITOR
//	superio_outb (SIO_REG_LDSEL, W83627EHF_LD_HWM);
//	// SET ADDRESS OF THE HARDWARE MONITOR
//	superio_outb (SIO_REG_ADDR, 0x02);
//	superio_outb (SIO_REG_ADDR + 1, 0x90);
//	// ACTIVATE THE HARDWARE MONITOR
//	superio_outb (SIO_REG_ENABLE, 0x01);
//}
//
//
//
//
//////////////////////////////////////////////////////////////////
////                        UART FUNCTIONS                      //
//////////////////////////////////////////////////////////////////
//
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
	//	value &= 0xF7;
	//	superio_writew (pdata[0].mapbase, 0x4, value);
	}
}


static struct plat_serial8250_port xr28v382_uart_a_platform_data[] = {
	{
		.mapbase               = (SUPERIO_UARTA_BASE << 1),
		.flags                 = UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,// | UPF_IOREMAP,
		.iotype                = UPIO_LPC,
		.regshift              = 0,
		.uartclk               = XR28V382_UART_CLOCK,
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


static struct plat_serial8250_port xr28v382_uart_b_platform_data[] = {
	{
		.mapbase               = (SUPERIO_UARTB_BASE << 1),
		.flags                 = UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,// | UPF_IOREMAP,
		.iotype                = UPIO_LPC,
		.regshift              = 0,
		.uartclk               = XR28V382_UART_CLOCK,
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


static struct platform_device xr28v382_uart_a_device = {
	.name = "serial8250",
	.id   = 0,
	.dev = {
		.platform_data = &xr28v382_uart_a_platform_data,
	},
};


static struct platform_device xr28v382_uart_b_device = {
	.name = "serial8250",
	.id   = 1,
	.dev = {
		.platform_data = &xr28v382_uart_b_platform_data,
	},
};




#if XR_DEBUG
static void uarta_stamp (void) {
	uint16_t data;

	superio_enter_ext_mode ();

	superio_select (DEVICE_SEL_UARTA);
	
	superio_inb (REG_UART_EN, &data);
	printk (KERN_INFO "XR28V382  REG_UART_EN   %#X\n", data);

	superio_inb (REG_UART_BADDR_H, &data);
	printk (KERN_INFO "XR28V382  REG_UART_BADDR_H   %#X\n", data);

	superio_inb (REG_UART_BADDR_L, &data);
	printk (KERN_INFO "XR28V382  REG_UART_BADDR_L   %#X\n", data);
	
	superio_inb (REG_UART_IRQ_CH_SEL, &data);
	printk (KERN_INFO "XR28V382  REG_UART_IRQ_CH_SEL   %#X\n", data);
	
	superio_inb (REG_UART_ENH_MULTIFUN, &data);
	printk (KERN_INFO "XR28V382  REG_UART_ENH_MULTIFUN   %#X\n", data);
	
	superio_inb (REG_UART_IRC, &data);
	printk (KERN_INFO "XR28V382  REG_UART_IRC   %#X\n", data);
	
	superio_inb (REG_UART_S_ADDR_MODE, &data);
	printk (KERN_INFO "XR28V382  REG_UART_S_ADDR_MODE   %#X\n", data);
	
	superio_inb (REG_UART_S_ADDR_MODE_MASK, &data);
	printk (KERN_INFO "XR28V382  REG_UART_S_ADDR_MODE_MASK   %#X\n", data);
	
	superio_inb (REG_UART_FIFO_MOD_SEL, &data);
	printk (KERN_INFO "XR28V382  REG_UART_FIFO_MOD_SEL   %#X\n", data);

	superio_exit_ext_mode ();
}


static void uartb_stamp (void) {
	uint16_t data;

	superio_enter_ext_mode ();

	superio_select (DEVICE_SEL_UARTB);
	
	superio_inb (REG_UART_EN, &data);
	printk (KERN_INFO "XR28V382  REG_UART_EN   %#X\n", data);

	superio_inb (REG_UART_BADDR_H, &data);
	printk (KERN_INFO "XR28V382  REG_UART_BADDR_H   %#X\n", data);

	superio_inb (REG_UART_BADDR_L, &data);
	printk (KERN_INFO "XR28V382  REG_UART_BADDR_L   %#X\n", data);
	
	superio_inb (REG_UART_IRQ_CH_SEL, &data);
	printk (KERN_INFO "XR28V382  REG_UART_IRQ_CH_SEL   %#X\n", data);
	
	superio_inb (REG_UART_ENH_MULTIFUN, &data);
	printk (KERN_INFO "XR28V382  REG_UART_ENH_MULTIFUN   %#X\n", data);
	
	superio_inb (REG_UART_IRC, &data);
	printk (KERN_INFO "XR28V382  REG_UART_IRC   %#X\n", data);
	
	superio_inb (REG_UART_S_ADDR_MODE, &data);
	printk (KERN_INFO "XR28V382  REG_UART_S_ADDR_MODE   %#X\n", data);
	
	superio_inb (REG_UART_S_ADDR_MODE_MASK, &data);
	printk (KERN_INFO "XR28V382  REG_UART_S_ADDR_MODE_MASK   %#X\n", data);
	
	superio_inb (REG_UART_FIFO_MOD_SEL, &data);
	printk (KERN_INFO "XR28V382  REG_UART_FIFO_MOD_SEL   %#X\n", data);

	superio_exit_ext_mode ();

}



static ssize_t uarta_state_show (struct device *dev,
		struct device_attribute *attr, char *buf) {
	int i;
	ssize_t status;
	unsigned char msg[500];
	unsigned char tmp[50];

	uint16_t data;
	uint16_t reg= (SUPERIO_UARTA_BASE << 1);

	for (i = 0 ; i < 7 ; i++) {
//		superio_outb (reg, &data);
		data = superio_readw (reg, i);
		sprintf (tmp, "%#02X : %#02X\n", ((reg & 0xFFFF) >> 1) + i, data);
		strcat (msg, tmp);
	}

//	superio_inb (0x03f8, 0xC);

	status = sprintf (buf, "%s\n", msg);
	return status;
}

static ssize_t uarta_reg_store (struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count) {

	int rc;
	unsigned long data;

	rc = strict_strtoul (buf, 0, &data);

	if (rc)
		return rc;

	superio_writew ((SUPERIO_UARTA_BASE << 1), 
			data & 0x0000F, 
			(uint16_t)((data >> 4) & 0xFFFF));


	return rc;	
}




static DEVICE_ATTR (uarta_state, 0666, uarta_state_show, uarta_reg_store);


static ssize_t uartb_state_show (struct device *dev,
		struct device_attribute *attr, char *buf) {
	int i;
	ssize_t status;
	unsigned char msg[500];
	unsigned char tmp[50];
	uint16_t data;
	uint16_t reg= (SUPERIO_UARTB_BASE << 1);
	for (i = 0 ; i < 7 ; i++) {
		data = superio_readw (reg, i);
		sprintf (tmp, "%#02X : %#02X\n", ((reg & 0xFFFF) >> 1) + i, data);
		strcat (msg, tmp);
	}
	status = sprintf (buf, "%s\n", msg);
	return status;
}

static ssize_t uartb_reg_store (struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count) {

	int rc;
	unsigned long data;

	rc = strict_strtoul (buf, 0, &data);

	if (rc)
		return rc;
	superio_writew ((unsigned long)(SUPERIO_UARTB_BASE << 1), 
			data & 0x0000F, 
			(int)((data >> 4) & 0xFFFF));

	rc = count;
	return rc;	
}


static DEVICE_ATTR (uartb_state, 0666, uartb_state_show, uartb_reg_store);


#endif


static void uart_a_init (void) {
	uint16_t data;
        /*  Select device UARTA  */	
	superio_select (DEVICE_SEL_UARTA);	
	
	superio_outb (REG_UART_FIFO_MOD_SEL, RX_TRIGGER_LEVEL_X1 | 
			FIFO_TXRX_SIZE_16 |
			TX_HOLDING_NO_DELAY);

	
	/*  Enable UARTA  */
	superio_outb (REG_UART_EN, UART_ENABLE); 

	uint16_t reg= (SUPERIO_UARTA_BASE << 1); 
	
	data = superio_readw (reg, 3);
	data |= 0x80;
	superio_writew (reg, 3, data);

	superio_writew (reg, 0, 0x01);
	superio_writew (reg, 1, 0x00);

	data &= 0x7F;
	superio_writew (reg, 3, data);

	superio_outb (0x3f8, 0xaa);

}


static void uart_b_init (void) {
	uint16_t data;
        /*  Select device UARTB  */	
	superio_select (DEVICE_SEL_UARTB);	
	
	superio_outb (REG_UART_FIFO_MOD_SEL, RX_TRIGGER_LEVEL_X1 | 
			FIFO_TXRX_SIZE_16 |
			TX_HOLDING_NO_DELAY);

	/*  Enable UARTB  */
	superio_outb (REG_UART_EN, UART_ENABLE); 

	uint16_t reg= (SUPERIO_UARTB_BASE << 1); 
	
	data = superio_readw (reg, 3);
	data |= 0x80;
	superio_writew (reg, 3, data);

	superio_writew (reg, 0, 0x01);
	superio_writew (reg, 1, 0x00);

	data &= 0x7F;
	superio_writew (reg, 3, data);


	superio_outb (0x2f8, 0xaa);
}


static int __devinit seco_xr28v382_probe (struct platform_device *pdev) {
	int ret, err = 0;
	uint16_t dev_id, ven_id;
	struct seco_xr28v382_platform_data *pdata;
	struct lpc_device *lpc_dev;
	int i;
	enum XR28V382_DEVICE *dev_list;

	XR_INFO ("probe starting ...");	

	pdata = pdev->dev.platform_data;
	if (pdata == NULL) {
		XR_ERR ("no platform_data!");
		return -EINVAL;
	}

	xr28v382 = kzalloc (sizeof(struct xr28v382_device), GFP_KERNEL);
	if (xr28v382 == NULL) {
		XR_ERR ("no device data!");
		err = -EINVAL;
		goto err_device;
	}

	/*  device software restetting  */
	superio_outb (REG_GBL_SREST, 0x01);
	udelay (10);

	/*  validate chip through device and vendor ID  */
	superio_enter_ext_mode ();
	dev_id = getDeviceID ();
	ven_id = getVendorID ();

	
	ret = chipIDvalidate (dev_id, ven_id);
	if (ret == 1) {
		dev_info (&pdev->dev, "EXAR chip ID: %#04X, vendor ID: %#04X\n", dev_id, ven_id);
	} else {
		dev_err (&pdev->dev, "SuperIO invalid chip ID: %#04X - %#04X\n", dev_id, ven_id);
		err = -EINVAL;
		goto probe_failed;
	}

	/*  add driver for the superio's devices  */
	if (pdata->num_devices < 1) {
		XR_INFO ("No devices associated to Winbond");
		goto dev_err_out;
	} else {
		xr28v382->num_devices = pdata->num_devices;
		xr28v382->pdevices = kzalloc(sizeof(struct platform_device *) * pdata->num_devices, GFP_KERNEL);
		xr28v382->lpc_dev = kzalloc(sizeof(struct lpc_device) * pdata->num_devices, GFP_KERNEL);
		if  (xr28v382->pdevices == NULL || xr28v382->lpc_dev == NULL) {
			XR_ERR ("Error in devices allocation");
		} else {
			for (i = 0, dev_list = pdata->devices ; i < pdata->num_devices ; i++, dev_list++) {
				switch (*dev_list) {
					case UARTA:
						XR_INFO ("UART_A device selected");
						xr28v382_uart_a_platform_data[0].mapbase += lpc_getMemBase();
						xr28v382_uart_a_platform_data[0].irq = lpc_getIRQ(0);
						xr28v382_uart_a_platform_data[0].irqflags = lpc_getIRQ_flags(0) | IRQF_LPCSTYLE;
						uart_a_init ();

						xr28v382->pdevices[i] = &xr28v382_uart_a_device;
						xr28v382->lpc_dev[i].lpc_slot = 3;
						xr28v382->lpc_dev[i].dev = &xr28v382->pdevices[i]->dev;

						xr28v382->lpc_dev[i].handler = &xr28v382_uart_a_platform_data[0].irq_management;
						xr28v382->lpc_dev[i].after_handler = irq_after_handler;
						xr28v382->lpc_dev[i].irq_dev_id = xr28v382_uart_a_platform_data[0].irq_data;
						xr28v382->lpc_dev[i].maskable = LPC_IRQ_MASKABLE;

						break;
					case UARTB:
						XR_INFO ("UART_B device selected");
						xr28v382_uart_b_platform_data[0].mapbase += lpc_getMemBase();
						xr28v382_uart_b_platform_data[0].irq = lpc_getIRQ(0);
						xr28v382_uart_b_platform_data[0].irqflags = lpc_getIRQ_flags(0) | IRQF_LPCSTYLE;
						uart_b_init ();

						xr28v382->pdevices[i] = &xr28v382_uart_b_device;
						xr28v382->lpc_dev[i].lpc_slot = 4;
						xr28v382->lpc_dev[i].dev = &xr28v382->pdevices[i]->dev;

						xr28v382->lpc_dev[i].handler = &xr28v382_uart_b_platform_data[0].irq_management;
						xr28v382->lpc_dev[i].after_handler = irq_after_handler;
						xr28v382->lpc_dev[i].irq_dev_id = xr28v382_uart_b_platform_data[0].irq_data;
						xr28v382->lpc_dev[i].maskable = LPC_IRQ_MASKABLE;
						break;

					default:
						XR_ERR ("Device not supported by SuperIO");
				}
			}
		}
	}
	superio_exit_ext_mode ();

	/*  device registration  */
	platform_add_devices (xr28v382->pdevices, xr28v382->num_devices);

	lpc_dev = xr28v382->lpc_dev;
	for (i = 0 ; i < xr28v382->num_devices ; i++, lpc_dev++) {
		if (lpc_dev->lpc_slot >= 0)
			lpc_add_device (lpc_dev);
	}
	
#if XR_DEBUG 

	uarta_stamp ();
	uartb_stamp ();
	ret = device_create_file (&pdev->dev, &dev_attr_uarta_state);
	ret = device_create_file (&pdev->dev, &dev_attr_uartb_state);
#endif

	return 0;
probe_failed:
err_device:
dev_err_out:
	return err;

}


static int __devexit seco_xr28v382_remove (struct platform_device *pdev) {

	platform_device_unregister (pdev); 
	
	kfree (pdev);

	return 0;
}


static struct platform_driver seco_xr28v382_driver = {
	.driver = {
		.name = "secoXR28V382",
		.owner = THIS_MODULE,
	},
	.probe = seco_xr28v382_probe,
	.remove = __devexit_p(seco_xr28v382_remove),
};

MODULE_ALIAS("platform:seco_xr28v382");


static int __init seco_xr28v382_init (void) {
	return platform_driver_register (&seco_xr28v382_driver);
}
subsys_initcall (seco_xr28v382_init);


static void __exit seco_xr28v382_exit (void) {
	platform_driver_unregister(&seco_xr28v382_driver);
}
module_exit (seco_xr28v382_exit);


MODULE_AUTHOR("DC SECO");
MODULE_DESCRIPTION("SECO SuperIO XR28V382");
MODULE_LICENSE("GPL");
