#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/compiler.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/ioctl.h>
#include <linux/major.h>
#include <linux/cdev.h>
#include <linux/seco_cpld.h>
#include <linux/seco_lpc.h>
#include "lpc_table.h"
#include <linux/lpc_io.h>

#define NR_LPC_SLOTS 16


#define LPC_INFO(fmt, arg...) printk(KERN_INFO "LPC_BUS: " fmt "\n" , ## arg)
#define LPC_ERR(fmt, arg...)  printk(KERN_ERR "%s: " fmt "\n" , __func__ , ## arg)
#define LPC_DBG(fmt, arg...)  pr_debug("%s: " fmt "\n" , __func__ , ## arg)

struct lpc_table_element *lpc_table;


struct lpc_data *lpc_d;


static int lpc_reg_read (unsigned int reg, int *data) {
	int ret = 0;
	ret = cpld_reg_read (reg, data);
	if (ret < 0) {
		pr_err ("%s: read operation failed!\n", __func__);
	}
	pr_debug ("%s: read operation on register %2x complite, read value %2x, returned code %d\n", __func__, reg, *data, ret);
	return ret;
}


static int lpc_reg_write (unsigned int reg, int value) {
	int ret = 0;
	ret = cpld_reg_write (reg, value);
	if (ret < 0) {
		pr_err("%s: write operation failed!\n", __func__);
	}
	pr_debug ("%s: write complite with code %d\n", __func__, ret);
	return ret;
}


static int inline lpc_read_status (void) {
	int state;
	int ret = cpld_reg_read (LPC_REG_LPC_BUSY, &state);
	if (ret < 0)
		return LPC_BUSY;
	else
		return ((state & 0x8000) >> 15);
}


int lpc_readw (unsigned long mem_addr, uint16_t *data) {
	unsigned long orig_jiffies;
	int try;
	int res = 0;
	int status = -EIO;

	orig_jiffies = jiffies;
	for (try = 0 ; try <= NR_RETRIES ; try++) {
		res = lpc_read_status();
		if (res != LPC_BUSY) {
			cpld_read (mem_addr, data);
			status = res;
			break;
		}
		if (time_after (jiffies, orig_jiffies + LPC_TIME_OUT)) {
			LPC_ERR ("read operation TIMEOUT!");
			break;
		}
	}

	return status;
}


int lpc_writew (unsigned long mem_addr, uint16_t value) {
	unsigned long orig_jiffies;
	int try;
	int res = 0;
	int status = -EIO;
	
	orig_jiffies = jiffies;
	for (try = 0 ; try <= NR_RETRIES ; try++) {
		res = lpc_read_status();
		if (res != LPC_BUSY) {
			cpld_write (mem_addr, value);
			status = res;
			break;
		}
		if (time_after (jiffies, orig_jiffies + LPC_TIME_OUT)) {
			LPC_ERR ("read operation TIMEOUT!");
			break;
		}
	}

	return status;
}


int lpc_getIRQ (int irq) {
	int _irq;
	if (irq < 0 || irq > NR_IRQ) {
		_irq = 0;
	} else {
		_irq = lpc_d->irqs[irq] ? lpc_d->irqs[irq] : 0;
	}
	return _irq;
}


unsigned long lpc_getMemBase (void) {
	return lpc_d->mem_base;
}

static ssize_t table_state_show (struct device *dev,
		struct device_attribute *attr, char *buf) {
	int i;
	ssize_t status;
	unsigned char msg[500];
	unsigned char tmp[50];
	struct lpc_table_element *t = lpc_table;
	
	strcat (msg, "  #     dev     status     irq_hits\n");
	for (i = 0 ; i < lpc_table_size (lpc_table) ; i++, t++) {
		if (t->state == SLOT_BUSY) {
			sprintf (tmp, "%02d\t%s\t%s\t%lu\n", t->lpc_slot,
				dev_name(t->dev), 
				t->state == SLOT_FREE ? "FREE" : "BUSY", 
				t->irq_hits);
		} else {
			sprintf (tmp, "%02d\t------------\t%s\t%lu\n",  t->lpc_slot,
					t->state == SLOT_FREE ? "FREE" : "BUSY", 
					t->irq_hits);
		}
		strcat (msg, tmp); 
	}

	status = sprintf (buf, "%s\n", msg);
	return status;
}


static DEVICE_ATTR (table_state, 0444, table_state_show, NULL);


unsigned long lpc_getIRQ_flags (int irq) {
	unsigned long flags;
	if (irq < 0 || irq > NR_IRQ) {
		flags = 0;
	} else {
		flags = lpc_d->irqs[irq] ? lpc_d->irq_flags[irq] : 0;
	}
	return flags;
}


void irq_slot_set (int slot, int en) {
	uint16_t irq_mask_reg;
	int val;
	lpc_reg_read (LPC_REG_IRQ_MASK, &val);
	irq_mask_reg = (uint16_t)val;
	irq_mask_reg = (en == 1 ? irq_mask_reg | (en << slot) : irq_mask_reg & ~(!en << slot));
	lpc_reg_write (LPC_REG_IRQ_MASK, (int)irq_mask_reg);
}


static irqreturn_t lpc_interrupt1_manager (int irq, void *dev_id)  {
	int index = 0;
	int ret = IRQ_NONE;
	int find_slot_active = 0;
	struct lpc_table_element **t;
	uint16_t irq_status_reg;
	uint16_t irq_mask_reg;
	uint16_t mask;
	int val;
	lpc_reg_read (LPC_REG_IRQ_BUFFER, &val);
	irq_status_reg = (uint16_t)val;
	lpc_reg_read (LPC_REG_IRQ_MASK, &val);
        irq_mask_reg = (uint16_t)val;

	for (index = 0, t = dev_id ; index < NR_LPC_SLOTS ; index++) {
		mask = 1u << index;
		// check if the idenx-th slot is active and unmaskered
		if (irq_status_reg & irq_mask_reg & mask) {
			// slot active and valid
			find_slot_active++;
			if (*((*t)[index].handler) == NULL) { 
				LPC_ERR ("irq slot %d not manageable, missing data", index);
				continue;
			}
			if ((*t)[index].irq_dev_id != NULL) {
				printk (KERN_ERR "%s: punto 4\n", __func__);
				ret = (*((*t)[index].handler)) (irq, *((*t)[index].irq_dev_id));
			} else {
				ret = (*((*t)[index].handler)) (irq, NULL);
			}
			(*t)[index].irq_hits++;
			// Then, the relative slot will not maskered permanently
			if (ret == IRQ_NONE) {
				if ((*t)[index].after_handler != NULL) {
					lpc_reg_write (LPC_REG_IRQ_MASK, 0x0);
					((*t)[index].after_handler) (irq, ((*t)[index].dev));
					udelay (10);
					lpc_reg_write (LPC_REG_IRQ_MASK, irq_mask_reg);

				}
			}
		}
	}
	return ret;
}


static irqreturn_t lpc_interrupt2_manager (int irq, void *dev_id)  {
	return 0;
}


static irqreturn_t lpc_interrupt3_manager (int irq, void *dev_id)  {
	return 0;
}


int lpc_add_device (struct lpc_device *lpc_dev) {
	int index = -1;
	struct lpc_table_element element;
	element.lpc_slot = lpc_dev->lpc_slot;
	element.dev = lpc_dev->dev;
	element.handler = lpc_dev->handler;
	element.after_handler = lpc_dev->after_handler;
	element.irq_dev_id = lpc_dev->irq_dev_id;
	element.maskable = lpc_dev->maskable;
	index = lpc_table_add_device (&lpc_table, element);
	return index;
}


static int lpc_open(struct inode *i, struct file *f) {
	    return 0;
}


static int lpc_close(struct inode *i, struct file *f) {
	    return 0;
}


static long lpc_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {   
	int err = 0;
	int retval = 0;
			struct lpc_reg reg;
			struct lpc_mem mem;
	switch (cmd) {
		case LPC_READ_REG: 
			if (copy_from_user (&reg, (const void __user *)arg, sizeof (reg))) {
				return -EFAULT;
			}
			err = lpc_reg_read (reg.reg_addr, &reg.data);
			if (err < 0)
				retval = err;
			if (copy_to_user ((void __user *)arg, &reg, sizeof (reg))) {
				retval = -EFAULT;
			}
			break; 
		case LPC_WRITE_REG:
			if (copy_from_user (&reg, (const void __user *)arg, sizeof (reg))) {
				return -EFAULT;
			}
			err = lpc_reg_write (reg.reg_addr, reg.data);
			if (err < 0)
				retval = err;
			if (copy_to_user ((void __user *)arg, &reg, sizeof (reg))) {
				retval = -EFAULT;
			}
			break;
		case LPC_READ_MEM:
			if (copy_from_user (&mem, (const void __user *)arg, sizeof (mem))) {
				return -EFAULT;
			}
			err = lpc_readw (mem.mem_addr, &mem.data);
			if (err < 0)
				retval = err;
			if (copy_to_user ((void __user *)arg, &mem, sizeof (mem))) {
				retval = -EFAULT;
			}
			break;
		case LPC_WRITE_MEM:
			if (copy_from_user (&mem, (const void __user *)arg, sizeof (mem))) {
				return -EFAULT;
			}
			err = lpc_writew (mem.mem_addr, mem.data);
			if (err < 0)
				retval = err;
			if (copy_to_user ((void __user *)arg, &mem, sizeof (mem))) {
				retval = -EFAULT;
			}
			break;
		default:
			break;
	}
	return retval;
}


const static struct file_operations lpc_fops = {
	.owner          = THIS_MODULE,
	.open           = lpc_open,
	.release        = lpc_close,
	.unlocked_ioctl	= lpc_ioctl,
};


static int lpc_init (struct platform_device *device, struct seco_lpc_platform_data *pdata) {
	int err = 0;
	int i;
	int ret = 0;

	lpc_d = devm_kzalloc(&device->dev, sizeof(struct lpc_data), GFP_KERNEL);
	if (lpc_d == NULL) {
		err = -ENOMEM;
		goto err_out;
	}

	lpc_d->mem_base = cpld_get_membase ();

	lpc_d->name = dev_name(&device->dev);

	if (pdata->irq_count < 1 || pdata->irq_count > NR_IRQ) {
		LPC_ERR ("Inavalid IRQ cout number");
		goto err_out;
	}

	ret = 0;
	for (i = 0 ; i < pdata->irq_count ; i++)
	       	ret += pdata->irqs[i];

	if (ret == 0) {
		LPC_ERR ("No IRQ assigned");
		err = -EINVAL;
		goto err_out;
	}
	
	for (i = 0 ; i < pdata->irq_count ; i++) {

		if (pdata->irqs[i] != 0) {
			// irq used
			lpc_d->irqs[i]      = pdata->irqs[i];
			lpc_d->irq_flags[i] = pdata->flags_irq[i];

			if (i == 0) {
				ret = request_irq (lpc_d->irqs[i], lpc_interrupt1_manager,
						lpc_d->irq_flags[i], "lpc_irq0", &lpc_table);
				if (ret) {
					LPC_ERR ("IRQ not acquired: error %d", ret);
					err = -EIO;
					goto err_out;
				}
			} else {
				LPC_ERR ("irq %d not allowed", i);
			}

			if (i == 1) {
				ret = request_irq (lpc_d->irqs[i], lpc_interrupt2_manager,
						lpc_d->irq_flags[i], "lpc_irq0", &lpc_table);
				if (ret) {
					LPC_ERR ("IRQ not acquired: error %d", ret);
					err = -EIO;
					goto err_out;
				}
			} else {
				LPC_ERR ("irq %d not allowed", i);
			}

			if (i == 2) {
				ret = request_irq (lpc_d->irqs[i], lpc_interrupt3_manager,
						lpc_d->irq_flags[i], "lpc_irq0", &lpc_table);
				if (ret) {
					LPC_ERR ("IRQ not acquired: error %d", ret);
					err = -EIO;
					goto err_out;
				}
			} else {
				LPC_ERR ("irq %d not allowed", i);
			}
			
			LPC_INFO ("LPC IRQ%d used", i);

		} else {
			lpc_d->irqs[i]      = 0;
			lpc_d->irq_flags[i] = 0;
		}

	}

	mutex_init (&lpc_d->bus_lock);

	// initialization of the register of LPC
	

	LPC_INFO ("LPC bus initialized!");

	return 0;

err_out:
	return err;
}


struct class *lpc_class;

static char *lpc_devnode(struct device *dev, mode_t *mode) {
	if (!mode)
		return NULL;
	if (dev->devt == MKDEV(LPC_SECO_MAJOR, 0) ||
			dev->devt == MKDEV(LPC_SECO_MAJOR, 2))
		*mode = 0x666;
	return NULL;
}


static int __init lpc_class_init(void)
{
	lpc_class = class_create(THIS_MODULE, "lpc");
	if (IS_ERR(lpc_class))
		return PTR_ERR(lpc_class);
	lpc_class->devnode = lpc_devnode;
	return 0;
}

postcore_initcall(lpc_class_init);

static struct cdev lpc_cdev;

static int __devinit seco_lpc_probe (struct platform_device *pdev) {
	struct seco_lpc_platform_data *pdata;
	int ret = 0;

	pdata = pdev->dev.platform_data;
	if (pdev == NULL) {
		dev_err (&pdev->dev, "No platform data\n");
		return -EINVAL;
	}

	ret = lpc_table_init (pdev, &lpc_table, NR_LPC_SLOTS);
	if (ret != NR_LPC_SLOTS) {
		LPC_ERR ("no LPC SLOT table!");
		goto probe_failed;
	}
	dev_info (&pdev->dev, "LPC IRQ table initializated with %d slots\n", lpc_table_size (lpc_table));

	ret = lpc_init (pdev, pdata);
	if (ret != 0) {
		dev_err (&pdev->dev, "LPC initialization failed. Probe stopped!\n");
		goto probe_failed;
	}

	// all irq slot will be maskered
	lpc_reg_write (LPC_REG_IRQ_MASK, 0x0000);

	// caricamento dei driver che utilizzano il bus LPC
	if (pdata->devices == NULL) {
		dev_info (&pdev->dev, "No drivers present in LPC bus\n");
	} else {
		platform_add_devices (pdata->devices, pdata->num_devices);
	}

	cdev_init (&lpc_cdev, &lpc_fops);
	if (cdev_add(&lpc_cdev, MKDEV(LPC_SECO_MAJOR, 0), 1) ||
		register_chrdev_region(MKDEV(LPC_SECO_MAJOR, 0), 1, "/dev/lpc") < 0)
			panic("Couldn't register /dev/lpc driver\n");
	device_create(lpc_class, NULL, MKDEV(LPC_SECO_MAJOR, 0), NULL, "lpc");



	ret = device_create_file (&pdev->dev, &dev_attr_table_state);

	return 0;
probe_failed:
	kfree (lpc_d);
	if (lpc_table != NULL)
		kfree(lpc_table);
	return -1;
}


static int __devexit seco_lpc_remove (struct platform_device *pdev) {
	int i;
	struct seco_lpc_platform_data *pdata;

	pdata = pdev->dev.platform_data;

	for (i = 0 ; i < pdata->irq_count ; i++) {

		if (pdata->irqs[i] != 0) {
			if (i == 0) {
				free_irq (lpc_d->irqs[i], pdev);
			}
		}
	}

	device_remove_file (&pdev->dev, &dev_attr_table_state);

	//cpld_remove (pdev);

	platform_device_unregister (pdev);

	kfree (lpc_d);
	kfree (lpc_table);
	kfree (pdev);

	return 0;
}


static struct platform_driver seco_lpc_driver = {
	.driver    = {
		.name    = "secoLPC",
		.owner   = THIS_MODULE,
	},
	.probe     = seco_lpc_probe,
	.remove    = __devexit_p(seco_lpc_remove),
};


MODULE_ALIAS("platform:secoLPC");


static int __init seco_lpc_init (void) {
	return platform_driver_register (&seco_lpc_driver);
}
subsys_initcall (seco_lpc_init);


static void __exit seco_lpc_exit (void) {
	platform_driver_unregister (&seco_lpc_driver);
}

MODULE_AUTHOR("DC SECO");
MODULE_DESCRIPTION("SECO LPC BUS");
MODULE_LICENSE("GLP");
