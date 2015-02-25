
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/major.h>
#include <linux/cdev.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/proc_fs.h> 
#include <linux/seq_file.h> 
#include <linux/mutex.h>
#include <linux/uaccess.h> 
#include <linux/input.h>
#include <linux/list.h>
#include <linux/seco_cpld.h>
#include <linux/seco_gpio_expander.h>
#include <linux/secoGPIO_io.h>


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

#define REG_FW_REV          0x0
#define REG_GPIO_BUFFER     0x1
#define REG_GPIO_DIR        0x2
#define REG_GPIO_INT_MASK   0x3
#define REG_GPIO_INT_EN     0x4
#define REG_GPIO_INT_CONF   0x5
#define REG_GPIO_WR_MASK    0x6
#define REG_GPIO_INT        0x7

#define INT_IGNORED   0x0
#define INT_GLOBAL    0x1

#define WRITABLE     0x0
#define NOWRITABLE   0x1

#define DIR_OUT  0x0
#define DIR_IN   0x1

#define INT_MASK    1
#define INT_UNMASK  0

#define INT_EN      1
#define INT_DIS     0

#define INT_CONF_LEVEL  0
#define INT_CONF_EDGE   1

#define POLL_PERIOD     2  // (ms)

#define SGPIO_INFO(fmt, arg...) printk(KERN_INFO "SecoGPIO: " fmt "\n" , ## arg)
#define SGPIO_ERR(fmt, arg...)  printk(KERN_ERR "%s: " fmt "\n" , __func__ , ## arg)
#define SGPIO_DBG(fmt, arg...)  pr_debug("%s: " fmt "\n" , __func__ , ## arg)


struct event_dev {
	struct input_dev        *input;
	char                    phys[64];
	char                    name[64];
};


struct seco_gpio_chip {
	struct gpio_chip        gpio_chip;

	unsigned int            mask;
    	uint8_t                 dir_input;
    	uint8_t                 dir_output;

	struct mutex            lock;
	struct mutex            lock_proc;

    	uint8_t                 reg_out;

	uint8_t           	irq_mask;

	struct delayed_work	work;
	unsigned long           poll_period;

	int                     irq;
	unsigned int            irq_counter[NR_GPIO];

	struct event_dev        **events;
};

static struct seco_gpio_chip *global_chip;


struct secoGPIO_proc_data {
	struct seco_gpio_chip          *chip;
	unsigned int                   gpio_id;
};


static int seco_gpio_exp_writeb (unsigned int reg, int value) {
	int ret = 0;
	ret = cpld_reg_write (reg, value);
	if (ret < 0) {
		pr_err("%s: write operation failed!\n", __func__);
	}
	pr_debug ("%s: write complite with code %d\n", __func__, ret);
	return ret;	
}


static int seco_gpio_exp_readb (unsigned int reg, int *data) {
	int ret = 0;
	ret = cpld_reg_read (reg, data);
	if (reg < 0) {
		pr_err ("%s: read operation failed!\n", __func__);
	}
	pr_debug ("%s: read operation on register %2x complite, read value %2x, returned code %d\n", __func__, reg, *data, ret);
	return ret;
}


static int seco_gpio_exp_get_value (struct gpio_chip *gc, unsigned offset) {
	struct seco_gpio_chip *chip;
	int reg_value;
	int ret = 0;
	uint8_t mask = 1u << offset;

	chip = container_of (gc, struct seco_gpio_chip, gpio_chip);
	ret = seco_gpio_exp_readb (REG_GPIO_BUFFER, &reg_value);
	if (ret < 0) {
		pr_err ("%s, operation failed!\n", __func__);
		return 0;
	}

	return (int)(((uint8_t)reg_value & mask) >> offset);
}


static void seco_gpio_exp_set_value (struct gpio_chip *gc, unsigned offset, int value) {
	struct seco_gpio_chip *chip;
	uint8_t reg_out;
	uint8_t mask = 1u << offset;
	int ret = 0;

	chip = container_of (gc, struct seco_gpio_chip, gpio_chip);

	mutex_lock (&chip->lock);

	reg_out = (value) ? chip->reg_out | mask : chip->reg_out & ~mask;

	ret = seco_gpio_exp_writeb (REG_GPIO_WR_MASK, 0xFF & ~(1 << offset));
	if (ret < 0) {
		pr_err ("%s, operation failed!\n", __func__);
		goto out;
	}

	ret = seco_gpio_exp_writeb (REG_GPIO_BUFFER, reg_out);
	if (ret < 0) {
		pr_err ("%s, operation failed!\n", __func__);
		goto out;
	}

	chip->reg_out = reg_out;

out:
	mutex_unlock (&chip->lock);
}


static uint8_t seco_gpio_exp_direction_reg (struct seco_gpio_chip *chip) {
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


static int seco_gpio_exp_int_enable_get (struct seco_gpio_chip *chip, int gpio_nr) {
	int ret;
	int reg = 0x0;
	uint8_t mask = (1u << gpio_nr);

	if ((gpio_nr < 0) || (gpio_nr >= NR_GPIO))
		return -EINVAL;

	ret = seco_gpio_exp_readb (REG_GPIO_INT_EN, &reg);
	if (ret < 0) {
		SGPIO_INFO("%s, operation failed!\n", __func__);
		goto out;
	}
	ret = (int)(((uint8_t)reg & mask) >> gpio_nr);
out:
	mutex_unlock (&chip->lock);
	return ret;
}


static int seco_gpio_exp_int_enable_set (struct seco_gpio_chip *chip, int gpio_nr, int en) {
	int ret;
	int reg = 0x0;
	uint8_t mask = (1u << gpio_nr);

	if ((gpio_nr < 0) || (gpio_nr >= NR_GPIO))
		return -EINVAL;
	if ((en != INT_EN) && (en != INT_DIS))
		return -EINVAL;

	ret = seco_gpio_exp_readb (REG_GPIO_INT_EN, &reg);
	if (ret < 0) {
		SGPIO_INFO("%s, operation failed!\n", __func__);
		goto out;
	}
	reg = en ? (uint8_t)reg | mask : (uint8_t)reg & ~mask;
	ret = seco_gpio_exp_writeb (REG_GPIO_INT_EN, reg);
	if (ret < 0) {
		SGPIO_INFO("%s, operation failed!\n", __func__);
		goto out;
	}
out:
	mutex_unlock (&chip->lock);
	return ret;
}


static int seco_gpio_exp_int_mask_get (struct seco_gpio_chip *chip, int gpio_nr) {
	int ret;
	int reg = 0x0;
	uint8_t mask = (1u << gpio_nr);

	if ((gpio_nr < 0) || (gpio_nr >= NR_GPIO))
		return -EINVAL;

	ret = seco_gpio_exp_readb (REG_GPIO_INT_MASK, &reg);
	if (ret < 0) {
		SGPIO_INFO("%s, operation failed!\n", __func__);
		goto out;
	}
	ret = (int)(((uint8_t)reg & mask) >> gpio_nr);
out:
	mutex_unlock (&chip->lock);
	return ret;
}


static int seco_gpio_exp_int_mask_set (struct seco_gpio_chip *chip, int gpio_nr, int maskset) {
	int ret;
	int reg = 0x0;
	uint8_t mask = (1u << gpio_nr);
	if ((gpio_nr < 0) || (gpio_nr >= NR_GPIO))
		return -EINVAL;
	if ((maskset != INT_MASK) && (maskset != INT_UNMASK))
		return -EINVAL;

	ret = seco_gpio_exp_readb (REG_GPIO_INT_MASK, &reg);
	if (ret < 0) {
		SGPIO_INFO("%s, operation failed!\n", __func__);
		goto out;
	}
	reg = maskset ? (uint8_t) reg | mask : (uint8_t)reg & ~mask;
	ret = seco_gpio_exp_writeb (REG_GPIO_INT_MASK, reg);
	if (ret < 0) {
		SGPIO_INFO("%s, operation failed!\n", __func__);
		goto out;
	}
out:
	mutex_unlock (&chip->lock);
	return ret;
}


static int seco_gpio_exp_int_config_get (struct seco_gpio_chip *chip, int gpio_nr) {
	int ret;
	int reg = 0x0;
	uint8_t mask = (1u << gpio_nr);

	if ((gpio_nr < 0) || (gpio_nr >= NR_GPIO))
		return -EINVAL;

	ret = seco_gpio_exp_readb (REG_GPIO_INT_CONF, &reg);
	if (ret < 0) {
		SGPIO_INFO("%s, operation failed!\n", __func__);
		goto out;
	}
	ret = (int)(((uint8_t)reg & mask) >> gpio_nr);
out:
	mutex_unlock (&chip->lock);
	return ret;
}


static int seco_gpio_exp_int_config_set (struct seco_gpio_chip *chip, int gpio_nr, int conf) {
	int ret;
	int reg = 0x0;
	uint8_t mask = (1u << gpio_nr);
	if ((gpio_nr < 0) || (gpio_nr >= NR_GPIO))
		return -EINVAL;
	if ((conf != INT_CONF_LEVEL) && (conf != INT_CONF_EDGE))
		return -EINVAL;

	ret = seco_gpio_exp_readb (REG_GPIO_INT_CONF, &reg);
	if (ret < 0) {
		SGPIO_INFO("%s, operation failed!\n", __func__);
		goto out;
	}
	reg = conf ? (uint8_t)reg | mask : (uint8_t)reg & ~mask;
	ret = seco_gpio_exp_writeb (REG_GPIO_INT_CONF, reg);
	if (ret < 0) {
		SGPIO_INFO("%s, operation failed!\n", __func__);
		goto out;
	}
out:
	mutex_unlock (&chip->lock);
	return ret;
}


/************************************************************************************************
 * /sys/devices/platform/secoGPIO								*
 *   /directions										*
 *        Read Only. Shows the current directions of all the GPIOs				*
 *   /values											*
 *        Read Only. Shows the current values of all the GPIOs.					*
 *        (for the GPIOs setted as output, the shown values will are the setted output values;	*
 *        for the GPIOs setted as input, the shown values will are the readed input values).	*
 ************************************************************************************************/


static ssize_t gpio_directions_show (struct device *dev, 
		struct device_attribute *attr, char *buf) {
	// three character, plus blank space, every gpio
	ssize_t status;
	char *leg =  " g0  g1  g2  g3  g4  g5  g6  g7 ";
	char *stat = "                                "; 
	int reg, i;
	int mask = 0x1;
        int ret = seco_gpio_exp_readb (REG_GPIO_DIR, &reg);
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
	ssize_t status;
	char *leg =  " g0  g1  g2  g3  g4  g5  g6  g7 ";
	char *stat = "                                "; 
	int reg, i;
	int mask = 0x1;
        int ret = seco_gpio_exp_readb (REG_GPIO_BUFFER, &reg);
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


/*----------------------------------------------------------------------
 *                                   PROC                              *
 *----------------------------------------------------------------------*/

#define INPUT_BUF_SIZE   256

static int secoGPIO_proc_dumpregs_show (struct seq_file *seq, void *offset) {
	int                             reg1, reg2, reg3, reg4, reg5, reg6, reg7;
	struct seco_gpio_chip           *chip;

	struct secoGPIO_proc_data *proc = (struct secoGPIO_proc_data *)seq->private;
	if (!proc)
		return -EINVAL;
	
	chip = proc->chip;
	
	mutex_lock(&chip->lock_proc);
	seco_gpio_exp_readb (REG_GPIO_BUFFER, &reg1);
	seco_gpio_exp_readb (REG_GPIO_DIR, &reg2);
	seco_gpio_exp_readb (REG_GPIO_INT_MASK, &reg3);
	seco_gpio_exp_readb (REG_GPIO_INT_EN, &reg4);
	seco_gpio_exp_readb (REG_GPIO_INT_CONF, &reg5);
	seco_gpio_exp_readb (REG_GPIO_WR_MASK, &reg6);
	seco_gpio_exp_readb (REG_GPIO_INT, &reg7);
	mutex_unlock(&chip->lock_proc);
	seq_printf(seq, "%d: %#x\n%d: %#x\n%d: %#x\n%d: %#x\n\
%d: %#x\n%d: %#x\n%d: %#x\n", 
		REG_GPIO_BUFFER, reg1, 
		REG_GPIO_DIR, reg2,
		REG_GPIO_INT_MASK, reg3,
		REG_GPIO_INT_EN, reg4,
		REG_GPIO_INT_CONF, reg5,
		REG_GPIO_WR_MASK, reg6,
		REG_GPIO_INT, reg7);

        return 0;
}

static int secoGPIO_proc_dumpregs_open_fs (struct inode *inode, struct file *file) {
        return single_open(file, secoGPIO_proc_dumpregs_show, PDE(inode)->data);
}

static const struct file_operations secoGPIO_proc_dumpregs_fops = {
        .owner = THIS_MODULE,
        .open = secoGPIO_proc_dumpregs_open_fs,
        .read = seq_read,
	.write = NULL,
        .llseek = seq_lseek,
        .release = single_release,
};


/* -------------------------- PROC ENABLE FILEs OPT (r/w) -------------------------- */

static int secoGPIO_proc_enable_show (struct seq_file *seq, void *offset) {
	int                             enable;
	struct seco_gpio_chip           *chip;

	struct secoGPIO_proc_data *proc = (struct secoGPIO_proc_data *)seq->private;
	if (!proc)
		return -EINVAL;
	
	chip = proc->chip;
	
	mutex_lock(&chip->lock_proc);

	enable = seco_gpio_exp_int_enable_get (chip, proc->gpio_id);
	if (enable < 0)
		 enable = 0;
	
	mutex_unlock(&chip->lock_proc);

	seq_printf(seq, "%s\n", enable == INT_EN ? "enable" : "disable");  
        return 0;
}


static int secoGPIO_proc_enable_write (struct file *file, const char __user *buf,
			size_t count, loff_t *pos) {

	char 				input[INPUT_BUF_SIZE];
	int                             err_conv, ret = 0;
	long                            enable;
	struct seco_gpio_chip           *chip;

	struct secoGPIO_proc_data *proc = (struct secoGPIO_proc_data *)PDE(file->f_path.dentry->d_inode)->data; 
	if (!proc)
		return -EINVAL;
	
	chip = proc->chip;

        if (!capable(CAP_SYS_ADMIN))                                                                     
                return -EACCES;                                                                          
        
        if (count >= INPUT_BUF_SIZE)
                count = INPUT_BUF_SIZE - 1;                                                              
        
        memset(input, 0, INPUT_BUF_SIZE);
        if (copy_from_user (input, buf, count))                                                          
                return -EFAULT;                                                                          
        
        err_conv = strict_strtol (input, 0, &enable);

        if (err_conv == 0) {
                if (enable == INT_EN || enable == INT_DIS) {
			mutex_lock (&chip->lock_proc);
			ret = seco_gpio_exp_int_enable_set (chip, proc->gpio_id, enable);
			mutex_unlock(&chip->lock_proc);
                } else  
                        return -EINVAL;                                                                  
        }
        return (ret < 0) ? -EINVAL :count;          
}


static int secoGPIO_proc_enable_open_fs (struct inode *inode, struct file *file) {
        return single_open(file, secoGPIO_proc_enable_show, PDE(inode)->data);
}


static const struct file_operations secoGPIO_proc_enable_fops = {
        .owner = THIS_MODULE,
        .open = secoGPIO_proc_enable_open_fs,
        .read = seq_read,
	.write = secoGPIO_proc_enable_write,
        .llseek = seq_lseek,
        .release = single_release,
};

/* -------------------------- PROC MASK FILEs OPT (r/w) -------------------------- */

static int secoGPIO_proc_mask_show (struct seq_file *seq, void *offset) {
	int                             mask;
	struct seco_gpio_chip           *chip;

	struct secoGPIO_proc_data *proc = (struct secoGPIO_proc_data *)seq->private;
	if (!proc)
		return -EINVAL;
	
	chip = proc->chip;
	
	mutex_lock(&chip->lock_proc);

	mask = seco_gpio_exp_int_mask_get (chip, proc->gpio_id);
	if (mask < 0)
		 mask = 0;
	
	mutex_unlock(&chip->lock_proc);

	seq_printf(seq, "%s\n", mask == INT_MASK ? "masked" : "unmasked");  
        return 0;
}


static int secoGPIO_proc_mask_write (struct file *file, const char __user *buf,
			size_t count, loff_t *pos) {

	char 				input[INPUT_BUF_SIZE];
	int                             err_conv, ret = 0;
	long                            mask;
	struct seco_gpio_chip           *chip;

	struct secoGPIO_proc_data *proc = (struct secoGPIO_proc_data *)PDE(file->f_path.dentry->d_inode)->data; 
	if (!proc)
		return -EINVAL;
	
	chip = proc->chip;

        if (!capable(CAP_SYS_ADMIN))                                                                     
                return -EACCES;                                                                          
        
        if (count >= INPUT_BUF_SIZE)
                count = INPUT_BUF_SIZE - 1;                                                              
        
        memset(input, 0, INPUT_BUF_SIZE);
        if (copy_from_user (input, buf, count))                                                          
                return -EFAULT;                                                                          
        
        err_conv = strict_strtol (input, 0, &mask);

        if (err_conv == 0) {
                if (mask == INT_MASK || mask == INT_UNMASK) {
			mutex_lock (&chip->lock_proc);
			ret = seco_gpio_exp_int_mask_set (chip, proc->gpio_id, mask);
			mutex_unlock(&chip->lock_proc);
                } else  
                        return -EINVAL;                                                                  
        }
        return (ret < 0) ? -EINVAL :count;          
}


static int secoGPIO_proc_mask_open_fs (struct inode *inode, struct file *file) {
        return single_open(file, secoGPIO_proc_mask_show, PDE(inode)->data);
}


static const struct file_operations secoGPIO_proc_mask_fops = {
        .owner = THIS_MODULE,
        .open = secoGPIO_proc_mask_open_fs,
        .read = seq_read,
	.write = secoGPIO_proc_mask_write,
        .llseek = seq_lseek,
        .release = single_release,
};

/* -------------------------- PROC CONFIG FILEs OPT (r/w) -------------------------- */

static int secoGPIO_proc_config_show (struct seq_file *seq, void *offset) {
	int                             config;
	struct seco_gpio_chip           *chip;

	struct secoGPIO_proc_data *proc = (struct secoGPIO_proc_data *)seq->private;
	if (!proc)
		return -EINVAL;
	
	chip = proc->chip;
	
	mutex_lock(&chip->lock_proc);

	config = seco_gpio_exp_int_config_get (chip, proc->gpio_id);
	if (config < 0)
		 config = 0;
	
	mutex_unlock(&chip->lock_proc);

	seq_printf(seq, "%s\n", config == INT_CONF_LEVEL ? "Level Triggering" : "Edge (Rising) Triggering");  
        return 0;
}


static int secoGPIO_proc_config_write (struct file *file, const char __user *buf,
			size_t count, loff_t *pos) {

	char 				input[INPUT_BUF_SIZE];
	int                             err_conv, ret = 0;
	long                            config;
struct seco_gpio_chip           *chip;

	struct secoGPIO_proc_data *proc = (struct secoGPIO_proc_data *)PDE(file->f_path.dentry->d_inode)->data; 
	if (!proc)
		return -EINVAL;
	
	chip = proc->chip;

        if (!capable(CAP_SYS_ADMIN))                                                                     
                return -EACCES;                                                                          
        
        if (count >= INPUT_BUF_SIZE)
                count = INPUT_BUF_SIZE - 1;                                                              
        
        memset(input, 0, INPUT_BUF_SIZE);
        if (copy_from_user (input, buf, count))                                                          
                return -EFAULT;                                                                          
        
        err_conv = strict_strtol (input, 0, &config);

        if (err_conv == 0) {
                if (config == INT_CONF_LEVEL || config == INT_CONF_EDGE) {
			mutex_lock (&chip->lock_proc);
			ret = seco_gpio_exp_int_config_set (chip, proc->gpio_id, config);
			mutex_unlock(&chip->lock_proc);
                } else  
                        return -EINVAL;                                                                  
        }
        return (ret < 0) ? -EINVAL :count;          
}


static int secoGPIO_proc_config_open_fs (struct inode *inode, struct file *file) {
        return single_open(file, secoGPIO_proc_config_show, PDE(inode)->data);
}


static const struct file_operations secoGPIO_proc_config_fops = {
        .owner = THIS_MODULE,
        .open = secoGPIO_proc_config_open_fs,
        .read = seq_read,
	.write = secoGPIO_proc_config_write,
        .llseek = seq_lseek,
        .release = single_release,
};


static struct proc_dir_entry *secoGPIO_root_dir;


#define SGPIO_PROC_ROOT             "secoGPIO"
#define SGPIO_PROC_EVN              "gpio_evn"  
#define SGPIO_ENTRY_ENABLE          "enable"
#define SGPIO_ENTRY_MASK            "mask"
#define SGPIO_ENTRY_CONFIG          "config"
#define SGPIO_ENTRY_CTRL_UNMASK     "auto_unmask"


struct proc_entry {
	char                   label[30];
	struct proc_dir_entry  *entry;
	struct list_head       list; 
};

static struct proc_entry *entry_list;

#define add_to_list(item, head, name, entry_dir)       item = kzalloc (sizeof (struct proc_entry), GFP_KERNEL); \
							strcpy (item->label, name); \
							item->entry = entry_dir; \
							list_add (&item->list, &head->list)


static struct proc_dir_entry **event_list; 

static int secoGPIO_add_proc_fs (struct seco_gpio_chip *chip) { 

	struct secoGPIO_proc_data      *proc;
	int j;
	struct proc_dir_entry *entry = NULL;
	struct proc_entry *tmp;
	struct list_head *pos, *q;
	char ctmp[30];  

	entry_list = kzalloc (sizeof (struct proc_entry), GFP_KERNEL);
	INIT_LIST_HEAD (&entry_list->list);

	event_list = kzalloc (sizeof (struct proc_dir_entry) * NR_GPIO, GFP_KERNEL);

	/* create /proc/secoGPIO */
	secoGPIO_root_dir = proc_mkdir (SGPIO_PROC_ROOT, NULL);
	if (!secoGPIO_root_dir)
		return -ENODEV;
	else 
		add_to_list (tmp, entry_list, SGPIO_PROC_ROOT, NULL);


	proc = kzalloc (sizeof (struct secoGPIO_proc_data), GFP_KERNEL);

	proc->chip = chip;
	proc->gpio_id = j;

	entry = proc_create_data("dumpregs",  S_IRUGO | S_IWUGO,
			secoGPIO_root_dir, &secoGPIO_proc_dumpregs_fops,
			proc);
	if (!entry) {
		goto remove_dir;
	} else {
		add_to_list (tmp, entry_list, "dumpregs", secoGPIO_root_dir);
	}


	for (j = 0 ; j < NR_GPIO ; j++) {

	sprintf(ctmp, "%s%d", SGPIO_PROC_EVN, j);
	event_list[j] = proc_mkdir (ctmp, secoGPIO_root_dir);
	if (!event_list[j]) {
		goto remove_dir;
	} else {
		add_to_list (tmp, entry_list, ctmp, secoGPIO_root_dir);
	}

	
	proc = kzalloc (sizeof (struct secoGPIO_proc_data), GFP_KERNEL);

	proc->chip = chip;
	proc->gpio_id = j;

	// file

	entry = proc_create_data(SGPIO_ENTRY_ENABLE,  S_IRUGO | S_IWUGO,
			event_list[j], &secoGPIO_proc_enable_fops,
			proc);
	if (!entry) {
		goto remove_dir;
	} else {
		add_to_list (tmp, entry_list, SGPIO_ENTRY_ENABLE, event_list[j]);
	}

	entry = proc_create_data(SGPIO_ENTRY_MASK,  S_IRUGO | S_IWUGO,
			event_list[j], &secoGPIO_proc_mask_fops,
			proc);
	if (!entry) {
		goto remove_dir;
	} else {
		add_to_list (tmp, entry_list, SGPIO_ENTRY_MASK, event_list[j]);
	}

	entry = proc_create_data(SGPIO_ENTRY_CONFIG,  S_IRUGO | S_IWUGO,
			event_list[j], &secoGPIO_proc_config_fops,
			proc);
	if (!entry) {
		goto remove_dir;
	} else {
		add_to_list (tmp, entry_list, SGPIO_ENTRY_CONFIG, event_list[j]);
	}

	}

	return 0;
remove_dir:
        list_for_each_safe (pos, q, &entry_list->list) {
                tmp = list_entry (pos, struct proc_entry, list);
                remove_proc_entry (tmp->label, tmp->entry);
                list_del (pos);
                kfree (tmp);
        }
        return -EINVAL;
}

/************************************************************************************/

static int seco_gpio_exp_direction_input (struct gpio_chip *gc, unsigned offset) {
	struct seco_gpio_chip *chip;
	uint8_t reg_value;

	chip = container_of (gc, struct seco_gpio_chip, gpio_chip);

	mutex_lock (&chip->lock);

	chip->dir_output &= ~(1 << offset);
	chip->dir_input |= 1 << offset;

	reg_value = seco_gpio_exp_direction_reg (chip);

	seco_gpio_exp_writeb (REG_GPIO_DIR, (int)reg_value);

	mutex_unlock (&chip->lock);

	return 0;
}


static int seco_gpio_exp_direction_output (struct gpio_chip *gc, unsigned offset, int value) {
	struct seco_gpio_chip *chip;
	uint8_t reg_value;

	chip = container_of (gc, struct seco_gpio_chip, gpio_chip);

	mutex_lock (&chip->lock);

	chip->dir_output |= 1 << offset;
	chip->dir_input &= ~(1 << offset);

	reg_value = seco_gpio_exp_direction_reg (chip);

	seco_gpio_exp_writeb (REG_GPIO_DIR, (int)reg_value);

	mutex_unlock (&chip->lock);

	// set the output
	seco_gpio_exp_set_value (gc, offset, value);
	
	return 0;

}


/* --------------------------------------------------------------------------
                                     IOCTL
   -------------------------------------------------------------------------- */

static int secoGPIO_open(struct inode *i, struct file *file) {
        file->private_data = global_chip;
        return 0;
}


int secoGPIO_release(struct inode *inode, struct file *file) {
        file->private_data = NULL;
        return 0;
}


static int secoGPIO_close(struct inode *i, struct file *f) {
            return 0;
}


static long secoGPIO_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {
	struct seco_gpio_chip *chip = global_chip; //file->private_data;
	int retval = 0;
	struct gpio_evn_seco evn;
	switch (cmd) {
		case GPIO_EVN_SECO_GET:
			if (copy_from_user (&evn, (const void __user *)arg, sizeof (evn))) {
				return -EFAULT;
			}
	
			if ((int)evn.id < 0 || (int)evn.id > NR_GPIO)
				return -EINVAL;
			retval = seco_gpio_exp_int_enable_get (chip, (int)evn.id);
			if (retval >= 0)
				evn.state = (GPIO_STATE)retval;
			else
				return retval;
			retval = seco_gpio_exp_int_mask_get (chip, (int)evn.id);
			if (retval >= 0)
				evn.mask = (GPIO_MASK)retval;
			else
				return retval;
			retval = seco_gpio_exp_int_config_get (chip, (int)evn.id);
			if (retval >= 0)
				evn.conf = (GPIO_TRIGGERING)retval;
			else
				return retval;

			if (copy_to_user ((void __user *)arg, &evn, sizeof (evn))) {
				return -EFAULT;
			}
			break;	
		case GPIO_EVN_SECO_SET:
			if (copy_from_user (&evn, (const void __user *)arg, sizeof (evn))) {
				return -EFAULT;
			}

			if ((int)evn.id < 0 || (int)evn.id > NR_GPIO)
				return -EINVAL;

			if (seco_gpio_exp_int_config_set (chip, (int)evn.id, (int)evn.conf) < 0 ||
				seco_gpio_exp_int_mask_set (chip, (int)evn.id, (int)evn.mask) < 0||
				seco_gpio_exp_int_enable_set (chip, (int)evn.id, (int)evn.state) < 0)
			{
				return -EIO;
			}
			
			retval = 0;
			if (copy_to_user ((void __user *)arg, &evn, sizeof (evn))) {
				return -EFAULT;
			}
			break;
		default:
			break;
	}
	return retval;
}


const static struct file_operations secoGPIO_fops = {
        .owner          = THIS_MODULE,
        .open           = secoGPIO_open,
        .release        = secoGPIO_close,
        .unlocked_ioctl = secoGPIO_ioctl,
};


/************************************/

static void chip_gpio_work (struct work_struct *work) {
	int i, is_active;
	static int mask_reg = 0xFF;
	static int  int_flag_reg = 0;
	struct seco_gpio_chip *chip = container_of(to_delayed_work(work), struct seco_gpio_chip, work);

	if (int_flag_reg == 0) {
		seco_gpio_exp_readb (REG_GPIO_INT, &int_flag_reg);
		// store the actual configuration of the mask register
		if (mask_reg == 0xFF) {
			seco_gpio_exp_readb (REG_GPIO_INT_MASK, &mask_reg);
			// enable all mask flags
			seco_gpio_exp_writeb (REG_GPIO_INT_MASK, 0xFF);
		}
	}

	if (int_flag_reg) {
		// scan of the flag reg, to find the gpio that has scatured the interrupt	
		for (i = 0 ; i < NR_GPIO ; i++) {
			is_active = !!(int_flag_reg & (1 << i));
			if (is_active) {	// notify the relative event to the file system layer
				input_report_key (chip->events[i]->input, KEY_OK, 1);
				input_sync (chip->events[i]->input);
				input_report_key (chip->events[i]->input, KEY_OK, 0);
				input_sync (chip->events[i]->input);
				// we have to clean the served event's flag
				int_flag_reg &= ~(1u << i);
				seco_gpio_exp_writeb (REG_GPIO_INT, (1u << i));

			}	
		}
		schedule_delayed_work (&chip->work, msecs_to_jiffies(chip->poll_period));
	} else {
		// there is no event to handle
		// this is a safe write, to ensure that the IRQ signal becomes high
		seco_gpio_exp_writeb (REG_GPIO_INT, 0xFF);
		enable_irq (chip->irq);
		seco_gpio_exp_writeb (REG_GPIO_INT_MASK, mask_reg);
		mask_reg = 0xFF;
	}
}


static irqreturn_t irq_int_manager (int irq, void *dev_id)  {
	struct  seco_gpio_chip *chip = ((struct seco_gpio_chip *)dev_id); 	

	disable_irq_nosync (chip->irq);
	schedule_delayed_work (&chip->work, msecs_to_jiffies(chip->poll_period));

	return IRQ_HANDLED;
}


static int seco_gpio_exp_setup_gpio (struct platform_device *device, struct seco_gpio_exp_platform_data *pdata, struct seco_gpio_chip *chip) {

	struct gpio_chip       *gc = &chip->gpio_chip;
	int                     port;
	uint8_t                 dir_init = pdata->direction_init;
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

	gc->direction_input = seco_gpio_exp_direction_input;
	gc->direction_output = seco_gpio_exp_direction_output;

	gc->set = seco_gpio_exp_set_value;
	gc->get = seco_gpio_exp_get_value;

	gc->can_sleep = 1;

	gc->base = pdata->gpio_base;
	gc->ngpio = NR_GPIO;
	gc->label = pdata->label;
	gc->dev = &device->dev;
	gc->owner = THIS_MODULE;

	return 1;
}



struct class *secoGPIO_class;

static char *secoGPIO_devnode(struct device *dev, mode_t *mode) {
        if (!mode)
                return NULL;
        if (dev->devt == MKDEV(GPIO_SECO_MAJOR, 0) ||
                        dev->devt == MKDEV(GPIO_SECO_MAJOR, 2))
                *mode = 0x666;
        return NULL;
}

static int __init secoGPIO_class_init(void) {
	secoGPIO_class = class_create(THIS_MODULE, "secoGPIO");
	if (IS_ERR(secoGPIO_class))
		return PTR_ERR(secoGPIO_class);
	secoGPIO_class->devnode = secoGPIO_devnode;
	return 0;
}

postcore_initcall(secoGPIO_class_init);
static struct cdev secoGPIO_cdev;

static int __devinit seco_gpio_exp_probe (struct platform_device *pdev) {
	struct seco_gpio_chip 			*chip;
	struct seco_gpio_exp_platform_data 	*pdata;
	int 					err, i, regs, ret = 0;
	struct event_dev        		**events;
	printk (KERN_INFO "SECO GPIO Expander start probe!\n");

	pdata = pdev->dev.platform_data;
	if (pdev == NULL) {
		dev_err (&pdev->dev, "no platform data!\n");
		return -EINVAL;
	}

	chip = kzalloc (sizeof(struct seco_gpio_chip), GFP_KERNEL);
	if (chip == NULL) {
		return -EINVAL;
	}

	mutex_init (&chip->lock);
	mutex_init (&chip->lock_proc);

	ret = seco_gpio_exp_setup_gpio (pdev, pdata, chip);

	/* irq resquest */

	chip->poll_period  = POLL_PERIOD;
	INIT_DELAYED_WORK (&chip->work, chip_gpio_work);

	// fist at all, disable all irq
	seco_gpio_exp_writeb (REG_GPIO_INT_EN, 0x00);
	seco_gpio_exp_writeb (REG_GPIO_INT, 0xFF);

	if (pdata->irq > 0) {
		chip->irq = pdata->irq;
		ret = request_irq (pdata->irq, irq_int_manager,
				pdata->flag_irq, pdata->label, chip);
	
		if (ret) {
			err = -EIO;
			SGPIO_ERR("IRQ (%d) request failed!!!", pdata->irq);
			goto err_free_irq;
		}
	}	


	/* This ensures the consistency of data */
	/*
	seco_gpio_exp_readb (REG_GPIO_BUFFER, &chip->reg_out);
	for (port = 0 ; port < NR_GPIO ; port++, dir_init >>= 1) {
		mask = 1u << port;
		if (chip->dir_output & mask) {  // direction as output

		}
		if (chip->dir_input & mask) {   // direction as input
		}
	}
		
*/
	global_chip = chip;

      	cdev_init (&secoGPIO_cdev, &secoGPIO_fops);
        if (cdev_add(&secoGPIO_cdev, MKDEV(GPIO_SECO_MAJOR, 0), 1) ||
                register_chrdev_region(MKDEV(GPIO_SECO_MAJOR, 0), 1, "/dev/secoGPIO") < 0)
                        panic("Couldn't register /dev/secoGPIO driver\n");
        device_create(secoGPIO_class, NULL, MKDEV(GPIO_SECO_MAJOR, 0), NULL, "secoGPIO");

	ret = device_create_file (&pdev->dev, &dev_attr_directions);
	ret = device_create_file (&pdev->dev, &dev_attr_values);

	ret = gpiochip_add (&chip->gpio_chip);
	if (ret < 0)
		goto probe_failed;

	/* input interface */
	events =  kzalloc (sizeof (struct event_dev ) * NR_GPIO, GFP_KERNEL);
        if (!events) {
                err = -ENOMEM;
                goto fail;
        }

        for (i = 0 ; i < NR_GPIO; i++) {
		events[i] = kzalloc (sizeof (struct event_dev ), GFP_KERNEL);
		events[i]->input = input_allocate_device();
		if (!events[i]->input) {
                        err = -ENOMEM;
                        goto fail;
                }

		snprintf (events[i]->phys, sizeof (events[i]->phys),
					"%s/input%d", dev_name(&pdev->dev), i);

		snprintf (events[i]->name, sizeof (events[i]->name),
					"Seco GPIO Expander");

		events[i]->input->phys = events[i]->phys;
		events[i]->input->name = events[i]->name;
		events[i]->input->id.bustype = BUS_SPI;
                events[i]->input->dev.parent = &pdev->dev;

                events[i]->input->evbit[0] = BIT_MASK(EV_KEY);
		set_bit (KEY_OK, events[i]->input->keybit);

        }

	chip->events = events;

	secoGPIO_add_proc_fs (chip);

	for (i = 0 ; i < NR_GPIO; i++) {
		err = input_register_device (events[i]->input);
                if (err) {
                        err = -EINVAL;
                        regs = i;
                        goto fail;
                }
        }


	printk (KERN_INFO "SECO GPIO Expander probed!\n");

	return 0;

err_free_irq:
fail:
probe_failed:
	kfree (chip);
	return ret;
}


static int __devexit seco_gpio_exp_remove (struct platform_device *pdev) {

	struct seco_gpio_exp_platform_data *pdata = pdev->dev.platform_data;

	device_remove_file (&pdev->dev, &dev_attr_directions);
	device_remove_file (&pdev->dev, &dev_attr_values);

	//cpld_remove (pdev); 

	platform_device_unregister (pdev);

	return 0;
}



static struct platform_driver seco_gpio_exp_driver = {
	.driver = {
		.name = "secoGPIO",
		.owner = THIS_MODULE,
	},
	.probe = seco_gpio_exp_probe,
	.remove = __devexit_p(seco_gpio_exp_remove),
};

MODULE_ALIAS("platform:secoGPIO");


static int __init seco_gpio_exp_init (void) {
	 return platform_driver_register(&seco_gpio_exp_driver);
}
subsys_initcall(seco_gpio_exp_init); 


static void __exit seco_gpio_exp_exit (void) {
	platform_driver_unregister(&seco_gpio_exp_driver);
}
module_exit(seco_gpio_exp_exit);


MODULE_AUTHOR("DC SECO");
MODULE_DESCRIPTION("SECO CPLD GPIO x8 I/O expander");
MODULE_LICENSE("GPL");
