/*
 * mcp23s08.c - SPI gpio expander driver
 */

#include <linux/module.h> 
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/spi/mcp23s08.h>
#include <linux/slab.h>
#include <asm/byteorder.h>
#include <linux/proc_fs.h> 
#include <linux/platform_device.h>
#include <linux/seq_file.h> 
#include <linux/uaccess.h> 
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/list.h>
#include <linux/interrupt.h> 


/**
 * MCP types supported by driver
 */
#define MCP_TYPE_S08	0
#define MCP_TYPE_S17	1

/* Registers are all 8 bits wide.
 *
 * The mcp23s17 has twice as many bits, and can be configured to work
 * with either 16 bit registers or with two adjacent 8 bit banks.
 *
 * Also, there are I2C versions of both chips.
 */
#define MCP_IODIR	0x00		/* init/reset:  all ones */
#define MCP_IPOL	0x01
#define MCP_GPINTEN	0x02
#define MCP_DEFVAL	0x03
#define MCP_INTCON	0x04
#define MCP_IOCON	0x05
#	define IOCON_SEQOP	(1 << 5)
#	define IOCON_HAEN	(1 << 3)
#	define IOCON_ODR	(1 << 2)
#	define IOCON_INTPOL	(1 << 1)
#define MCP_GPPU	0x06
#define MCP_INTF	0x07
#define MCP_INTCAP	0x08
#define MCP_GPIO	0x09
#define MCP_OLAT	0x0a


#define MCP_INFO(fmt, arg...) printk(KERN_INFO "MCP23xx: " fmt "\n" , ## arg)
#define MCP_ERR(fmt, arg...)  printk(KERN_ERR "%s: " fmt "\n" , __func__ , ## arg)
#define MCP_DBG(fmt, arg...)  pr_debug("%s: " fmt "\n" , __func__ , ## arg)

#define MCP23_POLL_PERIOD     2 // (ms)


struct mcp23s08;

struct mcp23s08_ops {
	int	(*read)(struct mcp23s08 *mcp, unsigned reg);
	int	(*write)(struct mcp23s08 *mcp, unsigned reg, unsigned val);
	int	(*read_regs)(struct mcp23s08 *mcp, unsigned reg,
			     u16 *vals, unsigned n);
};

struct mcp23s08 {
	struct spi_device	*spi;
	u8			addr;

	u16			cache[11];
	/* lock protects the cached values */
	struct mutex		lock;

	struct gpio_chip	chip;

	struct work_struct	work;

	const struct mcp23s08_ops	*ops;

	unsigned int            inta;
	unsigned int            intb;
	
	unsigned int            irq_counter[16];
};

struct event_dev {
	struct input_dev        *input;
	char                    phys[64];
	char                    name[64];
};


/* A given spi_device can represent up to eight mcp23sxx chips
 * sharing the same chipselect but using different addresses
 * (e.g. chips #0 and #3 might be populated, but not #1 or $2).
 * Driver data holds all the per-chip data.
 */
struct mcp23s08_driver_data {
	struct spi_device       *spi;
	struct event_dev        **events;
 	unsigned int            inta[8];
	unsigned long           irq_flags_a[8];
	unsigned int            intb[8];
	unsigned long           irq_flags_b[8];
	unsigned int            irq_call;
	struct delayed_work     work;
	unsigned long           poll_period;
	unsigned		ngpio;
	struct mcp23s08		*mcp[8];
	struct mcp23s08		chip[];
};

typedef enum {BLOCK_A = 0, BLOCK_B} block_id;


struct mcp23_proc_data {
	struct mcp23s08_driver_data    *data;
	int                            mcp_id;
	block_id                       bid;
	int                            gid;    // gpio id - position into the block register
};


static int mcp23s08_read(struct mcp23s08 *mcp, unsigned reg)
{
	u8	tx[2], rx[1];
	int	status;

	tx[0] = mcp->addr | 0x01;
	tx[1] = reg;
	status = spi_write_then_read(mcp->spi, tx, sizeof tx, rx, sizeof rx);
	return (status < 0) ? status : rx[0];
}

static int mcp23s08_write(struct mcp23s08 *mcp, unsigned reg, unsigned val)
{
	u8	tx[3];

	tx[0] = mcp->addr;
	tx[1] = reg;
	tx[2] = val;
	return spi_write_then_read(mcp->spi, tx, sizeof tx, NULL, 0);
}

static int
mcp23s08_read_regs(struct mcp23s08 *mcp, unsigned reg, u16 *vals, unsigned n)
{
	u8	tx[2], *tmp;
	int	status;

	if ((n + reg) > sizeof mcp->cache)
		return -EINVAL;
	tx[0] = mcp->addr | 0x01;
	tx[1] = reg;

	tmp = (u8 *)vals;
	status = spi_write_then_read(mcp->spi, tx, sizeof tx, tmp, n);
	if (status >= 0) {
		while (n--)
			vals[n] = tmp[n]; /* expand to 16bit */
	}
	return status;
}

static int mcp23s17_read(struct mcp23s08 *mcp, unsigned reg)
{
	u8	tx[2], rx[2];
	int	status;

	tx[0] = mcp->addr | 0x01;
	tx[1] = reg << 1;
	status = spi_write_then_read(mcp->spi, tx, sizeof tx, rx, sizeof rx);
	return (status < 0) ? status : (rx[0] | (rx[1] << 8));
}

static int mcp23s17_write(struct mcp23s08 *mcp, unsigned reg, unsigned val)
{
	u8	tx[4];

	tx[0] = mcp->addr;
	tx[1] = reg << 1;
	tx[2] = val;
	tx[3] = val >> 8;
	return spi_write_then_read(mcp->spi, tx, sizeof tx, NULL, 0);
}

static int
mcp23s17_read_regs(struct mcp23s08 *mcp, unsigned reg, u16 *vals, unsigned n)
{
	u8	tx[2];
	int	status;

	if ((n + reg) > sizeof mcp->cache)
		return -EINVAL;
	tx[0] = mcp->addr | 0x01;
	tx[1] = reg << 1;

	status = spi_write_then_read(mcp->spi, tx, sizeof tx,
				     (u8 *)vals, n * 2);
	if (status >= 0) {
		while (n--)
			vals[n] = __le16_to_cpu((__le16)vals[n]);
	}

	return status;
}

static const struct mcp23s08_ops mcp23s08_ops = {
	.read		= mcp23s08_read,
	.write		= mcp23s08_write,
	.read_regs	= mcp23s08_read_regs,
};

static const struct mcp23s08_ops mcp23s17_ops = {
	.read		= mcp23s17_read,
	.write		= mcp23s17_write,
	.read_regs	= mcp23s17_read_regs,
};


/*----------------------------------------------------------------------*/

static int mcp23s08_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct mcp23s08	*mcp = container_of(chip, struct mcp23s08, chip);
	int status;

	mutex_lock(&mcp->lock);
	mcp->cache[MCP_IODIR] |= (1 << offset);
	status = mcp->ops->write(mcp, MCP_IODIR, mcp->cache[MCP_IODIR]);
	mutex_unlock(&mcp->lock);
	return status;
}

static int mcp23s08_get(struct gpio_chip *chip, unsigned offset)
{
	struct mcp23s08	*mcp = container_of(chip, struct mcp23s08, chip);
	int status;

	mutex_lock(&mcp->lock);

	/* REVISIT reading this clears any IRQ ... */
	status = mcp->ops->read(mcp, MCP_GPIO);
	if (status < 0)
		status = 0;
	else {
		mcp->cache[MCP_GPIO] = status;
		status = !!(status & (1 << offset));
	}
	mutex_unlock(&mcp->lock);
	return status;
}

static int __mcp23s08_set(struct mcp23s08 *mcp, unsigned mask, int value)
{
	unsigned olat = mcp->cache[MCP_OLAT];

	if (value)
		olat |= mask;
	else
		olat &= ~mask;
	mcp->cache[MCP_OLAT] = olat;
	return mcp->ops->write(mcp, MCP_OLAT, olat);
}

static void mcp23s08_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct mcp23s08	*mcp = container_of(chip, struct mcp23s08, chip);
	unsigned mask = 1 << offset;

	mutex_lock(&mcp->lock);
	__mcp23s08_set(mcp, mask, value);
	mutex_unlock(&mcp->lock);
}

static int
mcp23s08_direction_output(struct gpio_chip *chip, unsigned offset, int value)
{
	struct mcp23s08	*mcp = container_of(chip, struct mcp23s08, chip);
	unsigned mask = 1 << offset;
	int status;

	mutex_lock(&mcp->lock);
	status = __mcp23s08_set(mcp, mask, value);
	if (status == 0) {
		mcp->cache[MCP_IODIR] &= ~mask;
		status = mcp->ops->write(mcp, MCP_IODIR, mcp->cache[MCP_IODIR]);
	}
	mutex_unlock(&mcp->lock);
	return status;
}

/*----------------------------------------------------------------------
 *                                   PROC                              *
 *----------------------------------------------------------------------*/

#define INPUT_BUF_SIZE   256

// --------------- for debug ---------------
static int mcp23_proc_irq_counter_show (struct seq_file *seq, void *offset) {
	struct mcp23s08_driver_data	*data;
	struct mcp23s08			*mcp;

	struct mcp23_proc_data *pmcp = (struct mcp23_proc_data *)seq->private;

	if (!pmcp)
		return -EINVAL;

	data = pmcp->data;
	mcp  = data->mcp[pmcp->mcp_id];

	
	mutex_lock(&mcp->lock);

	seq_printf(seq, "GPIO_00: %04d\nGPIO_01: %04d\nGPIO_02: %04d\nGPIO_03: %04d\nGPIO_04: %04d\n\
GPIO_05: %04d\nGPIO_06: %04d\nGPIO_07: %04d\nGPIO_08: %04d\nGPIO_09: %04d\nGPIO_10: %04d\n\
GPIO_11: %04d\nGPIO_12: %04d\nGPIO_13: %04d\nGPIO_14: %04d\nGPIO_15: %04d\n",
			mcp->irq_counter[0],
			mcp->irq_counter[1],
			mcp->irq_counter[2],
			mcp->irq_counter[3],
			mcp->irq_counter[4],
			mcp->irq_counter[5],
			mcp->irq_counter[6],
			mcp->irq_counter[7],
			mcp->irq_counter[8],
			mcp->irq_counter[9],
			mcp->irq_counter[10],
			mcp->irq_counter[11],
			mcp->irq_counter[12],
			mcp->irq_counter[13],
			mcp->irq_counter[14],
			mcp->irq_counter[15]);
	mutex_unlock(&mcp->lock);

        return 0;
}


static int mcp23_proc_irq_counter_open_fs (struct inode *inode, struct file *file) {
        return single_open(file, mcp23_proc_irq_counter_show, PDE(inode)->data);
}


static const struct file_operations mcp23_proc_irq_counter_fops = {
        .owner = THIS_MODULE,
        .open = mcp23_proc_irq_counter_open_fs,
        .read = seq_read,
	.write = NULL,
        .llseek = seq_lseek,
        .release = single_release,
};


	
/* -------------------------- PROC ENABLE FILEs OPT (r/w) -------------------------- */

static int mcp23_proc_enable_show (struct seq_file *seq, void *offset) {
	struct mcp23s08_driver_data	*data;
	struct mcp23s08			*mcp;
	int                             shift;
	int                             enable;

	struct mcp23_proc_data *pmcp = (struct mcp23_proc_data *)seq->private;

	if (!pmcp)
		return -EINVAL;

	data = pmcp->data;
	mcp  = data->mcp[pmcp->mcp_id];

	
	mutex_lock(&mcp->lock);

	shift = pmcp->gid + (8 * pmcp->bid);
	enable = mcp->ops->read(mcp, MCP_GPINTEN);
	if (enable < 0)
		enable = 0;
	else {
		mcp->cache[MCP_GPINTEN] = enable;
		enable = !!(enable & (1 << shift));
	}
	mutex_unlock(&mcp->lock);

	seq_printf(seq, "%s\n", enable ? "enable" : "disable");  
        return 0;
}


static int mcp23_proc_enable_write (struct file *file, const char __user *buf,
			size_t count, loff_t *pos) {

	char 				input[INPUT_BUF_SIZE];
        int 				err_conv, shift;
	struct mcp23s08_driver_data	*data;
	struct mcp23s08			*mcp;
        long 				enable;
	unsigned int                    cache;
  
	struct mcp23_proc_data *pmcp = (struct mcp23_proc_data *)PDE(file->f_path.dentry->d_inode)->data;

	if (!pmcp)
		return -EINVAL;

	data = pmcp->data;
	mcp  = data->mcp[pmcp->mcp_id];
	shift = pmcp->gid + (8 * pmcp->bid);

        if (!capable(CAP_SYS_ADMIN))                                                                     
                return -EACCES;                                                                          
        
        if (count >= INPUT_BUF_SIZE)
                count = INPUT_BUF_SIZE - 1;                                                              
        
        memset(input, 0, INPUT_BUF_SIZE);
        if (copy_from_user (input, buf, count))                                                          
                return -EFAULT;                                                                          
        
        err_conv = strict_strtol (input, 0, &enable);

        if (err_conv == 0) {
                if (enable == 0 || enable == 1) {
			mutex_lock(&mcp->lock);
			cache = mcp->cache[MCP_GPINTEN];
			if (enable)
				cache |= (1 << shift);
			else
				cache &= ~(1 << shift);
			mcp->cache[MCP_GPINTEN] = cache;
			mcp->ops->write(mcp, MCP_GPINTEN, cache);
			mutex_unlock(&mcp->lock);

                } else  
                        return -EINVAL;                                                                  
        }
        return count;          
}


static int mcp23_proc_enable_open_fs (struct inode *inode, struct file *file) {
        return single_open(file, mcp23_proc_enable_show, PDE(inode)->data);
}


static const struct file_operations mcp23_proc_enable_fops = {
        .owner = THIS_MODULE,
        .open = mcp23_proc_enable_open_fs,
        .read = seq_read,
	.write = mcp23_proc_enable_write,
        .llseek = seq_lseek,
        .release = single_release,
};


/* -------------------------- PROC DEFVAL FILEs OPT (r/w) -------------------------- */

static int mcp23_proc_defval_show (struct seq_file *seq, void *offset) {
       	struct mcp23s08_driver_data	*data;
	struct mcp23s08			*mcp;
	int                             shift;
	int                             defval;

	struct mcp23_proc_data *pmcp = (struct mcp23_proc_data *)seq->private;

	if (!pmcp)
		return -EINVAL;

	data = pmcp->data;
	mcp  = data->mcp[pmcp->mcp_id];

	mutex_lock(&mcp->lock);

	shift = pmcp->gid + (8 * pmcp->bid);
	defval = mcp->ops->read(mcp, MCP_DEFVAL);
	if (defval < 0)
		defval = 0;
	else {
		mcp->cache[MCP_DEFVAL] = defval;
		defval = !!(defval & (1 << shift));
	}
	mutex_unlock(&mcp->lock);

	seq_printf(seq, "%s\n", defval ? "1" : "0");  
        return 0;
}


static int mcp23_proc_defval_write (struct file *file, const char __user *buf,
			size_t count, loff_t *pos) {
	
	char 				input[INPUT_BUF_SIZE];
        int 				err_conv, shift;
	struct mcp23s08_driver_data	*data;
	struct mcp23s08			*mcp;
        long 				defval;
	unsigned int                    cache;
  
	struct mcp23_proc_data *pmcp = (struct mcp23_proc_data *)PDE(file->f_path.dentry->d_inode)->data;

	if (!pmcp)
		return -EINVAL;

	data = pmcp->data;
	mcp  = data->mcp[pmcp->mcp_id];
	shift = pmcp->gid + (8 * pmcp->bid);

        if (!capable(CAP_SYS_ADMIN))                                                                     
                return -EACCES;                                                                          
        
        if (count >= INPUT_BUF_SIZE)
                count = INPUT_BUF_SIZE - 1;                                                              
        
        memset(input, 0, INPUT_BUF_SIZE);
        if (copy_from_user (input, buf, count))                                                          
                return -EFAULT;                                                                          
        
        err_conv = strict_strtol (input, 0, &defval);

        if (err_conv == 0) {
                if (defval == 0 || defval == 1) {
			mutex_lock(&mcp->lock);
			cache = mcp->cache[MCP_DEFVAL];
			if (defval)
				cache |= (1 << shift);
			else
				cache &= ~(1 << shift);
			mcp->cache[MCP_DEFVAL] = cache;
			mcp->ops->write(mcp, MCP_DEFVAL, cache);
			mutex_unlock(&mcp->lock);

                } else  
                        return -EINVAL;                                                                  
        }
        return count;          
}


static int mcp23_proc_defval_open_fs (struct inode *inode, struct file *file) {
        return single_open(file, mcp23_proc_defval_show, PDE(inode)->data);
}


static const struct file_operations mcp23_proc_defval_fops = {
        .owner = THIS_MODULE,
        .open = mcp23_proc_defval_open_fs,
        .read = seq_read,
	.write = mcp23_proc_defval_write,
        .llseek = seq_lseek,
        .release = single_release,
};

/* -------------------------- PROC CONTROL FILEs OPT (r/w) -------------------------- */

static int mcp23_proc_control_show (struct seq_file *seq, void *offset) {
       	struct mcp23s08_driver_data	*data;
	struct mcp23s08			*mcp;
	int                             shift;
	int                             ioc;

	struct mcp23_proc_data *pmcp = (struct mcp23_proc_data *)seq->private;

	if (!pmcp)
		return -EINVAL;

	data = pmcp->data;
	mcp  = data->mcp[pmcp->mcp_id];

	
	mutex_lock(&mcp->lock);

	shift = pmcp->gid + (8 * pmcp->bid);
	ioc = mcp->ops->read(mcp, MCP_INTCON);
	if (ioc < 0)
		ioc = 0;
	else {
		mcp->cache[MCP_INTCON] = ioc;
		ioc = !!(ioc & (1 << shift));
	}
	mutex_unlock(&mcp->lock);

	seq_printf(seq, "%s\n", ioc ? "default" : "prevalue");  
        return 0;
}


static int mcp23_proc_control_write (struct file *file, const char __user *buf,
			size_t count, loff_t *pos) {
	
	char 				input[INPUT_BUF_SIZE];
        int 				err_conv, shift;
	struct mcp23s08_driver_data	*data;
	struct mcp23s08			*mcp;
        long 				control;
	unsigned int                    cache;
  
	struct mcp23_proc_data *pmcp = (struct mcp23_proc_data *)PDE(file->f_path.dentry->d_inode)->data;

	if (!pmcp)
		return -EINVAL;

	data = pmcp->data;
	mcp  = data->mcp[pmcp->mcp_id];
	shift = pmcp->gid + (8 * pmcp->bid);

        if (!capable(CAP_SYS_ADMIN))                                                                     
                return -EACCES;                                                                          
        
        if (count >= INPUT_BUF_SIZE)
                count = INPUT_BUF_SIZE - 1;                                                              
        
        memset(input, 0, INPUT_BUF_SIZE);
        if (copy_from_user (input, buf, count))                                                          
                return -EFAULT;                                                                          
        
        err_conv = strict_strtol (input, 0, &control);

        if (err_conv == 0) {
                if (control == 0 || control == 1) {
			mutex_lock(&mcp->lock);
			cache = mcp->cache[MCP_INTCON];
			if (control)
				cache |= (1 << shift);
			else
				cache &= ~(1 << shift);
			mcp->cache[MCP_INTCON] = cache;
			mcp->ops->write(mcp, MCP_INTCON, cache);
			mutex_unlock(&mcp->lock);

                } else  
                        return -EINVAL;                                                                  
        }
        return count;          
}


static int mcp23_proc_control_open_fs (struct inode *inode, struct file *file) {
        return single_open(file, mcp23_proc_control_show, PDE(inode)->data);
}


static const struct file_operations mcp23_proc_control_fops = {
        .owner = THIS_MODULE,
        .open = mcp23_proc_control_open_fs,
        .read = seq_read,
	.write = mcp23_proc_control_write,
        .llseek = seq_lseek,
        .release = single_release,
};



static struct proc_dir_entry *mcp23_root_dir;
static struct proc_dir_entry *mcp23_chip_dir;


#define MCP23_PROC_ROOT             "mcp23"
#define MCP23_PROC_CHIP             "chip"
#define MCP23_PROC_EVN              "evn"  
#define MCP23_ENTRY_ENABLE          "enable"
#define MCP23_ENTRY_DEFVAL          "defval"
#define MCP23_ENTRY_CONTROL         "control"


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

static int mcp23_add_proc_fs (struct spi_device *spi) { 

	struct mcp23s08_driver_data	*data = spi_get_drvdata(spi);
	struct mcp23s08			*mcp;
	struct mcp23_proc_data          *dproc;
	struct mcp23_proc_data          *proc;
	int i, j, addr;
	struct proc_dir_entry *entry = NULL;
	struct proc_entry *tmp;
	struct list_head *pos, *q;
	char ctmp[30];  

	entry_list = kzalloc (sizeof (struct proc_entry), GFP_KERNEL);
	INIT_LIST_HEAD (&entry_list->list);

	event_list = kzalloc (sizeof (struct proc_dir_entry) * data->ngpio, GFP_KERNEL);

	/* create /proc/mcp23 */
	mcp23_root_dir = proc_mkdir (MCP23_PROC_ROOT, NULL);
	if (!mcp23_root_dir)
		return -ENODEV;
	else 
		add_to_list (tmp, entry_list, MCP23_PROC_ROOT, NULL);


	for (j = 0 ; j < ARRAY_SIZE(data->mcp) ; j++) {

		addr = j;
		mcp = data->mcp[addr];
		if (mcp != NULL) {

			/* create /proc/mcp23/chip_x */
			sprintf(ctmp, "%s%d", MCP23_PROC_CHIP, addr);
			mcp23_chip_dir = proc_mkdir (ctmp, mcp23_root_dir);
			if (!mcp23_chip_dir)
				return -ENODEV;
			else 
				add_to_list (tmp, entry_list, MCP23_PROC_CHIP, mcp23_root_dir);

				proc = kzalloc (sizeof (struct mcp23_proc_data), GFP_KERNEL);
				proc->data = data;
				proc->mcp_id = addr;
				proc->bid = 0;
				proc->gid = 0;

				entry = proc_create_data("irq_counter",  S_IRUGO | S_IWUGO,
						mcp23_chip_dir, &mcp23_proc_irq_counter_fops,
						proc);
				if (!entry) {
					goto remove_dir;
				} else {
					add_to_list (tmp, entry_list, "irq_counter", mcp23_chip_dir);
				}

			for (i = 0; i < mcp->chip.ngpio ; i++) {

				dproc = kzalloc (sizeof (struct mcp23_proc_data), GFP_KERNEL);

				dproc->data = data;
				dproc->mcp_id = addr;
				dproc->bid = (int)(i / 8) ? BLOCK_B : BLOCK_A;
				dproc->gid = i % 8;

				sprintf(ctmp, "%s%d", MCP23_PROC_EVN, i);
				event_list[i] = proc_mkdir (ctmp, mcp23_chip_dir);
				if (!event_list[i]) {
					goto remove_dir;
				} else {
					add_to_list (tmp, entry_list, ctmp, mcp23_chip_dir);
				}

				// file

				entry = proc_create_data(MCP23_ENTRY_ENABLE,  S_IRUGO | S_IWUGO,
						event_list[i], &mcp23_proc_enable_fops,
						dproc);
				if (!entry) {
					goto remove_dir;
				} else {
					add_to_list (tmp, entry_list, MCP23_ENTRY_ENABLE, event_list[i]);
				}


				entry = proc_create_data(MCP23_ENTRY_DEFVAL,  S_IRUGO | S_IWUGO,
						event_list[i], &mcp23_proc_defval_fops,
						dproc);
				if (!entry) {
					goto remove_dir;
				} else {
					add_to_list (tmp, entry_list, MCP23_ENTRY_DEFVAL, event_list[i]);
				}


				entry = proc_create_data(MCP23_ENTRY_CONTROL,  S_IRUGO | S_IWUGO,
						event_list[i], &mcp23_proc_control_fops,
						dproc);
				if (!entry) {
					goto remove_dir;
				} else {
					add_to_list (tmp, entry_list, MCP23_ENTRY_CONTROL, event_list[i]);
				}

			}
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


static void mcp_remove_proc_fs (struct mcp23s08_driver_data *data) {
        struct proc_entry *tmp;
        struct list_head *pos, *q;
        list_for_each_safe (pos, q, &entry_list->list) {
                tmp = list_entry (pos, struct proc_entry, list);
                remove_proc_entry (tmp->label, tmp->entry);
                list_del (pos);
                kfree (tmp);
        }
}


#ifdef CONFIG_DEBUG_FS

#include <linux/seq_file.h>

/*
 * This shows more info than the generic gpio dump code:
 * pullups, deglitching, open drain drive.
 */
static void mcp23s08_dbg_show(struct seq_file *s, struct gpio_chip *chip)
{
	struct mcp23s08	*mcp;
	char		bank;
	int		t;
	unsigned	mask;

	mcp = container_of(chip, struct mcp23s08, chip);

	/* NOTE: we only handle one bank for now ... */
	bank = '0' + ((mcp->addr >> 1) & 0x7);

	mutex_lock(&mcp->lock);
	t = mcp->ops->read_regs(mcp, 0, mcp->cache, ARRAY_SIZE(mcp->cache));
	if (t < 0) {
		seq_printf(s, " I/O ERROR %d\n", t);
		goto done;
	}

	for (t = 0, mask = 1; t < chip->ngpio; t++, mask <<= 1) {
		const char	*label;

		label = gpiochip_is_requested(chip, t);
		if (!label)
			continue;

		seq_printf(s, " gpio-%-3d P%c.%d (%-12s) %s %s %s",
			chip->base + t, bank, t, label,
			(mcp->cache[MCP_IODIR] & mask) ? "in " : "out",
			(mcp->cache[MCP_GPIO] & mask) ? "hi" : "lo",
			(mcp->cache[MCP_GPPU] & mask) ? "  " : "up");
		/* NOTE:  ignoring the irq-related registers */
		seq_printf(s, "\n");
	}
done:
	mutex_unlock(&mcp->lock);
}

#else
#define mcp23s08_dbg_show	NULL
#endif

/*----------------------------------------------------------------------*/

static void disable_all_irqs (struct  mcp23s08_driver_data *data) {
	struct mcp23s08         *mcp;
	int                     i;
	for (i = 0 ; i < ARRAY_SIZE (data->mcp) ; i++) {
		if (data->mcp[i]) {
			mcp = data->mcp[i];
			if (mcp->inta != 0) 
				disable_irq_nosync (mcp->inta);
			if (mcp->intb != 0) 
				disable_irq_nosync (mcp->intb);
		}
	}
}


static void enable_all_irqs (struct  mcp23s08_driver_data *data) {
	struct mcp23s08         *mcp;
	int                     i;
	for (i = 0 ; i < ARRAY_SIZE (data->mcp) ; i++) {
		if (data->mcp[i]) {
			mcp = data->mcp[i];
			if (mcp->inta != 0) 
				enable_irq (mcp->inta);
			if (mcp->intb != 0) 
				enable_irq (mcp->intb);
		}
	}
}


static void mcp23_work (struct work_struct *work) {

	struct  mcp23s08_driver_data *data = container_of(to_delayed_work(work), 
				struct  mcp23s08_driver_data, work);
	struct mcp23s08         *mcp;	
	int                     i, j;
	int 			reg_f, reg_g;

	if (!data)
		return;
	
	for (i = 0 ; i < ARRAY_SIZE (data->mcp) ; i++) {
		
		if (data->mcp[i]) {
			mcp = data->mcp[i];
			if (mcp->inta == data->irq_call || mcp->intb == data->irq_call) {
				reg_f = mcp->ops->read(mcp, MCP_INTF);
				for (j = 0 ; j < 16 ; j++) {
					if (!!(reg_f & (1 << j))) {
						input_report_key (data->events[j]->input, KEY_POWER, 1);
			                        input_sync (data->events[j]->input);
                        			input_report_key (data->events[j]->input, KEY_POWER, 0);
			                        input_sync (data->events[j]->input);

						mcp->irq_counter[j]++;
					}
				}
				reg_g = mcp->ops->read(mcp, MCP_GPIO);
				reg_g = mcp->ops->read(mcp, MCP_INTCAP);
			}
		}
	}
	enable_all_irqs (data);
} 



static irqreturn_t irq_inta_manager (int irq, void *dev_id)  {
	struct  mcp23s08_driver_data *data = (struct mcp23s08_driver_data *)dev_id;

	disable_all_irqs (data); 
	data->irq_call = irq; 

	schedule_delayed_work (&data->work, msecs_to_jiffies(data->poll_period));

	return IRQ_HANDLED;
}


static irqreturn_t irq_intb_manager (int irq, void *dev_id)  {
	struct  mcp23s08_driver_data *data = (struct mcp23s08_driver_data *)dev_id;

	disable_all_irqs (data); 
	data->irq_call = irq; 

	schedule_delayed_work (&data->work, msecs_to_jiffies(data->poll_period));

	return IRQ_HANDLED;
}


static int mcp23s08_probe_one(struct spi_device *spi, unsigned addr,
			      unsigned type, unsigned base, unsigned pullups)
{
	struct mcp23s08_driver_data	*data = spi_get_drvdata(spi);
	struct mcp23s08			*mcp = data->mcp[addr];
	int				status, i, ret, err;

	mutex_init(&mcp->lock);

	mcp->spi = spi;
	mcp->addr = 0x40 | (addr << 1);

	mcp->chip.direction_input = mcp23s08_direction_input;
	mcp->chip.get = mcp23s08_get;
	mcp->chip.direction_output = mcp23s08_direction_output;
	mcp->chip.set = mcp23s08_set;
	mcp->chip.dbg_show = mcp23s08_dbg_show;

	if (type == MCP_TYPE_S17) {
		mcp->ops = &mcp23s17_ops;
		mcp->chip.ngpio = 16;
		mcp->chip.label = "mcp23s17";
	} else {
		mcp->ops = &mcp23s08_ops;
		mcp->chip.ngpio = 8;
		mcp->chip.label = "mcp23s08";
	}
	mcp->chip.base = base;
	mcp->chip.can_sleep = 1;
	mcp->chip.dev = &spi->dev;
	mcp->chip.owner = THIS_MODULE;

	/* verify MCP_IOCON.SEQOP = 0, so sequential reads work,
	 * and MCP_IOCON.HAEN = 1, so we work with all chips.
	 */
	status = mcp->ops->read(mcp, MCP_IOCON);
	if (status < 0)
		goto fail;
	if ((status & IOCON_SEQOP) || !(status & IOCON_HAEN)) {
		/* mcp23s17 has IOCON twice, make sure they are in sync */
		status &= ~(IOCON_SEQOP | (IOCON_SEQOP << 8));
		status |= IOCON_HAEN | (IOCON_HAEN << 8);
		status = mcp->ops->write(mcp, MCP_IOCON, status);
		if (status < 0)
			goto fail;
	}

	/* configure ~100K pullups */
	status = mcp->ops->write(mcp, MCP_GPPU, pullups);
	if (status < 0)
		goto fail;

	status = mcp->ops->read_regs(mcp, 0, mcp->cache, ARRAY_SIZE(mcp->cache));
	if (status < 0)
		goto fail;

	/* disable inverter on input */
	if (mcp->cache[MCP_IPOL] != 0) {
		mcp->cache[MCP_IPOL] = 0;
		status = mcp->ops->write(mcp, MCP_IPOL, 0);
		if (status < 0)
			goto fail;
	}

	/* disable irqs */
	if (mcp->cache[MCP_GPINTEN] != 0) {
		mcp->cache[MCP_GPINTEN] = 0;
		status = mcp->ops->write(mcp, MCP_GPINTEN, 0);
		if (status < 0)
			goto fail;
	}

	/* assign irqs, if present */
	if (data->inta[addr] != 0) {
		mcp->inta = data->inta[addr];	
		ret = request_threaded_irq(data->inta[addr], NULL, irq_inta_manager,
						data->irq_flags_a[addr], "mcp23_inta", data);
	
	        if (ret) {
        	        MCP_ERR ("IRQ_a not acquired: error %d", ret);
               		 err = -EIO;
	                goto err_free_irq_a;
        	}
	}
	
	if (data->intb[addr] != 0) { 
		mcp->intb = data->intb[addr];	
		ret = request_threaded_irq(data->intb[addr], NULL, irq_intb_manager,
						data->irq_flags_b[addr], "mcp23_intb", data);
	
	        if (ret) {
        	        MCP_ERR ("IRQ_b not acquired: error %d", ret);
               	 	err = -EIO;
                	goto err_free_irq_b;
        	}
	}

	for (i = 0 ; i < 16 ; i++)
		mcp->irq_counter[i] = 0;

	status = gpiochip_add(&mcp->chip);

err_free_irq_b:
err_free_irq_a:
fail:
	if (status < 0)
		dev_dbg(&spi->dev, "can't setup chip %d, --> %d\n",
				addr, status);
	return status;
}


static int __devinit mcp23s08_probe(struct spi_device *spi)
{
	struct mcp23s08_platform_data	*pdata;
	unsigned			addr;
	unsigned			chips = 0;
	struct mcp23s08_driver_data	*data;
	int				status = 0, type, i, regs = 0, err = 0;
	unsigned			base;
	struct event_dev                **events;

	type = spi_get_device_id(spi)->driver_data;

	pdata = spi->dev.platform_data;
	if (!pdata || !gpio_is_valid(pdata->base)) {
		dev_dbg(&spi->dev, "invalid or missing platform data\n");
		return -EINVAL;
	}

	for (addr = 0; addr < ARRAY_SIZE(pdata->chip); addr++) {
		if (!pdata->chip[addr].is_present)
			continue;
		chips++;
		if ((type == MCP_TYPE_S08) && (addr > 3)) {
			dev_err(&spi->dev,
				"mcp23s08 only supports address 0..3\n");
			return -EINVAL;
		}
	}
	if (!chips)
		return -ENODEV;

	data = kzalloc(sizeof *data + chips * sizeof(struct mcp23s08),
			GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	for (addr = 0; addr < ARRAY_SIZE(pdata->chip); addr++) {
		data->inta[addr] = pdata->irq_a[addr];
		data->intb[addr] = pdata->irq_b[addr];
		data->irq_flags_a[addr] = pdata->irq_flags_a[addr];
		data->irq_flags_b[addr] = pdata->irq_flags_b[addr];
	}

	spi_set_drvdata(spi, data);
	data->spi = spi;

	base = pdata->base;
	/*Reset GPIO Expander*/
	pdata->reset();
	for (addr = 0; addr < ARRAY_SIZE(pdata->chip); addr++) {
		if (!pdata->chip[addr].is_present) {
			data->mcp[addr] = NULL;
			continue;
		}
		chips--;
		data->mcp[addr] = &data->chip[chips];
		status = mcp23s08_probe_one(spi, addr, type, base,
					    pdata->chip[addr].pullups);
		if (status < 0)
			goto fail;

		base += (type == MCP_TYPE_S17) ? 16 : 8;
	}
	data->ngpio = base - pdata->base;

	/* NOTE:  these chips have a relatively sane IRQ framework, with
	 * per-signal masking and level/edge triggering.  It's not yet
	 * handled here...
	 */

	if (pdata->setup) {
		status = pdata->setup(spi,
				pdata->base, data->ngpio,
				pdata->context);
		if (status < 0)
			dev_dbg(&spi->dev, "setup --> %d\n", status);
	}
	


	INIT_DELAYED_WORK (&data->work, mcp23_work);
	data->poll_period  = MCP23_POLL_PERIOD;

	/* input interface */
	events =  kzalloc (sizeof (struct event_dev ) * data->ngpio, GFP_KERNEL);
        if (!events) {
                err = -ENOMEM;
                goto fail;
        }

        for (i = 0 ; i < data->ngpio; i++) {
		events[i] = kzalloc (sizeof (struct event_dev ), GFP_KERNEL);
		events[i]->input = input_allocate_device();
		if (!events[i]->input) {
                        err = -ENOMEM;
                        goto fail;
                }

		snprintf (events[i]->phys, sizeof (events[i]->phys),
					"%s/input%d", dev_name(&spi->dev), i);

		snprintf (events[i]->name, sizeof (events[i]->name),
					"MCP23 GPIO Expander");

		events[i]->input->phys = events[i]->phys;
		events[i]->input->name = events[i]->name;
		events[i]->input->id.bustype = BUS_SPI;
                events[i]->input->dev.parent = &spi->dev;

                events[i]->input->evbit[0] = BIT_MASK(EV_KEY);
		set_bit (KEY_POWER, events[i]->input->keybit);

        }

	data->events = events;

	mcp23_add_proc_fs (spi);

	for (i = 0 ; i < data->ngpio; i++) {
		err = input_register_device (events[i]->input);
                if (err) {
                        err = -EINVAL;
                        regs = i;
                        goto fail;
                }
        }
	return 0;

fail:
	printk (KERN_ERR "Probe Failed, error %d\n", err);
	for (addr = 0; addr < ARRAY_SIZE(data->mcp); addr++) {
		int tmp;

		if (!data->mcp[addr])
			continue;
		tmp = gpiochip_remove(&data->mcp[addr]->chip);
		if (tmp < 0)
			dev_err(&spi->dev, "%s --> %d\n", "remove", tmp);
	}
	kfree(data);
	return status;
}

static int mcp23s08_remove(struct spi_device *spi)
{
	struct mcp23s08_driver_data	*data = spi_get_drvdata(spi);
	struct mcp23s08_platform_data	*pdata = spi->dev.platform_data;
	unsigned			addr;
	int				status = 0;

	if (pdata->teardown) {
		status = pdata->teardown(spi,
				pdata->base, data->ngpio,
				pdata->context);
		if (status < 0) {
			dev_err(&spi->dev, "%s --> %d\n", "teardown", status);
			return status;
		}
	}

	for (addr = 0; addr < ARRAY_SIZE(data->mcp); addr++) {
		int tmp;

		if (!data->mcp[addr])
			continue;

		tmp = gpiochip_remove(&data->mcp[addr]->chip);
		if (tmp < 0) {
			dev_err(&spi->dev, "%s --> %d\n", "remove", tmp);
			status = tmp;
		}
	}

	mcp_remove_proc_fs (data);

	if (status == 0)
		kfree(data);
	return status;
}

static const struct spi_device_id mcp23s08_ids[] = {
	{ "mcp23s08", MCP_TYPE_S08 },
	{ "mcp23s17", MCP_TYPE_S17 },
	{ },
};
MODULE_DEVICE_TABLE(spi, mcp23s08_ids);

static struct spi_driver mcp23s08_driver = {
	.probe		= mcp23s08_probe,
	.remove		= mcp23s08_remove,
	.id_table	= mcp23s08_ids,
	.driver = {
		.name	= "mcp23s08",
		.owner	= THIS_MODULE,
	},
};

MODULE_ALIAS("platform:mcp23");

/*----------------------------------------------------------------------*/

static int __init mcp23s08_init(void)
{
	return spi_register_driver(&mcp23s08_driver);
}
/* register after spi postcore initcall and before
 * subsys initcalls that may rely on these GPIOs
 */
module_init(mcp23s08_init);

static void __exit mcp23s08_exit(void)
{
	spi_unregister_driver(&mcp23s08_driver);
}
module_exit(mcp23s08_exit);

MODULE_LICENSE("GPL");
