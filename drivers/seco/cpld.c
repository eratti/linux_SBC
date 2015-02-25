
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/seco_cpld.h>
#include <linux/slab.h>


#define REG_ADDR(x)  (CPLD_OFFSET_BASE + (x*2))
#define WEIM_ADDR(x) (cpld_d->virt + (x))


struct cpld_data *cpld_d;


int cpld_reg_read (unsigned int addr, int *data) {
	*data = readw (WEIM_ADDR(REG_ADDR(addr)));
	return 0;
}


int cpld_reg_write (unsigned int addr, int value) {
	writew (value, WEIM_ADDR(REG_ADDR(addr)));
	return 0;
}


int cpld_read (unsigned int addr, uint16_t *data) {
	*data = readw (WEIM_ADDR(addr << 1));
	return 0;
}


int cpld_write (unsigned int addr, uint16_t value) {
	writew (value, WEIM_ADDR(addr << 1));
	return 0;
}


int cpld_get_membase (void) {
	return cpld_d->mem_addr_base;
}


int cpld_get_revision (void) {
	int rev;
	cpld_reg_read (REG_REVISION, &rev);
	return rev;
}


int cpld_is_gpio (void) {
	int is_gpio = 0;
	int rev = cpld_get_revision ();
	switch (rev) {
		case 0x3a06:
		case 0x3a04:
		case 0x3a02:
			is_gpio = 1;
			break;
		default:
			is_gpio = 0;
			break;
	}
	return is_gpio;
}


int cpld_is_lpc (void) {
	return !cpld_is_gpio ();
}


#define CPLD_NAME   "CPLD_device"

int cpld_init (struct resource *resource) {
	int rev = 0;
	int err = 0;

	cpld_d = kzalloc(sizeof(struct cpld_data), GFP_KERNEL);
	if (cpld_d == NULL) {
		err = ENOMEM;
		goto err_out;
	}
	
	printk (KERN_INFO "platform CPLD device: %.8llx at %.8llx\n",
		       (unsigned long long)resource_size(&resource[0]), 
	               (unsigned long long)resource[0].start);	       

	if (!request_mem_region(resource[0].start,
				resource_size(&resource[0]),
				CPLD_NAME)) {
		pr_err("Could not reserve memory region\n");
		err = -ENOMEM;
		goto err_out;
	}
	pr_debug ("%s: memory request done!\n", __func__);

	cpld_d->name = CPLD_NAME;
	cpld_d->mem_addr_base = resource[0].start;
	cpld_d->mem_size = resource_size(&resource[0]);
	cpld_d->virt = ioremap_nocache (cpld_d->mem_addr_base, cpld_d->mem_size);
	if (cpld_d->virt == NULL) {
		pr_err ("Failed to ioremap seco CPLD region\n");
		err = -EIO;
		goto err_out;
	}

	mutex_init (&cpld_d->bus_lock);

	cpld_reg_read (REG_REVISION, &rev);
	printk (KERN_INFO "log: %04x\n", rev);
	return 0;

err_out:
	return err;
}
