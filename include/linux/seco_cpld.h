#ifndef __LINUX_SECO_CPLD_H
#define __LINUX_SEC0_CPLD_H


#define CPLD_OFFSET_BASE 	 0xF000

#define CPLD_REG_0               0x0
#define CPLD_REG_1               0x1
#define CPLD_REG_2               0x2
#define CPLD_REG_3               0x3
#define CPLD_REG_4               0x4
#define CPLD_REG_5               0x5
#define CPLD_REG_6               0x6
#define CPLD_REG_7               0x7
#define CPLD_REG_8               0x8
#define CPLD_REG_9               0x9
#define CPLD_REG_10              0xa

#define REG_REVISION             CPLD_REG_0


struct cpld_data {
	const char           *name;
	resource_size_t      mem_addr_base;
	unsigned long long   mem_size;
	void __iomem         *virt;
	struct mutex         bus_lock;
};


extern int cpld_reg_read (unsigned int addr, int *data);

extern int cpld_reg_write (unsigned int addr, int value);

extern int cpld_read (unsigned int addr, uint16_t *data);

extern int cpld_write (unsigned int addr, uint16_t value);

extern int cpld_get_membase (void);

extern int cpld_init (struct resource  *resource);

extern int cpld_get_revision (void);

extern int cpld_is_gpio (void);

extern int cpld_is_lpc (void);

//extern void cpld_remove (struct platform_device *device);

#endif  /* __LINUX_SECO_CPLD_H */
