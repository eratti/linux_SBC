
#define LPC_REG_IRQ_BUFFER     CPLD_REG_1
#define LPC_REG_MEM_PAGE_SEL   CPLD_REG_2
#define LPC_REG_IRQ_MASK       CPLD_REG_3
#define LPC_REG_LPC_BUSY       CPLD_REG_4
#define LPC_REG_IRQ_CONF       CPLD_REG_5

#define NR_IRQ 3

#define LPC_BUSY   0x1

#define NR_RETRIES     10
#define LPC_TIME_OUT   100

#define IRQ_MASKERED     0x00 
#define IRQ_UNMASKERED   0x01

#define LPC_IRQ_MASKABLE      0x1
#define LPC_IRQ_UNMASKABLE    0x0

struct lpc_device {
	int             lpc_slot;
	struct device   *dev;
	irqreturn_t     (**handler)(int irq, void *dev_id);
	void            (*after_handler)(int irq, void *dev_id);
	void            **irq_dev_id;
	int             maskable;
};


struct lpc_data {
	const char           *name;
	struct               mutex bus_lock;
	resource_size_t      mem_addr_base;
	unsigned long        mem_base;

	int                  irqs[NR_IRQ];
	int                  irq_flags[NR_IRQ];
};

extern int lpc_readw (unsigned long mem_addr, uint16_t *data);

extern int lpc_writew (unsigned long mem_addr, uint16_t value);
	

extern unsigned long lpc_getIRQ_flags (int irq);

extern int lpc_getIRQ (int irq); 

extern unsigned long lpc_getMemBase (void);

extern int lpc_add_device (struct lpc_device *lpc_dev);

extern void irq_slot_set (int slot, int en);

struct seco_lpc_platform_data {
	int                      *irqs;
	unsigned long            *flags_irq;
	int                      irq_count;
	struct platform_device   **devices;
	int                      num_devices;
};
