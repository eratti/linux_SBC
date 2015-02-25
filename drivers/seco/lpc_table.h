#define SLOT_FREE   0x00
#define SLOT_BUSY   0x01


struct lpc_table_element {
	int             lpc_slot;
// device info
	struct device   *dev;
	irqreturn_t     (**handler)(int irq, void *dev_id);
	void            (*after_handler)(int irq, void *dev_id);
	void  		**irq_dev_id;
	int             maskable;
// table info
	uint8_t         state;
	unsigned long   irq_hits;
};


extern int lpc_table_init (struct platform_device *device, struct lpc_table_element **table, int num_slots);


extern int lpc_table_size (struct lpc_table_element *table);


extern struct lpc_table_element *lpc_table_get_element (struct lpc_table_element *table, int num_slot);


extern int lpc_table_element_is_free (struct lpc_table_element *table, int num_slot);


extern int lpc_table_add_device (struct lpc_table_element **table, struct lpc_table_element element);


extern  int lpc_table_size (struct lpc_table_element *table);

