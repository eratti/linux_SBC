#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include "lpc_table.h"


int lpc_table_init (struct platform_device *device, struct lpc_table_element **table, int num_slots) {
	int err = 0;
	int ret = 0;
	int i;

	if (unlikely(num_slots <= 0)) {
		dev_err (&device->dev, "Error in LPC table initialing\n");
		err = -1;
		goto err_out;
	} else {
		(*table) = devm_kzalloc (&device->dev,
				// we add an additional element in tail and set its lpc_slots as -1
				// (this element acts as END OF LIST and is not usefully to anyone)
				sizeof(struct lpc_table_element) * (num_slots + 1), GFP_KERNEL);
		if ((*table) == NULL) {
			dev_err (&device->dev, "Error in LPC table allocation\n"); 
			err = -ENOMEM;
			goto err_out;
		}
		ret = 0;
		struct lpc_table_element *t = (*table);
		for (i = 0 ; i < num_slots ; i++, t++) {
			t->lpc_slot = i;
			t->dev            = NULL;
			t->handler        = NULL;
			t->after_handler  = NULL;
			t->maskable       = 0x0;
			t->state          = SLOT_FREE;
			t->irq_hits       = 0;
			ret++;
		}
		t->lpc_slot = -1;
		t->state    = SLOT_FREE;
	}
	return ret;
err_out:
	return err;
}


int lpc_table_size (struct lpc_table_element *table) {
	int size = 0;
	struct lpc_table_element *t;
	for (size = 0, t = table ; t->lpc_slot >= 0 ; size++, t++) {}
	return size;
}


int lpc_table_index_validator (struct lpc_table_element *table, int num_slot) {
	// 0 --> index not valid
	// 1 --> index valid
	return  (num_slot < 0 || num_slot > lpc_table_size (table)) ? 0 : 1;
}


struct lpc_table_element *lpc_table_get_element (struct lpc_table_element *table, int num_slot) {
	struct lpc_table_element *new_element;

	if (lpc_table_index_validator (table, num_slot) == 0) {
		goto err_out;
	}

	new_element = kzalloc (sizeof (struct lpc_table_element), GFP_KERNEL);
	if (new_element == NULL) {
		pr_debug ("%s: Element not allocated!\n", __func__);
		goto err_out;
	}

	new_element->lpc_slot       = table->lpc_slot;
	new_element->dev            = table->dev;
	new_element->handler        = table->handler;
	new_element->after_handler  = table->after_handler;
	new_element->state          = table->state;
	new_element->irq_hits       = table->irq_hits;
	new_element->maskable       = table->maskable;
		
	return new_element;
err_out:
	return NULL;
}


int lpc_table_element_is_free (struct lpc_table_element *table, int num_slot) {
	uint8_t ret;
	if (lpc_table_index_validator (table, num_slot) == 0) {
		ret = !SLOT_BUSY;
	} else {
		ret = !table[num_slot].state;
	}
	return ret;
}


int lpc_table_add_device (struct lpc_table_element **table, struct lpc_table_element element) {
	int ret = -1;
	int size = lpc_table_size (*table);
	int isValid = lpc_table_index_validator (*table, element.lpc_slot);
	if (!isValid) {
		ret = -1;
	} else {
		// slot index valid
		if (!lpc_table_element_is_free (*table, element.lpc_slot)) {
			ret = -1;
		} else {
			// slot free
			(*table)[element.lpc_slot].dev = element.dev;
			(*table)[element.lpc_slot].handler = element.handler;
			(*table)[element.lpc_slot].after_handler = element.after_handler;
			(*table)[element.lpc_slot].irq_dev_id = element.irq_dev_id;
			(*table)[element.lpc_slot].maskable = element.maskable;

			(*table)[element.lpc_slot].state = SLOT_BUSY;
			ret = element.lpc_slot;	
		}
	}
	
	return ret;	
}





