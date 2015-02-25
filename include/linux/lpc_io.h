#include <linux/ioctl.h>

struct lpc_reg {        
	unsigned int reg_addr;
	int data;               
};


struct lpc_mem {        
	unsigned long mem_addr; 
	uint16_t data;          
};


#define LPC_READ_REG    _IOR('q', 1, struct lpc_reg *)
#define LPC_WRITE_REG   _IOW('q', 2, struct lpc_reg *)
#define LPC_READ_MEM    _IOR('q', 3, struct lpc_mem *)
#define LPC_WRITE_MEM   _IOW('q', 4, struct lpc_mem *)
