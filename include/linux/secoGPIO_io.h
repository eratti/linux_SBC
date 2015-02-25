#include <linux/ioctl.h>

#define NR_GPIO_EVENT  8

typedef enum {
	GPIO_EVN_0 = 0,
	GPIO_EVN_1 = 1,
	GPIO_EVN_2 = 2,
	GPIO_EVN_3 = 3,
	GPIO_EVN_4 = 4,
	GPIO_EVN_5 = 5,
	GPIO_EVN_6 = 6,
	GPIO_EVN_7 = 7,
} GPIO_EVN_ID;


typedef enum {
	TRIGGERING_LEVEL = 0,
	TRIGGERING_EDGE  = 1,
} GPIO_TRIGGERING;


typedef enum {
	EVN_UNMASK = 0,
	EVN_MASK   = 1,
} GPIO_MASK;


typedef enum {
	EVN_DIS = 0,
	EVN_EN  = 1,
} GPIO_STATE;


struct gpio_evn_seco {
	GPIO_EVN_ID       id;
	GPIO_STATE        state;
	GPIO_MASK         mask;	
	GPIO_TRIGGERING   conf;
};


#define GPIO_EVN_SECO_GET     _IOR('q', 1, struct gpio_evn_seco *)
#define GPIO_EVN_SECO_SET     _IOW('q', 2, struct gpio_evn_seco *)

