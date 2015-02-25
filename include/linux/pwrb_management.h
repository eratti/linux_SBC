#ifndef __LINUX_PWRB_MNG_H
#define __LINUX_PWRB_MNG_H

/* linux/pwrb_management.h */

struct imx_seco_pwrb_platform_data {

		unsigned long irq_flags;
		unsigned long pwrb_gpio;	
		unsigned long halt_gpio;
};
#endif
