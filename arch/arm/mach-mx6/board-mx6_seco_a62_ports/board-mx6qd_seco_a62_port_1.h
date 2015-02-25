
//#ifndef _BOARD_MX6_SECO_A62_PORT_1_H
//#define _BOARD_MX6_SECO_A62_PORT_1_H

#include <mach/iomux-mx6q.h>

// Conf 1 - EXP_GPIO_1 - GENERIC GPIO
#ifdef CONFIG_A62_PORT_1_CONF_1 
static iomux_v3_cfg_t mx6qd_seco_a62_port_1_conf_1_pads[] = {

	MX6Q_PAD_GPIO_9__GPIO_1_9,

};
#endif
