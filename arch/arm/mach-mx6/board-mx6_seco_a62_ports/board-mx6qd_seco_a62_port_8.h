
//#ifndef _BOARD_MX6_SECO_A62_PORT_8_H
//#define _BOARD_MX6_SECO_A62_PORT_8_H

#include <mach/iomux-mx6q.h>
#ifdef CONFIG_A62_PORT_8_CONF_1
// Conf 1 - UART1
static iomux_v3_cfg_t mx6qd_seco_a62_port_8_conf_1_pads[] = {

	MX6Q_PAD_CSI0_DAT10__UART1_TXD,		
	MX6Q_PAD_CSI0_DAT11__UART1_RXD,		

};
#endif
#ifdef CONFIG_A62_PORT_8_CONF_2
// Conf 2 - GENERIC GPIO
static iomux_v3_cfg_t mx6qd_seco_a62_port_8_conf_2_pads[] = {

	MX6Q_PAD_CSI0_DAT10__GPIO_5_28,         
        MX6Q_PAD_CSI0_DAT11__GPIO_5_29,

};
#endif
