
//#ifndef _BOARD_MX6_SECO_A62_PORT_4_H
//#define _BOARD_MX6_SECO_A62_PORT_4_H

#include <mach/iomux-mx6q.h>
#ifdef CONFIG_A62_PORT_4_CONF_1
// Conf 1 - UART 4
static iomux_v3_cfg_t mx6qd_seco_a62_port_4_conf_1_pads[] = {

	MX6Q_PAD_CSI0_DAT12__UART4_TXD,	
	MX6Q_PAD_CSI0_DAT13__UART4_RXD,
	MX6Q_PAD_CSI0_DAT16__UART4_RTS,
	MX6Q_PAD_CSI0_DAT17__UART4_CTS,

};
#endif
#ifdef CONFIG_A62_PORT_4_CONF_2
// Conf 2 - GENERIC GPIO
static iomux_v3_cfg_t mx6qd_seco_a62_port_4_conf_2_pads[] = {

 	MX6Q_PAD_CSI0_DAT12__GPIO_5_30,
	MX6Q_PAD_CSI0_DAT13__GPIO_5_31,
	MX6Q_PAD_CSI0_DAT16__GPIO_6_2,
	MX6Q_PAD_CSI0_DAT17__GPIO_6_3,  

};
#endif
