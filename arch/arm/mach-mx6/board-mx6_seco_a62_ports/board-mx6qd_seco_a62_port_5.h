
//#ifndef _BOARD_MX6_SECO_A62_PORT_5_H
//#define _BOARD_MX6_SECO_A62_PORT_5_H

#include <mach/iomux-mx6q.h>
#ifdef CONFIG_A62_PORT_5_CONF_1
// Conf 1 - SPDIF
static iomux_v3_cfg_t mx6qd_seco_a62_port_5_conf_1_pads[] = {

	MX6Q_PAD_EIM_D28__GPIO_3_28,		// EXP_GPIO_12	
	MX6Q_PAD_EIM_D21__SPDIF_IN1,		// EXP_GPIO_13
	MX6Q_PAD_GPIO_19__SPDIF_OUT1,		// EXP_GPIO_14

};
#endif
#ifdef CONFIG_A62_PORT_5_CONF_2
// Conf 1 - I2C1
static iomux_v3_cfg_t mx6qd_seco_a62_port_5_conf_2_pads[] = {

	MX6Q_PAD_EIM_D28__I2C1_SDA,		// EXP_GPIO_12  
        MX6Q_PAD_EIM_D21__I2C1_SCL,		// EXP_GPIO_13
        MX6Q_PAD_GPIO_19__GPIO_4_5,		// EXP_GPIO_14

};
#endif
#ifdef CONFIG_A62_PORT_5_CONF_3
// Conf 2 - GENERIC GPIO
static iomux_v3_cfg_t mx6qd_seco_a62_port_5_conf_3_pads[] = {

	MX6Q_PAD_EIM_D28__GPIO_3_28,            // EXP_GPIO_12
	MX6Q_PAD_EIM_D21__GPIO_3_21,		// EXP_GPIO_13
	MX6Q_PAD_GPIO_19__GPIO_4_5,             // EXP_GPIO_14

};
#endif
