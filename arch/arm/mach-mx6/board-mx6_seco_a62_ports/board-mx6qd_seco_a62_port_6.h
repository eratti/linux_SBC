
//#ifndef _BOARD_MX6_SECO_A62_PORT_6_H
//#define _BOARD_MX6_SECO_A62_PORT_6_H

#include <mach/iomux-mx6q.h>
#ifdef CONFIG_A62_PORT_6_CONF_1
// Conf 1 - SDIO
static iomux_v3_cfg_t mx6qd_seco_a62_port_6_conf_1_pads[] = {

	MX6Q_PAD_SD1_CMD__USDHC1_CMD,		// EXP_GPIO_15
	MX6Q_PAD_SD1_CLK__USDHC1_CLK,		// EXP_GPIO_16
	MX6Q_PAD_SD1_DAT0__USDHC1_DAT0,		// EXP_GPIO_17
	MX6Q_PAD_SD1_DAT1__USDHC1_DAT1,		// EXP_GPIO_18
	MX6Q_PAD_SD1_DAT2__USDHC1_DAT2,		// EXP_GPIO_19
	MX6Q_PAD_SD1_DAT3__USDHC1_DAT3,		// EXP_GPIO_20

};
#endif
#ifdef CONFIG_A62_PORT_6_CONF_2
// Conf 2 - PWM
static iomux_v3_cfg_t mx6qd_seco_a62_port_6_conf_2_pads[] = {

	MX6Q_PAD_SD1_CMD__PWM4_PWMO,            // EXP_GPIO_15
        MX6Q_PAD_SD1_CLK__GPIO_1_20,            // EXP_GPIO_16
        MX6Q_PAD_SD1_DAT0__GPIO_1_16,           // EXP_GPIO_17
        MX6Q_PAD_SD1_DAT1__PWM3_PWMO,           // EXP_GPIO_18
        MX6Q_PAD_SD1_DAT2__PWM2_PWMO,           // EXP_GPIO_19
        MX6Q_PAD_SD1_DAT3__GPIO_1_21,           // EXP_GPIO_20

};
#endif
#ifdef CONFIG_A62_PORT_6_CONF_3
// Conf 3 - GENERIC GPIO
static iomux_v3_cfg_t mx6qd_seco_a62_port_6_conf_3_pads[] = {

	MX6Q_PAD_SD1_CMD__GPIO_1_18,		// EXP_GPIO_15
	MX6Q_PAD_SD1_CLK__GPIO_1_20,		// EXP_GPIO_16
	MX6Q_PAD_SD1_DAT0__GPIO_1_16,		// EXP_GPIO_17
	MX6Q_PAD_SD1_DAT1__GPIO_1_17,		// EXP_GPIO_18
        MX6Q_PAD_SD1_DAT2__GPIO_1_19,		// EXP_GPIO_19
        MX6Q_PAD_SD1_DAT3__GPIO_1_21,		// EXP_GPIO_20

};
#endif
