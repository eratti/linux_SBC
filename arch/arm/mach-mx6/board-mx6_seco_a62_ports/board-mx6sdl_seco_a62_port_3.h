
//#ifndef _BOARD_MX6_SECO_A62_PORT_3_H
//#define _BOARD_MX6_SECO_A62_PORT_3_H

#include <mach/iomux-mx6dl.h>
#ifdef CONFIG_A62_PORT_3_CONF_1
// Conf 1 - CAN
static iomux_v3_cfg_t mx6sdl_seco_a62_port_3_conf_1_pads[] = {

        MX6DL_PAD_GPIO_7__CAN1_TXCAN,    // EXP_GPIO_6
        MX6DL_PAD_GPIO_8__CAN1_RXCAN,    // EXP_GPIO_7

};
#endif
#ifdef CONFIG_A62_PORT_3_CONF_2
// Conf 2 - GENERIC GPIO
static iomux_v3_cfg_t mx6sdl_seco_a62_port_3_conf_2_pads[] = {

        MX6DL_PAD_GPIO_7__GPIO_1_7,   // EXP_GPIO_6
        MX6DL_PAD_GPIO_8__GPIO_1_8,    // EXP_GPIO_7

};
#endif
