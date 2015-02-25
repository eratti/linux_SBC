
//#ifndef _BOARD_MX6_SECO_A62_PORT_9_H
//#define _BOARD_MX6_SECO_A62_PORT_9_H

#include <mach/iomux-mx6dl.h>

#ifdef CONFIG_A62_PORT_9_CONF_1
// Conf 1 - UART5
static iomux_v3_cfg_t mx6sdl_seco_a62_port_9_conf_1_pads[] = {

        MX6DL_PAD_CSI0_DAT14__UART5_TXD,
        MX6DL_PAD_CSI0_DAT15__UART5_RXD,

};
#endif
#ifdef CONFIG_A62_PORT_9_CONF_2
// Conf 2 - GENERIC GPIO
static iomux_v3_cfg_t mx6sdl_seco_a62_port_9_conf_2_pads[] = {

        MX6DL_PAD_CSI0_DAT14__GPIO_6_0,
        MX6DL_PAD_CSI0_DAT15__GPIO_6_1,

};
#endif
