
//#ifndef _BOARD_MX6_SECO_A62_PORT_7_H
//#define _BOARD_MX6_SECO_A62_PORT_7_H

#include <mach/iomux-mx6dl.h>
#ifdef CONFIG_A62_PORT_7_CONF_1
// Conf 1 - I2C3
static iomux_v3_cfg_t mx6sdl_seco_a62_port_7_conf_1_pads[] = {

        MX6DL_PAD_GPIO_16__I2C3_SDA,             // EXP_GPIO_21
        MX6DL_PAD_GPIO_3__I2C3_SCL,              // EXP_GPIO_22

};
#endif
#ifdef CONFIG_A62_PORT_7_CONF_2
// Conf 2 - GENERIC GPIO
static iomux_v3_cfg_t mx6sdl_seco_a62_port_7_conf_2_pads[] = {

        MX6DL_PAD_GPIO_16__GPIO_7_11,            // EXP_GPIO_21
        MX6DL_PAD_GPIO_3__GPIO_1_3,              // EXP_GPIO_22

};
#endif
