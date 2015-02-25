
//#ifndef _BOARD_MX6_SECO_A62_PORT_2_H
//#define _BOARD_MX6_SECO_A62_PORT_2_H

#include <mach/iomux-mx6dl.h>
#ifdef CONFIG_A62_PORT_2_CONF_1 
// Conf 1 - SPI 2
static iomux_v3_cfg_t mx6sdl_seco_a62_port_2_conf_1_pads[] = {

        MX6DL_PAD_EIM_OE__ECSPI2_MISO,   // EXP_GPIO_2
        MX6DL_PAD_EIM_CS0__ECSPI2_SCLK,    // EXP_GPIO_3
        MX6DL_PAD_EIM_CS1__ECSPI2_MOSI,  // EXP_GPIO_4
        MX6DL_PAD_EIM_D24__GPIO_3_24,  // EXP_GPIO_5

};
#endif
#ifdef CONFIG_A62_PORT_2_CONF_2
// Conf 2 - GENERIC GPIO
static iomux_v3_cfg_t mx6sdl_seco_a62_port_2_conf_2_pads[] = {

        MX6DL_PAD_EIM_OE__GPIO_2_25,   // EXP_GPIO_2
        MX6DL_PAD_EIM_CS0__GPIO_2_23,    // EXP_GPIO_3
        MX6DL_PAD_EIM_CS1__GPIO_2_24,  // EXP_GPIO_4
        MX6DL_PAD_EIM_D24__GPIO_3_24,  // EXP_GPIO_5

};
#endif
