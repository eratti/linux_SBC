#include <linux/module.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/suspend.h>
#include <linux/regulator/machine.h>
#include <linux/proc_fs.h>
#include <linux/iram_alloc.h>
#include <linux/fsl_devices.h>
#include <asm/mach-types.h>
#include <asm/cacheflush.h>
#include <asm/tlb.h>
#include <asm/delay.h>
#include <asm/mach/map.h>
#include <mach/hardware.h>
#include <mach/imx-pm.h>
#include <mach/arc_otg.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/hardware/gic.h>

#include "board-mx6_seco_a62_all_ports.h"

/***********************************************************************
 *                    	   J8 Ports Configuration                      *
 ***********************************************************************/
// PORT 1
static void __init board_mx6_seco_a62_port_1_config(void)
{
#ifdef CONFIG_A62_PORT_1_CONF_1
	if (cpu_is_mx6q()){
                mxc_iomux_v3_setup_multiple_pads(mx6qd_seco_a62_port_1_conf_1_pads,
                        ARRAY_SIZE(mx6qd_seco_a62_port_1_conf_1_pads));
        }
        else if (cpu_is_mx6dl()) {
                mxc_iomux_v3_setup_multiple_pads(mx6sdl_seco_a62_port_1_conf_1_pads,
                        ARRAY_SIZE(mx6sdl_seco_a62_port_1_conf_1_pads));

        }	
#endif
}

// PORT 2 
static void __init board_mx6_seco_a62_port_2_config(void)
{
#ifdef CONFIG_A62_PORT_2_CONF_1
        if (cpu_is_mx6q()){
                mxc_iomux_v3_setup_multiple_pads(mx6qd_seco_a62_port_2_conf_1_pads,
                        ARRAY_SIZE(mx6qd_seco_a62_port_2_conf_1_pads));
        }
        else if (cpu_is_mx6dl()) {
                mxc_iomux_v3_setup_multiple_pads(mx6sdl_seco_a62_port_2_conf_1_pads,
                        ARRAY_SIZE(mx6sdl_seco_a62_port_2_conf_1_pads));

        }
#endif

#ifdef CONFIG_A62_PORT_2_CONF_2
        if (cpu_is_mx6q()){
                mxc_iomux_v3_setup_multiple_pads(mx6qd_seco_a62_port_2_conf_2_pads,
                        ARRAY_SIZE(mx6qd_seco_a62_port_2_conf_2_pads));
        }
        else if (cpu_is_mx6dl()) {
                mxc_iomux_v3_setup_multiple_pads(mx6sdl_seco_a62_port_2_conf_2_pads,
                        ARRAY_SIZE(mx6sdl_seco_a62_port_2_conf_2_pads));

        }
#endif

}

// PORT 3 
static void __init board_mx6_seco_a62_port_3_config(void)
{
#ifdef CONFIG_A62_PORT_3_CONF_1
        if (cpu_is_mx6q()){
                mxc_iomux_v3_setup_multiple_pads(mx6qd_seco_a62_port_3_conf_1_pads,
                        ARRAY_SIZE(mx6qd_seco_a62_port_3_conf_1_pads));
        }
        else if (cpu_is_mx6dl()) {
                mxc_iomux_v3_setup_multiple_pads(mx6sdl_seco_a62_port_3_conf_1_pads,
                        ARRAY_SIZE(mx6sdl_seco_a62_port_3_conf_1_pads));

        }
#endif

#ifdef CONFIG_A62_PORT_3_CONF_2
        if (cpu_is_mx6q()){
                mxc_iomux_v3_setup_multiple_pads(mx6qd_seco_a62_port_3_conf_2_pads,
                        ARRAY_SIZE(mx6qd_seco_a62_port_3_conf_2_pads));
        }
        else if (cpu_is_mx6dl()) {
                mxc_iomux_v3_setup_multiple_pads(mx6sdl_seco_a62_port_3_conf_2_pads,
                        ARRAY_SIZE(mx6sdl_seco_a62_port_3_conf_2_pads));

        }
#endif

}

// PORT 4 
static void __init board_mx6_seco_a62_port_4_config(void)
{
#ifdef CONFIG_A62_PORT_4_CONF_1
        if (cpu_is_mx6q()){
                mxc_iomux_v3_setup_multiple_pads(mx6qd_seco_a62_port_4_conf_1_pads,
                        ARRAY_SIZE(mx6qd_seco_a62_port_4_conf_1_pads));
        }
        else if (cpu_is_mx6dl()) {
                mxc_iomux_v3_setup_multiple_pads(mx6sdl_seco_a62_port_4_conf_1_pads,
                        ARRAY_SIZE(mx6sdl_seco_a62_port_4_conf_1_pads));

        }
#endif

#ifdef CONFIG_A62_PORT_4_CONF_2
        if (cpu_is_mx6q()){
                mxc_iomux_v3_setup_multiple_pads(mx6qd_seco_a62_port_4_conf_2_pads,
                        ARRAY_SIZE(mx6qd_seco_a62_port_4_conf_2_pads));
        }
        else if (cpu_is_mx6dl()) {
                mxc_iomux_v3_setup_multiple_pads(mx6sdl_seco_a62_port_4_conf_2_pads,
                        ARRAY_SIZE(mx6sdl_seco_a62_port_4_conf_2_pads));

        }
#endif

}

// PORT 5
static void __init board_mx6_seco_a62_port_5_config(void)
{
#ifdef CONFIG_A62_PORT_5_CONF_1
        if (cpu_is_mx6q()){
                mxc_iomux_v3_setup_multiple_pads(mx6qd_seco_a62_port_5_conf_1_pads,
                        ARRAY_SIZE(mx6qd_seco_a62_port_5_conf_1_pads));
        }
        else if (cpu_is_mx6dl()) {
                mxc_iomux_v3_setup_multiple_pads(mx6sdl_seco_a62_port_5_conf_1_pads,
                        ARRAY_SIZE(mx6sdl_seco_a62_port_5_conf_1_pads));
        
        }
#endif  

#ifdef CONFIG_A62_PORT_5_CONF_2
        if (cpu_is_mx6q()){
                mxc_iomux_v3_setup_multiple_pads(mx6qd_seco_a62_port_5_conf_2_pads,
                        ARRAY_SIZE(mx6qd_seco_a62_port_5_conf_2_pads));
        }
        else if (cpu_is_mx6dl()) {
                mxc_iomux_v3_setup_multiple_pads(mx6sdl_seco_a62_port_5_conf_2_pads,
                        ARRAY_SIZE(mx6sdl_seco_a62_port_5_conf_2_pads));

        }
#endif

#ifdef CONFIG_A62_PORT_5_CONF_3
        if (cpu_is_mx6q()){
                mxc_iomux_v3_setup_multiple_pads(mx6qd_seco_a62_port_5_conf_3_pads,
                        ARRAY_SIZE(mx6qd_seco_a62_port_5_conf_3_pads));
        }
        else if (cpu_is_mx6dl()) {
                mxc_iomux_v3_setup_multiple_pads(mx6sdl_seco_a62_port_5_conf_3_pads,
                        ARRAY_SIZE(mx6sdl_seco_a62_port_5_conf_3_pads));

        }
#endif

}

// PORT 6
static void __init board_mx6_seco_a62_port_6_config(void)
{
#ifdef CONFIG_A62_PORT_6_CONF_1
        if (cpu_is_mx6q()){
                mxc_iomux_v3_setup_multiple_pads(mx6qd_seco_a62_port_6_conf_1_pads,
                        ARRAY_SIZE(mx6qd_seco_a62_port_6_conf_1_pads));
        }
        else if (cpu_is_mx6dl()) {
                mxc_iomux_v3_setup_multiple_pads(mx6sdl_seco_a62_port_6_conf_1_pads,
                        ARRAY_SIZE(mx6sdl_seco_a62_port_6_conf_1_pads));
        
        }
#endif

#ifdef CONFIG_A62_PORT_6_CONF_2
        if (cpu_is_mx6q()){
                mxc_iomux_v3_setup_multiple_pads(mx6qd_seco_a62_port_6_conf_2_pads,
                        ARRAY_SIZE(mx6qd_seco_a62_port_6_conf_2_pads));
        }
        else if (cpu_is_mx6dl()) {
                mxc_iomux_v3_setup_multiple_pads(mx6sdl_seco_a62_port_6_conf_2_pads,
                        ARRAY_SIZE(mx6sdl_seco_a62_port_6_conf_2_pads));

        }
#endif

#ifdef CONFIG_A62_PORT_6_CONF_3
        if (cpu_is_mx6q()){
                mxc_iomux_v3_setup_multiple_pads(mx6qd_seco_a62_port_6_conf_3_pads,
                        ARRAY_SIZE(mx6qd_seco_a62_port_6_conf_3_pads));
        }
        else if (cpu_is_mx6dl()) {
                mxc_iomux_v3_setup_multiple_pads(mx6sdl_seco_a62_port_6_conf_3_pads,
                        ARRAY_SIZE(mx6sdl_seco_a62_port_6_conf_3_pads));

        }
#endif

}

// PORT 7
static void __init board_mx6_seco_a62_port_7_config(void)
{
#ifdef CONFIG_A62_PORT_7_CONF_1
        if (cpu_is_mx6q()){
                mxc_iomux_v3_setup_multiple_pads(mx6qd_seco_a62_port_7_conf_1_pads,
                        ARRAY_SIZE(mx6qd_seco_a62_port_7_conf_1_pads));
        }
        else if (cpu_is_mx6dl()) {
                mxc_iomux_v3_setup_multiple_pads(mx6sdl_seco_a62_port_7_conf_1_pads,
                        ARRAY_SIZE(mx6sdl_seco_a62_port_7_conf_1_pads));

        }
#endif

#ifdef CONFIG_A62_PORT_7_CONF_2
        if (cpu_is_mx6q()){
                mxc_iomux_v3_setup_multiple_pads(mx6qd_seco_a62_port_7_conf_2_pads,
                        ARRAY_SIZE(mx6qd_seco_a62_port_7_conf_2_pads));
        }
        else if (cpu_is_mx6dl()) {
                mxc_iomux_v3_setup_multiple_pads(mx6sdl_seco_a62_port_7_conf_2_pads,
                        ARRAY_SIZE(mx6sdl_seco_a62_port_7_conf_2_pads));

        }
#endif

}

// PORT 8
static void __init board_mx6_seco_a62_port_8_config(void)
{
#ifdef CONFIG_A62_PORT_8_CONF_1
        if (cpu_is_mx6q()){
                mxc_iomux_v3_setup_multiple_pads(mx6qd_seco_a62_port_8_conf_1_pads,
                        ARRAY_SIZE(mx6qd_seco_a62_port_8_conf_1_pads));
        }
        else if (cpu_is_mx6dl()) {
                mxc_iomux_v3_setup_multiple_pads(mx6sdl_seco_a62_port_8_conf_1_pads,
                        ARRAY_SIZE(mx6sdl_seco_a62_port_8_conf_1_pads));

        }
#endif

#ifdef CONFIG_A62_PORT_8_CONF_2
        if (cpu_is_mx6q()){
                mxc_iomux_v3_setup_multiple_pads(mx6qd_seco_a62_port_8_conf_2_pads,
                        ARRAY_SIZE(mx6qd_seco_a62_port_8_conf_2_pads));
        }
        else if (cpu_is_mx6dl()) {
                mxc_iomux_v3_setup_multiple_pads(mx6sdl_seco_a62_port_8_conf_2_pads,
                        ARRAY_SIZE(mx6sdl_seco_a62_port_8_conf_2_pads));

        }
#endif

}

// PORT 9
static void __init board_mx6_seco_a62_port_9_config(void)
{
#ifdef CONFIG_A62_PORT_9_CONF_1
        if (cpu_is_mx6q()){
                mxc_iomux_v3_setup_multiple_pads(mx6qd_seco_a62_port_9_conf_1_pads,
                        ARRAY_SIZE(mx6qd_seco_a62_port_9_conf_1_pads));
        }
        else if (cpu_is_mx6dl()) {
                mxc_iomux_v3_setup_multiple_pads(mx6sdl_seco_a62_port_9_conf_1_pads,
                        ARRAY_SIZE(mx6sdl_seco_a62_port_9_conf_1_pads));
        
        }
#endif  

#ifdef CONFIG_A62_PORT_9_CONF_2
        if (cpu_is_mx6q()){
                mxc_iomux_v3_setup_multiple_pads(mx6qd_seco_a62_port_9_conf_2_pads,
                        ARRAY_SIZE(mx6qd_seco_a62_port_9_conf_2_pads));
        }
        else if (cpu_is_mx6dl()) {
                mxc_iomux_v3_setup_multiple_pads(mx6sdl_seco_a62_port_9_conf_2_pads,
                        ARRAY_SIZE(mx6sdl_seco_a62_port_9_conf_2_pads));

        }
#endif

}

void __init board_mx6_seco_a62_ports_config_init(void)
{

        board_mx6_seco_a62_port_1_config(); //Port 1
        board_mx6_seco_a62_port_2_config(); //Port 2
        board_mx6_seco_a62_port_3_config(); //Port 3
	board_mx6_seco_a62_port_4_config(); //Port 4
	board_mx6_seco_a62_port_5_config(); //Port 5
	board_mx6_seco_a62_port_6_config(); //Port 6
	board_mx6_seco_a62_port_7_config(); //Port 7
	board_mx6_seco_a62_port_8_config(); //Port 8
	board_mx6_seco_a62_port_9_config(); //Port 9

}
