
//#ifndef _BOARD_MX6_SECO_uQ7_H
//#define _BOARD_MX6_SECO_uQ7_H

#include <mach/iomux-mx6dl.h>


static iomux_v3_cfg_t mx6sdl_seco_uq7_pads[] = {
	
      /* AUDMUX */
	MX6DL_PAD_DI0_PIN2__AUDMUX_AUD6_TXD,  // for AUDIO AUDMUX port 6
	MX6DL_PAD_DI0_PIN3__AUDMUX_AUD6_TXFS, // for AUDIO AUDMUX port 6
	MX6DL_PAD_DI0_PIN4__AUDMUX_AUD6_RXD,  // for AUDIO AUDMUX port 6
	MX6DL_PAD_DI0_PIN15__AUDMUX_AUD6_TXC, // for AUDIO AUDMUX port 6
	MX6DL_PAD_SD2_CMD__GPIO_1_11,         // for AUDIO RESET

    /* CAN1  */
	MX6DL_PAD_KEY_ROW2__CAN1_RXCAN,		// for can bus
	MX6DL_PAD_KEY_COL2__CAN1_TXCAN,		// for can bus
        
	/* UART2 for debug */
	MX6DL_PAD_EIM_D26__UART2_TXD,		/*DUART_TXD*/ // for UART2
	MX6DL_PAD_EIM_D27__UART2_RXD,		/*DUART_RXD*/ // for UART2

	//UART4
	MX6DL_PAD_KEY_ROW0__UART4_RXD,		// for UART4
	MX6DL_PAD_KEY_COL0__UART4_TXD,		// for UART4
	//UART5
	MX6DL_PAD_KEY_COL1__UART5_TXD,   	// for UART5
	MX6DL_PAD_KEY_ROW1__UART5_RXD,		// for UART5
	MX6DL_PAD_KEY_COL4__UART5_RTS,   	// for UART5
	MX6DL_PAD_KEY_ROW4__UART5_CTS,		// for UART5
       
	/* GENERIC GPIO INTERFACE*/
        MX6DL_PAD_SD1_CMD__GPIO_1_18,            /*GPIO_1_18*/
        MX6DL_PAD_SD1_CLK__GPIO_1_20,            /*GPIO_1_20*/
        MX6DL_PAD_SD1_DAT0__GPIO_1_16,           /*GPIO_1_16*/
#ifndef CONFIG_Q7_PWM2
	MX6DL_PAD_DISP0_DAT9__GPIO_4_30,
#endif
#ifndef CONFIG_Q7_PWM3
        MX6DL_PAD_SD1_DAT1__GPIO_1_17,           /*GPIO_1_17*/
#endif
        MX6DL_PAD_SD1_DAT2__GPIO_1_19,           /*GPIO_1_19*/
        MX6DL_PAD_SD1_DAT3__GPIO_1_21,           /*GPIO_1_21*/
        MX6DL_PAD_NANDF_CS2__GPIO_6_15,          /*GPIO_6_15*/
        MX6DL_PAD_NANDF_CS1__GPIO_6_14,          /*GPIO_6_14*/
 
#if defined(CONFIG_Q7_EMMC_INTERFACE)
	/* USDHC3 -> eMMC ONBOARD */
	MX6DL_PAD_SD3_CLK__USDHC3_CLK_50MHZ,			/*eMMC_CMD*/     
	MX6DL_PAD_SD3_CMD__USDHC3_CMD_50MHZ,			/*eMMC_CLK*/     
	MX6DL_PAD_SD3_DAT0__USDHC3_DAT0_50MHZ,		/*eMMC_DAT0*/
	MX6DL_PAD_SD3_DAT1__USDHC3_DAT1_50MHZ,		/*eMMC_DAT1*/
	MX6DL_PAD_SD3_DAT2__USDHC3_DAT2_50MHZ,		/*eMMC_DAT2*/
	MX6DL_PAD_SD3_DAT3__USDHC3_DAT3_50MHZ,		/*eMMC_DAT3*/
	MX6DL_PAD_SD3_DAT4__USDHC3_DAT4_50MHZ,		/*eMMC_DAT4*/
	MX6DL_PAD_SD3_DAT5__USDHC3_DAT5_50MHZ,		/*eMMC_DAT5*/
	MX6DL_PAD_SD3_DAT6__USDHC3_DAT6_50MHZ,		/*eMMC_DAT6*/ 
	MX6DL_PAD_SD3_DAT7__USDHC3_DAT7_50MHZ,		/*eMMC_DAT7*/
	MX6DL_PAD_SD3_RST__USDHC3_RST,				/*eMMC_RESETN*/   	
#endif
	
#if defined(CONFIG_Q7_SD_INTERFACE)
	/* USDHC4  -> SD on Carrier-Board */
	MX6DL_PAD_SD4_CLK__USDHC4_CLK_50MHZ,			/*uSD_CLK*/
	MX6DL_PAD_SD4_CMD__USDHC4_CMD_50MHZ,			/*uSD_CMD*/
	MX6DL_PAD_SD4_DAT0__USDHC4_DAT0_50MHZ,		/*uSD_DAT0*/
	MX6DL_PAD_SD4_DAT1__USDHC4_DAT1_50MHZ,		/*uSD_DAT1*/
	MX6DL_PAD_SD4_DAT2__USDHC4_DAT2_50MHZ,		/*uSD_DAT2*/
	MX6DL_PAD_SD4_DAT3__USDHC4_DAT3_50MHZ,		/*uSD_DAT3*/
	MX6DL_PAD_SD4_DAT4__USDHC4_DAT4_50MHZ,		/*uSD_DAT4*/
	MX6DL_PAD_SD4_DAT5__USDHC4_DAT5_50MHZ,		/*uSD_DAT5*/
	MX6DL_PAD_SD4_DAT6__USDHC4_DAT6_50MHZ,		/*uSD_DAT6*/
	MX6DL_PAD_SD4_DAT7__USDHC4_DAT7_50MHZ,		/*uSD_DAT7*/
	MX6DL_PAD_NANDF_D6__GPIO_2_6,				/*uSD_CD*/ 
	MX6DL_PAD_CSI0_DATA_EN__GPIO_5_20,			/*uSD_WP*/
	MX6DL_PAD_NANDF_D5__GPIO_2_5,				/*gpio for power enable*/
#endif

	/* GPIO */
	MX6DL_PAD_NANDF_D4__GPIO_2_4,			//   gpio generic
	MX6DL_PAD_NANDF_D7__GPIO_2_7,			//   gpio generic
	MX6DL_PAD_NANDF_WP_B__GPIO_6_9,			//   gpio generic
	MX6DL_PAD_GPIO_18__GPIO_7_13,			//   gpio generic
	MX6DL_PAD_GPIO_19__GPIO_4_5,			//   gpio generic
	//MX6DL_PAD_GPIO_2__GPIO_1_2,				//   gpio generic
	//MX6DL_PAD_GPIO_4__GPIO_1_4,				//   gpio generic
	MX6DL_PAD_GPIO_6__GPIO_1_6,				//   gpio generic
	MX6DL_PAD_GPIO_7__GPIO_1_7,				//   gpio generic
	MX6DL_PAD_GPIO_8__GPIO_1_8,				//   gpio generic
	/* LVDS */
    MX6DL_PAD_GPIO_2__GPIO_1_2,
    MX6DL_PAD_GPIO_4__GPIO_1_4,

	/* TSC2006 PEN IRQ */
	MX6DL_PAD_GPIO_3__GPIO_1_3,
	
    /* ECSPI1 */
//	MX6DL_PAD_EIM_D17__ECSPI1_MISO,      		//   - to SPI
//	MX6DL_PAD_EIM_D18__ECSPI1_MOSI,      		//   - to SPI
//	MX6DL_PAD_EIM_D16__ECSPI1_SCLK,      		//   - to SPI
//	MX6DL_PAD_EIM_D19__GPIO_3_19,	/*SS1*/      	//   - to SPI  CS1
//	MX6DL_PAD_EIM_D24__GPIO_3_24,			//   - to SPI  CS2
//	MX6DL_PAD_EIM_D25__GPIO_3_25,			//   - to SPI  CS3
        
    /* ENET */
	MX6DL_PAD_ENET_MDIO__ENET_MDIO,			//   - to ENET
	MX6DL_PAD_ENET_MDC__ENET_MDC,			//   - to ENET
	MX6DL_PAD_RGMII_TXC__ENET_RGMII_TXC,		//   - to ENET
	MX6DL_PAD_RGMII_TD0__ENET_RGMII_TD0,		//   - to ENET
	MX6DL_PAD_RGMII_TD1__ENET_RGMII_TD1,		//   - to ENET
	MX6DL_PAD_RGMII_TD2__ENET_RGMII_TD2,		//   - to ENET
	MX6DL_PAD_RGMII_TD3__ENET_RGMII_TD3,		//   - to ENET
	MX6DL_PAD_RGMII_TX_CTL__ENET_RGMII_TX_CTL,	//   - to ENET
	MX6DL_PAD_ENET_REF_CLK__ENET_TX_CLK,		//   - to ENET
	MX6DL_PAD_RGMII_RXC__ENET_RGMII_RXC,		//   - to ENET
	MX6DL_PAD_RGMII_RD0__ENET_RGMII_RD0,		//   - to ENET
	MX6DL_PAD_RGMII_RD1__ENET_RGMII_RD1,		//   - to ENET
	MX6DL_PAD_RGMII_RD2__ENET_RGMII_RD2,		//   - to ENET
	MX6DL_PAD_RGMII_RD3__ENET_RGMII_RD3,		//   - to ENET
	MX6DL_PAD_RGMII_RX_CTL__ENET_RGMII_RX_CTL,	//   - to ENET
	MX6DL_PAD_ENET_TX_EN__GPIO_1_28,			/* Micrel RGMII Phy Interrupt */		//   - to ENET
	MX6DL_PAD_EIM_D23__GPIO_3_23,			/* RGMII reset */ //   
        
	/* SATA */
	MX6DL_PAD_NANDF_CS0__GPIO_6_11,	//   - for SATA led activity

	// HDMI - check
	MX6DL_PAD_EIM_A25__GPIO_5_2, 				
        
    /* USBOTG ID pin */

	MX6DL_PAD_GPIO_0__GPIO_1_0,			//  
   	MX6DL_PAD_GPIO_17__GPIO_7_12,		/* USB Hub Reset */
	MX6DL_PAD_EIM_D22__GPIO_3_22,		// for OTG power enable

	/* USB OC pin */
	MX6DL_PAD_EIM_D30__USBOH3_USBH1_OC,	// to USBH1_OC Over Current
        
	/* PWM1 */
    MX6DL_PAD_GPIO_9__PWM1_PWMO,       	/* LVDS CTRL */  

#ifdef CONFIG_Q7_PWM2
    MX6DL_PAD_DISP0_DAT9__PWM2_PWMO,       /* SPEKER */
#endif
#ifdef CONFIG_Q7_PWM3
    MX6DL_PAD_SD1_DAT1__PWM3_PWMO,      /* FAN CONTROLLER */
#endif
 
    /*I2C BUS*/
	
	/* I2C1 */
	MX6DL_PAD_EIM_D21__I2C1_SCL,		/*I2C1_SCL - SMB*/ //   - for I2C
	MX6DL_PAD_EIM_D28__I2C1_SDA,		/*I2C1_SDA - SMB*/ //   - for I2C

	/* I2C2 */
	MX6DL_PAD_KEY_COL3__I2C2_SCL,     	/*I2C2_SCL*/
    MX6DL_PAD_KEY_ROW3__I2C2_SDA,    	/*I2C2_SDA*/	

	/* I2C3 */
	MX6DL_PAD_GPIO_5__I2C3_SCL,		/*I2C3_SCL*/ //   - for I2C3
	MX6DL_PAD_GPIO_16__I2C3_SDA,		/*I2C3_SDA*/ //   - for I2C3
	
	 /* SEL_CAN */
        MX6DL_PAD_EIM_D20__GPIO_3_20,

};

static iomux_v3_cfg_t mx6sdl_seco_uq7_pads_revA_B[] = {

	/* ECSPI1 */
    MX6DL_PAD_EIM_D17__ECSPI1_MISO,                 //   - to SPI
    MX6DL_PAD_EIM_D18__ECSPI1_MOSI,                 //   - to SPI
    MX6DL_PAD_EIM_D16__ECSPI1_SCLK,                 //   - to SPI
    MX6DL_PAD_EIM_D19__GPIO_3_19,   /*SS1*/         //   - to SPI  CS1
    MX6DL_PAD_EIM_D24__GPIO_3_24,   /*SS2*/         //   - to SPI  CS2
    MX6DL_PAD_EIM_D25__GPIO_3_25,   /*SS3*/         //   - to SPI  CS3
};

static iomux_v3_cfg_t mx6sdl_seco_uq7_pads_revC[] = {

	/* CSPI1 */
    MX6DL_PAD_EIM_D17__ECSPI1_MISO,                  //   - to SPI1
    MX6DL_PAD_EIM_D18__ECSPI1_MOSI,                  //   - to SPI1
    MX6DL_PAD_EIM_D16__ECSPI1_SCLK,                  //   - to SPI1
    MX6DL_PAD_EIM_D19__GPIO_3_19,        /*SS1*/     //   - to SPI2  CS1
    /* CSPI2 */
    MX6DL_PAD_DISP0_DAT17__ECSPI2_MISO,              //   - to SPI2
    MX6DL_PAD_DISP0_DAT16__ECSPI2_MOSI,              //   - to SPI2
    MX6DL_PAD_DISP0_DAT19__ECSPI2_SCLK,              //   - to SPI2
    MX6DL_PAD_DISP0_DAT18__GPIO_5_12,    /*SS0*/     //   - to SPI2  CS0
    MX6DL_PAD_DISP0_DAT15__GPIO_5_9,     /*SS1*/     //   - to SPI2  CS1

    /* CSPI3 */
    MX6DL_PAD_DISP0_DAT2__ECSPI3_MISO,              //   - to SPI3
    MX6DL_PAD_DISP0_DAT1__ECSPI3_MOSI,              //   - to SPI3
    MX6DL_PAD_DISP0_DAT0__ECSPI3_SCLK,              //   - to SPI3
    MX6DL_PAD_DISP0_DAT3__GPIO_4_24,    /*SS3*/     //   - to SPI3  CS3

};

static iomux_v3_cfg_t mx6dl_seco_uq7_hdmi_ddc_pads[] = {
	MX6DL_PAD_KEY_COL3__HDMI_TX_DDC_SCL, /* HDMI DDC SCL */
	MX6DL_PAD_KEY_ROW3__HDMI_TX_DDC_SDA, /* HDMI DDC SDA */
};

static iomux_v3_cfg_t mx6dl_seco_uq7_i2c2_pads[] = {
	MX6DL_PAD_KEY_COL3__I2C2_SCL,	/* I2C2 SCL */
	MX6DL_PAD_KEY_ROW3__I2C2_SDA,	/* I2C2 SDA */
};

static iomux_v3_cfg_t mx6dl_seco_uq7_otg_id_up_pads = MX6DL_PAD_GPIO_1__USBOTG_ID_UP;
static iomux_v3_cfg_t mx6dl_seco_uq7_otg_id_down_pads = MX6DL_PAD_GPIO_1__USBOTG_ID_DOWN;

#define MX6DL_USDHC_PAD_SETTING(id, speed)	\
mx6dl_sd##id##_##speed##mhz[] = {		\
	MX6DL_PAD_SD##id##_CLK__USDHC##id##_CLK_##speed##MHZ,	\
	MX6DL_PAD_SD##id##_CMD__USDHC##id##_CMD_##speed##MHZ,	\
	MX6DL_PAD_SD##id##_DAT0__USDHC##id##_DAT0_##speed##MHZ,	\
	MX6DL_PAD_SD##id##_DAT1__USDHC##id##_DAT1_##speed##MHZ,	\
	MX6DL_PAD_SD##id##_DAT2__USDHC##id##_DAT2_##speed##MHZ,	\
	MX6DL_PAD_SD##id##_DAT3__USDHC##id##_DAT3_##speed##MHZ,	\
}


//#endif
