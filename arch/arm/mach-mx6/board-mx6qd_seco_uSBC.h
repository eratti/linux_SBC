
//#ifndef _BOARD_MX6_SECO_uQ7_H
//#define _BOARD_MX6_SECO_uQ7_H

#include <mach/iomux-mx6q.h>


static iomux_v3_cfg_t mx6qd_seco_uSBC_pads[] = {

    	
        /* AUDMUX */
	MX6Q_PAD_DI0_PIN2__AUDMUX_AUD6_TXD,  // for AUDIO AUDMUX port 6
	MX6Q_PAD_DI0_PIN3__AUDMUX_AUD6_TXFS, // for AUDIO AUDMUX port 6
	MX6Q_PAD_DI0_PIN4__AUDMUX_AUD6_RXD,  // for AUDIO AUDMUX port 6
	MX6Q_PAD_DI0_PIN15__AUDMUX_AUD6_TXC, // for AUDIO AUDMUX port 6
	MX6Q_PAD_SD2_CMD__GPIO_1_11,	     // for AUDIO RESET
#ifdef CONFIG_USBC_DEBUG_UART5
	MX6Q_PAD_EIM_D26__GPIO_3_26,
	MX6Q_PAD_EIM_D27__GPIO_3_27,	
#else	
	/* UART2 for debug */
	MX6Q_PAD_EIM_D26__UART2_TXD,		/*DUART_TXD*/ // for UART2
	MX6Q_PAD_EIM_D27__UART2_RXD,		/*DUART_RXD*/ // for UART2
#endif
	//UART5
	MX6Q_PAD_KEY_COL1__UART5_TXD,   	// for UART5
	MX6Q_PAD_KEY_ROW1__UART5_RXD,		// for UART5
	MX6Q_PAD_KEY_COL4__UART5_RTS,   	// for UART5
	MX6Q_PAD_KEY_ROW4__UART5_CTS,		// for UART5
        
    	/* GENERIC GPIO INTERFACE*/
	MX6Q_PAD_SD1_CMD__GPIO_1_18,		/*GPIO_1_18*/
	MX6Q_PAD_SD1_CLK__GPIO_1_20,		/*GPIO_1_20*/
	MX6Q_PAD_SD1_DAT0__GPIO_1_16,		/*GPIO_1_16*/
	MX6Q_PAD_SD1_DAT1__GPIO_1_17,		/*GPIO_1_17*/
	MX6Q_PAD_SD1_DAT2__GPIO_1_19,		/*GPIO_1_19*/
	MX6Q_PAD_SD1_DAT3__GPIO_1_21,		/*GPIO_1_21*/
	MX6Q_PAD_NANDF_CS2__GPIO_6_15,		/*GPIO_6_15*/
	MX6Q_PAD_NANDF_CS1__GPIO_6_14,		/*GPIO_6_14*/

#if defined(CONFIG_uSBC_EMMC_INTERFACE)
	/* USDHC3 -> eMMC ONBOARD */
	MX6Q_PAD_SD3_CLK__USDHC3_CLK_50MHZ,			/*eMMC_CMD*/     
	MX6Q_PAD_SD3_CMD__USDHC3_CMD_50MHZ,			/*eMMC_CLK*/     
	MX6Q_PAD_SD3_DAT0__USDHC3_DAT0_50MHZ,		/*eMMC_DAT0*/
	MX6Q_PAD_SD3_DAT1__USDHC3_DAT1_50MHZ,		/*eMMC_DAT1*/
	MX6Q_PAD_SD3_DAT2__USDHC3_DAT2_50MHZ,		/*eMMC_DAT2*/
	MX6Q_PAD_SD3_DAT3__USDHC3_DAT3_50MHZ,		/*eMMC_DAT3*/
	MX6Q_PAD_SD3_DAT4__USDHC3_DAT4_50MHZ,		/*eMMC_DAT4*/
	MX6Q_PAD_SD3_DAT5__USDHC3_DAT5_50MHZ,		/*eMMC_DAT5*/
	MX6Q_PAD_SD3_DAT6__USDHC3_DAT6_50MHZ,		/*eMMC_DAT6*/ 
	MX6Q_PAD_SD3_DAT7__USDHC3_DAT7_50MHZ,		/*eMMC_DAT7*/
	MX6Q_PAD_SD3_RST__USDHC3_RST,				/*eMMC_RESETN*/   	
#endif
	
	/* GPIO */
	MX6Q_PAD_NANDF_D4__GPIO_2_4,			//   gpio generic
	MX6Q_PAD_NANDF_D7__GPIO_2_7,			//   gpio generic
	MX6Q_PAD_NANDF_WP_B__GPIO_6_9,			//   gpio generic
	MX6Q_PAD_GPIO_18__GPIO_7_13,			//   gpio generic
	MX6Q_PAD_GPIO_19__GPIO_4_5,				//   gpio generic
	//MX6Q_PAD_GPIO_2__GPIO_1_2,				//   gpio generic
	//MX6Q_PAD_GPIO_4__GPIO_1_4,				//   gpio generic
	MX6Q_PAD_GPIO_6__GPIO_1_6,				//   gpio generic
	MX6Q_PAD_GPIO_7__GPIO_1_7,				//   gpio generic
	MX6Q_PAD_GPIO_8__GPIO_1_8,				//   gpio generic
	/* LVDS */
    	MX6Q_PAD_GPIO_2__GPIO_1_2,
    	MX6Q_PAD_GPIO_4__GPIO_1_4,

	/* TSC2006 PEN IRQ */
	MX6Q_PAD_GPIO_3__GPIO_1_3,
	
    /* ECSPI1 */
//	MX6Q_PAD_EIM_D17__ECSPI1_MISO,      		//   - to SPI
//	MX6Q_PAD_EIM_D18__ECSPI1_MOSI,      		//   - to SPI
//	MX6Q_PAD_EIM_D16__ECSPI1_SCLK,      		//   - to SPI
//	MX6Q_PAD_EIM_D19__GPIO_3_19,	/*SS1*/      	//   - to SPI  CS1
//	MX6Q_PAD_EIM_D24__GPIO_3_24,			//   - to SPI  CS2
//	MX6Q_PAD_EIM_D25__GPIO_3_25,			//   - to SPI  CS3
        
    /* ENET */
	MX6Q_PAD_ENET_MDIO__ENET_MDIO,			//   - to ENET
	MX6Q_PAD_ENET_MDC__ENET_MDC,			//   - to ENET
	MX6Q_PAD_RGMII_TXC__ENET_RGMII_TXC,		//   - to ENET
	MX6Q_PAD_RGMII_TD0__ENET_RGMII_TD0,		//   - to ENET
	MX6Q_PAD_RGMII_TD1__ENET_RGMII_TD1,		//   - to ENET
	MX6Q_PAD_RGMII_TD2__ENET_RGMII_TD2,		//   - to ENET
	MX6Q_PAD_RGMII_TD3__ENET_RGMII_TD3,		//   - to ENET
	MX6Q_PAD_RGMII_TX_CTL__ENET_RGMII_TX_CTL,	//   - to ENET
	MX6Q_PAD_ENET_REF_CLK__ENET_TX_CLK,		//   - to ENET
	MX6Q_PAD_RGMII_RXC__ENET_RGMII_RXC,		//   - to ENET
	MX6Q_PAD_RGMII_RD0__ENET_RGMII_RD0,		//   - to ENET
	MX6Q_PAD_RGMII_RD1__ENET_RGMII_RD1,		//   - to ENET
	MX6Q_PAD_RGMII_RD2__ENET_RGMII_RD2,		//   - to ENET
	MX6Q_PAD_RGMII_RD3__ENET_RGMII_RD3,		//   - to ENET
	MX6Q_PAD_RGMII_RX_CTL__ENET_RGMII_RX_CTL,	//   - to ENET
	MX6Q_PAD_ENET_TX_EN__GPIO_1_28,			/* Micrel RGMII Phy Interrupt */		//   - to ENET
	MX6Q_PAD_EIM_D23__GPIO_3_23,			/* RGMII reset */ //   
        
	/* SATA */
	MX6Q_PAD_NANDF_CS0__GPIO_6_11,	//   - for SATA led activity

    /* USBOTG ID pin */
	MX6Q_PAD_ENET_RX_ER__ANATOP_USBOTG_ID,			//  
    MX6Q_PAD_GPIO_17__GPIO_7_12,		/* USB Hub Reset */
	MX6Q_PAD_EIM_D22__GPIO_3_22,		// for OTG power enable

	/* USB OC pin */
	MX6Q_PAD_EIM_D30__USBOH3_USBH1_OC,	// to USBH1_OC Over Current
        
	/* PWM1 */
    MX6Q_PAD_GPIO_9__PWM1_PWMO,       	/* LVDS CTRL */  
    MX6Q_PAD_DISP0_DAT9__PWM2_PWMO,   
                
    /*I2C BUS*/
	
	/* I2C1 */
	MX6Q_PAD_EIM_D21__I2C1_SCL,		/*I2C1_SCL - SMB*/ //   - for I2C
	MX6Q_PAD_EIM_D28__I2C1_SDA,		/*I2C1_SDA - SMB*/ //   - for I2C

	/* I2C2 */
	MX6Q_PAD_KEY_COL3__I2C2_SCL,     	/*I2C2_SCL*/
    MX6Q_PAD_KEY_ROW3__I2C2_SDA,    	/*I2C2_SDA*/		
	
	/* I2C3 */
	MX6Q_PAD_GPIO_5__I2C3_SCL,		/*I2C3_SCL*/ //   - for I2C3
	MX6Q_PAD_GPIO_6__I2C3_SDA,		/*I2C3_SDA*/ //   - for I2C3

};

static iomux_v3_cfg_t mx6qd_seco_uSBC_pads_revA[] = {

	 /* ECSPI1 */
        MX6Q_PAD_EIM_D17__ECSPI1_MISO,                  //   - to SPI
        MX6Q_PAD_EIM_D18__ECSPI1_MOSI,                  //   - to SPI
        MX6Q_PAD_EIM_D16__ECSPI1_SCLK,                  //   - to SPI
        MX6Q_PAD_EIM_D19__GPIO_3_19,    /*SS1*/         //   - to SPI  CS1

};


static iomux_v3_cfg_t mx6qd_seco_uSBC_pads_revB[] = {

	 /* CSPI1 */
        MX6Q_PAD_EIM_D17__ECSPI1_MISO,                  //   - to SPI1
        MX6Q_PAD_EIM_D18__ECSPI1_MOSI,                  //   - to SPI1
        MX6Q_PAD_EIM_D16__ECSPI1_SCLK,                  //   - to SPI1
        MX6Q_PAD_EIM_D19__GPIO_3_19,        /*SS1*/     //   - to SPI2  CS1
         /* CSPI2 */
        //MX6Q_PAD_DISP0_DAT17__ECSPI2_MISO,              //   - to SPI2
        //MX6Q_PAD_DISP0_DAT16__ECSPI2_MOSI,              //   - to SPI2
        //MX6Q_PAD_DISP0_DAT19__ECSPI2_SCLK,              //   - to SPI2
        //MX6Q_PAD_DISP0_DAT18__GPIO_5_12,    /*SS2*/     //   - to SPI2  CS2
	
	  /* CSPI3 */
        MX6Q_PAD_DISP0_DAT2__ECSPI3_MISO,              //   - to SPI3
        MX6Q_PAD_DISP0_DAT1__ECSPI3_MOSI,              //   - to SPI3
        MX6Q_PAD_DISP0_DAT0__ECSPI3_SCLK,              //   - to SPI3
        MX6Q_PAD_DISP0_DAT3__GPIO_4_24,    /*SS3*/     //   - to SPI3  CS3

	 /* CAN1  */
        MX6Q_PAD_KEY_ROW2__CAN1_RXCAN,          	// for can bus
        MX6Q_PAD_KEY_COL2__CAN1_TXCAN,          	// for can bus	
	MX6Q_PAD_GPIO_0__GPIO_1_0,			// for select this can output 
	MX6Q_PAD_GPIO_1__GPIO_1_1,			// for select termination 
};

static iomux_v3_cfg_t mx6qd_seco_uSBC_pads_revC[] = {

         /* CSPI1 */
        MX6Q_PAD_EIM_D17__ECSPI1_MISO,                  //   - to SPI1
        MX6Q_PAD_EIM_D18__ECSPI1_MOSI,                  //   - to SPI1
        MX6Q_PAD_EIM_D16__ECSPI1_SCLK,                  //   - to SPI1
        MX6Q_PAD_EIM_D19__GPIO_3_19,        /*SS1*/     //   - to SPI2  CS1
         /* CSPI2 */
        //MX6Q_PAD_DISP0_DAT17__ECSPI2_MISO,              //   - to SPI2
        //MX6Q_PAD_DISP0_DAT16__ECSPI2_MOSI,              //   - to SPI2
        //MX6Q_PAD_DISP0_DAT19__ECSPI2_SCLK,              //   - to SPI2
        //MX6Q_PAD_DISP0_DAT18__GPIO_5_12,    /*SS2*/     //   - to SPI2  CS2

          /* CSPI3 */
        MX6Q_PAD_DISP0_DAT2__ECSPI3_MISO,              //   - to SPI3
        MX6Q_PAD_DISP0_DAT1__ECSPI3_MOSI,              //   - to SPI3
        MX6Q_PAD_DISP0_DAT0__ECSPI3_SCLK,              //   - to SPI3
        MX6Q_PAD_DISP0_DAT3__GPIO_4_24,    /*SS3*/     //   - to SPI3  CS3

         /* CAN1  */
        MX6Q_PAD_KEY_ROW2__CAN1_RXCAN,                  // for can bus
        MX6Q_PAD_KEY_COL2__CAN1_TXCAN,                  // for can bus  
        MX6Q_PAD_GPIO_0__GPIO_1_0,                      // for select this can output 
        MX6Q_PAD_GPIO_1__GPIO_1_1,                      // for select termination 
#if (defined(CONFIG_MXC_CAMERA_OV5640_MIPI) || defined(CONFIG_MXC_CAMERA_OV5640_MIPI_MODULE)) && (defined(CONFIG_Q7_CAM))
	/* CAM */
	MX6Q_PAD_CSI0_DAT18__GPIO_6_4,			// POWER
	MX6Q_PAD_CSI0_DAT19__GPIO_6_5,			// CAM RST
	MX6Q_PAD_GPIO_3__CCM_CLKO2,			// CLK
#endif
	/* HDMI CHECK */
	MX6Q_PAD_EIM_A25__HDMI_TX_CEC_LINE,
	
};

static iomux_v3_cfg_t mx6q_seco_q7_csi0_sensor_pads[] = {
};

#define MX6Q_USDHC_PAD_SETTING(id, speed)	\
mx6q_sd##id##_##speed##mhz[] = {		\
	MX6Q_PAD_SD##id##_CLK__USDHC##id##_CLK_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_CMD__USDHC##id##_CMD_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT0__USDHC##id##_DAT0_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT1__USDHC##id##_DAT1_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT2__USDHC##id##_DAT2_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT3__USDHC##id##_DAT3_##speed##MHZ,	\
}


