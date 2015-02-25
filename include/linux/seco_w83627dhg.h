#ifndef __LINUX_SECO_W83627DHG_H
#define __LINUX_SECO_W83627DHG_H

enum W83627DHG_P_DEVICE {
	FDC, PARALLEL_PORT, UARTA, UARTB, NONE, KEYBOARD_CRTL,
	SPI, GPIO6, WDTO_PLED, GPIO2, GPIO3, GPIO4, GPIO5, ACPI, HWMON, PECI_SST
};

/* platform data for the Seco Winbond w83627dhg with LPC interface */

struct seco_w83627dhg_platform_data {
	enum W83627DHG_P_DEVICE   *devices;
	int                       num_devices;
        unsigned                  gpio2_base;
        unsigned                  gpio3_base;
};



int superio_readw (unsigned long addr, int offset);

void superio_writew (unsigned long addr, int offset, int value);




#endif  /* __LINUX_SECO_W83627DHG_H */
