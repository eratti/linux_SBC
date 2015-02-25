#ifndef __LINUX_SECO_XR28V382_H
#define __LINUX_SECO_XR28V382_H


enum XR28V382_DEVICE {
  	UARTA = 0x00,
	UARTB = 0x01,
	WDT   = 0x08,
};


/* platform data for the Seco EXAR XR28V382 with LPC interface */
struct seco_xr28v382_platform_data {
	enum XR28V382_DEVICE    *devices;
	int                     num_devices;
};


int superio_readw (unsigned long addr, int offset);

void superio_writew (unsigned long addr, int offset, int value);




#endif  /* __LINUX_SECO_XR28V382_H */
