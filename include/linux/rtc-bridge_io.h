#include <linux/ioctl.h>

struct alrm_pcf {
	int min;
	int hr;
	int wday;
	int mday;
	int enable;
};

#define RTC_BRIDGE_IOCTL_ALM_READ	_IOR('o', 1, struct alrm_pcf)
#define RTC_BRIDGE_IOCTL_ALM_WRITE	_IOWR('o', 2, struct alrm_pcf)
