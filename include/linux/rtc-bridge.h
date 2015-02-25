/*
 * rtc-bridge.h - Registers definition for the rtc-bridge RTC.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2013, Seco s.r.l.
 */
#ifndef __LINUX_BRIDGE_RTC_H
#define __LINUX_BRIDGE_RTC_H

struct rtc_dev_share {

	struct device *dev_internal;
	struct device *dev_external;
	
};
/*extern struct rtc_func_share {

        int (*set_time_internal)(struct device *dev, struct rtc_time *tm);
        int (*read_time_internal)(struct device *dev, struct rtc_time *tm);
	int (*set_time_external)(struct device *dev, struct rtc_time *tm);
        int (*read_time_external)(struct device *dev, struct rtc_time *tm);
        
};*/


#endif /* __LINUX_BRIDGE_RTC_H */
