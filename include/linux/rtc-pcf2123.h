/*
 * rtc-pcf2123.h - Registers definition for the rtc-pcf2123 RTC.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2013, Seco s.r.l.
 */
#ifndef __LINUX_PCF2123_RTC_H
#define __LINUX_PCF2123_RTC_H


int pcf2123_rtc_set_time(struct device *dev, struct rtc_time *tm);
int pcf2123_rtc_read_time(struct device *dev, struct rtc_time *tm);

int pcf2123_rtc_set_alrm (struct device *dev, struct rtc_wkalrm *alarm);
int pcf2123_rtc_read_alrm (struct device *dev, struct rtc_wkalrm *alarm);

#endif /* __LINUX_PCF2123_RTC_H */
