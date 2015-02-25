/*
 * rtc-snvs.h - Registers definition for the rtc-snvs RTC.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2013, Seco s.r.l.
 */
#ifndef __LINUX_SNVS_RTC_H
#define __LINUX_SNVS_RTC_H


int snvs_rtc_read_time(struct device *dev, struct rtc_time *tm);
int snvs_rtc_set_time(struct device *dev, struct rtc_time *tm);


#endif /* __LINUX_SNVS_RTC_H */
