/*
 * An RTC Virtual driver for manage two RTC
 * Copyright 2013 Seco S.r.l.
 *
 * Author: Matrco Sandrelli <marco.sandrelli@seco.com>
 * Maintainers: <marco.sandrelli@seco.com>
 *
 * based on the rtc-snvs.c & rtc-pcf2123 driver in this same directory.
 *
 * Thanks to Davide Cardillo for
 * contributions to this driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Add in board platform definition the following function to create exported device
 * 
 * struct rtc_dev_share *dev_share;
 *
 * static void rtc_bridge_share_init (void) {
 *
 *       dev_share = kmalloc(sizeof(*dev_share),GFP_KERNEL);
 *
 * }
 *
 * and call the function inside initialization 
 *
 */

#include <linux/bcd.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/rtc.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/rtc-snvs.h>
#include <linux/rtc-pcf2123.h>
#include <linux/rtc-bridge.h>
#include <linux/rtc-bridge_io.h>


struct rtc_bridge_plat_data {
        struct rtc_device *rtc;
};

extern struct rtc_dev_share *dev_share;

/* 
   We use extern rtc_dev_share structure  
   but the prototype of read & write rtc function are:

   "static int function_read_or_write (struct device *dev, struct rtc_time *tm)"

   we don't use the passed struct device because we use struct device of 
   of snvs and pcf2123 device.      
*/

/*
   This function reads time from external rtc and set this time to internal RTC
   in this way we launch "hwclock -r" and synchronize linux time and snvs time.
   The internal and external RTC are synchronized.
*/

static int rtc_bridge_read_time(struct device *dev, struct rtc_time *tm){

	//printk("RTC BRIDGE read TIME!\n");
	int err = -1, err_internal = -1, retry = 5;

	if ( dev_share->dev_external != NULL ){

		err = pcf2123_rtc_read_time(dev_share->dev_external, tm);
			
			while ( err_internal != 0 &&  retry > 0 ){

				if ( dev_share->dev_internal == NULL ){
					printk("Internal rtc not found - device not allocated");
					break;
				}
				
				err_internal = snvs_rtc_set_time(dev_share->dev_internal, tm);

				if ( err_internal != 0)
					printk("error writing imx6 rtc-snvs\n");
				retry --;
			}
	}
	else if ( dev_share->dev_internal != NULL )
                err = snvs_rtc_read_time(dev_share->dev_internal, tm);
	if ( err !=0)
		printk("Error reading rtc");
	
	return err;
}

/*
   This function writes time in external rtc and in internal RTC
*/

static int rtc_bridge_set_time(struct device *dev, struct rtc_time *tm){

	int err = -1, retry = 5;
	
	while ( err != 0 &&  retry > 0 ){
		if ( dev_share->dev_external == NULL ){
			printk("External rtc not found - device not allocated");
			break;	
		}
		err = pcf2123_rtc_set_time(dev_share->dev_external,tm); 
		if ( err != 0)
			printk("error writing rtc-pcf2123\n");
	retry --;
	}
	
	retry = 5;
	err = -1;
	
	while ( err != 0 &&  retry > 0 ){
		if ( dev_share->dev_internal == NULL ){
                        printk("Internal rtc not found - device not allocated");
                        break;
                }
		err = snvs_rtc_set_time(dev_share->dev_internal, tm);
	        if ( err != 0)
                        printk("error writing imx6 rtc-snvs\n");
        retry --;	
	}
	
	return err; 
}

#ifdef CONFIG_RT_EXT
static int rtc_bridge_read_alrm (struct device *dev, struct rtc_wkalrm *alrm) {

	int err = -1;

	if ( dev_share->dev_external != NULL ){

		err = pcf2123_rtc_read_alrm (dev_share->dev_external, alrm);
	
	}
	if ( err !=0)
		printk("Error reading rtc pcf2123");
	
	return err;
}


static int rtc_bridge_set_alrm (struct device *dev, struct rtc_wkalrm *alrm) {

	int err = -1;

	if ( dev_share->dev_external != NULL ){

		err = pcf2123_rtc_set_alrm (dev_share->dev_external, alrm);
			
	}
	if ( err !=0)
		printk("Error writing rtc pcf2123");
	
	return err;
}


static ssize_t rtc_bridge_sysfs_show_alarm(struct device *dev, struct device_attribute *attr,
                char *buf)
{
        ssize_t retval;
        struct rtc_wkalrm alm;

        retval = rtc_bridge_read_alrm (dev_share->dev_external, &alm);
        retval = sprintf (buf, "%02d:%02d:%02d:%01d\n", 
			alm.time.tm_min, alm.time.tm_hour, alm.time.tm_mday, alm.time.tm_wday);

        return retval;
}


static ssize_t rtc_bridge_sysfs_set_alarm(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t count)
{
	ssize_t retval;
	struct rtc_wkalrm alm;
	char *buf_ptr;
	int idx, err_conv;
	char *elm;
	long min, hr, dw, dm;

	/* mm:hh:DD:d 
	 * mm = minutes
	 * hh = hours
	 * DD = day of month
	 * d  = day of week
	 */

	buf_ptr = (char *)buf;	
	
	/* check if the string has the correct form */
#define STRING_LEN 11
	if (strlen (buf_ptr) != STRING_LEN) 
		return -EINVAL;
	
	idx = 0;
	elm = kzalloc (sizeof (char) * 2, GFP_KERNEL);
	while (idx != STRING_LEN) {
		switch (idx) {
			case 0:
				elm[0] = *buf_ptr;
				elm[1] = *(++buf_ptr);
				err_conv = strict_strtol (elm, 10, &min);
				if (err_conv) 
					return -EINVAL;
				if ((min < 0) || (min > 59))
					return -EINVAL;
				buf_ptr++;
				break;
			case 3:
				elm[0] = *buf_ptr;
				elm[1] = *(++buf_ptr);
				err_conv = strict_strtol (elm, 10, &hr);
				if (err_conv) 
					return -EINVAL;
				if ((hr < 0) || (hr > 23))
					return -EINVAL;
				buf_ptr++;
				break;
			case 6:
				elm[0] = *buf_ptr;
				elm[1] = *(++buf_ptr);
				err_conv = strict_strtol (elm, 10, &dm);
				if (err_conv)
					return -EINVAL;
				if ((dm < 0) || (dm > 31))
					return -EINVAL;
				buf_ptr++;
				break;
			case 9:
				elm[0] = '0';
				elm[1] = *buf_ptr;
				err_conv = strict_strtol (elm, 10, &dw);
				if (err_conv)
					return -EINVAL;
				if ((dw < 0) || (dw > 6))
					return -EINVAL;
				break;
			case 2:
			case 5:
			case 8:
				if (*buf_ptr != ':')
					return -EINVAL;
				buf_ptr++;
				break;
			default:
				break;
		}
		idx++;
	}

	retval = rtc_bridge_read_alrm (dev_share->dev_external, &alm);
	if (retval)
		return -EIO;
	
	alm.time.tm_min  = (int)min;
	alm.time.tm_hour = (int)hr;
	alm.time.tm_mday = (int)dm;
	alm.time.tm_wday = (int)dw;

	retval = rtc_bridge_set_alrm (dev_share->dev_external, &alm);
	return count;
}


static ssize_t rtc_bridge_sysfs_show_alarm_en (struct device *dev, struct device_attribute *attr, 
	char *buf) 
{
	 ssize_t retval;
        struct rtc_wkalrm alm;

        retval = rtc_bridge_read_alrm (dev_share->dev_external, &alm);
        retval = sprintf (buf, "min:hr:DD:d\n-----------\n  %d: %d: %d:%d\n", 
			(~alm.enabled & 0x1) >> 0,
			(~alm.enabled & 0x2) >> 1,
			(~alm.enabled & 0x4) >> 2,
			(~alm.enabled & 0x8) >> 3);

        return retval;

}


static ssize_t rtc_bridge_sysfs_set_alarm_en (struct device *dev, struct device_attribute *attr,
                const char *buf, size_t count)
{
	ssize_t retval;
	struct rtc_wkalrm alm;
	char *buf_ptr;
	int idx, err_conv;
	char *elm;
	long flag;
	u8 enable = 0x00;

	/* -------------------------
	 * |mm|hh|DD| d| 0| 0| 0| 0|
	 * -------------------------  
	 * mm = minutes
	 * hh = hours
	 * DD = day of month
	 * d  = day of week
	 *
	 * each flag can assume only 0/1 value.
	 */

	buf_ptr = (char *)buf;	
	
	/* check if the string has the correct form */
#define STRING_LEN_EN 8
#define POS_EN_MIN    0
#define POS_EN_HR     2
#define POS_EN_DM     4
#define POS_EN_DW     6
	if (strlen (buf_ptr) != STRING_LEN_EN) 
		return -EINVAL;
	
	idx = 0;
	elm = kzalloc (sizeof (char) * 2, GFP_KERNEL);
	elm[0] = '0';
	while (idx != STRING_LEN_EN) {
		switch (idx) {
			case POS_EN_MIN:
			case POS_EN_HR:
			case POS_EN_DM:
			case POS_EN_DW:
				elm[1] = *buf_ptr;
				err_conv = strict_strtol (elm, 10, &flag);
				if (err_conv) 
					return -EINVAL;
				if (flag != 0 && flag != 1)
					return -EINVAL;
				enable = !flag ? enable | (1 << (idx >> 1)) : enable & ~(1 << (idx >> 1));
				buf_ptr++;
				break;
			case 1:
			case 3:
			case 5:
				if (*buf_ptr != ':')
					return -EINVAL;
				buf_ptr++;
				break;

			default:
				break;
		}
		idx++;
	}

	retval = rtc_bridge_read_alrm (dev_share->dev_external, &alm);
	if (retval)
		return -EIO;
	
	alm.enabled  = (int)enable;

	retval = rtc_bridge_set_alrm (dev_share->dev_external, &alm);
	return count;
}


static DEVICE_ATTR(alarm, S_IRUGO | S_IWUSR,
                rtc_bridge_sysfs_show_alarm, rtc_bridge_sysfs_set_alarm);
static DEVICE_ATTR(enable_alarm, S_IRUGO | S_IWUSR,
                rtc_bridge_sysfs_show_alarm_en, rtc_bridge_sysfs_set_alarm_en);



static long rtc_bridge_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {
	int err = 0;
	int retval = 0, i;
	struct alrm_pcf alarm;
	struct rtc_wkalrm alrm;	
	switch (cmd) {
		case RTC_BRIDGE_IOCTL_ALM_READ:
			if (copy_from_user (&alarm, (const void __user *)arg, sizeof (alarm))) {
                        	return -EFAULT;
                        }
			err = rtc_bridge_read_alrm (dev_share->dev_external, &alrm);
			alarm.min    = alrm.time.tm_min;
			alarm.hr     = alrm.time.tm_hour;
			alarm.mday   = alrm.time.tm_mday;
			alarm.wday   = alrm.time.tm_wday;
			alarm.enable = 0;
			for (i = 0 ; i < 4 ; i++) 
				alarm.enable |= ~((u8)alrm.enabled & ~(1 << i)) & (1 << i);

                        if (err < 0)
                                retval = err;
                        if (copy_to_user ((void __user *)arg, &alarm, sizeof (alarm))) {
                                retval = -EFAULT;
                        }
			break;
		case RTC_BRIDGE_IOCTL_ALM_WRITE:
			if (copy_from_user (&alarm, (const void __user *)arg, sizeof (alarm))) {
                        	return -EFAULT;
                        }
			alrm.time.tm_min    = alarm.min;
			alrm.time.tm_hour   = alarm.hr;
			alrm.time.tm_mday   = alarm.mday;
			alrm.time.tm_wday   = alarm.wday;
			alrm.enabled        = 0x00;
			for (i = 0 ; i < 4 ; i++) 
				alrm.enabled |= ~((u8)alarm.enable & ~(1 << i)) & (1 << i);

			err = rtc_bridge_set_alrm (dev_share->dev_external, &alrm);
                        if (err < 0)
                                retval = err;
                        if (copy_to_user ((void __user *)arg, &alarm, sizeof (alarm))) {
                                retval = -EFAULT;
                        }

			break;
		default:
			break;
	}
	return retval;
}
#endif

static const struct rtc_class_ops rtc_bridge_ops = {
    .read_time      = rtc_bridge_read_time,
    .set_time       = rtc_bridge_set_time,
#ifdef CONFIG_RT_EXT
    .read_alarm     = rtc_bridge_read_alrm,
    .set_alarm      = rtc_bridge_set_alrm,
    .ioctl          = rtc_bridge_ioctl,
#endif
};

static int __devinit rtc_bridge_probe(struct platform_device *pdev)
{
	
	struct rtc_device *rtc;
	int ret;	

	printk("rtc_bridge probing...");

	if ( dev_share == NULL )
                dev_share = kmalloc(sizeof(struct rtc_dev_share),GFP_KERNEL);

	//rtc_share_func = pdev->dev.platform_data;
	//dev_share = kmalloc(sizeof(*dev_share),GFP_KERNEL);
	/* Finalize the initialization */
        rtc = rtc_device_register("rtc_bridge",&pdev->dev,
                        &rtc_bridge_ops, THIS_MODULE);

        if (IS_ERR(rtc)) {
                printk("RTC BRIDGE failed to register.\n");
                ret = PTR_ERR(rtc);
                goto kfree_exit;
        }
	
#ifdef CONFIG_RT_EXT
	ret = device_create_file(&rtc->dev, &dev_attr_alarm);
	ret = device_create_file(&rtc->dev, &dev_attr_enable_alarm);
#endif
        if (ret)
                dev_err(&rtc->dev, "failed to create alarm attribute, %d\n", ret);


	printk("done\n");
	
	return 0;
	
kfree_exit:
	printk("probing error\n");
        return -1;


}

static int __devexit rtc_bridge_remove(void)
{
	
	printk("rtc_bridge: clean memory");
        kfree(dev_share); 

        return 0;
}

/*!
 * Contains pointers to the power management callback functions.
 */
static struct platform_driver rtc_bridge_driver = {
        .driver = {
                   .name = "rtc_bridge",
                   },
        .probe = rtc_bridge_probe,
	.remove = __devexit_p(rtc_bridge_remove),
};

/*!
 * This function creates the /proc/driver/rtc file and registers the device RTC
 * in the /dev/misc directory. It also reads the RTC value from external source
 * and setup the internal RTC properly.
 *
 * @return  -1 if RTC is failed to initialize; 0 is successful.
 */
static int __init rtc_bridge_init(void)
{
        return platform_driver_register(&rtc_bridge_driver);
}

/*!
 * This function removes the /proc/driver/rtc file and un-registers the
 * device RTC from the /dev/misc directory.
 */
static void __exit rtc_bridge_exit(void)
{
        platform_driver_unregister(&rtc_bridge_driver);

}

module_init(rtc_bridge_init);
module_exit(rtc_bridge_exit);

MODULE_AUTHOR("MS SECO, Inc.");
MODULE_DESCRIPTION("BRIDGE Realtime Clock Driver (RTC)");
MODULE_LICENSE("GPL");
