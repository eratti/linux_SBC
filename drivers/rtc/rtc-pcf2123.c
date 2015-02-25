/*
 * An SPI driver for the Philips PCF2123 RTC
 * Copyright 2009 Cyber Switching, Inc.
 *
 * Author: Chris Verges <chrisv@cyberswitching.com>
 * Maintainers: http://www.cyberswitching.com
 *
 * based on the RS5C348 driver in this same directory.
 *
 * Thanks to Christian Pellegrin <chripell@fsfe.org> for
 * the sysfs contributions to this driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Please note that the CS is active high, so platform data
 * should look something like:
 *
 * static struct spi_board_info ek_spi_devices[] = {
 * 	...
 * 	{
 * 		.modalias		= "rtc-pcf2123",
 * 		.chip_select		= 1,
 * 		.controller_data	= (void *)AT91_PIN_PA10,
 *		.max_speed_hz		= 1000 * 1000,
 *		.mode			= SPI_CS_HIGH,
 *		.bus_num		= 0,
 *	},
 *	...
 *};
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
#include <linux/spi/spi.h>
#include <linux/rtc-pcf2123.h>
#include <linux/rtc-bridge.h>

#define DRV_VERSION "0.6"

#define PCF2123_REG_CTRL1	(0x00)	/* Control Register 1 */
#define PCF2123_REG_CTRL2	(0x01)	/* Control Register 2 */
/* -------- TIME -------- */
#define PCF2123_REG_SC		(0x02)	/* datetime */
#define PCF2123_REG_MN		(0x03)
#define PCF2123_REG_HR		(0x04)
#define PCF2123_REG_DM		(0x05)
#define PCF2123_REG_DW		(0x06)
#define PCF2123_REG_MO		(0x07)
#define PCF2123_REG_YR		(0x08)
/* -------- ALARM -------- */
#define PCF2123_REG_ALM_MN      (0x09)
#define PCF2123_REG_ALM_HR      (0x0A)
#define PCF2123_REG_ALM_DM      (0x0B)
#define PCF2123_REG_ALM_DW      (0x0C)

#define PCF2123_MASK_ENABLE     (1 << 7)
#define PCF2123_MASK_ALM_EN_MN  (1 << 0)
#define PCF2123_MASK_ALM_EN_HR  (1 << 1)
#define PCF2123_MASK_ALM_EN_DM  (1 << 2)
#define PCF2123_MASK_ALM_EN_DW  (1 << 3)
/*---------SECO---------------------*/
#define PCF2123_REG_OFFSET     		 (0x0d)   	/* Offset register */
#define PCF2123_MODE_NORMAL     	 (0 << 7) 	/* Normal mode flag offset register */
#define PCF2123_MODE_COURSE    		 (1 << 7) 	/* Course mode flag offset register */
#define PCF2123_SECO_OFFSET		 (0x0B)	/* Seco RTC Compensation Offset -2.057 s/day */
#define PCF2123_SET_SECO_OFFSET      	 ( PCF2123_MODE_NORMAL | PCF2123_SECO_OFFSET )
#define	SECO_OFFSET			 1
/*---------SECO---------------------*/
#define PCF2123_SUBADDR		(1 << 4)
#define PCF2123_WRITE		((0 << 7) | PCF2123_SUBADDR)
#define PCF2123_READ		((1 << 7) | PCF2123_SUBADDR)

static struct spi_driver pcf2123_driver;

extern struct rtc_dev_share *dev_share;

struct pcf2123_sysfs_reg {
	struct device_attribute attr;
	char name[2];
};

struct pcf2123_plat_data {
	struct rtc_device *rtc;
	struct pcf2123_sysfs_reg regs[16];
};

/*
 * Causes a 30 nanosecond delay to ensure that the PCF2123 chip select
 * is released properly after an SPI write.  This function should be
 * called after EVERY read/write call over SPI.
 */
static inline void pcf2123_delay_trec(void)
{
	ndelay(30);
}

static ssize_t pcf2123_show(struct device *dev, struct device_attribute *attr,
			    char *buffer)
{
	struct spi_device *spi = to_spi_device(dev);
	struct pcf2123_sysfs_reg *r;
	u8 txbuf[1], rxbuf[1];
	unsigned long reg;
	int ret;

	r = container_of(attr, struct pcf2123_sysfs_reg, attr);

	if (strict_strtoul(r->name, 16, &reg))
		return -EINVAL;

	txbuf[0] = PCF2123_READ | reg;
	ret = spi_write_then_read(spi, txbuf, 1, rxbuf, 1);
	if (ret < 0)
		return -EIO;
	pcf2123_delay_trec();
	return sprintf(buffer, "0x%x\n", rxbuf[0]);
}

static ssize_t pcf2123_store(struct device *dev, struct device_attribute *attr,
			     const char *buffer, size_t count) {
	struct spi_device *spi = to_spi_device(dev);
	struct pcf2123_sysfs_reg *r;
	u8 txbuf[2];
	unsigned long reg;
	unsigned long val;

	int ret;

	r = container_of(attr, struct pcf2123_sysfs_reg, attr);

	if (strict_strtoul(r->name, 16, &reg)
		|| strict_strtoul(buffer, 10, &val))
		return -EINVAL;

	txbuf[0] = PCF2123_WRITE | reg;
	txbuf[1] = val;
	ret = spi_write(spi, txbuf, sizeof(txbuf));
	if (ret < 0)
		return -EIO;
	pcf2123_delay_trec();
	return count;
}

//static int pcf2123_rtc_read_time(struct device *dev, struct rtc_time *tm)
int pcf2123_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct spi_device *spi = to_spi_device(dev);
	u8 txbuf[1], rxbuf[7];
	int ret;

	txbuf[0] = PCF2123_READ | PCF2123_REG_SC;
	ret = spi_write_then_read(spi, txbuf, sizeof(txbuf),
			rxbuf, sizeof(rxbuf));
	if (ret < 0)
		return ret;
	pcf2123_delay_trec();

	tm->tm_sec = bcd2bin(rxbuf[0] & 0x7F);
	tm->tm_min = bcd2bin(rxbuf[1] & 0x7F);
	tm->tm_hour = bcd2bin(rxbuf[2] & 0x3F); /* rtc hr 0-23 */
	tm->tm_mday = bcd2bin(rxbuf[3] & 0x3F);
	tm->tm_wday = rxbuf[4] & 0x07;
	tm->tm_mon = bcd2bin(rxbuf[5] & 0x1F) - 1; /* rtc mn 1-12 */
	tm->tm_year = bcd2bin(rxbuf[6]);
	if (tm->tm_year < 70)
		tm->tm_year += 100;	/* assume we are in 1970...2069 */

	dev_dbg(dev, "%s: tm is secs=%d, mins=%d, hours=%d, "
			"mday=%d, mon=%d, year=%d, wday=%d\n",
			__func__,
			tm->tm_sec, tm->tm_min, tm->tm_hour,
			tm->tm_mday, tm->tm_mon, tm->tm_year, tm->tm_wday);

	/* the clock can give out invalid datetime, but we cannot return
	 * -EINVAL otherwise hwclock will refuse to set the time on bootup.
	 */
	if (rtc_valid_tm(tm) < 0)
		dev_err(dev, "retrieved date/time is not valid.\n");

	return 0;
}

//static int pcf2123_rtc_set_time(struct device *dev, struct rtc_time *tm)
int pcf2123_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct spi_device *spi = to_spi_device(dev);
	u8 txbuf[8];
	int ret;

	dev_dbg(dev, "%s: tm is secs=%d, mins=%d, hours=%d, "
			"mday=%d, mon=%d, year=%d, wday=%d\n",
			__func__,
			tm->tm_sec, tm->tm_min, tm->tm_hour,
			tm->tm_mday, tm->tm_mon, tm->tm_year, tm->tm_wday);

	/* Stop the counter first */
	txbuf[0] = PCF2123_WRITE | PCF2123_REG_CTRL1;
	txbuf[1] = 0x20;
	ret = spi_write(spi, txbuf, 2);
	if (ret < 0)
		return ret;
	pcf2123_delay_trec();

	/* Set the new time */
	txbuf[0] = PCF2123_WRITE | PCF2123_REG_SC;
	txbuf[1] = bin2bcd(tm->tm_sec & 0x7F);
	txbuf[2] = bin2bcd(tm->tm_min & 0x7F);
	txbuf[3] = bin2bcd(tm->tm_hour & 0x3F);
	txbuf[4] = bin2bcd(tm->tm_mday & 0x3F);
	txbuf[5] = tm->tm_wday & 0x07;
	txbuf[6] = bin2bcd((tm->tm_mon + 1) & 0x1F); /* rtc mn 1-12 */
	txbuf[7] = bin2bcd(tm->tm_year < 100 ? tm->tm_year : tm->tm_year - 100);

	ret = spi_write(spi, txbuf, sizeof(txbuf));
	if (ret < 0)
		return ret;
	pcf2123_delay_trec();

	/* Start the counter */
	txbuf[0] = PCF2123_WRITE | PCF2123_REG_CTRL1;
	txbuf[1] = 0x00;
	ret = spi_write(spi, txbuf, 2);
	if (ret < 0)
		return ret;
	pcf2123_delay_trec();

	return 0;
}


#define GET_ALM_EN(reg)                   (!!((reg) & PCF2123_MASK_ENABLE))
#define SET_ALM_EN(reg, en)               ((reg) = (en) ? (reg) | PCF2123_MASK_ENABLE : \
						(reg) & ~PCF2123_MASK_ENABLE) 

#define GET_REG_ALM_EN(reg, mask)        (!!((reg) & (mask)))
#define SET_REG_ALM_EN(reg, mask, en)    ((reg) = (en) ? (reg) | (mask) : \
						(reg) & ~(mask))

int pcf2123_rtc_read_reg_alrm (struct device *dev) {
	struct spi_device *spi = to_spi_device(dev);
	u8 txbuf[1], rxbuf[4];
	int ret;

	txbuf[0] = PCF2123_READ | PCF2123_REG_ALM_MN;
	ret = spi_write_then_read(spi, txbuf, sizeof(txbuf),
			rxbuf, sizeof(rxbuf));
	if (ret < 0)
		return ret;
	pcf2123_delay_trec();

	printk (KERN_ERR "dump alarm reg:\n %02x  %02x  %02x  %02x\n",
			rxbuf[0], rxbuf[1], rxbuf[2], rxbuf[3]);

	txbuf[0] = PCF2123_READ | PCF2123_REG_CTRL2;
	ret = spi_write_then_read(spi, txbuf, sizeof(txbuf),
			rxbuf, 1);
	if (ret < 0)
		return ret;
	pcf2123_delay_trec();

	printk (KERN_ERR "dump ctrl2 reg:\n %02x\n",
			rxbuf[0]);

	return 0;
}

int pcf2123_rtc_clear_int_alrm (struct device *dev) {
	struct spi_device *spi = to_spi_device(dev);
	int ret;
	u8 txbuf_en[2], rxbuf_en[1];

	txbuf_en[0] = PCF2123_READ | PCF2123_REG_CTRL2;
	ret = spi_write_then_read(spi, txbuf_en, 1,
			rxbuf_en, sizeof(rxbuf_en));
	if (ret < 0)
		return ret;
	pcf2123_delay_trec();


	txbuf_en[0] = PCF2123_WRITE | PCF2123_REG_CTRL2;
	txbuf_en[1] = rxbuf_en[0] & ~((u8)1 << 3);
	ret = spi_write(spi, txbuf_en, sizeof(txbuf_en));
	if (ret < 0)
		return ret;
	pcf2123_delay_trec();

	//pcf2123_rtc_read_reg_alrm (dev);
	return 0;
}


int pcf2123_rtc_read_alrm (struct device *dev, struct rtc_wkalrm *alrm) {
	struct spi_device *spi = to_spi_device(dev);
	u8 txbuf[1], rxbuf[4];
	int ret;

	txbuf[0] = PCF2123_READ | PCF2123_REG_ALM_MN;
	ret = spi_write_then_read(spi, txbuf, sizeof(txbuf),
			rxbuf, sizeof(rxbuf));
	if (ret < 0)
		return ret;
	pcf2123_delay_trec();

	alrm->time.tm_min  = bcd2bin(rxbuf[0] & 0x7F);
	SET_REG_ALM_EN (alrm->enabled, PCF2123_MASK_ALM_EN_MN, GET_ALM_EN(rxbuf[0]));
	alrm->time.tm_hour = bcd2bin(rxbuf[1] & 0x7F);
	SET_REG_ALM_EN (alrm->enabled, PCF2123_MASK_ALM_EN_HR, GET_ALM_EN(rxbuf[1]));
	alrm->time.tm_mday = bcd2bin(rxbuf[2] & 0x3F);
	SET_REG_ALM_EN (alrm->enabled, PCF2123_MASK_ALM_EN_DM, GET_ALM_EN(rxbuf[2]));
	alrm->time.tm_wday = rxbuf[3] & 0x07;
	SET_REG_ALM_EN (alrm->enabled, PCF2123_MASK_ALM_EN_DW, GET_ALM_EN(rxbuf[3]));

	dev_dbg(dev, "%s: alrm is mins=%d, hours=%d, mday=%d, wday=%d\n",
			__func__,
			alrm->time.tm_min, alrm->time.tm_hour,
			alrm->time.tm_mday, alrm->time.tm_wday);

	if (rtc_valid_alrm (alrm) < 0)
		dev_err(dev, "retrieved alrm is not valid.\n");

//	pcf2123_rtc_read_reg_alrm (dev);

	return 0;
}


int pcf2123_rtc_set_alrm (struct device *dev, struct rtc_wkalrm *alrm) {
	struct spi_device *spi = to_spi_device(dev);
	u8 txbuf[5];
	int ret, en_int;
	u8 txbuf_en[2], rxbuf_en[1];

	/* Unset the AIE flag, temporarily */

	txbuf_en[0] = PCF2123_READ | PCF2123_REG_CTRL2;
	ret = spi_write_then_read(spi, txbuf_en, 1,
			rxbuf_en, sizeof(rxbuf_en));
	if (ret < 0)
		return ret;
	pcf2123_delay_trec();

	txbuf_en[0] = PCF2123_WRITE | PCF2123_REG_CTRL2;
	txbuf_en[1] = rxbuf_en[0] & ~((u8)1 << 1);
	ret = spi_write(spi, txbuf_en, sizeof(txbuf_en));
	if (ret < 0)
		return ret;
	pcf2123_delay_trec();

	dev_dbg(dev, "%s: alrm is mins=%d, hours=%d, mday=%d, wday=%d\n",
			__func__,
			alrm->time.tm_min, alrm->time.tm_hour,
			alrm->time.tm_mday, alrm->time.tm_wday);

	/* Set the alrm */
	txbuf[0] = PCF2123_WRITE | PCF2123_REG_ALM_MN;
	txbuf[1] = bin2bcd(alrm->time.tm_min & 0x7F);
	SET_ALM_EN (txbuf[1], GET_REG_ALM_EN(alrm->enabled, PCF2123_MASK_ALM_EN_MN));
	txbuf[2] = bin2bcd(alrm->time.tm_hour & 0x3F);
	SET_ALM_EN (txbuf[2], GET_REG_ALM_EN(alrm->enabled, PCF2123_MASK_ALM_EN_HR));
	txbuf[3] = bin2bcd(alrm->time.tm_mday & 0x3F);
	SET_ALM_EN (txbuf[3], GET_REG_ALM_EN(alrm->enabled, PCF2123_MASK_ALM_EN_DM));
	txbuf[4] = bin2bcd(alrm->time.tm_wday & 0x07);
	SET_ALM_EN (txbuf[4], GET_REG_ALM_EN(alrm->enabled, PCF2123_MASK_ALM_EN_DW));

	ret = spi_write(spi, txbuf, sizeof(txbuf));
	if (ret < 0)
		return ret;
	pcf2123_delay_trec();


	/* Set the AIE flag, if needed (enable == 1) */

	en_int = (u8)alrm->enabled == 0x0F? 0 : 1;

	if (en_int) {
		txbuf_en[0] = PCF2123_READ | PCF2123_REG_CTRL2;
		ret = spi_write_then_read(spi, txbuf_en, 1,
				rxbuf_en, sizeof(rxbuf_en));
		if (ret < 0)
			return ret;
		pcf2123_delay_trec();


		txbuf_en[0] = PCF2123_WRITE | PCF2123_REG_CTRL2;
		txbuf_en[1] = en_int ? rxbuf_en[0] | ((u8)1 << 1) : rxbuf_en[0] & ~((u8)1 << 1);
		ret = spi_write(spi, txbuf_en, sizeof(txbuf_en));
		if (ret < 0)
			return ret;
		pcf2123_delay_trec();
	}

	
//	pcf2123_rtc_read_reg_alrm (dev);
	return 0;
}


static const struct rtc_class_ops pcf2123_rtc_ops = {
	.read_time	= pcf2123_rtc_read_time,
	.set_time	= pcf2123_rtc_set_time,
};

static int __devinit pcf2123_probe(struct spi_device *spi)
{
	struct rtc_device *rtc;
	struct pcf2123_plat_data *pdata;
	u8 txbuf[2], rxbuf[2];
	int ret, i;

	pdata = kzalloc(sizeof(struct pcf2123_plat_data), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;
	spi->dev.platform_data = pdata;

	/* 
	   Reading OS flag for checking pcf2123 status 
	   OS Flag = 1 -> bad state
     	   OS Flag = 0 -> good state	
	*/
	txbuf[0] = PCF2123_READ | PCF2123_REG_SC;
        dev_dbg(&spi->dev, "reading RTC os flag\n");
        ret = spi_write_then_read(spi, txbuf, 1 * sizeof(u8),
                                        rxbuf, 1 * sizeof(u8));
        if (ret < 0)
                goto kfree_exit;

	/* Send a software reset command if necessary*/
	if ( rxbuf[0] >> 7 ){

	txbuf[0] = PCF2123_WRITE | PCF2123_REG_CTRL1;
	txbuf[1] = 0x58;
	dev_dbg(&spi->dev, "resetting RTC (0x%02X 0x%02X)\n",
			txbuf[0], txbuf[1]);
	ret = spi_write(spi, txbuf, 2 * sizeof(u8));
	if (ret < 0)
		goto kfree_exit;
	pcf2123_delay_trec();
	}


	/* Stop the counter */
	txbuf[0] = PCF2123_WRITE | PCF2123_REG_CTRL1;
	txbuf[1] = 0x20;
	dev_dbg(&spi->dev, "stopping RTC (0x%02X 0x%02X)\n",
			txbuf[0], txbuf[1]);
	ret = spi_write(spi, txbuf, 2 * sizeof(u8));
	if (ret < 0)
		goto kfree_exit;
	pcf2123_delay_trec();

	/* See if the counter was actually stopped */
	txbuf[0] = PCF2123_READ | PCF2123_REG_CTRL1;
	dev_dbg(&spi->dev, "checking for presence of RTC (0x%02X)\n",
			txbuf[0]);
	ret = spi_write_then_read(spi, txbuf, 1 * sizeof(u8),
					rxbuf, 2 * sizeof(u8));
	dev_dbg(&spi->dev, "received data from RTC (0x%02X 0x%02X)\n",
			rxbuf[0], rxbuf[1]);
	if (ret < 0)
		goto kfree_exit;
	pcf2123_delay_trec();

	if (!(rxbuf[0] & 0x20)) {
		dev_err(&spi->dev, "chip not found\n");
		goto kfree_exit;
	}

	dev_info(&spi->dev, "chip found, driver version " DRV_VERSION "\n");
	dev_info(&spi->dev, "spiclk %u KHz.\n",
			(spi->max_speed_hz + 500) / 1000);

	/* Set Seco RTC compensation */
	#if SECO_OFFSET
	
        txbuf[0] = PCF2123_WRITE | PCF2123_REG_OFFSET;
        txbuf[1] = PCF2123_SET_SECO_OFFSET;
        dev_dbg(&spi->dev, "setting RTC offset (0x%02X 0x%02X)\n",
                        txbuf[0], txbuf[1]);
        ret = spi_write(spi, txbuf, 2 * sizeof(u8));
        if (ret < 0)
                goto kfree_exit;
        pcf2123_delay_trec();
	#endif	

	/* Start the counter */
	txbuf[0] = PCF2123_WRITE | PCF2123_REG_CTRL1;
	txbuf[1] = 0x00;
	ret = spi_write(spi, txbuf, sizeof(txbuf));
	if (ret < 0)
		goto kfree_exit;
	pcf2123_delay_trec();

#ifndef CONFIG_Q7_RTC_BRIDGE

	/* Finalize the initialization */
	rtc = rtc_device_register(pcf2123_driver.driver.name, &spi->dev,
			&pcf2123_rtc_ops, THIS_MODULE);

	if (IS_ERR(rtc)) {
		dev_err(&spi->dev, "failed to register.\n");
		ret = PTR_ERR(rtc);
		goto kfree_exit;
	}

	pdata->rtc = rtc;

	for (i = 0; i < 16; i++) {
		sprintf(pdata->regs[i].name, "%1x", i);
		pdata->regs[i].attr.attr.mode = S_IRUGO | S_IWUSR;
		pdata->regs[i].attr.attr.name = pdata->regs[i].name;
		pdata->regs[i].attr.show = pcf2123_show;
		pdata->regs[i].attr.store = pcf2123_store;
		ret = device_create_file(&spi->dev, &pdata->regs[i].attr);
		if (ret) {
			dev_err(&spi->dev, "Unable to create sysfs %s\n",
				pdata->regs[i].name);
			goto sysfs_exit;
		}
	}
#else	

	pcf2123_rtc_clear_int_alrm (&spi->dev);
	pdata->rtc = rtc;
        
        for (i = 0; i < 16; i++) {
                sprintf(pdata->regs[i].name, "%1x", i);
                pdata->regs[i].attr.attr.mode = S_IRUGO | S_IWUSR;
                pdata->regs[i].attr.attr.name = pdata->regs[i].name;
                pdata->regs[i].attr.show = pcf2123_show;
                pdata->regs[i].attr.store = pcf2123_store;
                ret = device_create_file(&spi->dev, &pdata->regs[i].attr);
                if (ret) {
                        dev_err(&spi->dev, "Unable to create sysfs %s\n",
                                pdata->regs[i].name);
                        goto sysfs_exit;
                }
        }



	if ( dev_share == NULL )
                dev_share = kmalloc(sizeof(*dev_share),GFP_KERNEL);	
        dev_share->dev_external = &spi->dev;
	printk("RTC bridge defined: pcf2123 sysfs not created\n");

#endif

	return 0;

sysfs_exit:
	for (i--; i >= 0; i--)
		device_remove_file(&spi->dev, &pdata->regs[i].attr);

kfree_exit:
	kfree(pdata);
	spi->dev.platform_data = NULL;
	return ret;
}

static int __devexit pcf2123_remove(struct spi_device *spi)
{
	struct pcf2123_plat_data *pdata = spi->dev.platform_data;
	int i;

	if (pdata) {
		struct rtc_device *rtc = pdata->rtc;

		if (rtc)
			rtc_device_unregister(rtc);
		for (i = 0; i < 16; i++)
			if (pdata->regs[i].name[0])
				device_remove_file(&spi->dev,
						   &pdata->regs[i].attr);
		kfree(pdata);
	}

	return 0;
}

static struct spi_driver pcf2123_driver = {
	.driver	= {
			.name	= "rtc-pcf2123",
			.bus	= &spi_bus_type,
			.owner	= THIS_MODULE,
	},
	.probe	= pcf2123_probe,
	.remove	= __devexit_p(pcf2123_remove),
};

static int __init pcf2123_init(void)
{
	return spi_register_driver(&pcf2123_driver);
}

static void __exit pcf2123_exit(void)
{
	spi_unregister_driver(&pcf2123_driver);
}

MODULE_AUTHOR("Chris Verges <chrisv@cyberswitching.com>");
MODULE_DESCRIPTION("NXP PCF2123 RTC driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);

module_init(pcf2123_init);
module_exit(pcf2123_exit);
