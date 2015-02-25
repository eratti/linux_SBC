/*
 * 
 *
 * WDOG Trigger driver
 *
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include "apx_wdog-trigger.h"

#define WDOG_TRIGGER_REFRESH			msecs_to_jiffies(1)		// milliseconds
#define WDOG_TIME				msecs_to_jiffies(10*1000)	// 10s

static struct wdog_trigger_data {

        void (*refresh_wdt)(void);
	unsigned long wdt_time;
	unsigned long wdt_count;
	int wdt_refresh;
	int wdt_always_on;
	int wdt_enable;
        u32 wdtd_gpio_trg_iomux;
        u32 wdtd_gpio_trg_padctrl;
	u32 wdtd_gpio_trg_base;
	int wdtd_gpio_trg_num;
        u32 wdtd_gpio_en_iomux;
        u32 wdtd_gpio_en_padctrl;
	u32 wdtd_gpio_en_base;
	int wdtd_gpio_en_num;

} wdt;

/* 
--------------------| WDOG TRIGGER PIN WDT_TRG - WDT_TRG |---------------------------
*/

#define WDT_TRG_IOMUX_REG(x)	MX6_IO_ADDRESS(x)
#define WDT_TRG_PADCTRL_REG(x)	MX6_IO_ADDRESS(x)
#define WDT_TRG_GDIR_REG(x)     MX6_IO_ADDRESS(x + 0x4) 
#define WDT_TRG_DR_REG(x)       MX6_IO_ADDRESS(x)
/*
--------------------| WDOG TRIGGER ENABLE WDT_EN - WDT_EN |------------------------
*/

#define WDT_EN_IOMUX_REG(x)		MX6_IO_ADDRESS(x)
#define WDT_EN_PADCTRL_REG(x)		MX6_IO_ADDRESS(x)
#define WDT_EN_GDIR_REG(x)		MX6_IO_ADDRESS(x + 0x4) 
#define WDT_EN_DR_REG(x)		MX6_IO_ADDRESS(x)


static void wdog_trigger_work_handler(struct work_struct *w);
 
static struct workqueue_struct *wq = 0;
static DECLARE_DELAYED_WORK(wdog_trigger_work, wdog_trigger_work_handler);

static void imx6_a62_wdog_refresh(void) {

        u32 reg_data;

        reg_data = __raw_readl(WDT_TRG_DR_REG(wdt.wdtd_gpio_trg_base));
        __raw_writel(reg_data | (1<<wdt.wdtd_gpio_trg_num), WDT_TRG_DR_REG(wdt.wdtd_gpio_trg_base));
        udelay(1);
       
        reg_data = __raw_readl(WDT_TRG_DR_REG(wdt.wdtd_gpio_trg_base));
	__raw_writel(reg_data & ~(1<<wdt.wdtd_gpio_trg_num),WDT_TRG_DR_REG(wdt.wdtd_gpio_trg_base));

}

static void apx_wdog_trigger_configure_pins(char *dir) {

	u32 reg_data;

	switch (dir[0]) {
		case 'e':/*case  out */
			 /* MUX as GPIO */
		        reg_data = __raw_readl(WDT_TRG_IOMUX_REG(wdt.wdtd_gpio_trg_iomux));
		        __raw_writel(reg_data | 0x5, WDT_TRG_IOMUX_REG(wdt.wdtd_gpio_trg_iomux));
		        /* PAD_CTRL GPIO */
		        __raw_writel(0x6028, WDT_TRG_PADCTRL_REG(wdt.wdtd_gpio_trg_padctrl) );
		        /* SET GPIO direction output */
		        reg_data = __raw_readl(WDT_TRG_GDIR_REG(wdt.wdtd_gpio_trg_base));
		        __raw_writel(reg_data | (1<<wdt.wdtd_gpio_trg_num), WDT_TRG_GDIR_REG(wdt.wdtd_gpio_trg_base));
		
		        /* MUX as GPIO */
		        reg_data = __raw_readl(WDT_EN_IOMUX_REG(wdt.wdtd_gpio_en_iomux));
		        __raw_writel(reg_data | 0x5, WDT_EN_IOMUX_REG(wdt.wdtd_gpio_en_iomux));
		        /* PAD_CTRL GPIO */
		        __raw_writel(0x6028, WDT_EN_PADCTRL_REG(wdt.wdtd_gpio_en_padctrl) );
		        /* SET GPIO direction output */
		        reg_data = __raw_readl(WDT_EN_GDIR_REG(wdt.wdtd_gpio_en_base));
		        __raw_writel(reg_data | (1<<wdt.wdtd_gpio_en_num), WDT_EN_GDIR_REG(wdt.wdtd_gpio_en_base));
		        /* SET GPIO value High */
		        reg_data = __raw_readl(WDT_EN_DR_REG(wdt.wdtd_gpio_en_base));
		        __raw_writel(reg_data | (1<<wdt.wdtd_gpio_en_num), WDT_EN_DR_REG(wdt.wdtd_gpio_en_base));
		break;
		case 'd':/* case in */
			/* MUX as GPIO */
                        reg_data = __raw_readl(WDT_TRG_IOMUX_REG(wdt.wdtd_gpio_trg_iomux));
                        __raw_writel(reg_data | 0x5, WDT_TRG_IOMUX_REG(wdt.wdtd_gpio_trg_iomux));
                        /* PAD_CTRL GPIO */
                        __raw_writel(0x0, WDT_TRG_PADCTRL_REG(wdt.wdtd_gpio_trg_padctrl) );
                        /* SET GPIO direction input */
                        reg_data = __raw_readl(WDT_TRG_GDIR_REG(wdt.wdtd_gpio_trg_base));
                        __raw_writel(reg_data & ~(1<<wdt.wdtd_gpio_trg_num), WDT_TRG_GDIR_REG(wdt.wdtd_gpio_trg_base));

                        /* MUX as GPIO */
                        reg_data = __raw_readl(WDT_EN_IOMUX_REG(wdt.wdtd_gpio_en_iomux));
                        __raw_writel(reg_data | 0x5, WDT_EN_IOMUX_REG(wdt.wdtd_gpio_en_iomux));
                        /* PAD_CTRL GPIO */
                        __raw_writel(0x6028, WDT_EN_PADCTRL_REG(wdt.wdtd_gpio_en_padctrl) );
                        /* SET GPIO direction output */
                        reg_data = __raw_readl(WDT_EN_GDIR_REG(wdt.wdtd_gpio_en_base));
                        __raw_writel(reg_data | (1<<wdt.wdtd_gpio_en_num), WDT_EN_GDIR_REG(wdt.wdtd_gpio_en_base));
			/* SET GPIO value High */
                        reg_data = __raw_readl(WDT_EN_DR_REG(wdt.wdtd_gpio_en_base));
                        __raw_writel(reg_data | (1<<wdt.wdtd_gpio_en_num), WDT_EN_DR_REG(wdt.wdtd_gpio_en_base));
		break;
		default:
			printk(KERN_ERR "apx_wdog: unknown pins direction\n");
		break;

	}

}

void apx_wdog_trigger_early_init(const struct apx_wdog_trigger_data *apx_wdt_data) {

	wdt.wdtd_gpio_trg_iomux         = apx_wdt_data->gpio_trg__iomux_ctrl;
        wdt.wdtd_gpio_trg_padctrl       = apx_wdt_data->gpio_trg__pad_ctrl;
        wdt.wdtd_gpio_trg_base          = apx_wdt_data->gpio_trg__base;
	wdt.wdtd_gpio_trg_num		= apx_wdt_data->gpio_trg__num;
        wdt.wdtd_gpio_en_iomux          = apx_wdt_data->gpio_en__iomux_ctrl;
        wdt.wdtd_gpio_en_padctrl        = apx_wdt_data->gpio_en__pad_ctrl;
        wdt.wdtd_gpio_en_base           = apx_wdt_data->gpio_en__base;
	wdt.wdtd_gpio_en_num		= apx_wdt_data->gpio_en__num;
	
	apx_wdog_trigger_configure_pins("enable");

}

static void wdog_trigger_work_handler(struct work_struct *w)
{
	wdt.refresh_wdt();
	if( wdt.wdt_count > 0 ) {
		queue_delayed_work(wq, &wdog_trigger_work, WDOG_TRIGGER_REFRESH);
		if (wdt.wdt_refresh)
			wdt.wdt_refresh = 0;
		else
			wdt.wdt_count = wdt.wdt_count - 1;
		if ( wdt.wdt_always_on )
			 wdt.wdt_count = wdt.wdt_time;
	}
}

/*-------------------------------*/
/* Sysfs read and write function */
/*-------------------------------*/

/* Watchdog Time */

static ssize_t wdog_trigger_time_read (struct class *cls, struct class_attribute *attr, char *buf) {

	return sprintf(buf, "%u\n", jiffies_to_msecs(wdt.wdt_time) / 1000);
}

static ssize_t wdog_trigger_time_write(struct class *cls, struct class_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	long int secs;

	ret = strict_strtol(buf, 10, &secs);
	if(ret){
		printk(KERN_ERR"wdog_trigger: error in conversion\n");
		return -EINVAL;
	}
	wdt.wdt_time = msecs_to_jiffies(secs*1000);
	wdt.wdt_count = wdt.wdt_time;
	return count;

}

/* Watchdog Enable - Disable */

static ssize_t wdog_trigger_enable_read (struct class *cls, struct class_attribute *attr, char *buf) {

	 return sprintf(buf, "%d\n", wdt.wdt_enable);	
	
}

static ssize_t wdog_trigger_enable_write(struct class *cls, struct class_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	long int enable;
	ret = strict_strtol(buf, 10, &enable);
        if(ret){
                printk(KERN_ERR"apx_wdog: error in conversion\n");
                return -EINVAL;
        }
	wdt.wdt_enable = enable;

	if ( wdt.wdt_enable == 1 ) {

		/* Enable Wdog */
		apx_wdog_trigger_configure_pins("enable");
		wdt.refresh_wdt = imx6_a62_wdog_refresh;
		wdt.wdt_count = wdt.wdt_time;
		wdt.wdt_always_on = 0;	
	        wdt.refresh_wdt();
	        pr_info("apx_wdog: enabled wdog - %u s between refresh\n", jiffies_to_msecs(WDOG_TIME) / 1000 );
	        wdt.refresh_wdt();
	        if (!wq)
	        	wq = create_singlethread_workqueue("wdog_trigger");
	        if (wq)
	        	queue_delayed_work(wq, &wdog_trigger_work, WDOG_TRIGGER_REFRESH);
		
	} else if ( wdt.wdt_enable == 0 ) {
	
		if(wq)	
			flush_workqueue(wq);
		apx_wdog_trigger_configure_pins("disable");

	} else
		return -EINVAL;	
	
        return count;

}

/* Watchdog Refresh */

static ssize_t wdog_trigger_refresh_read (struct class *cls, struct class_attribute *attr, char *buf) {

	return sprintf(buf, "%d\n", wdt.wdt_refresh);
}               
                
static ssize_t wdog_trigger_refresh_write(struct class *cls, struct class_attribute *attr, const char *buf, size_t count)
{
        int ret = 0;
        long int refresh_trigger;

        ret = strict_strtol(buf, 10, &refresh_trigger);
        if(ret){
                printk(KERN_ERR"wdog_trigger: error in conversion\n");
                return -EINVAL;
        }

	if (refresh_trigger == 1 || refresh_trigger == 0) {
		wdt.wdt_refresh = refresh_trigger;
		wdt.wdt_count = wdt.wdt_time;
		wdt.wdt_always_on = 0; // Refreshing the watchdog disable always_on feature
	}
	else
		return -EINVAL;

        return count;

}

/* Watchdog Always on */

static ssize_t wdog_trigger_always_on_read (struct class *cls, struct class_attribute *attr, char *buf) {

        return sprintf(buf, "%d\n", wdt.wdt_always_on);
}

static ssize_t wdog_trigger_always_on_write(struct class *cls, struct class_attribute *attr, const char *buf, size_t count)
{
        int ret = 0;
        long int always_on;

        ret = strict_strtol(buf, 10, &always_on);
        if(ret){
                printk(KERN_ERR"wdog_trigger: error in conversion\n");
                return -EINVAL;
        }

        if (always_on == 1 || always_on == 0)
                wdt.wdt_always_on = always_on;
        else
                return -EINVAL;

        return count;

}


static struct class_attribute class_attr[] ={ 
		__ATTR(time_s, 0644, wdog_trigger_time_read, wdog_trigger_time_write),
		__ATTR(enable, 0644, wdog_trigger_enable_read, wdog_trigger_enable_write),
		__ATTR(refresh, 0644, wdog_trigger_refresh_read, wdog_trigger_refresh_write),
		__ATTR(always_on, 0644, wdog_trigger_always_on_read, wdog_trigger_always_on_write),
		__ATTR_NULL 
};

static struct class wdog_drv ={ 
		.name = "apx_wdog",
		.owner = THIS_MODULE, 
		.class_attrs =(struct class_attribute *) &class_attr, 
};

int __init apx_wdog_trigger_work_init(void) {
	
	int ret;
	pr_info("apx_wdog trigger loaded\n");
	/* Disable wdog trigger configuring Wdog Pins*/
	apx_wdog_trigger_configure_pins("disable");

	/* Variables init */
	wdt.refresh_wdt = imx6_a62_wdog_refresh;
	wdt.wdt_time = WDOG_TIME;
	wdt.wdt_refresh = 0;
	wdt.wdt_count = wdt.wdt_time;	
	wdt.wdt_enable = 0;
	wdt.wdt_always_on = 0;

	/* Create sysfs voices */
	ret = class_register(&wdog_drv);
	if (ret) {
		goto err_wdog;
	}
	
	return 0;

err_wdog:
	printk(KERN_ERR"Unable to start apx_wdog\n");
	return 0;

}
 
static void __exit wdog_trigger_exit(void)
{
	if (wq)
		destroy_workqueue(wq);

	pr_info("wdog_trigger exit\n");

}

MODULE_AUTHOR("MS SECO, Inc.");
MODULE_DESCRIPTION("WDOG Trigger Driver (APX823)");
MODULE_LICENSE("GPL");
