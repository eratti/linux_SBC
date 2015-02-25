/*
 * Power Button management 
 * Copyright 2013 Seco S.r.l.
 *
 * Author: Matrco Sandrelli <marco.sandrelli@seco.com>
 * Maintainers: <marco.sandrelli@seco.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Add in board platform definition the following function to create exported device
 * 
 */

#include <linux/bcd.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <linux/pwrb_management.h>

#define INPUT_DEV_NAME		"Embedded Controller"
#define PWR_BUTTON_NAME		"power_button"
#define INPUT_POWER_NAME	INPUT_DEV_NAME " - " PWR_BUTTON_NAME
#define MAX_LEN_NAME		32
#define DELAY_TIME		500
#define HALT_TIME		4000

struct event_dev {
        unsigned short int      id;
	struct input_dev        *input;
        char                    name[MAX_LEN_NAME];
};

struct pwrb_control {
        struct event_dev	*events;
        int			irq;
	unsigned long		irq_flags;
	unsigned long 		pwrb_gpio;
	unsigned long		halt_gpio;
	char			*phys;
	struct delayed_work 	pwbd_event_work;
	struct workqueue_struct *pwbd_wq;
	long			msecs;
};

/*
 *	Manage Power Button event
 *
 */
static void pwbd_pressure_event_worker(struct work_struct *work)
{
	/* Report power button event */
        struct pwrb_control *wm = container_of(work, struct pwrb_control, pwbd_event_work.work);

	if (gpio_get_value(wm->pwrb_gpio) && ((jiffies_to_msecs(jiffies) - wm->msecs) < HALT_TIME) ) {
		/* Shutdown the system calling dpwrevn  */
	      	input_report_key (wm->events->input, KEY_POWER, 1);
	      	input_sync (wm->events->input);
	      	input_report_key (wm->events->input, KEY_POWER, 0);
	      	input_sync (wm->events->input);
		wm->msecs = 0;
		enable_irq(wm->irq);
	} else {

		if((jiffies_to_msecs(jiffies) - wm->msecs) > HALT_TIME) {
			/* Shutdown the system brutally */
			printk(KERN_INFO"%s: halt the system!",__FUNCTION__);
			gpio_request(wm->halt_gpio,"HALT");
			gpio_direction_output(wm->halt_gpio, 1);
			mdelay(1);
			gpio_set_value(wm->halt_gpio, 0);
			queue_delayed_work(wm->pwbd_wq, &wm->pwbd_event_work,msecs_to_jiffies(DELAY_TIME));

		} else 
			queue_delayed_work(wm->pwbd_wq, &wm->pwbd_event_work,msecs_to_jiffies(DELAY_TIME));
	}

        return;
}

/*
 *      Catch Power Button Event
 *
 */
static irqreturn_t pwrb_irq_handler (int irq, void *dev_id)  {

	struct pwrb_control *pwbd =  ((struct pwrb_control *) dev_id);
	printk (KERN_INFO "event name: %s\n", pwbd->events->input->name);
	disable_irq_nosync (pwbd->irq);
	if (!delayed_work_pending(&pwbd->pwbd_event_work)){
		pwbd->msecs = jiffies_to_msecs(jiffies);		
		queue_delayed_work(pwbd->pwbd_wq, &pwbd->pwbd_event_work, msecs_to_jiffies(DELAY_TIME));
	}

        return IRQ_HANDLED;
}



/*
 *      Probing the driver
 *
 */
static int __devinit pwrb_probe(struct platform_device *pdev)
{
	int err, ret;
	struct pwrb_control *pwrb_data;	
	struct imx_seco_pwrb_platform_data *pdata = (struct imx_seco_pwrb_platform_data *)pdev->dev.platform_data;
        
	if (!pdata) {
                dev_err(&pdev->dev, "platform data is required!\n");
                return -EINVAL;
        }
	printk("pwrb probing...");
	
	pwrb_data = kzalloc (sizeof (struct pwrb_control), GFP_KERNEL);

        if (!pwrb_data) {
                err = -ENOMEM;
                goto err_pwrb;
        }

	pwrb_data->events = kzalloc (sizeof (struct event_dev), GFP_KERNEL);
        if (!pwrb_data->events) {
                err = -ENOMEM;
                goto err_pwrb;
        }

 	pwrb_data->events->input = input_allocate_device();
	if (!pwrb_data->events->input) {
		err = -ENOMEM;
		goto err_pwrb;
	}

        pwrb_data->phys = kzalloc (sizeof (char) * 32, GFP_KERNEL);
        snprintf(pwrb_data->phys, sizeof(char) * 32,
                                "%s/input%d", dev_name(&pdev->dev), 0);

	pwrb_data->events->input->name = INPUT_POWER_NAME;
	pwrb_data->events->input->phys = pwrb_data->phys;
	pwrb_data->events->input->dev.parent = &pdev->dev;

	pwrb_data->events->input->evbit[0] = BIT_MASK(EV_KEY);
	set_bit (KEY_POWER, pwrb_data->events->input->keybit);

	err = input_register_device (pwrb_data->events->input);
	if (err) {
	        printk ("pwrb: No input assigned");
	        err = -EINVAL;
	        goto err_pwrb;
	}	

	pwrb_data->irq = gpio_to_irq(pdata->pwrb_gpio);
	pwrb_data->irq_flags = pdata->irq_flags;
	pwrb_data->pwrb_gpio = pdata->pwrb_gpio;
	pwrb_data->halt_gpio = pdata->halt_gpio;
	ret = request_irq (pwrb_data->irq, pwrb_irq_handler,
                                pwrb_data->irq_flags, "pwrb_irq", pwrb_data);

	if (ret) {
                printk ("pwrb error %d: IRQ not acquired", ret);
                err = -EIO;
                goto err_free_irq;
        }
	
        pwrb_data->pwbd_wq = create_singlethread_workqueue("pwbd_work");
	if(!pwrb_data->pwbd_wq){
		printk ("pwrb error %d: Work not allocated", ret);
                err = -EINVAL;
                goto err_free_irq;
	}
	INIT_DELAYED_WORK(&pwrb_data->pwbd_event_work, pwbd_pressure_event_worker);
	pwrb_data->msecs = 0;	
	printk("done\n");
	
	return 0;
err_pwrb:
	printk("Errors pwrb\n");
	return -1;
err_free_irq:
	printk("pwrb: freeing irq");
	free_irq (pwrb_data->irq, pwrb_data);
	return -1;

}

static int __devexit pwrb_remove(struct platform_device *pdev)
{
	printk("pwrb: clean memory");
        return 0;
}

static struct platform_driver pwrb_driver = {
        .driver = {
		.name = "imx_seco_pwrb",
	},
        .probe = pwrb_probe,
	.remove = __devexit_p(pwrb_remove),
};

static int __init pwrb_init(void)
{
        return platform_driver_register(&pwrb_driver);
}

static void __exit pwrb_exit(void)
{
        platform_driver_unregister(&pwrb_driver);

}

module_init(pwrb_init);
module_exit(pwrb_exit);

MODULE_AUTHOR("MS SECO, Inc.");
MODULE_DESCRIPTION("Power Button Management Driver");
MODULE_LICENSE("GPL");
