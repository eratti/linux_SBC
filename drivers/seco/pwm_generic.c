#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/pwm_generic.h>
#include <linux/slab.h>


#define PWM_NAME_ID   "pwm_generic"

#define PWMG_INFO(fmt, arg...) printk(KERN_INFO "PWM driver: " fmt "\n" , ## arg)
#define PWMG_ERR(fmt, arg...)  dev_err(&pdev->dev, "%s: " fmt "\n" , __func__ , ## arg)
#define PWMG_DBG(fmt, arg...)  pr_debug("%s: " fmt "\n" , __func__ , ## arg)


struct pwmg_data {
	struct pwm_device    *pwm;
	struct device        *dev;
	int                  id;
	unsigned int         period;
	unsigned int         period_min;
	unsigned int         period_max;
	unsigned int         lth_duty;
	unsigned long        duty;
	unsigned long        max_duty;
	int                  enable;
	struct mutex         ops_lock;
};


static int pwmg_update_status (struct  pwmg_data *pd) {

	int duty = pd->duty;
	int max = pd->max_duty;
	
	if (duty == 0 || pd->enable == 0) {
		pwm_config (pd->pwm, 0, pd->period);
		pwm_disable (pd->pwm);
	} else if (pd->enable == 1) {
		duty = pd->lth_duty +
			(duty * (pd->period - pd->lth_duty) / max);
		pwm_config (pd->pwm, duty, pd->period);
		pwm_enable (pd->pwm);
	}
	return 0;
}


static ssize_t pwmg_show_enable (struct device *dev,
		struct device_attribute *attr, char *buf) {
	
	struct platform_pwm_generic_data *data = dev->platform_data;
	struct pwmg_data *pd = (struct pwmg_data *)data->reserved;
	if (!pd) return 0;
	return sprintf(buf, "%d\n", pd->enable);
}


static ssize_t pwmg_store_enable (struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count) {

	int rc;
	unsigned long int  enable;
	struct platform_pwm_generic_data *data = dev->platform_data;
	struct pwmg_data *pd = (struct pwmg_data *)data->reserved;
	if (!pd) return 0;

	rc = strict_strtoul (buf, 0, &enable);
	if (rc)
		return rc;

	rc = -ENXIO;

	mutex_lock (&pd->ops_lock);
	
	if (enable == 1 || enable == 0) {
		pd->enable = enable;
		pwmg_update_status (pd);
		rc = count;
	}
	
	mutex_unlock (&pd->ops_lock);

	return rc;
}


static ssize_t pwmg_show_duty (struct device *dev,
		struct device_attribute *attr, char *buf) {
	
	struct platform_pwm_generic_data *data = dev->platform_data;
	struct pwmg_data *pd = (struct pwmg_data *)data->reserved;
	if (!pd) return 0;

	return sprintf(buf, "%lu\n", pd->duty);
}


static ssize_t pwmg_store_duty (struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count) {

	int rc;
	unsigned long duty;
	struct platform_pwm_generic_data *data = dev->platform_data;
	struct pwmg_data *pd = (struct pwmg_data *)data->reserved;
	if (!pd) return 0;

	rc = strict_strtoul (buf, 0, &duty);
	if (rc)
		return rc;

	rc = -ENXIO;

	mutex_lock (&pd->ops_lock);
	if (duty > pd->max_duty)
			rc = -EINVAL;
	else {
		pd->duty = duty;
		pwmg_update_status (pd);
		rc = count;
	}
	
	mutex_unlock (&pd->ops_lock);

	return rc;
}


static ssize_t pwmg_show_max_duty (struct device *dev,
		struct device_attribute *attr, char *buf) {

	struct platform_pwm_generic_data *data = dev->platform_data;
	struct pwmg_data *pd = (struct pwmg_data *)data->reserved;
	if (!pd) return 0;

	return sprintf(buf, "%lu\n", pd->max_duty);
}


static ssize_t pwmg_show_period (struct device *dev,
		struct device_attribute *attr, char *buf) {

	struct platform_pwm_generic_data *data = dev->platform_data;
	struct pwmg_data *pd = (struct pwmg_data *)data->reserved;
	if (!pd) return 0;

	return sprintf(buf, "%d\n", pd->period);
}


static ssize_t pwmg_store_period (struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count) {
	
	int rc;
	unsigned long period;
	struct platform_pwm_generic_data *data = dev->platform_data;
	struct pwmg_data *pd = (struct pwmg_data *)data->reserved;
	if (!pd) return 0;

	rc = strict_strtoul (buf, 0, &period);
	if (rc)
		return rc;

	rc = -ENXIO;

	mutex_lock (&pd->ops_lock);
	if ((period > pd->period_max) || (period < pd->period_min))
		rc = -EINVAL;
	else {
		pd->lth_duty /= pd->period;
		pd->period = period;
		pd->lth_duty = pd->lth_duty * pd->period;
		pwmg_update_status (pd);
		rc = count;
	}

	mutex_unlock (&pd->ops_lock);

	return rc;	
}


static ssize_t pwmg_show_max_period (struct device *dev,
		struct device_attribute *attr, char *buf) {

	struct platform_pwm_generic_data *data = dev->platform_data;
	struct pwmg_data *pd = (struct pwmg_data *)data->reserved;
	if (!pd) return 0;
	return sprintf(buf, "%d\n", pd->period_max);
}


static ssize_t pwmg_show_min_period (struct device *dev,
		struct device_attribute *attr, char *buf) {

	struct platform_pwm_generic_data *data = dev->platform_data;
	struct pwmg_data *pd = (struct pwmg_data *)data->reserved;
	if (!pd) return 0;

	return sprintf(buf, "%d\n", pd->period_min);
}


static DEVICE_ATTR (enable, 0644, pwmg_show_enable, pwmg_store_enable);
static DEVICE_ATTR (duty, 0644, pwmg_show_duty, pwmg_store_duty);
static DEVICE_ATTR (max_duty, 0444, pwmg_show_max_duty, NULL);
static DEVICE_ATTR (period_ns, 0644, pwmg_show_period, pwmg_store_period);
static DEVICE_ATTR (period_ns_max, 0444, pwmg_show_max_period, NULL);
static DEVICE_ATTR (period_ns_min, 0444, pwmg_show_min_period, NULL);


static int pwmg_generic_probe (struct platform_device *pdev) {

	struct platform_pwm_generic_data *data = pdev->dev.platform_data;
	int ret;
	struct pwmg_data *pd;

	if (!data) {
		PWMG_ERR ("failed to find platform data");
		return -EINVAL;
	}

	pd = kzalloc (sizeof(*pd), GFP_KERNEL);
	if (!pd) {
		PWMG_ERR ("no memory for state");
		ret = -ENOMEM;
		goto err_alloc;
	}

	pd->dev = &pdev->dev;

	pd->pwm = pwm_request(data->pwm_id, PWM_NAME_ID);
	if (IS_ERR(pd->pwm)) {
		PWMG_ERR ("unable to request PWM");
		ret = PTR_ERR(pd->pwm);
		goto err_pwm;
	} else
		PWMG_DBG ("got pwm for backlight\n");

	data->reserved = pd;

	pd->duty = data->dft_duty;
	pd->max_duty = data->max_duty;
	
	pd->period = data->pwm_period_ns;
	pd->period_max = data->period_ns_max;
	pd->period_min = data->period_ns_min;

	pd->lth_duty = data->lth_duty *
		(data->pwm_period_ns / data->max_duty);

	pd->enable = 1;

	mutex_init(&pd->ops_lock);

	ret = device_create_file (pd->dev, &dev_attr_enable);
	if (ret < 0)
		goto err_fs;
	ret = device_create_file (pd->dev, &dev_attr_duty);
	if (ret < 0)
		goto err_fs;
	ret = device_create_file (pd->dev, &dev_attr_max_duty);
	if (ret < 0)
		goto err_fs;
	ret = device_create_file (pd->dev, &dev_attr_period_ns);
	if (ret < 0)
		goto err_fs;
	ret = device_create_file (pd->dev, &dev_attr_period_ns_max);
	if (ret < 0)
		goto err_fs;
	ret = device_create_file (pd->dev, &dev_attr_period_ns_min);
	if (ret < 0)
		goto err_fs;

	pwmg_update_status (pd);
	
	return 0;

err_fs:
	pwm_free(pd->pwm);
err_pwm:
	kfree(pd);
err_alloc:
	return ret;
}


static int pwmg_generic_remove(struct platform_device *pdev) {

	struct platform_pwm_generic_data *data = pdev->dev.platform_data;
	struct pwmg_data *pd = (struct pwmg_data *)data->reserved;

	pwm_config (pd->pwm, 0, pd->period);
        pwm_disable (pd->pwm);
        pwm_free (pd->pwm);
        kfree (pd);
	return 0;
}


static int pwmg_generic_suspend(struct platform_device *pdev,
				 pm_message_t state) {
	return 0;
}


static int pwmg_generic_resume (struct platform_device *pdev) {
	return 0;
}


static struct platform_driver pwmg_generic_driver = {
	.driver		= {
		.name	= "pwm-generic",
		.owner	= THIS_MODULE,
	},
	.probe		= pwmg_generic_probe,
	.remove		= pwmg_generic_remove,
	.suspend	= pwmg_generic_suspend,
	.resume		= pwmg_generic_resume,
};


static int __init pwmg_generic_init (void) {
	return platform_driver_register (&pwmg_generic_driver);
}
module_init(pwmg_generic_init);


static void __exit pwmg_generic_exit (void) {
	platform_driver_unregister (&pwmg_generic_driver);
}
module_exit(pwmg_generic_exit);


MODULE_AUTHOR("DC SECO");
MODULE_DESCRIPTION("SECO generic PWM Driver");
MODULE_LICENSE("GPL");


