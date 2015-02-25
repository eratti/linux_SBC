#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/pwm_seco.h>
#include <linux/seco_cpld.h>
#include <linux/slab.h>


#define PWM_LPC_REG1   CPLD_REG_6
#define PWM_LPC_REG2   CPLD_REG_7
#define PWM_LPC_REG3   CPLD_REG_8

#define PWM_GPIO_REG1   CPLD_REG_8
#define PWM_GPIO_REG2   CPLD_REG_9
#define PWM_GPIO_REG3   CPLD_REG_10


#define PWM_MASK_EN         0x8000
#define PWM_MASK_POL        0x4000
#define PWM_MASK_PRESCALE   0x3FFF
#define PWM_MASK_PERIOD     0xFFFF
#define PWM_MASK_DUTY       0xFFFF

#define MAIN_CLK      33000000    // (Hz)

#define PWM_NAME_ID   "pwm_seco"

#define PWM_INFO(fmt, arg...) printk(KERN_INFO "PWM driver: " fmt "\n" , ## arg)
#define PWM_ERR(fmt, arg...)  printk(KERN_ERR "%s: " fmt "\n" , __func__ , ## arg)
#define PWM_DBG(fmt, arg...)  pr_debug("%s: " fmt "\n" , __func__ , ## arg)


struct register_map {
	u8  PWM_REG_ENABLE;
	u8  PWM_REG_POLARITY;
	u8  PWM_REG_PRESCALE;
	u8  PWM_REG_PERIOD;
	u8  PWM_REG_DUTY;
};

static struct register_map reg_map;

typedef enum {
	POL_DIR = 0,
	POL_INV = 1,
} POLARITY;


struct pwm_device {
	int                  clk_rate;
	unsigned int         prescale;
	unsigned int         period;
	int                  enable;
	unsigned long        duty;
	POLARITY             polarity;             
};


struct pwmg_data {
	struct pwm_device    *pwm;
	struct device        *dev;
	unsigned int         period_ns;
	unsigned int         duty;
	unsigned int         period_min;
	unsigned int         period_max;
	unsigned int         lth_duty;
	unsigned long        max_duty;
	struct mutex         ops_lock;
};

static int seco_pwm_writeb (unsigned int reg, int value) {
        int ret = 0;
        ret = cpld_reg_write (reg, value);
        if (ret < 0) {
                PWM_ERR("write operation failed!");
        }
        PWM_DBG ("write complite with code %d", ret);
        return ret;
}


static int seco_pwm_readb (unsigned int reg, int *data) {
        int ret = 0;
        ret = cpld_reg_read (reg, data);
        if (reg < 0) {
        	PWM_ERR ("read operation failed!");
        }
        PWM_DBG ("read operation on register %2x complite, read value %2x, returned code %d", reg, *data, ret);
        return ret;
}


static int pwm_get_val (unsigned int reg, unsigned int mask) {
	int val;
	seco_pwm_readb (reg, &val);
	return val & mask;
}


#define PWM_ENABLE_GET (en)            (en)       = !!(pwm_get_val (reg_map.PWM_REG_ENABLE, PMW_MASK_EN))
#define PWM_POLARITY_GET (pol)         (pol)      = !!(pwm_get_val (reg_map.PWM_REG_POLARITY, PMW_MASK_POL))
#define PWM_PRESCALE_GET (prescale)    (prescale) = (pwm_get_val (reg_map.PWM_REG_PRESCALE, PWM_MASK_PRESCALE))
#define PWM_PERIOD_GET (per)           (per)      = (pwm_get_val (reg_map.PWM_REG_PERIOD, PWM_MASK_PERIOD))
#define PWM_DUTY_GET (duty)            (duty)     = (pwm_get_val (reg_map.PWM_REG_DUTY, PWM_MASK_DUTY))



int seco_pwm_enable (struct pwm_device *pwm) {
	int val_reg = pwm_get_val (reg_map.PWM_REG_ENABLE, 0xFFFF);
	return seco_pwm_writeb (reg_map.PWM_REG_ENABLE, val_reg | PWM_MASK_EN);
}


void seco_pwm_disable (struct pwm_device *pwm) {
	int val_reg = pwm_get_val (reg_map.PWM_REG_ENABLE, 0xFFFF);
	seco_pwm_writeb (reg_map.PWM_REG_ENABLE, val_reg & ~PWM_MASK_EN);
}


int seco_pwm_pol_inv (struct pwm_device *pwm) {
	int val_reg = pwm_get_val (reg_map.PWM_REG_POLARITY, 0xFFFF);
	return seco_pwm_writeb (reg_map.PWM_REG_POLARITY, val_reg & ~PWM_MASK_POL);
}


void seco_pwm_pol_dir (struct pwm_device *pwm) {
	int val_reg = pwm_get_val (reg_map.PWM_REG_POLARITY, 0xFFFF);
	seco_pwm_writeb (reg_map.PWM_REG_POLARITY, val_reg | PWM_MASK_POL);
}


int seco_pwm_config (struct pwm_device *pwm, int period_ns, int duty) {
	unsigned long period_cycles, duty_cycles, prescale;
	unsigned long long clk1;
	unsigned long long nclk;

	int val_reg;

	if (pwm == NULL || period_ns == 0)
		return -EINVAL;

	prescale = 0;
		
	do {
		clk1 = (MAIN_CLK >> 1);
		do_div (clk1, (prescale + 1));
		nclk = period_ns * clk1;	
		do_div (nclk, 1000000000);	
		nclk--;
		prescale++;
	} while (nclk & ~((unsigned long long)0xFFFF));
	prescale--;
		
	period_cycles = nclk;
	duty_cycles = nclk * duty;
	do_div (duty_cycles, 100);

	pwm->prescale = (unsigned int)prescale;
	pwm->period = (unsigned int)period_cycles;
	pwm->duty = (unsigned int)duty_cycles;

	seco_pwm_writeb (reg_map.PWM_REG_PERIOD, pwm->period  & PWM_MASK_PERIOD);
	seco_pwm_writeb (reg_map.PWM_REG_DUTY, pwm->duty & PWM_MASK_DUTY);
	val_reg = pwm_get_val (reg_map.PWM_REG_PRESCALE, PWM_MASK_EN | PWM_MASK_POL);
	seco_pwm_writeb (reg_map.PWM_REG_PRESCALE, val_reg | (PWM_MASK_PRESCALE & (~PWM_MASK_PRESCALE | pwm->prescale)));
	return 0;
}


static ssize_t pwmg_show_enable (struct device *dev,
		struct device_attribute *attr, char *buf) {
	
	int en;
	struct platform_pwm_seco_data *data = dev->platform_data;
	struct pwmg_data *pd = (struct pwmg_data *)data->reserved;
	if (!pd) return 0;
	en = !!(pwm_get_val (reg_map.PWM_REG_ENABLE, PWM_MASK_EN));
	return sprintf(buf, "%s\n", pd->pwm->enable ? "enable" : "disable");
}


static ssize_t pwmg_store_enable (struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count) {

	int rc;
	unsigned long int  enable;
	struct platform_pwm_seco_data *data = dev->platform_data;
	struct pwmg_data *pd = (struct pwmg_data *)data->reserved;
	if (!pd) return 0;

	rc = strict_strtoul (buf, 0, &enable);
	if (rc)
		return rc;

	rc = -ENXIO;

	mutex_lock (&pd->ops_lock);
	
	if (enable == 1 || enable == 0) {
		pd->pwm->enable = enable;
		if (enable == 1)
			seco_pwm_enable (pd->pwm);		
		else
			seco_pwm_disable (pd->pwm);
		seco_pwm_config (pd->pwm, pd->period_ns, pd->duty);
		rc = count;
	}
	
	mutex_unlock (&pd->ops_lock);

	return rc;
}


static ssize_t pwmg_show_polarity (struct device *dev,
		struct device_attribute *attr, char *buf) {
	
	struct platform_pwm_seco_data *data = dev->platform_data;
	struct pwmg_data *pd = (struct pwmg_data *)data->reserved;
	if (!pd) return 0;
	return sprintf(buf, "%s\n", pd->pwm->polarity == POL_DIR ? "direct" : "inverse");
}


static ssize_t pwmg_store_polarity (struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count) {

	int rc;
	unsigned long int  polarity;
	struct platform_pwm_seco_data *data = dev->platform_data;
	struct pwmg_data *pd = (struct pwmg_data *)data->reserved;
	if (!pd) return 0;

	rc = strict_strtoul (buf, 0, &polarity);
	if (rc)
		return rc;
	rc = -ENXIO;

	mutex_lock (&pd->ops_lock);
	
	if (polarity == 1 || polarity == 0) {
		pd->pwm->polarity = (POLARITY)polarity;
		if (polarity == 0)
			seco_pwm_pol_dir (pd->pwm);		
		else
			seco_pwm_pol_inv (pd->pwm);

		seco_pwm_config (pd->pwm, pd->period_ns, pd->duty);
		rc = count;
	}
	
	mutex_unlock (&pd->ops_lock);

	return rc;
}


static ssize_t pwmg_show_duty (struct device *dev,
		struct device_attribute *attr, char *buf) {
	
	struct platform_pwm_seco_data *data = dev->platform_data;
	struct pwmg_data *pd = (struct pwmg_data *)data->reserved;
		if (!pd) return 0;

	return sprintf(buf, "%d\n", pd->duty);
}


static ssize_t pwmg_store_duty (struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count) {

	int rc;
	unsigned long duty;
	struct platform_pwm_seco_data *data = dev->platform_data;
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
		seco_pwm_config (pd->pwm, pd->period_ns, pd->duty);
		rc = count;
	}
	
	mutex_unlock (&pd->ops_lock);

	return rc;
}

static ssize_t pwmg_show_max_duty (struct device *dev,
		struct device_attribute *attr, char *buf) {

	struct platform_pwm_seco_data *data = dev->platform_data;
	struct pwmg_data *pd = (struct pwmg_data *)data->reserved;
	if (!pd) return 0;

	return sprintf(buf, "%lu\n", pd->max_duty);
}


static ssize_t pwmg_show_period (struct device *dev,
		struct device_attribute *attr, char *buf) {

	struct platform_pwm_seco_data *data = dev->platform_data;
	struct pwmg_data *pd = (struct pwmg_data *)data->reserved;
	if (!pd) return 0;

	return sprintf(buf, "%d\n", pd->period_ns);
}


static ssize_t pwmg_store_period (struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count) {
	
	int rc;
	unsigned long period;
	struct platform_pwm_seco_data *data = dev->platform_data;
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
		pd->period_ns = period;
		seco_pwm_config (pd->pwm, pd->period_ns, pd->duty);

		rc = count;
	}

	mutex_unlock (&pd->ops_lock);

	return rc;	
}


static ssize_t pwmg_show_max_period (struct device *dev,
		struct device_attribute *attr, char *buf) {

	struct platform_pwm_seco_data *data = dev->platform_data;
	struct pwmg_data *pd = (struct pwmg_data *)data->reserved;
	if (!pd) return 0;
	return sprintf(buf, "%d\n", pd->period_max);
}


static ssize_t pwmg_show_min_period (struct device *dev,
		struct device_attribute *attr, char *buf) {

	struct platform_pwm_seco_data *data = dev->platform_data;
	struct pwmg_data *pd = (struct pwmg_data *)data->reserved;
	if (!pd) return 0;

	return sprintf(buf, "%d\n", pd->period_min);
}


static DEVICE_ATTR (enable, 0644, pwmg_show_enable, pwmg_store_enable);
static DEVICE_ATTR (polarity, 0644, pwmg_show_polarity, pwmg_store_polarity);
static DEVICE_ATTR (duty, 0644, pwmg_show_duty, pwmg_store_duty);
static DEVICE_ATTR (max_duty, 0444, pwmg_show_max_duty, NULL);
static DEVICE_ATTR (period_ns, 0644, pwmg_show_period, pwmg_store_period);
static DEVICE_ATTR (period_ns_max, 0444, pwmg_show_max_period, NULL);
static DEVICE_ATTR (period_ns_min, 0444, pwmg_show_min_period, NULL);


static int pwmg_seco_probe (struct platform_device *pdev) {

	struct platform_pwm_seco_data *data = pdev->dev.platform_data;
	int ret;
	struct pwmg_data *pd;

	if (!data) {
		PWM_ERR ("failed to find platform data");
		return -EINVAL;
	}

	pd = kzalloc (sizeof(*pd), GFP_KERNEL);
	if (!pd) {
		PWM_ERR ("no memory for state");
		ret = -ENOMEM;
		goto err_alloc;
	}

	pd->dev = &pdev->dev;

	pd->pwm = kzalloc (sizeof (struct pwm_device), GFP_KERNEL);
	if (!pd->pwm) {
		PWM_ERR ("no pwm data allocated");
		ret = -ENOMEM;
		goto err_pwm;
	}

	if (cpld_is_gpio ()) {
		reg_map.PWM_REG_ENABLE   = PWM_GPIO_REG1;
		reg_map.PWM_REG_POLARITY = PWM_GPIO_REG1;
		reg_map.PWM_REG_PRESCALE = PWM_GPIO_REG1;
		reg_map.PWM_REG_PERIOD   = PWM_GPIO_REG2;
		reg_map.PWM_REG_DUTY     = PWM_GPIO_REG3;
	} else {
		reg_map.PWM_REG_ENABLE   = PWM_LPC_REG1;
		reg_map.PWM_REG_POLARITY = PWM_LPC_REG1;
		reg_map.PWM_REG_PRESCALE = PWM_LPC_REG1;
		reg_map.PWM_REG_PERIOD   = PWM_LPC_REG2;
		reg_map.PWM_REG_DUTY     = PWM_LPC_REG3;
	}
	

        data->reserved = pd;

	pd->max_duty   = PMW_SECO_MAX_DUTY;
	pd->period_max = PWM_SECO_MAX_PERIOD_NS;
	pd->period_min = PWM_SECO_MIN_PERIOD_NS;

	pd->pwm->clk_rate = MAIN_CLK;

	pd->period_ns = data->pwm_period_ns;

	if (data->pwm_period_ns > pd->period_max)
		pd->period_ns = pd->period_max;

	if (data->pwm_period_ns < pd->period_min)
		pd->period_ns = pd->period_min;

	if (data->dft_duty > 100)
		data->dft_duty = 100;
		
	pd->duty = data->dft_duty;

	if (data->dft_polarity == 0)
		pd->pwm->polarity = POL_DIR;
	
	if (data->dft_polarity == 1)
		pd->pwm->polarity = POL_INV;

	seco_pwm_config (pd->pwm, pd->period_ns, pd->duty);

	if (data->enable > 0) {
		seco_pwm_enable (pd->pwm);
		pd->pwm->enable = 1;
	} else {
		seco_pwm_disable (pd->pwm);
		pd->pwm->enable = 0;
	}

	mutex_init(&pd->ops_lock);

	ret = device_create_file (pd->dev, &dev_attr_enable);
	if (ret < 0)
		goto err_fs;
	ret = device_create_file (pd->dev, &dev_attr_polarity);
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

	printk (KERN_INFO "SECO PWM probe done\n");	
	return 0;

err_fs:
	device_remove_file (pd->dev, &dev_attr_enable);
	device_remove_file (pd->dev, &dev_attr_polarity);
	device_remove_file (pd->dev, &dev_attr_duty);
	device_remove_file (pd->dev, &dev_attr_max_duty);
	device_remove_file (pd->dev, &dev_attr_period_ns);
	device_remove_file (pd->dev, &dev_attr_period_ns_max);
	device_remove_file (pd->dev, &dev_attr_period_ns_min);
err_pwm:
	kfree(pd);
err_alloc:
	return ret;
}


static int pwmg_seco_remove(struct platform_device *pdev) {

	struct platform_pwm_seco_data *data = pdev->dev.platform_data;
	struct pwmg_data *pd = (struct pwmg_data *)data->reserved;

	device_remove_file (pd->dev, &dev_attr_enable);
	device_remove_file (pd->dev, &dev_attr_polarity);
	device_remove_file (pd->dev, &dev_attr_duty);
	device_remove_file (pd->dev, &dev_attr_max_duty);
	device_remove_file (pd->dev, &dev_attr_period_ns);
	device_remove_file (pd->dev, &dev_attr_period_ns_max);
	device_remove_file (pd->dev, &dev_attr_period_ns_min);

	seco_pwm_config (pd->pwm, 0, pd->pwm->period);
        seco_pwm_disable (pd->pwm);

	kfree (pd->pwm);
        kfree (pd);
	return 0;
}


static int pwmg_seco_suspend(struct platform_device *pdev,
				 pm_message_t state) {
	return 0;
}


static int pwmg_seco_resume (struct platform_device *pdev) {
	return 0;
}


static struct platform_driver pwmg_seco_driver = {
	.driver		= {
		.name	= "pwm-seco",
		.owner	= THIS_MODULE,
	},
	.probe		= pwmg_seco_probe,
	.remove		= pwmg_seco_remove,
	.suspend	= pwmg_seco_suspend,
	.resume		= pwmg_seco_resume,
};


static int __init pwmg_seco_init (void) {
	return platform_driver_register (&pwmg_seco_driver);
}
module_init(pwmg_seco_init);


static void __exit pwmg_seco_exit (void) {
	platform_driver_unregister (&pwmg_seco_driver);
}
module_exit(pwmg_seco_exit);


MODULE_AUTHOR("DC SECO");
MODULE_DESCRIPTION("SECO Logic PWM Driver");
MODULE_LICENSE("GPL");


