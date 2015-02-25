/*
 * Logic PWM data - see drivers/seco/pwm_seco.h
 */
#ifndef __LINUX_PWM_SECO_H
#define __LINUX_PWM_SECO_H


#define PWM_SECO_MAX_PERIOD_NS 1000000000    // 1 s
#define PWM_SECO_MIN_PERIOD_NS 150	     // 150 ns
#define PMW_SECO_MAX_DUTY      100

struct platform_pwm_seco_data {
	unsigned int   enable;
	unsigned int   dft_duty;
	unsigned int   dft_polarity;
	unsigned int   pwm_period_ns;
	void           *reserved;
};

#endif

