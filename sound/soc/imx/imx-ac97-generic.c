/*
 * sound/soc/imx/imx-ac97-generic.c --  SoC audio for i.MX Seco Q7 boards with
 *                                     AC97 generic codec
 *
 * Copyright 2014 Seco s.r.l.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/fsl_devices.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <asm/mach-types.h>
#include <mach/audmux.h>


#include "../codecs/generic.h"
#include "imx-ssi.h"

static struct imx_generic_priv {
        int sysclk;
        int hw;
        struct platform_device *pdev;
} card_priv;

static struct snd_soc_card imx_generic;

static int generic_params(struct snd_pcm_substream *substream,
        struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	u32 dai_format;
	int ret;
	unsigned int channels = params_channels(params);

	snd_soc_dai_set_sysclk(codec_dai, GENERIC_SYSCLK, 48000000, 0);

        snd_soc_dai_set_sysclk(codec_dai, GENERIC_LRCLK, params_rate(params), 0);

	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
            SND_SOC_DAIFMT_CBM_CFM;

	/* set codec DAI configuration */
        ret = snd_soc_dai_set_fmt(codec_dai, dai_format);
        if (ret < 0)
                return ret;

// Following settings stop AC97 flow in case of MX6 soc. So, exiting now. [gp000q7]
return 0;

	/* TODO: The SSI driver should figure this out for us */
	switch (channels) {
	case 2:
		snd_soc_dai_set_tdm_slot(cpu_dai, 0xfffffffc, 0xfffffffc, 2, 0);
		break;
	case 1:
		snd_soc_dai_set_tdm_slot(cpu_dai, 0xfffffffe, 0xfffffffe, 1, 0);
		break;
	default:
		return -EINVAL;
	}

	/* set cpu DAI configuration */
	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_IF |
            SND_SOC_DAIFMT_CBM_CFM;
        ret = snd_soc_dai_set_fmt(cpu_dai, dai_format);
        if (ret < 0)
                return ret;

	return 0;
}

static struct snd_soc_ops imx_generic_hifi_ops = {
	.hw_params = generic_params,
};

static int generic_jack_func;
static int generic_spk_func;
static int generic_line_in_func;

static const char *jack_function[] = { "off", "on"};

static const char *spk_function[] = { "off", "on" };

static const char *line_in_function[] = { "off", "on" };

static const struct soc_enum generic_enum[] = {
	SOC_ENUM_SINGLE_EXT(2, jack_function),
	SOC_ENUM_SINGLE_EXT(2, spk_function),
	SOC_ENUM_SINGLE_EXT(2, line_in_function),
};

static int generic_get_jack(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = generic_jack_func;
	return 0;
}

static int generic_set_jack(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	if (generic_jack_func == ucontrol->value.enumerated.item[0])
		return 0;

	generic_jack_func = ucontrol->value.enumerated.item[0];
	if (generic_jack_func)
		snd_soc_dapm_enable_pin(&codec->dapm, "Headphone Jack");
	else
		snd_soc_dapm_disable_pin(&codec->dapm, "Headphone Jack");

//	if (generic_jack_func)
//		snd_soc_dapm_enable_pin(codec, "Headphone Jack");
//	else
//		snd_soc_dapm_disable_pin(codec, "Headphone Jack");
//	snd_soc_dapm_sync(codec);
	snd_soc_dapm_sync(&codec->dapm);
	return 1;
}

static int generic_get_spk(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = generic_spk_func;
	return 0;
}

static int generic_set_spk(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	if (generic_spk_func == ucontrol->value.enumerated.item[0])
		return 0;

	generic_spk_func = ucontrol->value.enumerated.item[0];
	if (generic_spk_func)
		snd_soc_dapm_enable_pin(&codec->dapm, "Ext Spk");
	else
		snd_soc_dapm_disable_pin(&codec->dapm, "Ext Spk");

//	if (generic_spk_func)
//		snd_soc_dapm_enable_pin(codec, "Ext Spk");
//	else
//		snd_soc_dapm_disable_pin(codec, "Ext Spk");
//	snd_soc_dapm_sync(codec);
	snd_soc_dapm_sync(&codec->dapm);
	return 1;
}

static int generic_get_line_in(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = generic_line_in_func;
	return 0;
}

static int generic_set_line_in(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	if (generic_line_in_func == ucontrol->value.enumerated.item[0])
		return 0;

	generic_line_in_func = ucontrol->value.enumerated.item[0];
	if (generic_line_in_func)
		snd_soc_dapm_enable_pin(&codec->dapm, "Line In Jack");
	else
		snd_soc_dapm_disable_pin(&codec->dapm, "Line In Jack");

//	if (generic_line_in_func)
//		snd_soc_dapm_enable_pin(codec, "Line In Jack");
//	else
//		snd_soc_dapm_disable_pin(codec, "Line In Jack");
//	snd_soc_dapm_sync(codec);
	snd_soc_dapm_sync(&codec->dapm);
	return 1;
}

/* imx_3stack card dapm widgets */
static const struct snd_soc_dapm_widget imx_3stack_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_LINE("Line In Jack", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
};

static const struct snd_kcontrol_new generic_machine_controls[] = {
	SOC_ENUM_EXT("Jack Function", generic_enum[0], generic_get_jack,
		     generic_set_jack),
	SOC_ENUM_EXT("Speaker Function", generic_enum[1], generic_get_spk,
		     generic_set_spk),
	SOC_ENUM_EXT("Line In Function", generic_enum[1], generic_get_line_in,
		     generic_set_line_in),
};

/* imx_3stack machine connections to the codec pins */
static const struct snd_soc_dapm_route audio_map[] = {

	/* Mic Jack --> MIC_IN (with automatic bias) */
	{"MIC_IN", NULL, "Mic Jack"},

	/* Line in Jack --> LINE_IN */
	{"LINE_IN", NULL, "Line In Jack"},

	/* HP_OUT --> Headphone Jack */
	{"Headphone Jack", NULL, "HP_OUT"},

	/* LINE_OUT --> Ext Speaker */
	{"Ext Spk", NULL, "LINE_OUT"},
};

static int imx_seco_q7_generic_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	int ret;

	ret = snd_soc_add_controls(codec, generic_machine_controls,
                                ARRAY_SIZE(generic_machine_controls));
	if (ret)
		return ret;

	/* Add imx_3stack specific widgets */
	snd_soc_dapm_new_controls(dapm, imx_3stack_dapm_widgets,
				  ARRAY_SIZE(imx_3stack_dapm_widgets));

	/* Set up imx_3stack specific audio path audio_map */
//	snd_soc_dapm_add_routes(dapm, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_disable_pin(dapm, "Line In Jack");
	snd_soc_dapm_enable_pin(dapm, "Headphone Jack");

	snd_soc_dapm_sync(dapm);

	return 0;
}

static struct snd_soc_dai_link imx_generic_dai[] = {
	{
		.name		= "HiFi",
		.stream_name	= "HiFi",
		.codec_dai_name	= "generic",
		.codec_name	= "generic-ac97.0",
		.cpu_dai_name	= "imx-ssi.0",
		.platform_name	= "imx-pcm-audio.0",
		.init		= imx_seco_q7_generic_init,
		.ops		= &imx_generic_hifi_ops,
	},
};

/* Audio machine driver */
static struct snd_soc_card imx_generic = {
	.name		= "generic-audio",
	.dai_link	= imx_generic_dai,
	.num_links	= ARRAY_SIZE(imx_generic_dai),
};

static struct platform_device *imx_generic_snd_device;

static int imx_audmux_ac97_config(int slave, int master)
{
        unsigned int ptcr, pdcr;
        slave = slave - 1;
        master = master - 1;

	ptcr = MXC_AUDMUX_V2_PTCR_SYN |
                        MXC_AUDMUX_V2_PTCR_TCLKDIR |
                        MXC_AUDMUX_V2_PTCR_TCSEL(master);
	pdcr = MXC_AUDMUX_V2_PDCR_RXDSEL(master);
        mxc_audmux_v2_configure_port(slave, ptcr, pdcr);

	ptcr = MXC_AUDMUX_V2_PTCR_SYN |
                        MXC_AUDMUX_V2_PTCR_TFSDIR |
                        MXC_AUDMUX_V2_PTCR_TFSEL(slave);
	pdcr = MXC_AUDMUX_V2_PDCR_RXDSEL(slave);
        mxc_audmux_v2_configure_port(master, ptcr, pdcr);

        return 0;
}

static int __devinit imx_generic_probe(struct platform_device *pdev)
{
        struct mxc_audio_platform_data *plat = pdev->dev.platform_data;

        card_priv.pdev = pdev;
        imx_audmux_ac97_config(plat->src_port, plat->ext_port);
        return 0;
}


static int imx_generic_remove(struct platform_device *pdev)
{
        struct mxc_audio_platform_data *plat = pdev->dev.platform_data;

        if (plat->finit)
                plat->finit();
        return 0;
}

static struct platform_driver imx_generic_audio_driver = {
        .probe = imx_generic_probe,
        .remove = imx_generic_remove,
        .driver = {
                   .name = "imx-generic",
                   },
};

static int __init imx_generic_init(void)
{
	int ret;


        ret = platform_driver_register(&imx_generic_audio_driver);
        if (ret)
                return -ENOMEM;

	imx_generic_snd_device = platform_device_alloc("soc-audio", -1);
	if (!imx_generic_snd_device)
		return -ENOMEM;

	platform_set_drvdata(imx_generic_snd_device, &imx_generic);

	ret = platform_device_add(imx_generic_snd_device);

	if (ret) {
		printk(KERN_ERR "ASoC: Platform device allocation failed\n");
		platform_device_put(imx_generic_snd_device);
	}

	return ret;
}

static void __exit imx_generic_exit(void)
{
	platform_device_unregister(imx_generic_snd_device);
}

//late_initcall(imx_generic_init);
module_init(imx_generic_init);
module_exit(imx_generic_exit);

MODULE_AUTHOR("Seco <info@seco.it>");
MODULE_DESCRIPTION("Mx6 Seco Q7 ALSA SoC driver");
MODULE_LICENSE("GPL");
