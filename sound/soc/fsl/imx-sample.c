// SPDX-License-Identifier: GPL-2.0
//
// ASoC DPCM Machine driver for Baytrail / Cherrytrail platforms with
// CX2072X codec
//

#include <linux/acpi.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/jack.h>
#include <sound/soc.h>
#include <sound/soc-of.h>

static const struct snd_kcontrol_new sample_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphone"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),
	SOC_DAPM_PIN_SWITCH("Int Mic"),
	SOC_DAPM_PIN_SWITCH("Ext Spk"),
};

static const struct snd_soc_dapm_widget sample_widgets[] = {
       SND_SOC_DAPM_LINE("Line Out Jack", NULL),
       SND_SOC_DAPM_LINE("Line In Jack", NULL),
};

static const struct snd_soc_dapm_route sample_audio_map[] = {
       /* Line out jack */
      {"Line Out Jack", NULL, "AOUT1L"},
       {"Line Out Jack", NULL, "AOUT1R"},
       {"Line Out Jack", NULL, "AOUT2L"},
       {"Line Out Jack", NULL, "AOUT2R"},
       {"Line Out Jack", NULL, "AOUT3L"},
       {"Line Out Jack", NULL, "AOUT3R"},
       {"Line Out Jack", NULL, "AOUT4L"},
       {"Line Out Jack", NULL, "AOUT4R"},
       {"AIN1L", NULL, "Line In Jack"},
       {"AIN1R", NULL, "Line In Jack"},
       {"AIN2L", NULL, "Line In Jack"},
       {"AIN2R", NULL, "Line In Jack"},
       {"Playback",  NULL, "ESAI0.OUT"},/* dai route for be and fe */
       {"CPU-Capture",  NULL, "Capture"},
};

static struct snd_soc_jack sample_headset;

/* Headset jack detection DAPM pins */
static struct snd_soc_jack_pin sample_headset_pins[] = {
	{
		.pin = "Headset Mic",
		.mask = SND_JACK_MICROPHONE,
	},
	{
		.pin = "Headphone",
		.mask = SND_JACK_HEADPHONE,
	},
};

static const struct acpi_gpio_params sample_headset_gpios;
static const struct acpi_gpio_mapping sample_acpi_gpios[] = {
	{ "headset-gpios", &sample_headset_gpios, 1 },
	{},
};
#if 0
static int sample_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card *card = rtd->card;
	struct snd_soc_component *codec = rtd->codec_dai->component;
	int ret;

	if (devm_acpi_dev_add_driver_gpios(codec->dev,
					   sample_acpi_gpios))
		dev_warn(rtd->dev, "Unable to add GPIO mapping table\n");

	card->dapm.idle_bias_off = true;

	/* set the default PLL rate, the clock is handled by the codec driver */
	ret = snd_soc_dai_set_sysclk(rtd->codec_dai, CX2072X_MCLK_EXTERNAL_PLL,
				     19200000, SND_SOC_CLOCK_IN);
	if (ret) {
		dev_err(rtd->dev, "Could not set sysclk\n");
		return ret;
	}

	ret = snd_soc_card_jack_new(card, "Headset",
				    SND_JACK_HEADSET | SND_JACK_BTN_0,
				    &sample_headset,
				    sample_headset_pins,
				    ARRAY_SIZE(sample_headset_pins));
	if (ret)
		return ret;

	snd_soc_component_set_jack(codec, &sample_headset, NULL);

	snd_soc_dai_set_bclk_ratio(rtd->codec_dai, 50);

	return ret;
}
#endif
static int sample_fixup(struct snd_soc_pcm_runtime *rtd,
				 struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate =
		hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels =
		hw_param_interval(params, SNDRV_PCM_HW_PARAM_CHANNELS);
	int ret;

	/* The DSP will covert the FE rate to 48k, stereo, 24bits */
	rate->min = rate->max = 48000;
	channels->min = channels->max = 2;

	/* set SSP2 to 24-bit */
	params_set_format(params, SNDRV_PCM_FORMAT_S24_LE);

	/*
	 * Default mode for SSP configuration is TDM 4 slot, override config
	 * with explicit setting to I2S 2ch 24-bit. The word length is set with
	 * dai_set_tdm_slot() since there is no other API exposed
	 */
	ret = snd_soc_dai_set_fmt(rtd->cpu_dai,
				SND_SOC_DAIFMT_I2S     |
				SND_SOC_DAIFMT_NB_NF   |
				SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0) {
		dev_err(rtd->dev, "can't set format to I2S, err %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_tdm_slot(rtd->cpu_dai, 0x3, 0x3, 2, 24);
	if (ret < 0) {
		dev_err(rtd->dev, "can't set I2S config, err %d\n", ret);
		return ret;
	}

	return 0;
}

static int sample_aif1_startup(struct snd_pcm_substream *substream)
{
	return snd_pcm_hw_constraint_single(substream->runtime,
					    SNDRV_PCM_HW_PARAM_RATE, 48000);
}

static struct snd_soc_ops sample_aif1_ops = {
	.startup = sample_aif1_startup,
};

SND_SOC_DAILINK_DEF(dummy,
	DAILINK_COMP_ARRAY(COMP_DUMMY()));

SND_SOC_DAILINK_DEF(media,
	DAILINK_COMP_ARRAY(COMP_CPU("media-cpu-dai")));

SND_SOC_DAILINK_DEF(deepbuffer,
	DAILINK_COMP_ARRAY(COMP_CPU("deepbuffer-cpu-dai")));

SND_SOC_DAILINK_DEF(esai0,
	DAILINK_COMP_ARRAY(COMP_CPU("")));

SND_SOC_DAILINK_DEF(cs42888,
	DAILINK_COMP_ARRAY(COMP_CODEC("cs42xx8.2-0048", "cs42888")));

SND_SOC_DAILINK_DEF(platform,
	DAILINK_COMP_ARRAY(COMP_PLATFORM("sst-mfld-platform")));

SND_SOC_DAILINK_DEFS(imx8,
		      DAILINK_COMP_ARRAY(COMP_CPU("596e8000.dsp")),
		      DAILINK_COMP_ARRAY(COMP_CODEC("cs42xx8.2-0048", "cs42888")),
		      DAILINK_COMP_ARRAY(COMP_PLATFORM("596e8000.dsp")));

static struct snd_soc_dai_link sample_dais[] = {
	{
		.name =  "DSP-Audio",
		.stream_name = "Playback",
		.nonatomic = true,
		.dynamic = 1,
		.dpcm_playback = 1,
		.ops = &sample_aif1_ops,
		SND_SOC_DAILINK_REG(deepbuffer, dummy, dummy),
	},
	/* back ends */
	{
		.name = "ESAI0-Codec",
		.id = 0,
		.no_pcm = 1,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF
					      | SND_SOC_DAIFMT_CBS_CFS,
	///	.init = sample_init,
	//	.be_hw_params_fixup = sample_fixup,
		.nonatomic = true,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(imx8),
	},
};

/* SoC card */
static struct snd_soc_card sample_card = {
	.name = "imx-sample",
	.owner = THIS_MODULE,
	.dai_link = sample_dais,
	.num_links = ARRAY_SIZE(sample_dais),
	.dapm_widgets = sample_widgets,
	.num_dapm_widgets = ARRAY_SIZE(sample_widgets),
	.dapm_routes = sample_audio_map,
	.num_dapm_routes = ARRAY_SIZE(sample_audio_map),
	//.controls = sample_controls,
	//.num_controls = ARRAY_SIZE(sample_controls),
};

static int snd_sample_probe(struct platform_device *pdev)
{
	int dai_index = 0;
	int i, ret;
	struct device *dev = &pdev->dev;
	struct snd_soc_of_mach *mach = dev_get_platdata(dev);
	pr_info("imx-sample called pdev %px, mach %px name %s\n", pdev, mach, mach->common.drv_name);

	sample_card.dev = &pdev->dev;

#if 0
	/* fix index of codec dai */
	for (i = 0; i < ARRAY_SIZE(sample_dais); i++) {
		if (!strcmp(sample_dais[i].codecs->name,
			    "i2c-14F10720:00")) {
			dai_index = i;
			break;
		}
	}

	/* fixup codec name based on HID */
	adev = acpi_dev_get_first_match_dev(mach->id, NULL, -1);
	if (adev) {
		snprintf(codec_name, sizeof(codec_name), "i2c-%s",
			 acpi_dev_name(adev));
		put_device(&adev->dev);
		sample_dais[dai_index].codecs->name = codec_name;
	}

	/* override plaform name, if required */
	ret = snd_soc_fixup_dai_links_platform_name(&sample_card,
						    mach->mach_params.platform);
	if (ret)
		return ret;
#endif
	ret = devm_snd_soc_register_card(&pdev->dev, &sample_card);
	pr_info("register returned ret = %d\n", ret);
	return ret;
}

static struct platform_driver snd_sample_driver = {
	.driver = {
		.name = "imx-sample",
	},
	.probe = snd_sample_probe,
};
module_platform_driver(snd_sample_driver);

MODULE_DESCRIPTION("ASoC Intel(R) Baytrail/Cherrytrail Machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:bytcht_cx2072x");
