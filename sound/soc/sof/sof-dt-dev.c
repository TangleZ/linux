// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
//
// This file is provided under a dual BSD/GPLv2 license.  When using or
// redistributing this file, you may do so under either license.
//
// Copyright(c) 2018 Intel Corporation. All rights reserved.
//
// Author: Liam Girdwood <liam.r.girdwood@linux.intel.com>
//

#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <sound/sof.h>

#include "ops.h"

extern struct snd_sof_dsp_ops sof_imx8_ops;

/* platform specific devices */
#if IS_ENABLED(CONFIG_SND_SOC_SOF_IMX8)
static struct sof_dev_desc sof_dt_imx8qxp_desc = {
	.default_fw_path = "imx/sof",
	.default_tplg_path = "imx/sof-tplg",
	.nocodec_fw_filename = "sof-imx8.ri",
	.nocodec_tplg_filename = "sof-imx8-nocodec.tplg",
	.ops = &sof_imx8_ops,
};
#endif

static const struct dev_pm_ops sof_dt_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(snd_sof_suspend, snd_sof_resume)
	SET_RUNTIME_PM_OPS(snd_sof_runtime_suspend, snd_sof_runtime_resume,
			   NULL)
};

static void sof_acpi_probe_complete(struct device *dev)
{
		/* allow runtime_pm */
		pm_runtime_set_autosuspend_delay(dev, SND_SOF_SUSPEND_DELAY_MS);
		pm_runtime_use_autosuspend(dev);
		pm_runtime_enable(dev);
}

int dummy_lpuart(void);

static int sof_dt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct sof_dev_desc *desc;
	struct snd_soc_acpi_mach *mach;
	struct snd_sof_pdata *sof_pdata;
	struct snd_sof_dsp_ops *ops;
	int ret;

	dev_info(&pdev->dev, "DT DSP detected");

	ret = dummy_lpuart();
	if (ret < 0) {
		pr_info("LPUART not ready\n");
		return ret;
	}

	sof_pdata = devm_kzalloc(dev, sizeof(*sof_pdata), GFP_KERNEL);
	if (!sof_pdata)
		return -ENOMEM;

	if (of_device_is_compatible(pdev->dev.of_node, "fsl,imx8qxp-dsp-fake")) {
		pr_info("Fake detected\n");
		return 0;
	}

	desc = device_get_match_data(dev);
	if (!desc)
		return -ENODEV;

	/* get ops for platform */
	ops = desc->ops;
	if (!ops) {
		dev_err(dev, "error: no matching DT descriptor ops\n");
		return -ENODEV;
	}

	/* force nocodec mode */
	dev_warn(dev, "Force to use nocodec mode\n");
	mach = devm_kzalloc(dev, sizeof(*mach), GFP_KERNEL);
	if (!mach)
		return -ENOMEM;
	ret = sof_nocodec_setup(dev, sof_pdata, mach, desc, ops);
	if (ret < 0) {
		pr_info("No codec error\n");
		return ret;
	}

	if (mach) {
		mach->mach_params.platform = dev_name(dev);
	}

	sof_pdata->machine = mach;
	sof_pdata->desc = desc;
	sof_pdata->dev = &pdev->dev;
	sof_pdata->platform = dev_name(dev);

	sof_pdata->fw_filename_prefix = sof_pdata->desc->default_fw_path;
	sof_pdata->tplg_filename_prefix = sof_pdata->desc->default_tplg_path;

	ret = snd_sof_device_probe(dev, sof_pdata); 
	if (!ret) {
		pr_info("errr device probe ret = %d\n", ret);
		//return ret;
	}
	
	sof_acpi_probe_complete(dev);

	return ret;
}

static int sof_dt_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id sof_dt_ids[] = {
#if IS_ENABLED(CONFIG_SND_SOC_SOF_IMX8)
	{ .compatible = "fsl,imx8qxp-sof-dsp", .data = &sof_dt_imx8qxp_desc},
#endif
	{ }
};
MODULE_DEVICE_TABLE(of, sof_dt_ids);

/* DT driver definition */
static struct platform_driver snd_sof_dt_driver = {
	.probe = sof_dt_probe,
	.remove = sof_dt_remove,
	.driver = {
		.name = "sof-audio-dt",
		.pm = &sof_dt_pm,
		.of_match_table = sof_dt_ids
	},
};
module_platform_driver(snd_sof_dt_driver);

MODULE_LICENSE("Dual BSD/GPL");
