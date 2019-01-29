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

static struct sof_dev_desc sof_dt_imx8qxp_desc = {
	.nocodec_fw_filename = "imx/sof-imx8qxp.ri",
	.nocodec_tplg_filename = "imx/sof-imx8qxp-nocodec.tplg"
};
static const struct dev_pm_ops sof_dt_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(snd_sof_suspend, snd_sof_resume)
	SET_RUNTIME_PM_OPS(snd_sof_runtime_suspend, snd_sof_runtime_resume,
			   NULL)
};

static const struct sof_ops_table dt_mach_ops[] = {
	{&sof_dt_imx8qxp_desc, &sof_imx8_ops},

};
static int sof_dt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct sof_dev_desc *desc;
	struct snd_soc_acpi_mach *mach;
	struct snd_sof_pdata *sof_pdata;
	struct sof_platform_priv *priv;
	struct snd_sof_dsp_ops *ops;
	int ret = 0;

	dev_info(&pdev->dev, "DT DSP detected");

	if (of_device_is_compatible(pdev->dev.of_node, "fsl,imx8qxp-dsp-fake")) {
		pr_info("Fake detected\n");
		return 0;
	}
	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	sof_pdata = devm_kzalloc(dev, sizeof(*sof_pdata), GFP_KERNEL);
	if (!sof_pdata)
		return -ENOMEM;

	desc = (const struct sof_dev_desc *)device_get_match_data(dev);
	if (!desc)
		return -ENODEV;

	/* get ops for platform */
	ops = sof_get_ops(desc, dt_mach_ops, ARRAY_SIZE(dt_mach_ops));
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
	if (ret < 0)
		return ret;

	mach->pdata = ops;
	sof_pdata->machine = mach;
	sof_pdata->desc = desc;
	priv->sof_pdata = sof_pdata;
	sof_pdata->dev = &pdev->dev;
	sof_pdata->platform = "sof-audio";
	dev_set_drvdata(&pdev->dev, priv);

	/* register sof-audio platform driver */
	ret = sof_create_platform_device(priv);
	if (ret) {
		dev_err(dev, "error: failed to create platform device!\n");
		return ret;
	}

	/* allow runtime_pm */
	pm_runtime_set_autosuspend_delay(dev, SND_SOF_SUSPEND_DELAY_MS);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_allow(dev);

	return ret;
}

static int sof_dt_remove(struct platform_device *pdev)
{
	struct sof_platform_priv *priv = dev_get_drvdata(&pdev->dev);
	struct snd_sof_pdata *sof_pdata = priv->sof_pdata;
#if 0
	if (!IS_ERR(priv->pdev_pcm))
		platform_device_unregister(priv->pdev_pcm);
	release_firmware(sof_pdata->fw);
#endif
	return 0;
}

static const struct of_device_id sof_dt_ids[] = {
	{ .compatible = "fsl,imx8qxp-dsp", .data = &sof_dt_imx8qxp_desc},
	{ .compatible = "fsl,imx8qxp-dsp-fake", .data = &sof_dt_imx8qxp_desc},
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
