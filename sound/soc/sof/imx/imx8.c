// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
//
// This file is provided under a dual BSD/GPLv2 license.  When using or
// redistributing this file, you may do so under either license.
//
// Copyright(c) 2018 Intel Corporation. All rights reserved.
//
// Author: Liam Girdwood <liam.r.girdwood@linux.intel.com>
//

/*
 * Hardware interface for audio DSP on i.MX8
 */

#include <linux/firmware.h>
#include <linux/module.h>
#include <sound/sof.h>
#include <sound/sof/xtensa.h>
#include <linux/firmware/imx/ipc.h>
#include <linux/firmware/imx/svc/misc.h>
#include <dt-bindings/firmware/imx/rsrc.h>
#include "../ops.h"

/* DSP memories */
#define IRAM_OFFSET		0x10000
#define IRAM_SIZE		(2 * 1024)
#define DRAM0_OFFSET		0x0
#define DRAM0_SIZE		(32 * 1024)
#define DRAM1_OFFSET		0x8000
#define DRAM1_SIZE		(32 * 1024)
#define SYSRAM_OFFSET		0x18000
#define SYSRAM_SIZE		(256 * 1024)
#define SYSROM_OFFSET		0x58000
#define SYSROM_SIZE		(192 * 1024)

/* DSP peripherals */

/* BARs */
#define IMX8_DSP_BAR		0
#define IMX8_MU_BAR		1
#define IMX8_SRAM_BAR		3

static int byt_cmd_done(struct snd_sof_dev *sdev, int dir);

/*
 * IPC Firmware ready.
 */
static void imx8_get_windows(struct snd_sof_dev *sdev)
{
}

static int imx8_fw_ready(struct snd_sof_dev *sdev, u32 msg_id)
{
	return 0;
}

/*
 * Debug
 */

static void imx8_get_registers(struct snd_sof_dev *sdev,
			      struct sof_ipc_dsp_oops_xtensa *xoops,
			      struct sof_ipc_panic_info *panic_info,
			      u32 *stack, size_t stack_words)
{
}

static void imx8_dump(struct snd_sof_dev *sdev, u32 flags)
{
}

/*
 * IPC Doorbell IRQ handler and thread.
 */

static irqreturn_t imx8_irq_handler(int irq, void *context)
{
	struct snd_sof_dev *sdev = (struct snd_sof_dev *)context;
	int ret = IRQ_NONE;
	return ret;
}

static irqreturn_t imx8_irq_thread(int irq, void *context)
{
	return IRQ_HANDLED;
}

static int imx8_is_ready(struct snd_sof_dev *sdev)
{
	return 1;
}

static int imx8_send_msg(struct snd_sof_dev *sdev, struct snd_sof_ipc_msg *msg)
{
	return 0;
}

static int imx8_get_reply(struct snd_sof_dev *sdev, struct snd_sof_ipc_msg *msg)
{
	return 0;
}

static int imx8_cmd_done(struct snd_sof_dev *sdev, int dir)
{
	return 0;
}

/*
 * DSP control.
 */
static int imx8_run(struct snd_sof_dev *sdev)
{
	int ret;
	struct imx_sc_ipc *dsp_ipc_handle;

	pr_info("imx8_run\n");
		
	ret = imx_scu_get_handle(&dsp_ipc_handle);
	if (ret) {
		dev_err(sdev->dev, "Cannot obtain SCU handle (err = %d)\n", ret);
		return ret;
	}

	ret = imx_sc_misc_set_control(dsp_ipc_handle, IMX_SC_R_DSP, IMX_SC_C_OFS_SEL,
				      1);
	if (ret < 0) {
		dev_err(sdev->dev, "Error system address offset source select\n");
		return ret;
	}

	ret = imx_sc_misc_set_control(dsp_ipc_handle, IMX_SC_R_DSP,
				      IMX_SC_C_OFS_AUDIO, 0x80);
	if (ret < 0) {
		dev_err(sdev->dev, "Error system address offset of AUDIO\n");
		return ret;
	}

	ret = imx_sc_misc_set_control(dsp_ipc_handle, IMX_SC_R_DSP,
				      IMX_SC_C_OFS_PERIPH, 0x5A);
	if (ret < 0) {
		dev_err(sdev->dev, "Error system address offset of PERIPH %d\n",
			ret);
		return ret;
	}

	ret = imx_sc_misc_set_control(dsp_ipc_handle, IMX_SC_R_DSP,
				      IMX_SC_C_OFS_IRQ, 0x51);
	if (ret < 0) {
		dev_err(sdev->dev, "Error system address offset of IRQ\n");
		return ret;
	}

	pr_info("Starting dsp ...\n");
	imx_sc_pm_cpu_start(dsp_ipc_handle, IMX_SC_R_DSP, true, 0x596f8000);

	return 0;
}

static void imx_dsp_load_fw_cb(const struct firmware *fw, void *context)
{
	struct snd_sof_dev *sdev = context;

	pr_info("sof: imx8 fw data %p\n", fw);

	return 0;
}

int imx_dsp_load_fw(struct snd_sof_dev *sdev, bool first_boot)
{
	struct snd_sof_pdata *plat_data = dev_get_platdata(sdev->dev);
	const char *fw_filename;
	int ret;

	/* set code loading condition to true */
	sdev->code_loading = 1;
	fw_filename = plat_data->machine->sof_fw_filename;

	pr_info("imx_dsp_load_fw %s\n", fw_filename);

	ret = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				       fw_filename, sdev->dev, GFP_KERNEL,
				       sdev, imx_dsp_load_fw_cb);

	pr_info("imx_dsp_load_fw ret = %d\n", ret);
	return 0;
}

static irqreturn_t imx_irq_handler(int irq, void *context)
{
	return IRQ_WAKE_THREAD;
}

static irqreturn_t imx_irq_thread(int irq, void *context)
{
	return IRQ_HANDLED;
}

/*
 * Probe and remove.
 */
static int imx8_dt_probe(struct snd_sof_dev *sdev)
{
	struct snd_sof_pdata *pdata = sdev->pdata;
	const struct sof_dev_desc *desc = pdata->desc;
	struct platform_device *pdev =
		container_of(sdev->parent, struct platform_device, dev);
	struct resource *mmio;
	u32 base, size;
	int ret = 0;

	pr_info("imx8_dt_probe\n");

	/* DSP base */
	mmio = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (mmio) {
		base = mmio->start;
		size = resource_size(mmio);
	} else {
		dev_err(sdev->dev, "error: failed to get DSP base at idx 0\n");
		return -EINVAL;
	}

	dev_dbg(sdev->dev, "DSP PHY base at 0x%px size 0x%x", base, size);

	sdev->bar[IMX8_DSP_BAR] = devm_ioremap(sdev->dev, base, size);
	if (!sdev->bar[IMX8_DSP_BAR]) {
		dev_err(sdev->dev, "error: failed to ioremap DSP base 0x%x size 0x%x\n",
			base, size);
		return -ENODEV;
	}
	dev_dbg(sdev->dev, "DSP VADDR %px base %px size %d\n", sdev->bar[IMX8_DSP_BAR], 
		base, size);

	sdev->mmio_bar = IMX8_DSP_BAR;

	base = 0x92400000;
	size = 0x2000000;

	sdev->bar[IMX8_SRAM_BAR] = devm_ioremap_wc(sdev->dev, base, size);
	if (!sdev->bar[IMX8_SRAM_BAR]) {
		dev_err(sdev->dev, "error: failed to ioremap DSP base 0x%x size 0x%x\n",
			base, size);
		return -ENODEV;
	}
	dev_dbg(sdev->dev, "SDRAM VADDR %px base %px size %d\n", sdev->bar[IMX8_DSP_BAR], 
		base, size);

	sdev->ipc_irq = platform_get_irq(pdev, 0);
	if (sdev->ipc_irq < 0) {
		dev_err(sdev->dev, "error: failed to get IRQ at index 0");
		return sdev->ipc_irq;
	}

	dev_dbg(sdev->dev, "using IRQ %d\n", sdev->ipc_irq);
	ret = devm_request_threaded_irq(sdev->dev, sdev->ipc_irq, imx_irq_handler,
			       imx_irq_thread, 0, "AudioDSP", sdev);
	if (ret < 0) {
		dev_err(sdev->dev, "error: failed to register IRQ %d\n",
			sdev->ipc_irq);
		return ret;
	}

	return ret;
}

static int imx8_probe(struct snd_sof_dev *sdev)
{
	return imx8_dt_probe(sdev);
}

#if 0
static struct snd_soc_dai_driver imx_dai[] = {
{
};
#endif

/* i.MX8 QXP ops */
struct snd_sof_dsp_ops sof_imx8_ops = {
	/* device init */
	.probe		= imx8_probe,

	/* DSP core boot */
	.run		= imx8_run,

#if 0
	/* Register IO */
	.write		= sof_io_write,
	.read		= sof_io_read,
	.write64	= sof_io_write64,
	.read64		= sof_io_read64,
#endif
	/* Block IO */
	.block_read	= sof_block_read,
	.block_write	= sof_block_write,
#if 0
	/* doorbell */
	.irq_handler	= byt_irq_handler,
	.irq_thread	= byt_irq_thread,
#endif
	/* mailbox */
	.mailbox_read	= sof_mailbox_read,
	.mailbox_write	= sof_mailbox_write,

	/* ipc */
	.send_msg	= imx8_send_msg,
	.get_reply	= imx8_get_reply,
	.fw_ready	= imx8_fw_ready,
	.is_ready	= imx8_is_ready,
	.cmd_done	= imx8_cmd_done,
#if 0
	/* debug */
	.debug_map	= byt_debugfs,
	.debug_map_count	= ARRAY_SIZE(byt_debugfs),
	.dbg_dump	= byt_dump,
#endif
	/* module loading */
	.load_module	= snd_sof_parse_module_memcpy,

	/*Firmware loading */
	.load_firmware	= snd_sof_load_firmware_memcpy,
#if 0
	/* DAI drivers */
	.drv = imx_dai,
	.num_drv = 1, /* we have only 3 SSPs on byt*/
#endif
};
EXPORT_SYMBOL(sof_imx8_ops);

MODULE_LICENSE("Dual BSD/GPL");
