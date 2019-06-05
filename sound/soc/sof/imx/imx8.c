// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
//
// Copyright 2019 NXP
//
// Author: Daniel Baluta <daniel.baluta@nxp.com>
//
// Hardware interface for audio DSP on i.MX8

#include <linux/firmware.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/pm_domain.h>  

#include <linux/module.h>
#include <sound/sof.h>
#include <sound/sof/xtensa.h>
#include <linux/firmware/imx/ipc.h>
#include <linux/firmware/imx/svc/misc.h>
#include <linux/mx8_mu.h>
#include <dt-bindings/firmware/imx/rsrc.h>
#include "../ops.h"

#define IMX_MU_xCR_GIEn(x)	BIT(28 + (3 - (x)))
#define IMX_MU_xSR_GIPn(x)	BIT(28 + (3 - (x)))

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

#define MBOX_OFFSET	0
#define MBOX_SIZE	0x1000

/* DSP peripherals */

struct fsl_dsp {
	struct device *dev;
	struct snd_sof_dev *sdev;
	void __iomem *mu_base_virtaddr;
	int dsp_mu_id;
};

static void imx8_get_windows(struct snd_sof_dev *sdev)
{

	struct sof_ipc_window_elem *elem;
	u32 outbox_offset = 0;
	u32 stream_offset = 0;
	u32 inbox_offset = 0;
	u32 outbox_size = 0;
	u32 stream_size = 0;
	u32 inbox_size = 0;
	int i;

	if (!sdev->info_window) {
		dev_err(sdev->dev, "error: have no window info\n");
		return;
	}

	for (i = 0; i < sdev->info_window->num_windows; i++) {
		elem = &sdev->info_window->window[i];

		switch (elem->type) {
		case SOF_IPC_REGION_UPBOX:
			inbox_offset = elem->offset + MBOX_OFFSET;
			inbox_size = elem->size;
			snd_sof_debugfs_io_item(sdev,
						sdev->bar[SOF_FW_BLK_TYPE_SRAM] +
						inbox_offset,
						elem->size, "inbox",
						SOF_DEBUGFS_ACCESS_D0_ONLY);
			break;
		case SOF_IPC_REGION_DOWNBOX:
			outbox_offset = elem->offset + MBOX_OFFSET;
			outbox_size = elem->size;
			snd_sof_debugfs_io_item(sdev,
						sdev->bar[SOF_FW_BLK_TYPE_SRAM] +
						outbox_offset,
						elem->size, "outbox",
						SOF_DEBUGFS_ACCESS_D0_ONLY);
			break;
		case SOF_IPC_REGION_TRACE:
			snd_sof_debugfs_io_item(sdev,
						sdev->bar[SOF_FW_BLK_TYPE_SRAM] +
						elem->offset + MBOX_OFFSET,
						elem->size, "etrace",
						SOF_DEBUGFS_ACCESS_D0_ONLY);
			break;
		case SOF_IPC_REGION_DEBUG:
			snd_sof_debugfs_io_item(sdev,
						sdev->bar[SOF_FW_BLK_TYPE_SRAM] +
						elem->offset + MBOX_OFFSET,
						elem->size, "debug",
						SOF_DEBUGFS_ACCESS_D0_ONLY);
			break;
		case SOF_IPC_REGION_STREAM:
			stream_offset = elem->offset + MBOX_OFFSET;
			stream_size = elem->size;
			snd_sof_debugfs_io_item(sdev,
						sdev->bar[SOF_FW_BLK_TYPE_SRAM] +
						stream_offset,
						elem->size, "stream",
						SOF_DEBUGFS_ACCESS_D0_ONLY);
			break;
		case SOF_IPC_REGION_REGS:
			snd_sof_debugfs_io_item(sdev,
						sdev->bar[SOF_FW_BLK_TYPE_SRAM] +
						elem->offset + MBOX_OFFSET,
						elem->size, "regs",
						SOF_DEBUGFS_ACCESS_D0_ONLY);
			break;
		case SOF_IPC_REGION_EXCEPTION:
			sdev->dsp_oops_offset = elem->offset + MBOX_OFFSET;
			snd_sof_debugfs_io_item(sdev,
						sdev->bar[SOF_FW_BLK_TYPE_SRAM] +
						elem->offset + MBOX_OFFSET,
						elem->size, "exception",
						SOF_DEBUGFS_ACCESS_D0_ONLY);
			break;
		default:
			dev_err(sdev->dev, "error: get illegal window info\n");
			return;
		}
	}

	if (outbox_size == 0 || inbox_size == 0) {
		dev_err(sdev->dev, "error: get illegal mailbox window\n");
		return;
	}

	snd_sof_dsp_mailbox_init(sdev, inbox_offset, inbox_size,
				 outbox_offset, outbox_size);
	sdev->stream_box.offset = stream_offset;
	sdev->stream_box.size = stream_size;

	dev_dbg(sdev->dev, " mailbox upstream 0x%x - size 0x%x\n",
		inbox_offset, inbox_size);
	dev_dbg(sdev->dev, " mailbox downstream 0x%x - size 0x%x\n",
		outbox_offset, outbox_size);
	dev_dbg(sdev->dev, " stream region 0x%x - size 0x%x\n",
		stream_offset, stream_size);
}

/*
 * IPC Firmware ready.
 */
static int imx8_fw_ready(struct snd_sof_dev *sdev, u32 msg_id)
{
	struct sof_ipc_fw_ready *fw_ready = &sdev->fw_ready;
	u32 offset;
	int ret;

	/* mailbox must be on 4k boundary */
	offset = MBOX_OFFSET;

	dev_dbg(sdev->dev, "ipc: DSP is ready 0x%8.8x offset 0x%x\n",
		msg_id, offset);

	 /* no need to re-check version/ABI for subsequent boots */
	if (!sdev->first_boot)
		return 0;

	/* copy data from the DSP FW ready offset */
	sof_block_read(sdev, sdev->mailbox_bar, offset, fw_ready,
		       sizeof(*fw_ready));
	snd_sof_dsp_mailbox_init(sdev, fw_ready->dspbox_offset,
				 fw_ready->dspbox_size,
				 fw_ready->hostbox_offset,
				 fw_ready->hostbox_size);

	/* make sure ABI version is compatible */
	ret = snd_sof_ipc_valid(sdev);
	if (ret < 0)
		return ret;

	/* now check for extended data */
	snd_sof_fw_parse_ext_data(sdev, SOF_FW_BLK_TYPE_ROM, MBOX_OFFSET +
				  sizeof(struct sof_ipc_fw_ready));

	imx8_get_windows(sdev);

	return 0;
}

static void imx8_get_reply(struct snd_sof_dev *sdev)
{
	struct snd_sof_ipc_msg *msg = sdev->msg;
	struct sof_ipc_reply reply;
	unsigned long flags;
	int ret = 0;

	if (!msg) {
		dev_warn(sdev->dev, "unexpected ipc interrupt\n");
		return ;
	}

	/* get reply */
	sof_mailbox_read(sdev, sdev->host_box.offset, &reply, sizeof(reply));

	spin_lock_irqsave(&sdev->ipc_lock, flags);

	if (reply.error < 0) {
		memcpy(msg->reply_data, &reply, sizeof(reply));
		ret = reply.error;
	} else {
		/* reply has correct size? */
		if (reply.hdr.size != msg->reply_size) {
			dev_err(sdev->dev, "error: reply expected %zu got %u bytes\n",
				msg->reply_size, reply.hdr.size);
			ret = -EINVAL;
		 }

		/* read the message */
		if (msg->reply_size > 0)
			sof_mailbox_read(sdev, sdev->host_box.offset,
					 msg->reply_data, msg->reply_size);
	}

	msg->reply_error = ret;

	spin_unlock_irqrestore(&sdev->ipc_lock, flags);
}

static int imx8_host_done(struct snd_sof_dev *sdev) {
	struct fsl_dsp *dsp_priv = (struct fsl_dsp *)sdev->private;

	MU_EnableGeneralInt(dsp_priv->mu_base_virtaddr, 0);
}

static int imx8_dsp_done(struct snd_sof_dev *sdev)
{
	struct fsl_dsp *dsp_priv = (struct fsl_dsp *)sdev->private;

	MU_EnableGeneralInt(dsp_priv->mu_base_virtaddr, 1);
	MU_TriggerGIR(dsp_priv->mu_base_virtaddr, 1);
}

/*
 * IPC Doorbell IRQ handler and thread.
 */
static irqreturn_t imx8_irq_handler(int irq, void *context)
{
	struct fsl_dsp *dsp_priv = (struct fsl_dsp *)context;
	uint32_t status, control;
	struct snd_sof_dev *sdev = dsp_priv->sdev;

	status = MU_ReadStatus(dsp_priv->mu_base_virtaddr);
	control = MU_ReadControl(dsp_priv->mu_base_virtaddr);

	if (status & IMX_MU_xSR_GIPn(0)) {
		MU_DisableGeneralInt(dsp_priv->mu_base_virtaddr, 0);
		/* clear interrupt */
		MU_ClearGIP(dsp_priv->mu_base_virtaddr, 0);
		
		imx8_get_reply(sdev);
		snd_sof_ipc_reply(sdev, 0);
		imx8_dsp_done(sdev);
	}
	
	if (status & IMX_MU_xSR_GIPn(1)) {
		MU_DisableGeneralInt(dsp_priv->mu_base_virtaddr, 1);
		MU_ClearGIP(dsp_priv->mu_base_virtaddr, 1);
		snd_sof_ipc_msgs_rx(sdev);
		imx8_host_done(sdev);
	}

	return IRQ_HANDLED;
}

#if 0
static irqreturn_t imx8_irq_thread(int irq, void *context)
{
	return IRQ_HANDLED;
}
#endif

static int dsp_mu_init(struct fsl_dsp *dsp_priv)
{
	struct device *dev = dsp_priv->dev;
	struct device_node *np;
	unsigned int	dsp_mu_id;
	u32 irq;
	int ret = 0;

	/*
	 * Get the address of MU to be used for communication with the dsp
	 */
	np = of_find_compatible_node(NULL, NULL, "fsl,imx8-mu-dsp");
	if (!np) {
		dev_err(dev, "Cannot find MU entry in device tree\n");
		return -EINVAL;
	}
	dsp_priv->mu_base_virtaddr = of_iomap(np, 0);
	WARN_ON(!dsp_priv->mu_base_virtaddr);
	
	ret = of_property_read_u32_index(np,
				"fsl,dsp_ap_mu_id", 0, &dsp_mu_id);
	if (ret) {
		dev_err(dev, "Cannot get mu_id %d\n", ret);
		return -EINVAL;
	}

	dsp_priv->dsp_mu_id = dsp_mu_id;

	irq = of_irq_get(np, 0);

	ret = devm_request_irq(dsp_priv->dev, irq, imx8_irq_handler,
			IRQF_EARLY_RESUME, "dsp_mu_isr", dsp_priv);
	if (ret) {
		dev_err(dev, "request_irq failed %d, err = %d\n", irq, ret);
		return -EINVAL;
	}
	pr_info("DSP MU init id %d, irq %d\n", dsp_mu_id, irq);

	return ret;
}

static int imx8_send_msg(struct snd_sof_dev *sdev, struct snd_sof_ipc_msg *msg)
{
	struct fsl_dsp *dsp_priv = (struct fsl_dsp *)sdev->private;
	u64 cmd = msg->header;

	pr_info("imx8_send_msg ... %x\n", cmd);

	sof_mailbox_write(sdev, sdev->host_box.offset, msg->msg_data,
			  msg->msg_size);
	MU_TriggerGIR(dsp_priv->mu_base_virtaddr, 0);

	return 0;
}

/*
 * DSP control.
 */
static int imx8_run(struct snd_sof_dev *sdev)
{
	int ret;
	struct imx_sc_ipc *dsp_ipc_handle;
	struct fsl_dsp *dsp_priv = (struct fsl_dsp *)sdev->private;

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

	dsp_mu_init(dsp_priv);

	MU_Init(dsp_priv->mu_base_virtaddr);
	
	MU_EnableGeneralInt(dsp_priv->mu_base_virtaddr, 0);
	MU_EnableGeneralInt(dsp_priv->mu_base_virtaddr, 1);

	imx_sc_pm_cpu_start(dsp_ipc_handle, IMX_SC_R_DSP, true, 0x596f8000);

	return 0;
}

static void imx_dsp_load_fw_cb(const struct firmware *fw, void *context)
{
	struct snd_sof_dev *sdev = context;

	pr_info("sof: imx8 fw data %p\n", fw);
}

int imx_dsp_load_fw(struct snd_sof_dev *sdev, bool first_boot)
{
	struct snd_sof_pdata *plat_data = dev_get_platdata(sdev->dev);
	const char *fw_filename;
	int ret;

	/* set code loading condition to true */
	sdev->code_loading = 1;
	fw_filename = plat_data->machine->sof_fw_filename;

	ret = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				       fw_filename, sdev->dev, GFP_KERNEL,
				       sdev, imx_dsp_load_fw_cb);

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
static int imx8_probe(struct snd_sof_dev *sdev)
{
	struct snd_sof_pdata *pdata = sdev->pdata;
	struct fsl_dsp *dsp_priv;
	int i;
	const struct sof_dev_desc *desc = pdata->desc;
	struct platform_device *pdev =
		container_of(sdev->dev, struct platform_device, dev);
	struct resource *mmio;
	int num_domains = 0;
	u32 base, size;
	int ret = 0;
	struct device_node *np = pdev->dev.of_node;

	pr_info("imx8_dt_probe %x\n", sdev);

	dsp_priv = devm_kzalloc(&pdev->dev, sizeof(*dsp_priv), GFP_KERNEL);
	if (!dsp_priv)
		return -ENOMEM;

	sdev->private = dsp_priv;
	dsp_priv->dev = sdev->dev;
	dsp_priv->sdev = sdev;

	num_domains = of_count_phandle_with_args(np, "power-domains",
						 "#power-domain-cells");


	for (i = 0; i < num_domains; i++) {
		struct device *pd_dev;
		struct device_link *link;

		pd_dev = dev_pm_domain_attach_by_id(&pdev->dev, i);
		if (IS_ERR(pd_dev))
			return PTR_ERR(pd_dev);

		link = device_link_add(&pdev->dev, pd_dev,
			DL_FLAG_STATELESS |
			DL_FLAG_PM_RUNTIME |
			DL_FLAG_RPM_ACTIVE);
		if (IS_ERR(link))
			return PTR_ERR(link);
	}

	/* DSP base */
	mmio = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (mmio) {
		base = mmio->start;
		size = resource_size(mmio);
	} else {
		dev_err(sdev->dev, "error: failed to get DSP base at idx 0\n");
		return -EINVAL;
	}

	dev_info(sdev->dev, "DSP PHY base at 0x%px size 0x%x", base, size);

	sdev->bar[SOF_FW_BLK_TYPE_IRAM] = devm_ioremap(sdev->dev, base, size);
	if (!sdev->bar[SOF_FW_BLK_TYPE_IRAM]) {
		dev_err(sdev->dev, "error: failed to ioremap DSP base 0x%x size 0x%x\n",
			base, size);
		return -ENODEV;
	}
	dev_info(sdev->dev, "DSP VADDR %px base %px size %d\n", sdev->bar[SOF_FW_BLK_TYPE_IRAM], 
		base, size);

	sdev->mmio_bar = SOF_FW_BLK_TYPE_IRAM;

	base = 0x92400000;
	size = 0x800000;

	sdev->bar[SOF_FW_BLK_TYPE_SRAM] = devm_ioremap_wc(sdev->dev, base, size);
	if (!sdev->bar[SOF_FW_BLK_TYPE_IRAM]) {
		dev_err(sdev->dev, "error: failed to ioremap DSP base 0x%x size 0x%x\n",
			base, size);
		return -ENODEV;
	}

	dev_dbg(sdev->dev, "SDRAM VADDR %px base %px size %d\n", sdev->bar[SOF_FW_BLK_TYPE_SRAM], 
		base, size);


	base = 0x92C00000;
	size = 0x800000;

	sdev->bar[SOF_FW_BLK_TYPE_ROM] = devm_ioremap_wc(sdev->dev, base, size);
	if (!sdev->bar[SOF_FW_BLK_TYPE_ROM]) {
		dev_err(sdev->dev, "error: failed to ioremap DSP base 0x%x size 0x%x\n",
			base, size);
		return -ENODEV;
	}

	dev_info(sdev->dev, "SDRAM VADDR %px base %px size %d\n", sdev->bar[SOF_FW_BLK_TYPE_ROM], 
		base, size);

	sdev->mailbox_bar = SOF_FW_BLK_TYPE_ROM;

	return ret;
}

void imx8_ipc_msg_data(struct snd_sof_dev *sdev,
		      struct snd_pcm_substream *substream,
		      void *p, size_t sz)
{
	sof_mailbox_read(sdev, sdev->dsp_box.offset, p, sz);
}

int imx8_ipc_pcm_params(struct snd_sof_dev *sdev,
			struct snd_pcm_substream *substream,
			const struct sof_ipc_pcm_params_reply *reply)
{
	return 0;
}

#define IMX8_FORMAT (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | \
		      SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver imx8_dai[] = {
{
	.name = "esai-port",
},
};

/* i.MX8 QXP ops */
struct snd_sof_dsp_ops sof_imx8_ops = {
	/* device init */
	.probe		= imx8_probe,

	/* DSP core boot */
	.run		= imx8_run,

	/* Block IO */
	.block_read	= sof_block_read,
	.block_write	= sof_block_write,

	/* ipc */
	.send_msg	= imx8_send_msg,
	.fw_ready	= imx8_fw_ready,

	.ipc_msg_data = imx8_ipc_msg_data,
	.ipc_pcm_params = imx8_ipc_pcm_params,

	/* module loading */
	.load_module	= snd_sof_parse_module_memcpy,

	/* firmware loading */
	.load_firmware	= snd_sof_load_firmware_memcpy,

	/* DAI drivers */
	.drv = imx8_dai,
	.num_drv = 1, /* we have only 1 ESAI interface on i.MX8 */
};
EXPORT_SYMBOL(sof_imx8_ops);

MODULE_LICENSE("Dual BSD/GPL");
