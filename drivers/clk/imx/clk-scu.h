/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2018 NXP
 *   Dong Aisheng <aisheng.dong@nxp.com>
 */

#ifndef __IMX_CLK_SCU_H
#define __IMX_CLK_SCU_H

#include <linux/firmware/imx/sci.h>

extern struct imx_sc_ipc *ccm_ipc_handle;

static inline int imx_clk_scu_init(void)
{
	return imx_scu_get_handle(&ccm_ipc_handle);
}

struct clk_hw *imx_clk_scu(const char *name, u32 rsrc_id, u8 clk_type);

#endif
