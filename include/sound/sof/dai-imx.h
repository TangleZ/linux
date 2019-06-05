/* SPDX-License-Identifier: ((GPL-2.0 WITH Linux-syscall-note) OR BSD-3-Clause) */
/*
 * Copyright 2019 NXP
 */

#ifndef __INCLUDE_SOUND_SOF_DAI_IMX_H__
#define __INCLUDE_SOUND_SOF_DAI_IMX_H__

#include <sound/sof/header.h>

/* SAI Configuration Request - SOF_IPC_DAI_SAI_CONFIG */
struct sof_ipc_dai_sai_params {
	struct sof_ipc_hdr hdr;
	uint16_t reserved1;
	uint16_t mclk_id;

	uint32_t mclk_rate;	/* mclk frequency in Hz */
	uint32_t fsync_rate;	/* fsync frequency in Hz */
	uint32_t bclk_rate;	/* bclk frequency in Hz */

	/* TDM */
	uint32_t tdm_slots;
	uint32_t rx_slots;
	uint32_t tx_slots;

	/* data */
	uint32_t sample_valid_bits;
	uint16_t tdm_slot_width;
	uint16_t reserved2;	/* alignment */

	/* MCLK */
	uint32_t mclk_direction;

	uint16_t frame_pulse_width;
	uint16_t tdm_per_slot_padding_flag;
	uint32_t clks_control;
	uint32_t quirks;
} __packed;

#endif
