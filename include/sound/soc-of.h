/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright 2019 NXP
 */

#ifndef __LINUX_SND_SOC_OF_H
#define __LINUX_SND_SOC_OF_H

#include <sound/soc.h>

struct snd_soc_of_mach_params {
	const char *platform;
};

/**
 * snd_soc_of_mach: OF-based machine driver descriptor
 *
 * @common: common fields shared between DT/ACPI (see struct snd_soc_fw_mach)
 */
struct snd_soc_of_mach {
	struct snd_soc_fw_mach common;
	struct snd_soc_of_mach_params mach_params;
};

#endif
