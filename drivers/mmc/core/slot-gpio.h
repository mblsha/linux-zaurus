/* SPDX-License-Identifier: GPL-2.0-only */
/*
  * Copyright (C) 2014 Linaro Ltd
 *
 * Author: Ulf Hansson <ulf.hansson@linaro.org>
 */
#ifndef _MMC_CORE_SLOTGPIO_H
#define _MMC_CORE_SLOTGPIO_H

#include <linux/math.h>
#include <linux/types.h>

struct mmc_host;

int mmc_gpio_alloc(struct mmc_host *host);

static inline u32 mmc_gpio_debounce_delay_ms(u32 current_delay_ms,
					     unsigned int debounce_us,
					     int debounce_ret)
{
	if (!debounce_us)
		return current_delay_ms;

	return debounce_ret < 0 ? DIV_ROUND_UP(debounce_us, 1000) : 0;
}

#endif
