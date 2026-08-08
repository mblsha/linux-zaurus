/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * PXA clocksource, clockevents, and OST interrupt handlers.
 *
 * Copyright (C) 2014 Robert Jarzmik
 */

#ifndef _CLOCKSOURCE_PXA_H
#define _CLOCKSOURCE_PXA_H

#include <linux/types.h>

#define PXA_TIMER_NO_USER_MMIO	(~(phys_addr_t)0)

void pxa_timer_nodt_init(int irq, void __iomem *base,
			 phys_addr_t phys_base);

#endif
