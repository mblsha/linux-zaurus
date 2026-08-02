// SPDX-License-Identifier: GPL-2.0-only
/* Native Device Tree machine support for the Sharp Zaurus SL-C860. */

#include <linux/init.h>
#include <linux/memblock.h>
#include <linux/regulator/machine.h>

#include <asm/irq.h>
#include <asm/mach/arch.h>
#include <asm/mach/sharpsl_param.h>

#include "generic.h"
#include "pxa25x.h"
#include "pxa2xx-regs.h"

#ifdef CONFIG_SHARP_SL_C860_DEEP_RESUME
static void __init sl_c860_reserve_resume_page(void)
{
	if (memblock_reserve(0xa0000000, SZ_4K))
		panic("SL-C860 could not reserve the deep-resume trampoline page");
}
#endif

static void __init sl_c860_dt_init(void)
{
	/* Stop the 3.6864 MHz oscillator and drive PCMCIA/CS pins high. */
	PCFR |= PCFR_OPDE;
	regulator_has_full_constraints();
}

static const char * const sharp_sl_c860_dt_compat[] __initconst = {
	"sharp,sl-c860",
	NULL,
};

DT_MACHINE_START(SHARP_SL_C860_DT, "Sharp SL-C860 (Device Tree)")
	.map_io		= pxa25x_map_io,
	.nr_irqs	= PXA_NR_IRQS,
#ifdef CONFIG_SHARP_SL_C860_DEEP_RESUME
	.reserve	= sl_c860_reserve_resume_page,
#endif
	.init_early	= sharpsl_save_param,
	.init_machine	= sl_c860_dt_init,
	.restart	= pxa_restart,
	.dt_compat	= sharp_sl_c860_dt_compat,
MACHINE_END
