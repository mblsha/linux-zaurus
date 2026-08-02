// SPDX-License-Identifier: GPL-2.0-only
/*
 *  linux/arch/arm/mach-pxa/pxa25x.c
 *
 *  Author:	Nicolas Pitre
 *  Created:	Jun 15, 2001
 *  Copyright:	MontaVista Software Inc.
 *
 * Code specific to PXA21x/25x/26x variants.
 *
 * Since this file should be linked before any other machine specific file,
 * the __initcall() here will be executed first.  This serves as default
 * initialization stuff for PXA machines which can be overridden later if
 * need be.
 */
#include <linux/dmaengine.h>
#include <linux/dma/pxa-dma.h>
#include <linux/gpio.h>
#include <linux/gpio-pxa.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/suspend.h>
#include <linux/syscore_ops.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/platform_data/mmp_dma.h>
#include <linux/soc/pxa/cpu.h>

#include <asm/cacheflush.h>
#include <asm/mach/map.h>
#include <asm/mach-types.h>
#include <asm/proc-fns.h>
#include <asm/suspend.h>
#include "irqs.h"
#include "pxa25x.h"
#include "reset.h"
#include "pm.h"
#include "addr-map.h"
#include "smemc.h"
#include "sl-c860-resume.h"

#include "generic.h"
#include "devices.h"

/*
 * Various clock factors driven by the CCCR register.
 */

#ifdef CONFIG_PM

#ifdef CONFIG_SHARP_SL_C860_DEEP_RESUME
#define SL_C860_RESUME_PHYS	0xa0000000

static bool sl_c860_deep_resume_active(void)
{
	return machine_is_husky() ||
	       of_machine_is_compatible("sharp,sl-c860");
}

static int sl_c860_resume_trampoline_readback(void)
{
	const u8 *source = sl_c860_resume_trampoline;
	const u8 *target = phys_to_virt(SL_C860_RESUME_PHYS);
	size_t code_size = sl_c860_resume_trampoline_context_cell - source;
	size_t context_offset =
		sl_c860_resume_trampoline_context_cell - source;
	size_t resume_offset =
		sl_c860_resume_trampoline_cpu_do_resume - source;
	size_t size = sl_c860_resume_trampoline_end - source;

	if (!size || size > SZ_4K ||
	    resume_offset != context_offset + sizeof(u32) ||
	    size != resume_offset + sizeof(u32) ||
	    memcmp(target, source, code_size) ||
	    *(u32 *)(target + context_offset) !=
		    __pa_symbol(&sleep_save_sp) +
		    offsetof(struct sleep_save_sp, save_ptr_stash_phys) ||
	    *(u32 *)(target + resume_offset) !=
		    __pa_symbol(cpu_do_resume))
		return -EIO;

	return 0;
}

static int sl_c860_resume_trampoline_stage(void)
{
	const u8 *source = sl_c860_resume_trampoline;
	u8 *target = phys_to_virt(SL_C860_RESUME_PHYS);
	size_t context_offset =
		sl_c860_resume_trampoline_context_cell - source;
	size_t resume_offset =
		sl_c860_resume_trampoline_cpu_do_resume - source;
	size_t size = sl_c860_resume_trampoline_end - source;

	if (!size || size > SZ_4K ||
	    resume_offset != context_offset + sizeof(u32) ||
	    size != resume_offset + sizeof(u32))
		return -EINVAL;

	memcpy(target, source, size);
	*(u32 *)(target + context_offset) =
		__pa_symbol(&sleep_save_sp) +
		offsetof(struct sleep_save_sp, save_ptr_stash_phys);
	*(u32 *)(target + resume_offset) = __pa_symbol(cpu_do_resume);
	flush_icache_range((unsigned long)target,
			   (unsigned long)target + size);

	return sl_c860_resume_trampoline_readback();
}
#endif

#define SAVE(x)		sleep_save[SLEEP_SAVE_##x] = x
#define RESTORE(x)	x = sleep_save[SLEEP_SAVE_##x]

/*
 * List of global PXA peripheral registers to preserve.
 * More ones like CP and general purpose register values are preserved
 * with the stack pointer in sleep.S.
 */
enum {
	SLEEP_SAVE_PSTR,
	SLEEP_SAVE_COUNT
};


static void pxa25x_cpu_pm_save(unsigned long *sleep_save)
{
	SAVE(PSTR);
}

static void pxa25x_cpu_pm_restore(unsigned long *sleep_save)
{
	RESTORE(PSTR);
}

static void pxa25x_cpu_pm_enter(suspend_state_t state)
{
	/* Clear reset status */
	RCSR = RCSR_HWR | RCSR_WDR | RCSR_SMR | RCSR_GPR;

	switch (state) {
	case PM_SUSPEND_MEM:
#ifdef CONFIG_SHARP_SL_C860_DEEP_RESUME
		if (sl_c860_deep_resume_active()) {
			if (sl_c860_resume_trampoline_readback() ||
			    PSPR != SL_C860_RESUME_PHYS) {
				pr_emerg("SL-C860 deep resume: trampoline or PSPR readback failed\n");
				break;
			}
			cpu_suspend(PWRMODE_SLEEP,
				    sl_c860_pxa25x_finish_suspend);
			break;
		}
#endif
		cpu_suspend(PWRMODE_SLEEP, pxa25x_finish_suspend);
		break;
	}
}

static int pxa25x_cpu_pm_prepare(void)
{
	/* set resume return address */
#ifdef CONFIG_SHARP_SL_C860_DEEP_RESUME
	if (sl_c860_deep_resume_active()) {
		int error = sl_c860_resume_trampoline_stage();

		if (error) {
			pr_err("SL-C860 deep resume: cannot stage trampoline: %d\n",
			       error);
			return error;
		}
		PSPR = SL_C860_RESUME_PHYS;
		if (PSPR != SL_C860_RESUME_PHYS)
			return -EIO;
		return 0;
	}
#endif
	PSPR = __pa_symbol(cpu_resume);
	return 0;
}

static void pxa25x_cpu_pm_finish(void)
{
	/* ensure not to come back here if it wasn't intended */
	PSPR = 0;
}

static struct pxa_cpu_pm_fns pxa25x_cpu_pm_fns = {
	.save_count	= SLEEP_SAVE_COUNT,
	.valid		= suspend_valid_only_mem,
	.save		= pxa25x_cpu_pm_save,
	.restore	= pxa25x_cpu_pm_restore,
	.enter		= pxa25x_cpu_pm_enter,
	.prepare	= pxa25x_cpu_pm_prepare,
	.finish		= pxa25x_cpu_pm_finish,
};

static void __init pxa25x_init_pm(void)
{
	pxa_cpu_pm_fns = &pxa25x_cpu_pm_fns;
}
#else
static inline void pxa25x_init_pm(void) {}
#endif

/* PXA25x: supports wakeup from GPIO0..GPIO15 and RTC alarm
 */

static int pxa25x_set_wake(struct irq_data *d, unsigned int on)
{
	int gpio = pxa_irq_to_gpio(d->irq);
	uint32_t mask = 0;

	if (gpio >= 0 && gpio < 85)
		return gpio_set_wake(gpio, on);

	if (d->irq == IRQ_RTCAlrm) {
		mask = PWER_RTC;
		goto set_pwer;
	}

	return -EINVAL;

set_pwer:
	if (on)
		PWER |= mask;
	else
		PWER &=~mask;

	return 0;
}

void __init pxa25x_init_irq(void)
{
	pxa_init_irq(32, pxa25x_set_wake);
}

#ifdef CONFIG_CPU_PXA26x
void __init pxa26x_init_irq(void)
{
	pxa_init_irq(32, pxa25x_set_wake);
}
#endif

static int __init __init
pxa25x_dt_init_irq(struct device_node *node, struct device_node *parent)
{
	pxa_dt_irq_init(pxa25x_set_wake);
	set_handle_irq(icip_handle_irq);

	return 0;
}
IRQCHIP_DECLARE(pxa25x_intc, "marvell,pxa-intc", pxa25x_dt_init_irq);

static struct map_desc pxa25x_io_desc[] __initdata = {
	{	/* Mem Ctl */
		.virtual	= (unsigned long)SMEMC_VIRT,
		.pfn		= __phys_to_pfn(PXA2XX_SMEMC_BASE),
		.length		= SMEMC_SIZE,
		.type		= MT_DEVICE
	}, {	/* UNCACHED_PHYS_0 */
		.virtual	= UNCACHED_PHYS_0,
		.pfn		= __phys_to_pfn(0x00000000),
		.length		= UNCACHED_PHYS_0_SIZE,
		.type		= MT_DEVICE
	},
};

void __init pxa25x_map_io(void)
{
	pxa_map_io();
	iotable_init(ARRAY_AND_SIZE(pxa25x_io_desc));
	pxa25x_get_clk_frequency_khz(1);
}

static struct pxa_gpio_platform_data pxa25x_gpio_info __initdata = {
	.irq_base	= PXA_GPIO_TO_IRQ(0),
	.gpio_set_wake	= gpio_set_wake,
};

static struct platform_device *pxa25x_devices[] __initdata = {
	&pxa25x_device_udc,
	&pxa_device_pmu,
	&pxa_device_i2s,
	&sa1100_device_rtc,
	&pxa25x_device_ssp,
	&pxa25x_device_nssp,
	&pxa25x_device_assp,
	&pxa25x_device_pwm0,
	&pxa25x_device_pwm1,
	&pxa_device_asoc_platform,
};

#ifdef CONFIG_MACH_SHARP_SL_C860_DT
/*
 * These devices still consume Corgi platform data during the hybrid DT phase.
 * Clocks, RTC, PWM, and the interrupt controller remain DT-owned. GPIO and
 * pinctrl stay legacy-owned because the PXA25x DT nodes reserve overlapping
 * MMIO and dynamically allocate GPIO IRQs that board-file clients cannot use.
 */
static struct platform_device *pxa25x_sl_c860_legacy_devices[] __initdata = {
#if !IS_ENABLED(CONFIG_SHARP_SL_C860_DT_UDC)
	&pxa25x_device_udc,
#endif
	&pxa_device_pmu,
#if !IS_ENABLED(CONFIG_SHARP_SL_C860_DT_AUDIO)
	&pxa_device_i2s,
#endif
#if !IS_ENABLED(CONFIG_SHARP_SL_C860_DT_SPI)
	&pxa25x_device_ssp,
#endif
#if !IS_ENABLED(CONFIG_SHARP_SL_C860_DT_AUDIO)
	&pxa_device_asoc_platform,
#endif
};
#endif

static const struct dma_slave_map pxa25x_slave_map[] = {
	/* PXA25x, PXA27x and PXA3xx common entries */
	{ "pxa2xx-ac97", "pcm_pcm_mic_mono", PDMA_FILTER_PARAM(LOWEST, 8) },
	{ "pxa2xx-ac97", "pcm_pcm_aux_mono_in", PDMA_FILTER_PARAM(LOWEST, 9) },
	{ "pxa2xx-ac97", "pcm_pcm_aux_mono_out",
	  PDMA_FILTER_PARAM(LOWEST, 10) },
	{ "pxa2xx-ac97", "pcm_pcm_stereo_in", PDMA_FILTER_PARAM(LOWEST, 11) },
	{ "pxa2xx-ac97", "pcm_pcm_stereo_out", PDMA_FILTER_PARAM(LOWEST, 12) },
	{ "pxa-ssp-dai.1", "rx", PDMA_FILTER_PARAM(LOWEST, 13) },
	{ "pxa-ssp-dai.1", "tx", PDMA_FILTER_PARAM(LOWEST, 14) },
	{ "pxa-ssp-dai.2", "rx", PDMA_FILTER_PARAM(LOWEST, 15) },
	{ "pxa-ssp-dai.2", "tx", PDMA_FILTER_PARAM(LOWEST, 16) },
	{ "pxa2xx-ir", "rx", PDMA_FILTER_PARAM(LOWEST, 17) },
	{ "pxa2xx-ir", "tx", PDMA_FILTER_PARAM(LOWEST, 18) },
	{ "pxa2xx-mci.0", "rx", PDMA_FILTER_PARAM(LOWEST, 21) },
	{ "pxa2xx-mci.0", "tx", PDMA_FILTER_PARAM(LOWEST, 22) },
	/* DT-created PXA25x MMC device; native PDMA ownership moves later. */
	{ "41100000.mmc", "rx", PDMA_FILTER_PARAM(LOWEST, 21) },
	{ "41100000.mmc", "tx", PDMA_FILTER_PARAM(LOWEST, 22) },

	{ "pxa2xx-i2s", "rx", PDMA_FILTER_PARAM(LOWEST, 2) },
	{ "pxa2xx-i2s", "tx", PDMA_FILTER_PARAM(LOWEST, 3) },
	/* DT-created PXA25x I2S device; native PDMA ownership moves later. */
	{ "40400000.audio-controller", "rx", PDMA_FILTER_PARAM(LOWEST, 2) },
	{ "40400000.audio-controller", "tx", PDMA_FILTER_PARAM(LOWEST, 3) },
	/* PXA25x specific map */
	{ "pxa25x-ssp.0", "rx", PDMA_FILTER_PARAM(LOWEST, 13) },
	{ "pxa25x-ssp.0", "tx", PDMA_FILTER_PARAM(LOWEST, 14) },
	{ "pxa25x-nssp.1", "rx", PDMA_FILTER_PARAM(LOWEST, 15) },
	{ "pxa25x-nssp.1", "tx", PDMA_FILTER_PARAM(LOWEST, 16) },
	{ "pxa25x-nssp.2", "rx", PDMA_FILTER_PARAM(LOWEST, 23) },
	{ "pxa25x-nssp.2", "tx", PDMA_FILTER_PARAM(LOWEST, 24) },
};

static struct mmp_dma_platdata pxa25x_dma_pdata = {
	.dma_channels	= 16,
	.nb_requestors	= 40,
	.slave_map	= pxa25x_slave_map,
	.slave_map_cnt	= ARRAY_SIZE(pxa25x_slave_map),
};

static int __init pxa25x_init(void)
{
	int ret = 0;

	if (cpu_is_pxa25x()) {

		pxa_register_wdt(RCSR);

		pxa25x_init_pm();

		register_syscore_ops(&pxa_irq_syscore_ops);
		register_syscore_ops(&pxa2xx_mfp_syscore_ops);

#ifdef CONFIG_MACH_SHARP_SL_C860_DT
		if (of_machine_is_compatible("sharp,sl-c860")) {
			if (IS_ENABLED(CONFIG_SHARP_SL_C860_FULL_DT))
				return 0;

			/*
			 * GPIO and pinctrl remain legacy-owned until the board
			 * file no longer needs their fixed numbering and IRQ base.
			 * Keep PDMA legacy-owned only for rollback configurations;
			 * production uses the standard OF DMA controller.
			 */
			if (!IS_ENABLED(CONFIG_SHARP_SL_C860_DT_PDMA))
				pxa2xx_set_dmac_info(&pxa25x_dma_pdata);
			if (IS_ENABLED(CONFIG_SHARP_SL_C860_DT_CHARGE_LED)) {
				struct device_node *gpio_node;

				/*
				 * Pass the disabled provider node through platform
				 * data. The GPIO driver assigns it only to the
				 * gpiochip, leaving device resources, its clock,
				 * and the fixed legacy IRQ domain unchanged.
				 * Keep this node reference for the gpiochip's
				 * lifetime.
				 */
				gpio_node = of_find_compatible_node(NULL, NULL,
								    "intel,pxa25x-gpio");
				if (!gpio_node) {
					pr_err("SL-C860 DT: GPIO provider node missing\n");
					return -ENODEV;
				}
				pxa25x_gpio_info.of_node = gpio_node;
			}
			pxa_register_device(&pxa25x_device_gpio,
					    &pxa25x_gpio_info);
			ret = platform_add_devices(
				pxa25x_sl_c860_legacy_devices,
				ARRAY_SIZE(pxa25x_sl_c860_legacy_devices));
		} else
#endif
		if (!of_have_populated_dt()) {
			pxa2xx_set_dmac_info(&pxa25x_dma_pdata);
			pxa_register_device(&pxa25x_device_gpio, &pxa25x_gpio_info);
			ret = platform_add_devices(pxa25x_devices,
						   ARRAY_SIZE(pxa25x_devices));
		}
	}

	return ret;
}

postcore_initcall(pxa25x_init);
