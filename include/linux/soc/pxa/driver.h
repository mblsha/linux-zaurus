/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __LINUX_SOC_PXA_DRIVER_H
#define __LINUX_SOC_PXA_DRIVER_H

#include <linux/errno.h>
#include <linux/irq.h>
#include <linux/types.h>

static inline bool pxa_gpio_irq_type_valid(unsigned int type)
{
	type &= IRQ_TYPE_SENSE_MASK;
	return type && !(type & ~(IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING));
}

static inline u32 pxa_register_update(u32 old, u32 mask, u32 value)
{
	return (old & ~mask) | (value & mask);
}

static inline bool pxa_spi_dma_length_valid(bool enabled, size_t len,
					    size_t max_len,
					    unsigned int burst,
					    unsigned int bits_per_word)
{
	unsigned int bytes_per_word = (bits_per_word + 7) / 8;
	size_t quantum = burst * bytes_per_word;

	return enabled && quantum && len <= max_len && len >= quantum &&
	       !(len % quantum);
}

static inline bool pxa_dma_address_supported(bool has_dalgn, dma_addr_t addr)
{
	return has_dalgn || !(addr & 7);
}

static inline int pxa_i2s_rate_divisor(unsigned int rate)
{
	switch (rate) {
	case 11025:
		return 0x34;
	case 16000:
		return 0x24;
	case 22050:
		return 0x1a;
	case 44100:
		return 0xd;
	case 48000:
		return 0xc;
	default:
		return -EINVAL;
	}
}

static inline int pxa_camera_clock_divisor(unsigned long lcdclk,
					   unsigned long requested,
					   unsigned long *actual)
{
	unsigned long target = requested;
	unsigned int divisor;

	if (!lcdclk)
		return -EINVAL;
	if (target > lcdclk / 4)
		target = lcdclk / 4;
	if (!target)
		return -EINVAL;

	divisor = (lcdclk + 2 * target - 1) / (2 * target) - 1;
	if (actual)
		*actual = lcdclk / (2 * (divisor + 1));
	return divisor;
}

static inline bool pxa_ethernet_port_valid(unsigned int port)
{
	return port < 3;
}

static inline bool pxa_udc_link_active(bool vbus, bool pullup, bool suspended)
{
	return vbus && pullup && !suspended;
}

static inline u32 pxa_rtc_enable_value(u32 value, u32 trigger_mask)
{
	return value & ~trigger_mask;
}

static inline bool pxa25x_frequency_supported(unsigned long rate)
{
	return rate == 99532800 || rate == 199065600 ||
	       rate == 298598400 || rate == 398131200;
}

#endif /* __LINUX_SOC_PXA_DRIVER_H */
