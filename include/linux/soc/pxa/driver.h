/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __LINUX_SOC_PXA_DRIVER_H
#define __LINUX_SOC_PXA_DRIVER_H

#include <linux/errno.h>
#include <linux/irq.h>
#include <linux/types.h>

struct notifier_block;

enum pxa_cpufreq_transition_event {
	PXA_CPUFREQ_PRECHANGE,
	PXA_CPUFREQ_POSTCHANGE,
	PXA_CPUFREQ_ABORT,
};

struct pxa_cpufreq_transition {
	unsigned long old_hz;
	unsigned long new_hz;
	unsigned long memory_hz;
};

int pxa_cpufreq_register_transition_notifier(struct notifier_block *nb);
int pxa_cpufreq_unregister_transition_notifier(struct notifier_block *nb);
int pxa_cpufreq_notify_transition(unsigned long event,
				  struct pxa_cpufreq_transition *transition);

static inline bool pxa_gpio_irq_type_valid(unsigned int type)
{
	type &= IRQ_TYPE_SENSE_MASK;
	return type && !(type & ~(IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING));
}

static inline unsigned long pxa_gpio_pending(unsigned long latched,
					      unsigned long enabled)
{
	return latched & enabled;
}

static inline u32 pxa_register_update(u32 old, u32 mask, u32 value)
{
	return (old & ~mask) | (value & mask);
}

static inline u32 pxa_w1c_ack_value(u32 observed, u32 current_value,
				     u32 control_mask, u32 status_mask)
{
	return (current_value & control_mask) | (observed & status_mask);
}

static inline u32 pxa_w1c_status(u32 observed, u32 status_mask)
{
	return observed & status_mask;
}

static inline bool pxa_ficp_sir_pinmux_valid(unsigned int rx_af,
					     unsigned int tx_af)
{
	return rx_af == 2 && tx_af == 1;
}

static inline bool pxa_uart_transition_safe(bool is_ffuart,
					    bool console_active,
					    bool transmitter_empty)
{
	return !is_ffuart || (!console_active && transmitter_empty);
}

static inline u32 pxa_i2s_disable_stream(u32 value, u32 disable_bit)
{
	return value | disable_bit;
}

static inline bool pxa_i2s_all_streams_disabled(u32 value, u32 disable_mask)
{
	return (value & disable_mask) == disable_mask;
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

static inline bool pxa_dma_hotchain_allowed(bool running, bool issued_empty,
					    bool alignment_changes)
{
	return running && !issued_empty && !alignment_changes;
}

static inline bool pxa_dma_hotchain_link_allowed(bool running,
						 bool has_predecessor,
						 bool alignment_changes,
						 bool self_link)
{
	return pxa_dma_hotchain_allowed(running, !has_predecessor,
					 alignment_changes) && !self_link;
}

static inline bool pxa_lifecycle_can_release(int stop_result, bool running,
					     bool callback_pending)
{
	return !stop_result && !running && !callback_pending;
}

static inline bool pxa_i2c_last_read_should_stop(bool explicit_stop,
						 bool last_message)
{
	return explicit_stop || last_message;
}

static inline bool pxa_i2c_message_valid(bool read, size_t len)
{
	return !read || len;
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

static inline u32 pxa_udc_irq_ack_value(u32 pending)
{
	return pending;
}

static inline bool pxa_udc_timer_sync_allowed(bool hard_irq)
{
	return !hard_irq;
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

static inline bool pxa_memory_frequency_supported(unsigned long requested,
					   unsigned long current_hz)
{
	return requested == current_hz;
}

static inline const char *pxa_memory_clock_name(bool pxa25x)
{
	return pxa25x ? "system_bus" : "memory";
}

static inline unsigned long pxa_pcmcia_timing_clock(unsigned long memory_hz)
{
	return memory_hz / 10000;
}

static inline bool pxa_reset_complete(u32 value, u32 reset_mask)
{
	return !(value & reset_mask);
}

static inline bool pxa_gcu_buffer_available(const void *buffer)
{
	return buffer != NULL;
}

#endif /* __LINUX_SOC_PXA_DRIVER_H */
