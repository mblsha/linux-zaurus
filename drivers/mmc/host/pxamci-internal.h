/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _PXAMCI_INTERNAL_H
#define _PXAMCI_INTERNAL_H

#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/limits.h>
#include <linux/math64.h>
#include <linux/mmc/host.h>
#include <linux/time64.h>
#include <linux/types.h>

#include "pxamci.h"

#define PXAMCI_CLKRT_OFF		UINT_MAX
#define PXAMCI_MAX_CLKRT		6
#define PXAMCI_26MHZ_CLKRT		7
#define PXAMCI_WATCHDOG_INTERVAL_MS	100
#define PXAMCI_COMMAND_TIMEOUT_MS	1000
#define PXAMCI_TIMEOUT_GRACE_MS		1000

struct pxamci_clock_config {
	unsigned int clkrt;
	unsigned int actual_clock;
};

enum pxamci_lifecycle_action {
	PXAMCI_ACTION_IGNORE,
	PXAMCI_ACTION_START_DATA,
	PXAMCI_ACTION_START_COMMAND,
	PXAMCI_ACTION_START_STOP,
	PXAMCI_ACTION_FINISH_REQUEST,
	PXAMCI_ACTION_WAIT_FOR_EVENT,
	PXAMCI_ACTION_WAIT_FOR_DMA,
	PXAMCI_ACTION_RECORD_DMA_DONE,
	PXAMCI_ACTION_RECORD_PROGRAM_DONE,
	PXAMCI_ACTION_FINISH_DATA,
	PXAMCI_ACTION_RECOVER,
};

enum pxamci_dma_result {
	PXAMCI_DMA_RUNNING,
	PXAMCI_DMA_COMPLETE,
	PXAMCI_DMA_FAILED,
};

enum pxamci_recovery_reason {
	PXAMCI_RECOVERY_NONE,
	PXAMCI_RECOVERY_COMMAND,
	PXAMCI_RECOVERY_DMA,
	PXAMCI_RECOVERY_CONTROLLER,
	PXAMCI_RECOVERY_LOST_COMPLETION,
	PXAMCI_RECOVERY_PROGRAM,
	PXAMCI_RECOVERY_TIMEOUT,
};

struct pxamci_watchdog_state {
	bool request_current;
	bool data_active;
	bool finishing;
	bool abort_request;
	bool dma_failed;
	bool data_failed;
	bool dma_complete;
	bool controller_done;
	bool data_done_pending;
	bool deadline_expired;
	bool command_active;
	bool program_active;
	bool program_done;
};

struct pxamci_watchdog_decision {
	enum pxamci_lifecycle_action action;
	enum pxamci_recovery_reason reason;
	bool abort_request;
	bool set_command_timeout;
	bool set_data_timeout;
	bool set_program_timeout;
};

struct pxamci_quiesce_ops {
	void (*cancel_work)(void *data);
	int (*terminate_dma)(void *data, bool tx);
};

static inline struct pxamci_clock_config
pxamci_clock_config(unsigned long rate, unsigned int requested,
		    bool supports_26mhz)
{
	struct pxamci_clock_config config = {
		.clkrt = PXAMCI_CLKRT_OFF,
	};
	unsigned int divisor;

	if (!requested || !rate)
		return config;

	if (supports_26mhz && requested == 26000000) {
		config.clkrt = PXAMCI_26MHZ_CLKRT;
		config.actual_clock = 26000000;
		return config;
	}

	divisor = DIV_ROUND_UP(rate, requested);
	config.clkrt = min_t(unsigned int, fls(divisor - 1),
			     PXAMCI_MAX_CLKRT);
	config.actual_clock = rate >> config.clkrt;

	return config;
}

static inline unsigned int
pxamci_read_timeout_reg(unsigned int timeout_ns, unsigned int timeout_clks,
			unsigned long clkrate, unsigned int card_clock)
{
	u64 cycles;
	u64 value;

	if (!clkrate || !card_clock)
		return U16_MAX;

	cycles = DIV_ROUND_UP_ULL((u64)timeout_ns * clkrate,
				  NSEC_PER_SEC);
	cycles += DIV_ROUND_UP_ULL((u64)timeout_clks * clkrate, card_clock);
	value = DIV_ROUND_UP_ULL(cycles, 256);

	return clamp_t(u64, value, 1, U16_MAX);
}

static inline unsigned int
pxamci_data_timeout_ms(unsigned int timeout_ns, unsigned int timeout_clks,
		       unsigned int clock, unsigned int bytes,
		       unsigned int bus_width)
{
	u64 timeout_ms;
	u64 transfer_ms;

	timeout_ms = DIV_ROUND_UP_ULL(timeout_ns, NSEC_PER_MSEC);
	if (timeout_clks) {
		if (!clock)
			return UINT_MAX;
		timeout_ms += DIV_ROUND_UP_ULL((u64)timeout_clks * MSEC_PER_SEC,
					       clock);
	}

	if (clock && bytes) {
		bus_width = max(bus_width, 1U);
		transfer_ms = DIV_ROUND_UP_ULL((u64)bytes * 8 * MSEC_PER_SEC,
					       (u64)clock * bus_width);
		/* Leave room for controller and card turn-around delays. */
		timeout_ms += transfer_ms * 2;
	}

	timeout_ms += PXAMCI_TIMEOUT_GRACE_MS;
	return min_t(u64, timeout_ms, UINT_MAX);
}

static inline unsigned int
pxamci_command_timeout_ms(unsigned int busy_timeout_ms)
{
	u64 timeout_ms = busy_timeout_ms ?: PXAMCI_COMMAND_TIMEOUT_MS;

	timeout_ms += PXAMCI_TIMEOUT_GRACE_MS;
	return min_t(u64, timeout_ms, UINT_MAX);
}

static inline unsigned int
pxamci_merge_caps(unsigned int caps, bool pxa25x, bool supports_26mhz,
		  bool firmware_bus_width)
{
	if (!pxa25x) {
		if (!firmware_bus_width)
			caps |= MMC_CAP_4_BIT_DATA;
		caps |= MMC_CAP_SDIO_IRQ | MMC_CAP_CMD23;
	}
	if (supports_26mhz)
		caps |= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED;

	return caps;
}

static inline bool
pxamci_use_legacy_ro_active_high(bool has_platform_data, bool has_firmware,
				 bool gpio_card_ro_invert)
{
	return has_platform_data && !has_firmware && !gpio_card_ro_invert;
}

static inline unsigned int pxamci_irq_mask_all(bool pxa25x)
{
	return pxa25x ? MMC_I_MASK_ALL_PXA25X : MMC_I_MASK_ALL_PXA27X;
}

static inline bool
pxamci_data_size_supported(unsigned int blocks, unsigned int blksz)
{
	u64 bytes = (u64)blocks * blksz;

	return bytes && bytes != 1 && bytes != 3;
}

static inline int pxamci_data_error(unsigned int stat, bool pxa3xx)
{
	if (stat & STAT_READ_TIME_OUT)
		return -ETIMEDOUT;
	if (stat & (STAT_CRC_READ_ERROR | STAT_CRC_WRITE_ERROR))
		return -EILSEQ;
	if (pxa3xx && stat & STAT_FLASH_ERR)
		return -EIO;

	return 0;
}

static inline unsigned int
pxamci_bytes_xfered(unsigned int blocks, unsigned int blksz,
		    unsigned int blocks_remaining, bool counter_valid,
		    bool transfer_ok)
{
	if (transfer_ok)
		return blocks * blksz;
	if (!counter_valid)
		return 0;

	blocks_remaining = min(blocks_remaining, blocks);
	return (blocks - blocks_remaining) * blksz;
}

static inline unsigned int
pxamci_stop_cmdat(unsigned int data_cmdat, bool supports_stop)
{
	data_cmdat &= ~CMDAT_INIT;

	return supports_stop ? data_cmdat | CMDAT_STOP_TRAN :
		data_cmdat & ~(CMDAT_DATAEN | CMDAT_DMAEN | CMDAT_WRITE |
				CMDAT_STREAM);
}

static inline bool
pxamci_power_failure_disables_clock(unsigned int old_clkrt,
				    unsigned int new_clkrt)
{
	return old_clkrt == PXAMCI_CLKRT_OFF && new_clkrt != PXAMCI_CLKRT_OFF;
}

static inline bool pxamci_bus_width_caps_valid(unsigned int caps, bool pxa25x)
{
	if (caps & MMC_CAP_8_BIT_DATA)
		return false;

	return !pxa25x || !(caps & MMC_CAP_4_BIT_DATA);
}

static inline unsigned int pxamci_detect_debounce_us(unsigned long delay_ms)
{
	if (delay_ms > UINT_MAX / USEC_PER_MSEC)
		return UINT_MAX;

	return delay_ms * USEC_PER_MSEC;
}

static inline bool pxamci_dma_safe_to_release(int terminate_ret)
{
	return terminate_ret == 0;
}

static inline enum pxamci_lifecycle_action
pxamci_cmd_done_action(bool data_active, bool command_failed,
		       bool command_is_sbc, bool needs_program_done,
		       bool program_done)
{
	if (command_failed)
		return data_active ? PXAMCI_ACTION_RECOVER :
				     PXAMCI_ACTION_FINISH_REQUEST;
	if (command_is_sbc)
		return PXAMCI_ACTION_START_COMMAND;
	if (!data_active)
		return needs_program_done && !program_done ?
			PXAMCI_ACTION_WAIT_FOR_EVENT :
			PXAMCI_ACTION_FINISH_REQUEST;

	return PXAMCI_ACTION_START_DATA;
}

static inline bool
pxamci_dma_starts_before_command(bool pxa27x, bool data_read)
{
	return !pxa27x || data_read;
}

static inline bool
pxamci_dma_starts_after_command(bool pxa27x, bool data_write)
{
	return pxa27x && data_write;
}

static inline enum pxamci_lifecycle_action
pxamci_data_done_action(bool data_failed, bool dma_done)
{
	if (data_failed)
		return PXAMCI_ACTION_RECOVER;
	if (!dma_done)
		return PXAMCI_ACTION_WAIT_FOR_DMA;

	return PXAMCI_ACTION_FINISH_DATA;
}

static inline bool
pxamci_dma_is_current(const void *active_dma, const void *callback_dma,
		      const void *active_data, const void *callback_data,
		      unsigned int active_request,
		      unsigned int callback_request, bool finishing)
{
	return callback_dma && callback_data && active_dma == callback_dma &&
	       active_data == callback_data && active_request == callback_request &&
	       !finishing;
}

static inline enum pxamci_lifecycle_action
pxamci_dma_done_action(enum pxamci_dma_result result,
		       bool data_done_pending)
{
	switch (result) {
	case PXAMCI_DMA_COMPLETE:
		return data_done_pending ? PXAMCI_ACTION_FINISH_DATA :
					   PXAMCI_ACTION_RECORD_DMA_DONE;
	case PXAMCI_DMA_FAILED:
		return PXAMCI_ACTION_RECOVER;
	case PXAMCI_DMA_RUNNING:
	default:
		return PXAMCI_ACTION_IGNORE;
	}
}

static inline enum pxamci_lifecycle_action
pxamci_finish_data_action(bool abort_request, bool data_failed, bool has_sbc,
			  bool has_stop, bool data_write, bool program_done)
{
	if (abort_request)
		return PXAMCI_ACTION_FINISH_REQUEST;
	if (data_failed && has_stop)
		return PXAMCI_ACTION_START_STOP;
	if (!data_failed && has_sbc)
		return data_write && !program_done ?
			PXAMCI_ACTION_WAIT_FOR_EVENT :
			PXAMCI_ACTION_FINISH_REQUEST;
	if (has_stop)
		return PXAMCI_ACTION_START_STOP;
	if (data_write && !program_done)
		return PXAMCI_ACTION_WAIT_FOR_EVENT;

	return PXAMCI_ACTION_FINISH_REQUEST;
}

static inline enum pxamci_lifecycle_action
pxamci_program_done_action(bool request_current, bool waiting_for_program)
{
	if (!request_current)
		return PXAMCI_ACTION_IGNORE;

	return waiting_for_program ? PXAMCI_ACTION_FINISH_REQUEST :
				     PXAMCI_ACTION_RECORD_PROGRAM_DONE;
}

static inline struct pxamci_watchdog_decision
pxamci_watchdog_decide(const struct pxamci_watchdog_state *state)
{
	struct pxamci_watchdog_decision decision = {
		.action = PXAMCI_ACTION_IGNORE,
	};

	if (!state->request_current)
		return decision;

	if (state->command_active) {
		if (!state->deadline_expired) {
			decision.action = PXAMCI_ACTION_WAIT_FOR_EVENT;
			return decision;
		}

		decision.action = state->data_active ? PXAMCI_ACTION_RECOVER :
						     PXAMCI_ACTION_FINISH_REQUEST;
		decision.reason = PXAMCI_RECOVERY_COMMAND;
		decision.abort_request = state->data_active;
		decision.set_command_timeout = true;
		return decision;
	}

	if (state->program_active) {
		if (state->program_done) {
			decision.action = PXAMCI_ACTION_FINISH_REQUEST;
			decision.reason = PXAMCI_RECOVERY_LOST_COMPLETION;
			return decision;
		}
		if (!state->deadline_expired) {
			decision.action = PXAMCI_ACTION_WAIT_FOR_EVENT;
			return decision;
		}

		decision.action = PXAMCI_ACTION_FINISH_REQUEST;
		decision.reason = PXAMCI_RECOVERY_PROGRAM;
		decision.set_program_timeout = true;
		return decision;
	}

	if (!state->data_active ||
	    (state->finishing && !state->abort_request))
		return decision;

	decision.action = PXAMCI_ACTION_RECOVER;
	decision.abort_request = state->abort_request;

	if (state->abort_request) {
		decision.reason = PXAMCI_RECOVERY_COMMAND;
	} else if (state->dma_failed) {
		decision.reason = PXAMCI_RECOVERY_DMA;
	} else if (state->data_failed) {
		decision.reason = PXAMCI_RECOVERY_CONTROLLER;
	} else if (state->dma_complete &&
		   (state->controller_done || state->data_done_pending)) {
		decision.reason = PXAMCI_RECOVERY_LOST_COMPLETION;
	} else if (state->deadline_expired) {
		decision.reason = PXAMCI_RECOVERY_TIMEOUT;
		decision.set_data_timeout = true;
	} else {
		decision.action = PXAMCI_ACTION_WAIT_FOR_EVENT;
	}

	/*
	 * PXA erratum 17 forbids stopping DMA while the controller can still
	 * request data.  A command failure happens before data starts; every
	 * other recovery must observe the controller's terminal state first.
	 */
	if (decision.action == PXAMCI_ACTION_RECOVER &&
	    decision.reason != PXAMCI_RECOVERY_COMMAND &&
	    !state->controller_done && !state->data_done_pending)
		decision.action = PXAMCI_ACTION_WAIT_FOR_EVENT;

	return decision;
}

static inline int
pxamci_quiesce_sequence(const struct pxamci_quiesce_ops *ops, void *data)
{
	int ret;
	int tx_ret;

	ops->cancel_work(data);
	ret = ops->terminate_dma(data, false);
	tx_ret = ops->terminate_dma(data, true);
	ops->cancel_work(data);

	return ret ?: tx_ret;
}

#endif
