/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _PXAMCI_INTERNAL_H
#define _PXAMCI_INTERNAL_H

#include <linux/types.h>

enum pxamci_lifecycle_action {
	PXAMCI_ACTION_IGNORE,
	PXAMCI_ACTION_START_DATA,
	PXAMCI_ACTION_START_STOP,
	PXAMCI_ACTION_FINISH_REQUEST,
	PXAMCI_ACTION_WAIT_FOR_DMA,
	PXAMCI_ACTION_RECORD_DMA_DONE,
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
	PXAMCI_RECOVERY_TIMEOUT,
};

struct pxamci_watchdog_state {
	bool transfer_current;
	bool finishing;
	bool abort_request;
	bool dma_failed;
	bool data_failed;
	bool dma_complete;
	bool controller_done;
	bool data_done_pending;
	bool dma_has_residue;
	bool deadline_expired;
	bool command_active;
};

struct pxamci_watchdog_decision {
	enum pxamci_lifecycle_action action;
	enum pxamci_recovery_reason reason;
	bool abort_request;
	bool set_command_timeout;
	bool set_data_timeout;
};

struct pxamci_quiesce_ops {
	void (*cancel_work)(void *data);
	int (*terminate_dma)(void *data, bool tx);
};

static inline enum pxamci_lifecycle_action
pxamci_cmd_done_action(bool data_active, bool command_failed)
{
	if (!data_active)
		return PXAMCI_ACTION_FINISH_REQUEST;
	if (command_failed)
		return PXAMCI_ACTION_RECOVER;

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
		      bool finishing)
{
	return callback_dma && callback_data && active_dma == callback_dma &&
	       active_data == callback_data && !finishing;
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
pxamci_finish_data_action(bool abort_request, bool has_stop)
{
	if (!abort_request && has_stop)
		return PXAMCI_ACTION_START_STOP;

	return PXAMCI_ACTION_FINISH_REQUEST;
}

static inline struct pxamci_watchdog_decision
pxamci_watchdog_decide(const struct pxamci_watchdog_state *state)
{
	struct pxamci_watchdog_decision decision = {
		.action = PXAMCI_ACTION_IGNORE,
	};

	if (!state->transfer_current ||
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
	} else if ((state->dma_complete &&
		    (state->controller_done || state->data_done_pending)) ||
		   (state->controller_done && !state->dma_has_residue)) {
		decision.reason = PXAMCI_RECOVERY_LOST_COMPLETION;
	} else if (state->deadline_expired) {
		if (state->command_active) {
			decision.reason = PXAMCI_RECOVERY_COMMAND;
			decision.abort_request = true;
			decision.set_command_timeout = true;
		} else {
			decision.reason = PXAMCI_RECOVERY_TIMEOUT;
			decision.set_data_timeout = true;
		}
	} else {
		decision.action = PXAMCI_ACTION_WAIT_FOR_DMA;
	}

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
