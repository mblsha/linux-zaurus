// SPDX-License-Identifier: GPL-2.0-only

#include <kunit/test.h>
#include <linux/errno.h>
#include <linux/kernel.h>

#include "pxamci-internal.h"

static void pxamci_command_completion_test(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, PXAMCI_ACTION_START_DATA,
			pxamci_cmd_done_action(true, false));
	KUNIT_EXPECT_EQ(test, PXAMCI_ACTION_RECOVER,
			pxamci_cmd_done_action(true, true));
	KUNIT_EXPECT_EQ(test, PXAMCI_ACTION_FINISH_REQUEST,
			pxamci_cmd_done_action(false, true));
}

static void pxamci_controller_completion_test(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, PXAMCI_ACTION_RECOVER,
			pxamci_data_done_action(true, false));
	KUNIT_EXPECT_EQ(test, PXAMCI_ACTION_WAIT_FOR_DMA,
			pxamci_data_done_action(false, false));
	KUNIT_EXPECT_EQ(test, PXAMCI_ACTION_FINISH_DATA,
			pxamci_data_done_action(false, true));
}

static void pxamci_stale_dma_callback_test(struct kunit *test)
{
	int data_a;
	int data_b;
	int dma_a;
	int dma_b;
	bool callback_current;

	callback_current = pxamci_dma_is_current(&dma_a, &dma_a, &data_a, &data_a, false);
	KUNIT_EXPECT_TRUE(test, callback_current);

	callback_current = pxamci_dma_is_current(&dma_b, &dma_a, &data_b, &data_a, false);
	KUNIT_EXPECT_FALSE(test, callback_current);

	callback_current = pxamci_dma_is_current(&dma_a, &dma_a, &data_a, &data_a, true);
	KUNIT_EXPECT_FALSE(test, callback_current);
}

static void pxamci_dma_completion_test(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, PXAMCI_ACTION_RECORD_DMA_DONE,
			pxamci_dma_done_action(PXAMCI_DMA_COMPLETE, false));
	KUNIT_EXPECT_EQ(test, PXAMCI_ACTION_FINISH_DATA,
			pxamci_dma_done_action(PXAMCI_DMA_COMPLETE, true));
	KUNIT_EXPECT_EQ(test, PXAMCI_ACTION_RECOVER,
			pxamci_dma_done_action(PXAMCI_DMA_FAILED, false));
	KUNIT_EXPECT_EQ(test, PXAMCI_ACTION_IGNORE,
			pxamci_dma_done_action(PXAMCI_DMA_RUNNING, true));
}

static void pxamci_read_happy_path_test(struct kunit *test)
{
	enum pxamci_lifecycle_action action;

	KUNIT_EXPECT_TRUE(test, pxamci_dma_starts_before_command(false, true));
	KUNIT_EXPECT_FALSE(test, pxamci_dma_starts_after_command(false, false));

	action = pxamci_cmd_done_action(true, false);
	KUNIT_ASSERT_EQ(test, PXAMCI_ACTION_START_DATA, action);

	/* The controller can complete before the DMA callback. */
	action = pxamci_data_done_action(false, false);
	KUNIT_ASSERT_EQ(test, PXAMCI_ACTION_WAIT_FOR_DMA, action);
	action = pxamci_dma_done_action(PXAMCI_DMA_COMPLETE, true);
	KUNIT_ASSERT_EQ(test, PXAMCI_ACTION_FINISH_DATA, action);

	action = pxamci_finish_data_action(false, false);
	KUNIT_EXPECT_EQ(test, PXAMCI_ACTION_FINISH_REQUEST, action);
}

static void pxamci_write_with_stop_happy_path_test(struct kunit *test)
{
	enum pxamci_lifecycle_action action;

	KUNIT_EXPECT_TRUE(test, pxamci_dma_starts_before_command(false, false));
	KUNIT_EXPECT_FALSE(test, pxamci_dma_starts_before_command(true, false));
	KUNIT_EXPECT_TRUE(test, pxamci_dma_starts_after_command(true, true));

	action = pxamci_cmd_done_action(true, false);
	KUNIT_ASSERT_EQ(test, PXAMCI_ACTION_START_DATA, action);

	/* The DMA callback can complete before the controller interrupt. */
	action = pxamci_dma_done_action(PXAMCI_DMA_COMPLETE, false);
	KUNIT_ASSERT_EQ(test, PXAMCI_ACTION_RECORD_DMA_DONE, action);
	action = pxamci_data_done_action(false, true);
	KUNIT_ASSERT_EQ(test, PXAMCI_ACTION_FINISH_DATA, action);

	action = pxamci_finish_data_action(false, true);
	KUNIT_ASSERT_EQ(test, PXAMCI_ACTION_START_STOP, action);
	action = pxamci_cmd_done_action(false, false);
	KUNIT_EXPECT_EQ(test, PXAMCI_ACTION_FINISH_REQUEST, action);
}

struct pxamci_watchdog_case {
	const char *name;
	struct pxamci_watchdog_state state;
	enum pxamci_lifecycle_action action;
	enum pxamci_recovery_reason reason;
	bool abort_request;
	bool set_command_timeout;
	bool set_data_timeout;
};

static const struct pxamci_watchdog_case pxamci_watchdog_cases[] = {
	{
		.name = "stale transfer",
		.state = { },
		.action = PXAMCI_ACTION_IGNORE,
	}, {
		.name = "completion already claimed",
		.state = {
			.transfer_current = true,
			.finishing = true,
		},
		.action = PXAMCI_ACTION_IGNORE,
	}, {
		.name = "command abort",
		.state = {
			.transfer_current = true,
			.finishing = true,
			.abort_request = true,
		},
		.action = PXAMCI_ACTION_RECOVER,
		.reason = PXAMCI_RECOVERY_COMMAND,
		.abort_request = true,
	}, {
		.name = "DMA error wins over recorded data error",
		.state = {
			.transfer_current = true,
			.dma_failed = true,
			.data_failed = true,
		},
		.action = PXAMCI_ACTION_RECOVER,
		.reason = PXAMCI_RECOVERY_DMA,
	}, {
		.name = "controller error",
		.state = {
			.transfer_current = true,
			.data_failed = true,
		},
		.action = PXAMCI_ACTION_RECOVER,
		.reason = PXAMCI_RECOVERY_CONTROLLER,
	}, {
		.name = "lost DMA callback",
		.state = {
			.transfer_current = true,
			.dma_complete = true,
			.data_done_pending = true,
		},
		.action = PXAMCI_ACTION_RECOVER,
		.reason = PXAMCI_RECOVERY_LOST_COMPLETION,
	}, {
		.name = "lost controller callback",
		.state = {
			.transfer_current = true,
			.controller_done = true,
		},
		.action = PXAMCI_ACTION_RECOVER,
		.reason = PXAMCI_RECOVERY_LOST_COMPLETION,
	}, {
		.name = "ordinary transfer still active",
		.state = {
			.transfer_current = true,
			.dma_has_residue = true,
		},
		.action = PXAMCI_ACTION_WAIT_FOR_DMA,
	}, {
		.name = "command deadline",
		.state = {
			.transfer_current = true,
			.dma_has_residue = true,
			.deadline_expired = true,
			.command_active = true,
		},
		.action = PXAMCI_ACTION_RECOVER,
		.reason = PXAMCI_RECOVERY_COMMAND,
		.abort_request = true,
		.set_command_timeout = true,
	}, {
		.name = "data deadline",
		.state = {
			.transfer_current = true,
			.dma_has_residue = true,
			.deadline_expired = true,
		},
		.action = PXAMCI_ACTION_RECOVER,
		.reason = PXAMCI_RECOVERY_TIMEOUT,
		.set_data_timeout = true,
	},
};

static void pxamci_watchdog_test(struct kunit *test)
{
	struct pxamci_watchdog_decision decision;
	const struct pxamci_watchdog_case *test_case;
	size_t i;

	for (i = 0; i < ARRAY_SIZE(pxamci_watchdog_cases); i++) {
		test_case = &pxamci_watchdog_cases[i];
		decision = pxamci_watchdog_decide(&test_case->state);

		KUNIT_EXPECT_EQ_MSG(test, test_case->action, decision.action,
				    "%s: action", test_case->name);
		KUNIT_EXPECT_EQ_MSG(test, test_case->reason, decision.reason,
				    "%s: reason", test_case->name);
		KUNIT_EXPECT_EQ_MSG(test, test_case->abort_request,
				    decision.abort_request,
				    "%s: abort request", test_case->name);
		KUNIT_EXPECT_EQ_MSG(test, test_case->set_command_timeout,
				    decision.set_command_timeout,
				    "%s: command timeout", test_case->name);
		KUNIT_EXPECT_EQ_MSG(test, test_case->set_data_timeout,
				    decision.set_data_timeout,
				    "%s: data timeout", test_case->name);
	}
}

enum pxamci_quiesce_event {
	PXAMCI_QUIESCE_CANCEL,
	PXAMCI_QUIESCE_RX,
	PXAMCI_QUIESCE_TX,
};

struct pxamci_quiesce_trace {
	enum pxamci_quiesce_event events[4];
	unsigned int event_count;
	int rx_ret;
	int tx_ret;
};

static void pxamci_test_cancel_work(void *data)
{
	struct pxamci_quiesce_trace *trace = data;

	trace->events[trace->event_count++] = PXAMCI_QUIESCE_CANCEL;
}

static int pxamci_test_terminate_dma(void *data, bool tx)
{
	struct pxamci_quiesce_trace *trace = data;

	trace->events[trace->event_count++] =
		tx ? PXAMCI_QUIESCE_TX : PXAMCI_QUIESCE_RX;

	return tx ? trace->tx_ret : trace->rx_ret;
}

static void pxamci_quiesce_test(struct kunit *test)
{
	static const struct pxamci_quiesce_ops ops = {
		.cancel_work = pxamci_test_cancel_work,
		.terminate_dma = pxamci_test_terminate_dma,
	};
	struct pxamci_quiesce_trace trace = {
		.rx_ret = -EIO,
		.tx_ret = -ETIMEDOUT,
	};
	int ret;

	ret = pxamci_quiesce_sequence(&ops, &trace);

	KUNIT_EXPECT_EQ(test, -EIO, ret);
	KUNIT_ASSERT_EQ(test, 4U, trace.event_count);
	KUNIT_EXPECT_EQ(test, PXAMCI_QUIESCE_CANCEL, trace.events[0]);
	KUNIT_EXPECT_EQ(test, PXAMCI_QUIESCE_RX, trace.events[1]);
	KUNIT_EXPECT_EQ(test, PXAMCI_QUIESCE_TX, trace.events[2]);
	KUNIT_EXPECT_EQ(test, PXAMCI_QUIESCE_CANCEL, trace.events[3]);

	trace = (struct pxamci_quiesce_trace) {
		.tx_ret = -ETIMEDOUT,
	};
	ret = pxamci_quiesce_sequence(&ops, &trace);
	KUNIT_EXPECT_EQ(test, -ETIMEDOUT, ret);
	KUNIT_EXPECT_EQ(test, 4U, trace.event_count);

	trace = (struct pxamci_quiesce_trace) { };
	ret = pxamci_quiesce_sequence(&ops, &trace);
	KUNIT_EXPECT_EQ(test, 0, ret);
	KUNIT_ASSERT_EQ(test, 4U, trace.event_count);
	KUNIT_EXPECT_EQ(test, PXAMCI_QUIESCE_CANCEL, trace.events[0]);
	KUNIT_EXPECT_EQ(test, PXAMCI_QUIESCE_RX, trace.events[1]);
	KUNIT_EXPECT_EQ(test, PXAMCI_QUIESCE_TX, trace.events[2]);
	KUNIT_EXPECT_EQ(test, PXAMCI_QUIESCE_CANCEL, trace.events[3]);
}

static struct kunit_case pxamci_test_cases[] = {
	KUNIT_CASE(pxamci_command_completion_test),
	KUNIT_CASE(pxamci_controller_completion_test),
	KUNIT_CASE(pxamci_stale_dma_callback_test),
	KUNIT_CASE(pxamci_dma_completion_test),
	KUNIT_CASE(pxamci_read_happy_path_test),
	KUNIT_CASE(pxamci_write_with_stop_happy_path_test),
	KUNIT_CASE(pxamci_watchdog_test),
	KUNIT_CASE(pxamci_quiesce_test),
	{ }
};

static struct kunit_suite pxamci_test_suite = {
	.name = "pxamci-lifecycle",
	.test_cases = pxamci_test_cases,
};

kunit_test_suite(pxamci_test_suite);

MODULE_DESCRIPTION("KUnit tests for the PXA MMC request lifecycle");
MODULE_LICENSE("GPL");
