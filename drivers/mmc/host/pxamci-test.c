// SPDX-License-Identifier: GPL-2.0-only

#include <kunit/test.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/limits.h>

#include "pxamci-internal.h"

static void pxamci_clock_config_test(struct kunit *test)
{
	struct pxamci_clock_config config;

	config = pxamci_clock_config(19500000, 6500000, false, false);
	KUNIT_EXPECT_EQ(test, 2U, config.clkrt);
	KUNIT_EXPECT_EQ(test, 4875000U, config.actual_clock);

	config = pxamci_clock_config(19500000, 9750000, false, false);
	KUNIT_EXPECT_EQ(test, 1U, config.clkrt);
	KUNIT_EXPECT_EQ(test, 9750000U, config.actual_clock);

	config = pxamci_clock_config(13000000, 26000000, true, false);
	KUNIT_EXPECT_EQ(test, PXAMCI_26MHZ_CLKRT, config.clkrt);
	KUNIT_EXPECT_EQ(test, 26000000U, config.actual_clock);

	config = pxamci_clock_config(19500000, 0, false, false);
	KUNIT_EXPECT_EQ(test, PXAMCI_CLKRT_OFF, config.clkrt);
	KUNIT_EXPECT_EQ(test, 0U, config.actual_clock);

	/* PXA320 B2 must never select the glitching divide-by-four rate. */
	config = pxamci_clock_config(19500000, 6500000, false, true);
	KUNIT_EXPECT_EQ(test, 3U, config.clkrt);
	KUNIT_EXPECT_EQ(test, 2437500U, config.actual_clock);
}

static void pxamci_timeout_calculation_test(struct kunit *test)
{
	unsigned int timeout;

	/* The 16-bit hardware timeout must saturate instead of wrapping. */
	timeout = pxamci_read_timeout_reg(NSEC_PER_SEC, 0, 19500000, 19500000);
	KUNIT_EXPECT_EQ(test, (unsigned int)U16_MAX, timeout);
	timeout = pxamci_read_timeout_reg(100 * NSEC_PER_MSEC, 0,
					  19500000, 19500000);
	KUNIT_EXPECT_EQ(test, 7618U, timeout);
	timeout = pxamci_read_timeout_reg(0, 256, 19500000, 9750000);
	KUNIT_EXPECT_EQ(test, 2U, timeout);
	/* CLKRT=7 selects 26 MHz; it is not a divide-by-128 encoding. */
	timeout = pxamci_read_timeout_reg(0, 256, 19500000, 26000000);
	KUNIT_EXPECT_EQ(test, 1U, timeout);

	/* A legal three-second write gets its full timeout plus headroom. */
	timeout = pxamci_data_timeout_ms(3 * NSEC_PER_SEC, 0, 19500000,
					 512, 1);
	KUNIT_EXPECT_GT(test, timeout, 3000U);
	KUNIT_EXPECT_EQ(test, 4002U, timeout);

	timeout = pxamci_data_timeout_ms(0, 19500, 19500000, 0, 1);
	KUNIT_EXPECT_EQ(test, 1001U, timeout);
	KUNIT_EXPECT_EQ(test, UINT_MAX,
			pxamci_data_timeout_ms(0, 1, 0, 0, 1));

	KUNIT_EXPECT_EQ(test, 2000U, pxamci_command_timeout_ms(0, 0));
	KUNIT_EXPECT_EQ(test, 6000U, pxamci_command_timeout_ms(5000, 0));
	KUNIT_EXPECT_EQ(test, 4002U, pxamci_command_timeout_ms(0, 4002));
	KUNIT_EXPECT_EQ(test, 6000U, pxamci_command_timeout_ms(5000, 4002));
}

static void pxamci_platform_helpers_test(struct kunit *test)
{
	unsigned int caps = MMC_CAP_NONREMOVABLE;
	bool valid;

	caps = pxamci_merge_caps(caps, false, true, false);
	KUNIT_EXPECT_TRUE(test, caps & MMC_CAP_NONREMOVABLE);
	KUNIT_EXPECT_TRUE(test, caps & MMC_CAP_4_BIT_DATA);
	KUNIT_EXPECT_TRUE(test, caps & MMC_CAP_SDIO_IRQ);
	KUNIT_EXPECT_TRUE(test, caps & MMC_CAP_CMD23);
	KUNIT_EXPECT_TRUE(test, caps & MMC_CAP_SD_HIGHSPEED);
	KUNIT_EXPECT_TRUE(test, pxamci_bus_width_caps_valid(caps, false));
	valid = pxamci_bus_width_caps_valid(caps | MMC_CAP_8_BIT_DATA, false);
	KUNIT_EXPECT_FALSE(test, valid);
	valid = pxamci_bus_width_caps_valid(MMC_CAP_4_BIT_DATA, true);
	KUNIT_EXPECT_FALSE(test, valid);

	caps = pxamci_merge_caps(0, false, false, true);
	KUNIT_EXPECT_FALSE(test, caps & MMC_CAP_4_BIT_DATA);
	KUNIT_EXPECT_TRUE(test, caps & MMC_CAP_SDIO_IRQ);
	caps = pxamci_merge_caps(0, true, false, false);
	KUNIT_EXPECT_FALSE(test, caps & MMC_CAP_4_BIT_DATA);
	KUNIT_EXPECT_FALSE(test, caps & MMC_CAP_CMD23);

	KUNIT_EXPECT_TRUE(test, pxamci_use_legacy_ro_active_high(true, false,
								false));
	KUNIT_EXPECT_FALSE(test, pxamci_use_legacy_ro_active_high(true, true,
								 false));
	KUNIT_EXPECT_FALSE(test, pxamci_use_legacy_ro_active_high(true, false,
								 true));
	KUNIT_EXPECT_EQ(test, MMC_I_MASK_ALL_PXA25X,
			pxamci_irq_mask_all(true));
	KUNIT_EXPECT_EQ(test, MMC_I_MASK_ALL_PXA27X,
			pxamci_irq_mask_all(false));

	KUNIT_EXPECT_EQ(test, 250000U, pxamci_detect_debounce_us(250));
	KUNIT_EXPECT_EQ(test, UINT_MAX, pxamci_detect_debounce_us(ULONG_MAX));
	KUNIT_EXPECT_TRUE(test, pxamci_dma_safe_to_release(0));
	KUNIT_EXPECT_FALSE(test, pxamci_dma_safe_to_release(-EIO));
	KUNIT_EXPECT_TRUE(test,
			  pxamci_data_size_supported(1, 2, false, true, false));
	KUNIT_EXPECT_FALSE(test,
			   pxamci_data_size_supported(1, 1, false, true, false));
	KUNIT_EXPECT_FALSE(test,
			   pxamci_data_size_supported(1, 3, false, true, false));
	KUNIT_EXPECT_TRUE(test,
			  pxamci_data_size_supported(2, 512, true, true, true));

	/* PXA27x erratum E58 minimums apply only to reads. */
	KUNIT_EXPECT_FALSE(test,
			   pxamci_data_size_supported(1, 2, true, true, false));
	KUNIT_EXPECT_FALSE(test,
			   pxamci_data_size_supported(1, 4, true, true, false));
	KUNIT_EXPECT_TRUE(test,
			  pxamci_data_size_supported(1, 8, true, true, false));
	KUNIT_EXPECT_FALSE(test,
			   pxamci_data_size_supported(1, 8, true, true, true));
	KUNIT_EXPECT_FALSE(test,
			   pxamci_data_size_supported(1, 16, true, true, true));
	KUNIT_EXPECT_TRUE(test,
			  pxamci_data_size_supported(1, 32, true, true, true));
	KUNIT_EXPECT_TRUE(test,
			  pxamci_data_size_supported(1, 2, true, false, true));

	KUNIT_EXPECT_EQ(test, 19500000U,
			pxamci_limit_sdio_clock(19500000, false, true));
	KUNIT_EXPECT_EQ(test, 19500000U,
			pxamci_limit_sdio_clock(19500000, true, false));
	KUNIT_EXPECT_EQ(test, 9750000U,
			pxamci_limit_sdio_clock(19500000, true, true));
	KUNIT_EXPECT_TRUE(test, pxamci_is_pxa27x_c0(0x69054114));
	KUNIT_EXPECT_FALSE(test, pxamci_is_pxa27x_c0(0x69054117));
	KUNIT_EXPECT_TRUE(test, pxamci_is_pxa320_b2(0x69056826));
	KUNIT_EXPECT_FALSE(test, pxamci_is_pxa320_b2(0x69056825));

	KUNIT_EXPECT_TRUE(test, pxamci_power_failure_disables_clock(
						PXAMCI_CLKRT_OFF, 2));
	KUNIT_EXPECT_FALSE(test, pxamci_power_failure_disables_clock(2, 1));
}

static void pxamci_status_and_progress_test(struct kunit *test)
{
	unsigned int data_cmdat = CMDAT_INIT | CMDAT_SD_4DAT | CMDAT_DMAEN |
		CMDAT_DATAEN | CMDAT_WRITE;

	KUNIT_EXPECT_EQ(test, -ETIMEDOUT,
			pxamci_data_error(STAT_READ_TIME_OUT, true));
	KUNIT_EXPECT_EQ(test, -EILSEQ,
			pxamci_data_error(STAT_CRC_WRITE_ERROR, true));
	KUNIT_EXPECT_EQ(test, -EIO,
			pxamci_data_error(STAT_FLASH_ERR, true));
	KUNIT_EXPECT_EQ(test, 0,
			pxamci_data_error(STAT_FLASH_ERR, false));

	KUNIT_EXPECT_TRUE(test,
			  pxamci_blocks_remaining_valid(false,
							STAT_READ_TIME_OUT,
							-ETIMEDOUT));
	KUNIT_EXPECT_FALSE(test,
			   pxamci_blocks_remaining_valid(false,
							 STAT_CRC_READ_ERROR,
							 -EILSEQ));
	KUNIT_EXPECT_FALSE(test,
			   pxamci_blocks_remaining_valid(false, 0, -EIO));
	KUNIT_EXPECT_FALSE(test,
			   pxamci_blocks_remaining_valid(true,
							 STAT_READ_TIME_OUT,
							 -ETIMEDOUT));

	KUNIT_EXPECT_EQ(test, 4096U,
			pxamci_bytes_xfered(8, 512, 8, true, true));
	KUNIT_EXPECT_EQ(test, 1536U,
			pxamci_bytes_xfered(8, 512, 5, true, false));
	KUNIT_EXPECT_EQ(test, 0U,
			pxamci_bytes_xfered(8, 512, 5, false, false));
	KUNIT_EXPECT_EQ(test, -EILSEQ,
			pxamci_preserve_error(-EILSEQ, -ETIMEDOUT));
	KUNIT_EXPECT_EQ(test, -ETIMEDOUT,
			pxamci_preserve_error(0, -ETIMEDOUT));

	KUNIT_EXPECT_EQ(test, (data_cmdat & ~CMDAT_INIT) | CMDAT_STOP_TRAN,
			pxamci_stop_cmdat(data_cmdat, true));
	KUNIT_EXPECT_EQ(test, (unsigned int)CMDAT_SD_4DAT,
			pxamci_stop_cmdat(data_cmdat, false));
}

static void pxamci_command_completion_test(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, PXAMCI_ACTION_START_DATA,
			pxamci_cmd_done_action(true, false, false, false,
					       false, false));
	KUNIT_EXPECT_EQ(test, PXAMCI_ACTION_RECOVER,
			pxamci_cmd_done_action(true, true, false, false,
					       false, false));
	KUNIT_EXPECT_EQ(test, PXAMCI_ACTION_WAIT_FOR_DATA,
			pxamci_cmd_done_action(true, true, true, false,
					       false, false));
	KUNIT_EXPECT_EQ(test, PXAMCI_ACTION_FINISH_REQUEST,
			pxamci_cmd_done_action(false, true, false, false,
					       false, false));
	KUNIT_EXPECT_EQ(test, PXAMCI_ACTION_START_COMMAND,
			pxamci_cmd_done_action(false, false, false, true,
					       false, false));
	KUNIT_EXPECT_EQ(test, PXAMCI_ACTION_WAIT_FOR_EVENT,
			pxamci_cmd_done_action(false, false, false, false,
					       true, false));
	KUNIT_EXPECT_EQ(test, PXAMCI_ACTION_FINISH_REQUEST,
			pxamci_cmd_done_action(false, false, false, false, true,
					       true));

	KUNIT_EXPECT_EQ(test, PXAMCI_ACTION_RECORD_PROGRAM_DONE,
			pxamci_program_done_action(true, false));
	KUNIT_EXPECT_EQ(test, PXAMCI_ACTION_FINISH_REQUEST,
			pxamci_program_done_action(true, true));
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
	int data_a = 0;
	int data_b = 0;
	int dma_a = 0;
	int dma_b = 0;
	bool is_current;

	is_current = pxamci_dma_is_current(&dma_a, &dma_a, &data_a, &data_a,
					   1, 1, false);
	KUNIT_EXPECT_TRUE(test, is_current);

	is_current = pxamci_dma_is_current(&dma_b, &dma_a, &data_b, &data_a,
					   1, 1, false);
	KUNIT_EXPECT_FALSE(test, is_current);

	is_current = pxamci_dma_is_current(&dma_a, &dma_a, &data_a, &data_a,
					   1, 1, true);
	KUNIT_EXPECT_FALSE(test, is_current);

	/* Pointer reuse cannot make a callback from an old request current. */
	is_current = pxamci_dma_is_current(&dma_a, &dma_a, &data_a, &data_a,
					   2, 1, false);
	KUNIT_EXPECT_FALSE(test, is_current);
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

	action = pxamci_cmd_done_action(true, false, false, false, false, false);
	KUNIT_ASSERT_EQ(test, PXAMCI_ACTION_START_DATA, action);

	/* The controller can complete before the DMA callback. */
	action = pxamci_data_done_action(false, false);
	KUNIT_ASSERT_EQ(test, PXAMCI_ACTION_WAIT_FOR_DMA, action);
	action = pxamci_dma_done_action(PXAMCI_DMA_COMPLETE, true);
	KUNIT_ASSERT_EQ(test, PXAMCI_ACTION_FINISH_DATA, action);

	action = pxamci_finish_data_action(false, false, false, false, false,
					   false);
	KUNIT_EXPECT_EQ(test, PXAMCI_ACTION_FINISH_REQUEST, action);
}

static void pxamci_write_with_stop_happy_path_test(struct kunit *test)
{
	enum pxamci_lifecycle_action action;

	KUNIT_EXPECT_TRUE(test, pxamci_dma_starts_before_command(false, false));
	KUNIT_EXPECT_FALSE(test, pxamci_dma_starts_before_command(true, false));
	KUNIT_EXPECT_TRUE(test, pxamci_dma_starts_after_command(true, true));

	action = pxamci_cmd_done_action(true, false, false, false, true, false);
	KUNIT_ASSERT_EQ(test, PXAMCI_ACTION_START_DATA, action);

	/* The DMA callback can complete before the controller interrupt. */
	action = pxamci_dma_done_action(PXAMCI_DMA_COMPLETE, false);
	KUNIT_ASSERT_EQ(test, PXAMCI_ACTION_RECORD_DMA_DONE, action);
	action = pxamci_data_done_action(false, true);
	KUNIT_ASSERT_EQ(test, PXAMCI_ACTION_FINISH_DATA, action);

	action = pxamci_finish_data_action(false, false, false, true, true,
					   false);
	KUNIT_ASSERT_EQ(test, PXAMCI_ACTION_START_STOP, action);
	action = pxamci_cmd_done_action(false, false, false, false, true, false);
	KUNIT_ASSERT_EQ(test, PXAMCI_ACTION_WAIT_FOR_EVENT, action);
	action = pxamci_program_done_action(true, true);
	KUNIT_EXPECT_EQ(test, PXAMCI_ACTION_FINISH_REQUEST, action);
}

static void pxamci_sbc_write_happy_path_test(struct kunit *test)
{
	enum pxamci_lifecycle_action action;

	action = pxamci_cmd_done_action(false, false, false, true, false, false);
	KUNIT_ASSERT_EQ(test, PXAMCI_ACTION_START_COMMAND, action);
	action = pxamci_cmd_done_action(true, false, false, false, true, false);
	KUNIT_ASSERT_EQ(test, PXAMCI_ACTION_START_DATA, action);
	action = pxamci_data_done_action(false, true);
	KUNIT_ASSERT_EQ(test, PXAMCI_ACTION_FINISH_DATA, action);
	action = pxamci_finish_data_action(false, false, true, true, true,
					   false);
	KUNIT_ASSERT_EQ(test, PXAMCI_ACTION_WAIT_FOR_EVENT, action);
	action = pxamci_program_done_action(true, true);
	KUNIT_EXPECT_EQ(test, PXAMCI_ACTION_FINISH_REQUEST, action);

	/* A failed predefined transfer still sends CMD12 for recovery. */
	action = pxamci_finish_data_action(false, true, true, true, true,
					   false);
	KUNIT_EXPECT_EQ(test, PXAMCI_ACTION_START_STOP, action);

	/* A failed single-block write cannot wait for a nonexistent PRG_DONE. */
	action = pxamci_finish_data_action(false, true, false, false, true,
					   false);
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
	bool set_program_timeout;
	bool start_recovery;
	bool mark_host_dead;
};

static const struct pxamci_watchdog_case pxamci_watchdog_cases[] = {
	{
		.name = "stale transfer",
		.state = { },
		.action = PXAMCI_ACTION_IGNORE,
	}, {
		.name = "completion already claimed",
		.state = {
			.request_current = true,
			.data_active = true,
			.finishing = true,
		},
		.action = PXAMCI_ACTION_IGNORE,
	}, {
		.name = "lost command interrupt",
		.state = {
			.request_current = true,
			.command_active = true,
			.command_done = true,
		},
		.action = PXAMCI_ACTION_COMPLETE_COMMAND,
		.reason = PXAMCI_RECOVERY_LOST_COMPLETION,
	}, {
		.name = "command abort",
		.state = {
			.request_current = true,
			.data_active = true,
			.finishing = true,
			.abort_request = true,
		},
		.action = PXAMCI_ACTION_RECOVER,
		.reason = PXAMCI_RECOVERY_COMMAND,
		.abort_request = true,
	}, {
		.name = "DMA error wins over recorded data error",
		.state = {
			.request_current = true,
			.data_active = true,
			.dma_failed = true,
			.data_failed = true,
		},
		.action = PXAMCI_ACTION_WAIT_FOR_EVENT,
		.reason = PXAMCI_RECOVERY_DMA,
		.start_recovery = true,
	}, {
		.name = "DMA error after controller terminal",
		.state = {
			.request_current = true,
			.data_active = true,
			.dma_failed = true,
			.controller_done = true,
		},
		.action = PXAMCI_ACTION_RECOVER,
		.reason = PXAMCI_RECOVERY_DMA,
	}, {
		.name = "controller error",
		.state = {
			.request_current = true,
			.data_active = true,
			.data_failed = true,
		},
		.action = PXAMCI_ACTION_WAIT_FOR_EVENT,
		.reason = PXAMCI_RECOVERY_CONTROLLER,
		.start_recovery = true,
	}, {
		.name = "lost DMA callback",
		.state = {
			.request_current = true,
			.data_active = true,
			.dma_complete = true,
			.data_done_pending = true,
		},
		.action = PXAMCI_ACTION_RECOVER,
		.reason = PXAMCI_RECOVERY_LOST_COMPLETION,
	}, {
		.name = "controller status alone is not completion proof",
		.state = {
			.request_current = true,
			.data_active = true,
			.controller_done = true,
		},
		.action = PXAMCI_ACTION_WAIT_FOR_EVENT,
	}, {
		.name = "both terminal states lost their callbacks",
		.state = {
			.request_current = true,
			.data_active = true,
			.controller_done = true,
			.dma_complete = true,
		},
		.action = PXAMCI_ACTION_RECOVER,
		.reason = PXAMCI_RECOVERY_LOST_COMPLETION,
	}, {
		.name = "ordinary transfer still active",
		.state = {
			.request_current = true,
			.data_active = true,
		},
		.action = PXAMCI_ACTION_WAIT_FOR_EVENT,
	}, {
		.name = "data command still within deadline",
		.state = {
			.request_current = true,
			.data_active = true,
			.command_active = true,
		},
		.action = PXAMCI_ACTION_WAIT_FOR_EVENT,
	}, {
		.name = "data command deadline",
		.state = {
			.request_current = true,
			.data_active = true,
			.deadline_expired = true,
			.command_active = true,
			.dma_started = true,
		},
		.action = PXAMCI_ACTION_WAIT_FOR_EVENT,
		.reason = PXAMCI_RECOVERY_COMMAND,
		.start_recovery = true,
	}, {
		.name = "unstarted data command deadline",
		.state = {
			.request_current = true,
			.data_active = true,
			.deadline_expired = true,
			.command_active = true,
		},
		.action = PXAMCI_ACTION_RECOVER,
		.reason = PXAMCI_RECOVERY_COMMAND,
		.abort_request = true,
		.set_command_timeout = true,
	}, {
		.name = "data command recovery still within grace",
		.state = {
			.request_current = true,
			.data_active = true,
			.command_active = true,
			.dma_started = true,
			.recovery_pending = true,
		},
		.action = PXAMCI_ACTION_WAIT_FOR_EVENT,
	}, {
		.name = "data command reaches controller terminal",
		.state = {
			.request_current = true,
			.data_active = true,
			.command_active = true,
			.dma_started = true,
			.recovery_pending = true,
			.controller_done = true,
		},
		.action = PXAMCI_ACTION_RECOVER,
		.reason = PXAMCI_RECOVERY_COMMAND,
		.abort_request = true,
		.set_command_timeout = true,
	}, {
		.name = "data command recovery deadline",
		.state = {
			.request_current = true,
			.data_active = true,
			.command_active = true,
			.dma_started = true,
			.recovery_pending = true,
			.deadline_expired = true,
		},
		.action = PXAMCI_ACTION_RECOVER,
		.reason = PXAMCI_RECOVERY_COMMAND,
		.abort_request = true,
		.set_command_timeout = true,
		.mark_host_dead = true,
	}, {
		.name = "command-only deadline",
		.state = {
			.request_current = true,
			.deadline_expired = true,
			.command_active = true,
		},
		.action = PXAMCI_ACTION_FINISH_REQUEST,
		.reason = PXAMCI_RECOVERY_COMMAND,
		.set_command_timeout = true,
	}, {
		.name = "data deadline",
		.state = {
			.request_current = true,
			.data_active = true,
			.deadline_expired = true,
		},
		.action = PXAMCI_ACTION_WAIT_FOR_EVENT,
		.reason = PXAMCI_RECOVERY_TIMEOUT,
		.set_data_timeout = true,
		.start_recovery = true,
	}, {
		.name = "data recovery still within grace",
		.state = {
			.request_current = true,
			.data_active = true,
			.data_failed = true,
			.recovery_pending = true,
		},
		.action = PXAMCI_ACTION_WAIT_FOR_EVENT,
		.reason = PXAMCI_RECOVERY_CONTROLLER,
	}, {
		.name = "data recovery deadline",
		.state = {
			.request_current = true,
			.data_active = true,
			.data_failed = true,
			.recovery_pending = true,
			.deadline_expired = true,
		},
		.action = PXAMCI_ACTION_RECOVER,
		.reason = PXAMCI_RECOVERY_CONTROLLER,
		.abort_request = true,
		.mark_host_dead = true,
	}, {
		.name = "data deadline after controller terminal",
		.state = {
			.request_current = true,
			.data_active = true,
			.controller_done = true,
			.deadline_expired = true,
		},
		.action = PXAMCI_ACTION_RECOVER,
		.reason = PXAMCI_RECOVERY_TIMEOUT,
		.set_data_timeout = true,
	}, {
		.name = "programming still busy",
		.state = {
			.request_current = true,
			.program_active = true,
		},
		.action = PXAMCI_ACTION_WAIT_FOR_EVENT,
	}, {
		.name = "lost programming interrupt",
		.state = {
			.request_current = true,
			.program_active = true,
			.program_done = true,
		},
		.action = PXAMCI_ACTION_FINISH_REQUEST,
		.reason = PXAMCI_RECOVERY_LOST_COMPLETION,
	}, {
		.name = "programming deadline",
		.state = {
			.request_current = true,
			.program_active = true,
			.deadline_expired = true,
		},
		.action = PXAMCI_ACTION_FINISH_REQUEST,
		.reason = PXAMCI_RECOVERY_PROGRAM,
		.set_program_timeout = true,
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
		KUNIT_EXPECT_EQ_MSG(test, test_case->set_program_timeout,
				    decision.set_program_timeout,
				    "%s: program timeout", test_case->name);
		KUNIT_EXPECT_EQ_MSG(test, test_case->start_recovery,
				    decision.start_recovery,
				    "%s: start recovery", test_case->name);
		KUNIT_EXPECT_EQ_MSG(test, test_case->mark_host_dead,
				    decision.mark_host_dead,
				    "%s: mark host dead", test_case->name);
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
	KUNIT_CASE(pxamci_clock_config_test),
	KUNIT_CASE(pxamci_timeout_calculation_test),
	KUNIT_CASE(pxamci_platform_helpers_test),
	KUNIT_CASE(pxamci_status_and_progress_test),
	KUNIT_CASE(pxamci_command_completion_test),
	KUNIT_CASE(pxamci_controller_completion_test),
	KUNIT_CASE(pxamci_stale_dma_callback_test),
	KUNIT_CASE(pxamci_dma_completion_test),
	KUNIT_CASE(pxamci_read_happy_path_test),
	KUNIT_CASE(pxamci_write_with_stop_happy_path_test),
	KUNIT_CASE(pxamci_sbc_write_happy_path_test),
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
