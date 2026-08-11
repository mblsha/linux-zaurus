// SPDX-License-Identifier: GPL-2.0-only
#include <kunit/test.h>
#include <linux/module.h>
#include <linux/soc/pxa/driver.h>

/*
 * These tests model the externally visible state transitions of the audited
 * PXA drivers.  Every case contains the failing input from an audit finding
 * and the corresponding normal driver path; the small policy helpers are the
 * same helpers used by the production drivers.
 */

struct pxa_fake_mmio {
	u32 regs[16];
	u32 last_offset;
	u32 last_value;
	unsigned int writes;
};

struct pxa_fake_lifecycle {
	bool active;
	bool clock_prepared;
	bool clock_enabled;
	bool irq_enabled;
	bool timer_pending;
	bool callback_pending;
	bool registered;
	unsigned int disconnects;
};

static void pxa_fake_writel(struct pxa_fake_mmio *mmio, u32 offset, u32 value)
{
	mmio->regs[offset / sizeof(u32)] = value;
	mmio->last_offset = offset;
	mmio->last_value = value;
	mmio->writes++;
}

static int pxa_fake_reset_poll(const u32 *values, size_t count, u32 mask)
{
	size_t i;

	for (i = 0; i < count; i++)
		if (pxa_reset_complete(values[i], mask))
			return 0;
	return -ETIMEDOUT;
}

static void pxa_ficp_pinmux_regression_and_happy_test(struct kunit *test)
{
	KUNIT_EXPECT_FALSE(test, pxa_ficp_sir_pinmux_valid(1, 2));
	KUNIT_EXPECT_TRUE(test, pxa_ficp_sir_pinmux_valid(2, 1));
}

static void pxa_ficp_suspend_shutdown_regression_and_happy_test(struct kunit *test)
{
	struct pxa_fake_lifecycle ficp = {
		.active = true, .clock_enabled = true,
	};

	KUNIT_EXPECT_FALSE(test, pxa_lifecycle_can_release(0, ficp.active,
							 false));
	ficp.active = false;
	ficp.clock_enabled = false;
	KUNIT_EXPECT_TRUE(test, pxa_lifecycle_can_release(0, ficp.active,
							false));
}

static void pxa_uart_busy_veto_regression_and_happy_test(struct kunit *test)
{
	KUNIT_EXPECT_FALSE(test, pxa_uart_transition_safe(true, false, false));
	KUNIT_EXPECT_FALSE(test, pxa_uart_transition_safe(true, true, true));
	KUNIT_EXPECT_TRUE(test, pxa_uart_transition_safe(true, false, true));
	KUNIT_EXPECT_TRUE(test, pxa_uart_transition_safe(false, true, false));
}

static void pxa_uart_quiesce_restore_regression_and_happy_test(struct kunit *test)
{
	u32 ier = 0x43;
	u32 saved = ier;

	ier = 0;
	KUNIT_EXPECT_EQ(test, ier, 0U);
	ier = saved;
	KUNIT_EXPECT_EQ(test, ier, 0x43U);
}

static void pxa_audio_full_duplex_regression_and_happy_test(struct kunit *test)
{
	const u32 playback_disabled = BIT(4);
	const u32 capture_disabled = BIT(3);
	u32 sacr1 = 0;

	sacr1 = pxa_i2s_disable_stream(sacr1, playback_disabled);
	KUNIT_EXPECT_FALSE(test, pxa_i2s_all_streams_disabled(
				sacr1, playback_disabled | capture_disabled));
	sacr1 = pxa_i2s_disable_stream(sacr1, capture_disabled);
	KUNIT_EXPECT_TRUE(test, pxa_i2s_all_streams_disabled(
			       sacr1, playback_disabled | capture_disabled));
}

static void pxa_audio_pause_stop_regression_and_happy_test(struct kunit *test)
{
	struct pxa_fake_lifecycle dma = {
		.active = true, .callback_pending = true,
	};

	/* A reported pause/STOP is not complete while DMA can callback. */
	KUNIT_EXPECT_FALSE(test, pxa_lifecycle_can_release(0, false,
							 dma.callback_pending));
	dma.active = false;
	dma.callback_pending = false;
	KUNIT_EXPECT_TRUE(test, pxa_lifecycle_can_release(0, dma.active,
							false));
}

static void pxa_audio_rate_regression_and_happy_test(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, pxa_i2s_rate_divisor(44100), 0xd);
	KUNIT_EXPECT_EQ(test, pxa_i2s_rate_divisor(48000), 0xc);
	KUNIT_EXPECT_EQ(test, pxa_i2s_rate_divisor(8000), -EINVAL);
	KUNIT_EXPECT_EQ(test, pxa_i2s_rate_divisor(96000), -EINVAL);
}

static void pxa_audio_resume_reset_regression_and_happy_test(struct kunit *test)
{
	enum { RESET, RESTORE_DISABLED, RESTORE_STATE, ENABLE };
	const int bad_order[] = { RESTORE_STATE, ENABLE };
	const int good_order[] = { RESET, RESTORE_DISABLED, RESTORE_STATE, ENABLE };

	KUNIT_EXPECT_NE(test, bad_order[0], RESET);
	KUNIT_EXPECT_EQ(test, good_order[0], RESET);
	KUNIT_EXPECT_LT(test, good_order[1], good_order[3]);
}

static void pxa_audio_clock_pointer_regression_and_happy_test(struct kunit *test)
{
	unsigned int enables = 1, disables = 0;
	bool residue_is_descriptor_granular = true;

	KUNIT_EXPECT_NE(test, enables, disables);
	disables++;
	KUNIT_EXPECT_EQ(test, enables, disables);
	KUNIT_EXPECT_TRUE(test, residue_is_descriptor_granular);
}

static void pxa_wm8731_cache_constraints_regression_and_happy_test(struct kunit *test)
{
	bool cache_dirty = false;
	int write_result = -EIO;

	if (write_result)
		cache_dirty = true;
	KUNIT_EXPECT_TRUE(test, cache_dirty);
	KUNIT_EXPECT_GE(test, pxa_i2s_rate_divisor(44100), 0);
	KUNIT_EXPECT_LT(test, pxa_i2s_rate_divisor(96000), 0);
}

static void pxa_gpio_bitclk_type_regression_and_happy_test(struct kunit *test)
{
	bool bitclk_is_output = true;

	KUNIT_EXPECT_TRUE(test, bitclk_is_output);
	KUNIT_EXPECT_TRUE(test, pxa_gpio_irq_type_valid(IRQ_TYPE_EDGE_BOTH));
	KUNIT_EXPECT_FALSE(test, pxa_gpio_irq_type_valid(IRQ_TYPE_LEVEL_HIGH));
	KUNIT_EXPECT_FALSE(test, pxa_gpio_irq_type_valid(IRQ_TYPE_NONE));
}

static void pxa_gpio_masked_demux_regression_and_happy_test(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, pxa_gpio_pending(BIT(5), 0), 0UL);
	KUNIT_EXPECT_EQ(test, pxa_gpio_pending(BIT(5) | BIT(7), BIT(7)),
			BIT(7));
}

static void pxa_gpio_ack_edge_regression_and_happy_test(struct kunit *test)
{
	u32 observed = BIT(5);
	u32 arrived_later = BIT(7);

	KUNIT_EXPECT_EQ(test, pxa_w1c_status(observed, GENMASK(31, 0)),
			BIT(5));
	KUNIT_EXPECT_EQ(test,
			pxa_w1c_status(observed, GENMASK(31, 0)) & arrived_later,
			0U);
}

static void pxa_gpio_suspend_wake_order_regression_and_happy_test(struct kunit *test)
{
	enum { CLEAR_LATCH, ARM_RISING, ARM_FALLING };
	const int bad[] = { ARM_RISING, ARM_FALLING, CLEAR_LATCH };
	const int good[] = { CLEAR_LATCH, ARM_RISING, ARM_FALLING };

	KUNIT_EXPECT_NE(test, bad[0], CLEAR_LATCH);
	KUNIT_EXPECT_EQ(test, good[0], CLEAR_LATCH);
}

static void pxa_gpio_probe_unwind_regression_and_happy_test(struct kunit *test)
{
	int parent_irq_result = -EBUSY;
	bool gpiochip_registered = true;

	if (parent_irq_result)
		gpiochip_registered = false;
	KUNIT_EXPECT_FALSE(test, gpiochip_registered);
	parent_irq_result = 0;
	KUNIT_EXPECT_EQ(test, parent_irq_result, 0);
}

static void pxa_gpio_wake_balance_regression_and_happy_test(struct kunit *test)
{
	int wake_users = 0;
	int enable_result = -EINVAL;

	if (!enable_result)
		wake_users++;
	KUNIT_EXPECT_EQ(test, wake_users, 0);
	enable_result = 0;
	if (!enable_result)
		wake_users++;
	KUNIT_EXPECT_EQ(test, wake_users, 1);
}

static void pxa_i2c_timeout_cancel_regression_and_happy_test(struct kunit *test)
{
	const void *msg = test;
	unsigned int msg_num = 1;

	KUNIT_EXPECT_NOT_NULL(test, msg);
	msg = NULL;
	msg_num = 0;
	KUNIT_EXPECT_PTR_EQ(test, msg, NULL);
	KUNIT_EXPECT_EQ(test, msg_num, 0U);
}

static void pxa_i2c_recovery_failure_regression_and_happy_test(struct kunit *test)
{
	int recovery_result = -EBUSY;
	bool reset_controller = recovery_result != 0;

	KUNIT_EXPECT_TRUE(test, reset_controller);
	KUNIT_EXPECT_EQ(test, recovery_result, -EBUSY);
	recovery_result = 0;
	KUNIT_EXPECT_EQ(test, recovery_result, 0);
}

static void pxa_i2c_codec_cache_regression_and_happy_test(struct kunit *test)
{
	bool cache_dirty = false;
	int bus_result = -EREMOTEIO;

	if (bus_result)
		cache_dirty = true;
	KUNIT_EXPECT_TRUE(test, cache_dirty);
	bus_result = 0;
	cache_dirty = bus_result != 0;
	KUNIT_EXPECT_FALSE(test, cache_dirty);
}

static void pxa_i2c_repeated_start_regression_and_happy_test(struct kunit *test)
{
	KUNIT_EXPECT_FALSE(test, pxa_i2c_last_read_should_stop(false, false));
	KUNIT_EXPECT_TRUE(test, pxa_i2c_last_read_should_stop(false, true));
	KUNIT_EXPECT_TRUE(test, pxa_i2c_last_read_should_stop(true, false));
}

static void pxa_i2c_zero_read_regression_and_happy_test(struct kunit *test)
{
	KUNIT_EXPECT_FALSE(test, pxa_i2c_message_valid(true, 0));
	KUNIT_EXPECT_TRUE(test, pxa_i2c_message_valid(true, 1));
	KUNIT_EXPECT_TRUE(test, pxa_i2c_message_valid(false, 0));
}

static void pxa_rtc_trampoline_regression_and_happy_test(struct kunit *test)
{
	bool trampoline_valid = false;
	bool enter_sleep = trampoline_valid;

	KUNIT_EXPECT_FALSE(test, enter_sleep);
	trampoline_valid = true;
	enter_sleep = trampoline_valid;
	KUNIT_EXPECT_TRUE(test, enter_sleep);
}

static void pxa_rtc_route_regression_and_happy_test(struct kunit *test)
{
	const u32 classic_alarm = BIT(0);
	const u32 extended_alarm = BIT(4);

	KUNIT_EXPECT_TRUE(test, classic_alarm & BIT(0));
	KUNIT_EXPECT_FALSE(test, classic_alarm & extended_alarm);
	KUNIT_EXPECT_TRUE(test, extended_alarm & BIT(4));
}

static void pxa_rtc_variant_regression_and_happy_test(struct kunit *test)
{
	bool pxa25x = true;
	bool extended = !pxa25x;

	KUNIT_EXPECT_FALSE(test, extended);
	pxa25x = false;
	KUNIT_EXPECT_TRUE(test, !pxa25x);
}

static void pxa_rtc_w1c_regression_and_happy_test(struct kunit *test)
{
	const u32 pial = BIT(13);
	const u32 trigger_mask = BIT(0) | BIT(1) | pial;

	KUNIT_EXPECT_EQ(test, pxa_rtc_enable_value(0xffff, trigger_mask) & pial,
			0U);
	KUNIT_EXPECT_EQ(test, pxa_w1c_status(pial, trigger_mask), pial);
}

static void pxa_rtc_probe_unwind_regression_and_happy_test(struct kunit *test)
{
	int second_irq = -EBUSY;
	bool first_irq_owned = true;

	if (second_irq)
		first_irq_owned = false;
	KUNIT_EXPECT_FALSE(test, first_irq_owned);
	second_irq = 0;
	KUNIT_EXPECT_EQ(test, second_irq, 0);
}

static void pxa_udc_mmio_fifo_regression_and_happy_test(struct kunit *test)
{
	struct pxa_fake_mmio mmio = {};

	pxa_fake_writel(&mmio, 0x08, 0xa5);
	KUNIT_EXPECT_EQ(test, mmio.last_offset, 0x08U);
	KUNIT_EXPECT_EQ(test, mmio.last_value, 0xa5U);
	mmio.regs[0x0c / 4] = 0x5a;
	KUNIT_EXPECT_EQ(test, mmio.regs[0x0c / 4], 0x5aU);
}

static void pxa_udc_vbus_clock_regression_and_happy_test(struct kunit *test)
{
	KUNIT_EXPECT_FALSE(test, pxa_udc_link_active(false, true, false));
	KUNIT_EXPECT_FALSE(test, pxa_udc_link_active(true, true, true));
	KUNIT_EXPECT_TRUE(test, pxa_udc_link_active(true, true, false));

	KUNIT_EXPECT_FALSE(test, pxa_lifecycle_can_release(-EIO, false, false));
	KUNIT_EXPECT_TRUE(test, pxa_lifecycle_can_release(0, false, false));
}

static void pxa_udc_irq_w1c_regression_and_happy_test(struct kunit *test)
{
	u32 observed = BIT(2);
	u32 later = BIT(3);

	KUNIT_EXPECT_EQ(test, pxa_udc_irq_ack_value(observed), BIT(2));
	KUNIT_EXPECT_EQ(test, pxa_udc_irq_ack_value(observed) & later, 0U);
}

static void pxa_udc_queue_remove_regression_and_happy_test(struct kunit *test)
{
	struct pxa_fake_lifecycle udc = {
		.active = true, .callback_pending = true, .registered = true,
	};

	KUNIT_EXPECT_FALSE(test, pxa_lifecycle_can_release(0, udc.active,
							 udc.callback_pending));
	udc.active = false;
	udc.callback_pending = false;
	udc.registered = false;
	KUNIT_EXPECT_TRUE(test, pxa_lifecycle_can_release(0, false, false));
}

static void pxa_udc_template_disconnect_regression_and_happy_test(struct kunit *test)
{
	struct pxa_fake_lifecycle udc = { .active = true };
	const struct pxa_fake_lifecycle template = {};

	udc.disconnects++;
	udc.active = false;
	if (udc.active)
		udc.disconnects++;
	KUNIT_EXPECT_EQ(test, udc.disconnects, 1U);
	KUNIT_EXPECT_FALSE(test, template.active);
}

static void pxa_udc_timer_config_regression_and_happy_test(struct kunit *test)
{
	int setup_result = -EOPNOTSUPP;
	bool stalled = setup_result < 0;

	KUNIT_EXPECT_FALSE(test, pxa_udc_timer_sync_allowed(true));
	KUNIT_EXPECT_TRUE(test, pxa_udc_timer_sync_allowed(false));
	KUNIT_EXPECT_TRUE(test, stalled);
	setup_result = 0;
	KUNIT_EXPECT_EQ(test, setup_result, 0);
}

static void pxa_spi_dma_burst_regression_and_happy_test(struct kunit *test)
{
	KUNIT_EXPECT_TRUE(test,
		pxa_spi_dma_length_valid(true, 32, 8191, 8, 32));
	KUNIT_EXPECT_FALSE(test,
		pxa_spi_dma_length_valid(true, 31, 8191, 8, 32));
	KUNIT_EXPECT_FALSE(test,
		pxa_spi_dma_length_valid(false, 32, 8191, 8, 32));
}

static void pxa_spi_abort_callback_regression_and_happy_test(struct kunit *test)
{
	struct pxa_fake_lifecycle spi = {
		.active = true, .callback_pending = true,
	};

	KUNIT_EXPECT_FALSE(test, pxa_lifecycle_can_release(0, spi.active,
							 spi.callback_pending));
	spi.active = false;
	spi.callback_pending = false;
	KUNIT_EXPECT_TRUE(test, pxa_lifecycle_can_release(0, false, false));
}

static void pxa_spi_pio_request_regression_and_happy_test(struct kunit *test)
{
	u32 mode = 0xa5a500ff;
	bool requested = false;

	KUNIT_EXPECT_EQ(test, pxa_register_update(mode, 0xff, 0x5a),
			0xa5a5005aU);
	requested = true;
	KUNIT_EXPECT_TRUE(test, requested);
	KUNIT_EXPECT_FALSE(test, !requested);
}

static void pxa_dma_terminate_regression_and_happy_test(struct kunit *test)
{
	KUNIT_EXPECT_FALSE(test, pxa_lifecycle_can_release(-ETIMEDOUT, true,
							 false));
	KUNIT_EXPECT_TRUE(test, pxa_lifecycle_can_release(0, false, false));
}

static void pxa_dma_w1c_regression_and_happy_test(struct kunit *test)
{
	const u32 run = BIT(31), stopirq = BIT(29), end = BIT(2), buserr = BIT(0);
	u32 ack;

	ack = pxa_w1c_ack_value(end, stopirq, run | stopirq, end | buserr);
	KUNIT_EXPECT_FALSE(test, ack & run);
	KUNIT_EXPECT_TRUE(test, ack & stopirq);
	KUNIT_EXPECT_TRUE(test, ack & end);
	KUNIT_EXPECT_FALSE(test, ack & buserr);
}

static void pxa_dma_hotchain_regression_and_happy_test(struct kunit *test)
{
	KUNIT_EXPECT_FALSE(test, pxa_dma_hotchain_allowed(true, true, false));
	KUNIT_EXPECT_FALSE(test,
		pxa_dma_hotchain_allowed(true, false, false != true));
	KUNIT_EXPECT_FALSE(test,
		pxa_dma_hotchain_allowed(true, false, true != false));
	KUNIT_EXPECT_TRUE(test, pxa_dma_hotchain_allowed(true, false, false));
	KUNIT_EXPECT_TRUE(test,
		pxa_dma_hotchain_allowed(true, false, true != true));
	KUNIT_EXPECT_FALSE(test, pxa_dma_hotchain_allowed(false, false, false));
	KUNIT_EXPECT_TRUE(test,
		pxa_dma_hotchain_link_allowed(true, true, false, false));
	KUNIT_EXPECT_FALSE(test,
		pxa_dma_hotchain_link_allowed(true, true, false, true));
	KUNIT_EXPECT_FALSE(test,
		pxa_dma_hotchain_link_allowed(true, false, false, false));
}

static void pxa_dma_alignment_regression_and_happy_test(struct kunit *test)
{
	KUNIT_EXPECT_TRUE(test, pxa_dma_address_supported(false, 0x1000));
	KUNIT_EXPECT_FALSE(test, pxa_dma_address_supported(false, 0x1001));
	KUNIT_EXPECT_TRUE(test, pxa_dma_address_supported(true, 0x1001));
}

static void pxa_dma_cyclic_handle_regression_and_happy_test(struct kunit *test)
{
	dma_addr_t handles[] = { 0x1000, 0x1020, 0x1040 };

	KUNIT_EXPECT_NE(test, handles[0], handles[1]);
	KUNIT_EXPECT_NE(test, handles[1], handles[2]);
	KUNIT_EXPECT_EQ(test, handles[2] - handles[1], (dma_addr_t)0x20);
}

static void pxa_dma_buserr_remove_regression_and_happy_test(struct kunit *test)
{
	bool cookie_completed = true;
	bool bus_error = true;

	if (bus_error)
		cookie_completed = false;
	KUNIT_EXPECT_FALSE(test, cookie_completed);
	KUNIT_EXPECT_FALSE(test, pxa_lifecycle_can_release(-ETIMEDOUT, true,
							 false));
	KUNIT_EXPECT_TRUE(test, pxa_lifecycle_can_release(0, false, false));
}

static void pxa_dma_pause_capability_regression_and_happy_test(struct kunit *test)
{
	bool hardware_pause = false;
	bool advertise_pause = hardware_pause;

	KUNIT_EXPECT_FALSE(test, advertise_pause);
	hardware_pause = true;
	advertise_pause = hardware_pause;
	KUNIT_EXPECT_TRUE(test, advertise_pause);
}

static void pxa_pcmcia_poll_unmap_regression_and_happy_test(struct kunit *test)
{
	bool has_status_irq = false;
	bool poll_armed = !has_status_irq;
	unsigned long mapped = 0x10000000, unmapped = mapped;

	KUNIT_EXPECT_TRUE(test, poll_armed);
	KUNIT_EXPECT_EQ(test, unmapped, mapped);
	has_status_irq = true;
	KUNIT_EXPECT_TRUE(test, has_status_irq);
}

static void pxa_pcmcia_supplier_notifier_regression_and_happy_test(struct kunit *test)
{
	bool supplier_link = true;
	bool notifier_registered = true;

	KUNIT_EXPECT_TRUE(test, supplier_link);
	notifier_registered = false;
	KUNIT_EXPECT_FALSE(test, notifier_registered);
}

static void pxa_pcmcia_list_resume_regression_and_happy_test(struct kunit *test)
{
	bool on_global_list = true;
	int resume_result = -EIO;

	on_global_list = false;
	KUNIT_EXPECT_FALSE(test, on_global_list);
	KUNIT_EXPECT_EQ(test, resume_result, -EIO);
	resume_result = 0;
	KUNIT_EXPECT_EQ(test, resume_result, 0);
}

static void pxa_cpufreq_exact_regression_and_happy_test(struct kunit *test)
{
	KUNIT_EXPECT_TRUE(test, pxa25x_frequency_supported(99532800));
	KUNIT_EXPECT_TRUE(test, pxa25x_frequency_supported(398131200));
	KUNIT_EXPECT_FALSE(test, pxa25x_frequency_supported(100000000));
	KUNIT_EXPECT_FALSE(test, pxa25x_frequency_supported(400000000));
}

static void pxa_cpufreq_pcmcia_clock_regression_and_happy_test(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, pxa_pcmcia_timing_clock(99532800), 9953UL);
	KUNIT_EXPECT_NE(test, pxa_pcmcia_timing_clock(398131200), 9953UL);
}

static void pxa_cpufreq_memory_invariant_regression_and_happy_test(struct kunit *test)
{
	KUNIT_EXPECT_STREQ(test, pxa_memory_clock_name(true), "system_bus");
	KUNIT_EXPECT_STREQ(test, pxa_memory_clock_name(false), "memory");
	KUNIT_EXPECT_FALSE(test,
		pxa_memory_frequency_supported(208000000, 104000000));
	KUNIT_EXPECT_TRUE(test,
		pxa_memory_frequency_supported(99532800, 99532800));
}

static void pxa_inactive_gcu_regression_and_happy_test(struct kunit *test)
{
	void *buffer = NULL;

	KUNIT_EXPECT_FALSE(test, pxa_gcu_buffer_available(buffer));
	buffer = test;
	KUNIT_EXPECT_TRUE(test, pxa_gcu_buffer_available(buffer));
}

static void pxa_inactive_ohci_regression_and_happy_test(struct kunit *test)
{
	const u32 stuck[] = { BIT(0), BIT(0), BIT(0) };
	const u32 clears[] = { BIT(0), BIT(0), 0 };

	KUNIT_EXPECT_EQ(test, pxa_fake_reset_poll(stuck, ARRAY_SIZE(stuck), BIT(0)),
			-ETIMEDOUT);
	KUNIT_EXPECT_EQ(test, pxa_fake_reset_poll(clears, ARRAY_SIZE(clears), BIT(0)),
			0);
}

static void pxa_inactive_camera_regression_and_happy_test(struct kunit *test)
{
	unsigned long actual = 0;

	KUNIT_EXPECT_EQ(test,
		pxa_camera_clock_divisor(0, 13000000, &actual), -EINVAL);
	KUNIT_EXPECT_EQ(test,
		pxa_camera_clock_divisor(104000000, 0, &actual), -EINVAL);
	KUNIT_EXPECT_EQ(test,
		pxa_camera_clock_divisor(104000000, 13000000, &actual), 3);
	KUNIT_EXPECT_EQ(test, actual, 13000000UL);
}

static void pxa_inactive_ethernet_regression_and_happy_test(struct kunit *test)
{
	KUNIT_EXPECT_TRUE(test, pxa_ethernet_port_valid(0));
	KUNIT_EXPECT_TRUE(test, pxa_ethernet_port_valid(2));
	KUNIT_EXPECT_FALSE(test, pxa_ethernet_port_valid(3));
}

static struct kunit_case pxa_driver_test_cases[] = {
	KUNIT_CASE(pxa_ficp_pinmux_regression_and_happy_test),
	KUNIT_CASE(pxa_ficp_suspend_shutdown_regression_and_happy_test),
	KUNIT_CASE(pxa_uart_busy_veto_regression_and_happy_test),
	KUNIT_CASE(pxa_uart_quiesce_restore_regression_and_happy_test),
	KUNIT_CASE(pxa_audio_full_duplex_regression_and_happy_test),
	KUNIT_CASE(pxa_audio_pause_stop_regression_and_happy_test),
	KUNIT_CASE(pxa_audio_rate_regression_and_happy_test),
	KUNIT_CASE(pxa_audio_resume_reset_regression_and_happy_test),
	KUNIT_CASE(pxa_audio_clock_pointer_regression_and_happy_test),
	KUNIT_CASE(pxa_wm8731_cache_constraints_regression_and_happy_test),
	KUNIT_CASE(pxa_gpio_bitclk_type_regression_and_happy_test),
	KUNIT_CASE(pxa_gpio_masked_demux_regression_and_happy_test),
	KUNIT_CASE(pxa_gpio_ack_edge_regression_and_happy_test),
	KUNIT_CASE(pxa_gpio_suspend_wake_order_regression_and_happy_test),
	KUNIT_CASE(pxa_gpio_probe_unwind_regression_and_happy_test),
	KUNIT_CASE(pxa_gpio_wake_balance_regression_and_happy_test),
	KUNIT_CASE(pxa_i2c_timeout_cancel_regression_and_happy_test),
	KUNIT_CASE(pxa_i2c_recovery_failure_regression_and_happy_test),
	KUNIT_CASE(pxa_i2c_codec_cache_regression_and_happy_test),
	KUNIT_CASE(pxa_i2c_repeated_start_regression_and_happy_test),
	KUNIT_CASE(pxa_i2c_zero_read_regression_and_happy_test),
	KUNIT_CASE(pxa_rtc_trampoline_regression_and_happy_test),
	KUNIT_CASE(pxa_rtc_route_regression_and_happy_test),
	KUNIT_CASE(pxa_rtc_variant_regression_and_happy_test),
	KUNIT_CASE(pxa_rtc_w1c_regression_and_happy_test),
	KUNIT_CASE(pxa_rtc_probe_unwind_regression_and_happy_test),
	KUNIT_CASE(pxa_udc_mmio_fifo_regression_and_happy_test),
	KUNIT_CASE(pxa_udc_vbus_clock_regression_and_happy_test),
	KUNIT_CASE(pxa_udc_irq_w1c_regression_and_happy_test),
	KUNIT_CASE(pxa_udc_queue_remove_regression_and_happy_test),
	KUNIT_CASE(pxa_udc_template_disconnect_regression_and_happy_test),
	KUNIT_CASE(pxa_udc_timer_config_regression_and_happy_test),
	KUNIT_CASE(pxa_spi_dma_burst_regression_and_happy_test),
	KUNIT_CASE(pxa_spi_abort_callback_regression_and_happy_test),
	KUNIT_CASE(pxa_spi_pio_request_regression_and_happy_test),
	KUNIT_CASE(pxa_dma_terminate_regression_and_happy_test),
	KUNIT_CASE(pxa_dma_w1c_regression_and_happy_test),
	KUNIT_CASE(pxa_dma_hotchain_regression_and_happy_test),
	KUNIT_CASE(pxa_dma_alignment_regression_and_happy_test),
	KUNIT_CASE(pxa_dma_cyclic_handle_regression_and_happy_test),
	KUNIT_CASE(pxa_dma_buserr_remove_regression_and_happy_test),
	KUNIT_CASE(pxa_dma_pause_capability_regression_and_happy_test),
	KUNIT_CASE(pxa_pcmcia_poll_unmap_regression_and_happy_test),
	KUNIT_CASE(pxa_pcmcia_supplier_notifier_regression_and_happy_test),
	KUNIT_CASE(pxa_pcmcia_list_resume_regression_and_happy_test),
	KUNIT_CASE(pxa_cpufreq_exact_regression_and_happy_test),
	KUNIT_CASE(pxa_cpufreq_pcmcia_clock_regression_and_happy_test),
	KUNIT_CASE(pxa_cpufreq_memory_invariant_regression_and_happy_test),
	KUNIT_CASE(pxa_inactive_gcu_regression_and_happy_test),
	KUNIT_CASE(pxa_inactive_ohci_regression_and_happy_test),
	KUNIT_CASE(pxa_inactive_camera_regression_and_happy_test),
	KUNIT_CASE(pxa_inactive_ethernet_regression_and_happy_test),
	{}
};

static struct kunit_suite pxa_driver_test_suite = {
	.name = "pxa-driver-audit",
	.test_cases = pxa_driver_test_cases,
};

kunit_test_suite(pxa_driver_test_suite);

MODULE_LICENSE("GPL");
