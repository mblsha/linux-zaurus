// SPDX-License-Identifier: GPL-2.0-only

#include <kunit/test.h>
#include <linux/errno.h>

#include "slot-gpio.h"

static void mmc_gpio_debounce_policy_test(struct kunit *test)
{
	/* Zero means unspecified and must retain the core's 200 ms default. */
	KUNIT_EXPECT_EQ(test, 200U,
			mmc_gpio_debounce_delay_ms(200, 0, -EOPNOTSUPP));

	/* Hardware debounce removes the threaded-IRQ fallback delay. */
	KUNIT_EXPECT_EQ(test, 0U,
			mmc_gpio_debounce_delay_ms(200, 250000, 0));

	/* Unsupported hardware retains the complete requested delay. */
	KUNIT_EXPECT_EQ(test, 250U,
			mmc_gpio_debounce_delay_ms(200, 250000, -EOPNOTSUPP));
	KUNIT_EXPECT_EQ(test, 251U,
			mmc_gpio_debounce_delay_ms(200, 250001, -EOPNOTSUPP));
}

static struct kunit_case mmc_slot_gpio_test_cases[] = {
	KUNIT_CASE(mmc_gpio_debounce_policy_test),
	{ }
};

static struct kunit_suite mmc_slot_gpio_test_suite = {
	.name = "mmc-slot-gpio",
	.test_cases = mmc_slot_gpio_test_cases,
};

kunit_test_suite(mmc_slot_gpio_test_suite);

MODULE_DESCRIPTION("KUnit tests for MMC GPIO card-detect policy");
MODULE_LICENSE("GPL");
