// SPDX-License-Identifier: GPL-2.0-only
#include <kunit/test.h>
#include <linux/module.h>
#include <linux/soc/pxa/driver.h>

static void pxa_gpio_irq_type_test(struct kunit *test)
{
	KUNIT_EXPECT_TRUE(test, pxa_gpio_irq_type_valid(IRQ_TYPE_EDGE_RISING));
	KUNIT_EXPECT_TRUE(test, pxa_gpio_irq_type_valid(IRQ_TYPE_EDGE_BOTH));
	KUNIT_EXPECT_FALSE(test, pxa_gpio_irq_type_valid(IRQ_TYPE_NONE));
	KUNIT_EXPECT_FALSE(test, pxa_gpio_irq_type_valid(IRQ_TYPE_LEVEL_HIGH));
}

static void pxa_register_update_test(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, pxa_register_update(0xa5a50000, 0xff, 0x5a),
			0xa5a5005aU);
	KUNIT_EXPECT_EQ(test, pxa_register_update(0xa5a500ff, 0xff, 0x100),
			0xa5a50000U);
}

static void pxa_spi_dma_length_test(struct kunit *test)
{
	KUNIT_EXPECT_TRUE(test,
			  pxa_spi_dma_length_valid(true, 32, 8191, 8, 32));
	KUNIT_EXPECT_FALSE(test,
			   pxa_spi_dma_length_valid(true, 31, 8191, 8, 32));
	KUNIT_EXPECT_FALSE(test,
			   pxa_spi_dma_length_valid(false, 32, 8191, 8, 32));
}

static void pxa_dma_alignment_test(struct kunit *test)
{
	KUNIT_EXPECT_TRUE(test, pxa_dma_address_supported(false, 0x1000));
	KUNIT_EXPECT_FALSE(test, pxa_dma_address_supported(false, 0x1001));
	KUNIT_EXPECT_TRUE(test, pxa_dma_address_supported(true, 0x1001));
}

static void pxa_i2s_divisor_test(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, pxa_i2s_rate_divisor(44100), 0xd);
	KUNIT_EXPECT_EQ(test, pxa_i2s_rate_divisor(48000), 0xc);
	KUNIT_EXPECT_EQ(test, pxa_i2s_rate_divisor(8000), -EINVAL);
	KUNIT_EXPECT_EQ(test, pxa_i2s_rate_divisor(96000), -EINVAL);
}

static void pxa_camera_divisor_test(struct kunit *test)
{
	unsigned long actual = 0;

	KUNIT_EXPECT_EQ(test,
			pxa_camera_clock_divisor(104000000, 13000000, &actual), 3);
	KUNIT_EXPECT_EQ(test, actual, 13000000UL);
	KUNIT_EXPECT_EQ(test,
			pxa_camera_clock_divisor(0, 13000000, &actual), -EINVAL);
	KUNIT_EXPECT_EQ(test,
			pxa_camera_clock_divisor(104000000, 0, &actual), -EINVAL);
}

static void pxa_bounds_and_link_test(struct kunit *test)
{
	KUNIT_EXPECT_TRUE(test, pxa_ethernet_port_valid(2));
	KUNIT_EXPECT_FALSE(test, pxa_ethernet_port_valid(3));
	KUNIT_EXPECT_TRUE(test, pxa_udc_link_active(true, true, false));
	KUNIT_EXPECT_FALSE(test, pxa_udc_link_active(false, true, false));
	KUNIT_EXPECT_FALSE(test, pxa_udc_link_active(true, true, true));
}

static void pxa_rtc_and_frequency_test(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, pxa_rtc_enable_value(0xffff, 0x33), 0xffccU);
	KUNIT_EXPECT_TRUE(test, pxa25x_frequency_supported(99532800));
	KUNIT_EXPECT_TRUE(test, pxa25x_frequency_supported(398131200));
	KUNIT_EXPECT_FALSE(test, pxa25x_frequency_supported(100000000));
	KUNIT_EXPECT_FALSE(test, pxa25x_frequency_supported(400000000));
}

static struct kunit_case pxa_driver_test_cases[] = {
	KUNIT_CASE(pxa_gpio_irq_type_test),
	KUNIT_CASE(pxa_register_update_test),
	KUNIT_CASE(pxa_spi_dma_length_test),
	KUNIT_CASE(pxa_dma_alignment_test),
	KUNIT_CASE(pxa_i2s_divisor_test),
	KUNIT_CASE(pxa_camera_divisor_test),
	KUNIT_CASE(pxa_bounds_and_link_test),
	KUNIT_CASE(pxa_rtc_and_frequency_test),
	{}
};

static struct kunit_suite pxa_driver_test_suite = {
	.name = "pxa-driver",
	.test_cases = pxa_driver_test_cases,
};

kunit_test_suite(pxa_driver_test_suite);

MODULE_LICENSE("GPL");
