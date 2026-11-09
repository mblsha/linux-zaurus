// SPDX-License-Identifier: GPL-2.0-only
/*
 * H3970 ASIC2 touchscreen bring-up. GPIO drive states and ADC mux assignments
 * follow the Familiar ASIC2 board sources by Compaq, HP and Phil Blundell.
 */
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define GPIO_DIR	0x0000
#define GPIO_DATA	0x0014
#define DRIVE_MASK	GENMASK(3, 0)
#define PEN_UP		BIT(5)
#define DRIVE_PEN	0x0b
#define DRIVE_X		0x0c
#define DRIVE_Y		0x03

#define CLOCK_ENABLE	0x1000
#define TOUCH_CLOCKS	(BIT(14) | BIT(4))	/* EX1 crystal and ADC clock */
#define ADC_MUX		0x1200
#define ADC_CONTROL	0x1204
#define ADC_DATA	0x1208
#define ADC_MUX_CLOCK	BIT(3)
#define ADC_START	BIT(6)
#define ADC_ENABLE	BIT(7)
#define ADC_PRESCALE	4

struct h3970_touch {
	void __iomem *base;
	struct device *dev;
	struct mutex lock;
	u32 saved_clocks;
	bool stopped;
	bool reported_timing;
};

static void update_bits(struct h3970_touch *touch, unsigned int reg, u32 mask, u32 value)
{
	u32 old = readl(touch->base + reg);

	writel((old & ~mask) | value, touch->base + reg);
}

static int measure_axis(struct h3970_touch *touch, unsigned int drive, unsigned int mux)
{
	void __iomem *base = touch->base;
	u32 control;
	int i;

	update_bits(touch, GPIO_DATA, DRIVE_MASK, drive);
	writel(mux | ADC_MUX_CLOCK, base + ADC_MUX);
	udelay(200);
	/* Familiar takes four conversions on each drive setting and uses the last. */
	for (i = 0; i < 4; i++) {
		writel(ADC_ENABLE | ADC_PRESCALE, base + ADC_CONTROL);
		writel(ADC_ENABLE | ADC_PRESCALE | ADC_START, base + ADC_CONTROL);
		if (readl_poll_timeout_atomic(base + ADC_CONTROL, control,
					      !(control & ADC_START), 5, 2000))
			return -ETIMEDOUT;
	}

	return readl(base + ADC_DATA) & 0x3ff;
}

static void h3970_touch_poll(struct input_dev *input)
{
	struct h3970_touch *touch = input_get_drvdata(input);
	ktime_t start;
	int x, y;

	mutex_lock(&touch->lock);
	if (touch->stopped)
		goto out;
	if (readl(touch->base + GPIO_DATA) & PEN_UP)
		goto release;

	start = ktime_get();
	x = measure_axis(touch, DRIVE_X, 3);
	y = x >= 0 ? measure_axis(touch, DRIVE_Y, 4) : x;
	writel(0, touch->base + ADC_CONTROL);
	writel(0, touch->base + ADC_MUX);
	update_bits(touch, GPIO_DATA, DRIVE_MASK, DRIVE_PEN);
	if (x < 0 || y < 0) {
		dev_warn_ratelimited(touch->dev, "ASIC2 ADC conversion timed out\n");
		goto release;
	}
	udelay(250);
	if (readl(touch->base + GPIO_DATA) & PEN_UP)
		goto release;

	if (!touch->reported_timing) {
		dev_info(touch->dev, "ASIC2 touchscreen sample in %lld us\n",
			 (long long)ktime_us_delta(ktime_get(), start));
		touch->reported_timing = true;
	}
	input_report_abs(input, ABS_X, x);
	input_report_abs(input, ABS_Y, y);
	input_report_key(input, BTN_TOUCH, 1);
	input_sync(input);
	goto out;

release:
	input_report_key(input, BTN_TOUCH, 0);
	input_sync(input);
out:
	mutex_unlock(&touch->lock);
}

static void h3970_touch_stop(void *data)
{
	struct h3970_touch *touch = data;

	mutex_lock(&touch->lock);
	if (!touch->stopped) {
		touch->stopped = true;
		writel(0, touch->base + ADC_CONTROL);
		writel(0, touch->base + ADC_MUX);
		update_bits(touch, GPIO_DATA, DRIVE_MASK, DRIVE_PEN);
		update_bits(touch, CLOCK_ENABLE, TOUCH_CLOCKS, touch->saved_clocks);
	}
	mutex_unlock(&touch->lock);
}

static int h3970_touch_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct h3970_touch *touch;
	struct input_dev *input;
	int ret;

	touch = devm_kzalloc(dev, sizeof(*touch), GFP_KERNEL);
	if (!touch)
		return -ENOMEM;
	touch->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(touch->base))
		return PTR_ERR(touch->base);
	touch->dev = dev;
	mutex_init(&touch->lock);
	touch->saved_clocks = readl(touch->base + CLOCK_ENABLE) & TOUCH_CLOCKS;
	update_bits(touch, CLOCK_ENABLE, TOUCH_CLOCKS, TOUCH_CLOCKS);
	update_bits(touch, GPIO_DATA, DRIVE_MASK, DRIVE_PEN);
	update_bits(touch, GPIO_DIR, DRIVE_MASK | PEN_UP, PEN_UP);
	usleep_range(1000, 2000);

	ret = devm_add_action_or_reset(dev, h3970_touch_stop, touch);
	if (ret)
		return ret;
	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;
	input->name = "H3970 ASIC2 Touchscreen";
	input->id.bustype = BUS_HOST;
	__set_bit(INPUT_PROP_DIRECT, input->propbit);
	input_set_drvdata(input, touch);
	input_set_capability(input, EV_KEY, BTN_TOUCH);
	input_set_abs_params(input, ABS_X, 0, 1023, 2, 0);
	input_set_abs_params(input, ABS_Y, 0, 1023, 2, 0);
	ret = input_setup_polling(input, h3970_touch_poll);
	if (ret)
		return ret;
	input_set_poll_interval(input, 8);
	platform_set_drvdata(pdev, touch);
	return input_register_device(input);
}

static void h3970_touch_shutdown(struct platform_device *pdev)
{
	h3970_touch_stop(platform_get_drvdata(pdev));
}

static const struct of_device_id h3970_touch_of_match[] = {
	{ .compatible = "compaq,h3970-asic2-touchscreen" },
	{ }
};
MODULE_DEVICE_TABLE(of, h3970_touch_of_match);

static struct platform_driver h3970_touch_driver = {
	.probe = h3970_touch_probe,
	.shutdown = h3970_touch_shutdown,
	.driver = {
		.name = "h3970-asic2-touchscreen",
		.of_match_table = h3970_touch_of_match,
	},
};
module_platform_driver(h3970_touch_driver);
MODULE_LICENSE("GPL");
