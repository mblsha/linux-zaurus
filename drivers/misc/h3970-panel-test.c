// SPDX-License-Identifier: GPL-2.0-only
/* Bounded H3970 button and speaker fixture for the RAM-only demo. */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>

#define KPIO_DIR	0x00
#define KPIO_DATA	0x14
#define KPIO_ALT	0x3c
#define KEY_LINES	GENMASK(11, 0)
#define DPAD_PATTERN	GENMASK(5, 1)

#define BANK_DIR	0x04
#define BANK_OUT	0x08
#define AUDIO_RESET	BIT(10)
#define SPEAKER_ON	BIT(12)
#define AUDIO_POWER	BIT(14)
#define AUDIO_GPIO	(AUDIO_RESET | SPEAKER_ON | AUDIO_POWER)

#define SACR0	0x00
#define SACR1	0x04
#define SASR0	0x0c
#define SAIMR	0x14
#define SAICR	0x18
#define SADIV	0x60
#define SADR	0x80
#define I2S_RESET	BIT(3)
#define I2S_MASTER	BIT(2)
#define I2S_ENABLE	BIT(0)
#define I2S_RX_OFF	BIT(3)
#define I2S_TX_SPACE	BIT(0)

struct h3970_demo {
	struct miscdevice speaker;
	struct mutex audio_lock;
	struct gpio_desc *power_button;
	struct clk *mclk;
	struct clk *i2s_clk;
	struct i2c_client *codec;
	void __iomem *keys, *audio, *i2s;
	u32 pending;
	u32 saved_dir, saved_alt;
};

static const struct {
	u16 key, mask, value;
} buttons[] = {
	{ KEY_RECORD, BIT(0), BIT(0) },
	{ KEY_CALENDAR, BIT(6), BIT(6) },
	{ KEY_ADDRESSBOOK, BIT(7), BIT(7) },
	{ KEY_MAIL, BIT(8), BIT(8) },
	{ KEY_HOMEPAGE, BIT(9), BIT(9) },
	{ KEY_UP, DPAD_PATTERN, BIT(1) | BIT(4) | BIT(5) },
	{ KEY_RIGHT, BIT(11), BIT(11) },
	{ KEY_LEFT, BIT(10), BIT(10) },
	{ KEY_DOWN, DPAD_PATTERN, BIT(2) | BIT(3) | BIT(5) },
	{ KEY_ENTER, DPAD_PATTERN, BIT(5) },
};

static void poll_buttons(struct input_dev *input)
{
	struct h3970_demo *demo = input_get_drvdata(input);
	u32 keys = ~readl(demo->keys + KPIO_DATA) & KEY_LINES;
	int power = gpiod_get_value_cansleep(demo->power_button);
	unsigned int i;

	if (power > 0)
		keys |= BIT(12);
	/* Two samples at 10 ms suppress transitions through other D-pad codes. */
	if (keys != demo->pending) {
		demo->pending = keys;
		return;
	}

	for (i = 0; i < ARRAY_SIZE(buttons); i++)
		input_report_key(input, buttons[i].key,
				 (keys & buttons[i].mask) == buttons[i].value);
	input_report_key(input, KEY_POWER, !!(keys & BIT(12)));
	input_sync(input);
}

static void restore_keys(void *ptr)
{
	struct h3970_demo *demo = ptr;

	writel(demo->saved_alt, demo->keys + KPIO_ALT);
	writel(demo->saved_dir, demo->keys + KPIO_DIR);
}

static void audio_gpio(struct h3970_demo *demo, u16 value)
{
	u16 state = readw(demo->audio + BANK_OUT);

	writew((state & ~AUDIO_GPIO) | value, demo->audio + BANK_OUT);
}

static int codec_write(struct i2c_client *codec, u8 address, u16 value)
{
	u8 bytes[] = { address, value >> 8, value };
	int ret = i2c_master_send(codec, bytes, sizeof(bytes));

	return ret == sizeof(bytes) ? 0 : (ret < 0 ? ret : -EIO);
}

static int speaker_tone(struct h3970_demo *demo)
{
	static const struct { u8 address; u16 value; } registers[] = {
		/* 16 kHz I2S, 256fs, DAC WSPLL, interpolator and DAC enabled. */
		{ 0x00, 0x0311 }, { 0x01, 0x0000 },
		/* PLL, DAC and bias. The headphone amp remains powered down. */
		{ 0x02, 0x8500 }, { 0x03, 0x0000 },
		{ 0x10, 0x1818 }, { 0x11, 0x0000 }, { 0x12, 0x0000 },
		{ 0x13, 0x0000 }, { 0x14, 0x0000 },
	};
	u16 saved_dir, saved_out;
	u32 status;
	unsigned int i, frame;
	int ret;

	ret = clk_prepare_enable(demo->mclk);
	if (ret)
		return ret;
	ret = clk_prepare_enable(demo->i2s_clk);
	if (ret)
		goto disable_mclk;

	saved_dir = readw(demo->audio + BANK_DIR) & AUDIO_GPIO;
	saved_out = readw(demo->audio + BANK_OUT) & AUDIO_GPIO;
	audio_gpio(demo, AUDIO_RESET);
	writew(readw(demo->audio + BANK_DIR) | AUDIO_GPIO, demo->audio + BANK_DIR);
	writel(I2S_RESET, demo->i2s + SACR0);
	writel(0, demo->i2s + SAIMR);
	writel(I2S_RX_OFF, demo->i2s + SACR1);
	writel(0x24, demo->i2s + SADIV); /* 16 kHz at 64 bit clocks per frame. */
	writel((8 << 12) | (8 << 8) | I2S_MASTER | I2S_ENABLE, demo->i2s + SACR0);

	audio_gpio(demo, AUDIO_RESET | AUDIO_POWER);
	usleep_range(1000, 2000);
	audio_gpio(demo, AUDIO_POWER);
	usleep_range(1000, 2000);

	for (i = 0; i < ARRAY_SIZE(registers); i++) {
		ret = codec_write(demo->codec, registers[i].address, registers[i].value);
		if (ret) {
			dev_err(&demo->codec->dev, "codec register %#x: %d\n",
				registers[i].address, ret);
			goto stop;
		}
	}
	msleep(15);
	writel(BIT(5), demo->i2s + SAICR);
	audio_gpio(demo, AUDIO_POWER | SPEAKER_ON);
	for (frame = 0; frame < 16000; frame++) {
		unsigned int period = frame < 8000 ? 40 : 32;
		unsigned int phase = frame % period;
		s16 value = 0;
		u32 sample;

		if (frame >= 160 && frame < 15680 &&
		    (frame < 7680 || frame >= 8000)) {
			if (phase >= period / 2)
				phase = period - phase;
			value = (int)phase * 7200 / (period / 2) - 3600;
		}
		sample = (u16)value | (u32)(u16)value << 16;
		ret = readl_poll_timeout_atomic(demo->i2s + SASR0, status,
						status & I2S_TX_SPACE, 2, 2000);
		if (ret)
			goto stop;
		writel(sample, demo->i2s + SADR);
		if (!(frame & 31))
			cond_resched();
	}
	usleep_range(1500, 2500);
	dev_info(demo->speaker.parent, "UDA1380 programmed and %u stereo frames sent\n", frame);

stop:
	audio_gpio(demo, AUDIO_POWER);
	writel(I2S_RESET, demo->i2s + SACR0);
	audio_gpio(demo, saved_out);
	writew((readw(demo->audio + BANK_DIR) & ~AUDIO_GPIO) | saved_dir,
	       demo->audio + BANK_DIR);
	clk_disable_unprepare(demo->i2s_clk);
disable_mclk:
	clk_disable_unprepare(demo->mclk);
	return ret;
}

static ssize_t play_test(struct file *file, const char __user *buffer,
			 size_t count, loff_t *offset)
{
	struct h3970_demo *demo = container_of(file->private_data,
					      struct h3970_demo, speaker);
	char request;
	int ret;

	if (count != 1)
		return -EINVAL;
	if (get_user(request, buffer))
		return -EFAULT;
	if (request != '1')
		return -EINVAL;
	if (!mutex_trylock(&demo->audio_lock))
		return -EBUSY;
	ret = speaker_tone(demo);
	mutex_unlock(&demo->audio_lock);
	return ret ? ret : count;
}

static const struct file_operations speaker_ops = {
	.owner = THIS_MODULE,
	.write = play_test,
};

static void remove_speaker(void *ptr)
{
	struct h3970_demo *demo = ptr;

	misc_deregister(&demo->speaker);
}

static int probe_demo(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct h3970_demo *demo;
	struct device_node *node;
	struct i2c_adapter *bus;
	struct input_dev *input;
	unsigned int i;
	int ret;

	demo = devm_kzalloc(dev, sizeof(*demo), GFP_KERNEL);
	if (!demo)
		return -ENOMEM;
	demo->keys = devm_platform_ioremap_resource_byname(pdev, "keys");
	if (IS_ERR(demo->keys))
		return PTR_ERR(demo->keys);
	demo->power_button = devm_gpiod_get(dev, "power-button", GPIOD_IN);
	if (IS_ERR(demo->power_button))
		return dev_err_probe(dev, PTR_ERR(demo->power_button), "power button\n");

	demo->saved_dir = readl(demo->keys + KPIO_DIR);
	demo->saved_alt = readl(demo->keys + KPIO_ALT);
	writel(demo->saved_dir | KEY_LINES, demo->keys + KPIO_DIR);
	writel(DPAD_PATTERN, demo->keys + KPIO_ALT);
	ret = devm_add_action_or_reset(dev, restore_keys, demo);
	if (ret)
		return ret;

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;
	input->name = "H3970 buttons";
	input->id.bustype = BUS_HOST;
	input_set_drvdata(input, demo);
	for (i = 0; i < ARRAY_SIZE(buttons); i++)
		input_set_capability(input, EV_KEY, buttons[i].key);
	input_set_capability(input, EV_KEY, KEY_POWER);
	ret = input_setup_polling(input, poll_buttons);
	if (ret)
		return ret;
	input_set_poll_interval(input, 10);

	demo->audio = devm_platform_ioremap_resource_byname(pdev, "audio-gpio");
	if (IS_ERR(demo->audio))
		return PTR_ERR(demo->audio);
	demo->i2s = devm_platform_ioremap_resource_byname(pdev, "i2s");
	if (IS_ERR(demo->i2s))
		return PTR_ERR(demo->i2s);
	demo->i2s_clk = devm_clk_get(dev, "i2s");
	if (IS_ERR(demo->i2s_clk))
		return dev_err_probe(dev, PTR_ERR(demo->i2s_clk), "I2S clock\n");
	demo->mclk = devm_clk_get(dev, "codec");
	if (IS_ERR(demo->mclk))
		return dev_err_probe(dev, PTR_ERR(demo->mclk), "codec clock\n");

	node = of_parse_phandle(dev->of_node, "codec-bus", 0);
	if (!node)
		return -EINVAL;
	bus = of_get_i2c_adapter_by_node(node);
	of_node_put(node);
	if (!bus)
		return dev_err_probe(dev, -EPROBE_DEFER, "codec bus\n");
	demo->codec = devm_i2c_new_dummy_device(dev, bus, 0x18);
	i2c_put_adapter(bus);
	if (IS_ERR(demo->codec))
		return dev_err_probe(dev, PTR_ERR(demo->codec), "UDA1380 address\n");

	mutex_init(&demo->audio_lock);
	demo->speaker.minor = MISC_DYNAMIC_MINOR;
	demo->speaker.name = "h3970-speaker-test";
	demo->speaker.fops = &speaker_ops;
	demo->speaker.parent = dev;
	ret = misc_register(&demo->speaker);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, remove_speaker, demo);
	if (ret)
		return ret;
	return input_register_device(input);
}

static const struct of_device_id demo_match[] = {
	{ .compatible = "compaq,h3970-panel-test" },
	{}
};

static struct platform_driver demo_driver = {
	.probe = probe_demo,
	.driver = {
		.name = "h3970-panel-test",
		.of_match_table = demo_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(demo_driver);
