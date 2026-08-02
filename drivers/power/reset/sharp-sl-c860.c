// SPDX-License-Identifier: GPL-2.0-only
/* Sharp SL-C860 bootloader restart/power-off selection. */

#include <linux/leds.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>

struct sl_c860_sys_off {
	struct led_classdev *mail_led;
	bool powering_off;
};

static void sl_c860_set_boot_mode(struct sl_c860_sys_off *sys_off,
				  bool reboot)
{
	/* The SCOOP GPIO is MMIO-backed and its brightness callback cannot sleep. */
	led_set_brightness(sys_off->mail_led, reboot ? LED_FULL : LED_OFF);
}

static int sl_c860_restart(struct sys_off_data *data)
{
	struct sl_c860_sys_off *sys_off = data->cb_data;

	/* Power-off deliberately re-enters this chain to reach pxa_restart(). */
	if (!READ_ONCE(sys_off->powering_off))
		sl_c860_set_boot_mode(sys_off, true);
	/* The PXA machine restart callback asserts the watchdog reset. */
	return NOTIFY_DONE;
}

static int sl_c860_power_off(struct sys_off_data *data)
{
	struct sl_c860_sys_off *sys_off = data->cb_data;

	WRITE_ONCE(sys_off->powering_off, true);
	sl_c860_set_boot_mode(sys_off, false);
	/* Reset into the bootloader, which observes the now-dark green LED. */
	emergency_restart();
	return NOTIFY_DONE;
}

static int sl_c860_sys_off_probe(struct platform_device *pdev)
{
	struct sl_c860_sys_off *sys_off;
	int ret;

	sys_off = devm_kzalloc(&pdev->dev, sizeof(*sys_off), GFP_KERNEL);
	if (!sys_off)
		return -ENOMEM;

	sys_off->mail_led = devm_of_led_get(&pdev->dev, 0);
	if (IS_ERR(sys_off->mail_led))
		return dev_err_probe(&pdev->dev, PTR_ERR(sys_off->mail_led),
				     "mail LED is not ready\n");

	/* Run before ARM's priority-128 machine restart handler. */
	ret = devm_register_sys_off_handler(&pdev->dev, SYS_OFF_MODE_RESTART,
					    192, sl_c860_restart, sys_off);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "cannot register restart handler\n");

	ret = devm_register_power_off_handler(&pdev->dev, sl_c860_power_off,
					      sys_off);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "cannot register power-off handler\n");

	platform_set_drvdata(pdev, sys_off);
	return 0;
}

static const struct of_device_id sl_c860_sys_off_of_match[] = {
	{ .compatible = "sharp,sl-c860-sys-off" },
	{ }
};
MODULE_DEVICE_TABLE(of, sl_c860_sys_off_of_match);

static struct platform_driver sl_c860_sys_off_driver = {
	.probe = sl_c860_sys_off_probe,
	.driver = {
		.name = "sharp-sl-c860-sys-off",
		.of_match_table = sl_c860_sys_off_of_match,
	},
};
module_platform_driver(sl_c860_sys_off_driver);

MODULE_DESCRIPTION("Sharp SL-C860 bootloader sys-off selector");
MODULE_LICENSE("GPL");
