// SPDX-License-Identifier: GPL-2.0-only
/*
 * Battery and power-management glue for the Sharp Zaurus SL-C860.
 *
 * The hardware policy remains in the shared SharpSL PM core. This driver
 * supplies the SL-C860 thresholds and acquires every board GPIO from DT.
 */

#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/spi/corgi_lcd.h>

#include "pxa2xx-regs.h"
#include "sharpsl_pm.h"

#define SLC860_CHARGE_ON_VOLT		0x99
#define SLC860_CHARGE_ON_TEMP		0xe0
#define SLC860_CHARGE_ON_ACIN_HIGH	0x9b
#define SLC860_CHARGE_ON_ACIN_LOW	0x34
#define SLC860_FATAL_ACIN_VOLT		182
#define SLC860_FATAL_NOACIN_VOLT		170

struct slc860_pm {
	struct platform_device *child;
	struct gpio_desc *temp_enable;
	struct gpio_desc *charge_enable;
	struct gpio_desc *charge_unknown;
	struct gpio_desc *discharge_enable;
	struct gpio_desc *ac_present;
	struct gpio_desc *battery_full;
	struct gpio_desc *battery_cover;
	struct gpio_desc *key_wakeup;
	struct gpio_desc *power_wakeup;
	bool last_ac_present;
};

static struct slc860_pm *slc860_pm_state(void)
{
	return dev_get_drvdata(sharpsl_pm.dev->parent);
}

static void slc860_pm_init(void)
{
}

static void slc860_measure_temp(int on)
{
	gpiod_set_value(slc860_pm_state()->temp_enable, on);
}

static void slc860_charge(int on)
{
	struct slc860_pm *pm = slc860_pm_state();

	/* Charging stays electrically disabled until separately qualified. */
	gpiod_set_value(pm->charge_enable, 0);
	gpiod_set_value(pm->charge_unknown, 0);
}

static void slc860_discharge(int on)
{
	gpiod_set_value(slc860_pm_state()->discharge_enable, on);
}

static void slc860_presuspend(void)
{
	struct slc860_pm *pm = slc860_pm_state();

	pm->last_ac_present = gpiod_get_value(pm->ac_present);
}

static void slc860_postsuspend(void)
{
}

static unsigned long slc860_read_devdata(int type)
{
	struct slc860_pm *pm = slc860_pm_state();

	switch (type) {
	case SHARPSL_STATUS_ACIN:
		return gpiod_get_value(pm->ac_present);
	case SHARPSL_STATUS_LOCK:
		return gpiod_get_value(pm->battery_cover);
	case SHARPSL_STATUS_CHRGFULL:
		return gpiod_get_value(pm->battery_full);
	case SHARPSL_STATUS_FATAL:
		return 1;
	case SHARPSL_ACIN_VOLT:
		return sharpsl_pm_pxa_read_max1111(MAX1111_ACIN_VOLT);
	case SHARPSL_BATT_TEMP:
		return sharpsl_pm_pxa_read_max1111(MAX1111_BATT_TEMP);
	case SHARPSL_BATT_VOLT:
	default:
		return sharpsl_pm_pxa_read_max1111(MAX1111_BATT_VOLT);
	}
}

static bool slc860_charger_wakeup(void)
{
	struct slc860_pm *pm = slc860_pm_state();

	return gpiod_get_value(pm->ac_present) ||
		gpiod_get_value(pm->key_wakeup) ||
		gpiod_get_value(pm->power_wakeup);
}

static int slc860_should_wakeup(unsigned int resume_on_alarm)
{
	struct slc860_pm *pm = slc860_pm_state();
	bool ac_present = gpiod_get_value(pm->ac_present);
	int is_resume = 0;

	if (pm->last_ac_present != ac_present)
		pm->last_ac_present = ac_present;
	if (gpiod_get_value(pm->key_wakeup))
		is_resume = 1;
	if (gpiod_get_value(pm->power_wakeup))
		is_resume = 1;
	if (resume_on_alarm && (PEDR & PWER_RTC))
		is_resume |= PWER_RTC;

	return is_resume;
}

static const struct sharpsl_charger_machinfo slc860_pm_machinfo = {
	.init			= slc860_pm_init,
	.discharge		= slc860_discharge,
	.charge		= slc860_charge,
	.measure_temp		= slc860_measure_temp,
	.presuspend		= slc860_presuspend,
	.postsuspend		= slc860_postsuspend,
	.read_devdata		= slc860_read_devdata,
	.charger_wakeup		= slc860_charger_wakeup,
	.should_wakeup		= slc860_should_wakeup,
#if IS_ENABLED(CONFIG_LCD_CORGI)
	.backlight_limit		= corgi_lcd_limit_intensity,
#endif
	.charge_on_volt		= SLC860_CHARGE_ON_VOLT,
	.charge_on_temp		= SLC860_CHARGE_ON_TEMP,
	.charge_acin_high	= SLC860_CHARGE_ON_ACIN_HIGH,
	.charge_acin_low	= SLC860_CHARGE_ON_ACIN_LOW,
	.fatal_acin_volt	= SLC860_FATAL_ACIN_VOLT,
	.fatal_noacin_volt	= SLC860_FATAL_NOACIN_VOLT,
	.bat_levels		= 40,
	.bat_levels_noac	= sharpsl_battery_levels_noac,
	.bat_levels_acin	= sharpsl_battery_levels_acin,
	.status_high_acin	= 188,
	.status_low_acin	= 178,
	.status_high_noac	= 185,
	.status_low_noac	= 175,
};

static int slc860_get_gpio(struct device *dev, const char *name,
			   enum gpiod_flags flags, struct gpio_desc **result)
{
	struct gpio_desc *desc;

	desc = devm_gpiod_get(dev, name, flags);
	if (IS_ERR(desc))
		return dev_err_probe(dev, PTR_ERR(desc),
				     "failed to get %s GPIO\n", name);
	*result = desc;
	return 0;
}

static int slc860_pm_probe(struct platform_device *pdev)
{
	struct sharpsl_charger_machinfo machinfo = slc860_pm_machinfo;
	struct device *dev = &pdev->dev;
	struct slc860_pm *pm;
	int ret;

	if (!device_property_read_bool(dev, "sharp,charging-disabled"))
		return dev_err_probe(dev, -EPERM,
			"refusing unqualified charging-enabled DT policy\n");

	pm = devm_kzalloc(dev, sizeof(*pm), GFP_KERNEL);
	if (!pm)
		return -ENOMEM;
	platform_set_drvdata(pdev, pm);

	ret = slc860_get_gpio(dev, "temperature-enable", GPIOD_OUT_LOW, &pm->temp_enable);
	if (ret)
		return ret;
	ret = slc860_get_gpio(dev, "charge-enable", GPIOD_OUT_LOW, &pm->charge_enable);
	if (ret)
		return ret;
	ret = slc860_get_gpio(dev, "charge-unknown", GPIOD_OUT_LOW, &pm->charge_unknown);
	if (ret)
		return ret;
	ret = slc860_get_gpio(dev, "discharge-enable", GPIOD_OUT_LOW, &pm->discharge_enable);
	if (ret)
		return ret;
	ret = slc860_get_gpio(dev, "ac-present", GPIOD_IN, &pm->ac_present);
	if (ret)
		return ret;
	ret = slc860_get_gpio(dev, "battery-full", GPIOD_IN, &pm->battery_full);
	if (ret)
		return ret;
	ret = slc860_get_gpio(dev, "battery-cover", GPIOD_IN, &pm->battery_cover);
	if (ret)
		return ret;
	ret = slc860_get_gpio(dev, "key-wakeup", GPIOD_IN, &pm->key_wakeup);
	if (ret)
		return ret;
	ret = slc860_get_gpio(dev, "power-wakeup", GPIOD_IN, &pm->power_wakeup);
	if (ret)
		return ret;

	machinfo.acin_desc = pm->ac_present;
	machinfo.batfull_desc = pm->battery_full;
	machinfo.batlock_desc = pm->battery_cover;
	machinfo.batfull_irq = 1;
	machinfo.wakeup_irq = gpiod_to_irq(pm->power_wakeup);
	if (machinfo.wakeup_irq < 0)
		return dev_err_probe(dev, machinfo.wakeup_irq,
				     "failed to map power-wakeup IRQ\n");
	machinfo.key_wakeup_irq = gpiod_to_irq(pm->key_wakeup);
	if (machinfo.key_wakeup_irq < 0)
		return dev_err_probe(dev, machinfo.key_wakeup_irq,
				     "failed to map key-wakeup IRQ\n");
	machinfo.charging_disabled = true;

	pm->child = platform_device_alloc("sharpsl-pm", PLATFORM_DEVID_NONE);
	if (!pm->child)
		return -ENOMEM;
	pm->child->dev.parent = dev;
	ret = platform_device_add_data(pm->child, &machinfo, sizeof(machinfo));
	if (ret)
		goto put_child;
	ret = platform_device_add(pm->child);
	if (ret)
		goto put_child;

	dev_info(dev, "PM/battery GPIO ownership transferred to Device Tree\n");
	return 0;

put_child:
	platform_device_put(pm->child);
	pm->child = NULL;
	return ret;
}

static void slc860_pm_remove(struct platform_device *pdev)
{
	struct slc860_pm *pm = platform_get_drvdata(pdev);

	platform_device_unregister(pm->child);
}

static const struct of_device_id slc860_pm_of_match[] = {
	{ .compatible = "sharp,sl-c860-power" },
	{ }
};
MODULE_DEVICE_TABLE(of, slc860_pm_of_match);

static struct platform_driver slc860_pm_driver = {
	.probe = slc860_pm_probe,
	.remove = slc860_pm_remove,
	.driver = {
		.name = "sharp-sl-c860-power",
		.of_match_table = slc860_pm_of_match,
	},
};
module_platform_driver(slc860_pm_driver);

MODULE_DESCRIPTION("Sharp SL-C860 Device Tree PM glue");
MODULE_LICENSE("GPL");
