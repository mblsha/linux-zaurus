// SPDX-License-Identifier: GPL-2.0-only
/*
 * Battery and power-management glue for the Sharp Zaurus SL-C860.
 *
 * The hardware policy remains in the shared SharpSL PM core. This driver
 * supplies the SL-C860 thresholds and acquires every board GPIO from DT.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/spi/corgi_lcd.h>

#include "pxa2xx-regs.h"
#include "regs-rtc.h"
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

#define SLC860_GPIO_KEY_INT		0
#define SLC860_GPIO_AC_IN		1
#define SLC860_GPIO_WAKEUP		3
#define SLC860_GPIO_AK_INT		4
#define SLC860_GPIO_MAIN_BAT_LOW	11

#ifdef CONFIG_SHARP_SL_C860_DEEP_RESUME
#define SLC860_GPLR0	__REG(0x40e00000)
#define SLC860_GPLR1	__REG(0x40e00004)
#define SLC860_GPLR2	__REG(0x40e00008)
#define SLC860_GPDR0	__REG(0x40e0000c)
#define SLC860_GPDR1	__REG(0x40e00010)
#define SLC860_GPDR2	__REG(0x40e00014)
#define SLC860_GPSR2	__REG(0x40e00020)
#define SLC860_GPCR2	__REG(0x40e0002c)
#define SLC860_GEDR1	__REG(0x40e0004c)
#define SLC860_GEDR2	__REG(0x40e00050)

#define SLC860_ALL_STROBE_BITS		0x00003ffc
#define SLC860_HIGH_SENSE_BITS		0xfc000000
#define SLC860_HIGH_SENSE_RSHIFT	26
#define SLC860_LOW_SENSE_BITS		0x00000003
#define SLC860_LOW_SENSE_LSHIFT		6
#define SLC860_STROBE_BIT(col)		GPIO_bit(66 + (col))

#define SLC860_STOCK_WAKE_RISING				\
	(GPIO_bit(SLC860_GPIO_AC_IN) |			\
	 GPIO_bit(SLC860_GPIO_AK_INT) |			\
	 GPIO_bit(SLC860_GPIO_MAIN_BAT_LOW))
#define SLC860_STOCK_WAKE_FALLING			\
	(GPIO_bit(SLC860_GPIO_KEY_INT) |			\
	 GPIO_bit(SLC860_GPIO_WAKEUP) |			\
	 GPIO_bit(SLC860_GPIO_AC_IN) |			\
	 GPIO_bit(SLC860_GPIO_MAIN_BAT_LOW))
#define SLC860_STOCK_WAKE_MASK				\
	(SLC860_STOCK_WAKE_RISING | SLC860_STOCK_WAKE_FALLING | PWER_RTC)
#define SLC860_STOCK_WAKE_BOTH				\
	(SLC860_STOCK_WAKE_RISING & SLC860_STOCK_WAKE_FALLING)

#define SLC860_STOCK_KEY_ROWS		8
#define SLC860_STOCK_KEY_COLS		12
#define SLC860_STOCK_KEY_CHATTER_US	100
#define SLC860_STOCK_KEY_SETTLE_US	10
#define SLC860_STOCK_KEY_DEBOUNCE_COUNT	2
#define SLC860_STOCK_KEY_HOLD_COUNT	120

static unsigned long slc860_saved_pcfr;

static bool slc860_deep_resume_active(void)
{
	return of_machine_is_compatible("sharp,sl-c860");
}

static void slc860_stock_keyboard_all_hiz(void)
{
	SLC860_GPCR2 = SLC860_ALL_STROBE_BITS;
	SLC860_GPDR2 &= ~SLC860_ALL_STROBE_BITS;
}

static void slc860_stock_keyboard_activate_col(unsigned int col)
{
	u32 bit = SLC860_STROBE_BIT(col);

	SLC860_GPSR2 = bit;
	SLC860_GPDR2 = (SLC860_GPDR2 & ~SLC860_ALL_STROBE_BITS) | bit;
}

static void slc860_stock_keyboard_reset_col(unsigned int col)
{
	u32 bit = SLC860_STROBE_BIT(col);

	SLC860_GPCR2 = bit;
	SLC860_GPDR2 = (SLC860_GPDR2 & ~SLC860_ALL_STROBE_BITS) | bit;
}

static void slc860_stock_keyboard_drive_all(void)
{
	SLC860_GPSR2 = SLC860_ALL_STROBE_BITS;
	SLC860_GPDR2 |= SLC860_ALL_STROBE_BITS;
	udelay(SLC860_STOCK_KEY_SETTLE_US);
	SLC860_GEDR1 |= SLC860_HIGH_SENSE_BITS;
	SLC860_GEDR2 |= SLC860_LOW_SENSE_BITS;
}

/* This is the electrical scan performed by Sharp's keyscan routine. */
static int slc860_stock_keyboard_scan(void)
{
	unsigned long flags;
	int pressed_col = -1;
	int pressed_row = -1;
	unsigned int col;

	udelay(SLC860_STOCK_KEY_CHATTER_US);
	local_irq_save(flags);

	for (col = 0; col < SLC860_STOCK_KEY_COLS; col++) {
		u32 rows;
		unsigned int row;
		bool multiple = false;

		slc860_stock_keyboard_all_hiz();
		udelay(SLC860_STOCK_KEY_SETTLE_US);
		slc860_stock_keyboard_activate_col(col);
		udelay(SLC860_STOCK_KEY_SETTLE_US);

		rows = ((SLC860_GPLR1 & SLC860_HIGH_SENSE_BITS) >>
			SLC860_HIGH_SENSE_RSHIFT) |
		       ((SLC860_GPLR2 & SLC860_LOW_SENSE_BITS) <<
			SLC860_LOW_SENSE_LSHIFT);
		for (row = 0; row < SLC860_STOCK_KEY_ROWS; row++) {
			if (!(rows & BIT(row)))
				continue;
			if (pressed_row >= 0) {
				pressed_row = -1;
				multiple = true;
				break;
			}
			pressed_row = row;
			pressed_col = col;
		}
		slc860_stock_keyboard_reset_col(col);
		if (multiple)
			break;
	}

	if (pressed_col != 0)
		pressed_row = -1;
	slc860_stock_keyboard_drive_all();
	local_irq_restore(flags);

	return pressed_row;
}

static bool slc860_stock_keyboard_is_wakeup(int *accepted_row)
{
	int row = slc860_stock_keyboard_scan();
	unsigned int count;

	if (row < 3 || row > 7)
		return false;
	if (row == 7) {
		*accepted_row = row;
		return true;
	}

	for (count = 0; count < SLC860_STOCK_KEY_DEBOUNCE_COUNT; count++) {
		mdelay(5);
		if (slc860_stock_keyboard_scan() != row)
			return false;
	}

	if (row == 3 || row == 5) {
		for (count = 0; count < SLC860_STOCK_KEY_HOLD_COUNT; count++) {
			mdelay(5);
			if (slc860_stock_keyboard_scan() != row)
				break;
		}
	}

	*accepted_row = row;
	return true;
}

static u32 slc860_stock_wakeup_factor(u32 wake_pedr)
{
	u32 factor = wake_pedr & SLC860_STOCK_WAKE_MASK;
	u32 gplr = SLC860_GPLR0 & ~GPIO_bit(SLC860_GPIO_KEY_INT);
	unsigned int gpio;

	factor &= ~PWER_RTC;
	if ((RTSR & RTSR_AL) && (RTSR & RTSR_ALE))
		factor |= PWER_RTC;

	for (gpio = 0; gpio <= 15; gpio++) {
		u32 bit = GPIO_bit(gpio);

		if (gpio == SLC860_GPIO_AK_INT || !(factor & bit))
			continue;
		if ((PRER & SLC860_STOCK_WAKE_MASK & bit) && !(gplr & bit))
			factor &= ~bit;
		if ((PFER & SLC860_STOCK_WAKE_MASK & bit) && (gplr & bit))
			factor &= ~bit;
	}

	return factor;
}
#endif

static void slc860_presuspend(void)
{
	struct slc860_pm *pm = slc860_pm_state();

	pm->last_ac_present = gpiod_get_value(pm->ac_present);

#ifdef CONFIG_SHARP_SL_C860_DEEP_RESUME
	if (slc860_deep_resume_active()) {
		u32 rising = SLC860_STOCK_WAKE_RISING;
		u32 falling = SLC860_STOCK_WAKE_FALLING;
		u32 both = SLC860_STOCK_WAKE_BOTH;
		unsigned int gpio;

		for (gpio = 0; gpio <= 15; gpio++) {
			u32 bit = GPIO_bit(gpio);

			if (!(both & bit))
				continue;
			if (SLC860_GPLR0 & bit)
				rising &= ~bit;
			else
				falling &= ~bit;
		}
		PWER = SLC860_STOCK_WAKE_MASK;
		PRER = rising;
		PFER = falling;
		PEDR = SLC860_STOCK_WAKE_MASK;
		RCSR = RCSR_HWR | RCSR_WDR | RCSR_SMR | RCSR_GPR;
		slc860_saved_pcfr = PCFR;
		PCFR = PCFR_OPDE;
		PGSR0 = 0x0158c000;
		PGSR1 = 0x00ff0080;
		PGSR2 = 0x0001c004;
		SLC860_GPDR0 = 0xd3f83040;
		SLC860_GPDR1 = 0x00ffafc3;
		SLC860_GPDR2 = 0x0001c004;
		dev_info(sharpsl_pm.dev,
			 "ZAURUS-WAKE-POLICY stock pwer=%08x prer=%08x pfer=%08x gplr0=%08x pgsr=%08x/%08x/%08x strobe0=%08x\n",
			 PWER, PRER, PFER, SLC860_GPLR0, PGSR0, PGSR1, PGSR2,
			 SLC860_STROBE_BIT(0));
	}
#endif
}

static void slc860_postsuspend(void)
{
#ifdef CONFIG_SHARP_SL_C860_DEEP_RESUME
	if (slc860_deep_resume_active())
		PCFR = slc860_saved_pcfr;
#endif
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

#ifndef CONFIG_SHARP_SL_C860_DEEP_RESUME
	if (pm->last_ac_present != ac_present)
		pm->last_ac_present = ac_present;
	if (gpiod_get_value(pm->key_wakeup))
		is_resume = 1;
	if (gpiod_get_value(pm->power_wakeup))
		is_resume = 1;
	if (resume_on_alarm && (PEDR & PWER_RTC))
		is_resume |= PWER_RTC;
	return is_resume;
#else
	u32 wake_pedr = PEDR;
	bool deep_resume = slc860_deep_resume_active();
	u32 wake_factor;
	int keyboard_row = -1;
	bool keyboard_accepted = false;

	if (!deep_resume) {
		if (pm->last_ac_present != ac_present)
			pm->last_ac_present = ac_present;
		if (gpiod_get_value(pm->key_wakeup))
			is_resume = 1;
		if (gpiod_get_value(pm->power_wakeup))
			is_resume = 1;
		if (resume_on_alarm && (wake_pedr & PWER_RTC))
			is_resume |= PWER_RTC;
		return is_resume;
	}
	wake_factor = slc860_stock_wakeup_factor(wake_pedr);

	if (wake_factor & GPIO_bit(SLC860_GPIO_AC_IN))
		pm->last_ac_present = ac_present;

	if (wake_factor & GPIO_bit(SLC860_GPIO_KEY_INT)) {
		keyboard_accepted =
			slc860_stock_keyboard_is_wakeup(&keyboard_row);
		if (keyboard_accepted)
			is_resume |= GPIO_bit(SLC860_GPIO_KEY_INT);
		dev_info(sharpsl_pm.dev,
			 "ZAURUS-WAKE keyboard-summary pedr=%08x row=%d accepted=%u\n",
			 wake_pedr, keyboard_row, keyboard_accepted);
	}

	if (wake_factor & GPIO_bit(SLC860_GPIO_WAKEUP))
		is_resume |= GPIO_bit(SLC860_GPIO_WAKEUP);

	if (wake_factor & GPIO_bit(SLC860_GPIO_AK_INT))
		dev_info(sharpsl_pm.dev,
			 "ZAURUS-WAKE ak-remocon pedr=%08x factor=%08x accepted=0 reason=factory-hook-always-rejects-resume\n",
			 wake_pedr, wake_factor);

	if (wake_factor & GPIO_bit(SLC860_GPIO_MAIN_BAT_LOW))
		dev_info(sharpsl_pm.dev,
			 "ZAURUS-WAKE main-battery-low pedr=%08x factor=%08x accepted=0 reason=reset-combo-hook-not-ported\n",
			 wake_pedr, wake_factor);

	if (wake_factor & PWER_RTC) {
		const char *rtc_class;

		if (resume_on_alarm)
			rtc_class = "scheduled-user";
		else if (sharpsl_pm.flags & SHARPSL_ALARM_ACTIVE)
			rtc_class = "maintenance";
		else
			rtc_class = "unexpected-unarmed";
		if (resume_on_alarm)
			is_resume |= PWER_RTC;
		dev_info(sharpsl_pm.dev,
			 "ZAURUS-WAKE rtc pedr=%08x factor=%08x class=%s accepted=%u rcnr=%08x rtar=%08x rtsr=%08x\n",
			 wake_pedr, wake_factor, rtc_class,
			 !!resume_on_alarm, RCNR, RTAR, RTSR);
	}

	dev_info(sharpsl_pm.dev,
		 "ZAURUS-WAKE-FACTOR pedr=%08x filtered=%08x prer=%08x pfer=%08x gplr0=%08x\n",
		 wake_pedr, wake_factor, PRER, PFER, SLC860_GPLR0);

	return is_resume;
#endif
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
