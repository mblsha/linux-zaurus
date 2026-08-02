// SPDX-License-Identifier: GPL-2.0-only
/*
 * Battery and Power Management code for the Sharp SL-C7xx
 *
 * Copyright (c) 2005 Richard Purdie
 */

#include <linux/module.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio-pxa.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/apm-emulation.h>
#include <linux/io.h>

#include <asm/irq.h>
#include <asm/mach-types.h>

#include "corgi.h"
#include "pxa2xx-regs.h"
#include "regs-rtc.h"
#include "sharpsl_pm.h"

#include "generic.h"

#define SHARPSL_CHARGE_ON_VOLT         0x99  /* 2.9V */
#define SHARPSL_CHARGE_ON_TEMP         0xe0  /* 2.9V */
#define SHARPSL_CHARGE_ON_ACIN_HIGH    0x9b  /* 6V */
#define SHARPSL_CHARGE_ON_ACIN_LOW     0x34  /* 2V */
#define SHARPSL_FATAL_ACIN_VOLT        182   /* 3.45V */
#define SHARPSL_FATAL_NOACIN_VOLT      170   /* 3.40V */

struct corgi_pm_gpio_state {
	struct gpio_desc *temp_enable;
	struct gpio_desc *charge_enable;
	struct gpio_desc *charge_unknown;
	struct gpio_desc *discharge_enable;
	struct gpio_desc *ac_present;
	struct gpio_desc *battery_full;
	struct gpio_desc *battery_cover;
	struct gpio_desc *key_wakeup;
	struct gpio_desc *power_wakeup;
	bool dt_owned;
	bool charging_disabled;
};

static struct corgi_pm_gpio_state corgi_pm_gpios;

static int corgi_pm_gpio_get(struct gpio_desc *desc, unsigned int gpio)
{
	return desc ? gpiod_get_value(desc) : gpio_get_value(gpio);
}

static void corgi_pm_gpio_set(struct gpio_desc *desc, unsigned int gpio,
			      int value)
{
	if (desc)
		gpiod_set_value(desc, value);
	else
		gpio_set_value(gpio, value);
}

static struct gpio charger_gpios[] = {
	{ CORGI_GPIO_ADC_TEMP_ON, GPIOF_OUT_INIT_LOW, "ADC Temp On" },
	{ CORGI_GPIO_CHRG_ON,	  GPIOF_OUT_INIT_LOW, "Charger On" },
	{ CORGI_GPIO_CHRG_UKN,	  GPIOF_OUT_INIT_LOW, "Charger Unknown" },
	{ CORGI_GPIO_AC_IN,	  GPIOF_IN, "Charger Detection" },
	{ CORGI_GPIO_KEY_INT,	  GPIOF_IN, "Key Interrupt" },
	{ CORGI_GPIO_WAKEUP,	  GPIOF_IN, "System wakeup notification" },
};

static void corgi_charger_init(void)
{
	if (!corgi_pm_gpios.dt_owned)
		gpio_request_array(ARRAY_AND_SIZE(charger_gpios));
}

static void corgi_measure_temp(int on)
{
	corgi_pm_gpio_set(corgi_pm_gpios.temp_enable,
			  CORGI_GPIO_ADC_TEMP_ON, on);
}

static void corgi_charge(int on)
{
	if (corgi_pm_gpios.charging_disabled)
		on = 0;

	if (corgi_pm_gpios.dt_owned) {
		corgi_pm_gpio_set(corgi_pm_gpios.charge_enable,
				  CORGI_GPIO_CHRG_ON, on);
		corgi_pm_gpio_set(corgi_pm_gpios.charge_unknown,
				  CORGI_GPIO_CHRG_UKN, 0);
		return;
	}

	if (on) {
		if (machine_is_corgi() && (sharpsl_pm.flags & SHARPSL_SUSPENDED)) {
			gpio_set_value(CORGI_GPIO_CHRG_ON, 0);
			gpio_set_value(CORGI_GPIO_CHRG_UKN, 1);
		} else {
			gpio_set_value(CORGI_GPIO_CHRG_ON, 1);
			gpio_set_value(CORGI_GPIO_CHRG_UKN, 0);
		}
	} else {
		gpio_set_value(CORGI_GPIO_CHRG_ON, 0);
		gpio_set_value(CORGI_GPIO_CHRG_UKN, 0);
	}
}

static void corgi_discharge(int on)
{
	corgi_pm_gpio_set(corgi_pm_gpios.discharge_enable,
			  CORGI_GPIO_DISCHARGE_ON, on);
}

#ifdef CONFIG_SHARP_SL_C860_DEEP_RESUME
#define CORGI_GPLR0	__REG(0x40e00000)
#define CORGI_GPLR1	__REG(0x40e00004)
#define CORGI_GPLR2	__REG(0x40e00008)
#define CORGI_GPDR0	__REG(0x40e0000c)
#define CORGI_GPDR1	__REG(0x40e00010)
#define CORGI_GPDR2	__REG(0x40e00014)
#define CORGI_GPSR2	__REG(0x40e00020)
#define CORGI_GPCR2	__REG(0x40e0002c)
#define CORGI_GEDR1	__REG(0x40e0004c)
#define CORGI_GEDR2	__REG(0x40e00050)

#define CORGI_STOCK_WAKE_RISING					\
	(GPIO_bit(CORGI_GPIO_AC_IN) |				\
	 GPIO_bit(CORGI_GPIO_AK_INT) |				\
	 GPIO_bit(CORGI_GPIO_MAIN_BAT_LOW))
#define CORGI_STOCK_WAKE_FALLING				\
	(GPIO_bit(CORGI_GPIO_KEY_INT) |				\
	 GPIO_bit(CORGI_GPIO_WAKEUP) |				\
	 GPIO_bit(CORGI_GPIO_AC_IN) |				\
	 GPIO_bit(CORGI_GPIO_MAIN_BAT_LOW))
#define CORGI_STOCK_WAKE_MASK					\
	(CORGI_STOCK_WAKE_RISING | CORGI_STOCK_WAKE_FALLING | PWER_RTC)
#define CORGI_STOCK_WAKE_BOTH					\
	(CORGI_STOCK_WAKE_RISING & CORGI_STOCK_WAKE_FALLING)

#define CORGI_STOCK_KEY_ROWS		8
#define CORGI_STOCK_KEY_COLS		12
#define CORGI_STOCK_KEY_CHATTER_US	100
#define CORGI_STOCK_KEY_SETTLE_US	10
#define CORGI_STOCK_KEY_DEBOUNCE_COUNT	2
#define CORGI_STOCK_KEY_HOLD_COUNT	120

static unsigned long corgi_saved_pcfr;

static bool corgi_deep_resume_active(void)
{
	return machine_is_husky() ||
	       of_machine_is_compatible("sharp,sl-c860");
}

static void corgi_stock_keyboard_all_hiz(void)
{
	CORGI_GPCR2 = CORGI_GPIO_ALL_STROBE_BIT;
	CORGI_GPDR2 &= ~CORGI_GPIO_ALL_STROBE_BIT;
}

static void corgi_stock_keyboard_activate_col(unsigned int col)
{
	u32 bit = CORGI_GPIO_STROBE_BIT(col);

	CORGI_GPSR2 = bit;
	CORGI_GPDR2 = (CORGI_GPDR2 & ~CORGI_GPIO_ALL_STROBE_BIT) | bit;
}

static void corgi_stock_keyboard_reset_col(unsigned int col)
{
	u32 bit = CORGI_GPIO_STROBE_BIT(col);

	CORGI_GPCR2 = bit;
	CORGI_GPDR2 = (CORGI_GPDR2 & ~CORGI_GPIO_ALL_STROBE_BIT) | bit;
}

static void corgi_stock_keyboard_drive_all(void)
{
	CORGI_GPSR2 = CORGI_GPIO_ALL_STROBE_BIT;
	CORGI_GPDR2 |= CORGI_GPIO_ALL_STROBE_BIT;
	udelay(CORGI_STOCK_KEY_SETTLE_US);
	CORGI_GEDR1 |= CORGI_GPIO_HIGH_SENSE_BIT;
	CORGI_GEDR2 |= CORGI_GPIO_LOW_SENSE_BIT;
}

/*
 * This is the electrical scan performed by Sharp's sharppda_kbd_keyscan().
 * A qualified wake is exactly one key in physical strobe column zero.
 */
static int corgi_stock_keyboard_scan(void)
{
	unsigned long flags;
	int pressed_col = -1;
	int pressed_row = -1;
	unsigned int col;

	udelay(CORGI_STOCK_KEY_CHATTER_US);
	local_irq_save(flags);

	for (col = 0; col < CORGI_STOCK_KEY_COLS; col++) {
		u32 rows;
		unsigned int row;
		bool multiple = false;

		corgi_stock_keyboard_all_hiz();
		udelay(CORGI_STOCK_KEY_SETTLE_US);
		corgi_stock_keyboard_activate_col(col);
		udelay(CORGI_STOCK_KEY_SETTLE_US);

		rows = ((CORGI_GPLR1 & CORGI_GPIO_HIGH_SENSE_BIT) >>
			CORGI_GPIO_HIGH_SENSE_RSHIFT) |
		       ((CORGI_GPLR2 & CORGI_GPIO_LOW_SENSE_BIT) <<
			CORGI_GPIO_LOW_SENSE_LSHIFT);
		for (row = 0; row < CORGI_STOCK_KEY_ROWS; row++) {
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
		corgi_stock_keyboard_reset_col(col);
		if (multiple)
			break;
	}

	if (pressed_col != 0)
		pressed_row = -1;
	corgi_stock_keyboard_drive_all();
	local_irq_restore(flags);

	return pressed_row;
}

/*
 * Match Sharp's wake filter: rows 3..7 in physical column zero, two 5 ms
 * chatter checks, and the additional 600 ms hold classification for rows
 * 3 and 5. Event replay remains the normal matrix-keypad driver's job.
 */
static bool corgi_stock_keyboard_is_wakeup(int *accepted_row)
{
	int row = corgi_stock_keyboard_scan();
	unsigned int count;

	if (row < 3 || row > 7)
		return false;
	if (row == 7) {
		*accepted_row = row;
		return true;
	}

	for (count = 0; count < CORGI_STOCK_KEY_DEBOUNCE_COUNT; count++) {
		mdelay(5);
		if (corgi_stock_keyboard_scan() != row)
			return false;
	}

	if (row == 3 || row == 5) {
		for (count = 0; count < CORGI_STOCK_KEY_HOLD_COUNT; count++) {
			mdelay(5);
			if (corgi_stock_keyboard_scan() != row)
				break;
		}
	}

	*accepted_row = row;
	return true;
}

/*
 * Match sharpsl_wakeup_check(): RTC is reported by RTSR_AL+RTSR_ALE rather
 * than trusted from PEDR, and GPIO edges must agree with the post-wake level.
 * GPIO0 is a matrix summary, so Sharp deliberately forced its sampled level
 * low before applying the falling-edge plausibility check.
 */
static u32 corgi_stock_wakeup_factor(u32 wake_pedr)
{
	u32 factor = wake_pedr & CORGI_STOCK_WAKE_MASK;
	u32 gplr = CORGI_GPLR0 & ~GPIO_bit(CORGI_GPIO_KEY_INT);
	unsigned int gpio;

	factor &= ~PWER_RTC;
	if ((RTSR & RTSR_AL) && (RTSR & RTSR_ALE))
		factor |= PWER_RTC;

	for (gpio = 0; gpio <= 15; gpio++) {
		u32 bit = GPIO_bit(gpio);

		if (gpio == CORGI_GPIO_AK_INT || !(factor & bit))
			continue;
		if ((PRER & CORGI_STOCK_WAKE_MASK & bit) && !(gplr & bit))
			factor &= ~bit;
		if ((PFER & CORGI_STOCK_WAKE_MASK & bit) && (gplr & bit))
			factor &= ~bit;
	}

	return factor;
}
#endif

static void corgi_presuspend(void)
{
#ifdef CONFIG_SHARP_SL_C860_DEEP_RESUME
	u32 rising = CORGI_STOCK_WAKE_RISING;
	u32 falling = CORGI_STOCK_WAKE_FALLING;
	u32 both = CORGI_STOCK_WAKE_BOTH;
	unsigned int gpio;

	if (!corgi_deep_resume_active())
		return;

	/*
	 * Match the board-qualified Sharp sleep state.  The generic PXA2xx
	 * MFP syscore callback has already saved the run-time GPIO state when
	 * platform ->enter() reaches here, so its resume callback will restore
	 * these registers.
	 */
	for (gpio = 0; gpio <= 15; gpio++) {
		u32 bit = GPIO_bit(gpio);

		if (!(both & bit))
			continue;
		if (CORGI_GPLR0 & bit)
			rising &= ~bit;
		else
			falling &= ~bit;
	}
	PWER = CORGI_STOCK_WAKE_MASK;
	PRER = rising;
	PFER = falling;
	PEDR = CORGI_STOCK_WAKE_MASK;
	RCSR = RCSR_HWR | RCSR_WDR | RCSR_SMR | RCSR_GPR;
	corgi_saved_pcfr = PCFR;
	PCFR = PCFR_OPDE;
	/* Exact values installed by Sharp's Corgi/Shepherd board setup. */
	PGSR0 = 0x0158c000;
	PGSR1 = 0x00ff0080;
	PGSR2 = 0x0001c004;
	CORGI_GPDR0 = 0xd3f83040;
	CORGI_GPDR1 = 0x00ffafc3;
	CORGI_GPDR2 = 0x0001c004;
	dev_info(sharpsl_pm.dev,
		 "ZAURUS-WAKE-POLICY stock pwer=%08x prer=%08x pfer=%08x gplr0=%08x pgsr=%08x/%08x/%08x strobe0=%08x\n",
		 PWER, PRER, PFER, CORGI_GPLR0, PGSR0, PGSR1, PGSR2,
		 CORGI_GPIO_STROBE_BIT(0));
#endif
}

static void corgi_postsuspend(void)
{
#ifdef CONFIG_SHARP_SL_C860_DEEP_RESUME
	if (corgi_deep_resume_active())
		PCFR = corgi_saved_pcfr;
#endif
}

/*
 * Check what brought us out of the suspend.
 * Return: 0 to sleep, otherwise wake
 */
static int corgi_should_wakeup(unsigned int resume_on_alarm)
{
	int is_resume = 0;
	u32 wake_pedr = PEDR;
#ifdef CONFIG_SHARP_SL_C860_DEEP_RESUME
	bool deep_resume = corgi_deep_resume_active();
	u32 wake_factor = deep_resume ?
		corgi_stock_wakeup_factor(wake_pedr) : wake_pedr;
	int keyboard_row = -1;
	bool keyboard_accepted = false;
#else
	u32 wake_factor = wake_pedr;
#endif

	dev_dbg(sharpsl_pm.dev, "PEDR = %x, GPIO_AC_IN = %d, "
		"GPIO_CHRG_FULL = %d, GPIO_KEY_INT = %d, GPIO_WAKEUP = %d\n",
		PEDR, gpio_get_value(CORGI_GPIO_AC_IN),
		gpio_get_value(CORGI_GPIO_CHRG_FULL),
		gpio_get_value(CORGI_GPIO_KEY_INT),
		gpio_get_value(CORGI_GPIO_WAKEUP));

	if ((wake_factor & GPIO_bit(CORGI_GPIO_AC_IN))) {
		if (sharpsl_pm.machinfo->read_devdata(SHARPSL_STATUS_ACIN)) {
			/* charge on */
			dev_dbg(sharpsl_pm.dev, "ac insert\n");
			if (!sharpsl_pm.machinfo->charging_disabled)
				sharpsl_pm.flags |= SHARPSL_DO_OFFLINE_CHRG;
		} else {
			/* charge off */
			dev_dbg(sharpsl_pm.dev, "ac remove\n");
			sharpsl_pm_led(SHARPSL_LED_OFF);
			sharpsl_pm.machinfo->charge(0);
			sharpsl_pm.charge_mode = CHRG_OFF;
		}
	}

	if ((wake_pedr & GPIO_bit(CORGI_GPIO_CHRG_FULL)))
		dev_dbg(sharpsl_pm.dev, "Charge full interrupt\n");

	if (wake_factor & GPIO_bit(CORGI_GPIO_KEY_INT)) {
#ifdef CONFIG_SHARP_SL_C860_DEEP_RESUME
		if (deep_resume)
			keyboard_accepted =
				corgi_stock_keyboard_is_wakeup(&keyboard_row);
		else
			keyboard_accepted = true;
		if (keyboard_accepted)
			is_resume |= GPIO_bit(CORGI_GPIO_KEY_INT);
		if (deep_resume)
			dev_info(sharpsl_pm.dev,
				 "ZAURUS-WAKE keyboard-summary pedr=%08x row=%d accepted=%u\n",
				 wake_pedr, keyboard_row,
				 keyboard_accepted);
#else
		is_resume |= GPIO_bit(CORGI_GPIO_KEY_INT);
#endif
	}

	if (wake_factor & GPIO_bit(CORGI_GPIO_WAKEUP))
		is_resume |= GPIO_bit(CORGI_GPIO_WAKEUP);

#ifdef CONFIG_SHARP_SL_C860_DEEP_RESUME
	if (deep_resume &&
	    (wake_factor & GPIO_bit(CORGI_GPIO_AK_INT)))
		dev_info(sharpsl_pm.dev,
			 "ZAURUS-WAKE ak-remocon pedr=%08x factor=%08x accepted=0 reason=factory-hook-always-rejects-resume\n",
			 wake_pedr, wake_factor);

	if (deep_resume &&
	    (wake_factor & GPIO_bit(CORGI_GPIO_MAIN_BAT_LOW)))
		dev_info(sharpsl_pm.dev,
			 "ZAURUS-WAKE main-battery-low pedr=%08x factor=%08x accepted=0 reason=reset-combo-hook-not-ported\n",
			 wake_pedr, wake_factor);
#endif

	if (wake_factor & PWER_RTC) {
#ifdef CONFIG_SHARP_SL_C860_DEEP_RESUME
		const char *rtc_class;

		if (resume_on_alarm)
			rtc_class = "scheduled-user";
		else if (sharpsl_pm.flags & SHARPSL_ALARM_ACTIVE)
			rtc_class = "maintenance";
		else
			rtc_class = "unexpected-unarmed";
#endif
		if (resume_on_alarm)
			is_resume |= PWER_RTC;
#ifdef CONFIG_SHARP_SL_C860_DEEP_RESUME
		if (deep_resume)
			dev_info(sharpsl_pm.dev,
				 "ZAURUS-WAKE rtc pedr=%08x factor=%08x class=%s accepted=%u rcnr=%08x rtar=%08x rtsr=%08x\n",
				 wake_pedr, wake_factor, rtc_class,
				 !!resume_on_alarm, RCNR, RTAR, RTSR);
#endif
	}

#ifdef CONFIG_SHARP_SL_C860_DEEP_RESUME
	if (deep_resume)
		dev_info(sharpsl_pm.dev,
			 "ZAURUS-WAKE-FACTOR pedr=%08x filtered=%08x prer=%08x pfer=%08x gplr0=%08x\n",
			 wake_pedr, wake_factor, PRER, PFER, CORGI_GPLR0);
#endif
	dev_dbg(sharpsl_pm.dev, "is_resume: %x\n",is_resume);
	return is_resume;
}

static bool corgi_charger_wakeup(void)
{
	if (corgi_pm_gpios.dt_owned)
		return corgi_pm_gpio_get(corgi_pm_gpios.ac_present,
					 CORGI_GPIO_AC_IN) ||
			corgi_pm_gpio_get(corgi_pm_gpios.key_wakeup,
					  CORGI_GPIO_KEY_INT) ||
			corgi_pm_gpio_get(corgi_pm_gpios.power_wakeup,
					  CORGI_GPIO_WAKEUP);

	return !gpio_get_value(CORGI_GPIO_AC_IN) ||
		!gpio_get_value(CORGI_GPIO_KEY_INT) ||
		!gpio_get_value(CORGI_GPIO_WAKEUP);
}

unsigned long corgipm_read_devdata(int type)
{
	switch(type) {
	case SHARPSL_STATUS_ACIN:
		if (corgi_pm_gpios.dt_owned)
			return corgi_pm_gpio_get(corgi_pm_gpios.ac_present,
						 CORGI_GPIO_AC_IN);
		return !gpio_get_value(CORGI_GPIO_AC_IN);
	case SHARPSL_STATUS_LOCK:
		if (corgi_pm_gpios.dt_owned)
			return corgi_pm_gpio_get(corgi_pm_gpios.battery_cover,
						 CORGI_GPIO_BAT_COVER);
		return gpio_get_value(sharpsl_pm.machinfo->gpio_batlock);
	case SHARPSL_STATUS_CHRGFULL:
		if (corgi_pm_gpios.dt_owned)
			return corgi_pm_gpio_get(corgi_pm_gpios.battery_full,
						 CORGI_GPIO_CHRG_FULL);
		return gpio_get_value(sharpsl_pm.machinfo->gpio_batfull);
	case SHARPSL_STATUS_FATAL:
		/* An unset gpio_fatal is absence, not PXA GPIO0. */
		if (!sharpsl_pm.machinfo->gpio_fatal)
			return 1;
		return gpio_get_value(sharpsl_pm.machinfo->gpio_fatal);
	case SHARPSL_ACIN_VOLT:
		return sharpsl_pm_pxa_read_max1111(MAX1111_ACIN_VOLT);
	case SHARPSL_BATT_TEMP:
		return sharpsl_pm_pxa_read_max1111(MAX1111_BATT_TEMP);
	case SHARPSL_BATT_VOLT:
	default:
		return sharpsl_pm_pxa_read_max1111(MAX1111_BATT_VOLT);
	}
}

static struct sharpsl_charger_machinfo corgi_pm_machinfo = {
	.init            = corgi_charger_init,
	.exit            = NULL,
	.gpio_batlock    = CORGI_GPIO_BAT_COVER,
	.gpio_acin       = CORGI_GPIO_AC_IN,
	.gpio_batfull    = CORGI_GPIO_CHRG_FULL,
	.wakeup_irq      = CORGI_IRQ_GPIO_WAKEUP,
	.key_wakeup_irq  = CORGI_IRQ_GPIO_KEY_INT,
	.discharge       = corgi_discharge,
	.charge          = corgi_charge,
	.measure_temp    = corgi_measure_temp,
	.presuspend      = corgi_presuspend,
	.postsuspend     = corgi_postsuspend,
	.read_devdata    = corgipm_read_devdata,
	.charger_wakeup  = corgi_charger_wakeup,
	.should_wakeup   = corgi_should_wakeup,
#if defined(CONFIG_LCD_CORGI)
	.backlight_limit = corgi_lcd_limit_intensity,
#endif
	.charge_on_volt	  = SHARPSL_CHARGE_ON_VOLT,
	.charge_on_temp	  = SHARPSL_CHARGE_ON_TEMP,
	.charge_acin_high = SHARPSL_CHARGE_ON_ACIN_HIGH,
	.charge_acin_low  = SHARPSL_CHARGE_ON_ACIN_LOW,
	.fatal_acin_volt  = SHARPSL_FATAL_ACIN_VOLT,
	.fatal_noacin_volt= SHARPSL_FATAL_NOACIN_VOLT,
	.bat_levels       = 40,
	.bat_levels_noac  = sharpsl_battery_levels_noac,
	.bat_levels_acin  = sharpsl_battery_levels_acin,
	.status_high_acin = 188,
	.status_low_acin  = 178,
	.status_high_noac = 185,
	.status_low_noac  = 175,
};

static struct platform_device *legacy_corgipm_device;

static int corgipm_get_gpio(struct device *dev, const char *name,
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

static int corgipm_of_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct platform_device *child;
	int ret;

	ret = corgipm_get_gpio(dev, "temperature-enable", GPIOD_OUT_LOW,
			       &corgi_pm_gpios.temp_enable);
	if (ret)
		return ret;
	ret = corgipm_get_gpio(dev, "charge-enable", GPIOD_OUT_LOW,
			       &corgi_pm_gpios.charge_enable);
	if (ret)
		return ret;
	ret = corgipm_get_gpio(dev, "charge-unknown", GPIOD_OUT_LOW,
			       &corgi_pm_gpios.charge_unknown);
	if (ret)
		return ret;
	ret = corgipm_get_gpio(dev, "discharge-enable", GPIOD_OUT_LOW,
			       &corgi_pm_gpios.discharge_enable);
	if (ret)
		return ret;
	ret = corgipm_get_gpio(dev, "ac-present", GPIOD_IN,
			       &corgi_pm_gpios.ac_present);
	if (ret)
		return ret;
	ret = corgipm_get_gpio(dev, "battery-full", GPIOD_IN,
			       &corgi_pm_gpios.battery_full);
	if (ret)
		return ret;
	ret = corgipm_get_gpio(dev, "battery-cover", GPIOD_IN,
			       &corgi_pm_gpios.battery_cover);
	if (ret)
		return ret;
	ret = corgipm_get_gpio(dev, "key-wakeup", GPIOD_IN,
			       &corgi_pm_gpios.key_wakeup);
	if (ret)
		return ret;
	ret = corgipm_get_gpio(dev, "power-wakeup", GPIOD_IN,
			       &corgi_pm_gpios.power_wakeup);
	if (ret)
		return ret;

	corgi_pm_gpios.dt_owned = true;
	corgi_pm_gpios.charging_disabled =
		device_property_read_bool(dev, "sharp,charging-disabled");
	if (!corgi_pm_gpios.charging_disabled)
		return dev_err_probe(dev, -EPERM,
			"refusing unqualified charging-enabled DT policy\n");

	corgi_pm_machinfo.acin_desc = corgi_pm_gpios.ac_present;
	corgi_pm_machinfo.batfull_desc = corgi_pm_gpios.battery_full;
	corgi_pm_machinfo.batlock_desc = corgi_pm_gpios.battery_cover;
	corgi_pm_machinfo.wakeup_irq =
		gpiod_to_irq(corgi_pm_gpios.power_wakeup);
	corgi_pm_machinfo.key_wakeup_irq =
		gpiod_to_irq(corgi_pm_gpios.key_wakeup);
	corgi_pm_machinfo.batfull_irq = 1;
	corgi_pm_machinfo.charging_disabled = true;
	if (corgi_pm_machinfo.wakeup_irq < 0)
		return dev_err_probe(dev, corgi_pm_machinfo.wakeup_irq,
				     "failed to map power-wakeup IRQ\n");
	if (corgi_pm_machinfo.key_wakeup_irq < 0)
		return dev_err_probe(dev, corgi_pm_machinfo.key_wakeup_irq,
				     "failed to map key-wakeup IRQ\n");

	child = platform_device_alloc("sharpsl-pm", PLATFORM_DEVID_NONE);
	if (!child)
		return -ENOMEM;
	child->dev.parent = dev;
	child->dev.platform_data = &corgi_pm_machinfo;
	ret = platform_device_add(child);
	if (ret) {
		platform_device_put(child);
		return ret;
	}
	platform_set_drvdata(pdev, child);
	dev_info(dev, "SL-C860 PM/battery GPIO ownership transferred to DT\n");
	return 0;
}

static int corgipm_of_remove(struct platform_device *pdev)
{
	struct platform_device *child = platform_get_drvdata(pdev);

	platform_device_unregister(child);
	corgi_pm_machinfo.acin_desc = NULL;
	corgi_pm_machinfo.batfull_desc = NULL;
	corgi_pm_machinfo.batlock_desc = NULL;
	corgi_pm_machinfo.wakeup_irq = CORGI_IRQ_GPIO_WAKEUP;
	corgi_pm_machinfo.key_wakeup_irq = CORGI_IRQ_GPIO_KEY_INT;
	corgi_pm_machinfo.charging_disabled = false;
	memset(&corgi_pm_gpios, 0, sizeof(corgi_pm_gpios));
	return 0;
}

static const struct of_device_id corgipm_of_match[] = {
	{ .compatible = "sharp,sl-c860-power" },
	{ }
};
MODULE_DEVICE_TABLE(of, corgipm_of_match);

static struct platform_driver corgipm_of_driver = {
	.probe = corgipm_of_probe,
	.remove = corgipm_of_remove,
	.driver = {
		.name = "sharp-sl-c860-power",
		.of_match_table = corgipm_of_match,
	},
};

static int corgipm_init(void)
{
	int ret;

	if (!machine_is_corgi() && !machine_is_shepherd()
			&& !machine_is_husky()
			&& !of_machine_is_compatible("sharp,sl-c860"))
		return -ENODEV;
	ret = platform_driver_register(&corgipm_of_driver);
	if (ret)
		return ret;

	if (IS_ENABLED(CONFIG_SHARP_SL_C860_DT_PM) &&
	    of_machine_is_compatible("sharp,sl-c860"))
		return 0;

	legacy_corgipm_device = platform_device_alloc("sharpsl-pm", -1);
	if (!legacy_corgipm_device) {
		platform_driver_unregister(&corgipm_of_driver);
		return -ENOMEM;
	}

	if (!machine_is_corgi())
	    corgi_pm_machinfo.batfull_irq = 1;

	legacy_corgipm_device->dev.platform_data = &corgi_pm_machinfo;
	ret = platform_device_add(legacy_corgipm_device);

	if (ret) {
		platform_device_put(legacy_corgipm_device);
		legacy_corgipm_device = NULL;
		platform_driver_unregister(&corgipm_of_driver);
	}

	return ret;
}

static void corgipm_exit(void)
{
	if (legacy_corgipm_device)
		platform_device_unregister(legacy_corgipm_device);
	platform_driver_unregister(&corgipm_of_driver);
}

module_init(corgipm_init);
module_exit(corgipm_exit);
