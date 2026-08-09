// SPDX-License-Identifier: GPL-2.0-only
/*
 * Bounded hardware diagnostics for the PXA2xx FICP Device Tree device.
 *
 * Linux no longer carries its IrDA network stack. This driver intentionally
 * stops below that layer: it proves the board power GPIO, PXA pinctrl, clocks,
 * and SIR selection register and leaves framing to a future protocol port.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>

#define FICP_ICCR0		0x00
#define FICP_ICCR1		0x04
#define FICP_ICCR2		0x08
#define FICP_ICSR0		0x14
#define FICP_ICSR1		0x18

#define STUART_IRSEL		0x20
#define STISR_RXPL		BIT(4)
#define STISR_XMODE		BIT(2)
#define STISR_RCVEIR		BIT(1)
#define STISR_XMITIR		BIT(0)

#define STISR_SIR_RX		(STISR_RXPL | STISR_XMODE | STISR_RCVEIR)
#define STISR_SIR_TX		(STISR_RXPL | STISR_XMODE | STISR_XMITIR)

#define FICP_CAP_OFF		BIT(0)
#define FICP_CAP_SIR		BIT(1)

enum pxa2xx_ficp_diag_mode {
	PXA2XX_FICP_OFF,
	PXA2XX_FICP_SIR_RX,
	PXA2XX_FICP_SIR_TX,
};

struct pxa2xx_ficp_diag {
	struct device *dev;
	struct gpio_desc *powerdown_gpio;
	struct clk *fir_clk;
	struct clk *sir_clk;
	struct pinctrl *pinctrl;
	struct pinctrl_state *default_state;
	struct pinctrl_state *sleep_state;
	void __iomem *ficp;
	void __iomem *stuart;
	struct mutex lock;
	enum pxa2xx_ficp_diag_mode mode;
	enum pxa2xx_ficp_diag_mode resume_mode;
	u32 reset_iccr0;
	u32 reset_iccr1;
	u32 reset_iccr2;
	u32 reset_icsr0;
	u32 reset_icsr1;
	bool sir_clock_enabled;
};

static const char *pxa2xx_ficp_mode_name(enum pxa2xx_ficp_diag_mode mode)
{
	switch (mode) {
	case PXA2XX_FICP_SIR_RX:
		return "sir-rx";
	case PXA2XX_FICP_SIR_TX:
		return "sir-tx";
	default:
		return "off";
	}
}

static int pxa2xx_ficp_board_mode(struct pxa2xx_ficp_diag *diag,
				   enum pxa2xx_ficp_diag_mode mode)
{
	struct pinctrl_state *state;
	int ret;

	if (mode == PXA2XX_FICP_OFF && diag->powerdown_gpio)
		gpiod_set_value_cansleep(diag->powerdown_gpio, 1);

	if (!diag->pinctrl)
		goto power_on;

	state = mode == PXA2XX_FICP_OFF ? diag->sleep_state :
						 diag->default_state;
	if (IS_ERR(state))
		goto power_on;

	ret = pinctrl_select_state(diag->pinctrl, state);
	if (ret)
		dev_err(diag->dev, "could not select %s pin state: %d\n",
			mode == PXA2XX_FICP_OFF ? "sleep" : "default", ret);
	if (ret)
		return ret;

power_on:
	/* Do not power the transceiver until the active mux is established. */
	if (mode != PXA2XX_FICP_OFF && diag->powerdown_gpio)
		gpiod_set_value_cansleep(diag->powerdown_gpio, 0);
	return 0;
}

static int pxa2xx_ficp_set_mode(struct pxa2xx_ficp_diag *diag,
				enum pxa2xx_ficp_diag_mode mode)
{
	int ret;

	if (mode == diag->mode)
		return 0;

	if (mode == PXA2XX_FICP_OFF) {
		writel_relaxed(0, diag->stuart + STUART_IRSEL);
		ret = pxa2xx_ficp_board_mode(diag, mode);
		if (diag->sir_clock_enabled) {
			clk_disable_unprepare(diag->sir_clk);
			diag->sir_clock_enabled = false;
		}
		/* Hardware is safe-off even if the optional sleep mux failed. */
		diag->mode = mode;
		return ret;
	}

	if (!diag->sir_clock_enabled) {
		ret = clk_prepare_enable(diag->sir_clk);
		if (ret)
			return ret;
		diag->sir_clock_enabled = true;
	}

	ret = pxa2xx_ficp_board_mode(diag, mode);
	if (ret) {
		if (diag->powerdown_gpio)
			gpiod_set_value_cansleep(diag->powerdown_gpio, 1);
		clk_disable_unprepare(diag->sir_clk);
		diag->sir_clock_enabled = false;
		return ret;
	}
	writel_relaxed(mode == PXA2XX_FICP_SIR_RX ?
		       STISR_SIR_RX : STISR_SIR_TX,
		       diag->stuart + STUART_IRSEL);
	diag->mode = mode;
	return 0;
}

static ssize_t mode_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct pxa2xx_ficp_diag *diag = dev_get_drvdata(dev);
	const char *name;

	mutex_lock(&diag->lock);
	name = pxa2xx_ficp_mode_name(diag->mode);
	mutex_unlock(&diag->lock);
	return sysfs_emit(buf, "%s\n", name);
}

static ssize_t mode_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct pxa2xx_ficp_diag *diag = dev_get_drvdata(dev);
	enum pxa2xx_ficp_diag_mode mode;
	int ret;

	if (sysfs_streq(buf, "off"))
		mode = PXA2XX_FICP_OFF;
	else if (sysfs_streq(buf, "sir-rx"))
		mode = PXA2XX_FICP_SIR_RX;
	else if (sysfs_streq(buf, "sir-tx"))
		mode = PXA2XX_FICP_SIR_TX;
	else
		return -EINVAL;

	mutex_lock(&diag->lock);
	ret = pxa2xx_ficp_set_mode(diag, mode);
	mutex_unlock(&diag->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(mode);

static ssize_t status_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct pxa2xx_ficp_diag *diag = dev_get_drvdata(dev);
	const char *name;
	int gpio = -1;
	u32 irsel;
	bool clock_enabled;

	mutex_lock(&diag->lock);
	name = pxa2xx_ficp_mode_name(diag->mode);
	clock_enabled = diag->sir_clock_enabled;
	irsel = readl_relaxed(diag->stuart + STUART_IRSEL);
	if (diag->powerdown_gpio)
		gpio = gpiod_get_value_cansleep(diag->powerdown_gpio);
	mutex_unlock(&diag->lock);

	return sysfs_emit(buf,
			  "mode=%s caps=0x%lx gpio_pwdown=-1 gpio_value=%d "
			  "sir_clock=%u stisr=0x%08x "
			  "reset_iccr0=0x%08x reset_iccr1=0x%08x "
			  "reset_iccr2=0x%08x reset_icsr0=0x%08x "
			  "reset_icsr1=0x%08x protocol_stack=absent\n",
			  name, FICP_CAP_OFF | FICP_CAP_SIR, gpio,
			  clock_enabled, irsel,
			  diag->reset_iccr0, diag->reset_iccr1,
			  diag->reset_iccr2, diag->reset_icsr0,
			  diag->reset_icsr1);
}
static DEVICE_ATTR_RO(status);

static struct attribute *pxa2xx_ficp_diag_attrs[] = {
	&dev_attr_mode.attr,
	&dev_attr_status.attr,
	NULL,
};
ATTRIBUTE_GROUPS(pxa2xx_ficp_diag);

static int pxa2xx_ficp_diag_probe(struct platform_device *pdev)
{
	struct pxa2xx_ficp_diag *diag;
	struct resource *resource;
	int ret;

	diag = devm_kzalloc(&pdev->dev, sizeof(*diag), GFP_KERNEL);
	if (!diag)
		return -ENOMEM;

	diag->dev = &pdev->dev;
	mutex_init(&diag->lock);

	resource = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ficp");
	diag->ficp = devm_ioremap_resource(&pdev->dev, resource);
	if (IS_ERR(diag->ficp))
		return PTR_ERR(diag->ficp);

	resource = platform_get_resource_byname(pdev, IORESOURCE_MEM, "stuart");
	diag->stuart = devm_ioremap(&pdev->dev, resource->start,
				    resource_size(resource));
	if (!diag->stuart)
		return -ENOMEM;

	diag->fir_clk = devm_clk_get(&pdev->dev, "ficp");
	if (IS_ERR(diag->fir_clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(diag->fir_clk),
				     "could not get FICP clock\n");
	diag->sir_clk = devm_clk_get(&pdev->dev, "stuart");
	if (IS_ERR(diag->sir_clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(diag->sir_clk),
				     "could not get STUART clock\n");

	diag->powerdown_gpio = devm_gpiod_get_optional(&pdev->dev,
						       "powerdown",
						       GPIOD_OUT_HIGH);
	if (IS_ERR(diag->powerdown_gpio))
		return dev_err_probe(&pdev->dev, PTR_ERR(diag->powerdown_gpio),
				     "could not request power-down GPIO\n");

	diag->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(diag->pinctrl)) {
		if (PTR_ERR(diag->pinctrl) != -ENODEV)
			return dev_err_probe(&pdev->dev, PTR_ERR(diag->pinctrl),
					     "could not get pinctrl\n");
		diag->pinctrl = NULL;
	} else {
		diag->default_state = pinctrl_lookup_state(diag->pinctrl,
							  PINCTRL_STATE_DEFAULT);
		diag->sleep_state = pinctrl_lookup_state(diag->pinctrl,
							PINCTRL_STATE_SLEEP);
	}

	ret = clk_prepare_enable(diag->fir_clk);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "could not enable FICP clock\n");
	diag->reset_iccr0 = readl_relaxed(diag->ficp + FICP_ICCR0);
	diag->reset_iccr1 = readl_relaxed(diag->ficp + FICP_ICCR1);
	diag->reset_iccr2 = readl_relaxed(diag->ficp + FICP_ICCR2);
	diag->reset_icsr0 = readl_relaxed(diag->ficp + FICP_ICSR0);
	diag->reset_icsr1 = readl_relaxed(diag->ficp + FICP_ICSR1);
	clk_disable_unprepare(diag->fir_clk);

	ret = pxa2xx_ficp_board_mode(diag, PXA2XX_FICP_OFF);
	if (ret)
		return ret;
	writel_relaxed(0, diag->stuart + STUART_IRSEL);
	platform_set_drvdata(pdev, diag);
	dev_info(&pdev->dev,
		 "diagnostic ready; full IrDA protocol stack is not present\n");
	return 0;
}

static void pxa2xx_ficp_diag_remove(struct platform_device *pdev)
{
	struct pxa2xx_ficp_diag *diag = platform_get_drvdata(pdev);

	mutex_lock(&diag->lock);
	pxa2xx_ficp_set_mode(diag, PXA2XX_FICP_OFF);
	mutex_unlock(&diag->lock);
}

static int pxa2xx_ficp_diag_suspend(struct device *dev)
{
	struct pxa2xx_ficp_diag *diag = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&diag->lock);
	diag->resume_mode = diag->mode;
	ret = pxa2xx_ficp_set_mode(diag, PXA2XX_FICP_OFF);
	mutex_unlock(&diag->lock);
	return ret;
}

static int pxa2xx_ficp_diag_resume(struct device *dev)
{
	struct pxa2xx_ficp_diag *diag = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&diag->lock);
	ret = pxa2xx_ficp_set_mode(diag, diag->resume_mode);
	mutex_unlock(&diag->lock);
	return ret;
}

static void pxa2xx_ficp_diag_shutdown(struct platform_device *pdev)
{
	pxa2xx_ficp_diag_remove(pdev);
}

static DEFINE_SIMPLE_DEV_PM_OPS(pxa2xx_ficp_diag_pm_ops,
				 pxa2xx_ficp_diag_suspend,
				 pxa2xx_ficp_diag_resume);

static const struct of_device_id pxa2xx_ficp_diag_of_match[] = {
	{ .compatible = "marvell,pxa25x-ficp" },
	{ }
};
MODULE_DEVICE_TABLE(of, pxa2xx_ficp_diag_of_match);

static struct platform_driver pxa2xx_ficp_diag_driver = {
	.probe = pxa2xx_ficp_diag_probe,
	.remove = pxa2xx_ficp_diag_remove,
	.shutdown = pxa2xx_ficp_diag_shutdown,
	.driver = {
		.name = "pxa2xx-ir",
		.of_match_table = pxa2xx_ficp_diag_of_match,
		.dev_groups = pxa2xx_ficp_diag_groups,
		.pm = pm_sleep_ptr(&pxa2xx_ficp_diag_pm_ops),
	},
};
module_platform_driver(pxa2xx_ficp_diag_driver);

MODULE_AUTHOR("Zaurus SD bring-up");
MODULE_DESCRIPTION("PXA2xx FICP hardware diagnostic interface");
MODULE_LICENSE("GPL");
