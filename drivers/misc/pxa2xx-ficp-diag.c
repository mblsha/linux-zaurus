// SPDX-License-Identifier: GPL-2.0-only
/*
 * Bounded hardware diagnostics for the legacy PXA2xx FICP platform device.
 *
 * Linux no longer carries its IrDA network stack.  This driver intentionally
 * stops below that layer: it proves the board power GPIO, PXA pinmux, clocks,
 * and SIR selection register and leaves framing to a future protocol port.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/platform_data/irda-pxaficp.h>

#define FICP_ICCR0		0x00
#define FICP_ICCR1		0x04
#define FICP_ICCR2		0x08
#define FICP_ICSR0		0x14
#define FICP_ICSR1		0x18

#define STUART_IRSEL		0x20
#define STISR_RXPL		BIT(4)
#define STISR_TXPL		BIT(3)
#define STISR_XMODE		BIT(2)
#define STISR_RCVEIR		BIT(1)
#define STISR_XMITIR		BIT(0)

#define STISR_SIR_RX		(STISR_RXPL | STISR_XMODE | STISR_RCVEIR)
#define STISR_SIR_TX		(STISR_RXPL | STISR_XMODE | STISR_XMITIR)

enum pxa2xx_ficp_diag_mode {
	PXA2XX_FICP_OFF,
	PXA2XX_FICP_SIR_RX,
	PXA2XX_FICP_SIR_TX,
};

struct pxa2xx_ficp_diag {
	struct device *dev;
	struct pxaficp_platform_data *pdata;
	struct clk *fir_clk;
	struct clk *sir_clk;
	void __iomem *ficp;
	void __iomem *stuart;
	struct mutex lock;
	enum pxa2xx_ficp_diag_mode mode;
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

static void pxa2xx_ficp_board_mode(struct pxa2xx_ficp_diag *diag, int mode)
{
	if (diag->pdata->transceiver_mode) {
		diag->pdata->transceiver_mode(diag->dev, mode);
		return;
	}

	if (gpio_is_valid(diag->pdata->gpio_pwdown))
		gpio_set_value(diag->pdata->gpio_pwdown,
			       !(mode & IR_OFF) ^
			       !diag->pdata->gpio_pwdown_inverted);
	pxa2xx_transceiver_mode(diag->dev, mode);
}

static int pxa2xx_ficp_set_mode(struct pxa2xx_ficp_diag *diag,
				enum pxa2xx_ficp_diag_mode mode)
{
	int ret;

	if (mode != PXA2XX_FICP_OFF &&
	    !(diag->pdata->transceiver_cap & IR_SIRMODE))
		return -EOPNOTSUPP;

	if (mode == diag->mode)
		return 0;

	if (mode == PXA2XX_FICP_OFF) {
		writel_relaxed(0, diag->stuart + STUART_IRSEL);
		pxa2xx_ficp_board_mode(diag, IR_OFF);
		if (diag->sir_clock_enabled) {
			clk_disable_unprepare(diag->sir_clk);
			diag->sir_clock_enabled = false;
		}
		diag->mode = mode;
		return 0;
	}

	if (!diag->sir_clock_enabled) {
		ret = clk_prepare_enable(diag->sir_clk);
		if (ret)
			return ret;
		diag->sir_clock_enabled = true;
	}

	pxa2xx_ficp_board_mode(diag, IR_SIRMODE);
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
	if (gpio_is_valid(diag->pdata->gpio_pwdown))
		gpio = gpio_get_value(diag->pdata->gpio_pwdown);
	mutex_unlock(&diag->lock);

	return sysfs_emit(buf,
			  "mode=%s caps=0x%x gpio_pwdown=%d gpio_value=%d "
			  "sir_clock=%u stisr=0x%08x "
			  "reset_iccr0=0x%08x reset_iccr1=0x%08x "
			  "reset_iccr2=0x%08x reset_icsr0=0x%08x "
			  "reset_icsr1=0x%08x protocol_stack=absent\n",
			  name, diag->pdata->transceiver_cap,
			  diag->pdata->gpio_pwdown, gpio,
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
	struct pxaficp_platform_data *pdata = dev_get_platdata(&pdev->dev);
	struct pxa2xx_ficp_diag *diag;
	struct resource *resource;
	unsigned long gpio_flags;
	int ret;

	if (!pdata)
		return -EINVAL;

	diag = devm_kzalloc(&pdev->dev, sizeof(*diag), GFP_KERNEL);
	if (!diag)
		return -ENOMEM;

	diag->dev = &pdev->dev;
	diag->pdata = pdata;
	mutex_init(&diag->lock);

	resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	diag->ficp = devm_ioremap_resource(&pdev->dev, resource);
	if (IS_ERR(diag->ficp))
		return PTR_ERR(diag->ficp);

	/*
	 * STUART is already owned by the serial driver on these board files.
	 * Map only its IrDA-selection register without claiming the region;
	 * mode changes are serialized by userspace and never touch UART data,
	 * baud, FIFO, or interrupt registers.
	 */
	resource = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	diag->stuart = devm_ioremap(&pdev->dev, resource->start,
				    resource_size(resource));
	if (!diag->stuart)
		return -ENOMEM;

	diag->fir_clk = devm_clk_get(&pdev->dev, "FICPCLK");
	if (IS_ERR(diag->fir_clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(diag->fir_clk),
				     "could not get FICP clock\n");
	diag->sir_clk = devm_clk_get(&pdev->dev, "UARTCLK");
	if (IS_ERR(diag->sir_clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(diag->sir_clk),
				     "could not get STUART clock\n");

	if (gpio_is_valid(pdata->gpio_pwdown)) {
		gpio_flags = pdata->gpio_pwdown_inverted ?
			     GPIOF_OUT_INIT_LOW : GPIOF_OUT_INIT_HIGH;
		ret = devm_gpio_request_one(&pdev->dev, pdata->gpio_pwdown,
					    gpio_flags, "ficp-powerdown");
		if (ret)
			return dev_err_probe(&pdev->dev, ret,
					     "could not request power GPIO\n");
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

	pxa2xx_ficp_board_mode(diag, IR_OFF);
	writel_relaxed(0, diag->stuart + STUART_IRSEL);
	platform_set_drvdata(pdev, diag);

	dev_info(&pdev->dev,
		 "diagnostic ready; full IrDA protocol stack is not present\n");
	return 0;
}

static int pxa2xx_ficp_diag_remove(struct platform_device *pdev)
{
	struct pxa2xx_ficp_diag *diag = platform_get_drvdata(pdev);

	mutex_lock(&diag->lock);
	pxa2xx_ficp_set_mode(diag, PXA2XX_FICP_OFF);
	mutex_unlock(&diag->lock);
	return 0;
}

static struct platform_driver pxa2xx_ficp_diag_driver = {
	.probe = pxa2xx_ficp_diag_probe,
	.remove = pxa2xx_ficp_diag_remove,
	.driver = {
		.name = "pxa2xx-ir",
		.dev_groups = pxa2xx_ficp_diag_groups,
	},
};
module_platform_driver(pxa2xx_ficp_diag_driver);

MODULE_AUTHOR("Zaurus SD bring-up");
MODULE_DESCRIPTION("PXA2xx FICP hardware diagnostic interface");
MODULE_LICENSE("GPL");
