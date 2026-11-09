// SPDX-License-Identifier: GPL-2.0-only
/* Clocks shared by the H3970 ASIC2 ADC and ASIC3 SD interface. */
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

enum { EX1, ADC, SD, NUM_CLOCKS };

struct asic2_clk {
	struct clk_hw hw;
	void __iomem *reg;
	u32 mask;
	u32 value;
	u32 saved;
	unsigned int divisor;
};

static struct asic2_clk *to_asic2_clk(struct clk_hw *hw)
{
	return container_of(hw, struct asic2_clk, hw);
}

/* The clock core serializes enable and disable for this shared register. */
static int asic2_clk_enable(struct clk_hw *hw)
{
	struct asic2_clk *clk = to_asic2_clk(hw);
	u32 value = readl(clk->reg);

	clk->saved = value & clk->mask;
	writel((value & ~clk->mask) | clk->value, clk->reg);
	return 0;
}

static void asic2_clk_disable(struct clk_hw *hw)
{
	struct asic2_clk *clk = to_asic2_clk(hw);

	/* Preserve firmware state for consumers Linux does not manage yet. */
	writel((readl(clk->reg) & ~clk->mask) | clk->saved, clk->reg);
}

static int asic2_clk_is_enabled(struct clk_hw *hw)
{
	struct asic2_clk *clk = to_asic2_clk(hw);

	return (readl(clk->reg) & clk->mask) == clk->value;
}

static unsigned long asic2_clk_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct asic2_clk *clk = to_asic2_clk(hw);

	return clk->divisor ? parent_rate / clk->divisor : 24576000;
}

static const struct clk_ops asic2_clk_ops = {
	.enable = asic2_clk_enable,
	.disable = asic2_clk_disable,
	.is_enabled = asic2_clk_is_enabled,
	.recalc_rate = asic2_clk_rate,
};

static int asic2_clk_probe(struct platform_device *pdev)
{
	static const struct {
		const char *name;
		u32 mask;
		u32 value;
		unsigned int divisor;
	} desc[NUM_CLOCKS] = {
		[EX1] = { "h3970-asic2-ex1", BIT(14), BIT(14), 0 },
		[ADC] = { "h3970-asic2-adc", BIT(4), BIT(4), 24 },
		[SD] = { "h3970-asic2-sd", GENMASK(11, 10), BIT(11), 1 },
	};
	struct device *dev = &pdev->dev;
	struct clk_hw_onecell_data *data;
	struct asic2_clk *clks;
	void __iomem *reg;
	unsigned int i;
	int ret;

	reg = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(reg))
		return PTR_ERR(reg);

	clks = devm_kcalloc(dev, NUM_CLOCKS, sizeof(*clks), GFP_KERNEL);
	data = devm_kzalloc(dev, struct_size(data, hws, NUM_CLOCKS), GFP_KERNEL);
	if (!clks || !data)
		return -ENOMEM;

	data->num = NUM_CLOCKS;
	for (i = 0; i < NUM_CLOCKS; i++) {
		const struct clk_hw *parent = &clks[EX1].hw;
		const struct clk_init_data init = {
			.name = desc[i].name,
			.ops = &asic2_clk_ops,
			.parent_hws = i == EX1 ? NULL : &parent,
			.num_parents = i != EX1,
			/* Only restore the gate on a matched consumer disable. */
			.flags = CLK_IGNORE_UNUSED,
		};

		clks[i].reg = reg;
		clks[i].mask = desc[i].mask;
		clks[i].value = desc[i].value;
		clks[i].divisor = desc[i].divisor;
		clks[i].hw.init = &init;
		ret = devm_clk_hw_register(dev, &clks[i].hw);
		if (ret)
			return ret;
		data->hws[i] = &clks[i].hw;
	}

	return devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, data);
}

static const struct of_device_id asic2_clk_match[] = {
	{ .compatible = "compaq,h3970-asic2-clocks" },
	{}
};

static struct platform_driver asic2_clk_driver = {
	.probe = asic2_clk_probe,
	.driver = {
		.name = "h3970-asic2-clocks",
		.of_match_table = asic2_clk_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(asic2_clk_driver);
