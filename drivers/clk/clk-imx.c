#include <linux/clk-provider.h>
#include <linux/of.h>

#include "clk-imx.h"

static const __initconst struct of_device_id root_clk_match[] = {
	{ .compatible = "fixed-clock", .data = of_fixed_clk_setup, },
	{ .compatible = "fsl,imx-pllv2", .data = imx_clk_pllv2, },
	{ },
};

static const __initconst struct of_device_id ccm_clk_match[] = {
	{ .compatible = "fsl,imx51-ccm", .data = imx51_ccm_setup, },
	{ .compatible = "fsl,imx53-ccm", .data = imx53_ccm_setup, },
	{ },
};

void __init imx5_clocks_init(void)
{
	/*
	 * We have to do this in two steps because CCM setup relies on
	 * the PLL and OSC clocks to be present in the clock tree
	 * and if we just match based on for_each_matching_node
	 * there ends up being a race condition..
	 */
	of_clk_init(root_clk_match);
	of_clk_init(ccm_clk_match);
}

