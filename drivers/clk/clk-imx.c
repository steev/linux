#include <linux/clk-provider.h>
#include <linux/of.h>

#include "clk-imx.h"

static const __initconst struct of_device_id clk_match[] = {
	{ .compatible = "fixed-clock", .data = of_fixed_clk_setup, },
        { .compatible = "fsl,imx-pllv2", .data = imx_clk_pllv2, },
        {}
};

void __init imx5_clocks_init(void)
{
        of_clk_init(clk_match);
}

