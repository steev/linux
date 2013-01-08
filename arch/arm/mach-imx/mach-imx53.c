/*
 * Copyright 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2011 Linaro Ltd.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "common.h"
#include "mx53.h"

static void __init imx53_qsb_init(void)
{
	struct clk *clk;

	clk = clk_get_sys(NULL, "ssi_ext1");
	if (IS_ERR(clk)) {
		pr_err("failed to get clk ssi_ext1\n");
		return;
	}

	clk_register_clkdev(clk, NULL, "0-000a");
}

static void __init imx53_clocks_fixup(void)
{
	struct clk *pll2, *usboh3_per_gate;

	pll2 = __clk_lookup("pll2_sw");

	/* Set SDHC parents to be PLL2 */
	clk_set_parent(__clk_lookup("esdhc_a_sel"), pll2);
	clk_set_parent(__clk_lookup("esdhc_b_sel"), pll2);

	/* set SDHC root clock to 200MHZ*/
	clk_set_rate(__clk_lookup("esdhc_a_podf"), 200000000);
	clk_set_rate(__clk_lookup("esdhc_b_podf"), 200000000);

	usboh3_per_gate = __clk_lookup("usboh3_per_gate");
	//r = clk_round_rate(usboh3_per_gate, 54000000);
	clk_set_rate(usboh3_per_gate, 54000000);

	/* move usb phy clk to 24MHz */
	clk_set_parent(__clk_lookup("usb_phy_sel"), __clk_lookup("osc"));
}

static void __init imx53_dt_init(void)
{
	if (of_machine_is_compatible("fsl,imx53-qsb"))
		imx53_qsb_init();

	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
}

static void __init imx53_timer_init(void)
{
	struct clk *iim;

	/* this is finally calling drivers/clk/clk-imx.c */
	imx5_clocks_init();

	imx53_clocks_fixup();
	imx_gpt_register();
	imx_epit_register();

	iim = __clk_lookup("iim_gate");
	if (iim) {
		clk_prepare_enable(iim);
		imx_print_silicon_rev("i.MX53", mx53_revision());
		clk_disable_unprepare(iim);
	}
}

static struct sys_timer imx53_timer = {
	.init = imx53_timer_init,
};

static const char *imx53_dt_board_compat[] __initdata = {
	"fsl,imx53",
	NULL
};

DT_MACHINE_START(IMX53_DT, "Freescale i.MX53 (Device Tree Support)")
	.map_io		= mx53_map_io,
	.init_early	= imx53_init_early,
	.init_irq	= mx53_init_irq,
	.handle_irq	= imx53_handle_irq,
	.timer		= &imx53_timer,
	.init_machine	= imx53_dt_init,
	.init_late	= imx53_init_late,
	.dt_compat	= imx53_dt_board_compat,
	.restart	= mxc_restart,
MACHINE_END
