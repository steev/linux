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
#include <linux/clk-provider.h>
#include <linux/irq.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "common.h"
#include "mx51.h"

#define GPIO_BACKLIGHT_POWER 108
#define GPIO_LVDS_POWER 71
#define GPIO_LVDS_RESET 69
#define GPIO_LVDS_ENABLE 76
#define GPIO_LCD_ENABLE 77

static struct gpio efika_gpio_lvds[] __initdata = {
	{ GPIO_BACKLIGHT_POWER, GPIOF_OUT_INIT_LOW, "backlight power" },
	{ GPIO_LVDS_POWER, GPIOF_OUT_INIT_HIGH, "lvds power" },
	{ GPIO_LVDS_RESET, GPIOF_OUT_INIT_LOW, "lvds reset" },
	{ GPIO_LVDS_ENABLE, GPIOF_OUT_INIT_LOW, "lvds enable" },
	{ GPIO_LCD_ENABLE, GPIOF_OUT_INIT_LOW, "lcd enable" },
};

static int __init efikasb_lvds_init(void)
{
	if (!of_machine_is_compatible("genesi,imx51-efikasb"))
		return 0;

	gpio_request_array(efika_gpio_lvds, ARRAY_SIZE(efika_gpio_lvds));

	mdelay(5);
	gpio_direction_output(GPIO_LVDS_RESET, 0);

	mdelay(5);

	gpio_direction_output(GPIO_LVDS_ENABLE, 1);
	gpio_direction_output(GPIO_BACKLIGHT_POWER, 0);
	gpio_direction_output(GPIO_LCD_ENABLE, 1);

	return 0;
}
device_initcall(efikasb_lvds_init);

static void __init imx51_dt_init(void)
{
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
}

static void __init imx51_clocks_fixup(void)
{
	struct clk *pll2;

	pll2 = __clk_lookup("pll2_sw");

	/* Set SDHC parents to be PLL2 */
	clk_set_parent(__clk_lookup("esdhc_a_sel"), pll2);
	clk_set_parent(__clk_lookup("esdhc_b_sel"), pll2);

	/* set SDHC root clock to 166.25MHZ*/
	clk_set_rate(__clk_lookup("esdhc_a_podf"), 166250000);
	clk_set_rate(__clk_lookup("esdhc_b_podf"), 166250000);

	/* set the usboh3 parent to pll2_sw */
	clk_set_parent(__clk_lookup("usboh3_sel"), pll2);

	/* set the usboh3 clock to 66.5MHz */
	clk_set_rate(__clk_lookup("usboh3_per_gate"), 665000000);

	/* move usb phy clk to 24MHz */
	clk_set_parent(__clk_lookup("usb_phy_sel"), __clk_lookup("osc"));

	/* ipu di external clock to pll3_sw */
	clk_set_parent(__clk_lookup("ipu_di0_sel"), __clk_lookup("di_pred"));
}

static void __init imx51_timer_init(void)
{
	struct clk *iim;

	/* this is finally calling drivers/clk/clk-imx.c */
	imx5_clocks_init();

	imx51_clocks_fixup();
	imx_gpt_register();
	imx_epit_register();

	iim = __clk_lookup("iim_gate");
	if (iim) {
		clk_prepare_enable(iim);
		imx_print_silicon_rev("i.MX51", mx51_revision());
		clk_disable_unprepare(iim);
	}
}

static struct sys_timer imx51_timer = {
	.init = imx51_timer_init,
};

static const char *imx51_dt_board_compat[] __initdata = {
	"fsl,imx51",
	NULL
};

DT_MACHINE_START(IMX51_DT, "Freescale i.MX51 (Device Tree Support)")
	.map_io		= mx51_map_io,
	.init_early	= imx51_init_early,
	.init_irq	= mx51_init_irq,
	.handle_irq	= imx51_handle_irq,
	.timer		= &imx51_timer,
	.init_machine	= imx51_dt_init,
	.init_late	= imx51_init_late,
	.dt_compat	= imx51_dt_board_compat,
	.restart	= mxc_restart,
MACHINE_END
