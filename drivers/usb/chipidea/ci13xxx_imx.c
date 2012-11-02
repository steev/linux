/*
 * Copyright 2012 Freescale Semiconductor, Inc.
 * Copyright (C) 2012 Marek Vasut <marex@denx.de>
 * on behalf of DENX Software Engineering GmbH
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/dma-mapping.h>
#include <linux/usb/chipidea.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#include "ci.h"
#include "ci13xxx_imx.h"

#define pdev_to_phy(pdev) \
	((struct usb_phy *)platform_get_drvdata(pdev))
#define IOMUXC_IOMUXC_GPR1			0x00000004
#define USB_OTG_ID_SEL_BIT			(1<<13)

struct ci13xxx_imx_data {
	struct device_node *phy_np;
	struct usb_phy *phy;
	struct platform_device *ci_pdev;
	struct clk *clk;
	struct regulator *reg_vbus;
};

static const struct usbmisc_ops *usbmisc_ops;

/* Common functions shared by usbmisc drivers */

int usbmisc_set_ops(const struct usbmisc_ops *ops)
{
	if (usbmisc_ops)
		return -EBUSY;

	usbmisc_ops = ops;

	return 0;
}
EXPORT_SYMBOL_GPL(usbmisc_set_ops);

void usbmisc_unset_ops(const struct usbmisc_ops *ops)
{
	usbmisc_ops = NULL;
}
EXPORT_SYMBOL_GPL(usbmisc_unset_ops);

int usbmisc_get_init_data(struct device *dev, struct usbmisc_usb_device *usbdev)
{
	struct device_node *np = dev->of_node;
	struct of_phandle_args args;
	int ret;

	usbdev->dev = dev;

	ret = of_parse_phandle_with_args(np, "fsl,usbmisc", "#index-cells",
					0, &args);
	if (ret) {
		dev_err(dev, "Failed to parse property fsl,usbmisc, errno %d\n",
			ret);
		memset(usbdev, 0, sizeof(*usbdev));
		return ret;
	}
	usbdev->index = args.args[0];
	of_node_put(args.np);

	if (of_find_property(np, "disable-over-current", NULL))
		usbdev->disable_oc = 1;

	return 0;
}
EXPORT_SYMBOL_GPL(usbmisc_get_init_data);

/* End of common functions shared by usbmisc drivers*/

static struct ci13xxx_platform_data ci13xxx_imx_platdata __devinitdata  = {
	.name			= "ci13xxx_imx",
	.flags			= CI13XXX_REQUIRE_TRANSCEIVER |
				  CI13XXX_DISABLE_STREAMING |
				  CI13XXX_REGS_SHARED,
	.capoffset		= DEF_CAPOFFSET,
};

static int ci13xxx_otg_set_vbus(struct usb_otg *otg, bool enabled)
{
	struct ci13xxx	*ci = container_of(otg, struct ci13xxx, otg);
	struct regulator *reg_vbus = ci->reg_vbus;
	int ret;

	WARN_ON(!reg_vbus);

	if (reg_vbus) {
		if (enabled) {
			ret = regulator_enable(reg_vbus);
			if (ret) {
				dev_err(ci->dev,
				"Failed to enable vbus regulator, ret=%d\n",
				ret);
				return ret;
			}
		} else {
			ret = regulator_disable(reg_vbus);
			if (ret) {
				dev_err(ci->dev,
				"Failed to disable vbus regulator, ret=%d\n",
				ret);
				return ret;
			}
		}
	}

	return 0;
}

static int __devinit ci13xxx_imx_probe(struct platform_device *pdev)
{
	struct ci13xxx_imx_data *data;
	struct platform_device *plat_ci, *phy_pdev;
	struct ci13xxx	*ci;
	struct device_node *phy_np;
	struct resource *res;
	struct regulator *reg_vbus;
	struct pinctrl *pinctrl;
	struct regmap *iomuxc_gpr;
	u32 gpr1 = 0;
	int ret;

	if (of_find_property(pdev->dev.of_node, "fsl,usbmisc", NULL)
		&& !usbmisc_ops)
		return -EPROBE_DEFER;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		dev_err(&pdev->dev, "Failed to allocate CI13xxx-IMX data!\n");
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Can't get device resources!\n");
		return -ENOENT;
	}

	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(pinctrl))
		dev_warn(&pdev->dev, "pinctrl get/select failed, err=%ld\n",
			PTR_ERR(pinctrl));

	data->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(data->clk)) {
		dev_err(&pdev->dev,
			"Failed to get clock, err=%ld\n", PTR_ERR(data->clk));
		return PTR_ERR(data->clk);
	}

	ret = clk_prepare_enable(data->clk);
	if (ret) {
		dev_err(&pdev->dev,
			"Failed to prepare or enable clock, err=%d\n", ret);
		return ret;
	}

	phy_np = of_parse_phandle(pdev->dev.of_node, "fsl,usbphy", 0);
	if (phy_np) {
		data->phy_np = phy_np;
		phy_pdev = of_find_device_by_node(phy_np);
		if (phy_pdev) {
			struct usb_phy *phy;
			phy = pdev_to_phy(phy_pdev);
			if (phy &&
			    try_module_get(phy_pdev->dev.driver->owner)) {
				usb_phy_init(phy);
				data->phy = phy;
			}
		}
	}

	reg_vbus = devm_regulator_get(&pdev->dev, "vbus");
	if (!IS_ERR(reg_vbus))
		data->reg_vbus = reg_vbus;
	else
		reg_vbus = NULL;

	ci13xxx_imx_platdata.phy = data->phy;

	if (!pdev->dev.dma_mask) {
		pdev->dev.dma_mask = devm_kzalloc(&pdev->dev,
				      sizeof(*pdev->dev.dma_mask), GFP_KERNEL);
		if (!pdev->dev.dma_mask) {
			ret = -ENOMEM;
			dev_err(&pdev->dev, "Failed to alloc dma_mask!\n");
			goto put_np;
		}
		*pdev->dev.dma_mask = DMA_BIT_MASK(32);
		dma_set_coherent_mask(&pdev->dev, *pdev->dev.dma_mask);
	}

	if (usbmisc_ops && usbmisc_ops->init) {
		ret = usbmisc_ops->init(&pdev->dev);
		if (ret) {
			dev_err(&pdev->dev,
				"usbmisc init failed, ret=%d\n", ret);
			goto put_np;
		}
	}

	plat_ci = ci13xxx_add_device(&pdev->dev,
				pdev->resource, pdev->num_resources,
				&ci13xxx_imx_platdata);
	if (IS_ERR(plat_ci)) {
		ret = PTR_ERR(plat_ci);
		dev_err(&pdev->dev,
			"Can't register ci_hdrc platform device, err=%d\n",
			ret);
		goto put_np;
	}

	data->ci_pdev = plat_ci;
	platform_set_drvdata(pdev, data);

	ci = platform_get_drvdata(plat_ci);
	/*
	 * Internal vbus on/off policy
	 * - Always on for host only function
	 * - Always off for gadget only function
	 * - call otg.set_vbus to control on/off according usb role
	 */

	if (ci->roles[CI_ROLE_HOST] && !ci->roles[CI_ROLE_GADGET]
			&& reg_vbus) {
		ret = regulator_enable(reg_vbus);
		if (ret) {
			dev_err(&pdev->dev,
				"Failed to enable vbus regulator, ret=%d\n",
				ret);
			goto put_np;
		}
	} else if (ci->is_otg) {
		ci->otg.set_vbus = ci13xxx_otg_set_vbus;
		ci->reg_vbus = data->reg_vbus;
	}

	if (ci->is_otg) {
		iomuxc_gpr = syscon_regmap_lookup_by_compatible
			("fsl,imx6q-iomuxc-gpr");
		if (!IS_ERR(iomuxc_gpr)) {
			/* Select USB ID pin at iomuxc grp1 */
			regmap_read(iomuxc_gpr, IOMUXC_IOMUXC_GPR1, &gpr1);
			regmap_write(iomuxc_gpr, IOMUXC_IOMUXC_GPR1,
					gpr1 | USB_OTG_ID_SEL_BIT);
		} else {
			/* only imx6 platform has gpr, do not return error */
			dev_warn(&pdev->dev,
				"failed to find imx6q-iomuxc-gpr regmap:%ld\n",
				PTR_ERR(iomuxc_gpr));
		}
	}

	pm_runtime_no_callbacks(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	return 0;

put_np:
	if (phy_np)
		of_node_put(phy_np);
	clk_disable_unprepare(data->clk);

	return ret;
}

static int __devexit ci13xxx_imx_remove(struct platform_device *pdev)
{
	struct ci13xxx_imx_data *data = platform_get_drvdata(pdev);

	pm_runtime_disable(&pdev->dev);
	ci13xxx_remove_device(data->ci_pdev);

	if (data->reg_vbus)
		regulator_disable(data->reg_vbus);

	if (data->phy) {
		usb_phy_shutdown(data->phy);
		module_put(data->phy->dev->driver->owner);
	}

	of_node_put(data->phy_np);

	clk_disable_unprepare(data->clk);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static const struct of_device_id ci13xxx_imx_dt_ids[] = {
	{ .compatible = "fsl,imx27-usb", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ci13xxx_imx_dt_ids);

static struct platform_driver ci13xxx_imx_driver = {
	.probe = ci13xxx_imx_probe,
	.remove = __devexit_p(ci13xxx_imx_remove),
	.driver = {
		.name = "imx_usb",
		.owner = THIS_MODULE,
		.of_match_table = ci13xxx_imx_dt_ids,
	 },
};

module_platform_driver(ci13xxx_imx_driver);

MODULE_ALIAS("platform:imx-usb");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("CI13xxx i.MX USB binding");
MODULE_AUTHOR("Marek Vasut <marex@denx.de>");
MODULE_AUTHOR("Richard Zhao <richard.zhao@freescale.com>");
