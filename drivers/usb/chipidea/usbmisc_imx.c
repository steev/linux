/*
 * Copyright 2012 Freescale Semiconductor, Inc.
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
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>

#include "ci13xxx_imx.h"

#define USB_DEV_MAX 4

#define MX25_USB_PHY_CTRL_OFFSET	0x08
#define MX25_EXTERNAL_VBUS_DIVIDER	BIT(23)

#define MX51_USB_CTRL_OFFSET		0x00	// H1PM, OPM MX51
#define MX51_USB_CTRL_H1PM		BIT(8)
#define MX51_USB_CTRL_OPM		BIT(24)

#define MX51_USB_PHY_CTRL_0_OFFSET	0x08
#define MX51_OVER_CUR_DIS_UH1		BIT(5)
#define MX51_OVER_CUR_DIS_OTG		BIT(8)

#define MX51_USB_PHY_CTRL_1_OFFSET	0x0C
#define MX51_USB_PHY_CTRL_1_PLLDIVMASK	0x03

struct phy_clk_table {
	unsigned int rate;
	unsigned int div;
};

static struct phy_clk_table phy_pll_div_rates[] = {
	{ .rate = 19200000, .div = 0x00, },
	{ .rate = 24000000, .div = 0x01, },
	{ .rate = 26000000, .div = 0x10, },
	{ .rate = 27000000, .div = 0x11, },
};

#define MX51_USB_CTRL_1_OFFSET		0x10	// MX51 - manual screwed, it's +0x810
#define MX51_USB_CTRL_1_EXTCLK_OTG	BIT(23)
#define MX51_USB_CTRL_1_EXTCLK_UH1	BIT(25)
#define MX51_USB_CTRL_1_EXTCLK_UH2	BIT(26)
#define MX51_USB_CTRL_1_EXTCLK_UH3	BIT(27)

#define MX51_USB_UH2_CTRL_OFFSET	0x14
#define MX51_USB_UH2_CTRL_H2PM		BIT(4)
#define MX53_OVER_CUR_DIS_UH2		BIT(30)

#define MX51_USB_UH3_CTRL_OFFSET	0x18
#define MX51_USB_UH3_CTRL_H3PM		BIT(4)
#define MX53_OVER_CUR_DIS_UH3		BIT(30)

#define MX6_OVER_CUR_DIS		BIT(7)

struct imx_usbmisc {
	void __iomem *base;
	spinlock_t lock;
	struct clk *clk_ahb;
	struct clk *clk_ipg;
	struct clk *clk_per;
	struct clk *clk_phy;
	struct usbmisc_usb_device usbdev[USB_DEV_MAX];
	const struct usbmisc_ops *ops;
};

static struct imx_usbmisc *usbmisc;

static struct usbmisc_usb_device *get_usbdev(struct device *dev)
{
	int i, ret;

	for (i = 0; i < USB_DEV_MAX; i++) {
		if (usbmisc->usbdev[i].dev == dev)
			return &usbmisc->usbdev[i];
		else if (!usbmisc->usbdev[i].dev)
			break;
	}

	if (i >= USB_DEV_MAX)
		return ERR_PTR(-EBUSY);

	ret = usbmisc_get_init_data(dev, &usbmisc->usbdev[i]);
	if (ret)
		return ERR_PTR(ret);

	return &usbmisc->usbdev[i];
}

static int usbmisc_imx25_post(struct device *dev)
{
	struct usbmisc_usb_device *usbdev;
	void __iomem *reg;
	unsigned long flags;
	u32 val;

	usbdev = get_usbdev(dev);
	if (IS_ERR(usbdev))
		return PTR_ERR(usbdev);

	reg = usbmisc->base + MX25_USB_PHY_CTRL_OFFSET;

	if (usbdev->evdo) {
		spin_lock_irqsave(&usbmisc->lock, flags);
		val = readl(reg);
		writel(val | MX25_EXTERNAL_VBUS_DIVIDER, reg);
		spin_unlock_irqrestore(&usbmisc->lock, flags);
		usleep_range(5000,10000); /* needed to stabilize voltage */
	}

	return 0;
}

static int usbmisc_imx51_init(struct device *dev)
{
	struct usbmisc_usb_device *usbdev;
	void __iomem *reg = NULL;
	unsigned long flags;
	unsigned long rate;
	int div = -1;
	u32 val = 0;

	dev_err(dev, "%s\n", __func__);

	usbdev = get_usbdev(dev);
	if (IS_ERR(usbdev))
		return PTR_ERR(usbdev);

	spin_lock_irqsave(&usbmisc->lock, flags);

	switch (usbdev->index) {
	case 0:
		WARN_ON(!usbmisc->clk_phy);

		rate = clk_get_rate(usbmisc->clk_phy);
		if (rate) {
			int i;

			for (i = 0; i < ARRAY_SIZE(phy_pll_div_rates); i++) {
				if (rate == phy_pll_div_rates[i].rate)
					div = phy_pll_div_rates[i].div;
			}
		}

		if (div == -1) {
			dev_err(dev, "%s: PHY clock rate %ld doesn't match any valid rate!\n",
					__func__, rate);
		} else {
			dev_err(dev, "%s: UTMI PHY clock %ld\n", __func__, rate);

			val = readl(usbmisc->base + MX51_USB_PHY_CTRL_1_OFFSET);
			dev_err(dev, "%s: PHY_CTRL_1 0x%08x\n", __func__, val);

			val &= ~MX51_USB_PHY_CTRL_1_PLLDIVMASK;
			dev_err(dev, "%s: PHY_CTRL_1 mask PLLDIV 0x%08x\n", __func__, val);

			val |= div;
			dev_err(dev, "%s: PHY_CTRL_1 PLLDIV %ldHz 0x%08x\n", __func__, rate, val);

			writel(val, usbmisc->base + MX51_USB_PHY_CTRL_1_OFFSET);
			mdelay(10);
		}

		if (usbdev->disable_oc) {
			dev_err(dev, "%s: oc disable otg\n", __func__);
			reg = usbmisc->base + MX51_USB_PHY_CTRL_0_OFFSET;
			val = readl(reg);

			dev_err(dev, "%s: PHY_CTRL_0 0x%08x\n", __func__, val);

			val |= MX51_OVER_CUR_DIS_OTG;

			dev_err(dev, "%s: PHY_CTRL_0 | OC_DIS_OTG 0x%08x\n", __func__, val);

			writel(val, reg);
			mdelay(10);

			reg = usbmisc->base + MX51_USB_CTRL_OFFSET;
			val = readl(reg);

			dev_err(dev, "%s: USB_CTRL 0x%08x\n", __func__, val);

			val |= MX51_USB_CTRL_OPM;

			dev_err(dev, "%s: USB_CTRL | OPM 0x%08x\n", __func__, val);

			writel(val, reg);
			mdelay(10);
		}

		reg = usbmisc->base + MX51_USB_CTRL_1_OFFSET;
		val = readl(reg);

		dev_err(dev, "%s: USB_CTRL_1 0x%08x\n", __func__, val);

		val &= ~MX51_USB_CTRL_1_EXTCLK_OTG;

		dev_err(dev, "%s: USB_CTRL_1 | EXTCLK_OTG 0x%08x\n", __func__, val);

		writel(val, reg);
		mdelay(10);

		break;
	case 1:
		if (usbdev->disable_oc) {
			dev_err(dev, "%s: oc disable h1\n", __func__);
			reg = usbmisc->base + MX51_USB_CTRL_OFFSET;
			val = readl(reg);

			dev_err(dev, "%s: USB_CTRL 0x%08x\n", __func__, val);

			val &= ~MX51_USB_CTRL_H1PM;

			dev_err(dev, "%s: USB_CTRL mask out H1PM 0x%08x\n", __func__, val);

			writel(val, reg);
			mdelay(10);

			reg = usbmisc->base + MX51_USB_PHY_CTRL_0_OFFSET;
			val = readl(reg);

			dev_err(dev, "%s: USB_PHY_CTRL_0 0x%08x\n", __func__, val);

			val |= MX51_OVER_CUR_DIS_UH1;

			dev_err(dev, "%s: USB_PHY_CTRL_0 | OC_DIS_UH1 0x%08x\n", __func__, val);

			writel(val, reg);
			mdelay(10);
		}

		reg = usbmisc->base + MX51_USB_CTRL_1_OFFSET;
		val = readl(reg);

		dev_err(dev, "%s: USB_CTRL_1 0x%08x\n", __func__, val);

		val |= MX51_USB_CTRL_1_EXTCLK_UH1;

		dev_err(dev, "%s: USB_CTRL_1 | EXTCLK_UH1 0x%08x\n", __func__, val);

		writel(val, reg);
		mdelay(10);

		break;
	case 2:
		if (usbdev->disable_oc) {
			dev_err(dev, "%s: oc disable h2\n", __func__);
			reg = usbmisc->base + MX51_USB_UH2_CTRL_OFFSET;
			val = readl(reg) & ~MX51_USB_UH2_CTRL_H2PM;

			writel(val, reg);
			mdelay(10);
		}

//		reg = usbmisc->base + MX51_USB_CTRL_1_OFFSET;
//		val = readl(reg);
//		writel((val | MX51_USB_CTRL_1_EXTCLK_H2), reg);
//		mdelay(10);

		break;
	case 3: /* is there even a H3 out on MX51? */
		if (usbdev->disable_oc) {
			dev_err(dev, "%s: oc disable h3\n", __func__);
			reg = usbmisc->base + MX51_USB_UH3_CTRL_OFFSET;
			val = readl(reg) | MX51_USB_UH3_CTRL_H3PM;

			writel(val, reg);
			mdelay(10);
		}
		break;
	}

	spin_unlock_irqrestore(&usbmisc->lock, flags);

	return 0;
}

static int usbmisc_imx53_init(struct device *dev)
{
	struct usbmisc_usb_device *usbdev;
	void __iomem *reg = NULL;
	unsigned long flags;
	u32 val = 0;

	usbdev = get_usbdev(dev);
	if (IS_ERR(usbdev))
		return PTR_ERR(usbdev);

	if (usbdev->disable_oc) {
		spin_lock_irqsave(&usbmisc->lock, flags);

		switch (usbdev->index) {
		case 0:
			reg = usbmisc->base + MX51_USB_PHY_CTRL_0_OFFSET;
			val = readl(reg) | MX51_OVER_CUR_DIS_OTG;
			break;
		case 1:
			reg = usbmisc->base + MX51_USB_PHY_CTRL_0_OFFSET;
			val = readl(reg) | MX51_OVER_CUR_DIS_UH1;
			break;
		case 2:
			reg = usbmisc->base + MX51_USB_UH2_CTRL_OFFSET;
			val = readl(reg) | MX53_OVER_CUR_DIS_UH2;
			break;
		case 3:
			reg = usbmisc->base + MX51_USB_UH3_CTRL_OFFSET;
			val = readl(reg) | MX53_OVER_CUR_DIS_UH3;
			break;
		}

		writel(val, reg);

		spin_unlock_irqrestore(&usbmisc->lock, flags);
	}

	return 0;
}

static int usbmisc_imx6q_init(struct device *dev)
{

	struct usbmisc_usb_device *usbdev;
	unsigned long flags;
	u32 reg;

	usbdev = get_usbdev(dev);
	if (IS_ERR(usbdev))
		return PTR_ERR(usbdev);

	if (usbdev->disable_oc) {
		spin_lock_irqsave(&usbmisc->lock, flags);
		/* what the crap is this? */
		reg = readl(usbmisc->base + usbdev->index * 4);
		writel(reg | MX6_OVER_CUR_DIS,
			usbmisc->base + usbdev->index * 4);
		spin_unlock_irqrestore(&usbmisc->lock, flags);
	}

	return 0;
}

static const struct usbmisc_ops imx25_usbmisc_ops = {
	.post = usbmisc_imx25_post,
};

static const struct usbmisc_ops imx51_usbmisc_ops = {
	.init = usbmisc_imx51_init,
};

static const struct usbmisc_ops imx53_usbmisc_ops = {
	.init = usbmisc_imx53_init,
};

static const struct usbmisc_ops imx6q_usbmisc_ops = {
	.init = usbmisc_imx6q_init,
};

static const struct of_device_id usbmisc_imx_dt_ids[] = {
	{ .compatible = "fsl,imx25-usbmisc", .data = (void *)&imx25_usbmisc_ops },
	{ .compatible = "fsl,imx51-usbmisc", .data = (void *)&imx51_usbmisc_ops },
	{ .compatible = "fsl,imx53-usbmisc", .data = (void *)&imx53_usbmisc_ops },
	{ .compatible = "fsl,imx6q-usbmisc", .data = (void *)&imx6q_usbmisc_ops },
	{ /* sentinel */ }
};

static int usbmisc_imx_probe(struct platform_device *pdev)
{
	struct resource	*res;
	struct imx_usbmisc *data;
	int ret;
	struct of_device_id *tmp_dev;

	if (usbmisc)
		return -EBUSY;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	spin_lock_init(&data->lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	data->base = devm_request_and_ioremap(&pdev->dev, res);
	if (!data->base)
		return -EADDRNOTAVAIL;

	dev_err(&pdev->dev, "%s\n", __func__);

	data->clk_ahb = devm_clk_get(&pdev->dev, "ahb");
	if (IS_ERR(data->clk_ahb)) {
		dev_err(&pdev->dev,
			"failed to get ahb clock, err=%ld\n", PTR_ERR(data->clk_ahb));
		return PTR_ERR(data->clk_ahb);
	}

	data->clk_ipg = devm_clk_get(&pdev->dev, "ipg");
	if (IS_ERR(data->clk_ipg)) {
		dev_err(&pdev->dev,
			"failed to get ipg clock, err=%ld\n", PTR_ERR(data->clk_ipg));
		return PTR_ERR(data->clk_ipg);
	}

	data->clk_per = devm_clk_get(&pdev->dev, "per");
	if (IS_ERR(data->clk_per)) {
		dev_err(&pdev->dev,
			"failed to get per clock, err=%ld\n", PTR_ERR(data->clk_per));
		return PTR_ERR(data->clk_per);
	}

	data->clk_phy = devm_clk_get(&pdev->dev, "phy");
	if (IS_ERR(data->clk_phy)) {
		dev_dbg(&pdev->dev,
			"failed to get phy clock, err=%ld\n", PTR_ERR(data->clk_phy));
		data->clk_phy = NULL;
	}

	ret = clk_prepare_enable(data->clk_ahb);
	if (ret)
		return ret;

	ret = clk_prepare_enable(data->clk_ipg);
	if (ret)
		goto err_ipg_failed;

	ret = clk_prepare_enable(data->clk_per);
	if (ret)
		goto err_per_failed;

	ret = clk_prepare_enable(data->clk_phy);
	if (ret)
		dev_dbg(&pdev->dev,
			"failed to enable phy clock, err=%d\n", ret);

	tmp_dev = (struct of_device_id *)
		of_match_device(usbmisc_imx_dt_ids, &pdev->dev);
	data->ops = (const struct usbmisc_ops *)tmp_dev->data;
	usbmisc = data;
	ret = usbmisc_set_ops(data->ops);
	if (ret)
		goto err_set_ops_failed;

	dev_err(&pdev->dev, "%s successful\n", __func__);

	return 0;

 err_set_ops_failed:
	usbmisc = NULL;
	clk_disable_unprepare(data->clk_per);
 err_per_failed:
	clk_disable_unprepare(data->clk_ipg);
 err_ipg_failed:
	clk_disable_unprepare(data->clk_ahb);

	if (data->clk_phy)
		clk_disable_unprepare(data->clk_phy);

	return ret;
}

static int usbmisc_imx_remove(struct platform_device *pdev)
{
	usbmisc_unset_ops(usbmisc->ops);

	clk_disable_unprepare(usbmisc->clk_per);
	clk_disable_unprepare(usbmisc->clk_ipg);
	clk_disable_unprepare(usbmisc->clk_ahb);
	clk_disable_unprepare(usbmisc->clk_phy);

	usbmisc = NULL;
	return 0;
}

static struct platform_driver usbmisc_imx_driver = {
	.probe = usbmisc_imx_probe,
	.remove = usbmisc_imx_remove,
	.driver = {
		.name = "usbmisc_imx",
		.owner = THIS_MODULE,
		.of_match_table = usbmisc_imx_dt_ids,
	 },
};

int __init usbmisc_imx_drv_init(void)
{
	return platform_driver_register(&usbmisc_imx_driver);
}
subsys_initcall(usbmisc_imx_drv_init);

void __exit usbmisc_imx_drv_exit(void)
{
	platform_driver_unregister(&usbmisc_imx_driver);
}
module_exit(usbmisc_imx_drv_exit);

MODULE_ALIAS("platform:usbmisc-imx");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("driver for imx usb non-core registers");
MODULE_AUTHOR("Richard Zhao <richard.zhao@freescale.com>");
