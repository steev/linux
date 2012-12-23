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

#include "ci13xxx_imx.h"

#define USB_DEV_MAX 4

#define BM_OVER_CUR_DIS		BIT(7)

struct imx_usbmisc {
	void __iomem *base;
	spinlock_t lock;
	struct clk *clk_ahb;
	struct clk *clk_ipg;
	struct clk *clk_per;
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

	reg = usbmisc->base + 0x08;

	if (usbdev->evdo) {
		spin_lock_irqsave(&usbmisc->lock, flags);
		val = readl(reg);
		writel(val | (1 << 23), reg);
		spin_unlock_irqrestore(&usbmisc->lock, flags);
		mdelay(5); /* needed to stabilize voltage */
	}

	return 0;
}

static int usbmisc_imx53_init(struct device *dev)
{
	struct usbmisc_usb_device *usbdev;
	void __iomem *reg;
	unsigned long flags;
	u32 val;

	usbdev = get_usbdev(dev);
	if (IS_ERR(usbdev))
		return PTR_ERR(usbdev);

	reg = usbmisc->base + usbdev->index * 8;

	if (usbdev->disable_oc) {
		spin_lock_irqsave(&usbmisc->lock, flags);
		val = readl(reg);
		writel(val | (1 << 5), reg);
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
		reg = readl(usbmisc->base + usbdev->index * 4);
		writel(reg | BM_OVER_CUR_DIS,
			usbmisc->base + usbdev->index * 4);
		spin_unlock_irqrestore(&usbmisc->lock, flags);
	}

	return 0;
}

static const struct usbmisc_ops imx25_usbmisc_ops = {
	.post = usbmisc_imx25_post,
};

static const struct usbmisc_ops imx53_usbmisc_ops = {
	.init = usbmisc_imx53_init,
};

static const struct usbmisc_ops imx6q_usbmisc_ops = {
	.init = usbmisc_imx6q_init,
};

static const struct of_device_id usbmisc_imx_dt_ids[] = {
	{ .compatible = "fsl,imx25-usbmisc", .data = (void *)&imx25_usbmisc_ops },
	{ .compatible = "fsl,imx51-usbmisc", .data = (void *)&imx53_usbmisc_ops },
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

	ret = clk_prepare_enable(data->clk_ahb);
	if (ret)
		return ret;

	ret = clk_prepare_enable(data->clk_ipg);
	if (ret)
		goto err_ipg_failed;

	ret = clk_prepare_enable(data->clk_per);
	if (ret)
		goto err_per_failed;

	tmp_dev = (struct of_device_id *)
		of_match_device(usbmisc_imx_dt_ids, &pdev->dev);
	data->ops = (const struct usbmisc_ops *)tmp_dev->data;
	usbmisc = data;
	ret = usbmisc_set_ops(data->ops);
	if (ret)
		goto err_set_ops_failed;

	return 0;

 err_set_ops_failed:
	usbmisc = NULL;
	clk_disable_unprepare(data->clk_per);
 err_per_failed:
	clk_disable_unprepare(data->clk_ipg);
 err_ipg_failed:
	clk_disable_unprepare(data->clk_ahb);

	return ret;
}

static int usbmisc_imx_remove(struct platform_device *pdev)
{
	usbmisc_unset_ops(usbmisc->ops);
	clk_disable_unprepare(usbmisc->clk_per);
	clk_disable_unprepare(usbmisc->clk_ipg);
	clk_disable_unprepare(usbmisc->clk_ahb);
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
