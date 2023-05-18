// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022, Linaro Limited
 */

#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <dt-bindings/clock/qcom,lpasscc-sc8280xp.h>
#include "common.h"
#include "reset.h"

static const struct qcom_reset_map lpass_audio_csr_sc8280xp_resets[] = {
	[LPASS_AUDIO_SWR_RX_CGCR] =  { 0xa0, 1 },
	[LPASS_AUDIO_SWR_WSA_CGCR] = { 0xb0, 1 },
	[LPASS_AUDIO_SWR_WSA2_CGCR] =  { 0xd8, 1 },
};

static struct regmap_config lpass_audio_csr_sc8280xp_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.name = "lpass-audio-csr",
	.max_register = 0x1000,
};

static const struct qcom_cc_desc lpass_audio_csr_reset_sc8280xp_desc = {
	.config = &lpass_audio_csr_sc8280xp_regmap_config,
	.resets = lpass_audio_csr_sc8280xp_resets,
	.num_resets = ARRAY_SIZE(lpass_audio_csr_sc8280xp_resets),
};

static const struct qcom_reset_map lpass_tcsr_sc8280xp_resets[] = {
	[LPASS_AUDIO_SWR_TX_CGCR] = { 0xc010, 1 },
};

static struct regmap_config lpass_tcsr_sc8280xp_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.name = "lpass-tcsr",
	.max_register = 0x12000,
};

static const struct qcom_cc_desc lpass_tcsr_reset_sc8280xp_desc = {
	.config = &lpass_tcsr_sc8280xp_regmap_config,
	.resets = lpass_tcsr_sc8280xp_resets,
	.num_resets = ARRAY_SIZE(lpass_tcsr_sc8280xp_resets),
};

static const struct of_device_id lpasscc_sc8280xp_match_table[] = {
	{
		.compatible = "qcom,sc8280xp-lpassaudiocc",
		.data = &lpass_audio_csr_reset_sc8280xp_desc,
	}, {
		.compatible = "qcom,sc8280xp-lpasscc",
		.data = &lpass_tcsr_reset_sc8280xp_desc,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, lpasscc_sc8280xp_match_table);

static int lpasscc_sc8280xp_probe(struct platform_device *pdev)
{
	const struct qcom_cc_desc *desc = of_device_get_match_data(&pdev->dev);

	return qcom_cc_probe_by_index(pdev, 0, desc);
}

static struct platform_driver lpasscc_sc8280xp_driver = {
	.probe = lpasscc_sc8280xp_probe,
	.driver = {
		.name = "lpasscc-sc8280xp",
		.of_match_table = lpasscc_sc8280xp_match_table,
	},
};

static int __init lpasscc_sc8280xp_init(void)
{
	return platform_driver_register(&lpasscc_sc8280xp_driver);
}
subsys_initcall(lpasscc_sc8280xp_init);

static void __exit lpasscc_sc8280xp_exit(void)
{
	platform_driver_unregister(&lpasscc_sc8280xp_driver);
}
module_exit(lpasscc_sc8280xp_exit);

MODULE_DESCRIPTION("QTI LPASSCC SC8280XP Driver");
MODULE_LICENSE("GPL");
