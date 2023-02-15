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
#include <dt-bindings/clock/qcom,lpass-csr-sc8280xp.h>
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
	.max_register = 0x21000,
};

static const struct qcom_cc_desc lpass_tcsr_reset_sc8280xp_desc = {
	.config = &lpass_tcsr_sc8280xp_regmap_config,
	.resets = lpass_tcsr_sc8280xp_resets,
	.num_resets = ARRAY_SIZE(lpass_tcsr_sc8280xp_resets),
};

static const struct of_device_id lpass_audio_csr_sc8280xp_match_table[] = {
	{	.compatible = "qcom,sc8280xp-lpass-audio-csr",
		.data = &lpass_audio_csr_reset_sc8280xp_desc,
	}, {
		.compatible = "qcom,sc8280xp-lpass-tcsr",
		.data = &lpass_tcsr_reset_sc8280xp_desc,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, lpass_audio_csr_sc8280xp_match_table);

static int lpass_audio_csr_sc8280xp_probe(struct platform_device *pdev)
{
	const struct qcom_cc_desc *desc = of_device_get_match_data(&pdev->dev);

	return qcom_cc_probe_by_index(pdev, 0, desc);
}

static struct platform_driver lpass_audio_csr_sc8280xp_driver = {
	.probe = lpass_audio_csr_sc8280xp_probe,
	.driver = {
		.name = "lpass-csr-sc8280xp",
		.of_match_table = lpass_audio_csr_sc8280xp_match_table,
	},
};

static int __init lpass_audio_csr_sc8280xp_init(void)
{
	return platform_driver_register(&lpass_audio_csr_sc8280xp_driver);
}
subsys_initcall(lpass_audio_csr_sc8280xp_init);

static void __exit lpass_audio_csr_sc8280xp_exit(void)
{
	platform_driver_unregister(&lpass_audio_csr_sc8280xp_driver);
}
module_exit(lpass_audio_csr_sc8280xp_exit);

MODULE_DESCRIPTION("QTI LPASS CSR SC8280XP Driver");
MODULE_LICENSE("GPL");
