// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2021 Linaro Limited
 *
 * Author: Daniel Lezcano <daniel.lezcano@linaro.org>
 *
 * DTPM hierarchy description
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/dtpm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

static struct dtpm_node __initdata sdm845_hierarchy[] = {
	[0]{ .name = "sdm845" },
	[1]{ .name = "package",
	     .parent = &sdm845_hierarchy[0] },
	[2]{ .name = "/cpus/cpu@0",
	     .type = DTPM_NODE_DT,
	     .parent = &sdm845_hierarchy[1] },
	[3]{ .name = "/cpus/cpu@100",
	     .type = DTPM_NODE_DT,
	     .parent = &sdm845_hierarchy[1] },
	[4]{ .name = "/cpus/cpu@200",
	     .type = DTPM_NODE_DT,
	     .parent = &sdm845_hierarchy[1] },
	[5]{ .name = "/cpus/cpu@300",
	     .type = DTPM_NODE_DT,
	     .parent = &sdm845_hierarchy[1] },
	[6]{ .name = "/cpus/cpu@400",
	     .type = DTPM_NODE_DT,
	     .parent = &sdm845_hierarchy[1] },
	[7]{ .name = "/cpus/cpu@500",
	     .type = DTPM_NODE_DT,
	     .parent = &sdm845_hierarchy[1] },
	[8]{ .name = "/cpus/cpu@600",
	     .type = DTPM_NODE_DT,
	     .parent = &sdm845_hierarchy[1] },
	[9]{ .name = "/cpus/cpu@700",
	     .type = DTPM_NODE_DT,
	     .parent = &sdm845_hierarchy[1] },
	[10]{ .name = "/soc@0/gpu@5000000",
	     .type = DTPM_NODE_DT,
	     .parent = &sdm845_hierarchy[1] },
	[11]{ },
};

static struct of_device_id __initdata sdm845_dtpm_match_table[] = {
        { .compatible = "qcom,sdm845", .data = sdm845_hierarchy },
        {},
};

static int __init sdm845_dtpm_init(void)
{
	return dtpm_create_hierarchy(sdm845_dtpm_match_table);
}
late_initcall(sdm845_dtpm_init);

MODULE_DESCRIPTION("Qualcomm DTPM driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:dtpm");
MODULE_AUTHOR("Daniel Lezcano <daniel.lezcano@kernel.org");

