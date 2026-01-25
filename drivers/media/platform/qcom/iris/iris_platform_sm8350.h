/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef __IRIS_PLATFORM_SM8350_H__
#define __IRIS_PLATFORM_SM8350_H__

static void iris_set_sm8350_preset_registers(struct iris_core *core)
{
	u32 val;

	val = readl(core->reg_base + 0xb0088);
	val &= ~0x11;
	writel(val, core->reg_base + 0xb0088);
}

static const char * const sm8350_clk_reset_table[] = { "core" };

#endif
