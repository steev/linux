// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2020, Linaro Limited

#include <linux/err.h>
#include <linux/init.h>
#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <dt-bindings/sound/qcom,q6dsp-lpass-ports.h>
#include "q6dsp-lpass-clocks.h"

#define Q6DSP_MAX_CLK_ID			105
#define Q6DSP_LPASS_CLK_ROOT_DEFAULT		0


struct q6dsp_clk {
	struct device *dev;
	int q6dsp_clk_id;
	int attributes;
	int rate;
	uint32_t handle;
	bool vote;
	bool prepared;
	/* Still prepared, but dropped by the DSP when it stopped */
	bool lost;
	struct clk_hw hw;
};

#define to_q6dsp_clk(_hw) container_of(_hw, struct q6dsp_clk, hw)

struct q6dsp_cc {
	struct device *dev;
	/* Serializes clock requests against DSP down/up */
	struct mutex lock;
	struct q6dsp_clk *clks[Q6DSP_MAX_CLK_ID];
	const struct q6dsp_clk_desc *desc;
};

static int q6dsp_clk_request(struct q6dsp_cc *cc, struct q6dsp_clk *clk)
{
	if (clk->vote)
		return cc->desc->lpass_vote_clk(clk->dev, clk->q6dsp_clk_id,
						clk_hw_get_name(&clk->hw),
						&clk->handle);

	return cc->desc->lpass_set_clk(clk->dev, clk->q6dsp_clk_id, clk->attributes,
				       Q6DSP_LPASS_CLK_ROOT_DEFAULT, clk->rate);
}

static void q6dsp_clk_release(struct q6dsp_cc *cc, struct q6dsp_clk *clk)
{
	if (clk->vote)
		cc->desc->lpass_unvote_clk(clk->dev, clk->q6dsp_clk_id, clk->handle);
	else
		cc->desc->lpass_set_clk(clk->dev, clk->q6dsp_clk_id, clk->attributes,
					Q6DSP_LPASS_CLK_ROOT_DEFAULT, 0);
}

static int clk_q6dsp_prepare(struct clk_hw *hw)
{
	struct q6dsp_clk *clk = to_q6dsp_clk(hw);
	struct q6dsp_cc *cc = dev_get_drvdata(clk->dev);
	int ret;

	guard(mutex)(&cc->lock);

	ret = q6dsp_clk_request(cc, clk);
	if (!ret)
		clk->prepared = true;

	return ret;
}

static void clk_q6dsp_unprepare(struct clk_hw *hw)
{
	struct q6dsp_clk *clk = to_q6dsp_clk(hw);
	struct q6dsp_cc *cc = dev_get_drvdata(clk->dev);

	guard(mutex)(&cc->lock);

	/* A lost clock is not held by the DSP, so there is nothing to release */
	if (!clk->lost)
		q6dsp_clk_release(cc, clk);

	clk->prepared = false;
	clk->lost = false;
}

static int clk_q6dsp_set_rate(struct clk_hw *hw, unsigned long rate,
			      unsigned long parent_rate)
{
	struct q6dsp_clk *clk = to_q6dsp_clk(hw);

	clk->rate = rate;

	return 0;
}

static unsigned long clk_q6dsp_recalc_rate(struct clk_hw *hw,
					   unsigned long parent_rate)
{
	struct q6dsp_clk *clk = to_q6dsp_clk(hw);

	return clk->rate;
}

static int clk_q6dsp_determine_rate(struct clk_hw *hw,
				    struct clk_rate_request *req)
{
	return 0;
}

static const struct clk_ops clk_q6dsp_ops = {
	.prepare	= clk_q6dsp_prepare,
	.unprepare	= clk_q6dsp_unprepare,
	.set_rate	= clk_q6dsp_set_rate,
	.determine_rate = clk_q6dsp_determine_rate,
	.recalc_rate	= clk_q6dsp_recalc_rate,
};

static const struct clk_ops clk_vote_q6dsp_ops = {
	.prepare	= clk_q6dsp_prepare,
	.unprepare	= clk_q6dsp_unprepare,
};


static struct clk_hw *q6dsp_of_clk_hw_get(struct of_phandle_args *clkspec,
					  void *data)
{
	struct q6dsp_cc *cc = data;
	unsigned int idx = clkspec->args[0];
	unsigned int attr = clkspec->args[1];

	if (idx >= Q6DSP_MAX_CLK_ID || attr > LPASS_CLK_ATTRIBUTE_COUPLE_DIVISOR) {
		dev_err(cc->dev, "Invalid clk specifier (%d, %d)\n", idx, attr);
		return ERR_PTR(-EINVAL);
	}

	if (cc->clks[idx]) {
		cc->clks[idx]->attributes = attr;
		return &cc->clks[idx]->hw;
	}

	return ERR_PTR(-ENOENT);
}

int q6dsp_clock_dev_probe(struct platform_device *pdev)
{
	struct q6dsp_cc *cc;
	struct device *dev = &pdev->dev;
	const struct q6dsp_clk_init *q6dsp_clks;
	const struct q6dsp_clk_desc *desc;
	int i, ret;

	cc = devm_kzalloc(dev, sizeof(*cc), GFP_KERNEL);
	if (!cc)
		return -ENOMEM;

	desc = of_device_get_match_data(&pdev->dev);
	if (!desc)
		return -EINVAL;

	cc->desc = desc;
	cc->dev = dev;
	q6dsp_clks = desc->clks;

	ret = devm_mutex_init(dev, &cc->lock);
	if (ret)
		return ret;

	for (i = 0; i < desc->num_clks; i++) {
		unsigned int id = q6dsp_clks[i].clk_id;
		struct clk_init_data init = {
			.name =  q6dsp_clks[i].name,
		};
		struct q6dsp_clk *clk;

		clk = devm_kzalloc(dev, sizeof(*clk), GFP_KERNEL);
		if (!clk)
			return -ENOMEM;

		clk->dev = dev;
		clk->q6dsp_clk_id = q6dsp_clks[i].q6dsp_clk_id;
		clk->rate = q6dsp_clks[i].rate;
		clk->vote = !clk->rate;
		clk->hw.init = &init;

		if (clk->vote)
			init.ops = &clk_vote_q6dsp_ops;
		else
			init.ops = &clk_q6dsp_ops;

		cc->clks[id] = clk;

		ret = devm_clk_hw_register(dev, &clk->hw);
		if (ret)
			return ret;
	}

	ret = devm_of_clk_add_hw_provider(dev, q6dsp_of_clk_hw_get, cc);
	if (ret)
		return ret;

	dev_set_drvdata(dev, cc);

	return 0;
}
EXPORT_SYMBOL_GPL(q6dsp_clock_dev_probe);

/**
 * q6dsp_clock_dsp_down() - Mark the clocks dropped by a stopped DSP
 * @dev: clock controller device
 *
 * The DSP forgets all clock requests when it stops. Remember the clocks that
 * consumers still hold prepared, so q6dsp_clock_dsp_up() can request them
 * again once the DSP is back.
 */
void q6dsp_clock_dsp_down(struct device *dev)
{
	struct q6dsp_cc *cc;
	int i;

	guard(device)(dev);

	cc = dev_get_drvdata(dev);
	if (!cc)
		return;

	guard(mutex)(&cc->lock);

	for (i = 0; i < Q6DSP_MAX_CLK_ID; i++) {
		struct q6dsp_clk *clk = cc->clks[i];

		if (clk && clk->prepared)
			clk->lost = true;
	}
}
EXPORT_SYMBOL_GPL(q6dsp_clock_dsp_down);

/**
 * q6dsp_clock_dsp_up() - Restore the clocks dropped by a stopped DSP
 * @dev: clock controller device
 *
 * Request again, at their current rate, the clocks that were still prepared
 * when the DSP stopped.
 */
void q6dsp_clock_dsp_up(struct device *dev)
{
	struct q6dsp_cc *cc;
	int i, ret;

	guard(device)(dev);

	cc = dev_get_drvdata(dev);
	if (!cc)
		return;

	guard(mutex)(&cc->lock);

	for (i = 0; i < Q6DSP_MAX_CLK_ID; i++) {
		struct q6dsp_clk *clk = cc->clks[i];

		if (!clk || !clk->lost)
			continue;

		ret = q6dsp_clk_request(cc, clk);
		if (ret) {
			dev_err(dev, "Failed to restore %s: %d\n",
				clk_hw_get_name(&clk->hw), ret);
			continue;
		}

		clk->lost = false;
	}
}
EXPORT_SYMBOL_GPL(q6dsp_clock_dsp_up);
