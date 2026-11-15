// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2021, Linaro Limited

#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/jiffies.h>
#include <linux/soc/qcom/apr.h>
#include <dt-bindings/soc/qcom,gpr.h>
#include <dt-bindings/sound/qcom,q6dsp-lpass-ports.h>
#include "q6apm.h"
#include "q6dsp-lpass-clocks.h"
#include "q6prm.h"
#include "audioreach.h"

struct q6prm_clkctl;

struct q6prm {
	struct device *dev;
	gpr_device_t *gdev;
	wait_queue_head_t wait;
	struct gpr_ibasic_rsp_result_t result;
	struct mutex lock;
	struct q6prm_clkctl *clkctl;
};

/*
 * The q6prm GPR device is removed whenever the DSP stops, but clock consumers
 * hold their clk handles across a DSP restart. The clock controller device is
 * therefore created only once, under the remoteproc device, and kept until the
 * module is unloaded. Its clock ops reach the q6prm instance of the running
 * DSP through @prm, which is NULL while the DSP is down.
 */
struct q6prm_clkctl {
	struct list_head node;
	struct device_node *np;
	struct platform_device *pdev;
	struct q6prm *prm;
};

static DEFINE_MUTEX(q6prm_clkctl_lock);
static LIST_HEAD(q6prm_clkctl_list);

/* Called with q6prm_clkctl_lock held */
static struct q6prm *q6prm_clkctl_get_prm(struct device *dev)
{
	struct q6prm_clkctl *ctl;

	list_for_each_entry(ctl, &q6prm_clkctl_list, node)
		if (ctl->np == dev->of_node)
			return ctl->prm;

	return NULL;
}

#define PRM_CMD_REQUEST_HW_RSC		0x0100100F
#define PRM_CMD_RSP_REQUEST_HW_RSC	0x02001002
#define PRM_CMD_RELEASE_HW_RSC		0x01001010
#define PRM_CMD_RSP_RELEASE_HW_RSC	0x02001003
#define PARAM_ID_RSC_HW_CORE		0x08001032
#define PARAM_ID_RSC_LPASS_CORE		0x0800102B
#define PARAM_ID_RSC_AUDIO_HW_CLK	0x0800102C
#define PARAM_ID_RSC_CPU_LPR		0x08001A6E

#define LPR_CPU_SS_SLEEP_DISABLE	0x1

struct prm_cmd_request_hw_core {
	struct apm_module_param_data param_data;
	union {
		u32 hw_clk_id;
		u32 lpr_state;
	};
} __packed;

struct prm_cmd_request_rsc {
	struct apm_module_param_data param_data;
	uint32_t num_clk_id;
	struct audio_hw_clk_cfg clock_id;
} __packed;

struct prm_cmd_release_rsc {
	struct apm_module_param_data param_data;
	uint32_t num_clk_id;
	struct audio_hw_clk_rel_cfg clock_id;
} __packed;

static int q6prm_send_cmd_sync(struct q6prm *prm, struct gpr_pkt *pkt, uint32_t rsp_opcode)
{
	pkt->hdr.dest_domain = audioreach_gpr_dest_domain(prm->gdev);

	return audioreach_send_cmd_sync(prm->dev, prm->gdev, &prm->result, &prm->lock,
					NULL, &prm->wait, pkt, rsp_opcode);
}

static int q6prm_set_hw_core_req(struct q6prm *prm, uint32_t hw_block_id, bool enable)
{
	struct apm_module_param_data *param_data;
	struct prm_cmd_request_hw_core *req;
	gpr_device_t *gdev = prm->gdev;
	uint32_t opcode, rsp_opcode;
	bool lpr_req = (hw_block_id == Q6PRM_HW_LPR_VOTE);

	if (enable) {
		opcode = PRM_CMD_REQUEST_HW_RSC;
		rsp_opcode = PRM_CMD_RSP_REQUEST_HW_RSC;
	} else {
		opcode = PRM_CMD_RELEASE_HW_RSC;
		rsp_opcode = PRM_CMD_RSP_RELEASE_HW_RSC;
	}

	struct gpr_pkt *pkt __free(kfree) =
		audioreach_alloc_cmd_pkt(sizeof(*req), opcode, 0, gdev->svc.id, GPR_PRM_MODULE_IID);
	if (IS_ERR(pkt))
		return PTR_ERR(pkt);

	req = (void *)pkt + GPR_HDR_SIZE + APM_CMD_HDR_SIZE;

	param_data = &req->param_data;

	param_data->module_instance_id = GPR_PRM_MODULE_IID;
	param_data->error_code = 0;
	param_data->param_id = lpr_req ? PARAM_ID_RSC_CPU_LPR : PARAM_ID_RSC_HW_CORE;
	param_data->param_size = sizeof(*req) - APM_MODULE_PARAM_DATA_SIZE;

	if (lpr_req)
		req->lpr_state = LPR_CPU_SS_SLEEP_DISABLE;
	else
		req->hw_clk_id = hw_block_id;

	return q6prm_send_cmd_sync(prm, pkt, rsp_opcode);
}

int q6prm_vote_lpass_core_hw(struct device *dev, uint32_t hw_block_id,
			     const char *client_name, uint32_t *client_handle)
{
	struct q6prm *prm;

	guard(mutex)(&q6prm_clkctl_lock);

	prm = q6prm_clkctl_get_prm(dev);
	if (!prm)
		return -EBUSY;

	return q6prm_set_hw_core_req(prm, hw_block_id, true);
}
EXPORT_SYMBOL_GPL(q6prm_vote_lpass_core_hw);

int q6prm_unvote_lpass_core_hw(struct device *dev, uint32_t hw_block_id, uint32_t client_handle)
{
	struct q6prm *prm;

	guard(mutex)(&q6prm_clkctl_lock);

	prm = q6prm_clkctl_get_prm(dev);
	if (!prm)
		return -EBUSY;

	return q6prm_set_hw_core_req(prm, hw_block_id, false);
}
EXPORT_SYMBOL_GPL(q6prm_unvote_lpass_core_hw);

static int q6prm_request_lpass_clock(struct q6prm *prm, int clk_id, int clk_attr, int clk_root,
				     unsigned int freq)
{
	struct apm_module_param_data *param_data;
	struct prm_cmd_request_rsc *req;
	gpr_device_t *gdev = prm->gdev;

	struct gpr_pkt *pkt __free(kfree) =
		audioreach_alloc_cmd_pkt(sizeof(*req), PRM_CMD_REQUEST_HW_RSC, 0,
					 gdev->svc.id, GPR_PRM_MODULE_IID);
	if (IS_ERR(pkt))
		return PTR_ERR(pkt);

	req = (void *)pkt + GPR_HDR_SIZE + APM_CMD_HDR_SIZE;

	param_data = &req->param_data;

	param_data->module_instance_id = GPR_PRM_MODULE_IID;
	param_data->error_code = 0;
	param_data->param_id = PARAM_ID_RSC_AUDIO_HW_CLK;
	param_data->param_size = sizeof(*req) - APM_MODULE_PARAM_DATA_SIZE;

	req->num_clk_id = 1;
	req->clock_id.clock_id = clk_id;
	req->clock_id.clock_freq = freq;
	req->clock_id.clock_attri = clk_attr;
	req->clock_id.clock_root = clk_root;

	return q6prm_send_cmd_sync(prm, pkt, PRM_CMD_RSP_REQUEST_HW_RSC);
}

static int q6prm_release_lpass_clock(struct q6prm *prm, int clk_id, int clk_attr, int clk_root,
			  unsigned int freq)
{
	struct apm_module_param_data *param_data;
	struct prm_cmd_release_rsc *rel;
	gpr_device_t *gdev = prm->gdev;

	struct gpr_pkt *pkt __free(kfree) =
		audioreach_alloc_cmd_pkt(sizeof(*rel), PRM_CMD_RELEASE_HW_RSC, 0,
					 gdev->svc.id, GPR_PRM_MODULE_IID);
	if (IS_ERR(pkt))
		return PTR_ERR(pkt);

	rel = (void *)pkt + GPR_HDR_SIZE + APM_CMD_HDR_SIZE;

	param_data = &rel->param_data;

	param_data->module_instance_id = GPR_PRM_MODULE_IID;
	param_data->error_code = 0;
	param_data->param_id = PARAM_ID_RSC_AUDIO_HW_CLK;
	param_data->param_size = sizeof(*rel) - APM_MODULE_PARAM_DATA_SIZE;

	rel->num_clk_id = 1;
	rel->clock_id.clock_id = clk_id;

	return q6prm_send_cmd_sync(prm, pkt, PRM_CMD_RSP_RELEASE_HW_RSC);
}

int q6prm_set_lpass_clock(struct device *dev, int clk_id, int clk_attr, int clk_root,
			  unsigned int freq)
{
	struct q6prm *prm;

	guard(mutex)(&q6prm_clkctl_lock);

	prm = q6prm_clkctl_get_prm(dev);
	if (!prm)
		return -EBUSY;

	if (freq)
		return q6prm_request_lpass_clock(prm, clk_id, clk_attr, clk_root, freq);

	return q6prm_release_lpass_clock(prm, clk_id, clk_attr, clk_root, freq);
}
EXPORT_SYMBOL_GPL(q6prm_set_lpass_clock);

static int prm_callback(const struct gpr_resp_pkt *data, void *priv, int op)
{
	gpr_device_t *gdev = priv;
	struct q6prm *prm = dev_get_drvdata(&gdev->dev);
	const struct gpr_ibasic_rsp_result_t *result;
	const struct gpr_hdr *hdr = &data->hdr;

	if (!prm)
		return 0;

	switch (hdr->opcode) {
	case PRM_CMD_RSP_REQUEST_HW_RSC:
	case PRM_CMD_RSP_RELEASE_HW_RSC:
		result = data->payload;
		prm->result.opcode = hdr->opcode;
		prm->result.status = result->status;
		wake_up(&prm->wait);
		break;
	default:
		break;
	}

	return 0;
}

/*
 * The GPR device sits on a chain of rpmsg and remoteproc devices that are all
 * torn down when the DSP stops. Its closest platform device ancestor is the
 * remoteproc platform device, which stays across DSP restarts.
 */
static struct device *q6prm_clkctl_parent(struct device *dev)
{
	for (dev = dev->parent; dev; dev = dev->parent)
		if (dev_is_platform(dev))
			return dev;

	return NULL;
}

static int q6prm_clkctl_attach(struct q6prm *prm)
{
	struct device *dev = prm->dev;
	struct q6prm_clkctl *ctl = NULL, *iter;
	struct device_node *np;

	np = of_get_child_by_name(dev->of_node, "clock-controller");
	if (!np)
		return 0;

	scoped_guard(mutex, &q6prm_clkctl_lock) {
		list_for_each_entry(iter, &q6prm_clkctl_list, node) {
			if (iter->np == np) {
				ctl = iter;
				break;
			}
		}

		if (ctl) {
			of_node_put(np);
		} else {
			ctl = kzalloc_obj(*ctl);
			if (!ctl) {
				of_node_put(np);
				return -ENOMEM;
			}

			ctl->np = np;
			list_add(&ctl->node, &q6prm_clkctl_list);
		}

		ctl->prm = prm;
	}

	/* Created on first probe, once its clock ops can already reach the DSP */
	if (!ctl->pdev) {
		ctl->pdev = of_platform_device_create(ctl->np, NULL,
						      q6prm_clkctl_parent(dev));
		if (!ctl->pdev) {
			scoped_guard(mutex, &q6prm_clkctl_lock)
				ctl->prm = NULL;
			return dev_err_probe(dev, -ENODEV,
					     "Failed to create clock controller\n");
		}
	}

	prm->clkctl = ctl;

	return 0;
}

static int prm_probe(gpr_device_t *gdev)
{
	struct device *dev = &gdev->dev;
	struct q6prm *cc;
	int ret;

	cc = devm_kzalloc(dev, sizeof(*cc), GFP_KERNEL);
	if (!cc)
		return -ENOMEM;

	cc->dev = dev;
	cc->gdev = gdev;
	mutex_init(&cc->lock);
	init_waitqueue_head(&cc->wait);
	dev_set_drvdata(dev, cc);

	if (!q6apm_is_adsp_ready())
		return -EPROBE_DEFER;

	ret = q6prm_clkctl_attach(cc);
	if (ret)
		return ret;

	if (cc->clkctl)
		q6dsp_clock_dsp_up(&cc->clkctl->pdev->dev);

	return 0;
}

static void prm_remove(gpr_device_t *gdev)
{
	struct q6prm *prm = dev_get_drvdata(&gdev->dev);
	struct q6prm_clkctl *ctl = prm->clkctl;

	if (!ctl)
		return;

	/* Fail new clock requests first, then mark what the DSP dropped */
	scoped_guard(mutex, &q6prm_clkctl_lock)
		ctl->prm = NULL;

	q6dsp_clock_dsp_down(&ctl->pdev->dev);
}

#ifdef CONFIG_OF
static const struct of_device_id prm_device_id[]  = {
	{ .compatible = "qcom,q6prm" },
	{},
};
MODULE_DEVICE_TABLE(of, prm_device_id);
#endif

static gpr_driver_t prm_driver = {
	.probe = prm_probe,
	.remove = prm_remove,
	.gpr_callback = prm_callback,
	.driver = {
		.name = "qcom-prm",
		.of_match_table = of_match_ptr(prm_device_id),
	},
};

static int __init q6prm_init(void)
{
	return apr_driver_register(&prm_driver);
}
module_init(q6prm_init);

static void __exit q6prm_exit(void)
{
	struct q6prm_clkctl *ctl, *tmp;

	apr_driver_unregister(&prm_driver);

	list_for_each_entry_safe(ctl, tmp, &q6prm_clkctl_list, node) {
		if (ctl->pdev)
			of_platform_device_destroy(&ctl->pdev->dev, NULL);
		of_node_put(ctl->np);
		list_del(&ctl->node);
		kfree(ctl);
	}
}
module_exit(q6prm_exit);

MODULE_DESCRIPTION("Q6 Proxy Resource Manager");
MODULE_LICENSE("GPL");
