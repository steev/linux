// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/cpu.h>
#include <linux/devfreq.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/scmi_protocol.h>
#include <linux/scmi_qcom_protocol.h>
#include <linux/units.h>

#define MAX_MEMORY_TYPES			4
#define MAX_MONITOR_CNT				5
#define MAX_NAME_LEN				20
#define MAX_MAP_ENTRIES				10

#include "scmi-qcom-memlat-cfg.h"

/**
 * enum scmi_memlat_protocol_cmd - parameter_ids supported by the "MEMLAT" algo_str hosted
 *                                 by the Qualcomm Generic Vendor Protocol on the SCMI controller.
 *
 * MEMLAT (Memory Latency) monitors the counters to detect memory latency bound workloads
 * and scales the frequency/levels of the memory buses accordingly.
 *
 * @MEMLAT_SET_MEM_GROUP: initializes the frequency/level scaling functions for the memory bus.
 * @MEMLAT_SET_MONITOR: configures the monitor to work on a specific memory bus.
 * @MEMLAT_SET_COMMON_EV_MAP: set up common counters used to monitor the cpu frequency.
 * @MEMLAT_SET_GRP_EV_MAP: set up any specific counters used to monitor the memory bus.
 * @MEMLAT_IPM_CEIL: set the IPM (Instruction Per Misses) ceiling per monitor.
 * @MEMLAT_SAMPLE_MS: set the sampling period for all the monitors.
 * @MEMLAT_MON_FREQ_MAP: setup the cpufreq to memfreq map.
 * @MEMLAT_SET_MIN_FREQ: set the max frequency of the memory bus.
 * @MEMLAT_SET_MAX_FREQ: set the min frequency of the memory bus.
 * @MEMLAT_START_TIMER: start all the monitors with the requested sampling period.
 * @MEMLAT_STOP_TIMER: stop all the running monitors.
 * @MEMLAT_SET_EFFECTIVE_FREQ_METHOD: set the method used to determine cpu frequency.
 */
enum scmi_memlat_protocol_cmd {
	MEMLAT_SET_MEM_GROUP = 16,
	MEMLAT_SET_MONITOR,
	MEMLAT_SET_COMMON_EV_MAP,
	MEMLAT_SET_GRP_EV_MAP,
	MEMLAT_IPM_CEIL = 23,
	MEMLAT_BE_STALL_FLOOR = 25,
	MEMLAT_SAMPLE_MS = 31,
	MEMLAT_MON_FREQ_MAP,
	MEMLAT_SET_MIN_FREQ,
	MEMLAT_SET_MAX_FREQ,
	MEMLAT_GET_CUR_FREQ,
	MEMLAT_START_TIMER = 36,
	MEMLAT_STOP_TIMER,
	MEMLAT_SET_EFFECTIVE_FREQ_METHOD = 39,
};

struct cpucp_map_table {
	u16 v1;
	u16 v2;
};

struct map_param_msg {
	u32 hw_type;
	u32 mon_idx;
	u32 nr_rows;
	struct cpucp_map_table tbl[MAX_MAP_ENTRIES];
} __packed;

struct node_msg {
	u32 cpumask;
	u32 hw_type;
	u32 mon_type;
	u32 mon_idx;
	char mon_name[MAX_NAME_LEN];
};

struct scalar_param_msg {
	u32 hw_type;
	u32 mon_idx;
	u32 val;
};

enum common_ev_idx {
	INST_IDX,
	CYC_IDX,
	CONST_CYC_IDX,
	FE_STALL_IDX,
	BE_STALL_IDX,
	NUM_COMMON_EVS
};

enum grp_ev_idx {
	MISS_IDX,
	WB_IDX,
	ACC_IDX,
	NUM_GRP_EVS
};

struct ev_map_msg {
	u32 num_evs;
	u32 hw_type;
	u32 cid[NUM_COMMON_EVS];
};

struct scmi_qcom_memlat_map {
	unsigned int cpufreq_mhz;
	unsigned int memfreq_khz;
};

struct scmi_qcom_monitor_info {
	struct scmi_qcom_memlat_map *freq_map;
	char name[MAX_NAME_LEN];
	u32 mon_idx;
	u32 mon_type;
	u32 ipm_ceil;
	u32 be_stall_floor;
	u32 mask;
	u32 freq_map_len;
};

struct scmi_qcom_memory_info {
	struct scmi_qcom_monitor_info *monitor[MAX_MONITOR_CNT];
	u32 hw_type;
	int monitor_cnt;
	u32 min_freq;
	u32 max_freq;
	struct devfreq_dev_profile profile;
	struct devfreq *devfreq;
	struct platform_device *pdev;
	struct scmi_protocol_handle *ph;
	const struct qcom_generic_ext_ops *ops;
};

struct scmi_qcom_memlat_info {
	struct scmi_protocol_handle *ph;
	const struct qcom_generic_ext_ops *ops;
	struct scmi_qcom_memory_info *memory[MAX_MEMORY_TYPES];
	u32 cpucp_freq_method;
	u32 cpucp_sample_ms;
	int memory_cnt;
};

static int configure_cpucp_common_events(struct scmi_qcom_memlat_info *info)
{
	const struct qcom_generic_ext_ops *ops = info->ops;
	u8 ev_map[NUM_COMMON_EVS];
	struct ev_map_msg msg;

	memset(ev_map, 0xFF, NUM_COMMON_EVS);

	msg.num_evs = NUM_COMMON_EVS;
	msg.cid[INST_IDX] = EV_INST_RETIRED;
	msg.cid[CYC_IDX] = EV_CPU_CYCLES;
	msg.cid[CONST_CYC_IDX] = EV_CNT_CYCLES;
	msg.cid[FE_STALL_IDX] = INVALID_IDX;
	msg.cid[BE_STALL_IDX] = EV_STALL_BACKEND_MEM;

	return ops->set_param(info->ph, &msg, sizeof(msg), MEMLAT_ALGO_STR,
			      MEMLAT_SET_COMMON_EV_MAP);
}

static int configure_cpucp_grp(struct device *dev, struct scmi_qcom_memlat_info *info,
			       int memory_index)
{
	struct scmi_qcom_memory_info *memory = info->memory[memory_index];
	const struct qcom_generic_ext_ops *ops = info->ops;
	struct ev_map_msg ev_msg;
	u8 ev_map[NUM_GRP_EVS];
	struct node_msg msg;
	int ret;

	msg.cpumask = 0;
	msg.hw_type = memory->hw_type;
	msg.mon_type = 0;
	msg.mon_idx = 0;
	ret = ops->set_param(info->ph, &msg, sizeof(msg), MEMLAT_ALGO_STR, MEMLAT_SET_MEM_GROUP);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to configure mem type %d\n",
				     memory->hw_type);

	memset(ev_map, 0xFF, NUM_GRP_EVS);
	ev_msg.num_evs = NUM_GRP_EVS;
	ev_msg.hw_type = memory->hw_type;
	ev_msg.cid[MISS_IDX] = EV_L2_D_RFILL;
	ev_msg.cid[WB_IDX] = INVALID_IDX;
	ev_msg.cid[ACC_IDX] = INVALID_IDX;
	ret = ops->set_param(info->ph, &ev_msg, sizeof(ev_msg), MEMLAT_ALGO_STR,
			     MEMLAT_SET_GRP_EV_MAP);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to configure event map for mem type %d\n",
				     memory->hw_type);

	return ret;
}

static int configure_cpucp_mon(struct device *dev, struct scmi_qcom_memlat_info *info,
			       int memory_index, int monitor_index)
{
	const struct qcom_generic_ext_ops *ops = info->ops;
	struct scmi_qcom_memory_info *memory = info->memory[memory_index];
	struct scmi_qcom_monitor_info *monitor = memory->monitor[monitor_index];
	struct scalar_param_msg scalar_msg;
	struct map_param_msg map_msg;
	struct node_msg msg;
	int ret;
	int i;

	msg.cpumask = monitor->mask;
	msg.hw_type = memory->hw_type;
	msg.mon_type = monitor->mon_type;
	msg.mon_idx = monitor->mon_idx;
	strscpy(msg.mon_name, monitor->name, sizeof(msg.mon_name));
	ret = ops->set_param(info->ph, &msg, sizeof(msg), MEMLAT_ALGO_STR, MEMLAT_SET_MONITOR);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to configure monitor %s\n",
				     monitor->name);

	scalar_msg.hw_type = memory->hw_type;
	scalar_msg.mon_idx = monitor->mon_idx;
	scalar_msg.val = monitor->ipm_ceil;
	ret = ops->set_param(info->ph, &scalar_msg, sizeof(scalar_msg), MEMLAT_ALGO_STR,
			     MEMLAT_IPM_CEIL);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to set ipm ceil for %s\n",
				     monitor->name);

	scalar_msg.hw_type = memory->hw_type;
	scalar_msg.mon_idx = monitor->mon_idx;
	scalar_msg.val = monitor->be_stall_floor;
	ret = ops->set_param(info->ph, &scalar_msg, sizeof(scalar_msg), MEMLAT_ALGO_STR,
			     MEMLAT_BE_STALL_FLOOR);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to set ipm ceil for %s\n", monitor->name);

	map_msg.hw_type = memory->hw_type;
	map_msg.mon_idx = monitor->mon_idx;
	map_msg.nr_rows = monitor->freq_map_len;
	for (i = 0; i < monitor->freq_map_len; i++) {
		map_msg.tbl[i].v1 = monitor->freq_map[i].cpufreq_mhz;

		if (monitor->freq_map[i].memfreq_khz > 1)
			map_msg.tbl[i].v2 = monitor->freq_map[i].memfreq_khz / 1000;
		else
			map_msg.tbl[i].v2 = monitor->freq_map[i].memfreq_khz;
	}
	ret = ops->set_param(info->ph, &map_msg, sizeof(map_msg), MEMLAT_ALGO_STR,
			     MEMLAT_MON_FREQ_MAP);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to configure freq_map for %s\n",
				     monitor->name);

	scalar_msg.hw_type = memory->hw_type;
	scalar_msg.mon_idx = monitor->mon_idx;
	scalar_msg.val = memory->min_freq;
	ret = ops->set_param(info->ph, &scalar_msg, sizeof(scalar_msg), MEMLAT_ALGO_STR,
			     MEMLAT_SET_MIN_FREQ);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to set min_freq for %s\n",
				     monitor->name);

	scalar_msg.hw_type = memory->hw_type;
	scalar_msg.mon_idx = monitor->mon_idx;
	scalar_msg.val = memory->max_freq;
	ret = ops->set_param(info->ph, &scalar_msg, sizeof(scalar_msg), MEMLAT_ALGO_STR,
			     MEMLAT_SET_MAX_FREQ);
	if (ret < 0)
		dev_err_probe(dev, ret, "failed to set max_freq for %s\n", monitor->name);

	return ret;
}

static int scmi_qcom_devfreq_get_cur_freq(struct device *dev, unsigned long *freq)
{
	struct scmi_qcom_memory_info *memory = dev_get_drvdata(dev);
	const struct qcom_generic_ext_ops *ops;
	struct scalar_param_msg scalar_msg;
	int ret;

	ops = memory->ops;

	scalar_msg.hw_type = memory->hw_type;
	scalar_msg.mon_idx = 0;
	u32 cur_freq;

	ret = ops->get_param(memory->ph, &scalar_msg, sizeof(scalar_msg), MEMLAT_ALGO_STR,
			     MEMLAT_GET_CUR_FREQ, sizeof(cur_freq));
	if (ret < 0) {
		pr_err("failed to get mon current frequency\n");
		return ret;
	}

	memcpy(&cur_freq, (void *)&scalar_msg, sizeof(cur_freq));

	if (memory->hw_type == 2)
		*freq = le32_to_cpu(cur_freq) ? 100 : 1;
	else
		*freq = le32_to_cpu(cur_freq) * 1000UL;

	return 0;
}

static int scmi_qcom_memlat_configure_events(struct scmi_device *sdev,
					     struct scmi_qcom_memlat_info *info)
{
	const struct qcom_generic_ext_ops *ops = info->ops;
	struct scmi_protocol_handle *ph = info->ph;
	int i, j, ret;

	/* Configure common events ids */
	ret = configure_cpucp_common_events(info);
	if (ret < 0)
		return dev_err_probe(&sdev->dev, ret, "failed to configure common events\n");

	for (i = 0; i < info->memory_cnt; i++) {
		/* Configure per group parameters */
		ret = configure_cpucp_grp(&sdev->dev, info, i);
		if (ret < 0)
			return ret;

		for (j = 0; j < info->memory[i]->monitor_cnt; j++) {
			/* Configure per monitor parameters */
			ret = configure_cpucp_mon(&sdev->dev, info, i, j);
			if (ret < 0)
				return ret;
		}
	}

	/* Set loop sampling time */
	ret = ops->set_param(ph, &info->cpucp_sample_ms, sizeof(info->cpucp_sample_ms),
			     MEMLAT_ALGO_STR, MEMLAT_SAMPLE_MS);
	if (ret < 0)
		return dev_err_probe(&sdev->dev, ret, "failed to set sample_ms\n");

	/* Set the effective cpu frequency calculation method */
	ret = ops->set_param(ph, &info->cpucp_freq_method, sizeof(info->cpucp_freq_method),
			     MEMLAT_ALGO_STR, MEMLAT_SET_EFFECTIVE_FREQ_METHOD);
	if (ret < 0)
		return dev_err_probe(&sdev->dev, ret,
				     "failed to set effective frequency calc method\n");

	/* Start sampling and voting timer */
	ret = ops->start_activity(ph, NULL, 0, MEMLAT_ALGO_STR, MEMLAT_START_TIMER);
	if (ret < 0)
		dev_err_probe(&sdev->dev, ret, "failed to start memory group timer\n");

	for (i = 0; i < info->memory_cnt; i++) {
		struct scmi_qcom_memory_info *memory = info->memory[i];
		struct platform_device *pdev = memory->pdev;
		struct devfreq_dev_profile *profile = &memory->profile;

		profile->polling_ms = info->cpucp_sample_ms;
		profile->get_cur_freq = scmi_qcom_devfreq_get_cur_freq;
		profile->initial_freq = memory->min_freq > 1 ?
					(memory->min_freq * 1000UL) : memory->min_freq;

		memory->ops = info->ops;
		memory->ph = info->ph;

		platform_set_drvdata(pdev, memory);

		memory->devfreq = devm_devfreq_add_device(&pdev->dev, profile,
							  DEVFREQ_GOV_REMOTE, NULL);
		if (IS_ERR(memory->devfreq)) {
			dev_err(&sdev->dev, "failed to add devfreq device\n");
			/* Start sampling and voting timer */
			ret = ops->start_activity(ph, NULL, 0, MEMLAT_ALGO_STR, MEMLAT_STOP_TIMER);
			if (ret < 0)
				dev_err_probe(&sdev->dev, ret,
					      "failed to stop memory group timer\n");
			return -EINVAL;
		}
	}

	return 0;
}

static struct scmi_qcom_memlat_map *scmi_qcom_parse_memlat_map(struct device *dev,
							       const struct scmi_qcom_monitor_cfg *mon_cfg)
{
	struct scmi_qcom_memlat_map *map_table;
	const struct scmi_qcom_map_table *table;

	map_table = devm_kzalloc(dev, MAX_MAP_ENTRIES * sizeof(struct scmi_qcom_memlat_map),
				 GFP_KERNEL);
	if (!map_table)
		return ERR_PTR(-ENOMEM);

	for (int i = 0; i < mon_cfg->table_len; i++) {
		table = &mon_cfg->table[i];

		map_table[i].cpufreq_mhz = table->cpu_freq;
		map_table[i].memfreq_khz = table->mem_freq;
	}

	return map_table;
}

static const struct of_device_id scmi_qcom_memlat_configs[] __maybe_unused = {
	{ .compatible = "qcom,glymur", .data = &glymur_memlat_data},
	{ .compatible = "qcom,mahua", .data = &glymur_memlat_data},
	{ .compatible = "qcom,x1e80100", .data = &hamoa_memlat_data},
	{ .compatible = "qcom,x1p42100", .data = &hamoa_memlat_data},
	{ }
};

static int scmi_qcom_memlat_parse_cfg(struct scmi_device *sdev, struct scmi_qcom_memlat_info *info)
{
	const struct scmi_qcom_memlat_cfg_data *cfg_data;
	struct scmi_qcom_monitor_info *monitor;
	struct scmi_qcom_memory_info *memory;
	int ret, i, j;

	cfg_data = of_machine_get_match_data(scmi_qcom_memlat_configs);
	if (!cfg_data) {
		return dev_err_probe(&sdev->dev, PTR_ERR(cfg_data),
				     "Couldn't find config data for this platform\n");
	}

	for (i = 0; i < cfg_data->memory_cnt; i++) {
		const struct scmi_qcom_memory_cfg *memory_cfg = &cfg_data->memory_cfg[i];
		struct platform_device_info pdevinfo = { 0 };

		pdevinfo.parent = &sdev->dev;
		pdevinfo.name = memory_cfg->name;
		pdevinfo.id = PLATFORM_DEVID_NONE;

		memory = devm_kzalloc(&sdev->dev, sizeof(*memory), GFP_KERNEL);
		if (!memory) {
			ret = -ENOMEM;
			goto out;
		}

		memory->ops = info->ops;
		memory->ph = info->ph;
		memory->hw_type = memory_cfg->memory_type;
		memory->monitor_cnt = memory_cfg->monitor_cnt;
		memory->min_freq = memory_cfg->memory_range.min_freq;
		memory->max_freq = memory_cfg->memory_range.max_freq;

		memory->pdev = platform_device_register_full(&pdevinfo);
		if (IS_ERR(memory->pdev)) {
			dev_err_probe(&sdev->dev, PTR_ERR(memory->pdev),
				      "failed to register platform device\n");
			ret = PTR_ERR(memory->pdev);
			goto out;
		}

		info->memory[i] = memory;

		for (j = 0; j < memory_cfg->num_opps; j++) {
			const struct scmi_qcom_opp_data *table = &memory_cfg->mem_table[j];
			struct platform_device *pdev = memory->pdev;
			struct dev_pm_opp_data data;

			data.freq = table->freq;
			data.level = table->level;

			ret = dev_pm_opp_add_dynamic(&pdev->dev, &data);
			if (ret) {
				dev_err_probe(&sdev->dev, ret, "failed to add OPP\n");
				dev_pm_opp_remove_all_dynamic(&pdev->dev);
				goto out;
			}
		}

		for (j = 0; j < memory_cfg->monitor_cnt; j++) {
			const struct scmi_qcom_monitor_cfg *monitor_cfg = &memory_cfg->monitor_cfg[j];

			monitor = devm_kzalloc(&sdev->dev, sizeof(*monitor), GFP_KERNEL);
			if (!monitor)
				return -ENOMEM;

			monitor->ipm_ceil = monitor_cfg->ipm_ceil;
			monitor->mon_type = monitor->ipm_ceil ? 0 : 1;
			monitor->be_stall_floor = monitor_cfg->be_stall_floor;
			monitor->mask = monitor_cfg->cpu_mask;
			monitor->freq_map_len = monitor_cfg->table_len;

			monitor->freq_map = scmi_qcom_parse_memlat_map(&sdev->dev, monitor_cfg);
			if (IS_ERR(monitor->freq_map)) {
				dev_err_probe(&sdev->dev, PTR_ERR(monitor->freq_map),
					      "failed to populate cpufreq-memfreq map\n");
				ret = -EINVAL;
				goto out;
			}

			strscpy(monitor->name, monitor_cfg->name, sizeof(monitor->name));
			monitor->mon_idx = j;
			memory->monitor[j] = monitor;
		}
	}

	info->cpucp_freq_method = cfg_data->cpucp_freq_method;
	info->cpucp_sample_ms = cfg_data->cpucp_sample_ms;
	info->memory_cnt = cfg_data->memory_cnt;

	return 0;

out:
	for (i = 0; i < cfg_data->memory_cnt; i++) {
		if (IS_ERR_OR_NULL(info->memory[i]))
			break;

		memory = info->memory[i];
		if (!IS_ERR(memory->pdev))
			platform_device_unregister(memory->pdev);
	}

	return ret;
}

static int scmi_qcom_devfreq_memlat_probe(struct scmi_device *sdev)
{
	const struct scmi_handle *handle = sdev->handle;
	const struct qcom_generic_ext_ops *ops;
	struct scmi_qcom_memlat_info *info;
	struct scmi_protocol_handle *ph;
	int ret;

	if (!handle)
		return -ENODEV;

	info = devm_kzalloc(&sdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	ops = handle->devm_protocol_get(sdev, SCMI_PROTOCOL_QCOM_GENERIC, &ph);
	if (IS_ERR(ops))
		return PTR_ERR(ops);

	info->ops = ops;
	info->ph = ph;

	ret = scmi_qcom_memlat_parse_cfg(sdev, info);
	if (ret)
		return ret;

	ret = scmi_qcom_memlat_configure_events(sdev, info);
	if (ret)
		return ret;

	dev_set_drvdata(&sdev->dev, info);

	return ret;
}

static void scmi_qcom_devfreq_memlat_remove(struct scmi_device *sdev)
{
	struct scmi_qcom_memlat_info *info = dev_get_drvdata(&sdev->dev);

	for (int i = 0; i < info->memory_cnt; i++) {
		struct scmi_qcom_memory_info *memory = info->memory[i];

		if (!IS_ERR(memory->pdev))
			platform_device_unregister(memory->pdev);
	}
}

static const struct scmi_device_id scmi_id_table[] = {
	{ SCMI_PROTOCOL_QCOM_GENERIC, "qcom-generic-ext" },
	{ },
};
MODULE_DEVICE_TABLE(scmi, scmi_id_table);

static struct scmi_driver scmi_qcom_devfreq_memlat_driver = {
	.name		= "scmi-qcom-devfreq-memlat",
	.probe		= scmi_qcom_devfreq_memlat_probe,
	.remove		= scmi_qcom_devfreq_memlat_remove,
	.id_table	= scmi_id_table,
};
module_scmi_driver(scmi_qcom_devfreq_memlat_driver);

MODULE_AUTHOR("Sibi Sankar <sibi.sankar@oss.qualcomm.com>");
MODULE_DESCRIPTION("SCMI QCOM DEVFREQ MEMLAT driver");
MODULE_LICENSE("GPL");
