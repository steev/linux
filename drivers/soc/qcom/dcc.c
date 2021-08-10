// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2021, The Linux Foundation. All rights reserved.
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#define TIMEOUT_US		5000

#define dcc_writel(drvdata, val, off)					\
	writel((val), drvdata->base + dcc_offset_conv(drvdata, off))
#define dcc_readl(drvdata, off)						\
	readl(drvdata->base + dcc_offset_conv(drvdata, off))

#define DCC_SRAM_NODE "dcc_sram"

/* DCC registers */
#define DCC_HW_INFO			0x04
#define DCC_LL_NUM_INFO			0x10
#define DCC_STATUS			0x1C
#define DCC_LL_LOCK(m)			(0x34 + 0x80 * m)
#define DCC_LL_CFG(m)			(0x38 + 0x80 * m)
#define DCC_LL_BASE(m)			(0x3c + 0x80 * m)
#define DCC_FD_BASE(m)			(0x40 + 0x80 * m)
#define DCC_LL_TIMEOUT(m)		(0x44 + 0x80 * m)
#define DCC_LL_INT_ENABLE(m)		(0x4C + 0x80 * m)
#define DCC_LL_INT_STATUS(m)		(0x50 + 0x80 * m)
#define DCC_LL_SW_TRIGGER(m)		(0x60 + 0x80 * m)
#define DCC_LL_BUS_ACCESS_STATUS(m)	(0x64 + 0x80 * m)

#define DCC_MAP_LEVEL1			0x18
#define DCC_MAP_LEVEL2			0x34
#define DCC_MAP_LEVEL3			0x4C

#define DCC_MAP_OFFSET1			0x10
#define DCC_MAP_OFFSET2			0x18
#define DCC_MAP_OFFSET3			0x1C
#define DCC_MAP_OFFSET4			0x8

#define DCC_FIX_LOOP_OFFSET		16
#define DCC_VER_INFO_BIT		9

#define DCC_READ			0
#define DCC_WRITE			1
#define DCC_LOOP			2
#define DCC_READ_WRITE			3

#define MAX_DCC_OFFSET			GENMASK(9, 2)
#define MAX_DCC_LEN			GENMASK(6, 0)
#define MAX_LOOP_CNT			GENMASK(7, 0)

#define DCC_ADDR_DESCRIPTOR		0x00
#define DCC_ADDR_LIMIT			27
#define DCC_ADDR_OFF_RANGE		8
#define DCC_ADDR_RANGE			GENMASK(31, 4)
#define DCC_LOOP_DESCRIPTOR		BIT(30)
#define DCC_RD_MOD_WR_DESCRIPTOR	BIT(31)
#define DCC_LINK_DESCRIPTOR		GENMASK(31, 30)

#define DCC_READ_IND			0x00
#define DCC_WRITE_IND			(BIT(28))

#define DCC_AHB_IND			0x00
#define DCC_APB_IND			BIT(29)

#define DCC_MAX_LINK_LIST		8
#define DCC_INVALID_LINK_LIST		GENMASK(7, 0)

#define DCC_VER_MASK1			GENMASK(6, 0)
#define DCC_VER_MASK2			GENMASK(5, 0)

#define DCC_RD_MOD_WR_ADDR              0xC105E

struct qcom_dcc_config {
	int dcc_ram_offset;
};

enum dcc_descriptor_type {
	DCC_ADDR_TYPE,
	DCC_LOOP_TYPE,
	DCC_READ_WRITE_TYPE,
	DCC_WRITE_TYPE
};

enum dcc_mem_map_ver {
	DCC_MEM_MAP_VER1 = 1,
	DCC_MEM_MAP_VER2 = 2,
	DCC_MEM_MAP_VER3 = 3
};

struct dcc_config_entry {
	u32				base;
	u32				offset;
	u32				len;
	u32				index;
	u32				loop_cnt;
	u32				write_val;
	u32				mask;
	bool				apb_bus;
	enum dcc_descriptor_type	desc_type;
	struct list_head		list;
};

/**
 * struct dcc_drvdata - configuration information related to a dcc device
 * @base:	      Base Address of the dcc device
 * @dev:	      The device attached to the driver data
 * @mutex:	      Lock to protect access and manipulation of dcc_drvdata
 * @ram_base:         Base address for the SRAM dedicated for the dcc device
 * @ram_offset:       Offset to the SRAM dedicated for dcc device
 * @mem_map_ver:      Memory map version of DCC hardware
 * @ram_cfg:          Used for address limit calculation for dcc
 * @ram_start:        Starting address of DCC SRAM
 * @enable:	      Flag to check if DCC linked list is enabled
 * @interrupt_disable:Flag to enable/disable interrupts
 * @sram_dev:	      Character device equivalent of dcc SRAM
 * @sram_class:	      Class equivalent of the DCC SRAM device
 * @cfg_head:	      Points to the head of the linked list of addresses
 * @nr_config:        Stores the number of addresses currently configured for a linkedlist
 * @nr_link_list:     Total number of linkedlists supported by the DCC configuration
 * @curr_list:        The index of the current linklist with which the driver is working
 * @loopoff:          Loop offset bits range for the addresses
 */
struct dcc_drvdata {
	void __iomem		*base;
	struct device		*dev;
	struct mutex		mutex;
	void __iomem		*ram_base;
	phys_addr_t		ram_size;
	phys_addr_t		ram_offset;
	enum dcc_mem_map_ver	mem_map_ver;
	phys_addr_t		ram_cfg;
	phys_addr_t		ram_start;
	bool			*enable;
	bool			interrupt_disable;
	struct cdev		sram_dev;
	struct class		*sram_class;
	struct list_head	*cfg_head;
	size_t			*nr_config;
	size_t			nr_link_list;
	u8			curr_list;
	u8			loopoff;
};

struct dcc_cfg_attr {
	u32	addr;
	u32	prev_addr;
	u32	prev_off;
	u32	link;
	u32	sram_offset;
};

struct dcc_cfg_loop_attr {
	u32	loop;
	bool	loop_start;
	u32	loop_cnt;
	u32	loop_len;
	u32	loop_off;
};

static size_t dcc_offset_conv(struct dcc_drvdata *drvdata, size_t off)
{
	if (drvdata->mem_map_ver == DCC_MEM_MAP_VER1) {
		if ((off & DCC_VER_MASK1) >= DCC_MAP_LEVEL3)
			return off - DCC_MAP_OFFSET3;
		if ((off & DCC_VER_MASK1) >= DCC_MAP_LEVEL2)
			return off - DCC_MAP_OFFSET2;
		else if ((off & DCC_VER_MASK1) >= DCC_MAP_LEVEL1)
			return off - DCC_MAP_OFFSET1;
	} else if (drvdata->mem_map_ver == DCC_MEM_MAP_VER2) {
		if ((off & DCC_VER_MASK1) >= DCC_MAP_LEVEL2)
			return off - DCC_MAP_OFFSET4;
	}

	return off;
}

static int dcc_sram_writel(struct dcc_drvdata *drvdata,
					u32 val, u32 off)
{
	if (unlikely(off > (drvdata->ram_size - 4)))
		return -EINVAL;

	writel(val, drvdata->ram_base + off);

	return 0;
}

static bool dcc_ready(struct dcc_drvdata *drvdata)
{
	u32 val;

	return !readl_poll_timeout((drvdata->base + dcc_offset_conv(drvdata, DCC_STATUS)),
				val, (FIELD_GET(GENMASK(1, 0), val) == 0), 1, TIMEOUT_US);

}

static int dcc_read_status(struct dcc_drvdata *drvdata)
{
	int curr_list;
	u32 bus_status;
	u32 ll_cfg;
	u32 tmp_ll_cfg;

	for (curr_list = 0; curr_list < drvdata->nr_link_list; curr_list++) {
		if (!drvdata->enable[curr_list])
			continue;

		bus_status = dcc_readl(drvdata, DCC_LL_BUS_ACCESS_STATUS(curr_list));

		if (bus_status) {
			dev_err(drvdata->dev,
				"Read access error for list %d err: 0x%x.\n",
				curr_list, bus_status);

			ll_cfg = dcc_readl(drvdata, DCC_LL_CFG(curr_list));
			tmp_ll_cfg = ll_cfg & ~BIT(9);
			dcc_writel(drvdata, tmp_ll_cfg, DCC_LL_CFG(curr_list));
			dcc_writel(drvdata, 0x3,
				DCC_LL_BUS_ACCESS_STATUS(curr_list));
			dcc_writel(drvdata, ll_cfg, DCC_LL_CFG(curr_list));
			return -ENODATA;
		}
	}

	return 0;
}

static int dcc_sw_trigger(struct dcc_drvdata *drvdata)
{
	int ret;
	int curr_list;
	u32 ll_cfg;
	u32 tmp_ll_cfg;

	mutex_lock(&drvdata->mutex);

	for (curr_list = 0; curr_list < drvdata->nr_link_list; curr_list++) {
		if (!drvdata->enable[curr_list])
			continue;
		ll_cfg = dcc_readl(drvdata, DCC_LL_CFG(curr_list));
		tmp_ll_cfg = ll_cfg & ~BIT(9);
		dcc_writel(drvdata, tmp_ll_cfg, DCC_LL_CFG(curr_list));
		dcc_writel(drvdata, 1, DCC_LL_SW_TRIGGER(curr_list));
		dcc_writel(drvdata, ll_cfg, DCC_LL_CFG(curr_list));
	}

	if (!dcc_ready(drvdata)) {
		dev_err(drvdata->dev,
			"DCC is busy after receiving sw tigger.\n");
		ret = -EBUSY;
		goto err;
	}

	ret = dcc_read_status(drvdata);

err:
	mutex_unlock(&drvdata->mutex);
	return ret;
}

static void _dcc_ll_cfg_reset_link(struct dcc_cfg_attr *cfg)
{
	cfg->addr = 0x00;
	cfg->link = 0;
	cfg->prev_off = 0;
	cfg->prev_addr = cfg->addr;
}

static int _dcc_ll_cfg_read_write(struct dcc_drvdata *drvdata,
				  struct dcc_config_entry *entry,
				  struct dcc_cfg_attr *cfg)
{
	int ret;

	if (cfg->link) {
		/*
		 * write new offset = 1 to continue
		 * processing the list
		 */

		ret = dcc_sram_writel(drvdata, cfg->link, cfg->sram_offset);
		if (ret)
			return ret;
		cfg->sram_offset += 4;
		/* Reset link and prev_off */
		_dcc_ll_cfg_reset_link(cfg);
	}

	cfg->addr = DCC_RD_MOD_WR_DESCRIPTOR;
	ret = dcc_sram_writel(drvdata, cfg->addr, cfg->sram_offset);
	if (ret)
		return ret;

	cfg->sram_offset += 4;
	ret = dcc_sram_writel(drvdata, entry->mask, cfg->sram_offset);
	if (ret)
		return ret;

	cfg->sram_offset += 4;
	ret = dcc_sram_writel(drvdata, entry->write_val, cfg->sram_offset);
	if (ret)
		return ret;

	cfg->sram_offset += 4;
	cfg->addr = 0;
	return ret;
}

static int _dcc_ll_cfg_loop(struct dcc_drvdata *drvdata, struct dcc_config_entry *entry,
			    struct dcc_cfg_attr *cfg,
			    struct dcc_cfg_loop_attr *cfg_loop,
			    u32 *total_len)
{

	int ret;

	/* Check if we need to write link of prev entry */
	if (cfg->link) {
		ret = dcc_sram_writel(drvdata, cfg->link, cfg->sram_offset);
		if (ret)
			return ret;
		cfg->sram_offset += 4;
	}

	if (cfg_loop->loop_start) {
		cfg_loop->loop = (cfg->sram_offset - cfg_loop->loop_off) / 4;
		cfg_loop->loop |= (cfg_loop->loop_cnt << drvdata->loopoff) &
		GENMASK(DCC_ADDR_LIMIT, drvdata->loopoff);
		cfg_loop->loop |= DCC_LOOP_DESCRIPTOR;
		*total_len += (*total_len - cfg_loop->loop_len) * cfg_loop->loop_cnt;

		ret = dcc_sram_writel(drvdata, cfg_loop->loop, cfg->sram_offset);
		if (ret)
			return ret;
		cfg->sram_offset += 4;

		cfg_loop->loop_start = false;
		cfg_loop->loop_len = 0;
		cfg_loop->loop_off = 0;
	} else {
		cfg_loop->loop_start = true;
		cfg_loop->loop_cnt = entry->loop_cnt - 1;
		cfg_loop->loop_len = *total_len;
		cfg_loop->loop_off = cfg->sram_offset;
	}

	/* Reset link and prev_off */
	_dcc_ll_cfg_reset_link(cfg);

	return ret;
}

static int _dcc_ll_cfg_write(struct dcc_drvdata *drvdata,
			     struct dcc_config_entry *entry,
			     struct dcc_cfg_attr *cfg,
			     u32 *total_len)
{
	u32 off;
	int ret;

	if (cfg->link) {
		/*
		 * write new offset = 1 to continue
		 * processing the list
		 */
		ret = dcc_sram_writel(drvdata, cfg->link, cfg->sram_offset);

		if (ret)
			return ret;

		cfg->sram_offset += 4;
		/* Reset link and prev_off */
		cfg->addr = 0x00;
		cfg->prev_off = 0;
		cfg->prev_addr = cfg->addr;
	}

	off = entry->offset/4;
	/* write new offset-length pair to correct position */
	cfg->link |= ((off & GENMASK(7, 0)) | BIT(15) | ((entry->len << 8) & GENMASK(14, 8)));
	cfg->link |= DCC_LINK_DESCRIPTOR;

	/* Address type */
	cfg->addr = (entry->base >> 4) & GENMASK(DCC_ADDR_LIMIT, 0);
	if (entry->apb_bus)
		cfg->addr |= DCC_ADDR_DESCRIPTOR | DCC_WRITE_IND | DCC_APB_IND;
	else
		cfg->addr |= DCC_ADDR_DESCRIPTOR | DCC_WRITE_IND | DCC_AHB_IND;
	ret = dcc_sram_writel(drvdata, cfg->addr, cfg->sram_offset);

	if (ret)
		return ret;
	cfg->sram_offset += 4;

	ret = dcc_sram_writel(drvdata, cfg->link, cfg->sram_offset);
	if (ret)
		return ret;
	cfg->sram_offset += 4;

	ret = dcc_sram_writel(drvdata, entry->write_val, cfg->sram_offset);

	if (ret)
		return ret;

	cfg->sram_offset += 4;
	cfg->addr = 0x00;
	cfg->link = 0;
	return ret;
}

static int _dcc_ll_cfg_default(struct dcc_drvdata *drvdata,
			       struct dcc_config_entry *entry,
			       struct dcc_cfg_attr *cfg,
			       u32 *pos, u32 *total_len)
{
	int ret;
	u32 off;
	u32 temp_off;

	cfg->addr = (entry->base >> 4) & GENMASK(27, 0);

	if (entry->apb_bus)
		cfg->addr |= DCC_ADDR_DESCRIPTOR | DCC_READ_IND | DCC_APB_IND;
	else
		cfg->addr |= DCC_ADDR_DESCRIPTOR | DCC_READ_IND | DCC_AHB_IND;

	off = entry->offset/4;

	*total_len += entry->len * 4;

	if (!cfg->prev_addr || cfg->prev_addr != cfg->addr || cfg->prev_off > off) {
		/* Check if we need to write prev link entry */
		if (cfg->link) {
			ret = dcc_sram_writel(drvdata, cfg->link, cfg->sram_offset);
			if (ret)
				return ret;
			cfg->sram_offset += 4;
		}
		dev_dbg(drvdata->dev, "DCC: sram address 0x%x\n", cfg->sram_offset);

		/* Write address */
		ret = dcc_sram_writel(drvdata, cfg->addr, cfg->sram_offset);

		if (ret)
			return ret;

		cfg->sram_offset += 4;

		/* Reset link and prev_off */
		cfg->link = 0;
		cfg->prev_off = 0;
	}

	if ((off - cfg->prev_off) > 0xFF || entry->len > MAX_DCC_LEN) {
		dev_err(drvdata->dev, "DCC: Programming error Base: 0x%x, offset 0x%x\n",
		entry->base, entry->offset);
		ret = -EINVAL;
		return ret;
	}

	if (cfg->link) {
		/*
		 * link already has one offset-length so new
		 * offset-length needs to be placed at
		 * bits [29:15]
		 */
		*pos = 15;

		/* Clear bits [31:16] */
		cfg->link &= GENMASK(14, 0);
	} else {
		/*
		 * link is empty, so new offset-length needs
		 * to be placed at bits [15:0]
		 */
		*pos = 0;
		cfg->link = 1 << 15;
	}

	/* write new offset-length pair to correct position */
	temp_off = (off-cfg->prev_off) & GENMASK(7, 0);
	cfg->link |= temp_off |((entry->len << 8) & GENMASK(14, 8)) << *pos;

	cfg->link |= DCC_LINK_DESCRIPTOR;

	if (*pos) {
		ret = dcc_sram_writel(drvdata, cfg->link, cfg->sram_offset);
		if (ret)
			return ret;
		cfg->sram_offset += 4;
		cfg->link = 0;
	}

	cfg->prev_off  = off + entry->len - 1;
	cfg->prev_addr = cfg->addr;
	return ret;
}

static int __dcc_ll_cfg(struct dcc_drvdata *drvdata, int curr_list)
{
	int ret = 0;
	u32 total_len, pos;
	struct dcc_config_entry *entry;
	struct dcc_cfg_attr cfg;
	struct dcc_cfg_loop_attr cfg_loop;

	memset(&cfg, 0, sizeof(cfg));
	memset(&cfg_loop, 0, sizeof(cfg_loop));
	cfg.sram_offset = drvdata->ram_cfg * 4;
	total_len = 0;

	list_for_each_entry(entry, &drvdata->cfg_head[curr_list], list) {
		switch (entry->desc_type) {
		case DCC_READ_WRITE_TYPE:
			ret = _dcc_ll_cfg_read_write(drvdata, entry, &cfg);
			if (ret)
				goto overstep;
			break;

		case DCC_LOOP_TYPE:
			ret = _dcc_ll_cfg_loop(drvdata, entry, &cfg, &cfg_loop, &total_len);
			if (ret)
				goto overstep;
			break;

		case DCC_WRITE_TYPE:
			ret = _dcc_ll_cfg_write(drvdata, entry, &cfg, &total_len);
			if (ret)
				goto overstep;
			break;

		default:
			ret = _dcc_ll_cfg_default(drvdata, entry, &cfg, &pos, &total_len);
			if (ret)
				goto overstep;
			break;
		}
	}

	if (cfg.link) {
		ret = dcc_sram_writel(drvdata, cfg.link, cfg.sram_offset);
		if (ret)
			goto overstep;
		cfg.sram_offset += 4;
	}

	if (cfg_loop.loop_start) {
		dev_err(drvdata->dev, "DCC: Programming error: Loop unterminated\n");
		ret = -EINVAL;
		goto err;
	}

	/* Handling special case of list ending with a rd_mod_wr */
	if (cfg.addr == DCC_RD_MOD_WR_DESCRIPTOR) {
		cfg.addr = (DCC_RD_MOD_WR_ADDR) & GENMASK(27, 0);
		cfg.addr |= DCC_ADDR_DESCRIPTOR;
		ret = dcc_sram_writel(drvdata, cfg.addr, cfg.sram_offset);
		if (ret)
			goto overstep;
		cfg.sram_offset += 4;
	}

	/* Setting zero to indicate end of the list */
	cfg.link = DCC_LINK_DESCRIPTOR;
	ret = dcc_sram_writel(drvdata, cfg.link, cfg.sram_offset);
	if (ret)
		goto overstep;
	cfg.sram_offset += 4;

	/* Update ram_cfg and check if the data will overstep */

	drvdata->ram_cfg = (cfg.sram_offset + total_len) / 4;

	if (cfg.sram_offset + total_len > drvdata->ram_size) {
		cfg.sram_offset += total_len;
		goto overstep;
	}

	drvdata->ram_start = cfg.sram_offset/4;
	return 0;
overstep:
	ret = -EINVAL;
	memset_io(drvdata->ram_base, 0, drvdata->ram_size);

err:
	return ret;
}

static int dcc_valid_list(struct dcc_drvdata *drvdata, int curr_list)
{
	u32 lock_reg;

	if (list_empty(&drvdata->cfg_head[curr_list]))
		return -EINVAL;

	if (drvdata->enable[curr_list]) {
		dev_err(drvdata->dev, "List %d is already enabled\n",
				curr_list);
		return -EINVAL;
	}

	lock_reg = dcc_readl(drvdata, DCC_LL_LOCK(curr_list));
	if (lock_reg & 0x1) {
		dev_err(drvdata->dev, "List %d is already locked\n",
				curr_list);
		return -EINVAL;
	}

	dev_err(drvdata->dev, "DCC list passed %d\n", curr_list);
	return 0;
}

static bool is_dcc_enabled(struct dcc_drvdata *drvdata)
{
	bool dcc_enable = false;
	int list;

	for (list = 0; list < DCC_MAX_LINK_LIST; list++) {
		if (drvdata->enable[list]) {
			dcc_enable = true;
			break;
		}
	}

	return dcc_enable;
}

static int dcc_enable(struct dcc_drvdata *drvdata)
{
	int ret = 0;
	int list;
	u32 ram_cfg_base;

	mutex_lock(&drvdata->mutex);

	if (!is_dcc_enabled(drvdata)) {
		memset_io(drvdata->ram_base,
			0xDE, drvdata->ram_size);
	}

	for (list = 0; list < drvdata->nr_link_list; list++) {

		if (dcc_valid_list(drvdata, list))
			continue;

		/* 1. Take ownership of the list */
		dcc_writel(drvdata, BIT(0), DCC_LL_LOCK(list));

		/* 2. Program linked-list in the SRAM */
		ram_cfg_base = drvdata->ram_cfg;
		ret = __dcc_ll_cfg(drvdata, list);
		if (ret) {
			dcc_writel(drvdata, 0, DCC_LL_LOCK(list));
			goto err;
		}

		/* 3. program DCC_RAM_CFG reg */
		dcc_writel(drvdata, ram_cfg_base +
			drvdata->ram_offset/4, DCC_LL_BASE(list));
		dcc_writel(drvdata, drvdata->ram_start +
			drvdata->ram_offset/4, DCC_FD_BASE(list));
		dcc_writel(drvdata, 0xFFF, DCC_LL_TIMEOUT(list));

		/* 4. Clears interrupt status register */
		dcc_writel(drvdata, 0, DCC_LL_INT_ENABLE(list));
		dcc_writel(drvdata, (BIT(0) | BIT(1) | BIT(2)),
					DCC_LL_INT_STATUS(list));

		drvdata->enable[list] = true;

		/* 5. Configure trigger */
		dcc_writel(drvdata, BIT(9), DCC_LL_CFG(list));
	}

err:
	mutex_unlock(&drvdata->mutex);
	return ret;
}

static void dcc_disable(struct dcc_drvdata *drvdata)
{
	int curr_list;

	mutex_lock(&drvdata->mutex);

	for (curr_list = 0; curr_list < drvdata->nr_link_list; curr_list++) {
		if (!drvdata->enable[curr_list])
			continue;
		dcc_writel(drvdata, 0, DCC_LL_CFG(curr_list));
		dcc_writel(drvdata, 0, DCC_LL_BASE(curr_list));
		dcc_writel(drvdata, 0, DCC_FD_BASE(curr_list));
		dcc_writel(drvdata, 0, DCC_LL_LOCK(curr_list));
		drvdata->enable[curr_list] = false;
	}
	memset_io(drvdata->ram_base, 0, drvdata->ram_size);
	drvdata->ram_cfg = 0;
	drvdata->ram_start = 0;
	mutex_unlock(&drvdata->mutex);
}

static ssize_t curr_list_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret;
	struct dcc_drvdata *drvdata = dev_get_drvdata(dev);

	mutex_lock(&drvdata->mutex);
	if (drvdata->curr_list == DCC_INVALID_LINK_LIST) {
		dev_err(dev, "curr_list is not set.\n");
		ret = -EINVAL;
		goto err;
	}

	ret = scnprintf(buf, PAGE_SIZE, "%d\n",	drvdata->curr_list);
err:
	mutex_unlock(&drvdata->mutex);
	return ret;
}

static ssize_t curr_list_store(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct dcc_drvdata *drvdata = dev_get_drvdata(dev);
	unsigned long val;
	u32 lock_reg;
	bool dcc_enable = false;

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;

	if (val >= drvdata->nr_link_list)
		return -EINVAL;

	mutex_lock(&drvdata->mutex);

	dcc_enable = is_dcc_enabled(drvdata);
	if (drvdata->curr_list != DCC_INVALID_LINK_LIST	&& dcc_enable) {
		dev_err(drvdata->dev, "DCC is enabled, please disable it first.\n");
		mutex_unlock(&drvdata->mutex);
		return -EINVAL;
	}

	lock_reg = dcc_readl(drvdata, DCC_LL_LOCK(val));
	if (lock_reg & 0x1) {
		dev_err(drvdata->dev, "DCC linked list is already configured\n");
		mutex_unlock(&drvdata->mutex);
		return -EINVAL;
	}
	drvdata->curr_list = val;
	mutex_unlock(&drvdata->mutex);

	return size;
}

static DEVICE_ATTR_RW(curr_list);


static ssize_t trigger_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	int ret = 0;
	unsigned long val;
	struct dcc_drvdata *drvdata = dev_get_drvdata(dev);

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;
	if (val != 1)
		return -EINVAL;

	ret = dcc_sw_trigger(drvdata);
	if (!ret)
		ret = size;

	return ret;
}
static DEVICE_ATTR_WO(trigger);

static ssize_t enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret;
	bool dcc_enable = false;
	struct dcc_drvdata *drvdata = dev_get_drvdata(dev);

	mutex_lock(&drvdata->mutex);
	if (drvdata->curr_list >= drvdata->nr_link_list) {
		dev_err(dev, "Select link list to program using curr_list\n");
		ret = -EINVAL;
		goto err;
	}

	dcc_enable = is_dcc_enabled(drvdata);

	ret = scnprintf(buf, PAGE_SIZE, "%u\n",
				(unsigned int)dcc_enable);
err:
	mutex_unlock(&drvdata->mutex);
	return ret;
}

static ssize_t enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int ret = 0;
	unsigned long val;
	struct dcc_drvdata *drvdata = dev_get_drvdata(dev);

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;

	if (val)
		ret = dcc_enable(drvdata);
	else
		dcc_disable(drvdata);

	if (!ret)
		ret = size;

	return ret;

}

static DEVICE_ATTR_RW(enable);

static ssize_t config_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct dcc_drvdata *drvdata = dev_get_drvdata(dev);
	struct dcc_config_entry *entry;
	char local_buf[64];
	int len = 0, count = 0;

	mutex_lock(&drvdata->mutex);
	if (drvdata->curr_list >= drvdata->nr_link_list) {
		dev_err(dev, "Select link list to program using curr_list\n");
		count = -EINVAL;
		goto err;
	}

	list_for_each_entry(entry,
	&drvdata->cfg_head[drvdata->curr_list], list) {
		switch (entry->desc_type) {
		case DCC_READ_WRITE_TYPE:
			len = snprintf(local_buf, 64, "Index: 0x%x, mask: 0x%x, val: 0x%x\n",
				entry->index, entry->mask, entry->write_val);
			break;
		case DCC_LOOP_TYPE:
			len = snprintf(local_buf, 64, "Index: 0x%x, Loop: %d\n",
				entry->index, entry->loop_cnt);
			break;
		case DCC_WRITE_TYPE:
			len = snprintf(local_buf, 64,
				"Write Index: 0x%x, Base: 0x%x, Offset: 0x%x, len: 0x%x APB: %d\n",
				entry->index, entry->base, entry->offset, entry->len,
				entry->apb_bus);
			break;
		default:
			len = snprintf(local_buf, 64,
				"Read Index: 0x%x, Base: 0x%x, Offset: 0x%x, len: 0x%x APB: %d\n",
				entry->index, entry->base, entry->offset,
				entry->len, entry->apb_bus);
		}

		if ((count + len) > PAGE_SIZE) {
			dev_err(dev, "DCC: Couldn't write complete config\n");
			break;
		}
		count += len;
	}

err:
	mutex_unlock(&drvdata->mutex);
	return count;
}

static int dcc_config_add(struct dcc_drvdata *drvdata, unsigned int addr,
				unsigned int len, int apb_bus)
{
	int ret;
	struct dcc_config_entry *entry, *pentry;
	unsigned int base, offset;

	mutex_lock(&drvdata->mutex);

	if (drvdata->curr_list >= drvdata->nr_link_list) {
		dev_err(drvdata->dev, "Select link list to program using curr_list\n");
		ret = -EINVAL;
		goto err;
	}

	if (!len || len > (drvdata->ram_size / DCC_ADDR_OFF_RANGE)) {
		dev_err(drvdata->dev, "DCC: Invalid length\n");
		ret = -EINVAL;
		goto err;
	}

	base = addr & DCC_ADDR_RANGE;

	if (!list_empty(&drvdata->cfg_head[drvdata->curr_list])) {
		pentry = list_last_entry(&drvdata->cfg_head[drvdata->curr_list],
			struct dcc_config_entry, list);

		if (pentry->desc_type == DCC_ADDR_TYPE &&
				addr >= (pentry->base + pentry->offset) &&
				addr <= (pentry->base +
					pentry->offset + MAX_DCC_OFFSET)) {

			/* Re-use base address from last entry */
			base = pentry->base;

			if ((pentry->len * 4 + pentry->base + pentry->offset)
					== addr) {
				len += pentry->len;

				if (len > MAX_DCC_LEN)
					pentry->len = MAX_DCC_LEN;
				else
					pentry->len = len;

				addr = pentry->base + pentry->offset +
					pentry->len * 4;
				len -= pentry->len;
			}
		}
	}

	offset = addr - base;

	while (len) {
		entry = devm_kzalloc(drvdata->dev, sizeof(*entry), GFP_KERNEL);
		if (!entry) {
			ret = -ENOMEM;
			goto err;
		}

		entry->base = base;
		entry->offset = offset;
		entry->len = min_t(u32, len, MAX_DCC_LEN);
		entry->index = drvdata->nr_config[drvdata->curr_list]++;
		entry->desc_type = DCC_ADDR_TYPE;
		entry->apb_bus = apb_bus;
		INIT_LIST_HEAD(&entry->list);
		list_add_tail(&entry->list,
			&drvdata->cfg_head[drvdata->curr_list]);

		len -= entry->len;
		offset += MAX_DCC_LEN * 4;
	}

	mutex_unlock(&drvdata->mutex);
	return 0;
err:
	mutex_unlock(&drvdata->mutex);
	return ret;
}

static ssize_t config_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int ret, len, apb_bus;
	unsigned int base;
	struct dcc_drvdata *drvdata = dev_get_drvdata(dev);
	int nval;

	nval = sscanf(buf, "%x %i %d", &base, &len, &apb_bus);
	if (nval <= 0 || nval > 3)
		return -EINVAL;

	if (nval == 1) {
		len = 1;
		apb_bus = 0;
	} else if (nval == 2) {
		apb_bus = 0;
	} else {
		apb_bus = 1;
	}

	ret = dcc_config_add(drvdata, base, len, apb_bus);
	if (ret)
		return ret;

	return size;

}

static DEVICE_ATTR_RW(config);

static void dcc_config_reset(struct dcc_drvdata *drvdata)
{
	struct dcc_config_entry *entry, *temp;
	int curr_list;

	mutex_lock(&drvdata->mutex);

	for (curr_list = 0; curr_list < drvdata->nr_link_list; curr_list++) {
		list_for_each_entry_safe(entry, temp,
			&drvdata->cfg_head[curr_list], list) {
			list_del(&entry->list);
			devm_kfree(drvdata->dev, entry);
			drvdata->nr_config[curr_list]--;
		}
	}
	drvdata->ram_start = 0;
	drvdata->ram_cfg = 0;
	mutex_unlock(&drvdata->mutex);
}


static ssize_t config_reset_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t size)
{
	unsigned long val;
	struct dcc_drvdata *drvdata = dev_get_drvdata(dev);

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;

	if (val)
		dcc_config_reset(drvdata);

	return size;
}

static DEVICE_ATTR_WO(config_reset);

static ssize_t ready_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret;
	struct dcc_drvdata *drvdata = dev_get_drvdata(dev);

	mutex_lock(&drvdata->mutex);

	if (drvdata->curr_list >= drvdata->nr_link_list) {
		dev_err(dev, "Select link list to program using curr_list\n");
		ret = -EINVAL;
		goto err;
	}

	if (!drvdata->enable[drvdata->curr_list]) {
		ret = -EINVAL;
		goto err;
	}

	ret = scnprintf(buf, PAGE_SIZE, "%u\n",
			(unsigned int)FIELD_GET(BIT(1), dcc_readl(drvdata, DCC_STATUS)));
err:
	mutex_unlock(&drvdata->mutex);
	return ret;
}

static DEVICE_ATTR_RO(ready);

static ssize_t interrupt_disable_show(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	struct dcc_drvdata *drvdata = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n",
				(unsigned int)drvdata->interrupt_disable);
}

static ssize_t interrupt_disable_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t size)
{
	unsigned long val;
	struct dcc_drvdata *drvdata = dev_get_drvdata(dev);

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;

	mutex_lock(&drvdata->mutex);
	drvdata->interrupt_disable = (val ? 1:0);
	mutex_unlock(&drvdata->mutex);
	return size;
}

static DEVICE_ATTR_RW(interrupt_disable);

static int dcc_add_loop(struct dcc_drvdata *drvdata, unsigned long loop_cnt)
{
	struct dcc_config_entry *entry;

	entry = devm_kzalloc(drvdata->dev, sizeof(*entry), GFP_KERNEL);
	if (!entry)
		return -ENOMEM;

	entry->loop_cnt = min_t(u32, loop_cnt, MAX_LOOP_CNT);
	entry->index = drvdata->nr_config[drvdata->curr_list]++;
	entry->desc_type = DCC_LOOP_TYPE;
	INIT_LIST_HEAD(&entry->list);
	list_add_tail(&entry->list, &drvdata->cfg_head[drvdata->curr_list]);

	return 0;
}

static ssize_t loop_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t size)
{
	int ret;
	unsigned long loop_cnt;
	struct dcc_drvdata *drvdata = dev_get_drvdata(dev);

	mutex_lock(&drvdata->mutex);

	if (kstrtoul(buf, 16, &loop_cnt)) {
		ret = -EINVAL;
		goto err;
	}

	if (drvdata->curr_list >= drvdata->nr_link_list) {
		dev_err(dev, "Select link list to program using curr_list\n");
		ret = -EINVAL;
		goto err;
	}

	ret = dcc_add_loop(drvdata, loop_cnt);
	if (ret)
		goto err;

	mutex_unlock(&drvdata->mutex);
	return size;
err:
	mutex_unlock(&drvdata->mutex);
	return ret;
}

static DEVICE_ATTR_WO(loop);

static int dcc_rd_mod_wr_add(struct dcc_drvdata *drvdata, unsigned int mask,
				unsigned int val)
{
	int ret = 0;
	struct dcc_config_entry *entry;

	mutex_lock(&drvdata->mutex);

	if (drvdata->curr_list >= drvdata->nr_link_list) {
		dev_err(drvdata->dev, "Select link list to program using curr_list\n");
		ret = -EINVAL;
		goto err;
	}

	if (list_empty(&drvdata->cfg_head[drvdata->curr_list])) {
		dev_err(drvdata->dev, "DCC: No read address programmed\n");
		ret = -EPERM;
		goto err;
	}

	entry = devm_kzalloc(drvdata->dev, sizeof(*entry), GFP_KERNEL);
	if (!entry) {
		ret = -ENOMEM;
		goto err;
	}

	entry->desc_type = DCC_READ_WRITE_TYPE;
	entry->mask = mask;
	entry->write_val = val;
	entry->index = drvdata->nr_config[drvdata->curr_list]++;
	INIT_LIST_HEAD(&entry->list);
	list_add_tail(&entry->list, &drvdata->cfg_head[drvdata->curr_list]);
err:
	mutex_unlock(&drvdata->mutex);
	return ret;
}

static ssize_t rd_mod_wr_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t size)
{
	int ret;
	int nval;
	unsigned int mask, val;
	struct dcc_drvdata *drvdata = dev_get_drvdata(dev);

	nval = sscanf(buf, "%x %x", &mask, &val);

	if (nval <= 1 || nval > 2)
		return -EINVAL;

	ret = dcc_rd_mod_wr_add(drvdata, mask, val);
	if (ret)
		return ret;

	return size;

}

static DEVICE_ATTR_WO(rd_mod_wr);

static int dcc_add_write(struct dcc_drvdata *drvdata, unsigned int addr,
				unsigned int write_val, int apb_bus)
{
	struct dcc_config_entry *entry;

	entry = devm_kzalloc(drvdata->dev, sizeof(*entry), GFP_KERNEL);
	if (!entry)
		return -ENOMEM;

	entry->desc_type = DCC_WRITE_TYPE;
	entry->base = addr & GENMASK(31, 4);
	entry->offset = addr - entry->base;
	entry->write_val = write_val;
	entry->index = drvdata->nr_config[drvdata->curr_list]++;
	entry->len = 1;
	entry->apb_bus = apb_bus;
	INIT_LIST_HEAD(&entry->list);
	list_add_tail(&entry->list, &drvdata->cfg_head[drvdata->curr_list]);

	return 0;
}

static ssize_t config_write_store(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t size)
{
	int ret;
	int nval;
	unsigned int addr, write_val;
	int apb_bus = 0;
	struct dcc_drvdata *drvdata = dev_get_drvdata(dev);

	mutex_lock(&drvdata->mutex);

	nval = sscanf(buf, "%x %x %d", &addr, &write_val, &apb_bus);

	if (nval <= 1 || nval > 3) {
		ret = -EINVAL;
		goto err;
	}

	if (drvdata->curr_list >= drvdata->nr_link_list) {
		dev_err(dev, "Select link list to program using curr_list\n");
		ret = -EINVAL;
		goto err;
	}

	if (nval == 3 && apb_bus != 0)
		apb_bus = 1;

	ret = dcc_add_write(drvdata, addr, write_val, apb_bus);
	if (ret)
		goto err;

	mutex_unlock(&drvdata->mutex);
	return size;
err:
	mutex_unlock(&drvdata->mutex);
	return ret;
}

static DEVICE_ATTR_WO(config_write);

static const struct device_attribute *dcc_attrs[] = {
	&dev_attr_trigger,
	&dev_attr_enable,
	&dev_attr_config,
	&dev_attr_config_reset,
	&dev_attr_ready,
	&dev_attr_interrupt_disable,
	&dev_attr_loop,
	&dev_attr_rd_mod_wr,
	&dev_attr_curr_list,
	&dev_attr_config_write,
	NULL,
};

static int dcc_create_files(struct device *dev,
					const struct device_attribute **attrs)
{
	int ret = 0, i;

	for (i = 0; attrs[i] != NULL; i++) {
		ret = device_create_file(dev, attrs[i]);
		if (ret) {
			dev_err(dev, "DCC: Couldn't create sysfs attribute: %s\n",
				attrs[i]->attr.name);
			break;
		}
	}
	return ret;
}

static int dcc_sram_open(struct inode *inode, struct file *file)
{
	struct dcc_drvdata *drvdata = container_of(inode->i_cdev,
		struct dcc_drvdata,
		sram_dev);
	file->private_data = drvdata;

	return	0;
}

static ssize_t dcc_sram_read(struct file *file, char __user *data,
						size_t len, loff_t *ppos)
{
	unsigned char *buf;
	struct dcc_drvdata *drvdata = file->private_data;

	/* EOF check */
	if (drvdata->ram_size <= *ppos)
		return 0;

	if ((*ppos + len) > drvdata->ram_size)
		len = (drvdata->ram_size - *ppos);

	buf = kzalloc(len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	memcpy_fromio(buf, drvdata->ram_base + *ppos, len);

	if (copy_to_user(data, buf, len)) {
		dev_err(drvdata->dev, "DCC: Couldn't copy all data to user\n");
		kfree(buf);
		return -EFAULT;
	}

	*ppos += len;

	kfree(buf);

	return len;
}

static const struct file_operations dcc_sram_fops = {
	.owner		= THIS_MODULE,
	.open		= dcc_sram_open,
	.read		= dcc_sram_read,
	.llseek		= no_llseek,
};

static int dcc_sram_dev_register(struct dcc_drvdata *drvdata)
{
	int ret;
	struct device *device;
	dev_t dev;

	ret = alloc_chrdev_region(&dev, 0, 1, DCC_SRAM_NODE);
	if (ret)
		goto err_alloc;

	cdev_init(&drvdata->sram_dev, &dcc_sram_fops);

	drvdata->sram_dev.owner = THIS_MODULE;
	ret = cdev_add(&drvdata->sram_dev, dev, 1);
	if (ret)
		goto err_cdev_add;

	drvdata->sram_class = class_create(THIS_MODULE, DCC_SRAM_NODE);
	if (IS_ERR(drvdata->sram_class)) {
		ret = PTR_ERR(drvdata->sram_class);
		goto err_class_create;
	}

	device = device_create(drvdata->sram_class, NULL,
						drvdata->sram_dev.dev, drvdata,
						DCC_SRAM_NODE);
	if (IS_ERR(device)) {
		ret = PTR_ERR(device);
		goto err_dev_create;
	}

	return 0;
err_dev_create:
	class_destroy(drvdata->sram_class);
err_class_create:
	cdev_del(&drvdata->sram_dev);
err_cdev_add:
	unregister_chrdev_region(drvdata->sram_dev.dev, 1);
err_alloc:
	return ret;
}

static void dcc_sram_dev_deregister(struct dcc_drvdata *drvdata)
{
	device_destroy(drvdata->sram_class, drvdata->sram_dev.dev);
	class_destroy(drvdata->sram_class);
	cdev_del(&drvdata->sram_dev);
	unregister_chrdev_region(drvdata->sram_dev.dev, 1);
}

static int dcc_sram_dev_init(struct dcc_drvdata *drvdata)
{
	int ret = 0;

	ret = dcc_sram_dev_register(drvdata);
	if (ret)
		dev_err(drvdata->dev, "DCC: sram node not registered.\n");

	return ret;
}

static void dcc_sram_dev_exit(struct dcc_drvdata *drvdata)
{
	dcc_sram_dev_deregister(drvdata);
}

static int dcc_probe(struct platform_device *pdev)
{
	u32 val;
	int ret = 0, i, enable_size, nr_config_size, cfg_head_size;
	struct device *dev = &pdev->dev;
	struct dcc_drvdata *dcc;
	struct resource *res;
	const struct qcom_dcc_config *cfg;

	dcc = devm_kzalloc(dev, sizeof(*dcc), GFP_KERNEL);
	if (!dcc)
		return -ENOMEM;

	dcc->dev = &pdev->dev;
	platform_set_drvdata(pdev, dcc);

	dcc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dcc->base))
		return PTR_ERR(dcc->base);

	dcc->ram_base = devm_platform_get_and_ioremap_resource(pdev, 1, &res);
	if (IS_ERR(dcc->ram_base))
		return PTR_ERR(dcc->ram_base);

	dcc->ram_size = resource_size(res);

	dcc->ram_base = devm_ioremap(dev, res->start, resource_size(res));
	if (!dcc->ram_base)
		return -ENOMEM;

	cfg = of_device_get_match_data(&pdev->dev);
	dcc->ram_offset = cfg->dcc_ram_offset;

	val = dcc_readl(dcc, DCC_HW_INFO);

	if (FIELD_GET(BIT(DCC_VER_INFO_BIT), val)) {
		dcc->mem_map_ver = DCC_MEM_MAP_VER3;
		dcc->nr_link_list = dcc_readl(dcc, DCC_LL_NUM_INFO);
		if (dcc->nr_link_list == 0)
			return	-EINVAL;
	} else if ((val & DCC_VER_MASK2) == DCC_VER_MASK2) {
		dcc->mem_map_ver = DCC_MEM_MAP_VER2;
		dcc->nr_link_list = dcc_readl(dcc, DCC_LL_NUM_INFO);
		if (dcc->nr_link_list == 0)
			return	-EINVAL;
	} else {
		dcc->mem_map_ver = DCC_MEM_MAP_VER1;
		dcc->nr_link_list = DCC_MAX_LINK_LIST;
	}

	if ((val & BIT(6)) == BIT(6))
		dcc->loopoff = DCC_FIX_LOOP_OFFSET;
	else
		dcc->loopoff = get_bitmask_order((dcc->ram_size +
				dcc->ram_offset) / 4 - 1);

	mutex_init(&dcc->mutex);
	/* Allocate space for all entries at once */
	enable_size = dcc->nr_link_list * sizeof(bool);
	nr_config_size = dcc->nr_link_list * sizeof(size_t);
	cfg_head_size = dcc->nr_link_list * sizeof(struct list_head);

	dcc->enable = devm_kzalloc(dev, enable_size + nr_config_size + cfg_head_size, GFP_KERNEL);
	if (!dcc->enable)
		return -ENOMEM;

	dcc->nr_config  = (size_t *)(dcc->enable + dcc->nr_link_list);
	dcc->cfg_head = (struct list_head *)(dcc->nr_config + dcc->nr_link_list);

	for (i = 0; i < dcc->nr_link_list; i++)
		INIT_LIST_HEAD(&dcc->cfg_head[i]);

	dcc->curr_list = DCC_INVALID_LINK_LIST;
	ret = dcc_sram_dev_init(dcc);
	if (ret)
		return ret;

	return dcc_create_files(dev, dcc_attrs);
}

static int dcc_remove(struct platform_device *pdev)
{
	struct dcc_drvdata *drvdata = platform_get_drvdata(pdev);

	dcc_sram_dev_exit(drvdata);

	dcc_config_reset(drvdata);

	return 0;
}

static const struct qcom_dcc_config sm8150_cfg = {
	.dcc_ram_offset	= 0x5000,
};

static const struct qcom_dcc_config sc7280_cfg = {
	.dcc_ram_offset = 0x12000,
};

static const struct qcom_dcc_config sc7180_cfg = {
	.dcc_ram_offset = 0x6000,
};

static const struct qcom_dcc_config sdm845_cfg = {
	.dcc_ram_offset = 0x6000,
};

static const struct of_device_id dcc_match_table[] = {
	{ .compatible = "qcom,sm8150-dcc", .data = &sm8150_cfg },
	{ .compatible = "qcom,sc7280-dcc", .data = &sc7280_cfg },
	{ .compatible = "qcom,sc7180-dcc", .data = &sc7180_cfg },
	{ .compatible = "qcom,sdm845-dcc", .data = &sdm845_cfg },
	{ }
};
MODULE_DEVICE_TABLE(of, dcc_match_table);

static struct platform_driver dcc_driver = {
	.probe = dcc_probe,
	.remove	= dcc_remove,
	.driver	= {
		.name = "qcom-dcc",
		.of_match_table	= dcc_match_table,
	},
};

module_platform_driver(dcc_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Qualcomm Technologies Inc. DCC driver");

