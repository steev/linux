/*
 * Copyright (c) 2012 Sascha Hauer <s.hauer@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */
#include <linux/types.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/bitrev.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>

#include "imx-ipu-v3.h"
#include "ipu-prv.h"

enum {
	IC_TASK_VIEWFINDER,
	IC_TASK_ENCODER,
	IC_TASK_POST_PROCESSOR,
	IC_TASK_MAX,
};

struct image_convert_ctx {
	void (*complete)(void *ctx, int err);
	void *complete_context;

	struct list_head list;
	struct ipu_image in;
	struct ipu_image in_n;
	struct ipu_image in_p;
	struct ipu_image out;

	void *freep;

	unsigned int rotate:1;
	unsigned int deinterlace:1;
};

struct ipu_ic_task {
	int id;
	struct ipu_ic_priv *priv;
	void __iomem *reg_rsc;
	u32 csc1_mask;
	u32 csc2_mask;
	u32 cmb_mask;
	void __iomem *tpm_base_csc1;
	void __iomem *tpm_base_csc2;
	u32 ic_conf_rot_en;
	u32 ic_conf_en;
	int first_deinterlace_frame;
	struct ipuv3_channel *input_channel_p;
	struct ipuv3_channel *input_channel;
	struct ipuv3_channel *input_channel_n;
	struct ipuv3_channel *output_channel;
	struct ipuv3_channel *rotation_input_channel;
	struct ipuv3_channel *rotation_output_channel;

	struct list_head image_list;

	struct workqueue_struct *workqueue;
	struct work_struct work;
	struct completion complete;
};

struct ipu_ic_priv {
	void __iomem *ic_base;
	void __iomem *tpm_base;
	void __iomem *vdi_base;
	struct device *dev;
	struct ipu_soc *ipu;
	struct mutex mutex;
	struct ipu_ic_task task[IC_TASK_MAX];
	spinlock_t lock;
	int ic_usecount;
};

#define IC_CONF			0x0
#define IC_PRP_ENC_RSC		0x4
#define IC_PRP_VF_RSC		0x8
#define IC_PP_RSC		0xC
#define IC_CMBP_1		0x10
#define IC_CMBP_2		0x14
#define IC_IDMAC_1		0x18
#define IC_IDMAC_2		0x1C
#define IC_IDMAC_3		0x20
#define IC_IDMAC_4		0x24

/* Image Converter Register bits */
#define	IC_CONF_PRPENC_EN	0x00000001
#define	IC_CONF_PRPENC_CSC1	0x00000002
#define	IC_CONF_PRPENC_ROT_EN	0x00000004
#define	IC_CONF_PRPVF_EN	0x00000100
#define	IC_CONF_PRPVF_CSC1	0x00000200
#define	IC_CONF_PRPVF_CSC2	0x00000400
#define	IC_CONF_PRPVF_CMB	0x00000800
#define	IC_CONF_PRPVF_ROT_EN	0x00001000
#define	IC_CONF_PP_EN		0x00010000
#define	IC_CONF_PP_CSC1		0x00020000
#define	IC_CONF_PP_CSC2		0x00040000
#define	IC_CONF_PP_CMB		0x00080000
#define	IC_CONF_PP_ROT_EN	0x00100000
#define	IC_CONF_IC_GLB_LOC_A	0x10000000
#define	IC_CONF_KEY_COLOR_EN	0x20000000
#define	IC_CONF_RWS_EN		0x40000000
#define	IC_CONF_CSI_MEM_WR_EN	0x80000000

struct ipu_ic_task_template {
	unsigned long ofs_rsc;
	u32 csc1_mask;
	u32 csc2_mask;
	u32 cmb_mask;
	u32 tpm_ofs_csc1;
	u32 tpm_ofs_csc2;
	int rotation_input_channel;
	int rotation_output_channel;
	int input_channel_p;
	int input_channel;
	int input_channel_n;
	int output_channel;
	u32 ic_conf_rot_en;
	u32 ic_conf_en;
};

struct ipu_ic_task_template templates[] = {
	{
		.ofs_rsc = IC_PRP_VF_RSC,
		.csc1_mask = IC_CONF_PRPVF_CSC1,
		.csc2_mask = IC_CONF_PRPVF_CSC2,
		.cmb_mask = IC_CONF_PRPVF_CMB,
		.tpm_ofs_csc1 = 0x4028,
		.tpm_ofs_csc2 = 0x4040,
		.rotation_input_channel = 46,
		.rotation_output_channel = 49,
		.input_channel_p = 8,
		.input_channel = 9,
		.input_channel_n = 10,
		.output_channel = 21,
		.ic_conf_rot_en = IC_CONF_PRPVF_ROT_EN,
		.ic_conf_en = IC_CONF_PRPVF_EN,
	}, {
		.ofs_rsc = IC_PRP_ENC_RSC,
		.csc1_mask = IC_CONF_PRPENC_CSC1,
		.csc2_mask = 0,
		.cmb_mask = 0,
		.tpm_ofs_csc1 = 0x2008,
		.tpm_ofs_csc2 = 0x0,
		.rotation_input_channel = 45,
		.rotation_output_channel = 48,
		.input_channel = 12,
		.output_channel = 20,
		.ic_conf_rot_en = IC_CONF_PRPENC_ROT_EN,
		.ic_conf_en = IC_CONF_PRPENC_EN,
	}, {
		.ofs_rsc = IC_PP_RSC,
		.csc1_mask = IC_CONF_PP_CSC1,
		.csc2_mask = IC_CONF_PP_CSC2,
		.cmb_mask = IC_CONF_PP_CMB,
		.tpm_ofs_csc1 = 0x6060,
		.tpm_ofs_csc2 = 0x6078,
		.rotation_input_channel = 47,
		.rotation_output_channel = 50,
		.input_channel = 11,
		.output_channel = 22,
		.ic_conf_rot_en = IC_CONF_PP_ROT_EN,
		.ic_conf_en = IC_CONF_PP_EN,
	},
};

#define	IC_IDMAC_1_CB0_BURST_16 0x00000001
#define	IC_IDMAC_1_CB1_BURST_16 0x00000002
#define	IC_IDMAC_1_CB2_BURST_16 0x00000004
#define	IC_IDMAC_1_CB3_BURST_16 0x00000008
#define	IC_IDMAC_1_CB4_BURST_16 0x00000010
#define	IC_IDMAC_1_CB5_BURST_16 0x00000020
#define	IC_IDMAC_1_CB6_BURST_16 0x00000040
#define	IC_IDMAC_1_CB7_BURST_16 0x00000080
#define	IC_IDMAC_1_PRPENC_ROT_MASK 0x00003800
#define	IC_IDMAC_1_PRPENC_ROT_OFFSET 11
#define	IC_IDMAC_1_PRPVF_ROT_MASK 0x0001C000
#define	IC_IDMAC_1_PRPVF_ROT_OFFSET 14
#define	IC_IDMAC_1_PP_ROT_MASK 0x000E0000
#define	IC_IDMAC_1_PP_ROT_OFFSET 17
#define	IC_IDMAC_1_PP_FLIP_RS 0x00400000
#define	IC_IDMAC_1_PRPVF_FLIP_RS 0x00200000
#define	IC_IDMAC_1_PRPENC_FLIP_RS 0x00100000

#define	IC_IDMAC_2_PRPENC_HEIGHT_MASK 0x000003FF
#define	IC_IDMAC_2_PRPENC_HEIGHT_OFFSET 0
#define	IC_IDMAC_2_PRPVF_HEIGHT_MASK 0x000FFC00
#define	IC_IDMAC_2_PRPVF_HEIGHT_OFFSET 10
#define	IC_IDMAC_2_PP_HEIGHT_MASK 0x3FF00000
#define	IC_IDMAC_2_PP_HEIGHT_OFFSET 20

#define	IC_IDMAC_3_PRPENC_WIDTH_MASK 0x000003FF
#define	IC_IDMAC_3_PRPENC_WIDTH_OFFSET 0
#define	IC_IDMAC_3_PRPVF_WIDTH_MASK 0x000FFC00
#define	IC_IDMAC_3_PRPVF_WIDTH_OFFSET 10
#define	IC_IDMAC_3_PP_WIDTH_MASK 0x3FF00000
#define	IC_IDMAC_3_PP_WIDTH_OFFSET 20

typedef enum {
        /* Note the enum values correspond to BAM value */
        IPU_ROTATE_NONE = 0,
        IPU_ROTATE_VERT_FLIP = 1,
        IPU_ROTATE_HORIZ_FLIP = 2,
        IPU_ROTATE_180 = 3,
        IPU_ROTATE_90_RIGHT = 4,
        IPU_ROTATE_90_RIGHT_VFLIP = 5,
        IPU_ROTATE_90_RIGHT_HFLIP = 6,
        IPU_ROTATE_90_LEFT = 7,
} ipu_rotate_mode_t;

struct ic_setup {
	u32 in_width;
	u32 in_height;
	enum ipu_color_space in_fmt;
	u32 out_width;
	u32 out_height;
	enum ipu_color_space out_fmt;
	bool graphics_combine_en;
	enum ipu_color_space in_g_fmt;
};

static u32 ipu_vdi_read(struct ipu_ic_priv *priv, unsigned offset)
{
	return readl(priv->vdi_base + offset);
}

static void ipu_vdi_write(struct ipu_ic_priv *priv, u32 value, unsigned offset)
{
	writel(value, priv->vdi_base + offset);
}

static void ipu_ic_enable(struct ipu_ic_priv *priv)
{
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);

	if (!priv->ic_usecount)
		ipu_module_enable(priv->ipu, IPU_CONF_IC_EN);

	priv->ic_usecount++;

	spin_unlock_irqrestore(&priv->lock, flags);
}

static void ipu_ic_disable(struct ipu_ic_priv *priv)
{
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);

	priv->ic_usecount--;

	BUG_ON(priv->ic_usecount < 0);

	if (!priv->ic_usecount)
		ipu_module_disable(priv->ipu, IPU_CONF_IC_EN);

	spin_unlock_irqrestore(&priv->lock, flags);
}

static int ipu_ic_enable_task(struct ipu_ic_task *task, struct image_convert_ctx *ctx)
{
	struct ipu_ic_priv *priv = task->priv;
	u32 ic_conf;

	ipu_idmac_enable_channel(task->input_channel);
	if (ctx->deinterlace) {
		ipu_idmac_enable_channel(task->input_channel_n);
		ipu_idmac_enable_channel(task->input_channel_p);
	}
	ipu_idmac_enable_channel(task->output_channel);

	ic_conf = readl(priv->ic_base + IC_CONF);

	if (ctx->rotate)
		ic_conf |= task->ic_conf_rot_en;
	ic_conf |= task->ic_conf_en;

	writel(ic_conf, priv->ic_base + IC_CONF);

	ipu_idmac_select_buffer(task->input_channel, 0);
	if (ctx->deinterlace) {
		ipu_idmac_select_buffer(task->input_channel_n, 0);
		ipu_idmac_select_buffer(task->input_channel_p, 0);
	}
	ipu_idmac_select_buffer(task->output_channel, 0);

	if (ctx->deinterlace && task->first_deinterlace_frame) {
		/*
		 * With the very first frame we have to trigger the vdic input
		 * channel twice. I have no idea why. This may be a bug in the
		 * IPU or in the code.
		 */
		mdelay(5);
		ipu_idmac_select_buffer(task->input_channel, 0);
		task->first_deinterlace_frame = 0;
	}

	return 0;
}

static int ipu_ic_disable_task(struct ipu_ic_task *task, struct image_convert_ctx *ctx)
{
	struct ipu_ic_priv *priv = task->priv;
	u32 ic_conf;

	ic_conf = readl(priv->ic_base + IC_CONF);

	if (ctx->rotate)
		ic_conf &= ~task->ic_conf_rot_en;
	ic_conf &= ~task->ic_conf_en;

	writel(ic_conf, priv->ic_base + IC_CONF);

	ipu_idmac_disable_channel(task->input_channel);
	if (ctx->deinterlace) {
		ipu_idmac_disable_channel(task->input_channel_n);
		ipu_idmac_disable_channel(task->input_channel_p);
	}
	ipu_idmac_disable_channel(task->output_channel);

	return 0;
}

static int ipu_ic_init_csc(struct ipu_ic_task *task, enum ipu_color_space in_format,
		      enum ipu_color_space out_format, int csc_index)
{
	struct ipu_ic_priv *priv = task->priv;

	/*
	 * Y = R *  .299 + G *  .587 + B *  .114
	 * U = R * -.169 + G * -.332 + B *  .500 + 128.
	 * V = R *  .500 + G * -.419 + B * -.0813 + 128.
	 */
	static const u32 rgb2ycbcr_coeff[4][3] = {
		{0x004D, 0x0096, 0x001D},
		{0x01D5, 0x01AB, 0x0080},
		{0x0080, 0x0195, 0x01EB},
		{0x0000, 0x0200, 0x0200},	/* A0, A1, A2 */
	};

	/*
	 * transparent IPUV3_COLORSPACE_RGB -> IPUV3_COLORSPACE_RGB matrix
	 * for combining
	 */
	static const u32 rgb2rgb_coeff[4][3] = {
		{0x0080, 0x0000, 0x0000},
		{0x0000, 0x0080, 0x0000},
		{0x0000, 0x0000, 0x0080},
		{0x0000, 0x0000, 0x0000},	/* A0, A1, A2 */
	};

	/*
	 * R = (1.164 * (Y - 16)) + (1.596 * (Cr - 128))
	 * G = (1.164 * (Y - 16)) - (0.392 * (Cb - 128)) - (0.813 * (Cr - 128))
	 * B = (1.164 * (Y - 16)) + (2.017 * (Cb - 128)
	 */
	static const u32 ycbcr2rgb_coeff[4][3] = {
		{149, 0, 204},
		{149, 462, 408},
		{149, 255, 0},
		{8192 - 446, 266, 8192 - 554},	/* A0, A1, A2 */
	};

	u32 param;
	u32 *base = NULL;

	if (csc_index == 1)
		base = task->tpm_base_csc1;
	else
		base = task->tpm_base_csc2;

	if ((in_format == IPUV3_COLORSPACE_YUV) && (out_format == IPUV3_COLORSPACE_RGB)) {
		/* Init CSC (IPUV3_COLORSPACE_YUV->IPUV3_COLORSPACE_RGB) */
		param = (ycbcr2rgb_coeff[3][0] << 27) |
			(ycbcr2rgb_coeff[0][0] << 18) |
			(ycbcr2rgb_coeff[1][1] << 9) | ycbcr2rgb_coeff[2][2];
		writel(param, base++);
		/* scale = 2, sat = 0 */
		param = (ycbcr2rgb_coeff[3][0] >> 5) | (2L << (40 - 32));
		writel(param, base++);

		param = (ycbcr2rgb_coeff[3][1] << 27) |
			(ycbcr2rgb_coeff[0][1] << 18) |
			(ycbcr2rgb_coeff[1][0] << 9) | ycbcr2rgb_coeff[2][0];
		writel(param, base++);
		param = (ycbcr2rgb_coeff[3][1] >> 5);
		writel(param, base++);

		param = (ycbcr2rgb_coeff[3][2] << 27) |
			(ycbcr2rgb_coeff[0][2] << 18) |
			(ycbcr2rgb_coeff[1][2] << 9) | ycbcr2rgb_coeff[2][1];
		writel(param, base++);
		param = (ycbcr2rgb_coeff[3][2] >> 5);
		writel(param, base++);
	} else if ((in_format == IPUV3_COLORSPACE_RGB) && (out_format == IPUV3_COLORSPACE_YUV)) {
		/* Init CSC (IPUV3_COLORSPACE_RGB->IPUV3_COLORSPACE_YUV) */
		param = (rgb2ycbcr_coeff[3][0] << 27) |
			(rgb2ycbcr_coeff[0][0] << 18) |
			(rgb2ycbcr_coeff[1][1] << 9) | rgb2ycbcr_coeff[2][2];
		writel(param, base++);
		/* scale = 1, sat = 0 */
		param = (rgb2ycbcr_coeff[3][0] >> 5) | (1UL << 8);
		writel(param, base++);

		param = (rgb2ycbcr_coeff[3][1] << 27) |
			(rgb2ycbcr_coeff[0][1] << 18) |
			(rgb2ycbcr_coeff[1][0] << 9) | rgb2ycbcr_coeff[2][0];
		writel(param, base++);
		param = (rgb2ycbcr_coeff[3][1] >> 5);
		writel(param, base++);

		param = (rgb2ycbcr_coeff[3][2] << 27) |
			(rgb2ycbcr_coeff[0][2] << 18) |
			(rgb2ycbcr_coeff[1][2] << 9) | rgb2ycbcr_coeff[2][1];
		writel(param, base++);
		param = (rgb2ycbcr_coeff[3][2] >> 5);
		writel(param, base++);
	} else if ((in_format == IPUV3_COLORSPACE_RGB) && (out_format == IPUV3_COLORSPACE_RGB)) {
		/* Init CSC */
		param =
		    (rgb2rgb_coeff[3][0] << 27) | (rgb2rgb_coeff[0][0] << 18) |
		    (rgb2rgb_coeff[1][1] << 9) | rgb2rgb_coeff[2][2];
		writel(param, base++);
		/* scale = 2, sat = 0 */
		param = (rgb2rgb_coeff[3][0] >> 5) | (2UL << 8);
		writel(param, base++);

		param =
		    (rgb2rgb_coeff[3][1] << 27) | (rgb2rgb_coeff[0][1] << 18) |
		    (rgb2rgb_coeff[1][0] << 9) | rgb2rgb_coeff[2][0];
		writel(param, base++);
		param = (rgb2rgb_coeff[3][1] >> 5);
		writel(param, base++);

		param =
		    (rgb2rgb_coeff[3][2] << 27) | (rgb2rgb_coeff[0][2] << 18) |
		    (rgb2rgb_coeff[1][2] << 9) | rgb2rgb_coeff[2][1];
		writel(param, base++);
		param = (rgb2rgb_coeff[3][2] >> 5);
		writel(param, base++);
	} else {
		dev_err(priv->dev, "Unsupported color space conversion\n");
		return -EINVAL;
	}

	return 0;
}

static int ipu_ic_calc_resize_coeffs(struct ipu_ic_priv *priv, u32 inSize, u32 outSize,
				u32 *resizeCoeff,
				u32 *downsizeCoeff)
{
	u32 tempSize;
	u32 tempDownsize;

	/* Input size cannot be more than 4096 */
	/* Output size cannot be more than 1024 */
	if ((inSize > 4096) || (outSize > 1024))
		return -EINVAL;

	/* Cannot downsize more than 8:1 */
	if ((outSize << 3) < inSize)
		return -EINVAL;

	/* Compute downsizing coefficient */
	/* Output of downsizing unit cannot be more than 1024 */
	tempDownsize = 0;
	tempSize = inSize;
	while (((tempSize > 1024) || (tempSize >= outSize * 2)) &&
	       (tempDownsize < 2)) {
		tempSize >>= 1;
		tempDownsize++;
	}
	*downsizeCoeff = tempDownsize;

	/* compute resizing coefficient using the following equation:
	   resizeCoeff = M*(SI -1)/(SO - 1)
	   where M = 2^13, SI - input size, SO - output size    */
	*resizeCoeff = (8192L * (tempSize - 1)) / (outSize - 1);
	if (*resizeCoeff >= 16384L) {
		dev_err(priv->dev, "Warning! Overflow on resize coeff.\n");
		*resizeCoeff = 0x3FFF;
	}

	dev_dbg(priv->dev, "resizing from %u -> %u pixels, "
		"downsize=%u, resize=%u.%lu (reg=%u)\n", inSize, outSize,
		*downsizeCoeff, (*resizeCoeff >= 8192L) ? 1 : 0,
		((*resizeCoeff & 0x1FFF) * 10000L) / 8192L, *resizeCoeff);

	return 0;
}

static int ipu_ic_init_task(struct ipu_ic_task *task, struct ic_setup *p, bool src_is_csi)
{
	struct ipu_ic_priv *priv = task->priv;
	u32 reg, ic_conf;
	u32 downsizeCoeff, resizeCoeff;
	int ret;

	/* Setup vertical resizing */
	ret = ipu_ic_calc_resize_coeffs(priv, p->in_height,
			    p->out_height,
			    &resizeCoeff, &downsizeCoeff);
	if (ret)
		return ret;
	reg = (downsizeCoeff << 30) | (resizeCoeff << 16);

	/* Setup horizontal resizing */
	/* Upadeted for IC split case */
	ret = ipu_ic_calc_resize_coeffs(priv, p->in_width,
			p->out_width,
			&resizeCoeff, &downsizeCoeff);
	if (ret)
		return ret;
	reg |= (downsizeCoeff << 14) | resizeCoeff;

	writel(reg, task->reg_rsc);

	ic_conf = readl(priv->ic_base + IC_CONF);
	ic_conf &= ~task->cmb_mask;

	/* Setup color space conversion */
	if (p->in_fmt != p->out_fmt) {
		ret = ipu_ic_init_csc(task, p->in_fmt, p->out_fmt, 1);
		if (ret)
			return ret;
		ic_conf |= task->csc1_mask;
	} else {
		ic_conf &= ~task->csc1_mask;
	}

	if (p->graphics_combine_en) {
		ic_conf |= task->cmb_mask;

		if (!(ic_conf & task->csc1_mask)) {
			/* need transparent CSC1 conversion */
			ipu_ic_init_csc(task, IPUV3_COLORSPACE_RGB, IPUV3_COLORSPACE_RGB, 1);
			/* Enable IPUV3_COLORSPACE_RGB -> IPUV3_COLORSPACE_RGB CSC */
			ic_conf |= task->csc1_mask;
		}
		if (p->in_g_fmt != p->out_fmt) {
			ret = ipu_ic_init_csc(task, p->in_g_fmt, p->out_fmt, 2);
			if (ret)
				return ret;
			ic_conf |= task->csc2_mask;
		}
	}

	writel(ic_conf, priv->ic_base + IC_CONF);

	return 0;
}

static int ipu_ic_idma_init(struct ipu_ic_task *task, int dma_chan, u16 width, u16 height,
		      int burst_size, ipu_rotate_mode_t rot)
{
	struct ipu_ic_priv *priv = task->priv;
	u32 ic_idmac_1, ic_idmac_2, ic_idmac_3;
	u32 temp_rot = bitrev8(rot) >> 5;
	bool need_hor_flip = false;

	if ((burst_size != 8) && (burst_size != 16)) {
		dev_dbg(priv->dev, "Illegal burst length for IC\n");
		return -EINVAL;
	}

	width--;
	height--;

	if (temp_rot & 0x2)	/* Need horizontal flip */
		need_hor_flip = true;

	ic_idmac_1 = readl(priv->ic_base + IC_IDMAC_1);
	ic_idmac_2 = readl(priv->ic_base + IC_IDMAC_2);
	ic_idmac_3 = readl(priv->ic_base + IC_IDMAC_3);
	switch (dma_chan) {
	case 22:	/* PP output - CB2 */
		if (burst_size == 16)
			ic_idmac_1 |= IC_IDMAC_1_CB2_BURST_16;
		else
			ic_idmac_1 &= ~IC_IDMAC_1_CB2_BURST_16;

		if (need_hor_flip)
			ic_idmac_1 |= IC_IDMAC_1_PP_FLIP_RS;
		else
			ic_idmac_1 &= ~IC_IDMAC_1_PP_FLIP_RS;

		ic_idmac_2 &= ~IC_IDMAC_2_PP_HEIGHT_MASK;
		ic_idmac_2 |= height << IC_IDMAC_2_PP_HEIGHT_OFFSET;

		ic_idmac_3 &= ~IC_IDMAC_3_PP_WIDTH_MASK;
		ic_idmac_3 |= width << IC_IDMAC_3_PP_WIDTH_OFFSET;
		break;
	case 11:	/* PP Input - CB5 */
		if (burst_size == 16)
			ic_idmac_1 |= IC_IDMAC_1_CB5_BURST_16;
		else
			ic_idmac_1 &= ~IC_IDMAC_1_CB5_BURST_16;
		break;
	case 47:	/* PP Rot input */
		ic_idmac_1 &= ~IC_IDMAC_1_PP_ROT_MASK;
		ic_idmac_1 |= temp_rot << IC_IDMAC_1_PP_ROT_OFFSET;
		break;
	case 12: /* PRP Input - CB6 */
		if (burst_size == 16)
			ic_idmac_1 |= IC_IDMAC_1_CB6_BURST_16;
		else
			ic_idmac_1 &= ~IC_IDMAC_1_CB6_BURST_16;
		break;
	case 20:	/* PRP ENC output - CB0 */
		if (burst_size == 16)
			ic_idmac_1 |= IC_IDMAC_1_CB0_BURST_16;
		else
			ic_idmac_1 &= ~IC_IDMAC_1_CB0_BURST_16;

		if (need_hor_flip)
			ic_idmac_1 |= IC_IDMAC_1_PRPENC_FLIP_RS;
		else
			ic_idmac_1 &= ~IC_IDMAC_1_PRPENC_FLIP_RS;

		ic_idmac_2 &= ~IC_IDMAC_2_PRPENC_HEIGHT_MASK;
		ic_idmac_2 |= height << IC_IDMAC_2_PRPENC_HEIGHT_OFFSET;

		ic_idmac_3 &= ~IC_IDMAC_3_PRPENC_WIDTH_MASK;
		ic_idmac_3 |= width << IC_IDMAC_3_PRPENC_WIDTH_OFFSET;
		break;
	case 45:	/* PRP ENC Rot input */
		ic_idmac_1 &= ~IC_IDMAC_1_PRPENC_ROT_MASK;
		ic_idmac_1 |= temp_rot << IC_IDMAC_1_PRPENC_ROT_OFFSET;
		break;

	case 21:	/* PRP VF output - CB1 */
		if (burst_size == 16)
			ic_idmac_1 |= IC_IDMAC_1_CB1_BURST_16;
		else
			ic_idmac_1 &= ~IC_IDMAC_1_CB1_BURST_16;

		if (need_hor_flip)
			ic_idmac_1 |= IC_IDMAC_1_PRPVF_FLIP_RS;
		else
			ic_idmac_1 &= ~IC_IDMAC_1_PRPVF_FLIP_RS;

		ic_idmac_2 &= ~IC_IDMAC_2_PRPVF_HEIGHT_MASK;
		ic_idmac_2 |= height << IC_IDMAC_2_PRPVF_HEIGHT_OFFSET;

		ic_idmac_3 &= ~IC_IDMAC_3_PRPVF_WIDTH_MASK;
		ic_idmac_3 |= width << IC_IDMAC_3_PRPVF_WIDTH_OFFSET;
		break;

	case 46:	/* PRP VF Rot input */
		ic_idmac_1 &= ~IC_IDMAC_1_PRPVF_ROT_MASK;
		ic_idmac_1 |= temp_rot << IC_IDMAC_1_PRPVF_ROT_OFFSET;
		break;
	case 14:	/* PRP VF graphics combining input - CB3 */
		if (burst_size == 16)
			ic_idmac_1 |= IC_IDMAC_1_CB3_BURST_16;
		else
			ic_idmac_1 &= ~IC_IDMAC_1_CB3_BURST_16;
		break;
	case 15:	/* PP graphics combining input - CB4 */
		if (burst_size == 16)
			ic_idmac_1 |= IC_IDMAC_1_CB4_BURST_16;
		else
			ic_idmac_1 &= ~IC_IDMAC_1_CB4_BURST_16;
		break;
	}

	writel(ic_idmac_1, priv->ic_base + IC_IDMAC_1);
	writel(ic_idmac_2, priv->ic_base + IC_IDMAC_2);
	writel(ic_idmac_3, priv->ic_base + IC_IDMAC_3);

	return 0;
}

static struct image_convert_ctx *ipu_image_convert_next(struct ipu_ic_task *task)
{
	struct ipu_ic_priv *priv = task->priv;
	struct ipu_ch_param *cpmem_in = ipu_get_cpmem(task->input_channel);
	struct ipu_ch_param *cpmem_in_p, *cpmem_in_n;
	struct ipu_ch_param *cpmem_out = ipu_get_cpmem(task->output_channel);
	struct image_convert_ctx *ctx;
	struct ipu_image *in_p, *in, *in_n;
	struct ipu_image *out;
	struct ic_setup setup;
	int ret;
	unsigned long flags;
	unsigned int inburst, outburst;

	spin_lock_irqsave(&priv->lock, flags);

	if (list_empty(&task->image_list)) {
		spin_unlock_irqrestore(&priv->lock, flags);
		return NULL;
	}

	ctx = list_first_entry(&task->image_list, struct image_convert_ctx, list);

	list_del(&ctx->list);

	spin_unlock_irqrestore(&priv->lock, flags);

	in_p = &ctx->in_p;
	in = &ctx->in;
	in_n = &ctx->in_n;
	out = &ctx->out;

	memset(cpmem_in, 0, sizeof(*cpmem_in));
	if (ctx->deinterlace) {
		cpmem_in_p = ipu_get_cpmem(task->input_channel_p);
		cpmem_in_n = ipu_get_cpmem(task->input_channel_n);
		memset(cpmem_in_p, 0, sizeof(*cpmem_in_p));
		memset(cpmem_in_n, 0, sizeof(*cpmem_in_n));
	}
	memset(cpmem_out, 0, sizeof(*cpmem_out));

	inburst = in->rect.width & 0xf ? 8 : 16;
	outburst = out->rect.width & 0xf ? 8 : 16;

	ipu_ic_enable(priv);

	ipu_ic_idma_init(task, task->input_channel->num, in->rect.width, in->rect.height, inburst, 0);
	if (ctx->deinterlace) {
		ipu_ic_idma_init(task, task->input_channel_p->num, in_p->rect.width, in_p->rect.height, inburst, 0);
		ipu_ic_idma_init(task, task->input_channel_n->num, in_n->rect.width, in_n->rect.height, inburst, 0);
	}
	ipu_ic_idma_init(task, task->output_channel->num, out->rect.width, out->rect.height, outburst, 0);

	ipu_cpmem_set_image(cpmem_in, &ctx->in);
	if (ctx->deinterlace) {
		ipu_cpmem_set_image(cpmem_in_p, &ctx->in_p);
		ipu_cpmem_set_image(cpmem_in_n, &ctx->in_n);
	}
	ipu_cpmem_set_image(cpmem_out, &ctx->out);

	ipu_cpmem_set_burstsize(cpmem_in, inburst);
	if (ctx->deinterlace) {
		ipu_cpmem_set_burstsize(cpmem_in_p, inburst);
		ipu_cpmem_set_burstsize(cpmem_in_n, inburst);
	}
	ipu_cpmem_set_burstsize(cpmem_out, outburst);

	memset(&setup, 0, sizeof(setup));
	setup.in_width = in->rect.width;
	if (ctx->deinterlace)
		setup.in_height = in->rect.height * 2;
	else
		setup.in_height = in->rect.height;

	setup.in_fmt = ipu_pixelformat_to_colorspace(in->pix.pixelformat);
	setup.out_width = out->rect.width;
	setup.out_height = out->rect.height;
	setup.out_fmt = ipu_pixelformat_to_colorspace(out->pix.pixelformat);

	dev_dbg(priv->dev, "%s: %dx%d(%dx%d@%dx%d) -> %dx%d(%dx%d@%dx%d)\n", __func__,
			in->pix.width, in->pix.height,
			in->rect.width, in->rect.height, in->rect.left, in->rect.top,
			out->pix.width, out->pix.height,
			out->rect.width, out->rect.height, out->rect.left, out->rect.top
			);

	ret = ipu_ic_init_task(task, &setup, 0);
	if (ret) {
		ipu_ic_disable(priv);
		return ERR_PTR(ret);
	}

	ipu_ic_enable_task(task, ctx);

	return ctx;
}

static void ipu_image_convert_work(struct work_struct *work)
{
	struct ipu_ic_task *task = container_of(work, struct ipu_ic_task, work);
	struct ipu_ic_priv *priv = task->priv;
	struct image_convert_ctx *ctx;
	int ret;

	while (1) {
		int task_error = 0;

		ctx = ipu_image_convert_next(task);
		if (!ctx)
			return;

		if (IS_ERR(ctx)) {
			task_error = PTR_ERR(ctx);
		} else {
			ret = wait_for_completion_interruptible_timeout(&task->complete,100* HZ);
			if (!ret)
				task_error = -ETIMEDOUT;
		}

		ipu_ic_disable_task(task, ctx);
		ipu_ic_disable(priv);

		if (ctx->complete)
			ctx->complete(ctx->complete_context, task_error);
		if (ctx->freep)
			kfree(ctx->freep);
	}
}

static irqreturn_t ipu_image_convert_handler(int irq, void *context)
{
	struct ipu_ic_task *task = context;

	complete(&task->complete);

	return IRQ_HANDLED;
}

#define VDI_FSIZE	0
#define VDI_C		0x4

#define VDI_C_CH_420		0x00000000
#define VDI_C_CH_422		0x00000002
#define VDI_C_MOT_SEL_FULL	0x00000008
#define VDI_C_MOT_SEL_LOW	0x00000004
#define VDI_C_MOT_SEL_MED	0x00000000
#define VDI_C_BURST_SIZE1_4	0x00000030
#define VDI_C_BURST_SIZE2_4	0x00000300
#define VDI_C_BURST_SIZE3_4	0x00003000
#define VDI_C_VWM1_CLR_2	0x00080000
#define VDI_C_VWM3_CLR_2	0x02000000
#define VDI_C_TOP_FIELD_MAN_1	0x40000000
#define VDI_C_TOP_FIELD_AUTO_1	0x80000000

static void ipu_vdi_init(struct ipu_ic_priv *priv, u32 pixelfmt, int xres, int yres, u32 field)
{
	uint32_t reg;
	uint32_t pixel_fmt;

	reg = ((yres - 1) << 16) | (xres - 1);
	ipu_vdi_write(priv, reg, VDI_FSIZE);

	/*
	 * Full motion, only vertical filter is used
	 * Burst size is 4 accesses
	 */
	if (pixelfmt == V4L2_PIX_FMT_UYVY || pixelfmt == V4L2_PIX_FMT_YUYV)
		pixel_fmt = VDI_C_CH_422;
	else
		pixel_fmt = VDI_C_CH_420;

	reg = ipu_vdi_read(priv, VDI_C);

	reg |= pixel_fmt;
	reg |= VDI_C_BURST_SIZE2_4;
	reg |= VDI_C_BURST_SIZE1_4 | VDI_C_VWM1_CLR_2;
	reg |= VDI_C_BURST_SIZE3_4 | VDI_C_VWM3_CLR_2;

	if (field == V4L2_FIELD_INTERLACED_TB)
		reg |= VDI_C_TOP_FIELD_MAN_1;

	reg |= VDI_C_MOT_SEL_LOW;

	ipu_vdi_write(priv, reg, VDI_C);
}

int ipu_image_deinterlace_convert(struct ipu_soc *ipu, struct ipu_image *in_p,
		struct ipu_image *in, struct ipu_image *in_n, struct ipu_image *out,
		void (*complete)(void *ctx, int err), void *complete_context)
{
	struct ipu_ic_priv *priv = ipu->ic_priv;
	struct ipu_ic_task *task = &priv->task[IC_TASK_VIEWFINDER];
	struct image_convert_ctx *ctx;
	unsigned long flags;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ipu_module_enable(ipu, 1<<30);
	ipu_module_enable(ipu, IPU_CONF_VDI_EN);
	ipu_vdi_init(priv, V4L2_PIX_FMT_UYVY, in->pix.width, in->pix.height * 2, V4L2_FIELD_INTERLACED_TB);

	memcpy(&ctx->in_p, in_p, sizeof(*in_p));
	memcpy(&ctx->in, in, sizeof(*in));
	memcpy(&ctx->in_n, in_n, sizeof(*in_n));
	memcpy(&ctx->out, out, sizeof(*out));

	ctx->complete = complete;
	ctx->complete_context = complete_context;
	ctx->freep = ctx;
	ctx->deinterlace = 1;

	spin_lock_irqsave(&priv->lock, flags);

	list_add_tail(&ctx->list, &task->image_list);

	queue_work(task->workqueue, &task->work);

	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(ipu_image_deinterlace_convert);

int ipu_image_convert(struct ipu_soc *ipu, struct ipu_image *in, struct ipu_image *out,
		void (*complete)(void *ctx, int err), void *complete_context)
{
	struct ipu_ic_priv *priv = ipu->ic_priv;
	struct ipu_ic_task *task = &priv->task[IC_TASK_POST_PROCESSOR];
	struct image_convert_ctx *ctx, *c;
	int htiles = 1, vtiles = 1;
	int x, y, i, numtiles, bppin, bppout;
	unsigned long flags;
	int in_rect_width;

	/* Force input width to be a multiple of 8 pixels, hardware limitation */
	in_rect_width = in->rect.width & ~0x7;

	if (out->rect.width > 1024) {
		/*
		 * With horizontal tiling force input width to be a multiple of 16
		 * pixels, we could do better with the cost of a better tiling
		 * logic below.
		 */
		in_rect_width = in->rect.width & ~0x7;
		htiles = 2;
	}

	if (out->rect.height > 1024)
		vtiles = 2;

        bppin = in->pix.bytesperline / in->pix.width;
        bppout = out->pix.bytesperline / out->pix.width;

        dev_dbg(priv->dev, "%s: in: %dx%d -> %dx%d phys 0x%08x -> 0x%08x\n",
                        __func__, in->pix.width, in->pix.height,
			out->pix.width, out->pix.height,
			in->phys, out->phys);

        ctx = kzalloc(sizeof(*ctx) * htiles * vtiles, GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	c = ctx;

	for (x = 0; x < htiles; x++) {
		for (y = 0; y < vtiles; y++) {
			c->in.rect.width = in_rect_width / htiles;
			c->in.rect.height = in->rect.height / vtiles;
			c->in.rect.top = y * c->in.rect.height + in->rect.top;
			c->in.rect.left = x * c->in.rect.width + in->rect.left;
			c->in.phys = in->phys;

			memcpy(&c->in.pix, &in->pix, sizeof(struct v4l2_pix_format));

			c->out.rect.width = out->rect.width / htiles;
			c->out.rect.height = out->rect.height / vtiles;
			c->out.rect.top = y * c->out.rect.height;
			c->out.rect.left = x * c->out.rect.width;
			c->out.phys = out->phys;

			memcpy(&c->out.pix, &out->pix, sizeof(struct v4l2_pix_format));

			c++;
		}
	}

	numtiles = htiles * vtiles;

	ctx[numtiles - 1].complete = complete;
	ctx[numtiles - 1].complete_context = complete_context;
	ctx[numtiles - 1].freep = ctx;

	spin_lock_irqsave(&priv->lock, flags);

	for (i = 0; i < numtiles; i++)
		list_add_tail(&ctx[i].list, &task->image_list);

	queue_work(task->workqueue, &task->work);

	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(ipu_image_convert);

static int ipu_image_convert_init(struct device *dev, struct ipu_soc *ipu,
		struct ipu_ic_priv *priv)
{
	int i, ret;

	for (i = 0; i < IC_TASK_MAX; i++) {
		struct ipu_ic_task *task = &priv->task[i];
		int irq = ipu_idmac_channel_irq(ipu, task->output_channel, IPU_IRQ_EOF);

		task->workqueue = create_singlethread_workqueue(
				dev_name(ipu->dev));
		if (!task->workqueue) {
			ret = -ENOMEM;
			goto err;
		}

		INIT_WORK(&task->work, ipu_image_convert_work);
		init_completion(&task->complete);

		ret = devm_request_threaded_irq(dev, irq, NULL, ipu_image_convert_handler,
				IRQF_ONESHOT, "IC", task);
		if (ret)
			goto err;
	}

	return 0;
err:
	while (i-- > 0) {
		struct ipu_ic_task *task = &priv->task[i];

		destroy_workqueue(task->workqueue);
	}

	return ret;
}

int ipu_ic_init(struct ipu_soc *ipu, struct device *dev, unsigned long ic_base,
		unsigned long tpm_base, unsigned long vdi_base)
{
	struct ipu_ic_priv *priv;
	int i;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	priv->dev = dev;
	priv->ipu = ipu;

	ipu->ic_priv = priv;

	priv->ic_base = devm_ioremap(dev, ic_base, PAGE_SIZE);
	if (!priv->ic_base)
		return -ENOMEM;

	priv->vdi_base = devm_ioremap(dev, vdi_base, PAGE_SIZE);
	if (!priv->vdi_base)
		return -ENOMEM;

	priv->tpm_base = devm_ioremap(dev, tpm_base, SZ_64K);
	if (!priv->tpm_base)
		return -ENOMEM;

	spin_lock_init(&priv->lock);

	for (i = 0; i < IC_TASK_MAX; i++) {
		struct ipu_ic_task_template *templ = &templates[i];
		struct ipu_ic_task *task = &priv->task[i];
		INIT_LIST_HEAD(&task->image_list);
		task->id = i;
		task->priv = priv;
		task->reg_rsc = priv->ic_base + templ->ofs_rsc;
		task->csc1_mask = templ->csc1_mask;
		task->csc2_mask = templ->csc2_mask;
		task->cmb_mask = templ->cmb_mask;
		task->tpm_base_csc1 = priv->tpm_base + templ->tpm_ofs_csc1;
		task->tpm_base_csc2 = priv->tpm_base + templ->tpm_ofs_csc2;
		task->ic_conf_rot_en = templ->ic_conf_rot_en;
		task->ic_conf_en = templ->ic_conf_en;
		task->first_deinterlace_frame = 1;
		task->input_channel = ipu_idmac_get(ipu, templ->input_channel);
		if (IS_ERR(task->input_channel))
			goto out;
		if (templ->input_channel_n) {
			task->input_channel_n = ipu_idmac_get(ipu, templ->input_channel_n);
			if (IS_ERR(task->input_channel_n))
				goto out;
		}
		if (templ->input_channel_p) {
			task->input_channel_p = ipu_idmac_get(ipu, templ->input_channel_p);
			if (IS_ERR(task->input_channel_p))
				goto out;
		}

		task->output_channel = ipu_idmac_get(ipu, templ->output_channel);
		if (IS_ERR(task->output_channel))
			goto out;

		task->rotation_input_channel = ipu_idmac_get(ipu,
				templ->rotation_input_channel);
		if (IS_ERR(task->rotation_input_channel))
			goto out;
		task->rotation_output_channel = ipu_idmac_get(ipu,
				templ->rotation_output_channel);
		if (IS_ERR(task->rotation_output_channel))
			goto out;
	}

	ipu_image_convert_init(dev, ipu, priv);

	mutex_init(&priv->mutex);

	return 0;
out:
	dev_err(priv->dev, "failed to initialize IC\n");

	/* FIXME: Cleanup */

	return -EINVAL;
}

void ipu_ic_exit(struct ipu_soc *ipu)
{
}
