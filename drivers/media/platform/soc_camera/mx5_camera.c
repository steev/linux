/*
 * V4L2 Driver for i.MXL/i.MXL camera (CSI) host
 *
 * Copyright (C) 2008, Paulius Zaleckas <paulius.zaleckas@teltonika.lt>
 * Copyright (C) 2009, Darius Augulis <augulis.darius@gmail.com>
 *
 * Based on PXA SoC camera driver
 * Copyright (C) 2006, Sascha Hauer, Pengutronix
 * Copyright (C) 2008, Guennadi Liakhovetski <kernel@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/version.h>
#include <linux/videodev2.h>
#include "../../../staging/imx-drm/ipu-v3/imx-ipu-v3.h"

#include <media/soc_camera.h>
#include <media/v4l2-common.h>
#include <media/v4l2-dev.h>
#include <media/videobuf2-dma-contig.h>
#include <media/soc_mediabus.h>
#include <asm/mach-types.h>
#define DRIVER_NAME "imx-ipuv3-camera"

/* CMOS Sensor Interface Registers */
#define CSI_SENS_CONF           0x0000
#define CSI_SENS_FRM_SIZE       0x0004
#define CSI_ACT_FRM_SIZE        0x0008
#define CSI_OUT_FRM_CTRL        0x000c
#define CSI_TST_CTRL            0x0010
#define CSI_CCIR_CODE_1         0x0014
#define CSI_CCIR_CODE_2         0x0018
#define CSI_CCIR_CODE_3         0x001c
#define CSI_MIPI_DI             0x0020
#define CSI_SKIP                0x0024
#define CSI_CPD_CTRL            0x0028
#define CSI_CPD_RC(n)           (0x002c + 4 * (n))
#define CSI_CPD_RS(n)           (0x004c + 4 * (n))
#define CSI_CPD_GRC(n)          (0x005c + 4 * (n))
#define CSI_CPD_GRS(n)          (0x007c + 4 * (n))
#define CSI_CPD_GBC(n)          (0x008c + 4 * (n))
#define CSI_CPD_GBS(n)          (0x00ac + 4 * (n))
#define CSI_CPD_BC(n)           (0x00bc + 4 * (n))
#define CSI_CPD_BS(n)           (0x00dc + 4 * (n))
#define CSI_CPD_OFFSET1         0x00ec
#define CSI_CPD_OFFSET2         0x00f0

#define CSI_SENS_CONF_VSYNC_POL_SHIFT		0
#define CSI_SENS_CONF_HSYNC_POL_SHIFT		1
#define CSI_SENS_CONF_DATA_POL_SHIFT		2
#define CSI_SENS_CONF_PIX_CLK_POL_SHIFT		3
#define CSI_SENS_CONF_SENS_PRTCL_SHIFT		4
#define CSI_SENS_CONF_PACK_TIGHT_SHIFT		7
#define CSI_SENS_CONF_DATA_FMT_SHIFT		8
#define CSI_SENS_CONF_DATA_WIDTH_SHIFT		11
#define CSI_SENS_CONF_EXT_VSYNC_SHIFT		15
#define CSI_SENS_CONF_DIVRATIO_SHIFT		16
#define CSI_SENS_CONF_DATA_DEST_SHIFT		24
#define CSI_SENS_CONF_JPEG8_EN_SHIFT		27
#define CSI_SENS_CONF_JPEG_EN_SHIFT		28
#define CSI_SENS_CONF_FORCE_EOF_SHIFT		29
#define CSI_SENS_CONF_DATA_EN_POL_SHIFT		31

#define CSI_SENS_CONF_SENS_PRTCL_MASK		(0x7 << CSI_SENS_CONF_SENS_PRTCL_SHIFT)
#define CSI_SENS_PRTCL_GATED			(0 << CSI_SENS_CONF_SENS_PRTCL_SHIFT)
#define CSI_SENS_PRTCL_NON_GATED		(1 << CSI_SENS_CONF_SENS_PRTCL_SHIFT)
#define CSI_SENS_PRTCL_BT656_PROGRESSIVE	(2 << CSI_SENS_CONF_SENS_PRTCL_SHIFT)
#define CSI_SENS_PRTCL_BT656_INTERLACED		(3 << CSI_SENS_CONF_SENS_PRTCL_SHIFT)
#define CSI_SENS_PRTCL_BT1120_DDR_PROGRESSIVE	(4 << CSI_SENS_CONF_SENS_PRTCL_SHIFT)
#define CSI_SENS_PRTCL_BT1120_SDR_PROGRESSIVE	(5 << CSI_SENS_CONF_SENS_PRTCL_SHIFT)
#define CSI_SENS_PRTCL_BT1120_DDR_INTERLACED	(6 << CSI_SENS_CONF_SENS_PRTCL_SHIFT)
#define CSI_SENS_PRTCL_BT1120_SDR_INTERLACED	(7 << CSI_SENS_CONF_SENS_PRTCL_SHIFT)

#define CSI_SENS_CONF_DATA_FMT_MASK		(0x7 << CSI_SENS_CONF_DATA_FMT_SHIFT)
#define CSI_SENS_CONF_DATA_FMT_RGB_YUV444	(0 << CSI_SENS_CONF_DATA_FMT_SHIFT)
#define CSI_SENS_CONF_DATA_FMT_YUV422_YUYV	(1 << CSI_SENS_CONF_DATA_FMT_SHIFT)
#define CSI_SENS_CONF_DATA_FMT_YUV422_UYVY	(2 << CSI_SENS_CONF_DATA_FMT_SHIFT)
#define CSI_SENS_CONF_DATA_FMT_BAYER		(3 << CSI_SENS_CONF_DATA_FMT_SHIFT)
#define CSI_SENS_CONF_DATA_FMT_RGB565		(4 << CSI_SENS_CONF_DATA_FMT_SHIFT)
#define CSI_SENS_CONF_DATA_FMT_RGB555		(5 << CSI_SENS_CONF_DATA_FMT_SHIFT)
#define CSI_SENS_CONF_DATA_FMT_RGB444		(6 << CSI_SENS_CONF_DATA_FMT_SHIFT)
#define CSI_SENS_CONF_DATA_FMT_JPEG		(7 << CSI_SENS_CONF_DATA_FMT_SHIFT)

#define CSI_SENS_CONF_DATA_WIDTH_MASK		(0xf << CSI_SENS_CONF_DATA_WIDTH_SHIFT)

#define CSI_SENS_CONF_DIVRATIO_MASK	0x00FF0000L
#define CSI_SENS_CONF_DATA_DEST_MASK	0x07000000L

#define CSI_DATA_DEST_ISP	1L
#define CSI_DATA_DEST_IC	2L
#define CSI_DATA_DEST_IDMAC	4L

#define CSI_CCIR_ERR_DET_EN	0x01000000L
#define CSI_HORI_DOWNSIZE_EN	0x80000000L
#define CSI_VERT_DOWNSIZE_EN	0x40000000L
#define CSI_TEST_GEN_MODE_EN	0x01000000L

#define CSI_HSC_MASK		0x1FFF0000
#define CSI_HSC_SHIFT		16
#define CSI_VSC_MASK		0x00000FFF
#define CSI_VSC_SHIFT		0

#define CSI_TEST_GEN_R_MASK	0x000000FFL
#define CSI_TEST_GEN_R_SHIFT	0
#define CSI_TEST_GEN_G_MASK	0x0000FF00L
#define CSI_TEST_GEN_G_SHIFT	8
#define CSI_TEST_GEN_B_MASK	0x00FF0000L
#define CSI_TEST_GEN_B_SHIFT	16

#define CSI_MIPI_DI0_MASK	0x000000FFL
#define CSI_MIPI_DI0_SHIFT	0
#define CSI_MIPI_DI1_MASK	0x0000FF00L
#define CSI_MIPI_DI1_SHIFT	8
#define CSI_MIPI_DI2_MASK	0x00FF0000L
#define CSI_MIPI_DI2_SHIFT	16
#define CSI_MIPI_DI3_MASK	0xFF000000L
#define CSI_MIPI_DI3_SHIFT	24

#define CSI_MAX_RATIO_SKIP_ISP_MASK 0x00070000L
#define CSI_MAX_RATIO_SKIP_ISP_SHIFT 16
#define CSI_SKIP_ISP_MASK	0x00F80000L
#define CSI_SKIP_ISP_SHIFT	19
#define CSI_MAX_RATIO_SKIP_SMFC_MASK 0x00000007L
#define CSI_MAX_RATIO_SKIP_SMFC_SHIFT 0
#define CSI_SKIP_SMFC_MASK 	x000000F8L
#define CSI_SKIP_SMFC_SHIFT	3
#define CSI_ID_2_SKIP_MASK	0x00000300L
#define CSI_ID_2_SKIP_SHIFT	8

#define CSI_COLOR_FIRST_ROW_MASK	0x00000002L
#define CSI_COLOR_FIRST_COMP_MASK	0x00000001L

#define MAX_VIDEO_MEM 48	/* Video memory limit in megabytes */

/* buffer for one video frame */
struct mx5_buffer {
	struct vb2_buffer		vb;
	enum v4l2_mbus_pixelcode	code;
	struct list_head		queue;
	int				init;
};

struct mx5_camera_dev {
	struct soc_camera_host		soc_host;
	struct soc_camera_device	*icd;
	struct mx5_buffer		*active; /* The currently active buffer, set by NFACK and cleared by EOF interrupt */
	struct list_head		capture;

	int				id; /* CSI<id> - 0 or 1 */
	void __iomem			*base;
	struct clk			*hclk;
	unsigned int			bus_flags;
	unsigned int			sens_conf; /* shadow the SENS_CONF register */

	spinlock_t			lock; /* locks CSI register access */
	struct vb2_alloc_ctx		*alloc_ctx;
	enum v4l2_field			field;
	int				sequence;
	struct ipuv3_channel		*ipuch;
	struct ipu_soc			*ipu;
	u32				fourcc;
};

static struct mx5_buffer *to_mx5_vb(struct vb2_buffer *vb)
{
	return container_of(vb, struct mx5_buffer, vb);
}

static const struct soc_mbus_pixelfmt mx5_camera_formats[] = {
	{
		.fourcc			= V4L2_PIX_FMT_UYVY,
		.name			= "YUV422 interleaved",
		.bits_per_sample	= 16,
		.packing		= SOC_MBUS_PACKING_NONE,
		.order			= SOC_MBUS_ORDER_LE,
	}, {
		.fourcc			= V4L2_PIX_FMT_RGB32,
		.name			= "RGB888 32bit",
		.bits_per_sample	= 8,
		.packing		= SOC_MBUS_PACKING_4X8_PADHI,
		.order			= SOC_MBUS_ORDER_LE,
	}, {
		.fourcc			= V4L2_PIX_FMT_UYVY,
		.name			= "YUV422 interleaved",
		.bits_per_sample	= 8,
		.packing		= SOC_MBUS_PACKING_2X8_PADHI,
		.order			= SOC_MBUS_ORDER_LE,
	},
};

static inline u32 ipu_csi_read(struct mx5_camera_dev *csi, unsigned offset)
{
	struct device *dev = csi->icd->parent;
	u32 ret = readl(csi->base + offset);
	dev_dbg(dev, "%s(%p + 0x%04x) = 0x%08x\n", __func__, csi->base, offset, ret);
	return ret;
}

static inline void ipu_csi_write(struct mx5_camera_dev *csi, u32 value, unsigned offset)
{
	struct device *dev = csi->icd->parent;
	dev_dbg(dev, "%s(0x%08x, %p + 0x%04x)\n", __func__, value, csi->base, offset);
	writel(value, csi->base + offset);
}

int ipu_csi_init_interface(struct mx5_camera_dev *csi,
			   uint16_t width, uint16_t height, u32 cfg_param)
{
	struct device *dev = csi->icd->parent;
	unsigned long lock_flags;
	u32 clk_mode = cfg_param & CSI_SENS_CONF_SENS_PRTCL_MASK;

	spin_lock_irqsave(&csi->lock, lock_flags);

	/* Set DATA_DEST to IDMAC unconditionally, for now */
	cfg_param &= ~CSI_SENS_CONF_DATA_DEST_MASK;
	cfg_param |= CSI_DATA_DEST_IDMAC << CSI_SENS_CONF_DATA_DEST_SHIFT;

	ipu_csi_write(csi, cfg_param, CSI_SENS_CONF);

	/* Setup sensor frame size */
	ipu_csi_write(csi, (width - 1) | (height - 1) << 16, CSI_SENS_FRM_SIZE);

	/* Set CCIR registers */
	switch (clk_mode) {
	case CSI_SENS_PRTCL_BT656_PROGRESSIVE:
		ipu_csi_write(csi, 0x40030, CSI_CCIR_CODE_1);
		ipu_csi_write(csi, 0xFF0000, CSI_CCIR_CODE_3);
		break;
	case CSI_SENS_PRTCL_BT656_INTERLACED:
		if (width == 720 && height == 576) {
			/* PAL case */
			/*
			 * Field0BlankEnd = 0x6, Field0BlankStart = 0x2,
			 * Field0ActiveEnd = 0x4, Field0ActiveStart = 0
			 */
			ipu_csi_write(csi, CSI_CCIR_ERR_DET_EN | 0x40596, CSI_CCIR_CODE_1);
			/*
			 * Field1BlankEnd = 0x7, Field1BlankStart = 0x3,
			 * Field1ActiveEnd = 0x5, Field1ActiveStart = 0x1
			 */
			ipu_csi_write(csi, 0xD07DF, CSI_CCIR_CODE_2);
			ipu_csi_write(csi, 0xFF0000, CSI_CCIR_CODE_3);
		} else if (width == 720 && height == 480) {
			/* NTSC case */
			/*
			 * Field0BlankEnd = 0x7, Field0BlankStart = 0x3,
			 * Field0ActiveEnd = 0x5, Field0ActiveStart = 0x1
			 */
			ipu_csi_write(csi, CSI_CCIR_ERR_DET_EN | 0xD07DF, CSI_CCIR_CODE_1);
			/*
			 * Field1BlankEnd = 0x6, Field1BlankStart = 0x2,
			 * Field1ActiveEnd = 0x4, Field1ActiveStart = 0
			 */
			ipu_csi_write(csi, 0x40596, CSI_CCIR_CODE_2);
			ipu_csi_write(csi, 0xFF0000, CSI_CCIR_CODE_3);
		} else {
			spin_unlock_irqrestore(&csi->lock, lock_flags);
			dev_err(dev, "Unsupported CCIR656 interlaced video mode\n");
			return -EINVAL;
		}
		break;
	case CSI_SENS_PRTCL_BT1120_DDR_PROGRESSIVE:
	case CSI_SENS_PRTCL_BT1120_SDR_PROGRESSIVE:
	case CSI_SENS_PRTCL_BT1120_DDR_INTERLACED:
	case CSI_SENS_PRTCL_BT1120_SDR_INTERLACED:
		ipu_csi_write(csi, CSI_CCIR_ERR_DET_EN | 0x40030, CSI_CCIR_CODE_1);
		ipu_csi_write(csi, 0xFF0000, CSI_CCIR_CODE_3);
		break;
	case CSI_SENS_PRTCL_GATED:
	case CSI_SENS_PRTCL_NON_GATED:
		break;
	}

	dev_dbg(dev, "CSI_SENS_CONF = 0x%08X\n",
		ipu_csi_read(csi, CSI_SENS_CONF));
	dev_dbg(dev, "CSI_SENS_FRM_SIZE = 0x%08X\n",
		ipu_csi_read(csi, CSI_SENS_FRM_SIZE));
	dev_dbg(dev, "CSI_ACT_FRM_SIZE = 0x%08X\n",
		ipu_csi_read(csi, CSI_ACT_FRM_SIZE));

	spin_unlock_irqrestore(&csi->lock, lock_flags);

	return 0;
}

int ipu_csi_mclk_set_rate(struct mx5_camera_dev *csi, uint32_t pixel_clk)
{
	struct device *dev = csi->icd->parent;
	uint32_t div_ratio;

	div_ratio = (clk_get_rate(csi->hclk) / pixel_clk) - 1;

	if (div_ratio > 0xFF || div_ratio < 0) {
		dev_dbg(dev, "The value of pixel_clk extends normal range\n");
		dev_dbg(dev, "smallest possible pixel clock at current ipu clk rate: %lu (wanted: %u)\n", clk_get_rate(csi->hclk) / 256, pixel_clk);
		return -EINVAL;
	}

	csi->sens_conf &= ~CSI_SENS_CONF_DIVRATIO_MASK;
	csi->sens_conf |= div_ratio << CSI_SENS_CONF_DIVRATIO_SHIFT;
	ipu_csi_write(csi, csi->sens_conf, CSI_SENS_CONF);

	dev_dbg(dev, "Set pixel_clk to %lu (div_ratio = %d)\n", clk_get_rate(csi->hclk) / (div_ratio + 1), div_ratio);

	return 0;
}

uint32_t ipu_csi_mclk_get_rate(struct mx5_camera_dev *csi)
{
	uint32_t temp, div_ratio;

	temp = ipu_csi_read(csi, CSI_SENS_CONF);
	temp &= ~CSI_SENS_CONF_DIVRATIO_MASK;
	div_ratio = temp >> CSI_SENS_CONF_DIVRATIO_SHIFT;

	return clk_get_rate(csi->hclk) / (div_ratio + 1);
}

void ipu_csi_get_window_size(struct mx5_camera_dev *csi, uint32_t *width, uint32_t *height)
{
	uint32_t reg;
	unsigned long lock_flags;

	spin_lock_irqsave(&csi->lock, lock_flags);

	reg = ipu_csi_read(csi, CSI_ACT_FRM_SIZE);
	*width = (reg & 0xFFFF) + 1;
	*height = (reg >> 16 & 0xFFFF) + 1;

	spin_unlock_irqrestore(&csi->lock, lock_flags);
}

void ipu_csi_set_window_size(struct mx5_camera_dev *csi, uint32_t width, uint32_t height)
{
	unsigned long lock_flags;

	spin_lock_irqsave(&csi->lock, lock_flags);

	ipu_csi_write(csi, (width - 1) | (height - 1) << 16, CSI_ACT_FRM_SIZE);

	spin_unlock_irqrestore(&csi->lock, lock_flags);
}

void ipu_csi_set_window_pos(struct mx5_camera_dev *csi, uint32_t left, uint32_t top)
{
	uint32_t temp;
	unsigned long lock_flags;

	spin_lock_irqsave(&csi->lock, lock_flags);

	temp = ipu_csi_read(csi, CSI_OUT_FRM_CTRL);
	temp &= ~(CSI_HSC_MASK | CSI_VSC_MASK);
	temp |= ((top << CSI_VSC_SHIFT) | (left << CSI_HSC_SHIFT));
	ipu_csi_write(csi, temp, CSI_OUT_FRM_CTRL);

	spin_unlock_irqrestore(&csi->lock, lock_flags);
}

void _ipu_csi_set_test_generator(struct mx5_camera_dev *csi, bool active, uint32_t r_value,
	uint32_t g_value, uint32_t b_value)
{
	uint32_t temp;

	temp = ipu_csi_read(csi, CSI_TST_CTRL);

	if (active == false) {
		temp &= ~CSI_TEST_GEN_MODE_EN;
		ipu_csi_write(csi, temp, CSI_TST_CTRL);
	} else {
		temp &= ~(CSI_TEST_GEN_R_MASK | CSI_TEST_GEN_G_MASK |
			CSI_TEST_GEN_B_MASK);
		temp |= CSI_TEST_GEN_MODE_EN;
		temp |= (r_value << CSI_TEST_GEN_R_SHIFT) |
			(g_value << CSI_TEST_GEN_G_SHIFT) |
			(b_value << CSI_TEST_GEN_B_SHIFT);
		ipu_csi_write(csi, temp, CSI_TST_CTRL);
	}
}

static inline void mx5cam_set_inactive_buffer(struct mx5_camera_dev *mx5_cam,
					      struct vb2_buffer *vb)
{
	int bufptr = !ipu_idmac_get_current_buffer(mx5_cam->ipuch);
	ipu_cpmem_set_buffer(ipu_get_cpmem(mx5_cam->ipuch), bufptr,
			     vb2_dma_contig_plane_dma_addr(vb, 0));
	ipu_idmac_select_buffer(mx5_cam->ipuch, bufptr);
}

static irqreturn_t mx5cam_new_frame_handler(int irq, void *context)
{
	struct mx5_camera_dev *mx5_cam = context;
	struct mx5_buffer *buf;
	struct vb2_buffer *vb;
	unsigned long flags;

	/* The IDMAC just started to write pixel data into the current buffer */

	spin_lock_irqsave(&mx5_cam->lock, flags);

	/*
	 * If there is a previously active frame, mark it as done to hand it off
	 * to userspace. Or, if there are no further frames queued, hold on to it.
	 */
	if (mx5_cam->active) {
		vb = &mx5_cam->active->vb;
		buf = to_mx5_vb(vb);

		if (vb2_is_streaming(vb->vb2_queue) && list_is_singular(&mx5_cam->capture)) {
			pr_debug("%s: reusing 0x%08x\n", __func__,
				vb2_dma_contig_plane_dma_addr(vb, 0));
			/* DEBUG: check if buf == EBA(active) */
		} else {
			/* Otherwise, mark buffer as finished */
			list_del_init(&buf->queue);

			vb2_buffer_done(vb, VB2_BUF_STATE_DONE);
		}
	}

	if (list_empty(&mx5_cam->capture))
		goto out;

	mx5_cam->active = list_first_entry(&mx5_cam->capture,
					   struct mx5_buffer, queue);
	vb = &mx5_cam->active->vb;
	do_gettimeofday(&vb->v4l2_buf.timestamp);
	vb->v4l2_buf.field = mx5_cam->field;
	vb->v4l2_buf.sequence = mx5_cam->sequence++;

	/*
	 * Point the inactive buffer address to the next queued buffer,
	 * if available. Otherwise, prepare to reuse the currently active
	 * buffer, unless mx5cam_videobuf_queue gets called in time.
	 */
	if (!list_is_singular(&mx5_cam->capture)) {
		buf = list_entry(mx5_cam->capture.next->next,
				 struct mx5_buffer, queue);
		vb = &buf->vb;
	}
	mx5cam_set_inactive_buffer(mx5_cam, vb);
out:
	spin_unlock_irqrestore(&mx5_cam->lock, flags);

	return IRQ_HANDLED;
}

/*
 *  Videobuf operations
 */
static int mx5_videobuf_setup(struct vb2_queue *vq, const struct v4l2_format *fmt,
		unsigned int *count, unsigned int *num_planes,
		unsigned int sizes[], void *alloc_ctxs[])
{
	struct soc_camera_device *icd = soc_camera_from_vb2q(vq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct mx5_camera_dev *mx5_cam = ici->priv;
	int bytes_per_line = soc_mbus_bytes_per_line(icd->user_width,
						icd->current_fmt->host_fmt);

	if (bytes_per_line < 0)
		return bytes_per_line;

	*num_planes = 1;

	mx5_cam->sequence = 0;
	sizes[0] = bytes_per_line * icd->user_height;
	alloc_ctxs[0] = mx5_cam->alloc_ctx;

	if (!*count)
		*count = 32;

	if (sizes[0] * *count > MAX_VIDEO_MEM * 1024 * 1024)
		*count = MAX_VIDEO_MEM * 1024 * 1024 / sizes[0];

	return 0;
}

static int mx5_videobuf_prepare(struct vb2_buffer *vb)
{
	struct soc_camera_device *icd = soc_camera_from_vb2q(vb->vb2_queue);
	size_t new_size;
	struct mx5_buffer *buf;
	int bytes_per_line = soc_mbus_bytes_per_line(icd->user_width,
						icd->current_fmt->host_fmt);

	if (bytes_per_line < 0)
		return bytes_per_line;

	buf = to_mx5_vb(vb);

	new_size = bytes_per_line * icd->user_height;

	if (vb2_plane_size(vb, 0) < new_size) {
		dev_err(icd->parent, "Buffer too small (%lu < %zu)\n",
				vb2_plane_size(vb, 0), new_size);
		return -ENOBUFS;
	}

	vb2_set_plane_payload(vb, 0, new_size);

	return 0;
}

/* Called under spinlock_irqsave(&pcdev->lock, ...) */
static void mx5_videobuf_queue(struct vb2_buffer *vb)
{
	struct soc_camera_device *icd = soc_camera_from_vb2q(vb->vb2_queue);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct mx5_camera_dev *mx5_cam = ici->priv;
	struct mx5_buffer *buf = to_mx5_vb(vb);
	unsigned long flags;

	spin_lock_irqsave(&mx5_cam->lock, flags);

	/*
	 * If there is no next buffer queued, point the inactive buffer
	 * address to the incoming buffer
	 */
	if (vb2_is_streaming(vb->vb2_queue) && list_is_singular(&mx5_cam->capture))
		mx5cam_set_inactive_buffer(mx5_cam, vb);

	list_add_tail(&buf->queue, &mx5_cam->capture);

	spin_unlock_irqrestore(&mx5_cam->lock, flags);
}

static void mx5_videobuf_release(struct vb2_buffer *vb)
{
	struct soc_camera_device *icd = soc_camera_from_vb2q(vb->vb2_queue);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct mx5_camera_dev *mx5_cam = ici->priv;
	struct mx5_buffer *buf = to_mx5_vb(vb);
	unsigned long flags;

	spin_lock_irqsave(&mx5_cam->lock, flags);

	if (mx5_cam->active == buf)
		mx5_cam->active = NULL;

	if (buf->init)
		list_del_init(&buf->queue);

	spin_unlock_irqrestore(&mx5_cam->lock, flags);
}

static int mx5_videobuf_init(struct vb2_buffer *vb)
{
	struct mx5_buffer *buf = to_mx5_vb(vb);

	/* This is for locking debugging only */
	INIT_LIST_HEAD(&buf->queue);

	buf->init = 1;

	return 0;
}

#define pixfmtstr(x) (x) & 0xff, ((x) >> 8) & 0xff, ((x) >> 16) & 0xff, \
        ((x) >> 24) & 0xff

static int mx5_videobuf_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct soc_camera_device *icd = soc_camera_from_vb2q(vq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct mx5_camera_dev *mx5_cam = ici->priv;
	int ret;
	int xres = icd->user_width, yres = icd->user_height;
	struct ipu_ch_param *cpmem = ipu_get_cpmem(mx5_cam->ipuch);
	struct device *dev = icd->parent;
	int capture_channel, burstsize;
	struct vb2_buffer *vb;
	struct mx5_buffer *buf;
	int nfack_irq;

	const struct soc_camera_format_xlate *xlate;
	xlate = soc_camera_xlate_by_fourcc(icd, mx5_cam->fourcc);
	if (!xlate) {
		dev_warn(icd->parent, "Format %c%c%c%c not found\n",
			 pixfmtstr(mx5_cam->fourcc));
		return -EINVAL;
	}

	memset(cpmem, 0, sizeof(*cpmem));

	nfack_irq = ipu_idmac_channel_irq(mx5_cam->ipu, mx5_cam->ipuch,
			IPU_IRQ_NFACK);
	ret = request_threaded_irq(nfack_irq, NULL, mx5cam_new_frame_handler, IRQF_ONESHOT,
			"mx5cam-nfack", mx5_cam);
	if (ret) {
		dev_err(dev, "Failed to request NFACK interrupt: %d\n", nfack_irq);
		return ret;
	}

	dev_dbg(dev, "width: %d height: %d, %c%c%c%c\n", icd->user_width, icd->user_height, pixfmtstr(xlate->host_fmt->fourcc));

	ipu_cpmem_set_resolution(cpmem, xres, yres);

	/*
	 * Since there are several possibilities to transmit YUV422 data,
	 * configure IDMAC depending on the exact wire format
	 */

	switch (xlate->host_fmt->fourcc) {
	case V4L2_PIX_FMT_RGB32:
		/* 24-bit RGB padded to 32 bits*/
		if (xlate->host_fmt == &mx5_camera_formats[1]) {
			/* Special case for RGB from the Test Image Generator */
			_ipu_csi_set_test_generator(mx5_cam, true, mx5_cam->id ? 0xff : 0x00, mx5_cam->id ? 0x00 : 0xff, 0x00);
		}
		ipu_cpmem_set_stride(cpmem, xres * 4);
		if (xlate->host_fmt->packing == SOC_MBUS_PACKING_4X8_PADHI) {
			struct ipu_rgb rgb = {
				.red = { 16, 8, 0 },
				.green = { 8, 8, 0 },
				.blue = { 0, 8, 0 },
				.bits_per_pixel = 32,
			};
			ipu_cpmem_set_format_rgb(cpmem, &rgb);
		}
		break;
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_YUYV:
		/* 16-bit YUV 4:2:2 */
		if (xlate->host_fmt->bits_per_sample == 16) {
			/* pass through raw 16-bit data */
			ipu_cpmem_set_stride(cpmem, xres * 2);
			ipu_cpmem_set_format_passthrough(cpmem, 16);
		} else if (xlate->host_fmt->bits_per_sample == 8) {
			/* two 8-bit components */
			ipu_cpmem_set_stride(cpmem, xres * 2);
			ipu_cpmem_set_yuv_interleaved(cpmem, xlate->host_fmt->fourcc);
		}
		break;
	case V4L2_PIX_FMT_YUV420:
		ipu_cpmem_set_stride(cpmem, xres);
		ipu_cpmem_set_yuv_planar(cpmem, V4L2_PIX_FMT_YUV420, xres, yres);
		break;
	default:
		dev_err(dev, "Fourcc not supported: %x\n", mx5_cam->fourcc);
		ret = -EINVAL;
		goto free_irq;
	}

	capture_channel = mx5_cam->id; /* CSI0: channel 0, CSI1: channel 1 */

	/* Configure SMFC Burst Size from CPMEM, according to Table 39-683 */
	if ((ipu_ch_param_read_field(cpmem, IPU_FIELD_PFS) == 6) && /* passthrough */
	    (ipu_ch_param_read_field(cpmem, IPU_FIELD_BPP) == 5 /* 8-bit */ ||
	     ipu_ch_param_read_field(cpmem, IPU_FIELD_BPP) == 3 /* 16-bit */)) {
		burstsize = (ipu_ch_param_read_field(cpmem, IPU_FIELD_NPB) + 1) >> 4;
	} else {
		burstsize = (ipu_ch_param_read_field(cpmem, IPU_FIELD_NPB) + 1) >> 2;
	}
	dev_dbg(dev, "would program SMFC BURST_SIZE=%d\n", burstsize);

	burstsize = 16;
	dev_dbg(dev, "programming SMFC BURST_SIZE=%d\n", burstsize);
	ipu_smfc_set_burstsize(mx5_cam->ipu, capture_channel, burstsize - 1);
	ipu_smfc_map_channel(mx5_cam->ipu, capture_channel, mx5_cam->id, 0);

	ipu_cpmem_set_high_priority(mx5_cam->ipuch);

	ipu_csi_set_window_size(mx5_cam, xres, yres);
	ipu_csi_set_window_pos(mx5_cam, 0, 0);

	ret = ipu_csi_init_interface(mx5_cam, xres, yres, mx5_cam->sens_conf);
	if (ret)
		goto free_irq;

	ipu_idmac_set_double_buffer(mx5_cam->ipuch, 1);

	if (list_empty(&mx5_cam->capture)) {
		dev_err(dev, "No capture buffers\n");
		ret = -ENOMEM;
		goto free_irq;
	}
	mx5_cam->active = NULL;

	/* Point the inactive buffer address to the first buffer */
	buf = list_first_entry(&mx5_cam->capture, struct mx5_buffer, queue);
	vb = &buf->vb;
	mx5cam_set_inactive_buffer(mx5_cam, vb);

	ipu_idmac_enable_channel(mx5_cam->ipuch);
	ipu_csi_enable(mx5_cam->ipu, mx5_cam->id);
	ipu_smfc_enable(mx5_cam->ipu);

	return 0;

free_irq:
	free_irq(nfack_irq, mx5_cam);
	return ret;
}

static int mx5_videobuf_stop_streaming(struct vb2_queue *vq)
{
	struct soc_camera_device *icd = soc_camera_from_vb2q(vq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct mx5_camera_dev *mx5_cam = ici->priv;
	int nfack_irq = ipu_idmac_channel_irq(mx5_cam->ipu, mx5_cam->ipuch,
				IPU_IRQ_NFACK);

	free_irq(nfack_irq, mx5_cam);
	ipu_idmac_disable_channel(mx5_cam->ipuch);
	ipu_csi_disable(mx5_cam->ipu, mx5_cam->id);
	ipu_smfc_disable(mx5_cam->ipu);

	if (mx5_cam->fourcc == V4L2_PIX_FMT_RGB32)
		/* RGB from Test Mode: 24 bit RGB padded to 32 bits*/
		_ipu_csi_set_test_generator(mx5_cam, 0, 0x00, 0xff, 0x00);

	return 0;
}

static struct vb2_ops mx5_videobuf_ops = {
	.queue_setup		= mx5_videobuf_setup,
	.buf_prepare		= mx5_videobuf_prepare,
	.buf_queue		= mx5_videobuf_queue,
	.buf_cleanup		= mx5_videobuf_release,
	.buf_init		= mx5_videobuf_init,
	.start_streaming	= mx5_videobuf_start_streaming,
	.stop_streaming		= mx5_videobuf_stop_streaming,
	.wait_prepare		= soc_camera_unlock,
	.wait_finish		= soc_camera_lock,
};

static int mx5_camera_init_videobuf(struct vb2_queue *q,
				     struct soc_camera_device *icd)
{
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_USERPTR;
	q->drv_priv = icd;
	q->ops = &mx5_videobuf_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->buf_struct_size = sizeof(struct mx5_buffer);

	return vb2_queue_init(q);
}

static int mx5_camera_get_formats(struct soc_camera_device *icd, unsigned int idx,
				  struct soc_camera_format_xlate *xlate)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct device *dev = icd->parent;
	int formats = 0, ret;
	enum v4l2_mbus_pixelcode code;
	const struct soc_mbus_pixelfmt *fmt;

	ret = v4l2_subdev_call(sd, video, enum_mbus_fmt, idx, &code);
	if (ret < 0)
		/* No more formats */
		return 0;

	if (code == V4L2_MBUS_FMT_FIXED) {
		formats++;
		if (xlate) {
			xlate->host_fmt = &mx5_camera_formats[1];
			xlate->code	= code;
			dev_dbg(dev, "Providing format %s using code %d\n",
				mx5_camera_formats[1].name, code);
		}
		return formats;
	}

	fmt = soc_mbus_get_fmtdesc(code);
	if (!fmt) {
		dev_err(icd->parent,
			"Invalid format code #%u: %d\n", idx, code);
		return 0;
	}

	switch (code) {
	case V4L2_MBUS_FMT_UYVY8_2X8:
		formats++;
		if (xlate) {
			xlate->host_fmt	= &mx5_camera_formats[2];
			xlate->code	= code;
			xlate++;
			dev_dbg(dev, "Providing format %s using code %d\n",
				mx5_camera_formats[2].name, code);
		}
		break;
	case V4L2_MBUS_FMT_UYVY8_1X16:
		formats++;
		if (xlate) {
			xlate->host_fmt	= &mx5_camera_formats[0];
			xlate->code	= code;
			xlate++;
			dev_dbg(dev, "Providing format %s using code %d\n",
				mx5_camera_formats[0].name, code);
		}
		break;
	default:
		break;
	}

	/* Generic pass-through */
	formats++;
	if (xlate) {
		xlate->host_fmt	= fmt;
		xlate->code	= code;
		dev_dbg(dev, "Providing format %c%c%c%c in pass-through mode\n",
			(fmt->fourcc >> (0*8)) & 0xFF,
			(fmt->fourcc >> (1*8)) & 0xFF,
			(fmt->fourcc >> (2*8)) & 0xFF,
			(fmt->fourcc >> (3*8)) & 0xFF);
		xlate++;
	}

	return formats;
}

static void mx5_camera_activate(struct mx5_camera_dev *mx5_cam)
{
	dev_dbg(mx5_cam->icd->parent, "Activate device\n");
}

static int mx5_camera_add_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct mx5_camera_dev *mx5_cam = ici->priv;
	int ret;

	if (mx5_cam->icd) {
		ret = -EBUSY;
		goto ebusy;
	}

	dev_info(icd->parent, "MX5 Camera driver attached to camera %d\n",
		 icd->devnum);

	mx5_cam->icd = icd;

	mx5_camera_activate(mx5_cam);

	return 0;

ebusy:
	return ret;
}

static void mx5_camera_remove_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct mx5_camera_dev *pcdev = ici->priv;

	BUG_ON(icd != pcdev->icd);

	dev_info(icd->parent, "MX5 Camera driver detached from camera %d\n",
		 icd->devnum);

	pcdev->icd = NULL;
}

static int mx5_camera_set_crop(struct soc_camera_device *icd,
			       const struct v4l2_crop *a)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);

	return v4l2_subdev_call(sd, video, s_crop, a);
}

static int mx5_camera_set_fmt(struct soc_camera_device *icd,
			      struct v4l2_format *f)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct mx5_camera_dev *mx5_cam = ici->priv;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	const struct soc_camera_format_xlate *xlate;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_mbus_framefmt mf;
	int ret;

	xlate = soc_camera_xlate_by_fourcc(icd, pix->pixelformat);
	if (!xlate) {
		dev_warn(icd->parent, "Format %c%c%c%c not found\n",
			 pixfmtstr(pix->pixelformat));
		return -EINVAL;
	}

	mf.width	= pix->width;
	mf.height	= pix->height;
	mf.field	= pix->field;
	mf.colorspace	= pix->colorspace;
	mf.code		= xlate->code;

	if (mf.width > 4096 || mf.height > 4096)
		return -EINVAL;

	ret = v4l2_subdev_call(sd, video, s_mbus_fmt, &mf);
	if (ret < 0)
		return ret;

	if (mf.code != xlate->code)
		return -EINVAL;

	mx5_cam->fourcc = f->fmt.pix.pixelformat;

	pix->width		= mf.width;
	pix->height		= mf.height;
	pix->field		= mf.field;
	pix->colorspace		= mf.colorspace;
	icd->current_fmt	= xlate;

	return ret;
}

static int mx5_camera_try_fmt(struct soc_camera_device *icd,
			      struct v4l2_format *f)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct mx5_camera_dev *mx5_cam = ici->priv;
	const struct soc_camera_format_xlate *xlate;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_mbus_framefmt mf;
	int ret;

	xlate = soc_camera_xlate_by_fourcc(icd, pix->pixelformat);
	if (!xlate) {
		dev_warn(icd->parent, "Format %c%c%c%c not found\n",
			 pixfmtstr(pix->pixelformat));
		return -EINVAL;
	}

	mf.width	= pix->width;
	mf.height	= pix->height;
	mf.field	= pix->field;
	mf.colorspace	= pix->colorspace;
	mf.code		= xlate->code;

	/* limit to host interface capabilities */
	if (mf.width > 4096)
		mf.width = 4096;
	if (mf.height > 4096)
		mf.height = 4096;

	if (mx5_cam->sens_conf & CSI_SENS_PRTCL_BT656_INTERLACED) {
		mf.width = 720;
		mf.height = 576;
	}

	/* limit to sensor capabilities */
	ret = v4l2_subdev_call(sd, video, try_mbus_fmt, &mf);
	if (ret < 0)
		return ret;

	pix->width	= mf.width;
	pix->height	= mf.height;
	pix->field	= mf.field;
	pix->colorspace	= mf.colorspace;

	return 0;
}

static int mx5_camera_reqbufs(struct soc_camera_device *icd,
			      struct v4l2_requestbuffers *p)
{
	return 0;
}

static int mx5_camera_querycap(struct soc_camera_host *ici,
			       struct v4l2_capability *cap)
{
	/* cap->name is set by the friendly caller:-> */
	strlcpy(cap->card, "imx-ipuv3-camera", sizeof(cap->card));
	cap->version = 0;
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;

	return 0;
}

static int mx5_camera_set_bus_param(struct soc_camera_device *icd)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct mx5_camera_dev *mx5_cam = ici->priv;
	struct v4l2_mbus_config cfg = {.type = V4L2_MBUS_PARALLEL,};
	u32 pixfmt = icd->current_fmt->host_fmt->fourcc;
	unsigned int common_flags;
	u32 dw, sens_conf, data_fmt;
	int ret;
	const struct soc_camera_format_xlate *xlate;
	struct device *dev = icd->parent;

	xlate = soc_camera_xlate_by_fourcc(icd, pixfmt);
	if (!xlate) {
		dev_err(dev, "Format %x not found\n", pixfmt);
		return -EINVAL;
	}

	ret = v4l2_subdev_call(sd, video, g_mbus_config, &cfg);
	if (!ret) {
		common_flags = soc_mbus_config_compatible(&cfg,
							  mx5_cam->bus_flags);
		if (!common_flags) {
			dev_warn(icd->parent,
				 "Flags incompatible: camera 0x%x, host 0x%x\n",
				 cfg.flags, mx5_cam->bus_flags);
			return -EINVAL;
		}
	} else if (ret != -ENOIOCTLCMD) {
		dev_err(dev, "video->g_mbus_config failed\n");
		return ret;
	} else {
		common_flags = mx5_cam->bus_flags;
	}

	dev_dbg(dev, "Flags cam: 0x%x host: 0x%x common: 0x%x\n",
		cfg.flags, mx5_cam->bus_flags, common_flags);

	cfg.flags = common_flags;
	ret = v4l2_subdev_call(sd, video, s_mbus_config, &cfg);
	if (ret < 0 && ret != -ENOIOCTLCMD) {
		dev_dbg(dev, "camera s_mbus_config(0x%x) returned %d\n",
			common_flags, ret);
		return ret;
	}

	/* Clear the SENS_CONF fields that we are going to set */
	sens_conf = mx5_cam->sens_conf &
		~((1 << CSI_SENS_CONF_VSYNC_POL_SHIFT) |
		  (1 << CSI_SENS_CONF_HSYNC_POL_SHIFT) |
		  (1 << CSI_SENS_CONF_DATA_POL_SHIFT) |
		  (1 << CSI_SENS_CONF_PIX_CLK_POL_SHIFT) |
		  CSI_SENS_CONF_SENS_PRTCL_MASK |
		  CSI_SENS_CONF_DATA_FMT_MASK |
		  CSI_SENS_CONF_DATA_WIDTH_MASK |
		  (1 << CSI_SENS_CONF_EXT_VSYNC_SHIFT));

	if (common_flags & V4L2_MBUS_SLAVE) {
		dev_dbg(dev, "Slave 'sensor': use external VSYNC and non-gated clock\n");

		sens_conf |= CSI_SENS_PRTCL_NON_GATED;
		sens_conf |= 1 << CSI_SENS_CONF_EXT_VSYNC_SHIFT;
	} else {
		switch (cfg.type) {
		case V4L2_MBUS_PARALLEL:
			sens_conf |= CSI_SENS_PRTCL_GATED;
			break;
		case V4L2_MBUS_BT656:
			sens_conf |= CSI_SENS_PRTCL_BT656_INTERLACED;
			/*
			 * FIXME: XXX
			 * take care about non interlaced modes, too
			 */
			break;
		default:
			dev_err(dev, "unsupported bus type %d\n", cfg.type);
			return -EINVAL;
		}
	}

	/* Data format */
	switch (xlate->host_fmt->fourcc) {
	case V4L2_PIX_FMT_YUYV:
		if (xlate->host_fmt->bits_per_sample == 8)
			data_fmt = CSI_SENS_CONF_DATA_FMT_YUV422_YUYV;
		else
			data_fmt = CSI_SENS_CONF_DATA_FMT_BAYER;
		break;
	case V4L2_PIX_FMT_UYVY:
		if (xlate->host_fmt->bits_per_sample == 8)
			data_fmt = CSI_SENS_CONF_DATA_FMT_YUV422_UYVY;
		else
			data_fmt = CSI_SENS_CONF_DATA_FMT_BAYER;
		break;
	case V4L2_PIX_FMT_RGB32:
	case V4L2_PIX_FMT_RGB24:
	case V4L2_PIX_FMT_BGR24:
		data_fmt = CSI_SENS_CONF_DATA_FMT_RGB_YUV444;
		break;
	case V4L2_PIX_FMT_RGB565:
		data_fmt = CSI_SENS_CONF_DATA_FMT_RGB565;
		break;
	case V4L2_PIX_FMT_RGB555:
		data_fmt = CSI_SENS_CONF_DATA_FMT_RGB555;
		break;
	default:
		dev_dbg(dev, "Unhandled fourcc %c%c%c%c\n",
			(xlate->host_fmt->fourcc >> (0*8)) & 0xff,
			(xlate->host_fmt->fourcc >> (1*8)) & 0xff,
			(xlate->host_fmt->fourcc >> (2*8)) & 0xff,
			(xlate->host_fmt->fourcc >> (3*8)) & 0xff);
		return -EINVAL;
	}
	sens_conf |= data_fmt;

	/* Signal polarity */
	if (common_flags & V4L2_MBUS_PCLK_SAMPLE_FALLING)
		sens_conf |= 1 << CSI_SENS_CONF_PIX_CLK_POL_SHIFT;
	if (common_flags & V4L2_MBUS_HSYNC_ACTIVE_LOW)
		sens_conf |= 1 << CSI_SENS_CONF_HSYNC_POL_SHIFT;
	if (common_flags & V4L2_MBUS_VSYNC_ACTIVE_LOW)
		sens_conf |= 1 << CSI_SENS_CONF_VSYNC_POL_SHIFT;
	if (common_flags & V4L2_MBUS_DATA_ACTIVE_LOW)
		sens_conf |= 1 << CSI_SENS_CONF_DATA_POL_SHIFT;

	/* Just do what we're asked to do */
	switch (xlate->host_fmt->bits_per_sample) {
	case 8:
		dw = 1 << CSI_SENS_CONF_DATA_WIDTH_SHIFT;
		break;
	case 10:
		dw = 3 << CSI_SENS_CONF_DATA_WIDTH_SHIFT;
		break;
	case 16:
		dw = 9 << CSI_SENS_CONF_DATA_WIDTH_SHIFT;
		break;
	default:
		dev_err(dev, "Invalid bits per sample: %d\n", xlate->host_fmt->bits_per_sample);
		return -EINVAL;
	}

	mx5_cam->sens_conf = sens_conf | dw;

	dev_dbg(dev, "Set SENS_CONF to 0x%08x\n", sens_conf | dw);

	return 0;
}

int mx5_camera_get_parm(struct soc_camera_device *icd,
			struct v4l2_streamparm *parms)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct mx5_camera_dev *mx5_cam = ici->priv;
	struct v4l2_captureparm *cp = &parms->parm.capture;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct device *dev = icd->parent;

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	if (mx5_cam->sens_conf & (1 << CSI_SENS_CONF_EXT_VSYNC_SHIFT)) {
		/* For slave sensors, calculate frame interval from MCLK */
		int pix_clk = ipu_csi_mclk_get_rate(mx5_cam);
		int xres = icd->user_width, yres = icd->user_height;

		dev_dbg(dev, "%s: pixel clock = %d\n", __func__, pix_clk);

		memset(cp, 0, sizeof(struct v4l2_captureparm));
		cp->capability = V4L2_CAP_TIMEPERFRAME;
		cp->timeperframe.numerator = xres * yres * 3 /* 24-bit RGB */;
		cp->timeperframe.denominator = pix_clk;

		dev_dbg(dev, "%s: pixel clock: %d, current frame time %d/%d\n",
			__func__, pix_clk, cp->timeperframe.numerator,
			cp->timeperframe.denominator);

		return 0;
	} else {
		/* For master sensors, ask the sensor */
		return v4l2_subdev_call(sd, video, g_parm, parms);
	}
}

int mx5_camera_set_parm(struct soc_camera_device *icd,
			struct v4l2_streamparm *parms)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct mx5_camera_dev *mx5_cam = ici->priv;
	struct v4l2_captureparm *cp = &parms->parm.capture;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct device *dev = icd->parent;
	int ret;

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	if (mx5_cam->sens_conf & (1 << CSI_SENS_CONF_EXT_VSYNC_SHIFT)) {
		int xres = icd->user_width, yres = icd->user_height;
		int pix_clk = xres * yres * 3 /* 24-bit RGB */
			      * cp->timeperframe.denominator
			      / cp->timeperframe.numerator;

		/* For slave sensors, set MCLK according to frame rate */
		dev_dbg(dev, "%s: set frame time %d/%d, pixel clock: %u\n",
			__func__, cp->timeperframe.numerator,
			cp->timeperframe.denominator, pix_clk);

		/* Set sensb_mclk div_ratio*/
		ret = ipu_csi_mclk_set_rate(mx5_cam, pix_clk);
		if (ret < 0)
			return ret;

		return 0;
	} else {
		/* For master sensors, tell the sensor */
		return v4l2_subdev_call(sd, video, s_parm, parms);
	}
}

static unsigned int mx5_camera_poll(struct file *file, poll_table *pt)
{
	struct soc_camera_device *icd = file->private_data;

	return vb2_poll(&icd->vb2_vidq, file, pt);
}

static struct soc_camera_host_ops mx5_soc_camera_host_ops = {
	.owner		= THIS_MODULE,
	.add		= mx5_camera_add_device,
	.remove		= mx5_camera_remove_device,
	.get_formats	= mx5_camera_get_formats,
	.set_crop	= mx5_camera_set_crop,
	.set_fmt	= mx5_camera_set_fmt,
	.try_fmt	= mx5_camera_try_fmt,
	.init_videobuf2	= mx5_camera_init_videobuf,
	.reqbufs	= mx5_camera_reqbufs,
	.querycap	= mx5_camera_querycap,
	.set_bus_param	= mx5_camera_set_bus_param,
	.get_parm	= mx5_camera_get_parm,
	.set_parm	= mx5_camera_set_parm,
	.poll		= mx5_camera_poll,
};

static u64 camera_mask = DMA_BIT_MASK(32);

static int __devinit mx5_camera_probe(struct platform_device *pdev)
{
        struct ipu_client_platformdata *pdata = pdev->dev.platform_data;
	struct ipu_soc *ipu = dev_get_drvdata(pdev->dev.parent);
	struct mx5_camera_dev *mx5_cam;
	struct resource *res;
	int err;

	pdev->dev.dma_mask		= &camera_mask,
	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32),

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	if (!pdata)
		return -EINVAL;

	mx5_cam = devm_kzalloc(&pdev->dev, sizeof(*mx5_cam), GFP_KERNEL);
	if (!mx5_cam)
		return -ENOMEM;

	mx5_cam->bus_flags = V4L2_MBUS_MASTER | V4L2_MBUS_SLAVE |
			     V4L2_MBUS_HSYNC_ACTIVE_HIGH | V4L2_MBUS_HSYNC_ACTIVE_LOW |
			     V4L2_MBUS_VSYNC_ACTIVE_HIGH | V4L2_MBUS_VSYNC_ACTIVE_LOW |
			     V4L2_MBUS_PCLK_SAMPLE_RISING | V4L2_MBUS_PCLK_SAMPLE_FALLING |
			     V4L2_MBUS_DATA_ACTIVE_HIGH | V4L2_MBUS_DATA_ACTIVE_LOW;

	mx5_cam->hclk = clk_get(pdev->dev.parent, "bus");
	if (IS_ERR(mx5_cam->hclk)) {
		err = PTR_ERR(mx5_cam->hclk);
		dev_warn(&pdev->dev, "clk_get failed with %d\n", err);
		/* Without HCLK, MCLK cannot be provided to the camera module */
		mx5_cam->bus_flags &= ~V4L2_MBUS_SLAVE;
	}

	/* pdev->id = 0: IPU1 CSI0, 1: IPU1 CSI1, 2: IPU2 CSI0, 3: IPU2 CSI1 */
	mx5_cam->id = pdata->csi; /* CSI0 or CSI1 */
	mx5_cam->ipu = ipu;

	mx5_cam->ipuch = ipu_idmac_get(ipu, pdata->dma[0]);
	if (!mx5_cam->ipuch)
		return -EBUSY;

	INIT_LIST_HEAD(&mx5_cam->capture);
	spin_lock_init(&mx5_cam->lock);

	mx5_cam->base = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!mx5_cam->base) {
		dev_err(&pdev->dev, "Couldn't map %x@%x\n", resource_size(res), res->start);
		return -ENOMEM;
	}

	dev_dbg(&pdev->dev, "mapped %x -> %lx, using dmach %d\n", res->start, (unsigned long)mx5_cam->base, pdata->dma[0]);

	mx5_cam->soc_host.drv_name	= DRIVER_NAME;
	mx5_cam->soc_host.ops		= &mx5_soc_camera_host_ops;
	mx5_cam->soc_host.priv		= mx5_cam;
	mx5_cam->soc_host.v4l2_dev.dev	= &pdev->dev;

	/*
	 * mx5_camera is instantiated from device tree, so encode the IPU
	 * device node's register address and the CSI index into the bus
	 * identifier. Those too can be obtained from the phandle+index
	 * that are referenced from sensors in the device tree.
	 */
	mx5_cam->soc_host.nr = of_soc_camera_host_nr(pdev->dev.parent->of_node,
					pdata->csi);

	mx5_cam->alloc_ctx = vb2_dma_contig_init_ctx(&pdev->dev);
	if (IS_ERR(mx5_cam->alloc_ctx)) {
		err = PTR_ERR(mx5_cam->alloc_ctx);
		goto failed_vb2;
	}

	err = soc_camera_host_register(&mx5_cam->soc_host);
	if (err)
		goto failed_register;

	platform_set_drvdata(pdev, mx5_cam);

	dev_info(&pdev->dev, "MX5 Camera driver loaded\n");

	return 0;

failed_register:
	vb2_dma_contig_cleanup_ctx(mx5_cam->alloc_ctx);
failed_vb2:
	ipu_idmac_put(mx5_cam->ipuch);
	return err;
}

static int __exit mx5_camera_remove(struct platform_device *pdev)
{
	struct mx5_camera_dev *mx5_cam = platform_get_drvdata(pdev);

	soc_camera_host_unregister(&mx5_cam->soc_host);
	vb2_dma_contig_cleanup_ctx(mx5_cam->alloc_ctx);
	ipu_idmac_put(mx5_cam->ipuch);

	return 0;
}

static struct platform_driver mx5_camera_driver = {
	.driver = {
		.name = DRIVER_NAME,
	},
	.probe = mx5_camera_probe,
	.remove = __exit_p(mx5_camera_remove),
};

static int __init mx5_camera_init(void)
{
	return platform_driver_register(&mx5_camera_driver);
}

static void __exit mx5_camera_exit(void)
{
	return platform_driver_unregister(&mx5_camera_driver);
}

module_init(mx5_camera_init);
module_exit(mx5_camera_exit);

MODULE_DESCRIPTION("i.MX51/53 SoC Camera Host driver");
MODULE_AUTHOR("Sascha Hauer <s.hauer@pengutronix.de>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
