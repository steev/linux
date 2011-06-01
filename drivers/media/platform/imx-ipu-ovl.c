/*
 * i.MX IPUv3 overlay driver
 *
 * Copyright (C) 2011 Sascha Hauer, Pengutronix
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <media/v4l2-dev.h>
#include <asm/poll.h>
#include <asm/io.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>
#include "../../staging/imx-drm/ipu-v3/imx-ipu-v3.h"
#include "../../staging/imx-drm/ipu-v3/ipu-prv.h"

#include <media/v4l2-dev.h>

#include "imx-ipu.h"

static int usealpha;

module_param(usealpha, int, 0);

static int allow_dynamic_resize;

module_param(allow_dynamic_resize, int, 0);

struct vout_buffer {
	struct vb2_buffer		vb;
};

#define vb2q_to_vout(q)	container_of(q, struct vout_data, vidq)

enum {
	VOUT_IDLE,
	VOUT_STARTING,
	VOUT_RUNNING,
	VOUT_STOPPING,
};

struct vout_queue {
	struct ipu_image	image;
	void			*virt;
	dma_addr_t		phys;
	size_t			size;
	struct list_head	list;
	struct vb2_buffer	*vb;
	struct vout_data	*vout;
};

#define NUMBUFS	3

struct vout_data {
	struct video_device	*video_dev;

	int			status;

	struct ipu_soc		*ipu;
	struct ipuv3_channel	*ipu_ch;
	struct dmfc_channel	*dmfc;
	struct ipu_dp		*dp;

	struct vb2_queue	vidq;

	struct vb2_alloc_ctx	*alloc_ctx;
	spinlock_t		lock;
	struct device		*dev;

	int			irq;

	struct ipu_image	out_image; /* output image */
	struct ipu_image	in_image; /* input image */
	struct v4l2_window	win; /* user selected output window (after scaler) */
	struct v4l2_rect	crop; /* cropping rectangle in the input image */

	struct list_head	idle_list;
	struct list_head	scale_list;
	struct list_head	show_list;

	int			width_base;
	int			height_base;

	int			opened;
};

static int vidioc_querycap(struct file *file, void  *priv,
		struct v4l2_capability *cap)
{
	strcpy(cap->driver, "i.MX v4l2 output");
	cap->version = 0;
	cap->capabilities = V4L2_CAP_VIDEO_OUTPUT | V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_OVERLAY;
	cap->card[0] = '\0';
	cap->bus_info[0] = '\0';

	return 0;
}

static int vidioc_cropcap(struct file *file, void *priv,
		struct v4l2_cropcap *cropcap)
{
	struct video_device *dev = video_devdata(file);
	struct vout_data *vout = video_get_drvdata(dev);
	struct v4l2_pix_format *pix = &vout->in_image.pix;

	if (cropcap->type != V4L2_BUF_TYPE_VIDEO_OUTPUT &&
		cropcap->type != V4L2_BUF_TYPE_VIDEO_OVERLAY)
		return -EINVAL;

	cropcap->bounds.left = 0;
	cropcap->bounds.top = 0;
	cropcap->bounds.width = pix->width;
	cropcap->bounds.height = pix->height;
	cropcap->defrect.left = 0;
	cropcap->defrect.top = 0;
	cropcap->defrect.width = pix->width;
	cropcap->defrect.height = pix->height;
	cropcap->pixelaspect.numerator = 1;
	cropcap->pixelaspect.denominator = 1;

	return 0;
}

static int ipu_ovl_vidioc_g_fmt_vid_out(struct file *file, void *fh,
					struct v4l2_format *f)
{
	struct video_device *dev = video_devdata(file);
	struct vout_data *vout = video_get_drvdata(dev);

	if (f->type != V4L2_BUF_TYPE_VIDEO_OUTPUT &&
		f->type != V4L2_BUF_TYPE_VIDEO_OVERLAY)
		return -EINVAL;

	return ipu_g_fmt(f, &vout->out_image.pix);
}

static int ipu_ovl_vidioc_try_fmt_vid_out(struct file *file, void *fh,
					struct v4l2_format *f)
{
	return ipu_try_fmt(file, fh, f);
}

/*
 * This function is a major hack. We can't leave the base framebuffer
 * with the overlay. To make this sure we read directly from cpmem
 * of the base layer. We rather need some notification mechanism
 * for this.
 */
static void ipu_ovl_get_base_resolution(struct vout_data *vout)
{
	struct ipu_ch_param *cpmem_base = vout->ipu->cpmem_base + 23;

	vout->width_base = ipu_ch_param_read_field(cpmem_base, IPU_FIELD_FW) + 1;
	vout->height_base = ipu_ch_param_read_field(cpmem_base, IPU_FIELD_FH) + 1;
}

static void ipu_ovl_sanitize(struct vout_data *vout)
{
	struct ipu_image *in = &vout->in_image;
	struct ipu_image *out = &vout->out_image;
	struct v4l2_rect *crop = &vout->crop;
	struct v4l2_window *win = &vout->win;

	ipu_ovl_get_base_resolution(vout);

	/* Do not allow to leave base framebuffer for now */
	if (win->w.left < 0)
		win->w.left = 0;
	if (win->w.top  < 0)
		win->w.top = 0;
	if (win->w.left + win->w.width > vout->width_base)
		win->w.left = vout->width_base - win->w.width;
	if (win->w.top + win->w.height > vout->height_base)
		win->w.top = vout->height_base - win->w.height;

	dev_dbg(vout->dev, "start: win:  %dx%d@%dx%d\n",
			win->w.width, win->w.height, win->w.left, win->w.top);

	in->rect.left = crop->left;
	in->rect.width = crop->width;
	in->rect.top = crop->top;
	in->rect.height = crop->height;

	out->pix.width = win->w.width;
	out->pix.height = win->w.height;
	out->rect.width = win->w.width;
	out->rect.height = win->w.height;
	out->rect.left = win->w.left;
	out->rect.top = win->w.top;

	out->rect.left = 0;
	out->rect.top = 0;

	dev_dbg(vout->dev, "result in: %dx%d crop: %dx%d@%dx%d\n",
			in->pix.width, in->pix.height, in->rect.width,
			in->rect.height, in->rect.left, in->rect.top);
	dev_dbg(vout->dev, "result out: %dx%d crop: %dx%d@%dx%d\n",
			out->pix.width, out->pix.height, out->rect.width,
			out->rect.height, out->rect.left, out->rect.top);
}

static int ipu_ovl_vidioc_s_fmt_vid_out(struct file *file, void *fh,
					struct v4l2_format *f)
{
	struct video_device *dev = video_devdata(file);
	struct vout_data *vout = video_get_drvdata(dev);
	struct v4l2_pix_format *pix = &vout->in_image.pix;
	int ret;

	ret = ipu_s_fmt(file, fh, f, pix);
	if (ret)
		return ret;

	vout->win.w.left = 0;
	vout->win.w.top = 0;
	vout->win.w.width = pix->width;
	vout->win.w.height = pix->height;

	vout->crop.left = 0;
	vout->crop.top = 0;
	vout->crop.width = pix->width;
	vout->crop.height = pix->height;

	ipu_ovl_sanitize(vout);

	return 0;
}

static int ipu_ovl_vidioc_g_fmt_vid_overlay(struct file *file, void *fh,
					struct v4l2_format *f)
{
	struct video_device *dev = video_devdata(file);
	struct vout_data *vout = video_get_drvdata(dev);

	f->fmt.win = vout->win;

	return 0;
}

static int ipu_ovl_vidioc_try_fmt_vid_overlay(struct file *file, void *fh,
					struct v4l2_format *f)
{
	struct video_device *dev = video_devdata(file);
	struct vout_data *vout = video_get_drvdata(dev);
	struct v4l2_window *win = &f->fmt.win;

	win->w.width &= ~0x3;
	win->w.height &= ~0x1;

	dev_dbg(vout->dev, "%s: %dx%d@%dx%d \n", __func__,
			win->w.width, win->w.height, win->w.left, win->w.top);

	return 0;
}

static int ipu_ovl_vidioc_s_fmt_vid_overlay(struct file *file, void *fh,
					struct v4l2_format *f)
{
	struct video_device *dev = video_devdata(file);
	struct vout_data *vout = video_get_drvdata(dev);
	struct v4l2_window *win = &f->fmt.win;

	win->w.width &= ~0x3;
	win->w.height &= ~0x1;

	dev_dbg(vout->dev, "%s: %dx%d@%dx%d \n", __func__,
			win->w.width, win->w.height, win->w.left, win->w.top);

	vout->win.w.width = win->w.width;
	vout->win.w.height = win->w.height;
	vout->win.w.left = win->w.left;
	vout->win.w.top = win->w.top;

	ipu_ovl_sanitize(vout);

	return 0;
}

static int ipu_ovl_vidioc_g_crop(struct file *file, void *fh,
				 struct v4l2_crop *crop)
{
	struct video_device *dev = video_devdata(file);
	struct vout_data *vout = video_get_drvdata(dev);

	crop->c = vout->crop;

	return 0;
}

static int ipu_ovl_vidioc_s_crop(struct file *file, void *fh,
				 const struct v4l2_crop *crop)
{
	struct video_device *dev = video_devdata(file);
	struct vout_data *vout = video_get_drvdata(dev);

	vout->crop = crop->c;

	ipu_ovl_sanitize(vout);

	return 0;
}

static int vout_videobuf_setup(struct vb2_queue *vq, const struct v4l2_format *fmt,
		unsigned int *count, unsigned int *num_planes,
		unsigned int sizes[], void *alloc_ctxs[])
{
	struct vout_data *vout = vb2q_to_vout(vq);
	struct ipu_ch_param *cpmem = ipu_get_cpmem(vout->ipu_ch);
	struct ipu_image *image = &vout->in_image;

	memset(cpmem, 0, sizeof(*cpmem));

	*num_planes = 1;
	sizes[0] = image->pix.sizeimage;
	alloc_ctxs[0] = vout->alloc_ctx;

	if (!*count)
		*count = 32;

	ipu_dp_set_global_alpha(vout->dp, usealpha ? 0 : 1, 0, 1);

	return 0;
}

static int vout_videobuf_prepare(struct vb2_buffer *vb)
{
	struct vout_data *vout = vb2q_to_vout(vb->vb2_queue);
	struct v4l2_pix_format *pix = &vout->in_image.pix;

	vb2_set_plane_payload(vb, 0, pix->bytesperline * pix->height);

	return 0;
}

static irqreturn_t vout_handler(int irq, void *context)
{
	struct vout_data *vout = context;
	unsigned long flags;
	struct vout_queue *q;
	struct ipu_ch_param *cpmem = ipu_get_cpmem(vout->ipu_ch);

	spin_lock_irqsave(&vout->lock, flags);

	if (vout->status == VOUT_IDLE)
		goto out;

	if (vout->status == VOUT_STOPPING) {
		list_splice_tail_init(&vout->show_list, &vout->idle_list);
		ipu_dp_disable_channel(vout->dp);
		vout->status = VOUT_IDLE;
		goto out;
	}

	if (list_is_singular(&vout->show_list))
		goto out;

	q = list_first_entry(&vout->show_list, struct vout_queue, list);

	vb2_buffer_done(q->vb, VB2_BUF_STATE_DONE);

	list_move_tail(&q->list, &vout->idle_list);

	q = list_first_entry(&vout->show_list, struct vout_queue, list);

	if (allow_dynamic_resize) {
		memset(cpmem, 0, sizeof(*cpmem));
		ipu_cpmem_set_image(cpmem, &q->image);
		ipu_cpmem_set_high_priority(vout->ipu_ch);
		ipu_dp_set_window_pos(vout->dp, vout->win.w.left, vout->win.w.top);
	} else {
		ipu_cpmem_set_buffer(cpmem, 0, q->image.phys);
	}
out:
	spin_unlock_irqrestore(&vout->lock, flags);

	return IRQ_HANDLED;
}

static int vout_enable(struct vout_queue *q)
{
	struct vout_data *vout = q->vout;
	struct ipu_image *image = &q->image;
	struct ipu_ch_param *cpmem = ipu_get_cpmem(vout->ipu_ch);
	int ret;

	ret = ipu_cpmem_set_image(cpmem, image);
	if (ret) {
		dev_err(vout->dev, "setup cpmem failed with %d\n", ret);
		return ret;
	}

	ipu_cpmem_set_high_priority(vout->ipu_ch);

	ret = ipu_dmfc_init_channel(vout->dmfc, image->pix.width);
	if (ret) {
		dev_err(vout->dev, "initializing dmfc channel failed with %d\n", ret);
		return ret;
	}

	ipu_dmfc_enable_channel(vout->dmfc);
	ipu_idmac_enable_channel(vout->ipu_ch);
	ipu_dp_setup_channel(vout->dp,
			ipu_pixelformat_to_colorspace(image->pix.pixelformat),
			IPUV3_COLORSPACE_UNKNOWN);
	ipu_dp_set_window_pos(vout->dp, vout->win.w.left, vout->win.w.top);
	ipu_dp_enable_channel(vout->dp);

	return 0;
}

static void vout_scaler_complete(void *context, int err)
{
	struct vout_queue *q = context;
	struct vout_data *vout = q->vout;
	unsigned long flags;

	spin_lock_irqsave(&vout->lock, flags);

	if (err) {
		vb2_buffer_done(q->vb, VB2_BUF_STATE_ERROR);
		list_move_tail(&q->list, &vout->idle_list);
		spin_unlock_irqrestore(&vout->lock, flags);
		return;
	}

	list_move_tail(&q->list, &vout->show_list);

	spin_unlock_irqrestore(&vout->lock, flags);

	if (vout->status == VOUT_STARTING) {
		vout_enable(q);
		vout->status = VOUT_RUNNING;
	}
}

static void vout_videobuf_queue(struct vb2_buffer *vb)
{
	struct vout_data *vout = vb2q_to_vout(vb->vb2_queue);
	unsigned long flags;
	struct ipu_image *image;
	struct vout_queue *q;
	int scale = 1;

	spin_lock_irqsave(&vout->lock, flags);

	if (list_empty(&vout->idle_list)) {
		vb2_buffer_done(vb, VB2_BUF_STATE_DONE);
		spin_unlock_irqrestore(&vout->lock, flags);
		return;
	}

	q = list_first_entry(&vout->idle_list, struct vout_queue, list);

	if (vout->in_image.rect.width == vout->out_image.rect.width &&
			vout->in_image.rect.height == vout->out_image.rect.height) {
		scale = 0;
		list_move_tail(&q->list, &vout->show_list);
	} else {
		list_move_tail(&q->list, &vout->scale_list);
	}

	q->vb = vb;

	image = &q->image;

	if (scale) {
		image->pix = vout->out_image.pix;
		image->rect = vout->out_image.rect;
		image->phys = q->phys;

		image->pix.pixelformat = V4L2_PIX_FMT_UYVY;
		image->pix.bytesperline = image->pix.width * 2;

		vout->in_image.phys = vb2_dma_contig_plane_dma_addr(vb, 0);

		ipu_image_convert(vout->ipu, &vout->in_image, image,
			vout_scaler_complete, q);
	} else {
		image->pix = vout->in_image.pix;
		image->rect = vout->in_image.rect;
		image->phys = vb2_dma_contig_plane_dma_addr(vb, 0);

		if (vout->status == VOUT_STARTING) {
			vout_enable(q);
			vout->status = VOUT_RUNNING;
		}
	}

	spin_unlock_irqrestore(&vout->lock, flags);
}

static void vout_videobuf_release(struct vb2_buffer *vb)
{
}

static int vout_videobuf_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct vout_data *vout = vb2q_to_vout(vq);
	int ret;

	vout->status = VOUT_STARTING;

	ret = request_threaded_irq(vout->irq, NULL, vout_handler, IRQF_ONESHOT,
			"imx-ipu-ovl", vout);
	if (ret)
		return ret;

	return 0;
}

static int vout_videobuf_stop_streaming(struct vb2_queue *vq)
{
	struct vout_data *vout = vb2q_to_vout(vq);
	unsigned long flags;

	spin_lock_irqsave(&vout->lock, flags);

	vout->status = VOUT_STOPPING;

	while (!list_empty(&vout->scale_list) || !list_empty(&vout->show_list)) {
		spin_unlock_irqrestore(&vout->lock, flags);
		schedule();
		spin_lock_irqsave(&vout->lock, flags);
	}

	ipu_idmac_disable_channel(vout->ipu_ch);
	ipu_dmfc_disable_channel(vout->dmfc);

	spin_unlock_irqrestore(&vout->lock, flags);

	free_irq(vout->irq, vout);

	return 0;
}

static int vout_videobuf_init(struct vb2_buffer *vb)
{
	return 0;
}

static struct vb2_ops vout_videobuf_ops = {
	.queue_setup		= vout_videobuf_setup,
	.buf_prepare		= vout_videobuf_prepare,
	.buf_queue		= vout_videobuf_queue,
	.buf_cleanup		= vout_videobuf_release,
	.buf_init		= vout_videobuf_init,
	.start_streaming	= vout_videobuf_start_streaming,
	.stop_streaming		= vout_videobuf_stop_streaming,
#if 0
	/* FIXME: do we need these? */
	.wait_prepare		= vout_videobuf_unlock,
	.wait_finish		= vout_videobuf_lock,
#endif
};

static int ipu_ovl_vidioc_reqbufs(struct file *file, void *priv,
			struct v4l2_requestbuffers *reqbuf)
{
	struct video_device *dev = video_devdata(file);
	struct vout_data *vout = video_get_drvdata(dev);
	int ret;

	ret = vb2_reqbufs(&vout->vidq, reqbuf);

	return ret;
}

static int ipu_ovl_vidioc_querybuf(struct file *file, void *fh, struct v4l2_buffer *buf)
{
	struct video_device *dev = video_devdata(file);
	struct vout_data *vout = video_get_drvdata(dev);

	return vb2_querybuf(&vout->vidq, buf);
}

static int ipu_ovl_vidioc_qbuf(struct file *file, void *fh, struct v4l2_buffer *buf)
{
	struct video_device *dev = video_devdata(file);
	struct vout_data *vout = video_get_drvdata(dev);

	return vb2_qbuf(&vout->vidq, buf);
}

static int ipu_ovl_vidioc_dqbuf(struct file *file, void *fh, struct v4l2_buffer *buf)
{
	struct video_device *dev = video_devdata(file);
	struct vout_data *vout = video_get_drvdata(dev);

	return vb2_dqbuf(&vout->vidq, buf, file->f_flags & O_NONBLOCK);
}

static int ipu_ovl_vidioc_streamon(struct file *file, void *fh, enum v4l2_buf_type i)
{
	struct video_device *dev = video_devdata(file);
	struct vout_data *vout = video_get_drvdata(dev);

	return vb2_streamon(&vout->vidq, i);
}

static int ipu_ovl_vidioc_streamoff(struct file *file, void *fh, enum v4l2_buf_type i)
{
	struct video_device *dev = video_devdata(file);
	struct vout_data *vout = video_get_drvdata(dev);

	return vb2_streamoff(&vout->vidq, i);
}

static int ipu_ovl_vidioc_enum_fmt_vid_out(struct file *file, void *fh,
		struct v4l2_fmtdesc *f)
{
	return ipu_enum_fmt(file, fh, f);
}

static int mxc_v4l2out_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct video_device *dev = video_devdata(file);
	struct vout_data *vout = video_get_drvdata(dev);

	return vb2_mmap(&vout->vidq, vma);
}

static int mxc_v4l2out_open(struct file *file)
{
	struct video_device *dev = video_devdata(file);
	struct vout_data *vout = video_get_drvdata(dev);
	struct vb2_queue *q = &vout->vidq;
	int i;

	if (vout->opened)
		return -EBUSY;

	vout->opened++;

	INIT_LIST_HEAD(&vout->idle_list);
	INIT_LIST_HEAD(&vout->scale_list);
	INIT_LIST_HEAD(&vout->show_list);

	ipu_ovl_get_base_resolution(vout);

	for (i = 0; i < NUMBUFS; i++) {
		struct vout_queue *q;

		q = kzalloc(sizeof (*q), GFP_KERNEL);
		if (!q)
			return -ENOMEM;
		q->size = vout->width_base * vout->height_base * 2;
		q->virt = dma_alloc_coherent(NULL, q->size, &q->phys,
					       GFP_DMA | GFP_KERNEL);
		q->vout = vout;
		BUG_ON(!q->virt);

		list_add_tail(&q->list, &vout->idle_list);
	}

	q->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	q->io_modes = VB2_MMAP | VB2_USERPTR;
	q->drv_priv = vout;
	q->ops = &vout_videobuf_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->buf_struct_size = sizeof(struct vb2_buffer);

	return vb2_queue_init(q);
}

static int mxc_v4l2out_close(struct file *file)
{
	struct video_device *dev = video_devdata(file);
	struct vout_data *vout = video_get_drvdata(dev);
	struct vout_queue *q, *tmp;

	vb2_queue_release(&vout->vidq);

	list_for_each_entry_safe(q, tmp, &vout->idle_list, list) {
		dma_free_coherent(NULL, q->size, q->virt, q->phys);
		kfree(q);
	}

	vout->opened--;

	return 0;
}

static const struct v4l2_ioctl_ops mxc_ioctl_ops = {
	.vidioc_querycap		= vidioc_querycap,
	.vidioc_cropcap			= vidioc_cropcap,

	.vidioc_enum_fmt_vid_out	= ipu_ovl_vidioc_enum_fmt_vid_out,
	.vidioc_g_fmt_vid_out		= ipu_ovl_vidioc_g_fmt_vid_out,
	.vidioc_s_fmt_vid_out		= ipu_ovl_vidioc_s_fmt_vid_out,
	.vidioc_try_fmt_vid_out		= ipu_ovl_vidioc_try_fmt_vid_out,

	.vidioc_enum_fmt_vid_overlay	= ipu_ovl_vidioc_enum_fmt_vid_out,
	.vidioc_g_fmt_vid_out_overlay	= ipu_ovl_vidioc_g_fmt_vid_overlay,
	.vidioc_s_fmt_vid_out_overlay	= ipu_ovl_vidioc_s_fmt_vid_overlay,
	.vidioc_try_fmt_vid_out_overlay	= ipu_ovl_vidioc_try_fmt_vid_overlay,

	.vidioc_s_crop			= ipu_ovl_vidioc_s_crop,
	.vidioc_g_crop			= ipu_ovl_vidioc_g_crop,

	.vidioc_reqbufs			= ipu_ovl_vidioc_reqbufs,
	.vidioc_querybuf		= ipu_ovl_vidioc_querybuf,
	.vidioc_qbuf			= ipu_ovl_vidioc_qbuf,
	.vidioc_dqbuf			= ipu_ovl_vidioc_dqbuf,
	.vidioc_streamon		= ipu_ovl_vidioc_streamon,
	.vidioc_streamoff		= ipu_ovl_vidioc_streamoff,
#ifdef CONFIG_VIDEO_V4L1_COMPAT
	.vidiocgmbuf			= vidiocgmbuf,
#endif
};

static struct v4l2_file_operations mxc_v4l2out_fops = {
	.owner		= THIS_MODULE,
	.open		= mxc_v4l2out_open,
	.release	= mxc_v4l2out_close,
	.ioctl		= video_ioctl2,
	.mmap		= mxc_v4l2out_mmap,
};

static u64 vout_dmamask = ~(u32)0;

static int mxc_v4l2out_probe(struct platform_device *pdev)
{
	struct ipu_client_platformdata *pdata = pdev->dev.platform_data;
	struct vout_data *vout;
	struct ipu_soc *ipu = dev_get_drvdata(pdev->dev.parent);
	int ret;

	if (!pdata)
		return -EINVAL;

	pdev->dev.dma_mask = &vout_dmamask;
	pdev->dev.coherent_dma_mask = 0xffffffff;

	vout = kzalloc(sizeof(struct vout_data), GFP_KERNEL);
	if (!vout)
		return -ENOMEM;

	vout->ipu = ipu;

	vout->video_dev = video_device_alloc();
	if (!vout->video_dev) {
		ret = -ENOMEM;
		goto failed_vdev_alloc;
	}

	vout->alloc_ctx = vb2_dma_contig_init_ctx(&pdev->dev);
	if (IS_ERR(vout->alloc_ctx)) {
		ret = PTR_ERR(vout->alloc_ctx);
		goto failed_vb2_alloc;
	}

	vout->ipu_ch = ipu_idmac_get(ipu, pdata->dma[0]);
	if (!vout->ipu_ch) {
		dev_err(&pdev->dev, "failed to get ipu channel\n");
		ret = -EINVAL;
		goto failed_idmac;
	}

	vout->irq = ipu_idmac_channel_irq(ipu, vout->ipu_ch, IPU_IRQ_EOF);

	vout->dmfc = ipu_dmfc_get(ipu, pdata->dma[0]);
	if (!vout->dmfc) {
		dev_err(&pdev->dev, "failed to get ipu dmfc channel\n");
		ret = -EINVAL;
		goto failed_dmfc;
	}
	vout->dp = ipu_dp_get(ipu, IPU_DP_FLOW_SYNC_FG);
	if (!vout->dp) {
		dev_err(&pdev->dev, "failed to get ipu dp channel\n");
		ret = -EINVAL;
		goto failed_dp;
	}

	vout->video_dev->minor = -1;

	ret = ipu_dmfc_alloc_bandwidth(vout->dmfc, 1280 * 720 * 70, 64);
	if (ret) {
		dev_err(&pdev->dev, "allocating dmfc bandwidth failed with %d\n", ret);
		goto failed_dmfc_bw;
	}

	strcpy(vout->video_dev->name, "vout");
	vout->video_dev->fops = &mxc_v4l2out_fops;
	vout->video_dev->ioctl_ops = &mxc_ioctl_ops;
	vout->video_dev->release = video_device_release;
	vout->video_dev->vfl_dir = VFL_DIR_TX;

	spin_lock_init(&vout->lock);
	vout->dev = &pdev->dev;

	ret = video_register_device(vout->video_dev, VFL_TYPE_GRABBER, -1);
	if (ret) {
		dev_err(&pdev->dev, "register failed with %d\n", ret);
		goto failed_register;
	}

	platform_set_drvdata(pdev, vout);
	video_set_drvdata(vout->video_dev, vout);

	return 0;

failed_register:
	ipu_dmfc_free_bandwidth(vout->dmfc);
failed_dmfc_bw:
	ipu_dp_put(vout->dp);
failed_dp:
	ipu_dmfc_put(vout->dmfc);
failed_dmfc:
	ipu_idmac_put(vout->ipu_ch);
failed_idmac:
	vb2_dma_contig_cleanup_ctx(vout->alloc_ctx);
failed_vb2_alloc:
	kfree(vout->video_dev);
failed_vdev_alloc:
	kfree(vout);
	return ret;

	return 0;
}

static int mxc_v4l2out_remove(struct platform_device *pdev)
{
	struct vout_data *vout = platform_get_drvdata(pdev);

	video_unregister_device(vout->video_dev);
	ipu_dmfc_free_bandwidth(vout->dmfc);
	ipu_dp_put(vout->dp);
	ipu_dmfc_put(vout->dmfc);
	ipu_idmac_put(vout->ipu_ch);
	vb2_dma_contig_cleanup_ctx(vout->alloc_ctx);

	kfree(vout);

	return 0;
}

static struct platform_driver mxc_v4l2out_driver = {
	.driver = {
		   .name = "imx-ipuv3-ovl",
	},
	.probe = mxc_v4l2out_probe,
	.remove = mxc_v4l2out_remove,
};

static int mxc_v4l2out_init(void)
{
	return platform_driver_register(&mxc_v4l2out_driver);
}

static void mxc_v4l2out_exit(void)
{
	platform_driver_unregister(&mxc_v4l2out_driver);
}

module_init(mxc_v4l2out_init);
module_exit(mxc_v4l2out_exit);

MODULE_AUTHOR("Sascha Hauer <s.hauer@pengutronix.de>");
MODULE_DESCRIPTION("V4L2-driver for MXC video output");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("video");
