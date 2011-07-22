/*
 * i.MX IPUv3 common v4l2 support
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
#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>

#include "imx-ipu.h"

static struct ipu_fmt ipu_fmt[] = {
	{
		.fourcc = V4L2_PIX_FMT_YUV420,
		.name = "YUV 4:2:0 planar, YCbCr",
		.bytes_per_pixel = 1,
	}, {
		.fourcc = V4L2_PIX_FMT_YVU420,
		.name = "YUV 4:2:0 planar, YCrCb",
		.bytes_per_pixel = 1,
	}, {
		.fourcc = V4L2_PIX_FMT_UYVY,
		.name = "4:2:2, packed, UYVY",
		.bytes_per_pixel = 2,
	}, {
		.fourcc = V4L2_PIX_FMT_YUYV,
		.name = "4:2:2, packed, YUYV",
		.bytes_per_pixel = 2,
	}, {
		.fourcc = V4L2_PIX_FMT_RGB32,
		.name = "RGB888",
		.bytes_per_pixel = 4,
	}, {
		.fourcc = V4L2_PIX_FMT_RGB24,
		.name = "RGB24",
		.bytes_per_pixel = 3,
	}, {
		.fourcc = V4L2_PIX_FMT_BGR24,
		.name = "BGR24",
		.bytes_per_pixel = 3,
	}, {
		.fourcc = V4L2_PIX_FMT_RGB565,
		.name = "RGB565",
		.bytes_per_pixel = 2,
	},
#if 0
	{
		.fourcc = V4L2_PIX_FMT_BGR32,
		.name = "RGB888",
		.bytes_per_pixel = 4,
	},
#endif
};

struct ipu_fmt *ipu_find_fmt(struct v4l2_format *f)
{
	struct ipu_fmt *fmt;
	int i;

	for (i = 0; i < ARRAY_SIZE(ipu_fmt); i++) {
                fmt = &ipu_fmt[i];
		if (fmt->fourcc == f->fmt.pix.pixelformat)
			return fmt;
	}

	return NULL;
}

int ipu_try_fmt(struct file *file, void *fh,
		struct v4l2_format *f)
{
	struct ipu_fmt *fmt;

	if (f->fmt.pix.width > 4096)
		f->fmt.pix.width = 4096;
	if (f->fmt.pix.width < 128)
		f->fmt.pix.width = 128;
	if (f->fmt.pix.height > 4096)
		f->fmt.pix.height = 4096;
	if (f->fmt.pix.height < 128)
		f->fmt.pix.height = 128;

	f->fmt.pix.width &= ~0x3;
	f->fmt.pix.height &= ~0x1;

	f->fmt.pix.field = V4L2_FIELD_ANY;

	fmt = ipu_find_fmt(f);
	if (!fmt)
		return -EINVAL;

	f->fmt.pix.field = V4L2_FIELD_ANY;
	f->fmt.pix.bytesperline = f->fmt.pix.width * fmt->bytes_per_pixel;
	if (fmt->fourcc == V4L2_PIX_FMT_YUV420 ||
	    fmt->fourcc == V4L2_PIX_FMT_YVU420)
		f->fmt.pix.sizeimage = f->fmt.pix.bytesperline * f->fmt.pix.height * 3 / 2;
	else
		f->fmt.pix.sizeimage = f->fmt.pix.bytesperline * f->fmt.pix.height;

	return 0;
}

int ipu_enum_fmt(struct file *file, void *fh,
		struct v4l2_fmtdesc *f)
{
	struct ipu_fmt *fmt;

	if (f->index >= ARRAY_SIZE(ipu_fmt))
		return -EINVAL;

	fmt = &ipu_fmt[f->index];

	strlcpy(f->description, fmt->name, sizeof(f->description));
	f->pixelformat = fmt->fourcc;

	return 0;
}

int ipu_s_fmt(struct file *file, void *fh,
		struct v4l2_format *f, struct v4l2_pix_format *pix)
{
	struct ipu_fmt *fmt;
	int ret;

	ret = ipu_try_fmt(file, fh, f);
	if (ret)
		return ret;

	fmt = ipu_find_fmt(f);
	if (!fmt)
		return -EINVAL;

	pix->width = f->fmt.pix.width;
	pix->height = f->fmt.pix.height;
	pix->pixelformat = f->fmt.pix.pixelformat;
	pix->bytesperline = f->fmt.pix.bytesperline;
	pix->sizeimage = f->fmt.pix.sizeimage;

	return 0;
}

int ipu_g_fmt(struct v4l2_format *f, struct v4l2_pix_format *pix)
{
	f->fmt.pix.field = V4L2_FIELD_ANY;
	f->fmt.pix.pixelformat = pix->pixelformat;
	f->fmt.pix.bytesperline = pix->width * pix->bytesperline;
	f->fmt.pix.width = pix->width;
	f->fmt.pix.height = pix->height;
	f->fmt.pix.sizeimage = pix->sizeimage;

	return 0;
}
