/*
 * i.MX drm driver - sii902x display implementation
 *
 * Copyright (C) 2012 Ahmed Ammar, Genesi
 *
 * Based on parallel-display.c
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

#define DEBUG 1

#include <linux/module.h>
#include <drm/drmP.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_crtc_helper.h>
#include <linux/videodev2.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>

#include "imx-drm.h"
#include "sii902x-display.h"

#define con_to_siihdmi(x) container_of(x, struct siihdmi_display, connector)
#define enc_to_siihdmi(x) container_of(x, struct siihdmi_display, encoder)

/* logging helpers */
#define PR_PREFIX		"siihdmi: "
#define CONTINUE(fmt, ...)	pr_cont(fmt, ## __VA_ARGS__)
#define DBG(fmt, ...)		pr_debug(PR_PREFIX fmt, ## __VA_ARGS__)
#define ERR(fmt, ...)		pr_err(PR_PREFIX fmt, ## __VA_ARGS__)
#define WARNING(fmt, ...)	pr_warning(PR_PREFIX fmt, ## __VA_ARGS__)
#define INFO(fmt, ...)		pr_info(PR_PREFIX fmt, ## __VA_ARGS__)

static unsigned int bus_timeout = 50;

struct siihdmi_display {
	struct drm_connector connector;
	struct imx_drm_connector *imx_drm_connector;
	struct drm_encoder encoder;
	struct imx_drm_encoder *imx_drm_encoder;
	struct device *dev;
	void *edid;
	int edid_len;
	u32 interface_pix_fmt;
	int mode_valid;
	struct drm_display_mode mode;
	unsigned int gpio_reset, gpio_enable, gpio_vga, gpio_irq;
	struct i2c_client *client;
};

static int siihdmi_initialize(struct siihdmi_display *siihdmi);

static enum drm_connector_status siihdmi_connector_detect(
		struct drm_connector *connector, bool force)
{
	return connector_status_connected;
}

static void siihdmi_connector_destroy(struct drm_connector *connector)
{
	/* do not free here */
}


static inline int siihdmi_power_up(struct siihdmi_display *siihdmi)
{
	int ret;

	ret = i2c_smbus_write_byte_data(siihdmi->client,
					SIIHDMI_TPI_REG_PWR_STATE,
					SIIHDMI_POWER_STATE_D0);
	if (ret < 0)
		ERR("unable to power up transmitter\n");
	else
		WARNING("powered to D0\n");

	return ret;
}

static inline int siihdmi_power_down(struct siihdmi_display *siihdmi)
{
	int ret;
	u8 ctrl;

	ctrl = SIIHDMI_SYS_CTRL_TMDS_OUTPUT_POWER_DOWN;
	/* this seems redundant since D2 will wipe the sink state, but just
	 * in case we actually want to keep our display if D2 doesn't work
	 */
	ctrl |= SIIHDMI_SYS_CTRL_OUTPUT_MODE_SELECT_HDMI;

	ret = i2c_smbus_write_byte_data(siihdmi->client,
					SIIHDMI_TPI_REG_SYS_CTRL, ctrl);
	if (ret < 0) {
		ERR("unable to power down transmitter\n");
		return ret;
	}
#if 0
	ret = i2c_smbus_write_byte_data(siihdmi->client,
					SIIHDMI_TPI_REG_PWR_STATE,
					SIIHDMI_POWER_STATE_D2);
	if (ret < 0) {
		ERR("unable to set transmitter into D2\n");
		return ret;
	} else
		WARNING("powered to D2\n");
#endif
	return 0;
}

static int siihdmi_connector_get_modes(struct drm_connector *connector)
{
	struct siihdmi_display *siihdmi = con_to_siihdmi(connector);
	struct i2c_client *client = siihdmi->client;
	struct i2c_adapter *adap = client->adapter;
	struct edid *edid;
	u8 ctrl;
	int ret;
	unsigned long start;

	/* step 1: (potentially) disable HDCP */

	/* step 2: request the DDC bus */
	ctrl = i2c_smbus_read_byte_data(siihdmi->client, SIIHDMI_TPI_REG_SYS_CTRL);
	ret = i2c_smbus_write_byte_data(siihdmi->client,
					SIIHDMI_TPI_REG_SYS_CTRL,
					ctrl | SIIHDMI_SYS_CTRL_DDC_BUS_REQUEST);
	if (ret < 0) {
		DBG("unable to request DDC bus\n");
		return ret;
	}

	/* step 3: poll for bus grant */
	start = jiffies;
	do {
		ctrl = i2c_smbus_read_byte_data(siihdmi->client,
						SIIHDMI_TPI_REG_SYS_CTRL);
	} while ((~ctrl & SIIHDMI_SYS_CTRL_DDC_BUS_GRANTED) &&
		 !time_after(jiffies, start + msecs_to_jiffies(bus_timeout)));

	if (~ctrl & SIIHDMI_SYS_CTRL_DDC_BUS_GRANTED)
		goto relinquish;

	/* step 4: take ownership of the DDC bus */
	ret = i2c_smbus_write_byte_data(siihdmi->client,
					SIIHDMI_TPI_REG_SYS_CTRL,
					SIIHDMI_SYS_CTRL_DDC_BUS_REQUEST |
					SIIHDMI_SYS_CTRL_DDC_BUS_OWNER_HOST);
	if (ret < 0) {
		DBG("unable to take ownership of the DDC bus\n");
		goto relinquish;
	}

	edid = drm_get_edid(connector, adap);
	if (edid) {
		drm_mode_connector_update_edid_property(connector, edid);
		ret = drm_add_edid_modes(connector, edid);
		kfree(edid);
	}

relinquish:
	/* step 6: relinquish ownership of the DDC bus while also setting
	 * the correct sink type (as per manual)
	 */
	start = jiffies;
	do {
		i2c_smbus_write_byte_data(siihdmi->client,
					  SIIHDMI_TPI_REG_SYS_CTRL, 0x00);
		ctrl = i2c_smbus_read_byte_data(siihdmi->client,
						SIIHDMI_TPI_REG_SYS_CTRL);
	} while ((ctrl & SIIHDMI_SYS_CTRL_DDC_BUS_GRANTED) &&
		 !time_after(jiffies, start + msecs_to_jiffies(bus_timeout)));

	/* now, force the operational mode (HDMI or DVI) based on sink
	 * type and make it stick with a power up request (pg 27)
	 */
	i2c_smbus_write_byte_data(siihdmi->client, SIIHDMI_TPI_REG_SYS_CTRL,
		    SIIHDMI_SYS_CTRL_OUTPUT_MODE_SELECT_HDMI);

	ret = siihdmi_power_up(siihdmi);

	/* step 7: (potentially) enable HDCP */

	return 0;
}

static int siihdmi_setup_display(struct siihdmi_display *siihdmi)
{
	int i, ret;
	u8 isr;

	isr = i2c_smbus_read_byte_data(siihdmi->client, SIIHDMI_TPI_REG_ISR);
	DBG("setup: display %s, receiver sense %s\n",
	      (isr & SIIHDMI_ISR_DISPLAY_ATTACHED) ? "attached" : "detached",
	      (isr & SIIHDMI_ISR_RECEIVER_SENSE) ? "active" : "inactive");

	if (~isr & SIIHDMI_ISR_DISPLAY_ATTACHED)
		return siihdmi_power_down(siihdmi);

	return 0;
}

static irqreturn_t sii902x_detect_handler(int irq, void *data)
{
	struct siihdmi_display *siihdmi = data;
	struct i2c_client *client = siihdmi->client;

	char *connected[]    = { "DISPLAY_CONNECTED=1", NULL };
	char *disconnected[] = { "DISPLAY_CONNECTED=0", NULL };
	char *power_on[]     = { "DISPLAY_POWERED_ON=1", NULL };
	char *power_off[]    = { "DISPLAY_POWERED_ON=0", NULL };

	int dat;
	u8 isr;

	isr = i2c_smbus_read_byte_data(siihdmi->client, SIIHDMI_TPI_REG_ISR);
	if (~isr & SIIHDMI_ISR_HOT_PLUG_EVENT)
		goto complete;

	DBG("hotplug: display %s, receiver sense %s\n",
	      (isr & SIIHDMI_ISR_DISPLAY_ATTACHED) ? "attached" : "detached",
	      (isr & SIIHDMI_ISR_RECEIVER_SENSE) ? "active" : "inactive");

	if (isr & SIIHDMI_ISR_HOT_PLUG_EVENT) {
		if (isr & SIIHDMI_ISR_DISPLAY_ATTACHED)
			kobject_uevent_env(&siihdmi->client->dev.kobj, KOBJ_CHANGE,
					   connected);
		else
			kobject_uevent_env(&siihdmi->client->dev.kobj, KOBJ_CHANGE,
					   disconnected);
	}

	if (isr & SIIHDMI_ISR_RECEIVER_SENSE_EVENT) {
		if (isr & SIIHDMI_ISR_RECEIVER_SENSE)
			kobject_uevent_env(&siihdmi->client->dev.kobj, KOBJ_CHANGE,
					   power_on);
		else
			kobject_uevent_env(&siihdmi->client->dev.kobj, KOBJ_CHANGE,
					   power_off);
	}

	if (isr & (SIIHDMI_ISR_DISPLAY_ATTACHED)) {
		siihdmi_initialize(siihdmi);
	} else {
		siihdmi_power_down(siihdmi);
	}

complete:
	/* clear the interrupt */
	i2c_smbus_write_byte_data(siihdmi->client, SIIHDMI_TPI_REG_ISR, isr);

	return IRQ_HANDLED;
}

static int siihdmi_connector_mode_valid(struct drm_connector *connector,
			  struct drm_display_mode *mode)
{
	return MODE_OK;
}

static struct drm_encoder *siihdmi_connector_best_encoder(
		struct drm_connector *connector)
{
	struct siihdmi_display *siihdmi = con_to_siihdmi(connector);

	return &siihdmi->encoder;
}

static bool siihdmi_encoder_mode_fixup(struct drm_encoder *encoder,
			   const struct drm_display_mode *mode,
			   struct drm_display_mode *adjusted_mode)
{
	return true;
}

static void siihdmi_encoder_dpms(struct drm_encoder *encoder, int mode)
{
	printk("%s mode:%u\n", __func__, mode);
	struct siihdmi_display *siihdmi = enc_to_siihdmi(encoder);

	if (mode)
		siihdmi_power_down(siihdmi);
	else
		siihdmi_power_up(siihdmi);
}

static void siihdmi_encoder_prepare(struct drm_encoder *encoder)
{
	printk("%s\n", __func__);
	struct siihdmi_display *siihdmi = enc_to_siihdmi(encoder);

	imx_drm_crtc_panel_format(encoder->crtc, DRM_MODE_ENCODER_TMDS,
			siihdmi->interface_pix_fmt);
}

static void siihdmi_encoder_commit(struct drm_encoder *encoder)
{
	printk("%s\n", __func__);
	struct siihdmi_display *siihdmi = enc_to_siihdmi(encoder);

	siihdmi_power_up(siihdmi);
}

static int siihdmi_set_avi_info_frame(struct siihdmi_display *siihdmi)
{
	int ret;

	struct avi_info_frame avi = {
		.header = {
			.type    = INFO_FRAME_TYPE_AUXILIARY_VIDEO_INFO,
			.version = CEA861_AVI_INFO_FRAME_VERSION,
			.length  = sizeof(avi) - sizeof(avi.header),
		},

		.active_format_info_valid  = true,
		.active_format_description = ACTIVE_FORMAT_DESCRIPTION_UNSCALED,

		.picture_aspect_ratio      = PICTURE_ASPECT_RATIO_UNSCALED,

		.video_format              = 4,
	};

//	BUG_ON(tx->sink.type != SINK_TYPE_HDMI);

	DBG("AVI InfoFrame sending Video Format %d\n", 4);

#if 0
	switch (tx->sink.scanning) {
	case SCANNING_UNDERSCANNED:
		avi.scan_information = SCAN_INFORMATION_UNDERSCANNED;
		break;
	case SCANNING_OVERSCANNED:
		avi.scan_information = SCAN_INFORMATION_OVERSCANNED;
		break;
	default:
		avi.scan_information = SCAN_INFORMATION_UNKNOWN;
		break;
	}
#endif
	avi.scan_information = SCAN_INFORMATION_UNKNOWN;

	cea861_checksum_hdmi_info_frame((u8 *) &avi);

	BUILD_BUG_ON(sizeof(avi) - SIIHDMI_AVI_INFO_FRAME_OFFSET != SIIHDMI_TPI_REG_AVI_INFO_FRAME_LENGTH);
	ret = i2c_smbus_write_i2c_block_data(siihdmi->client,
					     SIIHDMI_TPI_REG_AVI_INFO_FRAME_BASE,
					     sizeof(avi) - SIIHDMI_AVI_INFO_FRAME_OFFSET,
					     ((u8 *) &avi) + SIIHDMI_AVI_INFO_FRAME_OFFSET);
	if (ret < 0)
		DBG("unable to write avi info frame\n");

	return ret;
}

static int siihdmi_set_audio_info_frame(struct siihdmi_display *siihdmi)
{
	int ret;
	struct siihdmi_audio_info_frame packet = {
		.header = {
			.info_frame = SIIHDMI_INFO_FRAME_AUDIO,
			.repeat     = false,
			.enable     = false,
		},

		.info_frame = {
			.header = {
				.type    = INFO_FRAME_TYPE_AUDIO,
				.version = CEA861_AUDIO_INFO_FRAME_VERSION,
				.length  = sizeof(packet.info_frame) - sizeof(packet.info_frame.header),
			},

			.channel_count         = CHANNEL_COUNT_REFER_STREAM_HEADER,
			.channel_allocation    = CHANNEL_ALLOCATION_STEREO,

			.format_code_extension = CODING_TYPE_REFER_STREAM_HEADER,

			/* required to Refer to Stream Header by CEA861-D */
			.coding_type           = CODING_TYPE_REFER_STREAM_HEADER,

			.sample_size           = SAMPLE_SIZE_REFER_STREAM_HEADER,
			.sample_frequency      = FREQUENCY_REFER_STREAM_HEADER,
		},
	};

	//BUG_ON(tx->sink.type != SINK_TYPE_HDMI);

	cea861_checksum_hdmi_info_frame((u8 *) &packet.info_frame);

	BUILD_BUG_ON(sizeof(packet) != SIIHDMI_TPI_REG_AUDIO_INFO_FRAME_LENGTH);
	ret = i2c_smbus_write_i2c_block_data(siihdmi->client,
					     SIIHDMI_TPI_REG_MISC_INFO_FRAME_BASE,
					     sizeof(packet),
					     (u8 *) &packet);
	if (ret < 0)
		DBG("unable to write audio info frame\n");

	return ret;
}

static int siihdmi_set_spd_info_frame(struct siihdmi_display *siihdmi)
{
	int ret;
	struct siihdmi_spd_info_frame packet = {
		.header = {
			.info_frame = SIIHDMI_INFO_FRAME_SPD_ACP,
			.repeat     = false,
			.enable     = true,
		},

		.info_frame = {
			.header = {
				.type    = INFO_FRAME_TYPE_SOURCE_PRODUCT_DESCRIPTION,
				.version = CEA861_SPD_INFO_FRAME_VERSION,
				.length  = sizeof(packet.info_frame) - sizeof(packet.info_frame.header),
			},

			.source_device_info = SPD_SOURCE_PC_GENERAL,
		},
	};

	//BUG_ON(tx->sink.type != SINK_TYPE_HDMI);

	/*strncpy(packet.info_frame.vendor, siihdmi->platform->vendor,
		sizeof(packet.info_frame.vendor));

	strncpy(packet.info_frame.description, tx->platform->description,
		sizeof(packet.info_frame.description));*/

	cea861_checksum_hdmi_info_frame((u8 *) &packet.info_frame);

	BUILD_BUG_ON(sizeof(packet) != SIIHDMI_TPI_REG_MISC_INFO_FRAME_LENGTH);
	ret = i2c_smbus_write_i2c_block_data(siihdmi->client,
					     SIIHDMI_TPI_REG_MISC_INFO_FRAME_BASE,
					     sizeof(packet),
					     (u8 *) &packet);
	if (ret < 0)
		DBG("unable to write SPD info frame\n");

	return ret;
}

static void siihdmi_set_vmode_registers(struct siihdmi_display *siihdmi,
					const struct drm_display_mode *mode)
{
	enum basic_video_mode_fields {
		PIXEL_CLOCK,
		REFRESH_RATE,
		X_RESOLUTION,
		Y_RESOLUTION,
		FIELDS,
	};

	u16 vmode[FIELDS];
	u32 pixclk, htotal, vtotal, refresh;
	u8 format;
	int ret;

	BUILD_BUG_ON(sizeof(vmode) != 8);

	pixclk = mode->clock;
	htotal = mode->htotal;
	vtotal = mode->vtotal;

	/* explicitly use 64-bit division to avoid overflow truncation */
	refresh = (u32) div_u64(pixclk * 1000ull, htotal * vtotal);

	/* basic video mode data */
	vmode[PIXEL_CLOCK]  = (u16) (pixclk / 10);
	/*
	  Silicon Image example code implies refresh to be 6000 for 60Hz?
	  This may work simply because we only test it on little-endian :(
	*/
	vmode[REFRESH_RATE] = (u16) refresh;
	vmode[X_RESOLUTION] = (u16) htotal;
	vmode[Y_RESOLUTION] = (u16) vtotal;

	ret = i2c_smbus_write_i2c_block_data(siihdmi->client,
					     SIIHDMI_TPI_REG_VIDEO_MODE_DATA_BASE,
					     sizeof(vmode),
					     (u8 *) vmode);
	if (ret < 0)
		DBG("unable to write video mode data\n");

	/* input format */
	format = SIIHDMI_INPUT_COLOR_SPACE_RGB
	       | SIIHDMI_INPUT_VIDEO_RANGE_EXPANSION_AUTO
	       | SIIHDMI_INPUT_COLOR_DEPTH_8BIT;

	ret = i2c_smbus_write_byte_data(siihdmi->client,
					SIIHDMI_TPI_REG_AVI_INPUT_FORMAT,
					format);
	if (ret < 0)
		DBG("unable to set input format\n");

	/* output format */
	format = SIIHDMI_OUTPUT_VIDEO_RANGE_COMPRESSION_AUTO
	       | SIIHDMI_OUTPUT_COLOR_STANDARD_BT601
	       | SIIHDMI_OUTPUT_COLOR_DEPTH_8BIT;

	format |= SIIHDMI_OUTPUT_FORMAT_HDMI_RGB;

	ret = i2c_smbus_write_byte_data(siihdmi->client,
					SIIHDMI_TPI_REG_AVI_OUTPUT_FORMAT,
					format);
	if (ret < 0)
		DBG("unable to set output format\n");
}

static void siihdmi_encoder_mode_set(struct drm_encoder *encoder,
			 struct drm_display_mode *mode,
			 struct drm_display_mode *adjusted_mode)
{
	struct siihdmi_display *siihdmi = enc_to_siihdmi(encoder);
	struct i2c_client *client = siihdmi->client;
	u16 data[4];
	u32 refresh;
	u8 *tmp;
	int i;

	dev_dbg(&client->dev, "%s: %dx%d, pixclk %d\n", __func__,
			mode->hdisplay, mode->vdisplay,
			mode->clock * 1000);

	u8 ctrl;
	int ret;

	ctrl = i2c_smbus_read_byte_data(siihdmi->client, SIIHDMI_TPI_REG_SYS_CTRL);

	/* make sure we keep writing the sink type */
	ctrl |= SIIHDMI_SYS_CTRL_OUTPUT_MODE_SELECT_HDMI;

	/* step 0: (potentially) disable HDCP */

	/* step 1: (optionally) blank the display */
	/*
	 * Note that if we set the AV Mute, switching to DVI could result in a
	 * permanently muted display until a hardware reset.  Thus only do this
	 * if the sink is a HDMI connection
	 */
	/*if (siihdmi->sink.type == SINK_TYPE_HDMI) {
		if (useavmute) {
			ctrl |= SIIHDMI_SYS_CTRL_AV_MUTE_HDMI;
				ret = i2c_smbus_write_byte_data(siihdmi->client,
						SIIHDMI_TPI_REG_SYS_CTRL,
						ctrl);
			if (ret < 0)
				DBG("unable to AV Mute!\n");
		}
		msleep(SIIHDMI_CTRL_INFO_FRAME_DRAIN_TIME);
	}*/

	/* step 2: prepare for resolution change */
	ctrl |= SIIHDMI_SYS_CTRL_TMDS_OUTPUT_POWER_DOWN;
	ret = i2c_smbus_write_byte_data(siihdmi->client,
					SIIHDMI_TPI_REG_SYS_CTRL,
					ctrl);
	if (ret < 0)
		DBG("unable to prepare for resolution change\n");

	/*
	 * step 3: change video resolution
	 * and wait for it to stabilize (50-500ms);
	 */
	msleep(SIIHDMI_RESOLUTION_STABILIZE_TIME);

	/* step 4: set the vmode registers */
	siihdmi_set_vmode_registers(siihdmi, mode);

#if 0
	/*
	 * step 5:
	 *      [DVI]  clear AVI InfoFrame
	 *      [HDMI] set AVI InfoFrame
	 */
	if (siihdmi->sink.type == SINK_TYPE_HDMI) {
		int vic = siihdmi_find_vic_from_modedb(mode);
		siihdmi_set_avi_info_frame(siihdmi, vic);
	} else {
		siihdmi_clear_avi_info_frame(siihdmi);
	}

	/* step 6: [HDMI] set new audio information */
	if (siihdmi->sink.type == SINK_TYPE_HDMI) {
		if (siihdmi->audio.available)
			siihdmi_configure_audio(siihdmi);
		siihdmi_set_spd_info_frame(siihdmi);
	}

	/* step 7: enable display */
	ctrl &= ~SIIHDMI_SYS_CTRL_TMDS_OUTPUT_POWER_DOWN;
	ret = i2c_smbus_write_byte_data(siihdmi->client,
					SIIHDMI_TPI_REG_SYS_CTRL,
					ctrl);
	if (ret < 0)
		DBG("unable to enable the display\n");

	/* step 8: (optionally) un-blank the display */
//	if (siihdmi->sink.type == SINK_TYPE_HDMI && useavmute) {
		ctrl &= ~SIIHDMI_SYS_CTRL_AV_MUTE_HDMI;
		ret = i2c_smbus_write_byte_data(siihdmi->client,
					SIIHDMI_TPI_REG_SYS_CTRL,
					ctrl);
		if (ret < 0)
			DBG("unable to unmute the display\n");
//	}

	/* step 9: (potentially) enable HDCP */
#endif
	/* set video information */
	siihdmi_set_avi_info_frame(siihdmi);

	/* set audio information */
	siihdmi_set_spd_info_frame(siihdmi);
	

	ctrl = i2c_smbus_read_byte_data(siihdmi->client, SIIHDMI_TPI_REG_SYS_CTRL);
	/* step 7: enable display */
	ctrl &= ~SIIHDMI_SYS_CTRL_TMDS_OUTPUT_POWER_DOWN;
	ret = i2c_smbus_write_byte_data(siihdmi->client,
					SIIHDMI_TPI_REG_SYS_CTRL,
					ctrl);
	if (ret < 0)
		DBG("unable to enable the display\n");

	/* un-blank the display */
//	if (siihdmi->sink.type == SINK_TYPE_HDMI && useavmute) {
		ctrl &= ~SIIHDMI_SYS_CTRL_AV_MUTE_HDMI;
		ret = i2c_smbus_write_byte_data(siihdmi->client,
					SIIHDMI_TPI_REG_SYS_CTRL,
					ctrl);
		if (ret < 0)
			DBG("unable to unmute the display\n");
//	}
}

static void siihdmi_encoder_disable(struct drm_encoder *encoder)
{
}

static void siihdmi_encoder_destroy(struct drm_encoder *encoder)
{
	/* do not free here */
}

static struct drm_connector_funcs siihdmi_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = siihdmi_connector_detect,
	.destroy = siihdmi_connector_destroy,
};

static struct drm_connector_helper_funcs siihdmi_connector_helper_funcs = {
	.get_modes = siihdmi_connector_get_modes,
	.best_encoder = siihdmi_connector_best_encoder,
	.mode_valid = siihdmi_connector_mode_valid,
};

static struct drm_encoder_funcs siihdmi_encoder_funcs = {
	.destroy = siihdmi_encoder_destroy,
};

static struct drm_encoder_helper_funcs siihdmi_encoder_helper_funcs = {
	.dpms = siihdmi_encoder_dpms,
	.mode_fixup = siihdmi_encoder_mode_fixup,
	.prepare = siihdmi_encoder_prepare,
	.commit = siihdmi_encoder_commit,
	.mode_set = siihdmi_encoder_mode_set,
	.disable = siihdmi_encoder_disable,
};

static int siihdmi_register(struct siihdmi_display *siihdmi)
{
	int ret;

	drm_mode_connector_attach_encoder(&siihdmi->connector, &siihdmi->encoder);

	siihdmi->connector.funcs = &siihdmi_connector_funcs;
	siihdmi->encoder.funcs = &siihdmi_encoder_funcs;

	siihdmi->encoder.encoder_type = DRM_MODE_ENCODER_TMDS;
	siihdmi->connector.connector_type = DRM_MODE_CONNECTOR_HDMIA;

	drm_encoder_helper_add(&siihdmi->encoder, &siihdmi_encoder_helper_funcs);
	ret = imx_drm_add_encoder(&siihdmi->encoder, &siihdmi->imx_drm_encoder,
			THIS_MODULE);
	if (ret) {
		dev_err(siihdmi->dev, "adding encoder failed with %d\n", ret);
		return ret;
	}

	drm_connector_helper_add(&siihdmi->connector,
			&siihdmi_connector_helper_funcs);

	ret = imx_drm_add_connector(&siihdmi->connector,
			&siihdmi->imx_drm_connector, THIS_MODULE);
	if (ret) {
		imx_drm_remove_encoder(siihdmi->imx_drm_encoder);
		dev_err(siihdmi->dev, "adding connector failed with %d\n", ret);
		return ret;
	}

	siihdmi->connector.encoder = &siihdmi->encoder;

	return 0;
}

static int siihdmi_detect_revision(struct siihdmi_display *siihdmi)
{
	u8 data;
	unsigned long start;

	start = jiffies;
	do {
		data = i2c_smbus_read_byte_data(siihdmi->client,
						SIIHDMI_TPI_REG_DEVICE_ID);
	} while (data != SIIHDMI_DEVICE_ID_902x &&
		 !time_after(jiffies, start + msecs_to_jiffies(bus_timeout)));

	if (data != SIIHDMI_DEVICE_ID_902x)
		return -ENODEV;

	INFO("Device ID: %#02x", data);

	data = i2c_smbus_read_byte_data(siihdmi->client,
					SIIHDMI_TPI_REG_DEVICE_REVISION);
	if (data)
		CONTINUE(" (rev %01u.%01u)",
			 (data >> 4) & 0xf, (data >> 0) & 0xf);

	data = i2c_smbus_read_byte_data(siihdmi->client,
					SIIHDMI_TPI_REG_TPI_REVISION);
	CONTINUE(" (%s",
		 (data & SIIHDMI_VERSION_FLAG_VIRTUAL) ? "Virtual " : "");
	data &= ~SIIHDMI_VERSION_FLAG_VIRTUAL;
	data = data ? data : SIIHDMI_BASE_TPI_REVISION;
	CONTINUE("TPI revision %01u.%01u)",
		 (data >> 4) & 0xf, (data >> 0) & 0xf);

	data = i2c_smbus_read_byte_data(siihdmi->client,
					SIIHDMI_TPI_REG_HDCP_REVISION);
	if (data)
		CONTINUE(" (HDCP version %01u.%01u)",
			 (data >> 4) & 0xf, (data >> 0) & 0xf);

	CONTINUE("\n");

	return 0;
}

static int siihdmi_reset(struct siihdmi_display *siihdmi)
{
	struct device_node *np = siihdmi->client->dev.of_node;

	siihdmi->gpio_reset = of_get_named_gpio(np, "gpios", 0);
	if (siihdmi->gpio_reset) {
		if (gpio_request_one(siihdmi->gpio_reset, GPIOF_DIR_OUT, "SII902x reset")) {
			dev_err(&siihdmi->client->dev, "SII902x: could not request reset GPIO\n");
		} else {
			gpio_set_value(siihdmi->gpio_reset, 1);
			msleep(100);		/* SII9022 Treset >= 100us */
			gpio_set_value(siihdmi->gpio_reset, 0);
			msleep(2000);
		}
	} else {
		dev_info(&siihdmi->client->dev, "SII902x: reset GPIO not set\n");
	}

	/* Reset */
	siihdmi->gpio_enable = of_get_named_gpio(np, "gpios", 1);
	gpio_request_one(siihdmi->gpio_enable, GPIOF_DIR_OUT, "SII902x enable");
	gpio_set_value(siihdmi->gpio_enable, 0);

	siihdmi->gpio_vga = of_get_named_gpio(np, "gpios", 2);
	gpio_request_one(siihdmi->gpio_vga, GPIOF_DIR_OUT, "VGA enable");
	gpio_set_value(siihdmi->gpio_vga, 0);
/*
	siihdmi->gpio_irq = of_get_named_gpio(np, "gpios", 3);
	gpio_request_one(siihdmi->gpio_irq, GPIOF_DIR_IN, "SII902x irq");
*/
}

static int siihdmi_initialize(struct siihdmi_display *siihdmi)
{
	int ret;

	/* step 1: reset and initialise */
	siihdmi_reset(siihdmi);

	ret = i2c_smbus_write_byte_data(siihdmi->client, SIIHDMI_TPI_REG_RQB, 0x00);
	if (ret < 0) {
		WARNING("unable to initialise device to TPI mode\n");
		return ret;
	}

	/* step 2: detect revision */
	ret = siihdmi_detect_revision(siihdmi);
	if (ret < 0) {
		DBG("unable to detect device revision\n");
		return ret;
	}

	/* step 3: power up transmitter */
	ret = siihdmi_power_up(siihdmi);
	if (ret < 0)
		return ret;

	/* step 4: configure input bus and pixel repetition */

	/* step 5: select YC input mode */

	/* step 6: configure sync methods */

	/* step 7: configure explicit sync DE generation */

	/* step 8: configure embedded sync extraction */

	/* step 9: setup interrupt service
	 *
	 * (SII9022 programmer's reference p42:
	 *              Tplug_dly min. 400 typ. 480 max. 600ms)
	 * do we need to wait here, or do we know this has already
	 * happened in hardware?
	 */
	ret = i2c_smbus_write_byte_data(siihdmi->client,
					SIIHDMI_TPI_REG_IER,
					SIIHDMI_IER_HOT_PLUG_EVENT |
					SIIHDMI_IER_RECEIVER_SENSE_EVENT);
	if (ret < 0)
		WARNING("unable to setup interrupt request\n");

	return ret;
}

static int __devinit siihdmi_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device_node *np = client->dev.of_node;
	const u8 *edidp;
	struct siihdmi_display *siihdmi;
	int dat, ret;
	const char *fmt;
	struct pinctrl *pinctrl;

	siihdmi = devm_kzalloc(&client->dev, sizeof(*siihdmi), GFP_KERNEL);
	if (!siihdmi)
		return -ENOMEM;

	siihdmi->client = client;
	siihdmi->dev = &client->dev;
	i2c_set_clientdata(client, siihdmi);

	pinctrl = devm_pinctrl_get_select_default(&client->dev);
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		dev_warn(&client->dev, "pinctrl_get_select_default failed with %d",
				ret);
		return ret;
	}

	ret = of_property_read_string(np, "interface-pix-fmt", &fmt);
	if (!ret) {
		if (!strcmp(fmt, "rgb24"))
			siihdmi->interface_pix_fmt = V4L2_PIX_FMT_RGB24;
		else if (!strcmp(fmt, "rgb565"))
			siihdmi->interface_pix_fmt = V4L2_PIX_FMT_RGB565;
	}

	if (client->irq) {
		ret = request_threaded_irq(client->irq, NULL, sii902x_detect_handler,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"SII902x_det", siihdmi);
	}

	ret = siihdmi_initialize(siihdmi);
	if (ret)
	    return ret;

	ret = siihdmi_setup_display(siihdmi);
	if (ret)
	    return ret;

	dev_info(&client->dev, "initialized\n");

	ret = siihdmi_register(siihdmi);
	if (ret)
		return ret;

	ret = imx_drm_encoder_add_possible_crtcs(siihdmi->imx_drm_encoder, np);

	return 0;
}

static int __devexit siihdmi_remove(struct i2c_client *client)
{
	struct siihdmi_display *siihdmi = i2c_get_clientdata(client);
	struct drm_connector *connector = &siihdmi->connector;
	struct drm_encoder *encoder = &siihdmi->encoder;

	drm_mode_connector_detach_encoder(connector, encoder);

	imx_drm_remove_connector(siihdmi->imx_drm_connector);
	imx_drm_remove_encoder(siihdmi->imx_drm_encoder);

	return 0;
}

static struct i2c_device_id siihdmi_ids[] = {
	{ "sii9022", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sii902x_ids);

static const struct of_device_id siihdmi_dt_ids[] = {
	{ .compatible = "sii,sii9022", },
	{ .compatible = "sii,sii9022a", },
	{ /* sentinel */ }
};

static struct i2c_driver siihdmi_driver = {
	.probe		= siihdmi_probe,
	.remove		= __devexit_p(siihdmi_remove),
	.driver		= {
		.of_match_table = siihdmi_dt_ids,
		.name	= "sii902x",
		.owner	= THIS_MODULE,
	},
	.id_table = siihdmi_ids,
};

static int __init siihdmi_init(void)
{
	return i2c_add_driver(&siihdmi_driver);
}

static void __exit siihdmi_exit(void)
{
	i2c_del_driver(&siihdmi_driver);
}

MODULE_DESCRIPTION("i.MX sii902x display driver");
MODULE_AUTHOR("Ahmed Ammar, Genesi");
MODULE_LICENSE("GPL");

module_init(siihdmi_init);
module_exit(siihdmi_exit);
