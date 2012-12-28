/*
 * Copyright (C) 2011 Genesi USA, Inc.
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial
 * portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE COPYRIGHT OWNER(S) AND/OR ITS SUPPLIERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */
#define DEBUG 1

#include <linux/module.h>
#include <drm/drmP.h>
#include <drm/drmP.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_crtc_helper.h>
#include <linux/videodev2.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>

#include "imx-drm.h"
#include "siihdmi.h"

#define con_to_siihdmi(x) container_of(x, struct siihdmi_tx, connector)
#define enc_to_siihdmi(x) container_of(x, struct siihdmi_tx, encoder)

#define SIIHDMI_SUPPLY_NUM	3
static struct regulator_bulk_data siihdmi_supplies[SIIHDMI_SUPPLY_NUM] = {
	{ .supply = "AVCC", },
	{ .supply = "CAVCC", },
	{ .supply = "IOVCC", },
};

static inline int siihdmi_write(struct i2c_client *client, uint8_t addr, uint8_t val)
{
	int ret;
	dev_dbg(&client->dev, "%s(0x%x,0x%x)\n", __func__, addr, val);

	ret = i2c_smbus_write_byte_data(client, addr, val);
	if (ret < 0) {
		dev_dbg(&client->dev, "%s(0x%x,0x%x) failed with %d\n", __func__, addr, val, ret);
	}
	return ret;
}

static inline int siihdmi_writeblock(struct i2c_client *client, uint8_t addr, uint8_t *data, u32 len)
{
	int ret;
	dev_dbg(&client->dev, "%s(0x%x,%d)\n", __func__, addr, len);

	ret = i2c_smbus_write_i2c_block_data(client, addr, len, (u8 *) data);
	if (ret < 0) {
		dev_dbg(&client->dev, "%s(0x%x,%d) failed with %d\n", __func__, addr, len, ret);
	}
	return ret;
}

static inline uint8_t siihdmi_read(struct i2c_client *client, uint8_t addr)
{
	int dat;

	dat = i2c_smbus_read_byte_data(client, addr);

	dev_dbg(&client->dev, "%s(0x%x) returned 0x%x\n", __func__, addr, dat);

	return dat;
}

static void siihdmi_poweron(struct siihdmi_tx *tx)
{
	dev_dbg(tx->dev, "%s\n", __func__);

	/* Turn on DVI or HDMI */
	if (tx->sink.type == SINK_TYPE_HDMI) {
		siihdmi_write(tx->client, SIIHDMI_TPI_REG_SYS_CTRL, 0x01 | 4);
	} else {
		siihdmi_write(tx->client, SIIHDMI_TPI_REG_SYS_CTRL, 0x00);
	}

	return;
}

static void siihdmi_poweroff(struct siihdmi_tx *tx)
{
	dev_dbg(tx->dev, "%s\n", __func__);

	/* disable tmds before changing resolution */
	if (tx->sink.type == SINK_TYPE_HDMI) {
		siihdmi_write(tx->client, SIIHDMI_TPI_REG_SYS_CTRL, 0x11);
	} else {
		siihdmi_write(tx->client, SIIHDMI_TPI_REG_SYS_CTRL, 0x10);
	}
	return;
}

static int siihdmi_connector_get_modes(struct drm_connector *connector)
{
	struct siihdmi_tx *tx = con_to_siihdmi(connector);
	struct edid *edid;
	int ret;
	int old, dat, cnt = 100;

	dev_dbg(tx->dev, "%s\n", __func__);

	old = siihdmi_read(tx->client, SIIHDMI_TPI_REG_SYS_CTRL);

	siihdmi_write(tx->client, SIIHDMI_TPI_REG_SYS_CTRL, old | 0x4);
	do {
		cnt--;
		msleep(10);
		dat = siihdmi_read(tx->client, SIIHDMI_TPI_REG_SYS_CTRL);
	} while ((!(dat & 0x2)) && cnt);

	if (!cnt)
		return -ETIMEDOUT;

	siihdmi_write(tx->client, SIIHDMI_TPI_REG_SYS_CTRL, old | 0x06);

	edid = drm_get_edid(connector, tx->client->adapter);
	if (edid) {
		drm_mode_connector_update_edid_property(connector, edid);
		ret = drm_add_edid_modes(connector, edid);
//		connector->display_info.raw_edid = NULL;
		kfree(edid);
	}

	cnt = 100;
	do {
		cnt--;
		siihdmi_write(tx->client, SIIHDMI_TPI_REG_SYS_CTRL, old & ~0x6);
		msleep(10);
		dat = siihdmi_read(tx->client, SIIHDMI_TPI_REG_SYS_CTRL);
	} while ((dat & 0x6) && cnt);

	if (!cnt)
		ret = -1;

	siihdmi_write(tx->client, SIIHDMI_TPI_REG_SYS_CTRL, old);

	return 0;
}

static irqreturn_t siihdmi_hotplug_handler(int irq, void *data)
{
	struct siihdmi_tx *tx = data;
	int dat;

	dev_dbg(tx->dev, "%s\n", __func__);

	dat = siihdmi_read(tx->client, SIIHDMI_TPI_REG_ISR);
	if (dat & 0x1) {
		/* cable connection changes */
		if (dat & 0x4) {
			DRM_DEBUG("plugin\n");
		} else {
			DRM_DEBUG("plugout\n");
		}
	}
	siihdmi_write(tx->client, SIIHDMI_TPI_REG_ISR, dat);

	return IRQ_HANDLED;
}

static enum drm_connector_status siihdmi_connector_detect(
		struct drm_connector *connector, bool force)
{
	return connector_status_connected;
}

static void siihdmi_connector_destroy(struct drm_connector *connector)
{
	/* do not free here */
}

static int siihdmi_connector_mode_valid(struct drm_connector *connector,
			  struct drm_display_mode *mode)
{
	if (mode->clock > 128000000UL) // hack
		return MODE_CLOCK_HIGH;

	return MODE_OK;
}

static struct drm_encoder *siihdmi_connector_best_encoder(
		struct drm_connector *connector)
{
	struct siihdmi_tx *tx = con_to_siihdmi(connector);

	return &tx->encoder;
}

static bool siihdmi_encoder_mode_fixup(struct drm_encoder *encoder,
			const struct drm_display_mode *mode,
			struct drm_display_mode *adjusted_mode)
{
	return true;
}

static void siihdmi_encoder_mode_set(struct drm_encoder *encoder,
			 struct drm_display_mode *mode,
			 struct drm_display_mode *adjusted_mode)
{
	struct siihdmi_tx *tx = enc_to_siihdmi(encoder);
	const u8 aviif[SIIHDMI_TPI_REG_AVI_INFO_FRAME_LENGTH] = {0};
	u16 vmode[4];

	siihdmi_write(tx->client, SIIHDMI_TPI_REG_PWR_STATE, 0x00);

	dev_dbg(&tx->client->dev, "%s: %dx%d@%d, pixclk %d\n", __func__,
			mode->hdisplay, mode->vdisplay, mode->vrefresh,
			mode->clock * 1000);

	/* set TPI video mode */
	vmode[0] = mode->clock / 10;
	vmode[1] = mode->vrefresh * 100;
	vmode[2] = mode->htotal;
	vmode[3] = mode->vtotal;

	siihdmi_writeblock(tx->client, SIIHDMI_TPI_REG_VIDEO_MODE_DATA_BASE, (uint8_t *) vmode, 8);

	/* input bus/pixel: full pixel wide (24bit), rising edge */
	siihdmi_write(tx->client, SIIHDMI_TPI_REG_INPUT_BUS_PIXEL_REPETITION, 0x70);

	/* Set input format to RGB */
	siihdmi_write(tx->client, SIIHDMI_TPI_REG_AVI_INPUT_FORMAT, 0x00);

	/* set output format to RGB */
	siihdmi_write(tx->client, SIIHDMI_TPI_REG_AVI_OUTPUT_FORMAT, 0x00);

	/* audio setup */
	siihdmi_write(tx->client, SIIHDMI_TPI_REG_I2S_ORIGINAL_FREQ_SAMPLE_LENGTH, 0x00);
	siihdmi_write(tx->client, SIIHDMI_TPI_REG_I2S_AUDIO_PACKET_LAYOUT_CTRL, 0x40);
	siihdmi_write(tx->client, SIIHDMI_TPI_REG_I2S_AUDIO_SAMPLING_HBR, 0x00);

	/* clear avi infoframe */
	siihdmi_writeblock(tx->client, SIIHDMI_TPI_REG_AVI_INFO_FRAME_BASE, (uint8_t *)aviif,
						SIIHDMI_TPI_REG_AVI_INFO_FRAME_LENGTH);
}

static void siihdmi_encoder_dpms(struct drm_encoder *encoder, int mode)
{
	struct siihdmi_tx *tx = enc_to_siihdmi(encoder);

	if (mode)
		siihdmi_poweroff(tx);
	else
		siihdmi_poweron(tx);
}

static void siihdmi_encoder_prepare(struct drm_encoder *encoder)
{
	struct siihdmi_tx *tx = enc_to_siihdmi(encoder);

	dev_dbg(tx->dev, "%s\n", __func__);

	siihdmi_poweroff(tx);

	imx_drm_crtc_panel_format(encoder->crtc, DRM_MODE_ENCODER_TMDS,
	                        tx->interface_pix_fmt);

	/* resolution stabilize time (50-500ms) */
	msleep(100);
}

static void siihdmi_encoder_commit(struct drm_encoder *encoder)
{
	struct siihdmi_tx *tx = enc_to_siihdmi(encoder);

	dev_dbg(tx->dev, "%s\n", __func__);

	siihdmi_poweron(tx);
}


static void siihdmi_encoder_disable(struct drm_encoder *encoder)
{
	struct siihdmi_tx *tx = enc_to_siihdmi(encoder);

	dev_dbg(tx->dev, "%s\n", __func__);
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


static int siihdmi_register(struct siihdmi_tx *tx)
{
	int ret;

	drm_mode_connector_attach_encoder(&tx->connector, &tx->encoder);

	tx->connector.funcs = &siihdmi_connector_funcs;
	tx->encoder.funcs = &siihdmi_encoder_funcs;

	tx->encoder.encoder_type = DRM_MODE_ENCODER_TMDS;
	tx->connector.connector_type = DRM_MODE_CONNECTOR_HDMIA;

	drm_encoder_helper_add(&tx->encoder, &siihdmi_encoder_helper_funcs);
	ret = imx_drm_add_encoder(&tx->encoder, &tx->imx_drm_encoder,
			THIS_MODULE);
	if (ret) {
		dev_err(tx->dev, "adding encoder failed with %d\n", ret);
		return ret;
	}

	drm_connector_helper_add(&tx->connector,
			&siihdmi_connector_helper_funcs);

	ret = imx_drm_add_connector(&tx->connector,
			&tx->imx_drm_connector, THIS_MODULE);
	if (ret) {
		imx_drm_remove_encoder(tx->imx_drm_encoder);
		dev_err(tx->dev, "adding connector failed with %d\n", ret);
		return ret;
	}

	tx->connector.encoder = &tx->encoder;

	return 0;
}


static int siihdmi_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device_node *np = client->dev.of_node;
	struct siihdmi_tx *tx;
	int dat, ret;
	const char *fmt;
	struct pinctrl *pinctrl;

	tx = devm_kzalloc(&client->dev, sizeof(*tx), GFP_KERNEL);
	if (!tx)
		return -ENOMEM;

	tx->client = client;
	tx->dev = &client->dev; // ugh isn't this redundant?
	i2c_set_clientdata(client, tx);

	ret = devm_regulator_bulk_get(&client->dev, ARRAY_SIZE(siihdmi_supplies),
					siihdmi_supplies);

	ret = regulator_bulk_enable(ARRAY_SIZE(siihdmi_supplies),
					siihdmi_supplies);

	pinctrl = devm_pinctrl_get_select_default(&client->dev);
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		dev_warn(&client->dev, "pinctrl_get_select_default failed with %d\n", ret);
		return ret;
	}

	ret = of_property_read_string(np, "interface-pix-fmt", &fmt);
	if (!ret) {
		if (!strcmp(fmt, "rgb24"))
			tx->interface_pix_fmt = V4L2_PIX_FMT_RGB24;
		else if (!strcmp(fmt, "rgb565"))
			tx->interface_pix_fmt = V4L2_PIX_FMT_RGB565;
	}

	tx->gpio_enable1 = of_get_named_gpio(np, "gpios", 0);
	gpio_request_one(tx->gpio_enable1, GPIOF_DIR_OUT, "disp1_en");
	tx->gpio_enable2 = of_get_named_gpio(np, "gpios", 1);
	gpio_request_one(tx->gpio_enable2, GPIOF_DIR_OUT, "disp2_en");

	gpio_set_value(tx->gpio_enable1, 0);
	gpio_set_value(tx->gpio_enable2, 0);

	tx->gpio_reset = of_get_named_gpio(np, "gpios", 2);
	if (tx->gpio_reset) {
		if (gpio_request_one(tx->gpio_reset, GPIOF_DIR_OUT, "siihdmi_reset")) {
			dev_err(&tx->client->dev, "siihdmi: could not request reset GPIO\n");
			return -EINVAL;
		} else {
			gpio_set_value(tx->gpio_reset, 1);
			msleep(100);            /* SII9022 Treset >= 100us */
			gpio_set_value(tx->gpio_reset, 0);
			msleep(480);
		}
	} else {
		dev_info(&tx->client->dev, "siihdmi: reset GPIO not set\n");
		return -EINVAL;
	}

	/* Set 902x in hardware TPI mode on and jump out of D3 state */
	if (siihdmi_write(tx->client, SIIHDMI_TPI_REG_RQB, 0x00) < 0) {
		dev_err(&tx->client->dev, "siihdmi: cound not find device\n");
		return -ENODEV;
	}

	dat = siihdmi_read(tx->client, SIIHDMI_TPI_REG_DEVICE_ID);
	if (dat != 0xb0) {
		dev_err(&tx->client->dev, "not found. id is 0x%02x instead of 0xb0\n",
				dat);
		return -ENODEV;
	}

	siihdmi_poweron(tx);

	if (client->irq) {
		ret = request_threaded_irq(tx->client->irq, NULL, siihdmi_hotplug_handler,
				IRQF_TRIGGER_FALLING,
				"siihdmi_irq", tx);
		siihdmi_write(tx->client, SIIHDMI_TPI_REG_IER,
					SIIHDMI_IER_HOT_PLUG_EVENT |
					SIIHDMI_IER_RECEIVER_SENSE_EVENT);
	}

	ret = siihdmi_register(tx);
	if (ret)
		return ret;

	ret = imx_drm_encoder_add_possible_crtcs(tx->imx_drm_encoder, np);

	dev_info(&client->dev, "initialized\n");

	return 0;
}

static int siihdmi_remove(struct i2c_client *client)
{
	struct siihdmi_tx *tx = i2c_get_clientdata(client);
	struct drm_connector *connector = &tx->connector;
	struct drm_encoder *encoder = &tx->encoder;

	drm_mode_connector_attach_encoder(connector, encoder);

	imx_drm_remove_connector(tx->imx_drm_connector);
	imx_drm_remove_encoder(tx->imx_drm_encoder);

	return 0;
}

static struct i2c_device_id siihdmi_ids[] = {
	{ "siihdmi", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, siihdmi_ids);

static const struct of_device_id siihdmi_dt_ids[] = {
	{ .compatible = "sii,sii9022", },
	{ /* sentinel */ }
};

static struct i2c_driver siihdmi_i2c_driver = {
	.probe = siihdmi_probe,
	.remove = siihdmi_remove,
	.driver = {
		.of_match_table = siihdmi_dt_ids,
		.name = "siihdmi",
		.owner = THIS_MODULE,
	},
	.id_table = siihdmi_ids,
};

static int __init siihdmi_init(void)
{
	return i2c_add_driver(&siihdmi_i2c_driver);
}

static void __exit siihdmi_exit(void)
{
	i2c_del_driver(&siihdmi_i2c_driver);
}

MODULE_DESCRIPTION("Silicon Image HDMI transmitter driver");
MODULE_AUTHOR("Genesi USA, Inc.");
MODULE_LICENSE("GPL");

module_init(siihdmi_init);
module_exit(siihdmi_exit);
