// SPDX-License-Identifier: GPL-2.0-only
/*
 * Lenovo ThinkPad T14s/X13s Embedded Controller Driver
 *
 * Copyright (c) 2025 Sebastian Reichel <sre@kernel.org>
 * Copyright (c) 2025 Steev Klimaszewski <threeway@gmail.com>
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/container_of.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/dev_printk.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/sparse-keymap.h>
#include <linux/interrupt.h>
#include <linux/leds.h>
#include <linux/lockdep.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/pm.h>

#define T14S_EC_CMD_ECRD	0x02
#define T14S_EC_CMD_ECWR	0x03
#define T14S_EC_CMD_EVT		0xf0

/* T14s register definitions */
#define T14S_EC_REG_LED				0x0c
#define T14S_EC_REG_KBD_BL1			0x0d
#define T14S_EC_REG_MODERN_STANDBY		0xe0
#define T14S_EC_MODERN_STANDBY_ENTRY		BIT(1)
#define T14S_EC_MODERN_STANDBY_EXIT		BIT(0)
#define T14S_EC_REG_KBD_BL2			0xe1
#define T14S_EC_KBD_BL1_MASK			GENMASK_U8(7, 6)
#define T14S_EC_KBD_BL2_MASK			GENMASK_U8(3, 2)
#define T14S_EC_REG_AUD				0x30
#define T14S_EC_MIC_MUTE_LED			BIT(5)
#define T14S_EC_SPK_MUTE_LED			BIT(6)

/* X13s register definitions */
#define X13S_EC_REG_KBD_BL			0xc0
#define X13S_EC_KBD_BL_MASK			GENMASK_U8(5, 4)
#define X13S_EC_KBD_BL_EN			BIT(6)
#define X13S_EC_REG_SUSPEND			0x80
#define X13S_EC_SUSPEND_ENTER			0x55
#define X13S_EC_SUSPEND_EXIT			0xaa

/* T14s event codes */
#define T14S_EC_EVT_NONE			0x00
#define T14S_EC_EVT_KEY_FN_4			0x13
#define T14S_EC_EVT_KEY_FN_F7			0x16
#define T14S_EC_EVT_KEY_FN_SPACE		0x1f
#define T14S_EC_EVT_KEY_TP_DOUBLE_TAP		0x20
#define T14S_EC_EVT_AC_CONNECTED		0x26
#define T14S_EC_EVT_AC_DISCONNECTED		0x27
#define T14S_EC_EVT_KEY_POWER			0x28
#define T14S_EC_EVT_LID_OPEN			0x2a
#define T14S_EC_EVT_LID_CLOSED			0x2b
#define T14S_EC_EVT_THERMAL_TZ40		0x5c
#define T14S_EC_EVT_THERMAL_TZ42		0x5d
#define T14S_EC_EVT_THERMAL_TZ39		0x5e
#define T14S_EC_EVT_KEY_FN_F12			0x62
#define T14S_EC_EVT_KEY_FN_TAB			0x63
#define T14S_EC_EVT_KEY_FN_F8			0x64
#define T14S_EC_EVT_KEY_FN_F10			0x65
#define T14S_EC_EVT_KEY_FN_F4			0x6a
#define T14S_EC_EVT_KEY_FN_D			0x6b
#define T14S_EC_EVT_KEY_FN_T			0x6c
#define T14S_EC_EVT_KEY_FN_H			0x6d
#define T14S_EC_EVT_KEY_FN_M			0x6e
#define T14S_EC_EVT_KEY_FN_L			0x6f
#define T14S_EC_EVT_KEY_FN_RIGHT_SHIFT		0x71
#define T14S_EC_EVT_KEY_FN_ESC			0x74
#define T14S_EC_EVT_KEY_FN_N			0x79
#define T14S_EC_EVT_KEY_FN_F11			0x7a
#define T14S_EC_EVT_KEY_FN_G			0x7e

/* X13s event codes */
#define X13S_EC_EVT_KEY_FN_F7			0x19
#define X13S_EC_EVT_KEY_FN_F4			0x28
#define X13S_EC_EVT_KEY_FN_F8			0x2a
#define X13S_EC_EVT_PERF_MODE			0x3c
#define X13S_EC_EVT_AC_CONNECTED		0x50
#define X13S_EC_EVT_AC_DISCONNECTED		0x51
#define X13S_EC_EVT_LID_OPEN			0x52
#define X13S_EC_EVT_LID_CLOSED			0x53
#define X13S_EC_EVT_UNKNOWN_0x60		0x60
#define X13S_EC_EVT_KEY_FN_PRTSCR		0x62
#define X13S_EC_EVT_KEY_FN_F10			0x6c
#define X13S_EC_EVT_KEY_FN_F11			0x6d
#define X13S_EC_EVT_KEY_FN_F12			0x6e
#define X13S_EC_EVT_KEY_FN_ESC			0x75

/* Hardware LED blink rate is 1 Hz (500ms off, 500ms on) */
#define T14S_EC_BLINK_RATE_ON_OFF_MS		500

/*
 * Add a virtual offset on all key event codes for sparse keymap handling,
 * since the sparse keymap infrastructure does not map some raw key event
 * codes used by the EC. For example 0x16 (T14S_EC_EVT_KEY_FN_F7) is mapped
 * to KEY_MUTE if no offset is applied.
 *
 * X13s uses a separate offset to avoid conflicts between T14s and X13s event
 * codes (e.g., 0x28 means different things on each platform).
 */
#define T14S_EC_KEY_EVT_OFFSET			0x1000
#define T14S_EC_KEY_ENTRY(key, value) \
	{ KE_KEY, T14S_EC_KEY_EVT_OFFSET + T14S_EC_EVT_KEY_##key, { value } }

#define X13S_EC_KEY_EVT_OFFSET			0x2000
#define X13S_EC_KEY_ENTRY(key, value) \
	{ KE_KEY, X13S_EC_KEY_EVT_OFFSET + X13S_EC_EVT_KEY_##key, { value } }

enum ec_variant {
	EC_VARIANT_T14S,
	EC_VARIANT_X13S,
};

enum t14s_ec_led_status_t {
	T14S_EC_LED_OFF =	0x00,
	T14S_EC_LED_ON =	0x80,
	T14S_EC_LED_BLINK =	0xc0,
};

struct t14s_ec_led_classdev {
	struct led_classdev led_classdev;
	int led;
	enum t14s_ec_led_status_t cache;
	struct t14s_ec *ec;
};

struct t14s_ec {
	struct regmap *regmap;
	struct device *dev;
	enum ec_variant variant;
	struct t14s_ec_led_classdev led_pwr_btn;
	struct t14s_ec_led_classdev led_chrg_orange;
	struct t14s_ec_led_classdev led_chrg_white;
	struct t14s_ec_led_classdev led_lid_logo_dot;
	struct led_classdev kbd_backlight;
	struct led_classdev led_mic_mute;
	struct led_classdev led_spk_mute;
	struct input_dev *inputdev;
};

static const struct regmap_config t14s_ec_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
};

static int t14s_ec_write(void *context, unsigned int reg, unsigned int val)
{
	struct t14s_ec *ec = context;
	struct i2c_client *client = to_i2c_client(ec->dev);
	u8 buf[5] = {T14S_EC_CMD_ECWR, reg, 0x00, 0x01, val};
	int ret;

	ret = i2c_master_send(client, buf, sizeof(buf));
	if (ret < 0)
		return ret;

	fsleep(10000);
	return 0;
}

static int t14s_ec_read(void *context, unsigned int reg, unsigned int *val)
{
	struct t14s_ec *ec = context;
	struct i2c_client *client = to_i2c_client(ec->dev);
	u8 buf[4] = {T14S_EC_CMD_ECRD, reg, 0x00, 0x01};
	struct i2c_msg request, response;
	u8 result;
	int ret;

	request.addr = client->addr;
	request.flags = I2C_M_STOP;
	request.len = sizeof(buf);
	request.buf = buf;
	response.addr = client->addr;
	response.flags = I2C_M_RD;
	response.len = 1;
	response.buf = &result;

	i2c_lock_bus(client->adapter, I2C_LOCK_SEGMENT);

	ret = __i2c_transfer(client->adapter, &request, 1);
	if (ret < 0)
		goto out;

	ret = __i2c_transfer(client->adapter, &response, 1);
	if (ret < 0)
		goto out;

	*val = result;
	ret = 0;

out:
	i2c_unlock_bus(client->adapter, I2C_LOCK_SEGMENT);
	fsleep(10000);
	return ret;
}

static const struct regmap_bus t14s_ec_regmap_bus = {
	.reg_write = t14s_ec_write,
	.reg_read = t14s_ec_read,
};

static int t14s_ec_read_evt(struct t14s_ec *ec, u8 *val)
{
	struct i2c_client *client = to_i2c_client(ec->dev);
	u8 buf[4] = {T14S_EC_CMD_EVT, 0x00, 0x00, 0x01};
	struct i2c_msg request, response;
	int ret;

	request.addr = client->addr;
	request.flags = I2C_M_STOP;
	request.len = sizeof(buf);
	request.buf = buf;
	response.addr = client->addr;
	response.flags = I2C_M_RD;
	response.len = 1;
	response.buf = val;

	i2c_lock_bus(client->adapter, I2C_LOCK_SEGMENT);

	ret = __i2c_transfer(client->adapter, &request, 1);
	if (ret < 0)
		goto out;

	ret = __i2c_transfer(client->adapter, &response, 1);
	if (ret < 0)
		goto out;

	fsleep(10000);

	ret = 0;

out:
	i2c_unlock_bus(client->adapter, I2C_LOCK_SEGMENT);
	return ret;
}

static void t14s_ec_write_sequence(struct t14s_ec *ec, u8 reg, u8 val, u8 cnt)
{
	int i;

	for (i = 0; i < cnt; i++)
		regmap_write(ec->regmap, reg, val);
}

static int t14s_led_set_status(struct t14s_ec *ec,
			       struct t14s_ec_led_classdev *led,
			       const enum t14s_ec_led_status_t ledstatus)
{
	int ret;

	ret = regmap_write(ec->regmap, T14S_EC_REG_LED,
			   led->led | ledstatus);
	if (ret < 0)
		return ret;

	led->cache = ledstatus;
	return 0;
}

static int t14s_led_brightness_set(struct led_classdev *led_cdev,
				   enum led_brightness brightness)
{
	struct t14s_ec_led_classdev *led = container_of(led_cdev,
				struct t14s_ec_led_classdev, led_classdev);
	enum t14s_ec_led_status_t new_state;

	if (brightness == LED_OFF)
		new_state = T14S_EC_LED_OFF;
	else if (led->cache == T14S_EC_LED_BLINK)
		new_state = T14S_EC_LED_BLINK;
	else
		new_state = T14S_EC_LED_ON;

	return t14s_led_set_status(led->ec, led, new_state);
}

static int t14s_led_blink_set(struct led_classdev *led_cdev,
			      unsigned long *delay_on,
			      unsigned long *delay_off)
{
	struct t14s_ec_led_classdev *led = container_of(led_cdev,
				struct t14s_ec_led_classdev, led_classdev);

	if (*delay_on == 0 && *delay_off == 0) {
		/* Userspace does not provide a blink rate; we can choose it */
		*delay_on = T14S_EC_BLINK_RATE_ON_OFF_MS;
		*delay_off = T14S_EC_BLINK_RATE_ON_OFF_MS;
	} else if ((*delay_on != T14S_EC_BLINK_RATE_ON_OFF_MS) ||
		   (*delay_off != T14S_EC_BLINK_RATE_ON_OFF_MS))
		return -EINVAL;

	return t14s_led_set_status(led->ec, led, T14S_EC_LED_BLINK);
}

static int t14s_init_led(struct t14s_ec *ec, struct t14s_ec_led_classdev *led,
			 u8 id, const char *name)
{
	led->led_classdev.name = name;
	led->led_classdev.flags = LED_RETAIN_AT_SHUTDOWN;
	led->led_classdev.max_brightness = 1;
	led->led_classdev.brightness_set_blocking = t14s_led_brightness_set;
	led->led_classdev.blink_set = t14s_led_blink_set;
	led->ec = ec;
	led->led = id;

	return devm_led_classdev_register(ec->dev, &led->led_classdev);
}

static int t14s_leds_probe(struct t14s_ec *ec)
{
	int ret;

	ret = t14s_init_led(ec, &ec->led_pwr_btn, 0, "platform::power");
	if (ret)
		return ret;

	ret = t14s_init_led(ec, &ec->led_chrg_orange, 1,
			    "platform:amber:battery-charging");
	if (ret)
		return ret;

	ret = t14s_init_led(ec, &ec->led_chrg_white, 2,
			    "platform:white:battery-full");
	if (ret)
		return ret;

	ret = t14s_init_led(ec, &ec->led_lid_logo_dot, 10,
			    "platform::lid_logo_dot");
	if (ret)
		return ret;

	return 0;
}

static int t14s_kbd_bl_set(struct led_classdev *led_cdev,
			   enum led_brightness brightness)
{
	struct t14s_ec *ec = container_of(led_cdev, struct t14s_ec,
					  kbd_backlight);
	int ret;
	u8 val;

	if (ec->variant == EC_VARIANT_X13S) {
		/* X13s uses a single register with bits [5:4] for brightness */
		val = FIELD_PREP(X13S_EC_KBD_BL_MASK, brightness);
		/* Keep bit 6 set (enable bit) */
		val |= X13S_EC_KBD_BL_EN;
		ret = regmap_write(ec->regmap, X13S_EC_REG_KBD_BL, val);
		if (ret < 0)
			return ret;
	} else {
		/* T14s uses two registers */
		val = FIELD_PREP(T14S_EC_KBD_BL1_MASK, brightness);
		ret = regmap_update_bits(ec->regmap, T14S_EC_REG_KBD_BL1,
					 T14S_EC_KBD_BL1_MASK, val);
		if (ret < 0)
			return ret;

		val = FIELD_PREP(T14S_EC_KBD_BL2_MASK, brightness);
		ret = regmap_update_bits(ec->regmap, T14S_EC_REG_KBD_BL2,
					 T14S_EC_KBD_BL2_MASK, val);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static enum led_brightness t14s_kbd_bl_get(struct led_classdev *led_cdev)
{
	struct t14s_ec *ec = container_of(led_cdev, struct t14s_ec,
					  kbd_backlight);
	unsigned int val;
	int ret;

	if (ec->variant == EC_VARIANT_X13S) {
		ret = regmap_read(ec->regmap, X13S_EC_REG_KBD_BL, &val);
		if (ret < 0)
			return ret;
		return FIELD_GET(X13S_EC_KBD_BL_MASK, val);
	} else {
		ret = regmap_read(ec->regmap, T14S_EC_REG_KBD_BL1, &val);
		if (ret < 0)
			return ret;
		return FIELD_GET(T14S_EC_KBD_BL1_MASK, val);
	}
}

static void t14s_kbd_bl_update(struct t14s_ec *ec)
{
	enum led_brightness brightness = t14s_kbd_bl_get(&ec->kbd_backlight);

	led_classdev_notify_brightness_hw_changed(&ec->kbd_backlight,
						   brightness);
}

static int t14s_kbd_backlight_probe(struct t14s_ec *ec)
{
	ec->kbd_backlight.name = "platform::kbd_backlight";
	ec->kbd_backlight.flags = LED_BRIGHT_HW_CHANGED;
	ec->kbd_backlight.max_brightness = 2;
	ec->kbd_backlight.brightness_set_blocking = t14s_kbd_bl_set;
	ec->kbd_backlight.brightness_get = t14s_kbd_bl_get;

	return devm_led_classdev_register(ec->dev, &ec->kbd_backlight);
}

static enum led_brightness t14s_audio_led_get(struct t14s_ec *ec, u8 led_bit)
{
	unsigned int val;
	int ret;

	ret = regmap_read(ec->regmap, T14S_EC_REG_AUD, &val);
	if (ret < 0)
		return ret;

	return !!(val & led_bit) ? LED_ON : LED_OFF;
}

static int t14s_audio_led_set(struct t14s_ec *ec, u8 led_mask,
			       enum led_brightness brightness)
{
	return regmap_assign_bits(ec->regmap, T14S_EC_REG_AUD, led_mask,
				  brightness > 0);
}

static enum led_brightness t14s_mic_mute_led_get(struct led_classdev *led_cdev)
{
	struct t14s_ec *ec = container_of(led_cdev, struct t14s_ec,
					  led_mic_mute);

	return t14s_audio_led_get(ec, T14S_EC_MIC_MUTE_LED);
}

static int t14s_mic_mute_led_set(struct led_classdev *led_cdev,
				  enum led_brightness brightness)
{
	struct t14s_ec *ec = container_of(led_cdev, struct t14s_ec,
					  led_mic_mute);

	return t14s_audio_led_set(ec, T14S_EC_MIC_MUTE_LED, brightness);
}

static enum led_brightness t14s_spk_mute_led_get(struct led_classdev *led_cdev)
{
	struct t14s_ec *ec = container_of(led_cdev, struct t14s_ec,
					  led_spk_mute);

	return t14s_audio_led_get(ec, T14S_EC_SPK_MUTE_LED);
}

static int t14s_spk_mute_led_set(struct led_classdev *led_cdev,
				  enum led_brightness brightness)
{
	struct t14s_ec *ec = container_of(led_cdev, struct t14s_ec,
					  led_spk_mute);

	return t14s_audio_led_set(ec, T14S_EC_SPK_MUTE_LED, brightness);
}

static int t14s_kbd_audio_led_probe(struct t14s_ec *ec)
{
	int ret;

	ec->led_mic_mute.name = "platform::micmute";
	ec->led_mic_mute.max_brightness = 1;
	ec->led_mic_mute.default_trigger = "audio-micmute";
	ec->led_mic_mute.brightness_set_blocking = t14s_mic_mute_led_set;
	ec->led_mic_mute.brightness_get = t14s_mic_mute_led_get;

	ec->led_spk_mute.name = "platform::mute";
	ec->led_spk_mute.max_brightness = 1;
	ec->led_spk_mute.default_trigger = "audio-mute";
	ec->led_spk_mute.brightness_set_blocking = t14s_spk_mute_led_set;
	ec->led_spk_mute.brightness_get = t14s_spk_mute_led_get;

	ret = devm_led_classdev_register(ec->dev, &ec->led_mic_mute);
	if (ret)
		return ret;

	return devm_led_classdev_register(ec->dev, &ec->led_spk_mute);
}

static const struct key_entry t14s_keymap[] = {
	T14S_EC_KEY_ENTRY(FN_4, KEY_SLEEP),
	T14S_EC_KEY_ENTRY(FN_N, KEY_VENDOR),
	T14S_EC_KEY_ENTRY(FN_F4, KEY_MICMUTE),
	T14S_EC_KEY_ENTRY(FN_F7, KEY_SWITCHVIDEOMODE),
	T14S_EC_KEY_ENTRY(FN_F8, KEY_PERFORMANCE),
	T14S_EC_KEY_ENTRY(FN_F10, KEY_SELECTIVE_SCREENSHOT),
	T14S_EC_KEY_ENTRY(FN_F11, KEY_LINK_PHONE),
	T14S_EC_KEY_ENTRY(FN_F12, KEY_BOOKMARKS),
	T14S_EC_KEY_ENTRY(FN_SPACE, KEY_KBDILLUMTOGGLE),
	T14S_EC_KEY_ENTRY(FN_ESC, KEY_FN_ESC),
	T14S_EC_KEY_ENTRY(FN_TAB, KEY_ZOOM),
	T14S_EC_KEY_ENTRY(FN_RIGHT_SHIFT, KEY_FN_RIGHT_SHIFT),
	T14S_EC_KEY_ENTRY(TP_DOUBLE_TAP, KEY_PROG4),
	{ KE_END }
};

static const struct key_entry x13s_keymap[] = {
	/* X13s shares some event codes with T14s */
	T14S_EC_KEY_ENTRY(FN_4, KEY_SLEEP),
	T14S_EC_KEY_ENTRY(FN_SPACE, KEY_KBDILLUMTOGGLE),
	T14S_EC_KEY_ENTRY(TP_DOUBLE_TAP, KEY_PROG4),
	/* X13s-specific event codes */
	X13S_EC_KEY_ENTRY(FN_F4, KEY_MICMUTE),
	X13S_EC_KEY_ENTRY(FN_F7, KEY_SWITCHVIDEOMODE),
	X13S_EC_KEY_ENTRY(FN_F8, KEY_RFKILL),
	X13S_EC_KEY_ENTRY(FN_F10, KEY_PHONE),
	X13S_EC_KEY_ENTRY(FN_F11, KEY_SUSPEND),
	X13S_EC_KEY_ENTRY(FN_F12, KEY_FAVORITES),
	X13S_EC_KEY_ENTRY(FN_PRTSCR, KEY_SYSRQ),
	X13S_EC_KEY_ENTRY(FN_ESC, KEY_FN_ESC),
	{ KE_END }
};

static int t14s_input_probe(struct t14s_ec *ec)
{
	const struct key_entry *keymap;
	int ret;

	ec->inputdev = devm_input_allocate_device(ec->dev);
	if (!ec->inputdev)
		return -ENOMEM;

	ec->inputdev->name = "ThinkPad Extra Buttons";
	ec->inputdev->phys = "thinkpad/input0";
	ec->inputdev->id.bustype = BUS_HOST;
	ec->inputdev->dev.parent = ec->dev;

	keymap = (ec->variant == EC_VARIANT_X13S) ? x13s_keymap : t14s_keymap;
	ret = sparse_keymap_setup(ec->inputdev, keymap, NULL);
	if (ret)
		return ret;

	return input_register_device(ec->inputdev);
}

static irqreturn_t t14s_ec_irq_handler(int irq, void *data)
{
	struct t14s_ec *ec = data;
	int ret;
	u8 val;

	ret = t14s_ec_read_evt(ec, &val);
	if (ret < 0) {
		dev_err(ec->dev, "Failed to read event\n");
		return IRQ_HANDLED;
	}

	if (val == T14S_EC_EVT_NONE)
		return IRQ_HANDLED;

	pm_wakeup_event(ec->dev, 0);

	/*
	 * Handle events based on variant to avoid event code collisions
	 * between T14s and X13s. Several event codes have different meanings
	 * on each platform (e.g., 0x28, 0x2a, 0x6c, 0x6d, 0x6e).
	 */
	if (ec->variant == EC_VARIANT_X13S) {
		/* X13s-specific event handling */
		switch (val) {
		/* Events shared with T14s (same codes, same meaning) */
		case T14S_EC_EVT_KEY_FN_4:
		case T14S_EC_EVT_KEY_FN_SPACE:
		case T14S_EC_EVT_KEY_TP_DOUBLE_TAP:
			sparse_keymap_report_event(ec->inputdev,
						   X13S_EC_KEY_EVT_OFFSET + val,
						   1, true);
			break;

		/* X13s-specific event codes */
		case X13S_EC_EVT_KEY_FN_F4:
		case X13S_EC_EVT_KEY_FN_F7:
		case X13S_EC_EVT_KEY_FN_F8:
		case X13S_EC_EVT_KEY_FN_F10:
		case X13S_EC_EVT_KEY_FN_F11:
		case X13S_EC_EVT_KEY_FN_F12:
		case X13S_EC_EVT_KEY_FN_PRTSCR:
		case X13S_EC_EVT_KEY_FN_ESC:
			sparse_keymap_report_event(ec->inputdev,
						   X13S_EC_KEY_EVT_OFFSET + val,
						   1, true);
			break;

		case X13S_EC_EVT_AC_CONNECTED:
		case X13S_EC_EVT_AC_DISCONNECTED:
		case X13S_EC_EVT_LID_OPEN:
		case X13S_EC_EVT_LID_CLOSED:
			dev_dbg(ec->dev, "X13s AC/LID event: 0x%02x\n", val);
			break;

		case X13S_EC_EVT_PERF_MODE:
			dev_dbg(ec->dev, "Performance mode change\n");
			break;

		case X13S_EC_EVT_UNKNOWN_0x60:
			dev_dbg(ec->dev, "X13s event 0x60\n");
			break;

		default:
			dev_info(ec->dev, "Unknown EC event: 0x%02x\n", val);
			break;
		}
	} else {
		/* T14s-specific event handling */
		switch (val) {
		/* Events shared with X13s (same codes, same meaning) */
		case T14S_EC_EVT_KEY_FN_4:
		case T14S_EC_EVT_KEY_FN_SPACE:
			if (val == T14S_EC_EVT_KEY_FN_SPACE)
				t14s_kbd_bl_update(ec);
			sparse_keymap_report_event(ec->inputdev,
						   T14S_EC_KEY_EVT_OFFSET + val,
						   1, true);
			break;

		case T14S_EC_EVT_KEY_TP_DOUBLE_TAP:
			sparse_keymap_report_event(ec->inputdev,
						   T14S_EC_KEY_EVT_OFFSET + val,
						   1, true);
			break;

		/* T14s-specific event codes */
		case T14S_EC_EVT_KEY_POWER:
			dev_dbg(ec->dev, "power button\n");
			break;

		case T14S_EC_EVT_KEY_FN_F4:
		case T14S_EC_EVT_KEY_FN_F7:
		case T14S_EC_EVT_KEY_FN_F8:
		case T14S_EC_EVT_KEY_FN_F10:
		case T14S_EC_EVT_KEY_FN_F11:
		case T14S_EC_EVT_KEY_FN_F12:
		case T14S_EC_EVT_KEY_FN_TAB:
		case T14S_EC_EVT_KEY_FN_N:
		case T14S_EC_EVT_KEY_FN_ESC:
		case T14S_EC_EVT_KEY_FN_RIGHT_SHIFT:
			sparse_keymap_report_event(ec->inputdev,
						   T14S_EC_KEY_EVT_OFFSET + val,
						   1, true);
			break;

		case T14S_EC_EVT_AC_CONNECTED:
		case T14S_EC_EVT_AC_DISCONNECTED:
		case T14S_EC_EVT_LID_OPEN:
		case T14S_EC_EVT_LID_CLOSED:
			dev_dbg(ec->dev, "T14s AC/LID event: 0x%02x\n", val);
			break;

		case T14S_EC_EVT_THERMAL_TZ40:
		case T14S_EC_EVT_THERMAL_TZ42:
		case T14S_EC_EVT_THERMAL_TZ39:
			dev_dbg(ec->dev, "Thermal event: 0x%02x\n", val);
			break;

		case T14S_EC_EVT_KEY_FN_G:
		case T14S_EC_EVT_KEY_FN_L:
		case T14S_EC_EVT_KEY_FN_M:
		case T14S_EC_EVT_KEY_FN_H:
		case T14S_EC_EVT_KEY_FN_T:
		case T14S_EC_EVT_KEY_FN_D:
			dev_dbg(ec->dev, "Fn key event: 0x%02x\n", val);
			break;

		default:
			dev_info(ec->dev, "Unknown EC event: 0x%02x\n", val);
			break;
		}
	}

	return IRQ_HANDLED;
}

static const struct of_device_id t14s_ec_of_match[] = {
	{
		.compatible = "lenovo,thinkpad-t14s-ec",
		.data = (void *)(uintptr_t)EC_VARIANT_T14S
	},
	{
		.compatible = "lenovo,thinkpad-x13s-ec",
		.data = (void *)(uintptr_t)EC_VARIANT_X13S
	},
	{}
};
MODULE_DEVICE_TABLE(of, t14s_ec_of_match);

static int t14s_ec_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct t14s_ec *ec;
	int ret;

	ec = devm_kzalloc(dev, sizeof(*ec), GFP_KERNEL);
	if (!ec)
		return -ENOMEM;

	ec->dev = dev;
	i2c_set_clientdata(client, ec);

	/* Determine variant based on compatible string */
	if (of_device_is_compatible(dev->of_node, "lenovo,thinkpad-x13s-ec"))
		ec->variant = EC_VARIANT_X13S;
	else
		ec->variant = EC_VARIANT_T14S;

	ec->regmap = devm_regmap_init(dev, &t14s_ec_regmap_bus,
				      ec, &t14s_ec_regmap_config);
	if (IS_ERR(ec->regmap))
		return dev_err_probe(dev, PTR_ERR(ec->regmap),
				     "Failed to init regmap\n");

	/* T14s has controllable LEDs, X13s LEDs are automatic */
	if (ec->variant == EC_VARIANT_T14S) {
		ret = t14s_leds_probe(ec);
		if (ret < 0)
			return ret;

		ret = t14s_kbd_audio_led_probe(ec);
		if (ret < 0)
			return ret;
	}

	/* Both variants have keyboard backlight */
	ret = t14s_kbd_backlight_probe(ec);
	if (ret < 0)
		return ret;

	ret = t14s_input_probe(ec);
	if (ret < 0)
		return ret;

	ret = devm_request_threaded_irq(dev, client->irq, NULL,
					t14s_ec_irq_handler,
					IRQF_ONESHOT, dev_name(dev), ec);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to get IRQ\n");

	/*
	 * Enable wakeup capability but disable it by default.
	 * The driver currently does not support masking any events and
	 * the laptop should not wake up when the LID is closed.
	 */
	device_set_wakeup_capable(dev, true);
	device_wakeup_disable(dev);

	dev_info(dev, "Lenovo ThinkPad %s EC initialized\n",
		 ec->variant == EC_VARIANT_X13S ? "X13s" : "T14s");

	return 0;
}

static int t14s_ec_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct t14s_ec *ec = dev_get_drvdata(dev);

	/* Disable IRQ to prevent spurious events during suspend/resume */
	disable_irq(client->irq);

	led_classdev_suspend(&ec->kbd_backlight);

	if (ec->variant == EC_VARIANT_X13S) {
		/* X13s uses register 0x80 for suspend */
		t14s_ec_write_sequence(ec, X13S_EC_REG_SUSPEND,
				       X13S_EC_SUSPEND_ENTER, 3);
	} else {
		/* T14s uses modern standby register */
		t14s_ec_write_sequence(ec, T14S_EC_REG_MODERN_STANDBY,
				       T14S_EC_MODERN_STANDBY_ENTRY, 3);
	}

	return 0;
}

static int t14s_ec_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct t14s_ec *ec = dev_get_drvdata(dev);

	if (ec->variant == EC_VARIANT_X13S) {
		/* X13s uses register 0x80 for resume */
		t14s_ec_write_sequence(ec, X13S_EC_REG_SUSPEND,
				       X13S_EC_SUSPEND_EXIT, 3);
	} else {
		/* T14s uses modern standby register */
		t14s_ec_write_sequence(ec, T14S_EC_REG_MODERN_STANDBY,
				       T14S_EC_MODERN_STANDBY_EXIT, 3);
	}

	led_classdev_resume(&ec->kbd_backlight);

	/* Re-enable IRQ after I2C bus is operational */
	enable_irq(client->irq);

	return 0;
}

static const struct i2c_device_id t14s_ec_i2c_id_table[] = {
	{ "thinkpad-t14s-ec", },
	{ "thinkpad-x13s-ec", },
	{}
};
MODULE_DEVICE_TABLE(i2c, t14s_ec_i2c_id_table);

static const struct dev_pm_ops t14s_ec_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(t14s_ec_suspend, t14s_ec_resume)
};

static struct i2c_driver t14s_ec_i2c_driver = {
	.driver = {
		.name = "thinkpad-t14s-ec",
		.of_match_table = t14s_ec_of_match,
		.pm = &t14s_ec_pm_ops,
	},
	.probe = t14s_ec_probe,
	.id_table = t14s_ec_i2c_id_table,
};
module_i2c_driver(t14s_ec_i2c_driver);

MODULE_AUTHOR("Sebastian Reichel <sre@kernel.org>");
MODULE_AUTHOR("Steev Klimaszewski <threeway@gmail.com>");
MODULE_DESCRIPTION("Lenovo Thinkpad T14s/X13s Embedded Controller");
MODULE_LICENSE("GPL");
