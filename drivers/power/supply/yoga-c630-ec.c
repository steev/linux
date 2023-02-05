// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2022-2023 Linaro Ltd.
 */
#include <linux/bitops.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>

#define LENOVO_EC_CACHE_TIME		(10 * HZ)

#define LENOVO_EC_RESPONSE_REG		0x1
#define LENOVO_EC_REQUEST_REG		0x2

#define LENOVO_EC_READ_REG		0xb0
#define LENOVO_EC_QUERY_USB_EVENT	0x8
#define LENOVO_EC_USB_EVENT_MUX		GENMASK(1, 0)
#define LENOVO_EC_USB_EVENT_ORIENTATION	BIT(7)
#define LENOVO_EC_ADPT_STATUS		0xa3
#define LENOVO_EC_ADPT_PRESENT		BIT(7)
#define LENOVO_EC_BAT_STATUS		0xc1
#define LENOVO_EC_BAT_VOLTAGE		0xc6
#define LENOVO_EC_BAT_CURRENT		0xd2
#define LENOVO_EC_BAT_PRESENT		0xda
#define LENOVO_EC_BAT_ATTRIBUTES	0xc0
#define LENOVO_EC_BAT_ATTR_UNIT_IS_MA	BIT(1)
#define LENOVO_EC_BAT_REMAIN_CAPACITY	0xc2
#define LENOVO_EC_BAT_DESIGN_VOLTAGE	0xc8
#define LENOVO_EC_BAT_DESIGN_CAPACITY	0xca
#define LENOVO_EC_BAT_FULL_CAPACITY	0xcc
#define LENOVO_EC_BAT_FULL_FACTORY	0xd6
#define LENOVO_EC_BAT_FULL_REGISTER	0xdb

#define LENOVO_EC_REQUEST_NEXT_EVENT	0x84
#define LENOVO_EC_EVENT_USB		0x20
#define LENOVO_EC_EVENT_UCSI		0x21
#define LENOVO_EC_EVENT_HPD		0x22
#define LENOVO_EC_EVENT_BAT_STATUS	0x24
#define LENOVO_EC_EVENT_BAT_INFO	0x25
#define LENOVO_EC_EVENT_BAT_ADPT_STATUS	0x37

struct yoga_c630_ec {
	struct i2c_client *client;
	struct mutex lock;

	struct power_supply *adp_psy;
	struct power_supply *bat_psy;

	unsigned long last_status_update;

	bool adapter_online;

	bool unit_ma;

	unsigned int scale;

	bool bat_present;
	unsigned int bat_status;

	unsigned int design_capacity;
	unsigned int design_voltage;
	unsigned int full_charge_capacity;

	unsigned int capacity_now;
	unsigned int voltage_now;

	int rate_now;
};

static int yoga_c630_ec_request(struct yoga_c630_ec *ec, u8 *req, size_t req_len,
				u8 *resp, size_t resp_len)
{
	int ret;

	WARN_ON(!mutex_is_locked(&ec->lock));

	ret = i2c_smbus_write_i2c_block_data(ec->client, LENOVO_EC_REQUEST_REG,
					     req_len, req);
	if (ret < 0)
		return ret;

	return i2c_smbus_read_i2c_block_data(ec->client, LENOVO_EC_RESPONSE_REG,
					     resp_len, resp);
}

static int yoga_c630_ec_read8(struct yoga_c630_ec *ec, u8 addr)
{
	int ret;
	u8 val;

	ret = yoga_c630_ec_request(ec, (u8[]) { LENOVO_EC_READ_REG, addr }, 2,
				   &val, 1);
	if (ret < 0)
		return ret;

	return val;
}

static int yoga_c630_ec_read16(struct yoga_c630_ec *ec, u8 addr)
{
	int ret;
	u8 msb;
	u8 lsb;

	ret = yoga_c630_ec_request(ec, (u8[]) { LENOVO_EC_READ_REG, addr }, 2,
				   &lsb, 1);
	if (ret < 0)
		return ret;

	ret = yoga_c630_ec_request(ec, (u8[]) { LENOVO_EC_READ_REG, addr + 1 }, 2,
				   &msb, 1);
	if (ret < 0)
		return ret;

	return msb << 8 | lsb;
}

static void yoga_c630_ec_update_bat_info(struct yoga_c630_ec *ec)
{
	int val;

	mutex_lock(&ec->lock);

	val = yoga_c630_ec_read8(ec, LENOVO_EC_BAT_PRESENT);
	if (val < 0)
		goto out_unlock;
	ec->bat_present = !!(val & BIT(0));
	if (!ec->bat_present)
		goto out_unlock;

	val = yoga_c630_ec_read8(ec, LENOVO_EC_BAT_ATTRIBUTES);
	if (val < 0)
		goto out_unlock;
	ec->unit_ma = val & LENOVO_EC_BAT_ATTR_UNIT_IS_MA;
	if (!ec->unit_ma)
		ec->scale = 1000;
	else
		ec->scale = 1;

	val = yoga_c630_ec_read16(ec, LENOVO_EC_BAT_DESIGN_CAPACITY);
	if (val < 0)
		goto out_unlock;
	ec->design_capacity = val * ec->scale;

	msleep(50);

	val = yoga_c630_ec_read16(ec, LENOVO_EC_BAT_DESIGN_VOLTAGE);
	if (val < 0)
		goto out_unlock;
	ec->design_voltage = val;

	msleep(50);

	val = yoga_c630_ec_read8(ec, LENOVO_EC_BAT_FULL_REGISTER);
	if (val < 0)
		goto out_unlock;
	if (val & BIT(0))
		val = yoga_c630_ec_read16(ec, LENOVO_EC_BAT_FULL_FACTORY);
	else
		val = yoga_c630_ec_read16(ec, LENOVO_EC_BAT_FULL_CAPACITY);
	if (val < 0)
		goto out_unlock;
	ec->full_charge_capacity = val * ec->scale;

out_unlock:
	mutex_unlock(&ec->lock);
}

static void yoga_c630_ec_maybe_update_bat_status(struct yoga_c630_ec *ec)
{
	int val = 0;

	mutex_lock(&ec->lock);

	if (time_before(jiffies, ec->last_status_update + LENOVO_EC_CACHE_TIME))
		goto out_unlock;

	val = yoga_c630_ec_read8(ec, LENOVO_EC_BAT_STATUS);
	if (val < 0)
		goto out_unlock;
	ec->bat_status = val;

	val = yoga_c630_ec_read16(ec, LENOVO_EC_BAT_REMAIN_CAPACITY);
	if (val < 0)
		goto out_unlock;
	ec->capacity_now = val * ec->scale;

	val = yoga_c630_ec_read16(ec, LENOVO_EC_BAT_VOLTAGE);
	if (val < 0)
		goto out_unlock;
	ec->voltage_now = val;

	val = yoga_c630_ec_read16(ec, LENOVO_EC_BAT_CURRENT);
	if (val < 0)
		goto out_unlock;
	ec->rate_now = sign_extend32(val, 15) * ec->scale;

	if (ec->unit_ma)
		ec->rate_now = ec->rate_now * ec->voltage_now / 1000;

	ec->last_status_update = jiffies;

out_unlock:
	mutex_unlock(&ec->lock);
}

static int yoga_c630_ec_update_adapter_status(struct yoga_c630_ec *ec)
{
	int val;

	mutex_lock(&ec->lock);

	val = yoga_c630_ec_read8(ec, LENOVO_EC_ADPT_STATUS);
	if (val > 0)
		ec->adapter_online = FIELD_GET(LENOVO_EC_ADPT_PRESENT, val);

	mutex_unlock(&ec->lock);

	return val;
}

static bool yoga_c630_ec_is_charged(struct yoga_c630_ec *ec)
{
	if (ec->bat_status != 0)
		return false;

	if (ec->full_charge_capacity == ec->capacity_now)
		return true;

	if (ec->design_capacity == ec->capacity_now)
		return true;

	return false;
}

static int yoga_c630_ec_bat_get_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	struct yoga_c630_ec *ec = power_supply_get_drvdata(psy);
	int rc = 0;

	if (ec->bat_present)
		yoga_c630_ec_maybe_update_bat_status(ec);
	else if (psp != POWER_SUPPLY_PROP_PRESENT)
		return -ENODEV;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (ec->bat_status & BIT(0))
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (ec->bat_status & BIT(1))
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else if (yoga_c630_ec_is_charged(ec))
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = ec->bat_present;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = ec->design_voltage;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = ec->design_capacity;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = ec->full_charge_capacity;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = ec->capacity_now;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = ec->rate_now;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = ec->voltage_now;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "PABAS0241231";
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = "Compal";
		break;
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static enum power_supply_property yoga_c630_ec_bat_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

static const struct power_supply_desc yoga_c630_ec_bat_psy_desc = {
	.name = "yoga-c630-battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = yoga_c630_ec_bat_properties,
	.num_properties = ARRAY_SIZE(yoga_c630_ec_bat_properties),
	.get_property = yoga_c630_ec_bat_get_property,
};

static int yoga_c630_ec_adpt_get_property(struct power_supply *psy,
					  enum power_supply_property psp,
					  union power_supply_propval *val)
{
	struct yoga_c630_ec *ec = power_supply_get_drvdata(psy);
	int rc = 0;

	yoga_c630_ec_update_adapter_status(ec);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = ec->adapter_online;
		break;
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static enum power_supply_property yoga_c630_ec_adpt_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static const struct power_supply_desc yoga_c630_ec_adpt_psy_desc = {
	.name = "yoga-c630-adapter",
	.type = POWER_SUPPLY_TYPE_USB_TYPE_C,
	.properties = yoga_c630_ec_adpt_properties,
	.num_properties = ARRAY_SIZE(yoga_c630_ec_adpt_properties),
	.get_property = yoga_c630_ec_adpt_get_property,
};

static int yoga_c630_ec_query_usb_event(struct yoga_c630_ec *ec)
{
	struct device *dev = &ec->client->dev;
	int event;

	mutex_lock(&ec->lock);
	event = yoga_c630_ec_read8(ec, LENOVO_EC_QUERY_USB_EVENT);
	mutex_unlock(&ec->lock);
	if (event < 0) {
		dev_err(dev, "unable to query USB event\n");
		return event;
	}

	/* FIXME: handle the returned event to set the Type-C properties */

	return 0;
}

static irqreturn_t yoga_c630_ec_intr(int irq, void *data)
{
	struct yoga_c630_ec *ec = data;
	u8 event;
	int ret;

	mutex_lock(&ec->lock);
	ret = yoga_c630_ec_request(ec, (u8[]){ LENOVO_EC_REQUEST_NEXT_EVENT }, 1,
				   &event, 1);
	mutex_unlock(&ec->lock);
	if (ret < 0)
		return IRQ_HANDLED;

	switch (event) {
	case LENOVO_EC_EVENT_USB:
	case LENOVO_EC_EVENT_HPD:
		yoga_c630_ec_query_usb_event(ec);
		break;
	case LENOVO_EC_EVENT_BAT_INFO:
		yoga_c630_ec_update_bat_info(ec);
		break;
	case LENOVO_EC_EVENT_BAT_ADPT_STATUS:
		power_supply_changed(ec->adp_psy);
		fallthrough;
	case LENOVO_EC_EVENT_BAT_STATUS:
		power_supply_changed(ec->bat_psy);
		break;
	}

	return IRQ_HANDLED;
}

static int yoga_c630_ec_probe(struct i2c_client *client)
{
	struct power_supply_config adp_cfg = {};
	struct power_supply_config bat_cfg = {};
	struct yoga_c630_ec *ec;
	struct device *dev = &client->dev;
	int ret;

	ec = devm_kzalloc(dev, sizeof(*ec), GFP_KERNEL);
	if (!ec)
		return -ENOMEM;

	mutex_init(&ec->lock);
	ec->client = client;

	adp_cfg.drv_data = ec;
	adp_cfg.of_node = client->dev.of_node;
	ec->adp_psy = devm_power_supply_register(dev, &yoga_c630_ec_adpt_psy_desc, &adp_cfg);
	if (IS_ERR(ec->adp_psy)) {
		dev_err(dev, "failed to register AC adapter supply\n");
		return PTR_ERR(ec->adp_psy);
	}

	bat_cfg.drv_data = ec;
	bat_cfg.of_node = client->dev.of_node;
	ec->bat_psy = devm_power_supply_register(dev, &yoga_c630_ec_bat_psy_desc, &bat_cfg);
	if (IS_ERR(ec->bat_psy)) {
		dev_err(dev, "failed to register battery supply\n");
		return PTR_ERR(ec->bat_psy);
	}

	ret = devm_request_threaded_irq(dev, client->irq,
					NULL, yoga_c630_ec_intr,
					IRQF_ONESHOT, "yoga_c630_ec", ec);
	if (ret < 0)
		return dev_err_probe(dev, ret, "unable to request irq\n");

	yoga_c630_ec_update_bat_info(ec);

	return 0;
}

static const struct of_device_id yoga_c630_ec_of_match[] = {
	{ .compatible = "lenovo,yoga-c630-ec" },
	{}
};
MODULE_DEVICE_TABLE(of, yoga_c630_ec_of_match);

static const struct i2c_device_id yoga_c630_ec_i2c_id_table[] = {
	{ "yoga-c630-ec", },
	{}
};
MODULE_DEVICE_TABLE(i2c, yoga_c630_ec_i2c_id_table);

static struct i2c_driver yoga_c630_ec_i2c_driver = {
	.driver = {
		.name = "yoga-c630-ec",
		.of_match_table = yoga_c630_ec_of_match,
	},
	.probe = yoga_c630_ec_probe,
	.id_table = yoga_c630_ec_i2c_id_table,
};
module_i2c_driver(yoga_c630_ec_i2c_driver);

MODULE_DESCRIPTION("Lenovo Yoga C630 EC driver");
MODULE_LICENSE("GPL");
