/*
 * Copyright (c) 2018 Linaro Ltd.
 */
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/bitops.h>

#define SOME_CACHE_TIME		(10 * HZ)

#define ADPT	0xa3
#define BAM0	0xc0
#define BST0	0xc1 /* Battery Status */
#define BRC0	0xc2 /* Battery Remaining Capacity */
#define BRC1	0xc3
#define BPV0	0xc6 /* Battery Present Voltage */
#define BPV1	0xc7
#define BDV0	0xc8 /* Battery Design Voltage */
#define BDV1	0xc9
#define BDC0	0xca /* Battery Design Capacity */
#define BDC1	0xcb
#define BFC0	0xcc /* Battery Full Capacity */
#define BFC1	0xcd
#define BAC0	0xd2 /* Battery Active Current/Charge ? */
#define BAC1	0xd3
#define BFF0	0xd6
#define BFF1	0xd7
#define ORRF	0xdb

struct some_battery {
	struct i2c_client *client;
	struct mutex lock;

	unsigned long update_time;

	bool adapter_online;

	bool unit_ma;

	int basc;

	bool battery_present;
	int bat_status;

	unsigned int design_capacity;
	unsigned int design_voltage;
	unsigned int full_charge_capacity;

	unsigned int capacity_now;
	unsigned int voltage_now;

	int rate_now;
};

static int some_battery_read(struct some_battery *battery, u8 arg0)
{
	struct i2c_client *client = battery->client;
	struct i2c_msg req, resp[2];
	u8 read_cmd = 1;
	u8 ecr[6] = {0};
	u8 req_data[6] = { 0 };
	int ret;

	req_data[0] = 0x2;
	req_data[1] = 0xb0;
	req_data[2] = arg0;

	req.addr = client->addr;
	req.flags = client->flags;
	req.len = sizeof(req_data);
	req.buf = req_data;

	ret = i2c_transfer(client->adapter, &req, 1);
	if (ret < 0) {
		dev_err(&client->dev, "failed to issue read request\n");
		return ret;
	}

	resp[0].addr = client->addr;
	resp[0].flags = client->flags;
	resp[0].len = 1;
	resp[0].buf = &read_cmd;

	resp[1].addr = client->addr;
	resp[1].flags = client->flags | I2C_M_RD;
	resp[1].len = sizeof(ecr);
	resp[1].buf = ecr;

	ret = i2c_transfer(client->adapter, resp, 2);
	if (ret != 2) {
		dev_err(&client->dev, "failed to read response\n");
		return -EINVAL;
	}

	return ecr[0];
}

static bool some_battery_present(struct some_battery *battery)
{
	return battery->battery_present;
}

static int some_battery_update_info(struct some_battery *battery)
{
	int orrf;
	int msb;
	int lsb;
	int val;

	val = some_battery_read(battery, 0xda);
	if (val < 0)
		return val;

	battery->battery_present = !!(val & BIT(0));
	if (!battery->battery_present)
		return 0;

	val = some_battery_read(battery, BAM0);
	if (val < 0)
		return val;

	battery->unit_ma = val & BIT(1);
	if (!battery->unit_ma)
		battery->basc = 10;
	else
		battery->basc = 1;

	lsb = some_battery_read(battery, BDC0);
	msb = some_battery_read(battery, BDC1);
	if (lsb < 0 || msb < 0)
		return lsb < 0 ?  lsb : msb;

	battery->design_capacity = battery->basc * ((msb << 8) | lsb);

	msleep(50);

	lsb = some_battery_read(battery, BDV0);
	msb = some_battery_read(battery, BDV1);
	if (lsb < 0 || msb < 0)
		return lsb < 0 ? lsb : msb;

	battery->design_voltage = (msb << 8) | lsb;

	msleep(50);

	orrf = some_battery_read(battery, ORRF);
	if (orrf < 0)
		return orrf;

	if (orrf & BIT(0)) {
		lsb = some_battery_read(battery, BFF0);
		msb = some_battery_read(battery, BFF1);
	} else {
		lsb = some_battery_read(battery, BFC0);
		msb = some_battery_read(battery, BFC1);
	}

	battery->full_charge_capacity = ((msb << 8) | lsb) * battery->basc;

	return 0;;
}

static int some_battery_update_status(struct some_battery *battery)
{
	int val;
	int msb;
	int lsb;

	if (!battery->battery_present)
		return 0;

	val = some_battery_read(battery, BST0);
	if (val < 0)
		return val;

	battery->bat_status = val;

	lsb = some_battery_read(battery, BRC0);
	msb = some_battery_read(battery, BRC1);
	if (lsb < 0 || msb < 0)
		return lsb < 0 ? lsb : msb;

	battery->capacity_now = ((msb << 8) | lsb) * battery->basc;

	lsb = some_battery_read(battery, BPV0);
	msb = some_battery_read(battery, BPV1);
	if (lsb < 0 || msb < 0)
		return lsb < 0 ? lsb : msb;

	battery->voltage_now = (msb << 8) | lsb;

	lsb = some_battery_read(battery, BAC0);
	msb = some_battery_read(battery, BAC1);
	battery->rate_now = sign_extend32((msb << 8) | lsb, 15) * battery->basc;

	if (battery->unit_ma)
		battery->rate_now = battery->rate_now * battery->voltage_now / 1000;

	return 0;
}

static int some_battery_update_adapter(struct some_battery *battery)
{
	int val;

	val = some_battery_read(battery, ADPT);
	if (val < 0)
		return val;

	battery->adapter_online = !!(val & BIT(7));

	return 0;
}

static int some_battery_update(struct some_battery *battery)
{
	if (battery->update_time &&
	    time_before(jiffies, battery->update_time + SOME_CACHE_TIME))
		return 0;

	mutex_lock(&battery->lock);

	some_battery_update_info(battery);
	msleep(50);
	some_battery_update_status(battery);
	msleep(50);
	some_battery_update_adapter(battery);

	mutex_unlock(&battery->lock);

	battery->update_time = jiffies;

	return 0;
}

static bool some_battery_is_charged(struct some_battery *battery)
{
	if (battery->bat_status != 0)
		return false;

	if (battery->full_charge_capacity == battery->capacity_now)
		return true;

	if (battery->design_capacity == battery->capacity_now)
		return true;

	return false;
}

static int bat0_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct some_battery *battery = power_supply_get_drvdata(psy);
	int rc = 0;

	if (some_battery_present(battery))
		some_battery_update(battery);
	else if (psp != POWER_SUPPLY_PROP_PRESENT)
		return -ENODEV;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (battery->bat_status & BIT(0))
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (battery->bat_status & BIT(1))
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else if (some_battery_is_charged(battery))
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = battery->battery_present;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = battery->design_voltage;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = battery->design_capacity * 100;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = battery->full_charge_capacity * 100;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = battery->capacity_now * 100;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = battery->rate_now * 100;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = battery->voltage_now;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (battery->full_charge_capacity)
			val->intval = battery->capacity_now * 100 /
				      battery->full_charge_capacity;
		else
			val->intval = 0;
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
	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
		val->strval = "05072018";
		break;
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static enum power_supply_property bat0_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
};

static const struct power_supply_desc bat0_psy_desc = {
	.name = "some-battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = bat0_properties,
	.num_properties = ARRAY_SIZE(bat0_properties),
	.get_property = bat0_get_property,
};

static int adp_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct some_battery *battery = power_supply_get_drvdata(psy);
	int rc = 0;

	some_battery_update(battery);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = battery->adapter_online;
		break;
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static enum power_supply_property adp_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static const struct power_supply_desc adp_psy_desc = {
	.name = "some-adapter",
	.type = POWER_SUPPLY_TYPE_USB_TYPE_C,
	.properties = adp_properties,
	.num_properties = ARRAY_SIZE(adp_properties),
	.get_property = adp_get_property,
};

static int some_battery_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	struct power_supply_config adp_cfg = {};
	struct power_supply_config bat_cfg = {};
	struct some_battery *battery;
	struct power_supply *psy;

	battery = devm_kzalloc(&client->dev, sizeof(*battery), GFP_KERNEL);
	if (!battery)
		return -ENOMEM;

	mutex_init(&battery->lock);
	battery->client = client;

	/* Claim battery is present, to trigger at least one update */
	battery->battery_present = true;

	adp_cfg.drv_data = battery;
	adp_cfg.of_node = client->dev.of_node;
	psy = devm_power_supply_register(&client->dev, &adp_psy_desc, &adp_cfg);
	if (IS_ERR(psy)) {
		dev_err(&client->dev, "failed to register adapter\n");
		return PTR_ERR(psy);
	}

	bat_cfg.drv_data = battery;
	bat_cfg.of_node = client->dev.of_node;
	psy = devm_power_supply_register(&client->dev,
						   &bat0_psy_desc,
						   &bat_cfg);
	if (IS_ERR(psy)) {
		dev_err(&client->dev, "failed to register battery\n");
		return PTR_ERR(psy);
	}

	return 0;
}

static int some_battery_remove(struct i2c_client *client)
{
	return 0;
}

static const struct of_device_id some_battery_of_match[] = {
	{ .compatible = "some,battery" },
	{}
};
MODULE_DEVICE_TABLE(of, some_battery_of_match);

static const struct i2c_device_id some_battery_i2c_id_table[] = {
	{ "some-battery", },
	{}
};
MODULE_DEVICE_TABLE(i2c, some_battery_i2c_id_table);

static struct i2c_driver some_battery_i2c_driver = {
	.driver = {
		.name = "some-battery",
		.of_match_table = some_battery_of_match,
	},
	.probe = some_battery_probe,
	.remove = some_battery_remove,
	.id_table = some_battery_i2c_id_table,
};
module_i2c_driver(some_battery_i2c_driver);
MODULE_LICENSE("GPL v2");
