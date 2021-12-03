// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2019-2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2021, Linaro Ltd
 */

#include <linux/auxiliary_bus.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/soc/qcom/pdr.h>
#include <drm/drm_bridge.h>

#include <linux/usb/typec_altmode.h>
#include <linux/usb/typec_dp.h>
#include <linux/usb/typec_mux.h>

#include <linux/soc/qcom/pmic_glink.h>

#define PMIC_GLINK_MAX_PORTS	2

#define USBC_CMD_WRITE_REQ      0x15
#define USBC_NOTIFY_IND		0x16

#define ALTMODE_PAN_EN		0x10
#define ALTMODE_PAN_ACK		0x11

struct usbc_write_req {
	struct pmic_glink_hdr   hdr;
	u32 cmd;
	u32 arg;
	u32                     reserved;
};

#define NOTIFY_PAYLOAD_SIZE 16
struct usbc_notify_ind_msg {
	struct pmic_glink_hdr hdr;
	char payload[NOTIFY_PAYLOAD_SIZE];
	u32 reserved;
};

enum pmic_glink_altmode_pin_assignment {
	DPAM_HPD_OUT,
	DPAM_HPD_A,
	DPAM_HPD_B,
	DPAM_HPD_C,
	DPAM_HPD_D,
	DPAM_HPD_E,
	DPAM_HPD_F,
};

struct pmic_glink_altmode;

struct pmic_glink_altmode_port {
	struct pmic_glink_altmode *altmode;
	unsigned int index;

	struct typec_switch *typec_switch;
	struct typec_mux *typec_mux;
	struct typec_mux_state state;
	struct typec_altmode dp_alt;

	struct work_struct work;

	struct drm_bridge bridge;

	enum typec_orientation orientation;
	u16 svid;
	u8 dp_data;
	u8 dpam_state;
	u8 hpd_state;
	u8 hpd_irq;
};

struct pmic_glink_altmode {
	struct device *dev;

	/* To synchronize WRITE_REQ acks */
	struct mutex lock;

	struct completion pan_ack;
	struct pmic_glink_client *client;

	struct pmic_glink_altmode_port ports[PMIC_GLINK_MAX_PORTS];
};

static int pmic_glink_altmode_request(struct pmic_glink_altmode *altmode, u32 cmd, u32 arg)
{
	struct usbc_write_req req = {};
	unsigned long left;
	int ret;

	/*
	 * The USBC_CMD_WRITE_REQ ack doesn't identify the request, so wait for
	 * one ack at a time.
	 */
	mutex_lock(&altmode->lock);

	req.hdr.owner = PMIC_GLINK_OWNER_USBC_PAN;
	req.hdr.type = PMIC_GLINK_REQ_RESP;
	req.hdr.opcode = USBC_CMD_WRITE_REQ;
	req.cmd = cmd;
	req.arg = arg;

	ret = pmic_glink_send(altmode->client, &req, sizeof(req));
	if (ret) {
		dev_err(altmode->dev, "failed to send write request\n");
		goto out_unlock;
	}

	left = wait_for_completion_timeout(&altmode->pan_ack, 5 * HZ);
	if (!left) {
		dev_err(altmode->dev, "timeout waiting for write request ack\n");
		ret = -ETIMEDOUT;
	}

out_unlock:
	mutex_unlock(&altmode->lock);
	return ret;
}

static void pmic_glink_altmode_enable_dp(struct pmic_glink_altmode *altmode,
					 struct pmic_glink_altmode_port *port,
					 u8 dpam_state, bool hpd_state,
					 bool hpd_irq)
{
	struct typec_displayport_data dp_data = {};
	unsigned int mode;
	int ret;

	mode = dpam_state - DPAM_HPD_A;

	dp_data.status = DP_STATUS_ENABLED;
	if (hpd_state)
		dp_data.status |= DP_STATUS_HPD_STATE;
	if (hpd_irq)
		dp_data.status |= DP_STATUS_IRQ_HPD;
	dp_data.conf = DP_CONF_SET_PIN_ASSIGN(mode);

	port->state.alt = &port->dp_alt;
	port->state.data = &dp_data;
	port->state.mode = TYPEC_MODAL_STATE(mode);

	ret = typec_mux_set(port->typec_mux, &port->state);
	if (ret)
		dev_err(altmode->dev, "failed to switch mux to DP\n");
}

static void pmic_glink_altmode_enable_usb(struct pmic_glink_altmode *altmode,
					  struct pmic_glink_altmode_port *port)
{
	int ret;

	port->state.alt = NULL;
	port->state.data = NULL;
	port->state.mode = TYPEC_STATE_USB;

	ret = typec_mux_set(port->typec_mux, &port->state);
	if (ret)
		dev_err(altmode->dev, "failed to switch mux to USB\n");
}

static void pmic_glink_altmode_worker(struct work_struct *work)
{
	struct pmic_glink_altmode_port *alt_port = container_of(work, struct pmic_glink_altmode_port, work);
	struct pmic_glink_altmode *altmode = alt_port->altmode;

	typec_switch_set(alt_port->typec_switch, alt_port->orientation);

	if (alt_port->svid == USB_TYPEC_DP_SID) {
		pmic_glink_altmode_enable_dp(altmode, alt_port, alt_port->dpam_state,
					     alt_port->hpd_state, alt_port->hpd_irq);
	} else {
		pmic_glink_altmode_enable_usb(altmode, alt_port);
	}

	drm_bridge_hpd_notify(&alt_port->bridge, alt_port->hpd_state ? connector_status_connected : connector_status_disconnected);

	pmic_glink_altmode_request(altmode, ALTMODE_PAN_ACK, alt_port->index);
};

static void pmic_glink_altmode_callback(const void *data, size_t len, void *priv)
{
	struct pmic_glink_altmode_port *alt_port;
	struct pmic_glink_altmode *altmode = priv;
	const struct usbc_notify_ind_msg *notify;
	const struct pmic_glink_hdr *hdr = data;
	unsigned int port;
	u16 opcode;
	u16 svid;
	u8 dp_data;

	opcode = le32_to_cpu(hdr->opcode) & 0xff;
	svid = le32_to_cpu(hdr->opcode) >> 16;

	switch (opcode) {
	case USBC_CMD_WRITE_REQ:
		complete(&altmode->pan_ack);
		break;
	case USBC_NOTIFY_IND:
		if (len < sizeof(*notify)) {
			dev_warn(altmode->dev, "truncated USBC_NOTIFY_IND\n");
			break;
		}

		notify = data;

		port = notify->payload[0];

		if (!altmode->ports[port].altmode) {
			dev_dbg(altmode->dev, "notification on undefined port %d\n", port);
			break;
		}

		alt_port = &altmode->ports[port];

		switch (notify->payload[1]) {
		case 0:
			alt_port->orientation = TYPEC_ORIENTATION_NORMAL;
			break;
		case 1:
			alt_port->orientation = TYPEC_ORIENTATION_REVERSE;
			break;
		case 2:
		default:
			alt_port->orientation = TYPEC_ORIENTATION_NONE;
			break;
		}

		dp_data = notify->payload[8];

		alt_port->svid = svid;
		alt_port->dpam_state = dp_data & 0x3f;
		alt_port->hpd_state = dp_data & BIT(6);
		alt_port->hpd_irq = dp_data & BIT(7);
		schedule_work(&alt_port->work);
		break;
	}
}

static int pmic_glink_altmode_attach(struct drm_bridge *bridge,
				     enum drm_bridge_attach_flags flags)
{
	return flags & DRM_BRIDGE_ATTACH_NO_CONNECTOR ? 0 : -EINVAL;
}

static const struct drm_bridge_funcs pmic_glink_altmode_bridge_funcs = {
	.attach = pmic_glink_altmode_attach,
};

static void pmic_glink_altmode_put_mux(void *data)
{
	typec_mux_put(data);
}

static void pmic_glink_altmode_put_switch(void *data)
{
	typec_switch_put(data);
}

static void pmic_glink_altmode_pdr_notify(void *priv, int state)
{
	struct pmic_glink_altmode *altmode = priv;
	int ret;

	if (state == SERVREG_SERVICE_STATE_UP) {
		ret = pmic_glink_altmode_request(altmode, ALTMODE_PAN_EN, 0);
		if (ret)
			dev_err(altmode->dev, "failed to request altmode notifications\n");
	}
}

static int pmic_glink_altmode_probe(struct auxiliary_device *adev,
				    const struct auxiliary_device_id *id)
{
	struct pmic_glink_altmode_port *alt_port;
	struct pmic_glink_altmode *altmode;
	struct typec_altmode_desc mux_desc = {};
	struct fwnode_handle *fwnode;
	struct device *dev = &adev->dev;
	u32 port;
	int ret;

	altmode = devm_kzalloc(dev, sizeof(*altmode), GFP_KERNEL);
	if (!altmode)
		return -ENOMEM;

	altmode->dev = dev;

	init_completion(&altmode->pan_ack);
	mutex_init(&altmode->lock);

	device_for_each_child_node(dev, fwnode) {
		ret = fwnode_property_read_u32(fwnode, "reg", &port);
		if (ret < 0) {
			dev_err(dev, "missing reg property of %pOFn\n", fwnode);
			return ret;
		}

		if (port >= ARRAY_SIZE(altmode->ports)) {
			dev_warn(dev, "invalid connector number, ignoring\n");
			continue;
		}

		if (altmode->ports[port].altmode) {
			dev_err(dev, "multiple connector definition for port %u\n", port);
			return -EINVAL;
		}

		alt_port = &altmode->ports[port];
		alt_port->altmode = altmode;
		alt_port->index = port;
		INIT_WORK(&alt_port->work, pmic_glink_altmode_worker);

		alt_port->bridge.funcs = &pmic_glink_altmode_bridge_funcs;
		alt_port->bridge.of_node = to_of_node(fwnode);
		alt_port->bridge.ops = DRM_BRIDGE_OP_HPD;
		alt_port->bridge.type = DRM_MODE_CONNECTOR_USB;

		ret = devm_drm_bridge_add(dev, &alt_port->bridge);
		if (ret)
			return ret;
		dev_err(dev, "registered bridge for port%d: %pOF\n", port, alt_port->bridge.of_node);

		alt_port->dp_alt.svid = USB_TYPEC_DP_SID;
		alt_port->dp_alt.mode = USB_TYPEC_DP_MODE;
		alt_port->dp_alt.active = 1;

		mux_desc.svid = USB_TYPEC_DP_SID;
		mux_desc.mode = USB_TYPEC_DP_MODE;
		alt_port->typec_mux = fwnode_typec_mux_get(fwnode, &mux_desc);
		if (IS_ERR(alt_port->typec_mux))
			return dev_err_probe(dev, PTR_ERR(alt_port->typec_mux),
					     "failed to acquire mode-switch for port: %d\n",
					     port);

		ret = devm_add_action_or_reset(dev, pmic_glink_altmode_put_mux,
					       alt_port->typec_mux);
		if (ret)
			return ret;

		alt_port->typec_switch = fwnode_typec_switch_get(fwnode);
		if (IS_ERR(alt_port->typec_switch))
			return dev_err_probe(dev, PTR_ERR(alt_port->typec_switch),
					     "failed to acquire orientation-switch for port: %d\n",
					     port);

		ret = devm_add_action_or_reset(dev, pmic_glink_altmode_put_switch,
					       alt_port->typec_switch);
		if (ret)
			return ret;
	}

	altmode->client = devm_pmic_glink_register_client(dev,
							 PMIC_GLINK_OWNER_USBC_PAN,
							 pmic_glink_altmode_callback,
							 pmic_glink_altmode_pdr_notify,
							 altmode);
	if (IS_ERR(altmode->client))
		return PTR_ERR(altmode->client);

	dev_set_drvdata(dev, altmode);

	dev_err(dev, "SUCCESS!\n");

	return 0;
}

static const struct auxiliary_device_id pmic_glink_altmode_id_table[] = {
	{ .name = "pmic_glink.altmode", },
	{},
};
MODULE_DEVICE_TABLE(auxiliary, pmic_glink_altmode_id_table);

static struct auxiliary_driver pmic_glink_altmode_driver = {
	.name = "pmic_glink_altmode",
	.probe = pmic_glink_altmode_probe,
	.id_table = pmic_glink_altmode_id_table,
};

module_auxiliary_driver(pmic_glink_altmode_driver);

MODULE_DESCRIPTION("Qualcomm PMIC GLINK Altmode driver");
MODULE_LICENSE("GPL v2");
