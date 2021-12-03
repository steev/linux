// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2019-2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2021, Linaro Ltd
 */

#include <linux/auxiliary_bus.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <drm/drm_connector.h>

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
	struct fwnode_handle *dp_fwnode;

	struct work_struct work;

	enum typec_orientation orientation;
	u16 svid;
	u8 dp_data;
};

struct pmic_glink_altmode {
	struct device *dev;

	struct completion pan_ack;
	struct pmic_glink_client *client;

	struct pmic_glink_altmode_port ports[PMIC_GLINK_MAX_PORTS];
	unsigned int num_ports;
};

static int pmic_glink_altmode_write(struct pmic_glink_altmode *altmode, u32 cmd, u32 arg)
{
	struct usbc_write_req req = {};

	req.hdr.owner = PMIC_GLINK_OWNER_USBC_PAN;
	req.hdr.type = PMIC_GLINK_REQ_RESP;
	req.hdr.opcode = USBC_CMD_WRITE_REQ;
	req.cmd = cmd;
	req.arg = arg;

	return pmic_glink_send(altmode->client, &req, sizeof(req));
}

static int pmic_glink_altmode_enable(struct pmic_glink_altmode *altmode)
{
	unsigned long left;
	int ret;

	ret = pmic_glink_altmode_write(altmode, ALTMODE_PAN_EN, 0);
	if (ret)
		return ret;

	left = wait_for_completion_timeout(&altmode->pan_ack, 5 * HZ);
	if (!left) {
		dev_err(altmode->dev, "timeout waiting for pan enable ack\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static int pmic_glink_altmode_ack(struct pmic_glink_altmode *altmode, int port)
{
	unsigned long left;
	int ret;

	ret = pmic_glink_altmode_write(altmode, ALTMODE_PAN_ACK, port);
	if (ret)
		return ret;

	left = wait_for_completion_timeout(&altmode->pan_ack, 5 * HZ);
	if (!left) {
		dev_err(altmode->dev, "timeout waiting for pan enable ack\n");
		return -ETIMEDOUT;
	}

	return 0;
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
	u8 dpam_state;
	bool hpd_state;
	bool hpd_irq;

	dpam_state = alt_port->dp_data & 0x3f;
	hpd_state = alt_port->dp_data & BIT(6);
	hpd_irq = alt_port->dp_data & BIT(7);

	typec_switch_set(alt_port->typec_switch, alt_port->orientation);

	if (alt_port->svid == USB_TYPEC_DP_SID) {
		pmic_glink_altmode_enable_dp(altmode, alt_port, dpam_state,
					     hpd_state, hpd_irq);
	} else {
		pmic_glink_altmode_enable_usb(altmode, alt_port);
	}

	// drm_connector_oob_hotplug_event(alt_port->dp_fwnode, hpd_state);

	pmic_glink_altmode_ack(altmode, alt_port->index);
};

static void pmic_glink_altmode_callback(const void *data, size_t len, void *priv)
{
	struct pmic_glink_altmode_port *alt_port;
	struct pmic_glink_altmode *altmode = priv;
	const struct usbc_notify_ind_msg *notify;
	const struct pmic_glink_hdr *hdr = data;
	u16 opcode;
	u16 svid;

	opcode = le32_to_cpu(hdr->opcode) & 0xff;
	svid = le32_to_cpu(hdr->opcode) >> 16;
	switch (opcode) {
	case USBC_CMD_WRITE_REQ:
		complete(&altmode->pan_ack);
		break;
	case USBC_NOTIFY_IND:
		notify = data;

		alt_port = &altmode->ports[0];

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

		alt_port->svid = svid;
		alt_port->dp_data = notify->payload[8];
		schedule_work(&alt_port->work);

		break;
	}
}

static void pmic_glink_altmode_put_dp_fwnode(void *data)
{
	fwnode_handle_put(data);
}

static void pmic_glink_altmode_put_mux(void *data)
{
	typec_mux_put(data);
}

static void pmic_glink_altmode_put_switch(void *data)
{
	typec_switch_put(data);
}

static int pmic_glink_altmode_probe(struct auxiliary_device *adev,
				    const struct auxiliary_device_id *id)
{
	struct pmic_glink_altmode_port *alt_port;
	struct pmic_glink_altmode *altmode;
	struct typec_altmode_desc mux_desc = {};
	struct fwnode_handle *fwnode;
	struct fwnode_handle *ep;
	struct device *dev = &adev->dev;
	unsigned int port = 0;
	int ret;

	altmode = devm_kzalloc(dev, sizeof(*altmode), GFP_KERNEL);
	if (!altmode)
		return -ENOMEM;

	altmode->dev = dev;

	init_completion(&altmode->pan_ack);

	device_for_each_child_node(dev, fwnode) {
		if (port >= ARRAY_SIZE(altmode->ports)) {
			dev_err(dev, "too many connectors, ignoring\n");
			continue;
		}

		alt_port = &altmode->ports[port];
		alt_port->altmode = altmode;
		alt_port->index = port;
		INIT_WORK(&alt_port->work, pmic_glink_altmode_worker);

		ep = fwnode_graph_get_endpoint_by_id(fwnode, 0, 0, 0);
		if (ep) {
			alt_port->dp_fwnode = fwnode_graph_get_remote_port_parent(ep);
			fwnode_handle_put(ep);
		} else {
			dev_dbg(dev, "no displayport reference\n");
		}

		ret = devm_add_action_or_reset(dev, pmic_glink_altmode_put_dp_fwnode,
					       alt_port->dp_fwnode);
		if (ret)
			return ret;

		alt_port->dp_alt.svid = USB_TYPEC_DP_SID;
		alt_port->dp_alt.mode = USB_TYPEC_DP_MODE;
		alt_port->dp_alt.active = 1;

		alt_port->state.alt = NULL;
		alt_port->state.mode = TYPEC_STATE_USB;
		alt_port->state.data = NULL;

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

		port++;
	}

	altmode->num_ports = port;

	altmode->client = devm_pmic_glink_register_client(dev,
							 PMIC_GLINK_OWNER_USBC_PAN,
							 pmic_glink_altmode_callback,
							 altmode);
	if (IS_ERR(altmode->client))
		return PTR_ERR(altmode->client);

	ret = pmic_glink_altmode_enable(altmode);
	if (ret)
		return ret;

	dev_set_drvdata(dev, altmode);

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
