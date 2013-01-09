/*
 * host.c - ChipIdea USB host controller driver
 *
 * Copyright (c) 2012 Intel Corporation
 *
 * Author: Alexander Shishkin
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/usb/chipidea.h>
#include <linux/usb/ulpi.h>

#include "../host/ehci.h"

#include "ci.h"
#include "bits.h"
#include "host.h"

void hw_portsc_configure(struct ci13xxx *ci);

static struct hc_driver __read_mostly ci_ehci_hc_driver;

static irqreturn_t host_irq(struct ci13xxx *ci)
{
	return usb_hcd_irq(ci->irq, ci->hcd);
}

static int host_start(struct ci13xxx *ci)
{
	struct usb_hcd *hcd;
	struct ehci_hcd *ehci;
	int ret;
	u32 flags;

	if (usb_disabled())
		return -ENODEV;

	hw_portsc_configure(ci);

	mdelay(10);


	hcd = usb_create_hcd(&ci_ehci_hc_driver, ci->dev, dev_name(ci->dev));
	if (!hcd)
		return -ENOMEM;

	dev_set_drvdata(ci->dev, ci);
	hcd->rsrc_start = ci->hw_bank.phys;
	hcd->rsrc_len = ci->hw_bank.size;
	hcd->regs = ci->hw_bank.abs;
	hcd->has_tt = 1;

	hcd->power_budget = ci->platdata->power_budget;
	hcd->phy = ci->transceiver;

	ehci = hcd_to_ehci(hcd);
	ehci->caps = ci->hw_bank.cap;
	ehci->has_hostpc = ci->hw_bank.lpm;

	if (ci->otg) {
		pr_err("%s: otg_set_vbus true\n", __func__);
		otg_set_vbus(ci->otg, true);
	}

	ret = usb_add_hcd(hcd, 0, 0);
	if (ret) {
		pr_err("%s: add_hcd failed, putting hcd\n", __func__);
		usb_put_hcd(hcd);
	} else
		ci->hcd = hcd;

	/*
	 * EfikaMX Smartbook has a hardware quirk where host VBUS is
	 * actually routed to hub VBUS_DET and VBUS is supplied
	 * elsewhere. Without VBUS_DET the hub will not turn on the
	 * internal USB logic and therefore not work. Since the PHY
	 * is not connected to the VBUS drive logic, the only way
	 * to generate the PHY VBUS signal input to VBUS_DET on the
	 * hub is to connect the 100ohm resistor between the PHY 3.3V
	 * supply and it's VBUS output giving us an active high GPIO
	 * in all essence.
	 *
	 * Also, it seems, on the Smarttop despite being as per SMSC
	 * recommendations and compliant with all USB specifications,
	 * if CHRGVBUS is not set then the USB hub behind UH1 never
	 * powers up (is this a race condition?)
	 */
	if (ci->otg && (ci->platdata->flags & CI13XXX_PORTSC_PTS_ULPI) &&
		(ci->platdata->flags & CI13XXX_CHRGVBUS_IS_VBUS_DET)) {

		pr_err("%s: hacking CHRGVBUS\n", __func__);
		flags = usb_phy_io_read(ci->transceiver, ULPI_OTG_CTRL);
		flags |= ULPI_OTG_CTRL_CHRGVBUS;
		ret = usb_phy_io_write(ci->transceiver, flags, ULPI_OTG_CTRL);
	}

	pr_err("%s returning (ci hcd 0x%08x)\n", __func__, ci->hcd);
	return ret;
}

static void host_stop(struct ci13xxx *ci)
{
	struct usb_hcd *hcd = ci->hcd;

	pr_err("%s\n", __func__);

	usb_remove_hcd(hcd);
	usb_put_hcd(hcd);
}

int ci_hdrc_host_init(struct ci13xxx *ci)
{
	struct ci_role_driver *rdrv;

	if (!hw_read(ci, CAP_DCCPARAMS, DCCPARAMS_HC))
		return -ENXIO;

	rdrv = devm_kzalloc(ci->dev, sizeof(struct ci_role_driver), GFP_KERNEL);
	if (!rdrv)
		return -ENOMEM;

	rdrv->start	= host_start;
	rdrv->stop	= host_stop;
	rdrv->irq	= host_irq;
	rdrv->name	= "host";
	ci->roles[CI_ROLE_HOST] = rdrv;

	ehci_init_driver(&ci_ehci_hc_driver, NULL);

	pr_err("%s\n", __func__);

	return 0;
}
