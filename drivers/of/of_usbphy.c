/*
 * OF helpers for network devices.
 *
 * This file is released under the GPLv2
 *
 * Initially copied out of drivers/of/of_net.c
 */
#include <linux/etherdevice.h>
#include <linux/kernel.h>
#include <linux/of_usbphy.h>
#include <linux/usb/phy.h>
#include <linux/export.h>

/**
 * It maps 'enum usb_phy_interface' found in include/linux/usb/phy.h
 * into the device tree binding of 'phy-mode', so that USB
 * device driver can get phy interface from device tree.
 */
static const char *usbphy_modes[] = {
	[USBPHY_INTERFACE_MODE_NA]	= "",
	[USBPHY_INTERFACE_MODE_UTMI]	= "utmi",
	[USBPHY_INTERFACE_MODE_UTMIW]	= "utmiw",
	[USBPHY_INTERFACE_MODE_ULPI]	= "ulpi",
	[USBPHY_INTERFACE_MODE_SERIAL]	= "fsls",
};

/**
 * of_get_phy_mode - Get phy mode for given device_node
 * @np:	Pointer to the given device_node
 *
 * The function gets phy interface string from property 'phy-mode',
 * and return its index in phy_modes table, or errno in error case.
 */
const int of_get_usbphy_mode(struct device_node *np)
{
	const char *pm;
	int err, i;

	err = of_property_read_string(np, "phy-mode", &pm);
	if (err < 0)
		return err;

	for (i = 0; i < ARRAY_SIZE(usbphy_modes); i++)
		if (!strcasecmp(pm, usbphy_modes[i]))
			return i;

	return -ENODEV;
}
EXPORT_SYMBOL_GPL(of_get_usbphy_mode);
