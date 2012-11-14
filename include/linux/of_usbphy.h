/*
 * OF helpers for usb devices.
 *
 * This file is released under the GPLv2
 */

#ifndef __LINUX_OF_USBPHY_H
#define __LINUX_OF_USBPHY_H

#ifdef CONFIG_OF_USBPHY
#include <linux/of.h>
extern const int of_get_usbphy_mode(struct device_node *np);
#endif

#endif /* __LINUX_OF_USBPHY_H */
