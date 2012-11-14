/*
 * Platform data for the chipidea USB dual role controller
 */

#ifndef __LINUX_USB_CHIPIDEA_H
#define __LINUX_USB_CHIPIDEA_H

#include <linux/usb/otg.h>

struct ci13xxx;
struct ci13xxx_platform_data {
	const char	*name;
	/* offset of the capability registers */
	uintptr_t	 capoffset;
	unsigned	 power_budget;
	struct usb_phy	*phy;
	unsigned long	 flags;
#define CI13XXX_REGS_SHARED		BIT(0)
#define CI13XXX_REQUIRE_TRANSCEIVER	BIT(1)
#define CI13XXX_DISABLE_STREAMING	BIT(3)
#define CI13XXX_DR_MODE_HOST		BIT(4)
#define CI13XXX_DR_MODE_PERIPHERAL	BIT(5)
#define CI13XXX_DR_MODE_MASK \
	(CI13XXX_DR_MODE_HOST | CI13XXX_DR_MODE_PERIPHERAL)

#define CI13XXX_CONTROLLER_RESET_EVENT		0
#define CI13XXX_CONTROLLER_STOPPED_EVENT	1
	void	(*notify_event) (struct ci13xxx *ci, unsigned event);
};

/* Default offset of capability registers */
#define DEF_CAPOFFSET		0x100

/* Add ci13xxx device */
struct platform_device *ci13xxx_add_device(struct device *dev,
			struct resource *res, int nres,
			struct ci13xxx_platform_data *platdata);
/* Remove ci13xxx device */
void ci13xxx_remove_device(struct platform_device *pdev);

/* Parse of-tree "dr_mode" property */
void ci13xxx_get_dr_mode(struct device_node *of_node, struct ci13xxx_platform_data *pdata);

#endif
