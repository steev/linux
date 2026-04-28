// SPDX-License-Identifier: GPL-2.0-only
/*
 * Thunderbolt driver - PCI NHI driver
 *
 * Copyright (c) 2014 Andreas Noever <andreas.noever@gmail.com>
 * Copyright (C) 2018, Intel Corporation
 */

#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/iommu.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/platform_data/x86/apple.h>
#include <linux/property.h>
#include <linux/string_helpers.h>

#include "nhi.h"
#include "nhi_regs.h"
#include "pci.h"
#include "tb.h"

struct tb_nhi_pci {
	struct tb_nhi nhi;
	struct ida msix_ida;
};

static inline struct tb_nhi_pci *nhi_to_pci(struct tb_nhi *nhi)
{
	return container_of(nhi, struct tb_nhi_pci, nhi);
}

static void nhi_pci_check_quirks(struct tb_nhi_pci *nhi_pci)
{
	struct tb_nhi *nhi = &nhi_pci->nhi;
	struct pci_dev *pdev = to_pci_dev(nhi->dev);

	if (pdev->vendor == PCI_VENDOR_ID_INTEL) {
		/*
		 * Intel hardware supports auto clear of the interrupt
		 * status register right after interrupt is being
		 * issued.
		 */
		nhi->quirks |= QUIRK_AUTO_CLEAR_INT;

		switch (pdev->device) {
		case PCI_DEVICE_ID_INTEL_FALCON_RIDGE_2C_NHI:
		case PCI_DEVICE_ID_INTEL_FALCON_RIDGE_4C_NHI:
			/*
			 * Falcon Ridge controller needs the end-to-end
			 * flow control workaround to avoid losing Rx
			 * packets when RING_FLAG_E2E is set.
			 */
			nhi->quirks |= QUIRK_E2E;
			break;
		}
	}
}

static int nhi_pci_check_iommu_pdev(struct pci_dev *pdev, void *data)
{
	if (!pdev->external_facing ||
	    !device_iommu_capable(&pdev->dev, IOMMU_CAP_PRE_BOOT_PROTECTION))
		return 0;
	*(bool *)data = true;
	return 1; /* Stop walking */
}

static void nhi_pci_check_iommu(struct tb_nhi_pci *nhi_pci)
{
	struct tb_nhi *nhi = &nhi_pci->nhi;
	struct pci_dev *pdev = to_pci_dev(nhi->dev);
	struct pci_bus *bus = pdev->bus;
	bool port_ok = false;

	/*
	 * Ideally what we'd do here is grab every PCI device that
	 * represents a tunnelling adapter for this NHI and check their
	 * status directly, but unfortunately USB4 seems to make it
	 * obnoxiously difficult to reliably make any correlation.
	 *
	 * So for now we'll have to bodge it... Hoping that the system
	 * is at least sane enough that an adapter is in the same PCI
	 * segment as its NHI, if we can find *something* on that segment
	 * which meets the requirements for Kernel DMA Protection, we'll
	 * take that to imply that firmware is aware and has (hopefully)
	 * done the right thing in general. We need to know that the PCI
	 * layer has seen the ExternalFacingPort property which will then
	 * inform the IOMMU layer to enforce the complete "untrusted DMA"
	 * flow, but also that the IOMMU driver itself can be trusted not
	 * to have been subverted by a pre-boot DMA attack.
	 */
	while (bus->parent)
		bus = bus->parent;

	pci_walk_bus(bus, nhi_pci_check_iommu_pdev, &port_ok);

	nhi->iommu_dma_protection = port_ok;
	dev_dbg(nhi->dev, "IOMMU DMA protection is %s\n",
		str_enabled_disabled(port_ok));
}

int nhi_pci_init_msi(struct tb_nhi *nhi)
{
	struct tb_nhi_pci *nhi_pci = nhi_to_pci(nhi);
	struct pci_dev *pdev = to_pci_dev(nhi->dev);
	struct device *dev = &pdev->dev;
	int res, irq, nvec;

	ida_init(&nhi_pci->msix_ida);

	/*
	 * The NHI has 16 MSI-X vectors or a single MSI. We first try to
	 * get all MSI-X vectors and if we succeed, each ring will have
	 * one MSI-X. If for some reason that does not work out, we
	 * fallback to a single MSI.
	 */
	nvec = pci_alloc_irq_vectors(pdev, MSIX_MIN_VECS, MSIX_MAX_VECS,
				     PCI_IRQ_MSIX);
	if (nvec < 0) {
		nvec = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_MSI);
		if (nvec < 0)
			return nvec;

		INIT_WORK(&nhi->interrupt_work, nhi_interrupt_work);

		irq = pci_irq_vector(pdev, 0);
		if (irq < 0)
			return irq;

		res = devm_request_irq(&pdev->dev, irq, nhi_msi,
				       IRQF_NO_SUSPEND, "thunderbolt", nhi);
		if (res)
			return dev_err_probe(dev, res, "request_irq failed, aborting\n");
	}

	return 0;
}

static bool nhi_pci_imr_valid(struct pci_dev *pdev)
{
	u8 val;

	if (!device_property_read_u8(&pdev->dev, "IMR_VALID", &val))
		return !!val;

	return true;
}

void nhi_pci_start_dma_port(struct tb_nhi *nhi)
{
	struct pci_dev *pdev = to_pci_dev(nhi->dev);
	struct pci_dev *root_port;

	/*
	 * During host router NVM upgrade we should not allow root port to
	 * go into D3cold because some root ports cannot trigger PME
	 * itself. To be on the safe side keep the root port in D0 during
	 * the whole upgrade process.
	 */
	root_port = pcie_find_root_port(pdev);
	if (root_port)
		pm_runtime_get_noresume(&root_port->dev);
}

void nhi_pci_complete_dma_port(struct tb_nhi *nhi)
{
	struct pci_dev *pdev = to_pci_dev(nhi->dev);
	struct pci_dev *root_port;

	root_port = pcie_find_root_port(pdev);
	if (root_port)
		pm_runtime_put(&root_port->dev);
}

int nhi_pci_ring_request_msix(struct tb_ring *ring, bool no_suspend)
{
	struct tb_nhi *nhi = ring->nhi;
	struct tb_nhi_pci *nhi_pci = nhi_to_pci(nhi);
	struct pci_dev *pdev = to_pci_dev(nhi->dev);
	unsigned long irqflags;
	int ret;

	if (!pdev->msix_enabled)
		return 0;

	ret = ida_alloc_max(&nhi_pci->msix_ida, MSIX_MAX_VECS - 1, GFP_KERNEL);
	if (ret < 0)
		return ret;

	ring->vector = ret;

	ret = pci_irq_vector(pdev, ring->vector);
	if (ret < 0)
		goto err_ida_remove;

	ring->irq = ret;

	irqflags = no_suspend ? IRQF_NO_SUSPEND : 0;
	ret = request_irq(ring->irq, ring_msix, irqflags, "thunderbolt", ring);
	if (ret)
		goto err_ida_remove;

	return 0;

err_ida_remove:
	ida_free(&nhi_pci->msix_ida, ring->vector);

	return ret;
}

void nhi_pci_ring_release_msix(struct tb_ring *ring)
{
	struct tb_nhi_pci *nhi_pci = nhi_to_pci(ring->nhi);

	if (ring->irq <= 0)
		return;

	free_irq(ring->irq, ring);
	ida_free(&nhi_pci->msix_ida, ring->vector);
	ring->vector = 0;
	ring->irq = 0;
}

void nhi_pci_shutdown(struct tb_nhi *nhi)
{
	struct tb_nhi_pci *nhi_pci = nhi_to_pci(nhi);
	struct pci_dev *pdev = to_pci_dev(nhi->dev);

	/*
	 * We have to release the irq before calling flush_work. Otherwise an
	 * already executing IRQ handler could call schedule_work again.
	 */
	if (!pdev->msix_enabled) {
		devm_free_irq(nhi->dev, pdev->irq, nhi);
		flush_work(&nhi->interrupt_work);
	}
	ida_destroy(&nhi_pci->msix_ida);
}

bool nhi_pci_is_present(struct tb_nhi *nhi)
{
	return pci_device_is_present(to_pci_dev(nhi->dev));
}

static const struct tb_nhi_ops pci_nhi_default_ops = {
	.pre_nvm_auth = nhi_pci_start_dma_port,
	.post_nvm_auth = nhi_pci_complete_dma_port,
	.request_ring_irq = nhi_pci_ring_request_msix,
	.release_ring_irq = nhi_pci_ring_release_msix,
	.shutdown = nhi_pci_shutdown,
	.is_present = nhi_pci_is_present,
	.init_interrupts = nhi_pci_init_msi,
};

static int nhi_pci_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct device *dev = &pdev->dev;
	struct tb_nhi_pci *nhi_pci;
	struct tb_nhi *nhi;
	int res;

	if (!nhi_pci_imr_valid(pdev))
		return dev_err_probe(dev, -ENODEV, "firmware image not valid, aborting\n");

	res = pcim_enable_device(pdev);
	if (res)
		return dev_err_probe(dev, res, "cannot enable PCI device, aborting\n");

	nhi_pci = devm_kzalloc(dev, sizeof(*nhi_pci), GFP_KERNEL);
	if (!nhi_pci)
		return -ENOMEM;

	nhi = &nhi_pci->nhi;
	nhi->dev = dev;
	nhi->ops = (const struct tb_nhi_ops *)id->driver_data ?: &pci_nhi_default_ops;

	nhi->iobase = pcim_iomap_region(pdev, 0, "thunderbolt");
	res = PTR_ERR_OR_ZERO(nhi->iobase);
	if (res)
		return dev_err_probe(dev, res, "cannot obtain PCI resources, aborting\n");

	nhi_pci_check_quirks(nhi_pci);
	nhi_pci_check_iommu(nhi_pci);

	pci_set_master(pdev);

	res = nhi_probe(&nhi_pci->nhi);
	if (res)
		return dev_err_probe(dev, res, "NHI common probe failed\n");

	return 0;
}

static void nhi_pci_remove(struct pci_dev *pdev)
{
	struct tb *tb = pci_get_drvdata(pdev);
	struct tb_nhi *nhi = tb->nhi;

	pm_runtime_get_sync(&pdev->dev);
	pm_runtime_dont_use_autosuspend(&pdev->dev);
	pm_runtime_forbid(&pdev->dev);

	tb_domain_remove(tb);
	nhi_shutdown(nhi);
}

/*
 * During suspend the Thunderbolt controller is reset and all PCIe
 * tunnels are lost. The NHI driver will try to reestablish all tunnels
 * during resume. This adds device links between the tunneled PCIe
 * downstream ports and the NHI so that the device core will make sure
 * NHI is resumed first before the rest.
 */
bool tb_apple_add_links(struct tb_nhi *nhi)
{
	struct pci_dev *nhi_pdev = to_pci_dev(nhi->dev);
	struct pci_dev *upstream, *pdev;
	bool ret;

	if (!x86_apple_machine)
		return false;

	switch (nhi_pdev->device) {
	case PCI_DEVICE_ID_INTEL_LIGHT_RIDGE:
	case PCI_DEVICE_ID_INTEL_CACTUS_RIDGE_4C:
	case PCI_DEVICE_ID_INTEL_FALCON_RIDGE_2C_NHI:
	case PCI_DEVICE_ID_INTEL_FALCON_RIDGE_4C_NHI:
		break;
	default:
		return false;
	}

	upstream = pci_upstream_bridge(nhi_pdev);
	while (upstream) {
		if (!pci_is_pcie(upstream))
			return false;
		if (pci_pcie_type(upstream) == PCI_EXP_TYPE_UPSTREAM)
			break;
		upstream = pci_upstream_bridge(upstream);
	}

	if (!upstream)
		return false;

	/*
	 * For each hotplug downstream port, create add device link
	 * back to NHI so that PCIe tunnels can be re-established after
	 * sleep.
	 */
	ret = false;
	for_each_pci_bridge(pdev, upstream->subordinate) {
		const struct device_link *link;

		if (!pci_is_pcie(pdev))
			continue;
		if (pci_pcie_type(pdev) != PCI_EXP_TYPE_DOWNSTREAM ||
		    !pdev->is_pciehp)
			continue;

		link = device_link_add(&pdev->dev, nhi->dev,
				       DL_FLAG_AUTOREMOVE_SUPPLIER |
				       DL_FLAG_PM_RUNTIME);
		if (link) {
			dev_dbg(nhi->dev, "created link from %s\n",
				dev_name(&pdev->dev));
			ret = true;
		} else {
			dev_warn(nhi->dev, "device link creation from %s failed\n",
				 dev_name(&pdev->dev));
		}
	}

	return ret;
}

static struct pci_device_id nhi_ids[] = {
	/*
	 * We have to specify class, the TB bridges use the same device and
	 * vendor (sub)id on gen 1 and gen 2 controllers.
	 */
	{
		.class = PCI_CLASS_SYSTEM_OTHER << 8, .class_mask = ~0,
		.vendor = PCI_VENDOR_ID_INTEL,
		.device = PCI_DEVICE_ID_INTEL_LIGHT_RIDGE,
		.subvendor = 0x2222, .subdevice = 0x1111,
	},
	{
		.class = PCI_CLASS_SYSTEM_OTHER << 8, .class_mask = ~0,
		.vendor = PCI_VENDOR_ID_INTEL,
		.device = PCI_DEVICE_ID_INTEL_CACTUS_RIDGE_4C,
		.subvendor = 0x2222, .subdevice = 0x1111,
	},
	{
		.class = PCI_CLASS_SYSTEM_OTHER << 8, .class_mask = ~0,
		.vendor = PCI_VENDOR_ID_INTEL,
		.device = PCI_DEVICE_ID_INTEL_FALCON_RIDGE_2C_NHI,
		.subvendor = PCI_ANY_ID, .subdevice = PCI_ANY_ID,
	},
	{
		.class = PCI_CLASS_SYSTEM_OTHER << 8, .class_mask = ~0,
		.vendor = PCI_VENDOR_ID_INTEL,
		.device = PCI_DEVICE_ID_INTEL_FALCON_RIDGE_4C_NHI,
		.subvendor = PCI_ANY_ID, .subdevice = PCI_ANY_ID,
	},

	/* Thunderbolt 3 */
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_ALPINE_RIDGE_2C_NHI) },
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_ALPINE_RIDGE_4C_NHI) },
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_ALPINE_RIDGE_USBONLY_NHI) },
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_ALPINE_RIDGE_LP_NHI) },
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_ALPINE_RIDGE_LP_USBONLY_NHI) },
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_ALPINE_RIDGE_C_2C_NHI) },
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_ALPINE_RIDGE_C_4C_NHI) },
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_ALPINE_RIDGE_C_USBONLY_NHI) },
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_TITAN_RIDGE_2C_NHI) },
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_TITAN_RIDGE_4C_NHI) },
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_ICL_NHI0),
	  .driver_data = (kernel_ulong_t)&icl_nhi_ops },
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_ICL_NHI1),
	  .driver_data = (kernel_ulong_t)&icl_nhi_ops },
	/* Thunderbolt 4 */
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_TGL_NHI0),
	  .driver_data = (kernel_ulong_t)&icl_nhi_ops },
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_TGL_NHI1),
	  .driver_data = (kernel_ulong_t)&icl_nhi_ops },
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_TGL_H_NHI0),
	  .driver_data = (kernel_ulong_t)&icl_nhi_ops },
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_TGL_H_NHI1),
	  .driver_data = (kernel_ulong_t)&icl_nhi_ops },
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_ADL_NHI0),
	  .driver_data = (kernel_ulong_t)&icl_nhi_ops },
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_ADL_NHI1),
	  .driver_data = (kernel_ulong_t)&icl_nhi_ops },
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_RPL_NHI0),
	  .driver_data = (kernel_ulong_t)&icl_nhi_ops },
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_RPL_NHI1),
	  .driver_data = (kernel_ulong_t)&icl_nhi_ops },
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_MTL_M_NHI0),
	  .driver_data = (kernel_ulong_t)&icl_nhi_ops },
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_MTL_P_NHI0),
	  .driver_data = (kernel_ulong_t)&icl_nhi_ops },
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_MTL_P_NHI1),
	  .driver_data = (kernel_ulong_t)&icl_nhi_ops },
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_LNL_NHI0),
	  .driver_data = (kernel_ulong_t)&icl_nhi_ops },
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_LNL_NHI1),
	  .driver_data = (kernel_ulong_t)&icl_nhi_ops },
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_PTL_M_NHI0),
	  .driver_data = (kernel_ulong_t)&icl_nhi_ops },
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_PTL_M_NHI1),
	  .driver_data = (kernel_ulong_t)&icl_nhi_ops },
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_PTL_P_NHI0),
	  .driver_data = (kernel_ulong_t)&icl_nhi_ops },
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_PTL_P_NHI1),
	  .driver_data = (kernel_ulong_t)&icl_nhi_ops },
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_WCL_NHI0),
	  .driver_data = (kernel_ulong_t)&icl_nhi_ops },
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_BARLOW_RIDGE_HOST_80G_NHI) },
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_BARLOW_RIDGE_HOST_40G_NHI) },

	/* Any USB4 compliant host */
	{ PCI_DEVICE_CLASS(PCI_CLASS_SERIAL_USB_USB4, ~0) },

	{ 0,}
};

MODULE_DEVICE_TABLE(pci, nhi_ids);
MODULE_DESCRIPTION("Thunderbolt/USB4 core driver");
MODULE_LICENSE("GPL");

static struct pci_driver nhi_driver = {
	.name = "thunderbolt",
	.id_table = nhi_ids,
	.probe = nhi_pci_probe,
	.remove = nhi_pci_remove,
	.shutdown = nhi_pci_remove,
	.driver.pm = &nhi_pm_ops,
};

static int __init nhi_init(void)
{
	int ret;

	ret = tb_domain_init();
	if (ret)
		return ret;

	ret = pci_register_driver(&nhi_driver);
	if (ret)
		tb_domain_exit();

	return ret;
}

static void __exit nhi_unload(void)
{
	pci_unregister_driver(&nhi_driver);
	tb_domain_exit();
}

rootfs_initcall(nhi_init);
module_exit(nhi_unload);
