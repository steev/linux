/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef __TBT_PCI_H
#define __TBT_PCI_H

#include <linux/types.h>

void nhi_pci_start_dma_port(struct tb_nhi *nhi);
void nhi_pci_complete_dma_port(struct tb_nhi *nhi);
int nhi_pci_ring_request_msix(struct tb_ring *ring, bool no_suspend);
void nhi_pci_ring_release_msix(struct tb_ring *ring);
bool nhi_pci_is_present(struct tb_nhi *nhi);
void nhi_pci_shutdown(struct tb_nhi *nhi);
int nhi_pci_init_msi(struct tb_nhi *nhi);

#endif
