/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2023 Linaro Ltd.
 */

#ifndef __QCOM_TZMEM_H
#define __QCOM_TZMEM_H

#include <linux/cleanup.h>
#include <linux/gfp.h>
#include <linux/types.h>

struct device;
struct qcom_tzmem_pool;

struct qcom_tzmem_pool *qcom_tzmem_pool_new(size_t size);
void qcom_tzmem_pool_free(struct qcom_tzmem_pool *pool);
struct qcom_tzmem_pool *
devm_qcom_tzmem_pool_new(struct device *dev, size_t size);

void *qcom_tzmem_alloc(struct qcom_tzmem_pool *pool, size_t size, gfp_t gfp);
void qcom_tzmem_free(void *ptr);

DEFINE_FREE(qcom_tzmem, void *, if (_T) qcom_tzmem_free(_T));

phys_addr_t qcom_tzmem_to_phys(void *ptr);

#endif /* __QCOM_TZMEM */
