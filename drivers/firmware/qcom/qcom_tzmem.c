// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Memory allocator for buffers shared with the TrustZone.
 *
 * Copyright (C) 2023 Linaro Ltd.
 */

#include <linux/bug.h>
#include <linux/cleanup.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/firmware/qcom/qcom_tzmem.h>
#include <linux/genalloc.h>
#include <linux/gfp.h>
#include <linux/mm.h>
#include <linux/radix-tree.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>

#include "qcom_tzmem.h"

struct qcom_tzmem_pool {
	void *vbase;
	dma_addr_t pbase;
	size_t size;
	struct gen_pool *pool;
	void *priv;
};

struct qcom_tzmem_chunk {
	phys_addr_t paddr;
	size_t size;
	struct qcom_tzmem_pool *owner;
};

static struct device *qcom_tzmem_dev;
static RADIX_TREE(qcom_tzmem_chunks, GFP_ATOMIC);
static DEFINE_SPINLOCK(qcom_tzmem_chunks_lock);

#if IS_ENABLED(CONFIG_QCOM_TZMEM_MODE_DEFAULT)

static int qcom_tzmem_init(void)
{
	return 0;
}

static int qcom_tzmem_init_pool(struct qcom_tzmem_pool *pool)
{
	return 0;
}

static void qcom_tzmem_cleanup_pool(struct qcom_tzmem_pool *pool)
{

}

#endif /* CONFIG_QCOM_TZMEM_MODE_DEFAULT */

/**
 * qcom_tzmem_pool_new() - Create a new TZ memory pool.
 * @size: Size of the new pool in bytes.
 *
 * Create a new pool of memory suitable for sharing with the TrustZone.
 *
 * Must not be used in atomic context.
 *
 * Returns:
 * New memory pool address or ERR_PTR() on error.
 */
struct qcom_tzmem_pool *qcom_tzmem_pool_new(size_t size)
{
	struct qcom_tzmem_pool *pool;
	int ret = -ENOMEM;

	if (!size)
		return ERR_PTR(-EINVAL);

	size = PAGE_ALIGN(size);

	pool = kzalloc(sizeof(*pool), GFP_KERNEL);
	if (!pool)
		return ERR_PTR(-ENOMEM);

	pool->size = size;

	pool->vbase = dma_alloc_coherent(qcom_tzmem_dev, size, &pool->pbase,
					 GFP_KERNEL);
	if (!pool->vbase)
		goto err_kfree_pool;

	pool->pool = gen_pool_create(PAGE_SHIFT, -1);
	if (!pool)
		goto err_dma_free;

	gen_pool_set_algo(pool->pool, gen_pool_best_fit, NULL);

	ret = gen_pool_add_virt(pool->pool, (unsigned long)pool->vbase,
				(phys_addr_t)pool->pbase, size, -1);
	if (ret)
		goto err_destroy_genpool;

	ret = qcom_tzmem_init_pool(pool);
	if (ret)
		goto err_destroy_genpool;

	return pool;

err_destroy_genpool:
	gen_pool_destroy(pool->pool);
err_dma_free:
	dma_free_coherent(qcom_tzmem_dev, size, pool->vbase, pool->pbase);
err_kfree_pool:
	kfree(pool);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(qcom_tzmem_pool_new);

/**
 * qcom_tzmem_pool_free() - Destroy a TZ memory pool and free all resources.
 * @pool: Memory pool to free.
 *
 * Must not be called if any of the allocated chunks has not been freed.
 * Must not be used in atomic context.
 */
void qcom_tzmem_pool_free(struct qcom_tzmem_pool *pool)
{
	struct qcom_tzmem_chunk *chunk;
	struct radix_tree_iter iter;
	bool non_empty = false;
	void __rcu **slot;

	if (!pool)
		return;

	qcom_tzmem_cleanup_pool(pool);

	scoped_guard(spinlock_irqsave, &qcom_tzmem_chunks_lock) {
		radix_tree_for_each_slot(slot, &qcom_tzmem_chunks, &iter, 0) {
			chunk = radix_tree_deref_slot_protected(slot,
						&qcom_tzmem_chunks_lock);

			if (chunk->owner == pool)
				non_empty = true;
		}
	}

	WARN(non_empty, "Freeing TZ memory pool with memory still allocated");

	gen_pool_destroy(pool->pool);
	dma_free_coherent(qcom_tzmem_dev, pool->size, pool->vbase, pool->pbase);
	kfree(pool);
}
EXPORT_SYMBOL_GPL(qcom_tzmem_pool_free);

static void devm_qcom_tzmem_pool_free(void *data)
{
	struct qcom_tzmem_pool *pool = data;

	qcom_tzmem_pool_free(pool);
}

/**
 * devm_qcom_tzmem_pool_new() - Managed variant of qcom_tzmem_pool_new().
 * @dev: Device managing this resource.
 * @size: Size of the pool in bytes.
 *
 * Must not be used in atomic context.
 *
 * Returns:
 * Address of the managed pool or ERR_PTR() on failure.
 */
struct qcom_tzmem_pool *
devm_qcom_tzmem_pool_new(struct device *dev, size_t size)
{
	struct qcom_tzmem_pool *pool;
	int ret;

	pool = qcom_tzmem_pool_new(size);
	if (IS_ERR(pool))
		return pool;

	ret = devm_add_action_or_reset(dev, devm_qcom_tzmem_pool_free, pool);
	if (ret)
		return ERR_PTR(ret);

	return pool;
}

/**
 * qcom_tzmem_alloc() - Allocate a memory chunk suitable for sharing with TZ.
 * @pool: TZ memory pool from which to allocate memory.
 * @size: Number of bytes to allocate.
 * @gfp: GFP flags.
 *
 * Can be used in any context.
 *
 * Returns:
 * Address of the allocated buffer or NULL if no more memory can be allocated.
 * The buffer must be released using qcom_tzmem_free().
 */
void *qcom_tzmem_alloc(struct qcom_tzmem_pool *pool, size_t size, gfp_t gfp)
{
	struct qcom_tzmem_chunk *chunk;
	unsigned long vaddr;
	int ret;

	if (!size)
		return NULL;

	size = PAGE_ALIGN(size);

	chunk = kzalloc(sizeof(*chunk), gfp);
	if (!chunk)
		return NULL;

	vaddr = gen_pool_alloc(pool->pool, size);
	if (!vaddr) {
		kfree(chunk);
		return NULL;
	}

	chunk->paddr = gen_pool_virt_to_phys(pool->pool, vaddr);
	chunk->size = size;
	chunk->owner = pool;

	scoped_guard(spinlock_irqsave, &qcom_tzmem_chunks_lock) {
		ret = radix_tree_insert(&qcom_tzmem_chunks, vaddr, chunk);
		if (ret) {
			gen_pool_free(pool->pool, vaddr, size);
			kfree(chunk);
			return NULL;
		}
	}

	return (void *)vaddr;
}
EXPORT_SYMBOL_GPL(qcom_tzmem_alloc);

/**
 * qcom_tzmem_free() - Release a buffer allocated from a TZ memory pool.
 * @vaddr: Virtual address of the buffer.
 *
 * Can be used in any context.
 */
void qcom_tzmem_free(void *vaddr)
{
	struct qcom_tzmem_chunk *chunk;

	scoped_guard(spinlock_irqsave, &qcom_tzmem_chunks_lock)
		chunk = radix_tree_delete_item(&qcom_tzmem_chunks,
					       (unsigned long)vaddr, NULL);

	if (!chunk) {
		WARN(1, "Virtual address %p not owned by TZ memory allocator",
		     vaddr);
		return;
	}

	gen_pool_free(chunk->owner->pool, (unsigned long)vaddr, chunk->size);
	kfree(chunk);
}
EXPORT_SYMBOL_GPL(qcom_tzmem_free);

/**
 * qcom_tzmem_to_phys() - Map the virtual address of a TZ buffer to physical.
 * @vaddr: Virtual address of the buffer allocated from a TZ memory pool.
 *
 * Can be used in any context. The address must have been returned by a call
 * to qcom_tzmem_alloc().
 *
 * Returns:
 * Physical address of the buffer.
 */
phys_addr_t qcom_tzmem_to_phys(void *vaddr)
{
	struct qcom_tzmem_chunk *chunk;

	guard(spinlock_irqsave)(&qcom_tzmem_chunks_lock);

	chunk = radix_tree_lookup(&qcom_tzmem_chunks, (unsigned long)vaddr);
	if (!chunk)
		return 0;

	return chunk->paddr;
}
EXPORT_SYMBOL_GPL(qcom_tzmem_to_phys);

int qcom_tzmem_enable(struct device *dev)
{
	if (qcom_tzmem_dev)
		return -EBUSY;

	qcom_tzmem_dev = dev;

	return qcom_tzmem_init();
}
EXPORT_SYMBOL_GPL(qcom_tzmem_enable);

MODULE_DESCRIPTION("TrustZone memory allocator for Qualcomm firmware drivers");
MODULE_AUTHOR("Bartosz Golaszewski <bartosz.golaszewski@linaro.org>");
MODULE_LICENSE("GPL");
