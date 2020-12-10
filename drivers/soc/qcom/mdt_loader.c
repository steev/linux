// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm Peripheral Image Loader
 *
 * Copyright (C) 2016 Linaro Ltd
 * Copyright (C) 2015 Sony Mobile Communications Inc
 * Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 */

#include <linux/device.h>
#include <linux/elf.h>
#include <linux/firmware.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/qcom_scm.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/soc/qcom/mdt_loader.h>

static struct workqueue_struct *mdt_wq;

struct fw_desc {
	size_t mem_size;
	void *mem_region;
	const struct firmware *fw;
	char *fw_name;
	struct device *dev;
	size_t fw_name_len;
	phys_addr_t mem_reloc;
};

struct mdt_seg_data {
	struct work_struct load_seg_work;
	const struct elf32_phdr *phdr;
	int seg_num;
	int result;
	struct fw_desc *desc;
};

static bool mdt_phdr_valid(const struct elf32_phdr *phdr)
{
	if (phdr->p_type != PT_LOAD)
		return false;

	if ((phdr->p_flags & QCOM_MDT_TYPE_MASK) == QCOM_MDT_TYPE_HASH)
		return false;

	if (!phdr->p_memsz)
		return false;

	return true;
}

/**
 * qcom_mdt_get_size() - acquire size of the memory region needed to load mdt
 * @fw:		firmware object for the mdt file
 *
 * Returns size of the loaded firmware blob, or -EINVAL on failure.
 */
ssize_t qcom_mdt_get_size(const struct firmware *fw)
{
	const struct elf32_phdr *phdrs;
	const struct elf32_phdr *phdr;
	const struct elf32_hdr *ehdr;
	phys_addr_t min_addr = PHYS_ADDR_MAX;
	phys_addr_t max_addr = 0;
	int i;

	ehdr = (struct elf32_hdr *)fw->data;
	phdrs = (struct elf32_phdr *)(ehdr + 1);

	for (i = 0; i < ehdr->e_phnum; i++) {
		phdr = &phdrs[i];

		if (!mdt_phdr_valid(phdr))
			continue;

		if (phdr->p_paddr < min_addr)
			min_addr = phdr->p_paddr;

		if (phdr->p_paddr + phdr->p_memsz > max_addr)
			max_addr = ALIGN(phdr->p_paddr + phdr->p_memsz, SZ_4K);
	}

	return min_addr < max_addr ? max_addr - min_addr : -EINVAL;
}
EXPORT_SYMBOL_GPL(qcom_mdt_get_size);

/**
 * qcom_mdt_read_metadata() - read header and metadata from mdt or mbn
 * @fw:		firmware of mdt header or mbn
 * @data_len:	length of the read metadata blob
 *
 * The mechanism that performs the authentication of the loading firmware
 * expects an ELF header directly followed by the segment of hashes, with no
 * padding inbetween. This function allocates a chunk of memory for this pair
 * and copy the two pieces into the buffer.
 *
 * In the case of split firmware the hash is found directly following the ELF
 * header, rather than at p_offset described by the second program header.
 *
 * The caller is responsible to free (kfree()) the returned pointer.
 *
 * Return: pointer to data, or ERR_PTR()
 */
void *qcom_mdt_read_metadata(const struct firmware *fw, size_t *data_len)
{
	const struct elf32_phdr *phdrs;
	const struct elf32_hdr *ehdr;
	size_t hash_offset;
	size_t hash_size;
	size_t ehdr_size;
	void *data;

	ehdr = (struct elf32_hdr *)fw->data;
	phdrs = (struct elf32_phdr *)(ehdr + 1);

	if (ehdr->e_phnum < 2)
		return ERR_PTR(-EINVAL);

	if (phdrs[0].p_type == PT_LOAD || phdrs[1].p_type == PT_LOAD)
		return ERR_PTR(-EINVAL);

	if ((phdrs[1].p_flags & QCOM_MDT_TYPE_MASK) != QCOM_MDT_TYPE_HASH)
		return ERR_PTR(-EINVAL);

	ehdr_size = phdrs[0].p_filesz;
	hash_size = phdrs[1].p_filesz;

	data = kmalloc(ehdr_size + hash_size, GFP_KERNEL);
	if (!data)
		return ERR_PTR(-ENOMEM);

	/* Is the header and hash already packed */
	if (ehdr_size + hash_size == fw->size)
		hash_offset = phdrs[0].p_filesz;
	else
		hash_offset = phdrs[1].p_offset;

	memcpy(data, fw->data, ehdr_size);
	memcpy(data + ehdr_size, fw->data + hash_offset, hash_size);

	*data_len = ehdr_size + hash_size;

	return data;
}
EXPORT_SYMBOL_GPL(qcom_mdt_read_metadata);

static void mdt_load_seg_work_fn(struct work_struct *work)
{
	struct mdt_seg_data *mdt_seg_data = container_of(work, struct mdt_seg_data, load_seg_work);
	struct fw_desc *desc = mdt_seg_data->desc;
	int seg_num = mdt_seg_data->seg_num;
	const struct elf32_phdr *phdr = mdt_seg_data->phdr;
	const struct firmware *seg_fw;
	ssize_t offset;
	void *ptr;
	int ret;
	char *fw_name = kstrdup(desc->fw_name, GFP_KERNEL);

	if (!mdt_phdr_valid(phdr))
		goto fw_free;

	offset = phdr->p_paddr - desc->mem_reloc;
	if (offset < 0 || offset + phdr->p_memsz > desc->mem_size) {
		dev_err(desc->dev, "segment outside memory range\n");
		mdt_seg_data->result = -EINVAL;
		goto fw_free;
	}

	ptr = desc->mem_region + offset;

	if (phdr->p_filesz && phdr->p_offset < desc->fw->size) {
		/* Firmware is large enough to be non-split */
		if (phdr->p_offset + phdr->p_filesz > desc->fw->size) {
			dev_err(desc->dev,
				"failed to load segment %d from truncated file %s\n",
				seg_num, desc->fw_name);
			mdt_seg_data->result = -EINVAL;
			goto fw_free;
		}

		memcpy(ptr, desc->fw->data + phdr->p_offset, phdr->p_filesz);
	} else if (phdr->p_filesz) {
		/* Firmware not large enough, load split-out segments */
		sprintf(fw_name + desc->fw_name_len - 3, "b%02d", seg_num);
		ret = request_firmware_into_buf(&seg_fw, fw_name, desc->dev,
						ptr, phdr->p_filesz);
		if (ret) {
			dev_err(desc->dev, "failed to load %s\n", fw_name);
			mdt_seg_data->result = -EINVAL;
			goto fw_free;
		}

		release_firmware(seg_fw);
	}

	if (phdr->p_memsz > phdr->p_filesz)
		memset(ptr + phdr->p_filesz, 0, phdr->p_memsz - phdr->p_filesz);

fw_free:
	kfree(fw_name);
}

static int __qcom_mdt_load(struct device *dev, const struct firmware *fw,
			   const char *firmware, int pas_id, void *mem_region,
			   phys_addr_t mem_phys, size_t mem_size,
			   phys_addr_t *reloc_base, bool pas_init)
{
	const struct elf32_phdr *phdrs;
	const struct elf32_phdr *phdr;
	const struct elf32_hdr *ehdr;
	struct mdt_seg_data *segs;
	phys_addr_t mem_reloc;
	phys_addr_t min_addr = PHYS_ADDR_MAX;
	phys_addr_t max_addr = 0;
	size_t metadata_len;
	size_t fw_name_len;
	void *metadata;
	char *fw_name;
	bool relocate = false;
	int ret = 0;
	int i;
	struct fw_desc *desc = kzalloc(sizeof(*desc), GFP_KERNEL);

	if (!fw || !mem_region || !mem_phys || !mem_size)
		return -EINVAL;

	ehdr = (struct elf32_hdr *)fw->data;
	phdrs = (struct elf32_phdr *)(ehdr + 1);

	fw_name_len = strlen(firmware);
	if (fw_name_len <= 4)
		return -EINVAL;

	fw_name = kstrdup(firmware, GFP_KERNEL);
	if (!fw_name)
		return -ENOMEM;

	if (pas_init) {
		metadata = qcom_mdt_read_metadata(fw, &metadata_len);
		if (IS_ERR(metadata)) {
			ret = PTR_ERR(metadata);
			goto out;
		}

		ret = qcom_scm_pas_init_image(pas_id, metadata, metadata_len);

		kfree(metadata);
		if (ret) {
			dev_err(dev, "invalid firmware metadata\n");
			goto out;
		}
	}

	for (i = 0; i < ehdr->e_phnum; i++) {
		phdr = &phdrs[i];

		if (!mdt_phdr_valid(phdr))
			continue;

		if (phdr->p_flags & QCOM_MDT_RELOCATABLE)
			relocate = true;

		if (phdr->p_paddr < min_addr)
			min_addr = phdr->p_paddr;

		if (phdr->p_paddr + phdr->p_memsz > max_addr)
			max_addr = ALIGN(phdr->p_paddr + phdr->p_memsz, SZ_4K);
	}

	if (relocate) {
		if (pas_init) {
			ret = qcom_scm_pas_mem_setup(pas_id, mem_phys,
						     max_addr - min_addr);
			if (ret) {
				dev_err(dev, "unable to setup relocation\n");
				goto out;
			}
		}

		/*
		 * The image is relocatable, so offset each segment based on
		 * the lowest segment address.
		 */
		mem_reloc = min_addr;
	} else {
		/*
		 * Image is not relocatable, so offset each segment based on
		 * the allocated physical chunk of memory.
		 */
		mem_reloc = mem_phys;
	}

	if (!mdt_wq)
		mdt_wq = alloc_workqueue("mdt_workqueue", WQ_HIGHPRI | WQ_UNBOUND, 0);

	segs = kcalloc(ehdr->e_phnum, sizeof(struct mdt_seg_data), GFP_KERNEL);

	/* Fill in the firmware descriptor to be used by individual work items */
	desc->mem_size = mem_size;
	desc->mem_region = mem_region;
	desc->fw = fw;
	desc->fw_name_len = fw_name_len;
	desc->fw_name = fw_name;
	desc->dev = dev;
	desc->mem_reloc = mem_reloc;

	/* Queue in individual work item for each firmware blob */
	for (i = 0; i < ehdr->e_phnum; i++) {
		INIT_WORK(&segs[i].load_seg_work, mdt_load_seg_work_fn);
		segs[i].seg_num = i;
		segs[i].phdr = &phdrs[i];
		segs[i].desc = desc;
		queue_work(mdt_wq, &segs[i].load_seg_work);
	}

	/* Wait for the parallel loads to finish */
	for (i = 0; i < ehdr->e_phnum; i++) {
		flush_work(&segs[i].load_seg_work);
		ret |= segs[i].result;
	}

	/* Different blobs can fail with different errors, so return a generic error */
	if (ret)
		ret = -EFAULT;

	if (reloc_base)
		*reloc_base = mem_reloc;

out:
	kfree(fw_name);
	kfree(desc);
	kfree(segs);

	return ret;
}

/**
 * qcom_mdt_load() - load the firmware which header is loaded as fw
 * @dev:	device handle to associate resources with
 * @fw:		firmware object for the mdt file
 * @firmware:	name of the firmware, for construction of segment file names
 * @pas_id:	PAS identifier
 * @mem_region:	allocated memory region to load firmware into
 * @mem_phys:	physical address of allocated memory region
 * @mem_size:	size of the allocated memory region
 * @reloc_base:	adjusted physical address after relocation
 *
 * Returns 0 on success, negative errno otherwise.
 */
int qcom_mdt_load(struct device *dev, const struct firmware *fw,
		  const char *firmware, int pas_id, void *mem_region,
		  phys_addr_t mem_phys, size_t mem_size,
		  phys_addr_t *reloc_base)
{
	return __qcom_mdt_load(dev, fw, firmware, pas_id, mem_region, mem_phys,
			       mem_size, reloc_base, true);
}
EXPORT_SYMBOL_GPL(qcom_mdt_load);

/**
 * qcom_mdt_load_no_init() - load the firmware which header is loaded as fw
 * @dev:	device handle to associate resources with
 * @fw:		firmware object for the mdt file
 * @firmware:	name of the firmware, for construction of segment file names
 * @pas_id:	PAS identifier
 * @mem_region:	allocated memory region to load firmware into
 * @mem_phys:	physical address of allocated memory region
 * @mem_size:	size of the allocated memory region
 * @reloc_base:	adjusted physical address after relocation
 *
 * Returns 0 on success, negative errno otherwise.
 */
int qcom_mdt_load_no_init(struct device *dev, const struct firmware *fw,
			  const char *firmware, int pas_id,
			  void *mem_region, phys_addr_t mem_phys,
			  size_t mem_size, phys_addr_t *reloc_base)
{
	return __qcom_mdt_load(dev, fw, firmware, pas_id, mem_region, mem_phys,
			       mem_size, reloc_base, false);
}
EXPORT_SYMBOL_GPL(qcom_mdt_load_no_init);

MODULE_DESCRIPTION("Firmware parser for Qualcomm MDT format");
MODULE_LICENSE("GPL v2");
