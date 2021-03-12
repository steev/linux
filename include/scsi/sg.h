/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _SCSI_GENERIC_H
#define _SCSI_GENERIC_H

#include <linux/compiler.h>

#if defined(__KERNEL__)
extern int sg_big_buff; /* for sysctl */

/*
 * In version 3.9.01 of the sg driver, this file was split in two, with the
 * bulk of the user space interface being placed in the file being included
 * in the following line.
 */

#include <uapi/scsi/sg.h>

#include <linux/compat.h>

struct compat_sg_io_hdr {
	compat_int_t interface_id;	/* [i] 'S' for SCSI generic (required) */
	compat_int_t dxfer_direction;	/* [i] data transfer direction  */
	unsigned char cmd_len;		/* [i] SCSI command length ( <= 16 bytes) */
	unsigned char mx_sb_len;	/* [i] max length to write to sbp */
	unsigned short iovec_count;	/* [i] 0 implies no scatter gather */
	compat_uint_t dxfer_len;	/* [i] byte count of data transfer */
	compat_uint_t dxferp;		/* [i], [*io] points to data transfer memory
						or scatter gather list */
	compat_uptr_t cmdp;		/* [i], [*i] points to command to perform */
	compat_uptr_t sbp;		/* [i], [*o] points to sense_buffer memory */
	compat_uint_t timeout;		/* [i] MAX_UINT->no timeout (unit: millisec) */
	compat_uint_t flags;		/* [i] 0 -> default, see SG_FLAG... */
	compat_int_t pack_id;		/* [i->o] unused internally (normally) */
	compat_uptr_t usr_ptr;		/* [i->o] unused internally */
	unsigned char status;		/* [o] scsi status */
	unsigned char masked_status;	/* [o] shifted, masked scsi status */
	unsigned char msg_status;	/* [o] messaging level data (optional) */
	unsigned char sb_len_wr;	/* [o] byte count actually written to sbp */
	unsigned short host_status;	/* [o] errors from host adapter */
	unsigned short driver_status;	/* [o] errors from software driver */
	compat_int_t resid;		/* [o] dxfer_len - actual_transferred */
	compat_uint_t duration;		/* [o] time taken by cmd (unit: millisec) */
	compat_uint_t info;		/* [o] auxiliary information */
};

#define SG_DEFAULT_TIMEOUT_USER (60 * USER_HZ) /* HZ: jiffies in 1 second */
#endif

#undef SG_DEFAULT_TIMEOUT	/* because of conflicting define in sg.c */

#endif	/* end of ifndef _SCSI_GENERIC_H guard */
