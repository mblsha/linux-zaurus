/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_LINUX_PXA_OSCR_H
#define _UAPI_LINUX_PXA_OSCR_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define PXA_OSCR_ABI_VERSION		1
#define PXA_OSCR_INFO_SIZE_V1		64
#define PXA_OSCR_SNAPSHOT_SIZE_V1	64

/* The mmap is read-only device memory with execute VMA flags removed. */
#define PXA_OSCR_INFO_F_MMAP_READ_ONLY		(1U << 0)
/* The counter increments modulo 2^counter_bits. */
#define PXA_OSCR_INFO_F_COUNTER_WRAPS		(1U << 1)
/* PXA_OSCR_GET_SNAPSHOT is available. */
#define PXA_OSCR_INFO_F_SNAPSHOT		(1U << 2)
/* Snapshot timestamps use the CLOCK_MONOTONIC_RAW timebase. */
#define PXA_OSCR_INFO_F_SNAPSHOT_MONOTONIC_RAW	(1U << 3)
/* Snapshot generation changes after every timer resume callback. */
#define PXA_OSCR_INFO_F_SUSPEND_GENERATION	(1U << 4)
/* Reads of the documented OS timer registers have no side effects. */
#define PXA_OSCR_INFO_F_REGISTER_READS_SAFE	(1U << 5)

struct pxa_oscr_info {
	__u32 struct_size;
	__u32 version;
	__u64 rate_hz;
	__u32 counter_bits;
	__u32 mmap_oscr_offset;
	__u32 mmap_page_size;
	__u32 flags;
	__u32 generation;
	__u32 reserved0;
	__u64 reserved[3];
};

struct pxa_oscr_snapshot {
	__u32 struct_size;
	__u32 version;
	__u32 oscr_before;
	__u32 oscr_after;
	__u64 monotonic_raw_ns;
	__u32 generation_before;
	__u32 generation_after;
	__u64 reserved[4];
};

#define PXA_OSCR_IOC_MAGIC	0xb9
#define PXA_OSCR_GET_INFO \
	_IOR(PXA_OSCR_IOC_MAGIC, 0x00, struct pxa_oscr_info)
#define PXA_OSCR_GET_SNAPSHOT \
	_IOR(PXA_OSCR_IOC_MAGIC, 0x01, struct pxa_oscr_snapshot)

#endif /* _UAPI_LINUX_PXA_OSCR_H */
