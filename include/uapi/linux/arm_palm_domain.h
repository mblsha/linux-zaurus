/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_LINUX_ARM_PALM_DOMAIN_H
#define _UAPI_LINUX_ARM_PALM_DOMAIN_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define ARM_PALM_DOMAIN_ABI_VERSION	1

#define ARM_PALM_DOMAIN_MODE_CLIENT	0
#define ARM_PALM_DOMAIN_MODE_MANAGER	1

#define ARM_PALM_DOMAIN_INFO_MMAP_OWNS_STORAGE	(1U << 0)
#define ARM_PALM_DOMAIN_INFO_SINGLE_OWNER_THREAD	(1U << 1)
#define ARM_PALM_DOMAIN_INFO_NO_HOTPATH_TLBI	(1U << 2)

struct arm_palm_domain_mode {
	__u32 mode;
	__u32 reserved;
};

struct arm_palm_domain_info {
	__u32 abi_version;
	__u32 domain;
	__u32 flags;
	__u32 mode;
	__aligned_u64 start;
	__aligned_u64 length;
	__aligned_u64 transitions;
	__aligned_u64 pmd_mutations;
	__aligned_u64 tlb_flushes;
};

#define ARM_PALM_DOMAIN_IOC_MAGIC	'P'
#define ARM_PALM_DOMAIN_GET_INFO \
	_IOR(ARM_PALM_DOMAIN_IOC_MAGIC, 0x00, struct arm_palm_domain_info)
#define ARM_PALM_DOMAIN_SET_MODE \
	_IOW(ARM_PALM_DOMAIN_IOC_MAGIC, 0x01, struct arm_palm_domain_mode)

#endif /* _UAPI_LINUX_ARM_PALM_DOMAIN_H */
