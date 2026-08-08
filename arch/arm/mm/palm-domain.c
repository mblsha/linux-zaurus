// SPDX-License-Identifier: GPL-2.0-only
/*
 * Fast ARM short-descriptor protection switching for RePalm storage.
 */

#include <linux/capability.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/personality.h>
#include <linux/sched.h>
#include <linux/sched/task.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>

#include <linux/arm_palm_domain.h>

#include <asm/domain.h>
#include <asm/palm_domain.h>
#include <asm/pgalloc.h>
#include <asm/pgtable.h>
#include <asm/tlbflush.h>

#define PALM_ZIRE_STORAGE_START	0x70400000UL
#define PALM_ZIRE_STORAGE_SIZE	(8UL * SZ_1M)
#define PALM_T3_STORAGE_START	0x63400000UL
#define PALM_T3_STORAGE_SIZE	(11UL * SZ_1M)

struct arm_palm_domain_ctx {
	struct task_struct *owner;
	struct mm_struct *mm;
	struct vm_area_struct *vma;
	void *storage;
	unsigned long start;
	unsigned long length;
	u32 mode;
	bool mapped;
	bool revoked;
	u64 transitions;
	u64 pmd_mutations;
	u64 tlb_flushes;
};

static DEFINE_MUTEX(arm_palm_domain_lock);
static struct arm_palm_domain_ctx *arm_palm_domain_active;

static u32 palm_domain_dacr(u32 dacr, unsigned int type)
{
	return (dacr & ~domain_mask(DOMAIN_PALM_STORAGE)) |
		domain_val(DOMAIN_PALM_STORAGE, type);
}

static void palm_domain_set_task(struct task_struct *task, unsigned int type)
{
	struct thread_info *thread = task_thread_info(task);
	u32 dacr;

	if (task != current) {
		/* CONFIG_ARM_PALM_DOMAIN is UP-only, so task cannot be running. */
		thread->cpu_domain = palm_domain_dacr(thread->cpu_domain, type);
		return;
	}

	preempt_disable();
	dacr = palm_domain_dacr(get_domain(), type);
	thread->cpu_domain = dacr;
	set_domain(dacr);
	preempt_enable();
}

static bool palm_domain_supported_range(unsigned long start,
					unsigned long length)
{
	return (start == PALM_ZIRE_STORAGE_START &&
		length == PALM_ZIRE_STORAGE_SIZE) ||
	       (start == PALM_T3_STORAGE_START &&
		length == PALM_T3_STORAGE_SIZE);
}

static pmd_t *palm_domain_pmd(struct mm_struct *mm, unsigned long address)
{
	pgd_t *pgd = pgd_offset(mm, address);
	p4d_t *p4d = p4d_offset(pgd, address);
	pud_t *pud = pud_offset(p4d, address);
	pmd_t *pmd = pmd_offset(pud, address);

	/* A folded Linux PMD contains two 1 MiB hardware descriptors. */
	return pmd + ((address >> SECTION_SHIFT) & 1);
}

static int palm_domain_validate_pmds(struct arm_palm_domain_ctx *ctx,
				     unsigned int expected_domain)
{
	unsigned long address;

	for (address = ctx->start; address < ctx->start + ctx->length;
	     address += SECTION_SIZE) {
		pmd_t *pmd = palm_domain_pmd(ctx->mm, address);
		pmdval_t value = pmd_val(*pmd);

		if ((value & PMD_TYPE_MASK) != PMD_TYPE_TABLE)
			return -EINVAL;
		if ((value & PMD_DOMAIN_MASK) != PMD_DOMAIN(expected_domain))
			return -EINVAL;
	}

	return 0;
}

static void palm_domain_retag_pmds(struct arm_palm_domain_ctx *ctx,
				   unsigned int domain)
{
	unsigned long address;

	for (address = ctx->start; address < ctx->start + ctx->length;
	     address += SECTION_SIZE) {
		pmd_t *pmd = palm_domain_pmd(ctx->mm, address);
		pmdval_t value = pmd_val(*pmd);

		value &= ~PMD_DOMAIN_MASK;
		value |= PMD_DOMAIN(domain);
		*pmd = __pmd(value);
		flush_pmd_entry(pmd);
		ctx->pmd_mutations++;
	}
}

static void palm_domain_flush_mapping(struct arm_palm_domain_ctx *ctx)
{
	flush_tlb_range(ctx->vma, ctx->start, ctx->start + ctx->length);
	ctx->tlb_flushes++;
}

/* Caller holds mmap_write_lock(ctx->mm) and arm_palm_domain_lock. */
static void palm_domain_revoke_locked(struct arm_palm_domain_ctx *ctx)
{
	if (!ctx->mapped || ctx->revoked)
		return;

	palm_domain_set_task(ctx->owner, DOMAIN_CLIENT);
	if (!palm_domain_validate_pmds(ctx, DOMAIN_PALM_STORAGE)) {
		palm_domain_retag_pmds(ctx, DOMAIN_USER);
		palm_domain_flush_mapping(ctx);
	}
	ctx->mode = ARM_PALM_DOMAIN_MODE_CLIENT;
	ctx->revoked = true;
}

static int palm_domain_vma_may_split(struct vm_area_struct *vma,
				     unsigned long address)
{
	return -EPERM;
}

static int palm_domain_vma_mremap(struct vm_area_struct *vma)
{
	return -EPERM;
}

static int palm_domain_vma_mprotect(struct vm_area_struct *vma,
				    unsigned long start,
				    unsigned long end,
				    unsigned long newflags)
{
	return -EPERM;
}

static void palm_domain_vma_close(struct vm_area_struct *vma)
{
	struct arm_palm_domain_ctx *ctx = vma->vm_private_data;

	mutex_lock(&arm_palm_domain_lock);
	if (ctx->vma == vma) {
		palm_domain_revoke_locked(ctx);
		ctx->mapped = false;
		ctx->vma = NULL;
	}
	mutex_unlock(&arm_palm_domain_lock);
}

static const struct vm_operations_struct palm_domain_vm_ops = {
	.close = palm_domain_vma_close,
	.may_split = palm_domain_vma_may_split,
	.mremap = palm_domain_vma_mremap,
	.mprotect = palm_domain_vma_mprotect,
};

static int palm_domain_open(struct inode *inode, struct file *file)
{
	struct arm_palm_domain_ctx *ctx;

	if (!capable(CAP_SYS_RAWIO))
		return -EPERM;
	if (!current->mm)
		return -EINVAL;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	get_task_struct(current);
	mmgrab(current->mm);
	ctx->owner = current;
	ctx->mm = current->mm;
	ctx->mode = ARM_PALM_DOMAIN_MODE_CLIENT;

	mutex_lock(&arm_palm_domain_lock);
	if (arm_palm_domain_active) {
		mutex_unlock(&arm_palm_domain_lock);
		mmdrop(ctx->mm);
		put_task_struct(ctx->owner);
		kfree(ctx);
		return -EBUSY;
	}
	arm_palm_domain_active = ctx;
	file->private_data = ctx;
	mutex_unlock(&arm_palm_domain_lock);

	return 0;
}

static int palm_domain_release(struct inode *inode, struct file *file)
{
	struct arm_palm_domain_ctx *ctx = file->private_data;

	mutex_lock(&arm_palm_domain_lock);
	if (WARN_ON(ctx->mapped))
		palm_domain_set_task(ctx->owner, DOMAIN_CLIENT);
	if (arm_palm_domain_active == ctx)
		arm_palm_domain_active = NULL;
	mutex_unlock(&arm_palm_domain_lock);

	vfree(ctx->storage);
	mmdrop(ctx->mm);
	put_task_struct(ctx->owner);
	kfree(ctx);
	return 0;
}

static int palm_domain_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct arm_palm_domain_ctx *ctx = file->private_data;
	unsigned long length = vma->vm_end - vma->vm_start;
	void *storage;
	int ret;

	if (current != ctx->owner || current->mm != ctx->mm)
		return -EPERM;
	if (vma->vm_pgoff || !palm_domain_supported_range(vma->vm_start, length))
		return -EINVAL;
	if (!(vma->vm_flags & VM_READ) || !(vma->vm_flags & VM_SHARED) ||
	    (vma->vm_flags & VM_WRITE) ||
	    ((vma->vm_flags & VM_EXEC) &&
	     !(current->personality & READ_IMPLIES_EXEC)))
		return -EACCES;

	storage = vmalloc_user(length);
	if (!storage)
		return -ENOMEM;

	mutex_lock(&arm_palm_domain_lock);
	if (ctx->mapped || ctx->revoked) {
		ret = -EBUSY;
		goto out_unlock;
	}

	vfree(ctx->storage);
	ctx->storage = storage;
	storage = NULL;
	ctx->start = vma->vm_start;
	ctx->length = length;

	vm_flags_mod(vma, VM_DONTCOPY | VM_DONTDUMP | VM_DONTEXPAND,
		     VM_WRITE | VM_EXEC | VM_MAYWRITE | VM_MAYEXEC);
	vma->vm_page_prot = vm_get_page_prot(vma->vm_flags);
	ret = remap_vmalloc_range(vma, ctx->storage, 0);
	if (ret)
		goto out_unlock;

	ret = palm_domain_validate_pmds(ctx, DOMAIN_USER);
	if (ret)
		goto out_unlock;

	ctx->vma = vma;
	ctx->mapped = true;
	ctx->mode = ARM_PALM_DOMAIN_MODE_CLIENT;
	palm_domain_retag_pmds(ctx, DOMAIN_PALM_STORAGE);
	palm_domain_flush_mapping(ctx);
	palm_domain_set_task(ctx->owner, DOMAIN_CLIENT);

	vma->vm_ops = &palm_domain_vm_ops;
	vma->vm_private_data = ctx;

out_unlock:
	mutex_unlock(&arm_palm_domain_lock);
	vfree(storage);
	return ret;
}

static long palm_domain_ioctl(struct file *file, unsigned int command,
			      unsigned long argument)
{
	struct arm_palm_domain_ctx *ctx = file->private_data;
	void __user *argp = (void __user *)argument;
	struct arm_palm_domain_mode requested;
	struct arm_palm_domain_info info;
	unsigned int type;
	long ret = 0;

	if (_IOC_TYPE(command) != ARM_PALM_DOMAIN_IOC_MAGIC)
		return -ENOTTY;
	if (current != ctx->owner || current->mm != ctx->mm)
		return -EPERM;

	switch (command) {
	case ARM_PALM_DOMAIN_GET_INFO:
		mutex_lock(&arm_palm_domain_lock);
		info = (struct arm_palm_domain_info) {
			.abi_version = ARM_PALM_DOMAIN_ABI_VERSION,
			.domain = DOMAIN_PALM_STORAGE,
			.flags = ARM_PALM_DOMAIN_INFO_MMAP_OWNS_STORAGE |
				 ARM_PALM_DOMAIN_INFO_SINGLE_OWNER_THREAD |
				 ARM_PALM_DOMAIN_INFO_NO_HOTPATH_TLBI,
			.mode = ctx->mode,
			.start = ctx->start,
			.length = ctx->length,
			.transitions = ctx->transitions,
			.pmd_mutations = ctx->pmd_mutations,
			.tlb_flushes = ctx->tlb_flushes,
		};
		mutex_unlock(&arm_palm_domain_lock);
		if (copy_to_user(argp, &info, sizeof(info)))
			return -EFAULT;
		return 0;

	case ARM_PALM_DOMAIN_SET_MODE:
		if (copy_from_user(&requested, argp, sizeof(requested)))
			return -EFAULT;
		if (requested.reserved)
			return -EINVAL;
		if (requested.mode == ARM_PALM_DOMAIN_MODE_CLIENT)
			type = DOMAIN_CLIENT;
		else if (requested.mode == ARM_PALM_DOMAIN_MODE_MANAGER)
			type = DOMAIN_MANAGER;
		else
			return -EINVAL;

		mutex_lock(&arm_palm_domain_lock);
		if (!ctx->mapped || ctx->revoked) {
			ret = -ENXIO;
		} else if (ctx->mode != requested.mode) {
			palm_domain_set_task(ctx->owner, type);
			ctx->mode = requested.mode;
			ctx->transitions++;
		}
		mutex_unlock(&arm_palm_domain_lock);
		return ret;

	default:
		return -ENOTTY;
	}
}

static const struct file_operations palm_domain_fops = {
	.owner = THIS_MODULE,
	.open = palm_domain_open,
	.release = palm_domain_release,
	.mmap = palm_domain_mmap,
	.unlocked_ioctl = palm_domain_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = palm_domain_ioctl,
#endif
	.llseek = noop_llseek,
};

static struct miscdevice palm_domain_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "arm_palm_domain",
	.fops = &palm_domain_fops,
	.mode = 0600,
};

builtin_misc_device(palm_domain_miscdev);

static void palm_domain_revoke_task(struct task_struct *task)
{
	struct arm_palm_domain_ctx *ctx;
	struct mm_struct *mm = NULL;

	mutex_lock(&arm_palm_domain_lock);
	ctx = arm_palm_domain_active;
	if (ctx && ctx->owner == task && ctx->mapped && !ctx->revoked &&
	    mmget_not_zero(ctx->mm)) {
		mm = ctx->mm;
	}
	mutex_unlock(&arm_palm_domain_lock);
	if (!mm)
		return;

	mmap_write_lock(mm);
	mutex_lock(&arm_palm_domain_lock);
	ctx = arm_palm_domain_active;
	if (ctx && ctx->owner == task && ctx->mm == mm)
		palm_domain_revoke_locked(ctx);
	mutex_unlock(&arm_palm_domain_lock);
	mmap_write_unlock(mm);
	mmput(mm);
}

void arm_palm_domain_flush_task(struct task_struct *task)
{
	palm_domain_revoke_task(task);
}

void arm_palm_domain_exit_task(struct task_struct *task)
{
	palm_domain_revoke_task(task);
}
