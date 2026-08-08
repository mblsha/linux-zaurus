/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __ASM_ARM_PALM_DOMAIN_H
#define __ASM_ARM_PALM_DOMAIN_H

#include <asm/domain.h>

struct task_struct;
struct thread_info;

static inline void arm_palm_domain_sanitize_child(struct thread_info *thread)
{
#ifdef CONFIG_ARM_PALM_DOMAIN
	thread->cpu_domain &= ~domain_mask(DOMAIN_PALM_STORAGE);
#endif
}

#ifdef CONFIG_ARM_PALM_DOMAIN
void arm_palm_domain_flush_task(struct task_struct *task);
void arm_palm_domain_exit_task(struct task_struct *task);
#else
static inline void arm_palm_domain_flush_task(struct task_struct *task) { }
static inline void arm_palm_domain_exit_task(struct task_struct *task) { }
#endif

#endif /* __ASM_ARM_PALM_DOMAIN_H */
