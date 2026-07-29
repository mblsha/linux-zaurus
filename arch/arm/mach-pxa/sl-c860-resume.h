/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __SL_C860_RESUME_H
#define __SL_C860_RESUME_H

#include <linux/types.h>

#include <asm/suspend.h>

extern u8 sl_c860_resume_trampoline[];
extern u8 sl_c860_resume_trampoline_context_cell[];
extern u8 sl_c860_resume_trampoline_cpu_do_resume[];
extern u8 sl_c860_resume_trampoline_end[];
extern int sl_c860_pxa25x_finish_suspend(unsigned long mode);
extern struct sleep_save_sp sleep_save_sp;

#endif
