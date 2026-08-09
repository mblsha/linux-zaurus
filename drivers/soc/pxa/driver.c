// SPDX-License-Identifier: GPL-2.0-only
#include <linux/export.h>
#include <linux/notifier.h>

#include <linux/soc/pxa/driver.h>

static BLOCKING_NOTIFIER_HEAD(pxa_cpufreq_transition_chain);

int pxa_cpufreq_register_transition_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&pxa_cpufreq_transition_chain, nb);
}
EXPORT_SYMBOL_GPL(pxa_cpufreq_register_transition_notifier);

int pxa_cpufreq_unregister_transition_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&pxa_cpufreq_transition_chain,
						   nb);
}
EXPORT_SYMBOL_GPL(pxa_cpufreq_unregister_transition_notifier);

int pxa_cpufreq_notify_transition(unsigned long event,
				  struct pxa_cpufreq_transition *transition)
{
	return blocking_notifier_call_chain(&pxa_cpufreq_transition_chain, event,
						    transition);
}
EXPORT_SYMBOL_GPL(pxa_cpufreq_notify_transition);
