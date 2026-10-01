// SPDX-License-Identifier: GPL-2.0-only
/* Shallow WFI and suspend-to-idle timekeeping for the Daylight DC-1. */

#include <linux/cpuidle.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>

#include <asm/cpuidle.h>

static __cpuidle int jagar_enter_wfi(struct cpuidle_device *dev,
				     struct cpuidle_driver *drv, int index)
{
	/* Keep interrupts masked for the s2idle core's tick_freeze() pair. */
	cpu_do_idle();
	return index;
}

static struct cpuidle_driver jagar_idle_driver = {
	.name = "jagar_wfi",
	.owner = THIS_MODULE,
	.states = {
		{
			.enter = jagar_enter_wfi,
			.exit_latency = 1,
			.target_residency = 1,
			.name = "WFI",
			.desc = "Architectural WFI",
		},
		{
			/*
			 * The s2idle core excludes fallback state zero. This
			 * second state uses the same WFI, with enter_s2idle
			 * allowing the core to freeze ticks and timekeeping.
			 * Ordinary idle selection still executes only WFI.
			 */
			.enter = jagar_enter_wfi,
			.enter_s2idle = jagar_enter_wfi,
			.exit_latency = 1,
			.target_residency = 1,
			.name = "WFI-S2",
			.desc = "WFI with s2idle timekeeping",
		},
	},
	.safe_state_index = 0,
	.state_count = 2,
};

static int __init jagar_cpuidle_init(void)
{
	int ret;

	if (!of_machine_is_compatible("daylight,jagar"))
		return -ENODEV;

	ret = cpuidle_register(&jagar_idle_driver, NULL);
	if (ret)
		return ret;

	pr_info("jagar cpuidle: registered WFI with s2idle timekeeping\n");
	return 0;
}
device_initcall(jagar_cpuidle_init);
