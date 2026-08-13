/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef __RTC_S35390A_INTERNAL_H
#define __RTC_S35390A_INTERNAL_H

#include <linux/bits.h>
#include <linux/rtc.h>
#include <linux/types.h>

/* STATUS1 */
#define S35390A_FLAG_INT2	BIT(2)

/* STATUS2 INT2 pin output mode */
#define S35390A_INT2_MODE_MASK		GENMASK(3, 1)
#define S35390A_INT2_MODE_NOINTR	0
#define S35390A_INT2_MODE_ALARM		BIT(1)

static inline u8 s35390a_status2_alarm_mode(u8 status2, bool enabled)
{
	status2 &= ~S35390A_INT2_MODE_MASK;
	if (enabled)
		status2 |= S35390A_INT2_MODE_ALARM;

	return status2;
}

static inline unsigned long s35390a_status1_alarm_events(u8 status1)
{
	return status1 & S35390A_FLAG_INT2 ? RTC_IRQF | RTC_AF : 0;
}

#endif /* __RTC_S35390A_INTERNAL_H */
