// SPDX-License-Identifier: GPL-2.0-or-later
#include <kunit/test.h>
#include <linux/module.h>

#include "rtc-s35390a-internal.h"

static void s35390a_alarm_mode_test(struct kunit *test)
{
	u8 status2 = BIT(0) | GENMASK(3, 1) | BIT(6);

	KUNIT_EXPECT_EQ(test, s35390a_status2_alarm_mode(status2, false),
			(u8)(BIT(0) | BIT(6)));
	KUNIT_EXPECT_EQ(test, s35390a_status2_alarm_mode(status2, true),
			(u8)(BIT(0) | BIT(1) | BIT(6)));
}

static void s35390a_alarm_events_test(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, s35390a_status1_alarm_events(0), 0UL);
	KUNIT_EXPECT_EQ(test, s35390a_status1_alarm_events(S35390A_FLAG_INT2),
			(unsigned long)(RTC_IRQF | RTC_AF));
	KUNIT_EXPECT_EQ(test,
			s35390a_status1_alarm_events(S35390A_FLAG_INT2 | BIT(7)),
			(unsigned long)(RTC_IRQF | RTC_AF));
}

static struct kunit_case s35390a_test_cases[] = {
	KUNIT_CASE(s35390a_alarm_mode_test),
	KUNIT_CASE(s35390a_alarm_events_test),
	{}
};

static struct kunit_suite s35390a_test_suite = {
	.name = "rtc-s35390a",
	.test_cases = s35390a_test_cases,
};

kunit_test_suite(s35390a_test_suite);

MODULE_LICENSE("GPL");
