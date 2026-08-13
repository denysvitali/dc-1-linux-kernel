// SPDX-License-Identifier: GPL-2.0-only
#include <kunit/test.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#include "lvts_thermal_internal.h"

static void lvts_mt6789_make_valid_efuse(u8 *efuse)
{
	int i;

	memset(efuse, 0, LVTS_MT6789_EFUSE_SIZE);
	efuse[3] = 60;

	for (i = 0; i < LVTS_MT6789_NUM_SENSORS; i++)
		put_unaligned_le16(35000 + i, efuse + 4 + i * 2);

	for (i = 0; i < LVTS_MT6789_NUM_CONTROLLERS; i++)
		put_unaligned_le32(2700 + i, efuse + 32 + i * 4);
}

static void lvts_mt6789_decode_offsets_test(struct kunit *test)
{
	struct lvts_mt6789_calibration cal;
	u8 efuse[LVTS_MT6789_EFUSE_SIZE];
	int i, ret;

	lvts_mt6789_make_valid_efuse(efuse);

	ret = lvts_mt6789_decode_calibration(efuse, sizeof(efuse), &cal);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, cal.golden_temp, (u8)60);
	for (i = 0; i < LVTS_MT6789_NUM_SENSORS; i++)
		KUNIT_EXPECT_EQ(test, cal.count_r[i], (u16)(35000 + i));
	for (i = 0; i < LVTS_MT6789_NUM_CONTROLLERS; i++)
		KUNIT_EXPECT_EQ(test, cal.count_rc[i], (u32)(2700 + i));
}

static void lvts_mt6789_invalid_calibration_test(struct kunit *test)
{
	struct lvts_mt6789_calibration cal;
	u8 efuse[LVTS_MT6789_EFUSE_SIZE];
	int ret;

	lvts_mt6789_make_valid_efuse(efuse);
	ret = lvts_mt6789_decode_calibration(efuse, sizeof(efuse) - 1, &cal);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	ret = lvts_mt6789_decode_calibration(NULL, sizeof(efuse), &cal);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);

	lvts_mt6789_make_valid_efuse(efuse);
	efuse[3] = 0;
	ret = lvts_mt6789_decode_calibration(efuse, sizeof(efuse), &cal);
	KUNIT_EXPECT_EQ(test, ret, -ENODATA);

	lvts_mt6789_make_valid_efuse(efuse);
	efuse[3] = LVTS_MT6789_GOLDEN_TEMP_MAX;
	ret = lvts_mt6789_decode_calibration(efuse, sizeof(efuse), &cal);
	KUNIT_EXPECT_EQ(test, ret, -ENODATA);

	lvts_mt6789_make_valid_efuse(efuse);
	put_unaligned_le16(0, efuse + 4 + 6 * 2);
	ret = lvts_mt6789_decode_calibration(efuse, sizeof(efuse), &cal);
	KUNIT_EXPECT_EQ(test, ret, -ENODATA);

	lvts_mt6789_make_valid_efuse(efuse);
	put_unaligned_le32(0, efuse + 32 + 2 * 4);
	ret = lvts_mt6789_decode_calibration(efuse, sizeof(efuse), &cal);
	KUNIT_EXPECT_EQ(test, ret, -ENODATA);
}

static void lvts_mt6789_conversion_test(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, lvts_mt6789_raw_to_temp(0, 60), 280460);
	KUNIT_EXPECT_EQ(test, lvts_mt6789_raw_to_temp(BIT(14), 60), 30000);
}

static void lvts_mt6789_count_rc_rule_test(struct kunit *test)
{
	KUNIT_EXPECT_FALSE(test, lvts_mt6789_count_rc_needs_refine(2700, 2700));
	KUNIT_EXPECT_FALSE(test, lvts_mt6789_count_rc_needs_refine(2700, 2800));
	KUNIT_EXPECT_FALSE(test, lvts_mt6789_count_rc_needs_refine(2700, 2867));
	KUNIT_EXPECT_TRUE(test, lvts_mt6789_count_rc_needs_refine(2700, 2868));
	KUNIT_EXPECT_TRUE(test, lvts_mt6789_count_rc_needs_refine(2700, 3000));
	KUNIT_EXPECT_TRUE(test, lvts_mt6789_count_rc_needs_refine(2700, 0));
}

static struct kunit_case lvts_mt6789_test_cases[] = {
	KUNIT_CASE(lvts_mt6789_decode_offsets_test),
	KUNIT_CASE(lvts_mt6789_invalid_calibration_test),
	KUNIT_CASE(lvts_mt6789_conversion_test),
	KUNIT_CASE(lvts_mt6789_count_rc_rule_test),
	{}
};

static struct kunit_suite lvts_mt6789_test_suite = {
	.name = "lvts-mt6789",
	.test_cases = lvts_mt6789_test_cases,
};

kunit_test_suite(lvts_mt6789_test_suite);

MODULE_LICENSE("GPL");
