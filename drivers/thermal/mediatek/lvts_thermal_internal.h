/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __MEDIATEK_LVTS_THERMAL_INTERNAL_H
#define __MEDIATEK_LVTS_THERMAL_INTERNAL_H

#include <linux/bits.h>
#include <linux/errno.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/types.h>
#include <linux/unaligned.h>

#define LVTS_MT6789_NUM_SENSORS		13
#define LVTS_MT6789_NUM_CONTROLLERS	4
#define LVTS_MT6789_EFUSE_SIZE		48
#define LVTS_MT6789_GOLDEN_TEMP_MAX	62
#define LVTS_MT6789_COEFF_A		(-250460)
#define LVTS_MT6789_COEFF_B		250460

struct lvts_mt6789_calibration {
	u16 count_r[LVTS_MT6789_NUM_SENSORS];
	u32 count_rc[LVTS_MT6789_NUM_CONTROLLERS];
	u8 golden_temp;
};

static inline int lvts_mt6789_decode_calibration(const u8 *efuse, size_t len,
						 struct lvts_mt6789_calibration *cal)
{
	int i;

	if (!efuse || !cal || len != LVTS_MT6789_EFUSE_SIZE)
		return -EINVAL;

	/* Golden temperature is word 0 bits 31:24. */
	cal->golden_temp = efuse[3];
	if (!cal->golden_temp ||
	    cal->golden_temp >= LVTS_MT6789_GOLDEN_TEMP_MAX)
		return -ENODATA;

	/* Thirteen 16-bit count_r values occupy words 1 through 7. */
	for (i = 0; i < LVTS_MT6789_NUM_SENSORS; i++) {
		cal->count_r[i] = get_unaligned_le16(efuse + 4 + i * 2);
		if (!cal->count_r[i])
			return -ENODATA;
	}

	/* One 24-bit count_rc value per controller occupies words 8-11. */
	for (i = 0; i < LVTS_MT6789_NUM_CONTROLLERS; i++) {
		cal->count_rc[i] = get_unaligned_le32(efuse + 32 + i * 4) &
					GENMASK(23, 0);
		if (!cal->count_rc[i])
			return -ENODATA;
	}

	return 0;
}

static inline int lvts_mt6789_raw_to_temp(u32 raw, u8 golden_temp)
{
	return ((s64)LVTS_MT6789_COEFF_A * (raw & 0xffff) >> 14) +
		golden_temp * 500 + LVTS_MT6789_COEFF_B;
}

static inline bool lvts_mt6789_count_rc_needs_refine(u32 reference,
						     u32 measured)
{
	u32 larger, smaller;

	if (!reference || !measured)
		return true;

	larger = max(reference, measured);
	smaller = min(reference, measured);

	/* The shipped MT6789 driver refines ratios strictly above 1.061. */
	return div_u64((u64)larger * 1000, smaller) > 1061;
}

#endif /* __MEDIATEK_LVTS_THERMAL_INTERNAL_H */
