// SPDX-License-Identifier: GPL-2.0-only
/*
 * Richtek RT4539 I2C backlight driver
 *
 * Brightness is a 12-bit value latched by a consecutive write to registers
 * 0x04 and 0x05.  The MTP register at 0xff is deliberately outside this
 * driver's regmap so no kernel or userspace path can program nonvolatile data.
 *
 * Boards with a cool and a warm frontlight carry two of these chips.  When a
 * node labelled "<label>-amber" accompanies the node labelled "<label>", the
 * pair is exposed as the single backlight device "<label>": its brightness is
 * the total luminance and the "warmth" attribute of the cool chip's I2C device
 * selects the mix.  Both chips are then updated by one brightness request, so
 * no intermediate colour is ever displayed.  The warm chip registers no
 * backlight device of its own.
 */

#include <linux/backlight.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>

#define RT4539_REG_DIMMING		0x00
#define RT4539_REG_BOOST		0x01
#define RT4539_REG_CURRENT		0x02
#define RT4539_REG_BRIGHTNESS_CFG	0x03
#define RT4539_REG_BRIGHTNESS_MSB	0x04
#define RT4539_REG_TIME_CTRL		0x06
#define RT4539_REG_CONTROL		0x07
#define RT4539_REG_SOFT_START		0x08
#define RT4539_REG_PFM			0x09
#define RT4539_REG_ENABLE		0x0b

#define RT4539_DIMMING_MASK		0xef
#define RT4539_BOOST_MASK		0x1f
#define RT4539_CONTROL_MASK		0xe3
#define RT4539_SOFT_START_MASK		0xe3
#define RT4539_PFM_MASK			0x7f
#define RT4539_FB_ENABLE_MASK		GENMASK(6, 1)
#define RT4539_BL_ENABLE		BIT(7)
#define RT4539_RESOLUTION_MASK		GENMASK(2, 0)
#define RT4539_RESOLUTION_12_BIT	0x04
#define RT4539_MAX_HW_BRIGHTNESS	0x0fff

#define RT4539_WARM_LABEL_SUFFIX	"-amber"
#define RT4539_MAX_WARMTH		255

/* Slope codes live in TIME_CTRL's low three bits; see slope_delay_ms[]. */
#define RT4539_SLOPE_1_MS		1
#define RT4539_SLOPE_8_MS		2
#define RT4539_SLOPE_128_MS		3
#define RT4539_SLOPE_MASK		7
#define RT4539_MAX_PROFILE_STEPS	8

struct rt4539 {
	struct device *dev;
	struct regmap *regmap;
	struct gpio_desc *enable_gpio;
	struct backlight_device *backlight;
	/* Warm chip mixed into this backlight; NULL for a single-chip device. */
	struct rt4539 *warm;
	/*
	 * Serializes register writes with power and shutdown transitions.  A
	 * paired update takes the cool chip's lock, then the warm chip's.
	 */
	struct mutex lock;
	/* Completes a fade started by a lit backlight; see rt4539_apply(). */
	struct delayed_work off_work;
	unsigned long off_deadline;
	bool fading;
	bool removed;
	u32 warmth;
	u32 hw_brightness;
	u32 max_brightness;
	u32 max_hw_brightness;
	u32 min_hw_brightness;
	u32 enable_post_delay_ms;
	u32 off_settle_delay_ms;
	u32 profile_steps;
	u32 profile_step_ms;
	u8 fb_enable;
	u8 led_current;
	u8 control;
	u8 time_ctrl;
	u8 off_time_ctrl;
	u8 dimming;
	u8 boost;
	u8 soft_start;
	u8 pfm;
	bool enabled;
};

/* TIME_CTRL slope code to fade time, indexed by the code's low three bits. */
static const u16 slope_delay_ms[] = { 0, 1, 8, 128, 256, 512, 768, 1024 };

static const struct regmap_config rt4539_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = RT4539_REG_ENABLE,
	.cache_type = REGCACHE_NONE,
};

static int rt4539_read_required_u32(struct device *dev, const char *name,
				    u32 max, u32 *value)
{
	int ret;

	ret = device_property_read_u32(dev, name, value);
	if (ret)
		return dev_err_probe(dev, ret, "missing %s property\n", name);
	if (*value > max)
		return dev_err_probe(dev, -ERANGE,
				     "%s value %u exceeds %u\n",
				     name, *value, max);

	return 0;
}

static int rt4539_parse_properties(struct rt4539 *rt)
{
	struct device *dev = rt->dev;
	const char *default_state;
	u32 fbs[6];
	u32 value, pwm_sample_rate = 0;
	u32 off_safety_us, off_extra_us;
	size_t i;
	int count, ret;

	ret = device_property_read_string(dev, "default-state", &default_state);
	if (ret || strcmp(default_state, "off"))
		return dev_err_probe(dev, ret ?: -EINVAL,
				     "default-state must be off\n");

	ret = rt4539_read_required_u32(dev, "max-brightness", INT_MAX,
				       &rt->max_brightness);
	if (ret)
		return ret;
	if (!rt->max_brightness)
		return dev_err_probe(dev, -EINVAL,
				     "max-brightness must be nonzero\n");

	ret = rt4539_read_required_u32(dev, "max-hw-brightness",
				       RT4539_MAX_HW_BRIGHTNESS,
				       &rt->max_hw_brightness);
	if (ret)
		return ret;
	if (!rt->max_hw_brightness)
		return dev_err_probe(dev, -EINVAL,
				     "max-hw-brightness must be nonzero\n");

	ret = rt4539_read_required_u32(dev,
				       "hw-brightness-on-threshold",
				       rt->max_hw_brightness,
				       &rt->min_hw_brightness);
	if (ret)
		return ret;
	if (!rt->min_hw_brightness)
		return dev_err_probe(dev, -EINVAL,
				     "hw-brightness-on-threshold must be nonzero\n");

	ret = rt4539_read_required_u32(dev, "enable-post-delay", 1000,
				       &rt->enable_post_delay_ms);
	if (ret)
		return ret;

	ret = rt4539_read_required_u32(dev, "current", U8_MAX, &value);
	if (ret)
		return ret;
	rt->led_current = value;

	ret = rt4539_read_required_u32(dev, "control", 3, &value);
	if (ret)
		return ret;
	if (value != 1)
		return dev_err_probe(dev, -EINVAL,
				     "only I2C control mode 1 is supported\n");
	rt->control = value;

	if (device_property_present(dev, "pwm-sample-rate")) {
		ret = device_property_read_u32(dev, "pwm-sample-rate",
					       &pwm_sample_rate);
		if (ret)
			return dev_err_probe(dev, ret,
					     "invalid pwm-sample-rate property\n");
		if (pwm_sample_rate > 7)
			return dev_err_probe(dev, -ERANGE,
					     "pwm-sample-rate exceeds 7\n");
	}
	rt->control |= pwm_sample_rate << 5;

	ret = rt4539_read_required_u32(dev, "time-ctrl", U8_MAX, &value);
	if (ret)
		return ret;
	rt->time_ctrl = value;

	/*
	 * Brightness and mix changes should reach the panel within a frame or
	 * two instead of the 128 ms stock slope.  Without a profile, time-ctrl
	 * stays the slope for every live change.
	 */
	if (device_property_present(dev, "brightness-profile")) {
		ret = rt4539_read_required_u32(dev, "brightness-profile",
					       RT4539_MAX_PROFILE_STEPS,
					       &rt->profile_steps);
		if (ret)
			return ret;
		if (!rt->profile_steps)
			return dev_err_probe(dev, -EINVAL,
					     "brightness-profile must be nonzero\n");
		ret = rt4539_read_required_u32(dev,
					       "brightness-profile-step-ms",
					       256, &rt->profile_step_ms);
		if (ret)
			return ret;
	}

	ret = rt4539_read_required_u32(dev,
				       "screen-off-backlight-off-time-ctrl",
				       U8_MAX, &value);
	if (ret)
		return ret;
	rt->off_time_ctrl = value;

	ret = rt4539_read_required_u32(dev, "soft-start-ctrl", 3, &value);
	if (ret)
		return ret;
	rt->soft_start = value;

	ret = rt4539_read_required_u32(dev, "clk-pfm-ctrl", RT4539_PFM_MASK,
				       &value);
	if (ret)
		return ret;
	rt->pfm = value;

	/* These optional raw fields are absent on jagar and therefore zero. */
	if (device_property_present(dev, "reg0x00")) {
		ret = device_property_read_u32(dev, "reg0x00", &value);
		if (ret)
			return dev_err_probe(dev, ret,
					     "invalid reg0x00 property\n");
		if (value & ~RT4539_DIMMING_MASK)
			return dev_err_probe(dev, -ERANGE,
					     "reg0x00 sets reserved bits\n");
		rt->dimming = value;
	}
	if (device_property_present(dev, "reg0x01")) {
		ret = device_property_read_u32(dev, "reg0x01", &value);
		if (ret)
			return dev_err_probe(dev, ret,
					     "invalid reg0x01 property\n");
		if (value & ~RT4539_BOOST_MASK)
			return dev_err_probe(dev, -ERANGE,
					     "reg0x01 sets reserved bits\n");
		rt->boost = value;
	}

	count = device_property_count_u32(dev, "fbs");
	if (count != 6)
		return dev_err_probe(dev, -EINVAL,
				     "fbs must contain exactly six entries\n");
	ret = device_property_read_u32_array(dev, "fbs", fbs,
					     ARRAY_SIZE(fbs));
	if (ret)
		return dev_err_probe(dev, ret, "failed to read fbs\n");
	for (i = 0; i < ARRAY_SIZE(fbs); i++) {
		if (fbs[i] > 1)
			return dev_err_probe(dev, -EINVAL,
					     "fbs entry %zu is not boolean\n", i);
		rt->fb_enable |= (u8)(fbs[i] << (i + 1));
	}
	if (!rt->fb_enable)
		return dev_err_probe(dev, -EINVAL,
				     "at least one feedback channel is required\n");

	ret = rt4539_read_required_u32(dev,
				       "screen-off-backlight-off-safety-delay-us",
				       1000000, &off_safety_us);
	if (ret)
		return ret;
	ret = rt4539_read_required_u32(dev,
				       "screen-off-backlight-off-extra-latency-us",
				       1000000, &off_extra_us);
	if (ret)
		return ret;

	value = (rt->off_time_ctrl >> 3) & 7;
	rt->off_settle_delay_ms = slope_delay_ms[value] +
		DIV_ROUND_UP(off_safety_us + off_extra_us, 1000);

	return 0;
}

static int rt4539_write_brightness(struct rt4539 *rt, u32 brightness)
{
	u8 values[2] = {
		(brightness >> 8) & 0x0f,
		brightness & 0xff,
	};

	return regmap_bulk_write(rt->regmap, RT4539_REG_BRIGHTNESS_MSB,
				 values, ARRAY_SIZE(values));
}

static int rt4539_prepare(struct rt4539 *rt, u32 brightness)
{
	unsigned int resolution;
	int ret;

	gpiod_set_value_cansleep(rt->enable_gpio, 1);
	if (rt->enable_post_delay_ms)
		msleep(rt->enable_post_delay_ms);

	ret = regmap_read(rt->regmap, RT4539_REG_BRIGHTNESS_CFG, &resolution);
	if (ret)
		goto fail;
	if ((resolution & RT4539_RESOLUTION_MASK) !=
	    RT4539_RESOLUTION_12_BIT) {
		ret = -EINVAL;
		dev_err(rt->dev, "unexpected brightness resolution 0x%02x\n",
			resolution);
		goto fail;
	}

	/* Match the shipped first-enable sequence, with BL_EN kept clear. */
	ret = regmap_update_bits(rt->regmap, RT4539_REG_ENABLE,
				 RT4539_FB_ENABLE_MASK | RT4539_BL_ENABLE,
				 rt->fb_enable);
	if (ret)
		goto fail;
	ret = regmap_write(rt->regmap, RT4539_REG_CURRENT, rt->led_current);
	if (ret)
		goto fail;
	ret = regmap_update_bits(rt->regmap, RT4539_REG_CONTROL,
				 RT4539_CONTROL_MASK, rt->control);
	if (ret)
		goto fail;
	ret = regmap_write(rt->regmap, RT4539_REG_TIME_CTRL, rt->time_ctrl);
	if (ret)
		goto fail;
	ret = regmap_update_bits(rt->regmap, RT4539_REG_DIMMING,
				 RT4539_DIMMING_MASK, rt->dimming);
	if (ret)
		goto fail;
	ret = regmap_update_bits(rt->regmap, RT4539_REG_BOOST,
				 RT4539_BOOST_MASK, rt->boost);
	if (ret)
		goto fail;
	ret = regmap_update_bits(rt->regmap, RT4539_REG_SOFT_START,
				 RT4539_SOFT_START_MASK, rt->soft_start);
	if (ret)
		goto fail;
	ret = regmap_update_bits(rt->regmap, RT4539_REG_PFM,
				 RT4539_PFM_MASK, rt->pfm);
	if (ret)
		goto fail;
	ret = rt4539_write_brightness(rt, brightness);
	if (ret)
		goto fail;
	ret = regmap_update_bits(rt->regmap, RT4539_REG_ENABLE,
				 RT4539_BL_ENABLE, RT4539_BL_ENABLE);
	if (ret)
		goto fail;

	rt->enabled = true;
	rt->hw_brightness = brightness;
	return 0;

fail:
	gpiod_set_value_cansleep(rt->enable_gpio, 0);
	return ret;
}

static void rt4539_fail_safe(struct rt4539 *rt)
{
	/* EN low is the fail-safe even if an I2C transaction failed. */
	gpiod_set_value_cansleep(rt->enable_gpio, 0);
	rt->enabled = false;
}

/* Start the screen-off fade.  Returns the time to wait before power-down. */
/*
 * Live slope from @from towards @to for @steps transitions.  Near the target
 * the 128 ms slope completes in a fraction of a step, so every step stays
 * under brightness-profile-step-ms; farther away the 8 ms slope never
 * overshoots a step and still lands well inside it.  Without a profile live
 * changes use time-ctrl.  The screen-off slope stays reserved for the fade.
 */
static u8 rt4539_live_slope(const struct rt4539 *rt, u32 from, u32 to,
			   u32 steps)
{
	u32 distance = from > to ? from - to : to - from;
	u32 near;

	if (!rt->profile_steps)
		return rt->time_ctrl;

	near = max_t(u32, rt->max_hw_brightness / steps, 1);
	if (distance <= near && slope_delay_ms[RT4539_SLOPE_128_MS] <=
	    rt->profile_step_ms)
		near = RT4539_SLOPE_128_MS;
	else
		near = RT4539_SLOPE_8_MS;

	if (slope_delay_ms[near] > rt->profile_step_ms)
		near = RT4539_SLOPE_1_MS;

	return (rt->time_ctrl & ~RT4539_SLOPE_MASK) | near;
}

/* Start the screen-off fade.  Returns the time to wait before power-down. */
static int rt4539_fade_out(struct rt4539 *rt, u32 *settle_ms)
{
	int first_error;
	int ret;

	first_error = regmap_write(rt->regmap, RT4539_REG_TIME_CTRL,
				   rt->off_time_ctrl);
	ret = rt4539_write_brightness(rt, 0);
	if (ret && !first_error)
		first_error = ret;
	if (!first_error)
		*settle_ms = max(*settle_ms, rt->off_settle_delay_ms);

	return first_error;
}

static int rt4539_power_down(struct rt4539 *rt)
{
	int ret;

	ret = regmap_update_bits(rt->regmap, RT4539_REG_ENABLE,
				 RT4539_BL_ENABLE, 0);
	rt4539_fail_safe(rt);
	return ret;
}

static void rt4539_force_off(struct rt4539 *rt)
{
	rt->fading = false;
	if (!rt->enabled)
		return;

	rt4539_write_brightness(rt, 0);
	regmap_update_bits(rt->regmap, RT4539_REG_ENABLE,
			   RT4539_BL_ENABLE, 0);
	gpiod_set_value_cansleep(rt->enable_gpio, 0);
	rt->enabled = false;
}

/*
 * Convert a brightness and a mixing share (out of RT4539_MAX_WARMTH) into the
 * chip's 12-bit range.  Zero means the chip is powered down.
 */
static u32 rt4539_scale(const struct rt4539 *rt, u32 brightness,
			u32 max_brightness, u32 share)
{
	u64 divisor = (u64)max_brightness * RT4539_MAX_WARMTH;
	u64 value;

	if (!brightness || !share)
		return 0;

	value = div64_u64((u64)brightness * share * rt->max_hw_brightness +
			  divisor / 2, divisor);
	return clamp_t(u64, value, rt->min_hw_brightness,
		       rt->max_hw_brightness);
}

static void rt4539_off_work(struct work_struct *work)
{
	struct rt4539 *rt = container_of(to_delayed_work(work), struct rt4539,
					 off_work);
	int ret;

	mutex_lock(&rt->lock);
	if (rt->fading && rt->enabled) {
		/* Requeued after a cancel: honour the fade still in progress. */
		if (time_before(jiffies, rt->off_deadline)) {
			schedule_delayed_work(&rt->off_work,
					      rt->off_deadline - jiffies);
		} else {
			rt->fading = false;
			ret = rt4539_power_down(rt);
			if (ret)
				dev_err(rt->dev, "deferred power-down failed: %d\n",
					ret);
		}
	}
	mutex_unlock(&rt->lock);
}

/*
 * Apply one hardware brightness per chip; all chip locks are held.  A chip
 * that needs its 20 ms power-up goes first so the already lit chips change
 * together with it.
 *
 * A chip switched off starts the screen-off fade at once.  With @sync (system
 * suspend) the chips share one fade interval and are powered down before
 * returning.  Otherwise the power-down is deferred to off_work, so a mix or
 * blank request returns in a few milliseconds instead of waiting ~290 ms for
 * the fade, and a chip lit again before the fade ends is simply re-lit.
 */
static int rt4539_apply(struct rt4539 *const chips[], const u32 targets[],
			unsigned int count, bool sync)
{
	bool power_down[2] = { };
	u32 settle_ms = 0;
	int first_error = 0;
	unsigned int i;
	int ret;

	for (i = 0; i < count; i++) {
		if (!targets[i] || chips[i]->enabled)
			continue;
		ret = rt4539_prepare(chips[i], targets[i]);
		if (ret && !first_error)
			first_error = ret;
	}

	for (i = 0; i < count; i++) {
		struct rt4539 *rt = chips[i];

		ret = 0;
		if (!rt->enabled)
			continue;
		if (!targets[i]) {
			if (sync && rt->fading) {
				/* off_work rechecks fading under the lock. */
				cancel_delayed_work(&rt->off_work);
				rt->fading = false;
				settle_ms = max(settle_ms,
						rt->off_settle_delay_ms);
				power_down[i] = true;
			}
		} else if (!targets[i]) {
			/*
			 * Restart the fade: the chip may be powered down only
			 * off_settle_delay_ms after this request, and cancelled fades
			 * keep running out while this one does.
			 */
			ret = rt4539_fade_out(rt, &settle_ms);
			if (!ret && rt->hw_brightness)
				rt->fading = true;
			if (ret) {
				rt4539_fail_safe(rt);
			} else if (sync) {
				power_down[i] = true;
			} else if (rt->fading) {
				rt->off_deadline = jiffies +
					msecs_to_jiffies(rt->off_settle_delay_ms);
				schedule_delayed_work(&rt->off_work,
					msecs_to_jiffies(rt->off_settle_delay_ms));
			}
			rt->hw_brightness = 0;
		} else {
			/*
			 * A cancelled power-down keeps running out from brightness 0;
			 * there is no need to touch the slope before relighting.
			 */
			rt->fading = false;
			ret = regmap_write(rt->regmap, RT4539_REG_TIME_CTRL,
					   rt4539_live_slope(rt, rt->hw_brightness,
							     targets[i],
							     rt->profile_steps));
			if (!ret && targets[i] != rt->hw_brightness)
				ret = rt4539_write_brightness(rt, targets[i]);
			if (ret)
				rt4539_fail_safe(rt);
			else
				rt->hw_brightness = targets[i];
		}
		if (ret && !first_error)
			first_error = ret;
	}

	if (settle_ms && (power_down[0] || power_down[1]))
		msleep(settle_ms);

	for (i = 0; i < count; i++) {
		if (!power_down[i])
			continue;
		ret = rt4539_power_down(chips[i]);
		if (ret && !first_error)
			first_error = ret;
	}

	return first_error;
}

static int rt4539_backlight_update_status(struct backlight_device *backlight)
{
	struct rt4539 *rt = bl_get_data(backlight);
	u32 max_brightness = backlight->props.max_brightness;
	u32 brightness = backlight_get_brightness(backlight);
	bool sync = backlight->props.state & BL_CORE_SUSPENDED;
	struct rt4539 *chips[2] = { rt, rt->warm };
	u32 targets[2];
	int ret;

	mutex_lock(&rt->lock);
	if (rt->removed) {
		ret = -ENODEV;
		goto out;
	}
	if (!rt->warm) {
		targets[0] = rt4539_scale(rt, brightness, max_brightness,
					  RT4539_MAX_WARMTH);
		ret = rt4539_apply(chips, targets, 1, sync);
		goto out;
	}

	/*
	 * Constant-sum crossfade.  The lower half of the warmth range raises
	 * the warm chip beside an untouched cool chip; the upper half dims the
	 * cool chip while the warm chip keeps rising, so the light turns warm
	 * instead of dying.  The cool chip is off only at maximum warmth.
	 */
	targets[0] = rt4539_scale(rt, brightness, max_brightness,
				  min(2 * (RT4539_MAX_WARMTH - rt->warmth),
				      RT4539_MAX_WARMTH));
	targets[1] = rt4539_scale(rt->warm, brightness, max_brightness,
				  2 * rt->warmth);

	mutex_lock_nested(&rt->warm->lock, SINGLE_DEPTH_NESTING);
	ret = rt4539_apply(chips, targets, 2, sync);
	mutex_unlock(&rt->warm->lock);

out:
	mutex_unlock(&rt->lock);
	return ret;
}

static const struct backlight_ops rt4539_backlight_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = rt4539_backlight_update_status,
};

static ssize_t warmth_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct rt4539 *rt = dev_get_drvdata(dev);
	u32 warmth;

	mutex_lock(&rt->lock);
	warmth = rt->warmth;
	mutex_unlock(&rt->lock);

	return sysfs_emit(buf, "%u\n", warmth);
}

static ssize_t warmth_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct rt4539 *rt = dev_get_drvdata(dev);
	u32 warmth;
	int ret;

	ret = kstrtou32(buf, 0, &warmth);
	if (ret)
		return ret;
	if (warmth > RT4539_MAX_WARMTH)
		return -EINVAL;

	mutex_lock(&rt->lock);
	rt->warmth = warmth;
	mutex_unlock(&rt->lock);

	/* Re-mix the current brightness under the backlight's update lock. */
	ret = backlight_update_status(rt->backlight);
	return ret ?: count;
}
static DEVICE_ATTR_RW(warmth);

static ssize_t max_warmth_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%u\n", RT4539_MAX_WARMTH);
}
static DEVICE_ATTR_RO(max_warmth);

static struct attribute *rt4539_attrs[] = {
	&dev_attr_warmth.attr,
	&dev_attr_max_warmth.attr,
	NULL
};

static umode_t rt4539_attr_is_visible(struct kobject *kobj,
				      struct attribute *attr, int index)
{
	struct rt4539 *rt = dev_get_drvdata(kobj_to_dev(kobj));

	return rt->warm ? attr->mode : 0;
}

static const struct attribute_group rt4539_group = {
	.attrs = rt4539_attrs,
	.is_visible = rt4539_attr_is_visible,
};

static const struct attribute_group *rt4539_groups[] = {
	&rt4539_group,
	NULL
};

static struct i2c_driver rt4539_driver;

/* Find the enabled RT4539 node carrying @label, with a reference held. */
static struct device_node *rt4539_find_node(const char *label)
{
	struct device_node *np;
	const char *other;

	for_each_compatible_node(np, NULL, "richtek,rt4539") {
		if (of_device_is_available(np) &&
		    !of_property_read_string(np, "label", &other) &&
		    !strcmp(other, label))
			return np;
	}

	return NULL;
}

/* True when @label names the warm half of a pair described in the tree. */
static bool rt4539_is_warm_channel(const char *label)
{
	size_t suffix = strlen(RT4539_WARM_LABEL_SUFFIX);
	size_t length = strlen(label);
	struct device_node *np;
	bool paired;
	char *cool;

	if (length <= suffix ||
	    strcmp(label + length - suffix, RT4539_WARM_LABEL_SUFFIX))
		return false;

	cool = kstrndup(label, length - suffix, GFP_KERNEL);
	if (!cool)
		return false;
	np = rt4539_find_node(cool);
	kfree(cool);
	paired = np;
	of_node_put(np);

	return paired;
}

/*
 * Bind the warm chip paired with @rt, if the tree describes one.  The device
 * link unbinds this driver from the cool chip before the warm chip can go
 * away, so rt->warm stays valid for as long as the backlight exists.
 */
static int rt4539_attach_warm_channel(struct rt4539 *rt, const char *label)
{
	struct device *dev = rt->dev;
	struct i2c_client *client;
	struct device_node *np;
	char *warm_label;
	int ret = -EPROBE_DEFER;

	warm_label = kasprintf(GFP_KERNEL, "%s" RT4539_WARM_LABEL_SUFFIX,
			       label);
	if (!warm_label)
		return -ENOMEM;
	np = rt4539_find_node(warm_label);
	kfree(warm_label);
	if (!np)
		return 0;

	client = of_find_i2c_device_by_node(np);
	of_node_put(np);
	if (!client)
		return dev_err_probe(dev, -EPROBE_DEFER,
				     "warm channel is not registered yet\n");

	device_lock(&client->dev);
	if (client->dev.driver == &rt4539_driver.driver &&
	    device_link_add(dev, &client->dev, DL_FLAG_AUTOREMOVE_CONSUMER)) {
		rt->warm = i2c_get_clientdata(client);
		ret = 0;
	}
	device_unlock(&client->dev);
	put_device(&client->dev);

	if (ret)
		return dev_err_probe(dev, ret, "warm channel is not bound yet\n");
	return 0;
}

static int rt4539_probe(struct i2c_client *client)
{
	struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.scale = BACKLIGHT_SCALE_LINEAR,
	};
	struct device *dev = &client->dev;
	struct rt4539 *rt;
	const char *label;
	int ret;

	rt = devm_kzalloc(dev, sizeof(*rt), GFP_KERNEL);
	if (!rt)
		return -ENOMEM;
	rt->dev = dev;
	mutex_init(&rt->lock);
	INIT_DELAYED_WORK(&rt->off_work, rt4539_off_work);

	rt->enable_gpio = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(rt->enable_gpio))
		return dev_err_probe(dev, PTR_ERR(rt->enable_gpio),
				     "failed to request enable GPIO\n");

	ret = rt4539_parse_properties(rt);
	if (ret)
		return ret;

	rt->regmap = devm_regmap_init_i2c(client, &rt4539_regmap_config);
	if (IS_ERR(rt->regmap))
		return dev_err_probe(dev, PTR_ERR(rt->regmap),
				     "failed to initialize regmap\n");

	ret = device_property_read_string(dev, "label", &label);
	if (ret)
		label = dev_name(dev);

	i2c_set_clientdata(client, rt);
	if (rt4539_is_warm_channel(label)) {
		dev_info(dev, "%s is the warm channel of a paired backlight\n",
			 label);
		return 0;
	}

	ret = rt4539_attach_warm_channel(rt, label);
	if (ret)
		return ret;

	props.max_brightness = rt->max_brightness;
	props.brightness = 0;
	rt->backlight = devm_backlight_device_register(dev, label, dev, rt,
						       &rt4539_backlight_ops,
						       &props);
	if (IS_ERR(rt->backlight))
		return dev_err_probe(dev, PTR_ERR(rt->backlight),
				     "failed to register backlight\n");

	dev_info(dev, "registered %s%s, off until first brightness request\n",
		 label, rt->warm ? " with a warm channel" : "");
	return 0;
}

static void rt4539_remove(struct i2c_client *client)
{
	struct rt4539 *rt = i2c_get_clientdata(client);

	/*
	 * The backlight device outlives remove() until devres runs, so refuse
	 * later requests before stopping the deferred power-downs.
	 */
	mutex_lock(&rt->lock);
	rt->removed = true;
	mutex_unlock(&rt->lock);
	cancel_delayed_work_sync(&rt->off_work);
	if (rt->warm)
		cancel_delayed_work_sync(&rt->warm->off_work);

	mutex_lock(&rt->lock);
	rt4539_force_off(rt);
	if (rt->warm) {
		mutex_lock_nested(&rt->warm->lock, SINGLE_DEPTH_NESTING);
		rt4539_force_off(rt->warm);
		mutex_unlock(&rt->warm->lock);
	}
	mutex_unlock(&rt->lock);
}

static void rt4539_shutdown(struct i2c_client *client)
{
	rt4539_remove(client);
}

static const struct of_device_id rt4539_of_match[] = {
	{ .compatible = "richtek,rt4539" },
	{ }
};
MODULE_DEVICE_TABLE(of, rt4539_of_match);

static struct i2c_driver rt4539_driver = {
	.driver = {
		.name = "rt4539",
		.of_match_table = rt4539_of_match,
		.dev_groups = rt4539_groups,
	},
	.probe = rt4539_probe,
	.remove = rt4539_remove,
	.shutdown = rt4539_shutdown,
};
module_i2c_driver(rt4539_driver);

MODULE_AUTHOR("Denys Vitali <denys@denv.it>");
MODULE_DESCRIPTION("Richtek RT4539 backlight driver");
MODULE_LICENSE("GPL");
