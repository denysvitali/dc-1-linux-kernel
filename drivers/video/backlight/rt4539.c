// SPDX-License-Identifier: GPL-2.0-only
/*
 * Richtek RT4539 I2C backlight driver
 *
 * Brightness is a 12-bit value latched by a consecutive write to registers
 * 0x04 and 0x05.  The MTP register at 0xff is deliberately outside this
 * driver's regmap so no kernel or userspace path can program nonvolatile data.
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
#include <linux/property.h>
#include <linux/regmap.h>

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

struct rt4539 {
	struct device *dev;
	struct regmap *regmap;
	struct gpio_desc *enable_gpio;
	struct backlight_device *backlight;
	/* Serializes register writes with power and shutdown transitions. */
	struct mutex lock;
	u32 max_brightness;
	u32 max_hw_brightness;
	u32 min_hw_brightness;
	u32 enable_post_delay_ms;
	u32 off_settle_delay_ms;
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
	static const u16 slope_delay_ms[] = { 0, 1, 8, 128, 256, 512, 768, 1024 };
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
	return 0;

fail:
	gpiod_set_value_cansleep(rt->enable_gpio, 0);
	return ret;
}

static int rt4539_disable(struct rt4539 *rt)
{
	int first_error = 0;
	int ret;

	if (!rt->enabled)
		return 0;

	ret = regmap_write(rt->regmap, RT4539_REG_TIME_CTRL,
			   rt->off_time_ctrl);
	if (ret)
		first_error = ret;
	ret = rt4539_write_brightness(rt, 0);
	if (ret && !first_error)
		first_error = ret;
	if (!first_error && rt->off_settle_delay_ms)
		msleep(rt->off_settle_delay_ms);
	ret = regmap_update_bits(rt->regmap, RT4539_REG_ENABLE,
				 RT4539_BL_ENABLE, 0);
	if (ret && !first_error)
		first_error = ret;

	/* EN low is the fail-safe even if the fade transaction failed. */
	gpiod_set_value_cansleep(rt->enable_gpio, 0);
	rt->enabled = false;
	return first_error;
}

static void rt4539_force_off(struct rt4539 *rt)
{
	if (!rt->enabled)
		return;

	rt4539_write_brightness(rt, 0);
	regmap_update_bits(rt->regmap, RT4539_REG_ENABLE,
			   RT4539_BL_ENABLE, 0);
	gpiod_set_value_cansleep(rt->enable_gpio, 0);
	rt->enabled = false;
}

static int rt4539_backlight_update_status(struct backlight_device *backlight)
{
	struct rt4539 *rt = bl_get_data(backlight);
	u32 brightness = backlight_get_brightness(backlight);
	u32 hardware_brightness;
	int ret;

	mutex_lock(&rt->lock);
	if (!brightness) {
		ret = rt4539_disable(rt);
		goto out;
	}

	hardware_brightness = DIV_ROUND_CLOSEST_ULL((u64)brightness *
						    rt->max_hw_brightness,
						    backlight->props.max_brightness);
	hardware_brightness = clamp_t(u32, hardware_brightness,
				      rt->min_hw_brightness,
				      rt->max_hw_brightness);

	if (!rt->enabled)
		ret = rt4539_prepare(rt, hardware_brightness);
	else
		ret = rt4539_write_brightness(rt, hardware_brightness);
	if (ret && rt->enabled) {
		gpiod_set_value_cansleep(rt->enable_gpio, 0);
		rt->enabled = false;
	}

out:
	mutex_unlock(&rt->lock);
	return ret;
}

static const struct backlight_ops rt4539_backlight_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = rt4539_backlight_update_status,
};

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
	props.max_brightness = rt->max_brightness;
	props.brightness = 0;
	rt->backlight = devm_backlight_device_register(dev, label, dev, rt,
						       &rt4539_backlight_ops,
						       &props);
	if (IS_ERR(rt->backlight))
		return dev_err_probe(dev, PTR_ERR(rt->backlight),
				     "failed to register backlight\n");

	i2c_set_clientdata(client, rt);
	dev_info(dev, "registered %s, off until first brightness request\n",
		 label);
	return 0;
}

static void rt4539_remove(struct i2c_client *client)
{
	struct rt4539 *rt = i2c_get_clientdata(client);

	mutex_lock(&rt->lock);
	rt4539_force_off(rt);
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
	},
	.probe = rt4539_probe,
	.remove = rt4539_remove,
	.shutdown = rt4539_shutdown,
};
module_i2c_driver(rt4539_driver);

MODULE_AUTHOR("Denys Vitali <denys@denv.it>");
MODULE_DESCRIPTION("Richtek RT4539 backlight driver");
MODULE_LICENSE("GPL");
