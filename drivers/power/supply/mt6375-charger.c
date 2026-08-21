// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6375 PMU charger: telemetry plus minimal input-current policy.
 *
 * MT6375 is a multi-address device.  The PMU bank is exposed at 0x34 and the
 * charger registers occupy offsets 0x20..0xe1 in that bank.  Nothing binds
 * this platform's Type-C/PD banks, so there is no port negotiation: the
 * bootloader leaves a static configuration (a 500 mA input current limit
 * here) and the kernel inherits it.  At 500 mA the DC-1 cannot even hold
 * charge under desktop load -- measured 2026-08-21, the pack discharged
 * ~250 mA while plugged in until the input limit was raised over i2c.
 *
 * The driver therefore does four things, and deliberately no more:
 *
 *   - reports the charger state machine and the programmed limits,
 *   - raises the input current limit from the 500 mA bootloader default to
 *     1.5 A once VBUS appears (USB-C sources advertise at least that much
 *     without PD, and the 4.5 V MIVR regulation folds the current back if
 *     the source sags, so a weaker port simply yields less),
 *   - raises the fast-charge current target from the bootloader's 500 mA to
 *     2 A at the same time -- a trickle by this pack's standards, still
 *     bounded by whatever the input regulation can sustain,
 *   - exposes both as writable power_supply properties so userspace can
 *     override either value.
 *
 * It does not reset the charger, acknowledge interrupts, enable its
 * watchdog, or touch the Type-C/PD banks.  The constant-charge *voltage*
 * stays read-only: it is the cell-level protection, and the BQ78Z100 pack
 * monitor that would normally enforce it does not answer on this hardware.
 */

#include <linux/bitops.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#define MT6375_REG_DEV_INFO		0x00
#define MT6375_REG_CHG_TOP1		0x20
#define MT6375_REG_CHG_AICR		0x22
#define MT6375_REG_CHG_MIVR		0x23
#define MT6375_REG_CHG_VCHG		0x25
#define MT6375_REG_CHG_ICHG		0x26
#define MT6375_REG_CHG_WDT		0x2a
#define MT6375_REG_CHG_STAT		0x34
#define MT6375_REG_CHG_STAT0		0xe0

#define MT6375_VENDOR_ID_MASK		GENMASK(7, 4)
#define MT6375_VENDOR_ID		0x70
#define MT6375_CHG_EN			BIT(0)
#define MT6375_PWR_RDY			BIT(0)
#define MT6375_WDT_EN			BIT(3)

/* CHG_AICR: 25 mA steps starting at 100 mA; codes below 2 mean less. */
#define MT6375_AICR_MIN_UA		100000
#define MT6375_AICR_STEP_UA		25000
#define MT6375_AICR_MIN_CODE		2
#define MT6375_AICR_MAX_CODE		127
/* What the bootloader leaves; used to detect "nobody tuned this yet". */
#define MT6375_AICR_BOOT_CODE		18	/* 500 mA */
/* First level a USB-C source can be assumed to sustain without PD. */
#define MT6375_AICR_BOOST_UA		1500000

/*
 * CHG_ICHG: 50 mA steps from 300 mA. The bootloader leaves 500 mA -- a
 * trickle by this pack's standards; the vendor stack's own fast-phase value
 * is 1.5 A, and on the DC-1's adapter a 2 A request measures ~1.8 A into the
 * pack (0.23C) with the input-voltage regulation still idle.
 */
#define MT6375_ICHG_MIN_UA		300000
#define MT6375_ICHG_STEP_UA		50000
#define MT6375_ICHG_MIN_CODE		6
#define MT6375_ICHG_MAX_CODE		63
#define MT6375_ICHG_BOOT_CODE		10	/* 500 mA */
#define MT6375_ICHG_BOOST_UA		2000000

#define MT6375_POLL_INTERVAL		msecs_to_jiffies(10000)

enum mt6375_charge_state {
	MT6375_CHG_SLEEP,
	MT6375_CHG_VBUS_READY,
	MT6375_CHG_TRICKLE,
	MT6375_CHG_PRECHARGE,
	MT6375_CHG_FAST,
	MT6375_CHG_EOC,
	MT6375_CHG_BACKGROUND,
	MT6375_CHG_DONE,
	MT6375_CHG_FAULT,
	MT6375_CHG_OTG = 15,
};

struct mt6375_charger {
	struct regmap *regmap;
	struct delayed_work poll_work;
	bool aicr_boosted;
	bool ichg_boosted;
};

static bool mt6375_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MT6375_REG_DEV_INFO:
	case MT6375_REG_CHG_TOP1:
	case MT6375_REG_CHG_AICR:
	case MT6375_REG_CHG_MIVR:
	case MT6375_REG_CHG_VCHG:
	case MT6375_REG_CHG_ICHG:
	case MT6375_REG_CHG_WDT:
	case MT6375_REG_CHG_STAT:
	case MT6375_REG_CHG_STAT0:
		return true;
	default:
		return false;
	}
}

/* Input limit and fast-charge target are ours; the CV stays bootloader's. */
static bool mt6375_writeable_reg(struct device *dev, unsigned int reg)
{
	return reg == MT6375_REG_CHG_AICR || reg == MT6375_REG_CHG_ICHG;
}

static const struct regmap_config mt6375_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
	.cache_type = REGCACHE_NONE,
	.readable_reg = mt6375_readable_reg,
	.writeable_reg = mt6375_writeable_reg,
};

static int mt6375_get_online(struct mt6375_charger *charger, int *online)
{
	unsigned int value;
	int ret;

	ret = regmap_read(charger->regmap, MT6375_REG_CHG_STAT0, &value);
	if (ret)
		return ret;

	*online = !!(value & MT6375_PWR_RDY);
	return 0;
}

static int mt6375_get_status(struct mt6375_charger *charger, int *status)
{
	unsigned int state, top1;
	int online;
	int ret;

	ret = mt6375_get_online(charger, &online);
	if (ret)
		return ret;
	if (!online) {
		*status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		return 0;
	}

	ret = regmap_read(charger->regmap, MT6375_REG_CHG_STAT, &state);
	if (ret)
		return ret;
	state &= GENMASK(3, 0);

	switch (state) {
	case MT6375_CHG_DONE:
		*status = POWER_SUPPLY_STATUS_FULL;
		return 0;
	case MT6375_CHG_FAULT:
		*status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		return 0;
	case MT6375_CHG_OTG:
		*status = POWER_SUPPLY_STATUS_DISCHARGING;
		return 0;
	case MT6375_CHG_SLEEP:
	case MT6375_CHG_VBUS_READY:
	case MT6375_CHG_TRICKLE:
	case MT6375_CHG_PRECHARGE:
	case MT6375_CHG_FAST:
	case MT6375_CHG_EOC:
	case MT6375_CHG_BACKGROUND:
		break;
	default:
		*status = POWER_SUPPLY_STATUS_UNKNOWN;
		return 0;
	}

	ret = regmap_read(charger->regmap, MT6375_REG_CHG_TOP1, &top1);
	if (ret)
		return ret;

	*status = top1 & MT6375_CHG_EN ? POWER_SUPPLY_STATUS_CHARGING :
					 POWER_SUPPLY_STATUS_NOT_CHARGING;
	return 0;
}

static int mt6375_read_linear(struct mt6375_charger *charger,
			      unsigned int reg, unsigned int mask,
			      unsigned int min, unsigned int step,
			      unsigned int min_code,
			      unsigned int max_code, int *micro)
{
	unsigned int value;
	int ret;

	ret = regmap_read(charger->regmap, reg, &value);
	if (ret)
		return ret;

	value &= mask;
	if (value < min_code || value > max_code)
		return -ERANGE;
	*micro = (min + step * (value - min_code)) * 1000;
	return 0;
}

static int mt6375_aicr_encode(int ua, unsigned int *code)
{
	unsigned int value;

	if (ua < MT6375_AICR_MIN_UA ||
	    ua > (MT6375_AICR_MAX_CODE - MT6375_AICR_MIN_CODE) *
			 MT6375_AICR_STEP_UA +
			 MT6375_AICR_MIN_UA)
		return -EINVAL;

	value = DIV_ROUND_CLOSEST(ua - MT6375_AICR_MIN_UA, MT6375_AICR_STEP_UA);
	*code = value + MT6375_AICR_MIN_CODE;
	return 0;
}

static int mt6375_ichg_encode(int ua, unsigned int *code)
{
	unsigned int value;

	if (ua < MT6375_ICHG_MIN_UA ||
	    ua > (MT6375_ICHG_MAX_CODE - MT6375_ICHG_MIN_CODE) *
			 MT6375_ICHG_STEP_UA +
			 MT6375_ICHG_MIN_UA)
		return -EINVAL;

	value = DIV_ROUND_CLOSEST(ua - MT6375_ICHG_MIN_UA, MT6375_ICHG_STEP_UA);
	*code = value + MT6375_ICHG_MIN_CODE;
	return 0;
}

static int mt6375_get_property(struct power_supply *psy,
			       enum power_supply_property property,
			       union power_supply_propval *val)
{
	struct mt6375_charger *charger = power_supply_get_drvdata(psy);

	switch (property) {
	case POWER_SUPPLY_PROP_STATUS:
		return mt6375_get_status(charger, &val->intval);
	case POWER_SUPPLY_PROP_ONLINE:
		return mt6375_get_online(charger, &val->intval);
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		return mt6375_read_linear(charger, MT6375_REG_CHG_AICR,
					   GENMASK(6, 0), 100, 25, 2, 127,
					   &val->intval);
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		return mt6375_read_linear(charger, MT6375_REG_CHG_MIVR,
					   GENMASK(6, 0), 3900, 100, 0, 95,
					   &val->intval);
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		return mt6375_read_linear(charger, MT6375_REG_CHG_ICHG,
					   GENMASK(5, 0), 300, 50, 6, 63,
					   &val->intval);
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		return mt6375_read_linear(charger, MT6375_REG_CHG_VCHG,
					   GENMASK(6, 0), 3900, 10, 0, 81,
					   &val->intval);
	default:
		return -EINVAL;
	}
}

static int mt6375_set_property(struct power_supply *psy,
			       enum power_supply_property property,
			       const union power_supply_propval *val)
{
	struct mt6375_charger *charger = power_supply_get_drvdata(psy);
	unsigned int code;
	int ret;

	switch (property) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = mt6375_aicr_encode(val->intval, &code);
		if (ret)
			return ret;
		ret = regmap_write(charger->regmap, MT6375_REG_CHG_AICR, code);
		if (!ret)
			charger->aicr_boosted = true;
		return ret;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = mt6375_ichg_encode(val->intval, &code);
		if (ret)
			return ret;
		ret = regmap_write(charger->regmap, MT6375_REG_CHG_ICHG, code);
		if (!ret)
			charger->ichg_boosted = true;
		return ret;
	default:
		return -EINVAL;
	}
}

static int mt6375_property_is_writeable(struct power_supply *psy,
					enum power_supply_property psp)
{
	return psp == POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT ||
	       psp == POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT;
}

static int mt6375_validate_limits(struct mt6375_charger *charger)
{
	int value;
	int ret;

	ret = mt6375_read_linear(charger, MT6375_REG_CHG_AICR,
				 GENMASK(6, 0), 100, 25, 2, 127, &value);
	if (ret)
		return ret;
	ret = mt6375_read_linear(charger, MT6375_REG_CHG_MIVR,
				 GENMASK(6, 0), 3900, 100, 0, 95, &value);
	if (ret)
		return ret;
	ret = mt6375_read_linear(charger, MT6375_REG_CHG_ICHG,
				 GENMASK(5, 0), 300, 50, 6, 63, &value);
	if (ret)
		return ret;
	return mt6375_read_linear(charger, MT6375_REG_CHG_VCHG,
				   GENMASK(6, 0), 3900, 10, 0, 81, &value);
}

static const enum power_supply_property mt6375_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
};

static const struct power_supply_desc mt6375_power_supply_desc = {
	.name = "mt6375-charger",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = mt6375_properties,
	.num_properties = ARRAY_SIZE(mt6375_properties),
	.get_property = mt6375_get_property,
	.set_property = mt6375_set_property,
	.property_is_writeable = mt6375_property_is_writeable,
};

static void mt6375_poll_work(struct work_struct *work)
{
	struct delayed_work *dw = to_delayed_work(work);
	struct mt6375_charger *charger =
		container_of(dw, struct mt6375_charger, poll_work);
	struct device *dev = regmap_get_device(charger->regmap);
	unsigned int stat0;
	bool pending = false;
	int ret;

	/*
	 * One-shot per register: each limit is raised at most once, and only
	 * while it still holds the bootloader's value. Anything userspace wrote
	 * through sysfs marks that register done and is never touched again.
	 */
	ret = regmap_read(charger->regmap, MT6375_REG_CHG_STAT0, &stat0);
	if (!ret && (stat0 & MT6375_PWR_RDY)) {
		unsigned int val, code;

		if (!charger->aicr_boosted) {
			ret = regmap_read(charger->regmap, MT6375_REG_CHG_AICR,
					  &val);
			if (ret) {
				pending = true;
			} else if ((val & GENMASK(6, 0)) !=
				   MT6375_AICR_BOOT_CODE) {
				charger->aicr_boosted = true;
			} else if (!mt6375_aicr_encode(MT6375_AICR_BOOST_UA,
						       &code) &&
				   !regmap_write(charger->regmap,
						 MT6375_REG_CHG_AICR, code)) {
				charger->aicr_boosted = true;
				dev_info(dev, "raised input current limit to %d mA for VBUS\n",
					 MT6375_AICR_BOOST_UA / 1000);
			} else {
				pending = true;
			}
		}

		if (!charger->ichg_boosted) {
			ret = regmap_read(charger->regmap, MT6375_REG_CHG_ICHG,
					  &val);
			if (ret) {
				pending = true;
			} else if ((val & GENMASK(5, 0)) !=
				   MT6375_ICHG_BOOT_CODE) {
				charger->ichg_boosted = true;
			} else if (!mt6375_ichg_encode(MT6375_ICHG_BOOST_UA,
						       &code) &&
				   !regmap_write(charger->regmap,
						 MT6375_REG_CHG_ICHG, code)) {
				charger->ichg_boosted = true;
				dev_info(dev, "raised fast-charge target to %d mA for VBUS\n",
					 MT6375_ICHG_BOOST_UA / 1000);
			} else {
				pending = true;
			}
		}
	} else if (!charger->aicr_boosted || !charger->ichg_boosted) {
		pending = true;
	}

	if (pending)
		schedule_delayed_work(&charger->poll_work,
				      MT6375_POLL_INTERVAL);
}

static void mt6375_remove(struct i2c_client *client)
{
	struct mt6375_charger *charger = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&charger->poll_work);
}

static int mt6375_probe(struct i2c_client *client)
{
	struct power_supply_config psy_cfg = {};
	struct mt6375_charger *charger;
	struct power_supply *psy;
	unsigned int id, watchdog;
	int ret;

	charger = devm_kzalloc(&client->dev, sizeof(*charger), GFP_KERNEL);
	if (!charger)
		return -ENOMEM;

	charger->regmap = devm_regmap_init_i2c(client, &mt6375_regmap_config);
	if (IS_ERR(charger->regmap))
		return dev_err_probe(&client->dev, PTR_ERR(charger->regmap),
				     "failed to initialize regmap\n");

	ret = regmap_read(charger->regmap, MT6375_REG_DEV_INFO, &id);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to read device ID\n");
	if ((id & MT6375_VENDOR_ID_MASK) != MT6375_VENDOR_ID)
		return dev_err_probe(&client->dev, -ENODEV,
				     "unexpected device ID 0x%02x\n", id);

	ret = regmap_read(charger->regmap, MT6375_REG_CHG_WDT, &watchdog);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to read charger watchdog\n");
	if (watchdog & MT6375_WDT_EN)
		return dev_err_probe(&client->dev, -EBUSY,
				     "charger watchdog is enabled\n");

	ret = mt6375_validate_limits(charger);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "invalid bootloader charger limits\n");

	psy_cfg.drv_data = charger;
	psy_cfg.fwnode = dev_fwnode(&client->dev);
	psy_cfg.no_wakeup_source = true;
	psy = devm_power_supply_register(&client->dev,
					 &mt6375_power_supply_desc, &psy_cfg);
	if (IS_ERR(psy))
		return dev_err_probe(&client->dev, PTR_ERR(psy),
				     "failed to register power supply\n");

	i2c_set_clientdata(client, charger);

	INIT_DELAYED_WORK(&charger->poll_work, mt6375_poll_work);
	schedule_delayed_work(&charger->poll_work, 0);

	dev_info(&client->dev, "charger telemetry and current-limit policy, revision %u\n",
		 (unsigned int)(id & GENMASK(3, 0)));
	return 0;
}

static const struct of_device_id mt6375_of_match[] = {
	{ .compatible = "mediatek,mt6375" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6375_of_match);

static struct i2c_driver mt6375_driver = {
	.driver = {
		.name = "mt6375-charger",
		.of_match_table = mt6375_of_match,
	},
	.probe = mt6375_probe,
	.remove = mt6375_remove,
};
module_i2c_driver(mt6375_driver);

MODULE_DESCRIPTION("MediaTek MT6375 charger telemetry and input-limit policy");
MODULE_LICENSE("GPL");
