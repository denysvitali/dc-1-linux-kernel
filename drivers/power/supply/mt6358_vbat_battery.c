// SPDX-License-Identifier: GPL-2.0-only
/*
 * Voltage-estimated battery for MT6358/MT6366-based boards whose real fuel
 * gauge is unreachable.
 *
 * On the Daylight DC-1 (jagar) the pack's TI BQ78Z100 sits on i2c7 at 0x55 and
 * NAKs its own address in every transaction shape, so bq27xxx registers a
 * power supply that answers -ENXIO to every read.  The MT6366 PMIC's BATADC
 * channel still measures the pack directly, so this driver turns that one
 * number into a battery the power-supply class can present: voltage, and a
 * state of charge interpolated from an open-circuit-voltage table.
 *
 * WHAT THIS IS NOT.  It is not a fuel gauge.  There is no coulomb counter, no
 * current measurement (MT6358's IBAT channel does not exist -- the AUXADC
 * driver hardcodes ibat = 0), no learned capacity and no temperature
 * compensation.  The reported capacity is a table lookup on a loaded terminal
 * voltage, so it sags under CPU/GPU load and reads high just after charging
 * stops.  Treat it as a coarse indicator, and delete this driver the day the
 * BQ78Z100 answers -- see the gauge-side investigation in the port notes.
 *
 * It binds an MFD cell of the MT6397-family core rather than a DT node, and
 * reaches the AUXADC through that MFD's regmap.  The stock DTB does describe
 * the vendor coulomb-counter block ("mediatek,mt6358-gauge"), and binding it
 * would be the natural thing to do, but that node's nvmem-cells point into RTC
 * spare registers no mainline driver provides, so fw_devlink parks any
 * consumer of it in deferred probe permanently.  Nothing in this driver
 * implements or touches the vendor gauge block either way.
 */

#include <linux/bitops.h>
#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/math64.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/slab.h>

/*
 * AUXADC registers, from drivers/iio/adc/mt6359-auxadc.c's mt6358 tables.
 * BATADC is request bit 0 of RQST0, averages 128 samples at 10 us each, and
 * lands in the first ADC result slot with a 3/1 divider in front of a 1800 mV
 * reference -- i.e. 5400 mV full scale over 15 bits.
 */
#define MT6358_AUXADC_ADC0		0x1088
#define MT6358_AUXADC_RQST0		0x1108
#define MT6358_BATADC_RQST_BIT		BIT(0)
#define MT6358_BATADC_SAMPLES		128
#define MT6358_BATADC_AVG_TIME_US	10
#define AUXADC_RDY_BIT			BIT(15)
#define AUXADC_VALUE_MASK		GENMASK(14, 0)
#define AUXADC_FULL_SCALE_MV		5400
#define AUXADC_RESOLUTION		32768
#define AUXADC_POLL_DELAY_US		100
#define AUXADC_TIMEOUT_US		32000

/* Don't drive a PMIC conversion for every sysfs read of every property. */
#define VBAT_CACHE_MS			2000
/* Exponential smoothing, new = (old * (N - 1) + sample) / N. */
#define VBAT_SMOOTHING			4

/* Fallbacks for a DT with no simple-battery node. */
#define DEFAULT_CHARGE_FULL_DESIGN_UAH	8000000
#define DEFAULT_VOLTAGE_MIN_DESIGN_UV	3400000

/*
 * Open-circuit voltage to state of charge, single-cell Li-ion, descending.
 *
 * The endpoints are the board's own numbers: the MT6375's bootloader-
 * programmed constant-charge voltage is 4.35 V and the stock DTB's
 * simple-battery node gives voltage-min-design = 3.4 V.  Between them this is
 * the ordinary Li-ion discharge shape -- flat from 3.7 V to 4.0 V, falling off
 * a cliff below 3.6 V -- not a curve measured from this pack.  Interpolation
 * between points is linear.
 */
struct vbat_ocv_point {
	unsigned int uv;
	unsigned int percent;
};

static const struct vbat_ocv_point vbat_ocv_table[] = {
	{ 4350000, 100 },
	{ 4250000,  93 },
	{ 4150000,  85 },
	{ 4060000,  75 },
	{ 3980000,  65 },
	{ 3920000,  55 },
	{ 3870000,  45 },
	{ 3830000,  35 },
	{ 3790000,  25 },
	{ 3750000,  17 },
	{ 3700000,  11 },
	{ 3650000,   7 },
	{ 3600000,   4 },
	{ 3500000,   1 },
	{ 3400000,   0 },
};

struct mt6358_vbat {
	struct device *dev;
	struct regmap *regmap;
	struct mutex lock;		/* serialises the sample + cache below */
	unsigned long sampled_at;	/* jiffies; 0 = never sampled */
	unsigned int voltage_uv;	/* smoothed */
	unsigned int charge_full_design_uah;
	unsigned int energy_full_design_uwh;
	unsigned int voltage_min_design_uv;
};

static int mt6358_vbat_sample(struct mt6358_vbat *vbat, unsigned int *uv)
{
	unsigned int val;
	int ret;

	/*
	 * The AUXADC arbitrates requests in hardware and each channel has its
	 * own result register, so this races with the IIO driver only in the
	 * sense that both may have a conversion in flight; neither corrupts
	 * the other's result.  No lock is shared with it because none is
	 * exported.
	 */
	ret = regmap_write(vbat->regmap, MT6358_AUXADC_RQST0,
			   MT6358_BATADC_RQST_BIT);
	if (ret)
		return ret;

	fsleep(MT6358_BATADC_SAMPLES * MT6358_BATADC_AVG_TIME_US);

	ret = regmap_read_poll_timeout(vbat->regmap, MT6358_AUXADC_ADC0, val,
				       val & AUXADC_RDY_BIT,
				       AUXADC_POLL_DELAY_US, AUXADC_TIMEOUT_US);
	if (ret)
		return ret;

	/*
	 * 64-bit on purpose: the largest 15-bit code times a 5 400 000 uV
	 * full scale is ~1.8e11, which overflows the int arithmetic that
	 * mult_frac() would do here.
	 */
	val &= AUXADC_VALUE_MASK;
	*uv = div_u64((u64)val * AUXADC_FULL_SCALE_MV * 1000, AUXADC_RESOLUTION);
	return 0;
}

static int mt6358_vbat_get_voltage(struct mt6358_vbat *vbat, unsigned int *uv)
{
	unsigned int sample;
	int ret = 0;

	guard(mutex)(&vbat->lock);

	if (vbat->sampled_at &&
	    time_before(jiffies, vbat->sampled_at + msecs_to_jiffies(VBAT_CACHE_MS)))
		goto out;

	ret = mt6358_vbat_sample(vbat, &sample);
	if (ret) {
		/* A stale reading beats no battery at all; report it. */
		if (vbat->sampled_at)
			ret = 0;
		goto out;
	}

	if (vbat->sampled_at)
		vbat->voltage_uv = (vbat->voltage_uv * (VBAT_SMOOTHING - 1) +
				    sample) / VBAT_SMOOTHING;
	else
		vbat->voltage_uv = sample;

	vbat->sampled_at = jiffies ? jiffies : 1;

out:
	*uv = vbat->voltage_uv;
	return ret;
}

static int mt6358_vbat_capacity(unsigned int uv)
{
	const struct vbat_ocv_point *hi, *lo;
	unsigned int i;

	if (uv >= vbat_ocv_table[0].uv)
		return 100;

	for (i = 1; i < ARRAY_SIZE(vbat_ocv_table); i++) {
		if (uv < vbat_ocv_table[i].uv)
			continue;

		hi = &vbat_ocv_table[i - 1];
		lo = &vbat_ocv_table[i];
		return lo->percent + mult_frac(uv - lo->uv,
					       hi->percent - lo->percent,
					       hi->uv - lo->uv);
	}

	return 0;
}

/*
 * Charge direction comes from the charger, which is the only part of the path
 * that actually knows it. Absent that driver the state stays UNKNOWN rather
 * than being guessed from a rising voltage.
 */
static int mt6358_vbat_status(int capacity)
{
	union power_supply_propval val;
	struct power_supply *charger;
	int ret;

	charger = power_supply_get_by_name("mt6375-charger");
	if (!charger)
		return POWER_SUPPLY_STATUS_UNKNOWN;

	ret = power_supply_get_property(charger, POWER_SUPPLY_PROP_ONLINE, &val);
	power_supply_put(charger);
	if (ret)
		return POWER_SUPPLY_STATUS_UNKNOWN;

	if (!val.intval)
		return POWER_SUPPLY_STATUS_DISCHARGING;

	return capacity >= 100 ? POWER_SUPPLY_STATUS_FULL :
				 POWER_SUPPLY_STATUS_CHARGING;
}

static int mt6358_vbat_capacity_level(int capacity)
{
	if (capacity >= 100)
		return POWER_SUPPLY_CAPACITY_LEVEL_FULL;
	if (capacity > 15)
		return POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
	if (capacity > 5)
		return POWER_SUPPLY_CAPACITY_LEVEL_LOW;
	return POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
}

static int mt6358_vbat_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct mt6358_vbat *vbat = power_supply_get_drvdata(psy);
	unsigned int uv;
	int capacity, ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		return 0;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		return 0;
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_SYSTEM;
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = vbat->charge_full_design_uah;
		return 0;
	case POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN:
		if (!vbat->energy_full_design_uwh)
			return -ENODATA;
		val->intval = vbat->energy_full_design_uwh;
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = vbat->voltage_min_design_uv;
		return 0;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "voltage estimate (no gauge)";
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
	case POWER_SUPPLY_PROP_STATUS:
		break;
	default:
		return -EINVAL;
	}

	ret = mt6358_vbat_get_voltage(vbat, &uv);
	if (ret)
		return ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = uv;
		return 0;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = mt6358_vbat_capacity(uv);
		return 0;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = mt6358_vbat_capacity_level(mt6358_vbat_capacity(uv));
		return 0;
	case POWER_SUPPLY_PROP_STATUS:
		capacity = mt6358_vbat_capacity(uv);
		val->intval = mt6358_vbat_status(capacity);
		return 0;
	default:
		return -EINVAL;
	}
}

static enum power_supply_property mt6358_vbat_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_SCOPE,
};

static const struct power_supply_desc mt6358_vbat_desc = {
	.name = "mt6358-vbat",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = mt6358_vbat_properties,
	.num_properties = ARRAY_SIZE(mt6358_vbat_properties),
	.get_property = mt6358_vbat_get_property,
};

/*
 * Design data from the board's own simple-battery node -- on the DC-1 the
 * stock DTB carries one (8000 mAh / 30.8 Wh / 3.4 V min).  It is located by
 * compatible rather than by phandle because this driver's own node is the
 * vendor gauge node, which has no monitored-battery property and cannot be
 * given one: the DTB comes from a vendor_boot partition the port does not
 * rewrite.
 */
static void mt6358_vbat_read_design(struct mt6358_vbat *vbat)
{
	struct device_node *np;
	u32 val;

	vbat->charge_full_design_uah = DEFAULT_CHARGE_FULL_DESIGN_UAH;
	vbat->voltage_min_design_uv = DEFAULT_VOLTAGE_MIN_DESIGN_UV;

	np = of_find_compatible_node(NULL, NULL, "simple-battery");
	if (!np) {
		dev_info(vbat->dev, "no simple-battery node, using defaults\n");
		return;
	}

	if (!of_property_read_u32(np, "charge-full-design-microamp-hours", &val))
		vbat->charge_full_design_uah = val;
	if (!of_property_read_u32(np, "energy-full-design-microwatt-hours", &val))
		vbat->energy_full_design_uwh = val;
	if (!of_property_read_u32(np, "voltage-min-design-microvolt", &val))
		vbat->voltage_min_design_uv = val;

	of_node_put(np);
}

/*
 * If the real gauge ever starts answering, it is the better source and this
 * estimate should get out of the way.  bq27xxx reports present = 0 while its
 * reads fail, so a present gauge means working communication.  A NULL lookup
 * means bq27xxx has not probed yet (or is not built in), which is not evidence
 * either way -- register in that case and let the operator remove one.
 */
static bool mt6358_vbat_real_gauge_alive(void)
{
	union power_supply_propval val;
	struct power_supply *gauge;
	int ret;

	gauge = power_supply_get_by_name("bq78z100-0");
	if (!gauge)
		return false;

	ret = power_supply_get_property(gauge, POWER_SUPPLY_PROP_PRESENT, &val);
	power_supply_put(gauge);

	return !ret && val.intval;
}

static int mt6358_vbat_probe(struct platform_device *pdev)
{
	struct power_supply_config psy_cfg = {};
	struct device *dev = &pdev->dev;
	struct power_supply *psy;
	struct mt6358_vbat *vbat;
	unsigned int uv;
	int ret;

	if (mt6358_vbat_real_gauge_alive())
		return dev_err_probe(dev, -ENODEV,
				     "real fuel gauge is answering, not registering an estimate\n");

	vbat = devm_kzalloc(dev, sizeof(*vbat), GFP_KERNEL);
	if (!vbat)
		return -ENOMEM;

	vbat->dev = dev;

	/*
	 * Same two cases as mt6359-auxadc: under SPMI the regmap belongs to
	 * the MT6397 MFD itself, under the SoC PMIC wrapper it belongs to the
	 * MFD's parent (pwrap). MT6366 on the DC-1 is the pwrap case.
	 */
	vbat->regmap = dev_get_regmap(dev->parent, NULL);
	if (!vbat->regmap && dev->parent->parent)
		vbat->regmap = dev_get_regmap(dev->parent->parent, NULL);
	if (!vbat->regmap)
		return dev_err_probe(dev, -ENODEV, "no PMIC regmap\n");

	ret = devm_mutex_init(dev, &vbat->lock);
	if (ret)
		return ret;

	mt6358_vbat_read_design(vbat);

	/* Prove the ADC path before publishing a battery that depends on it. */
	ret = mt6358_vbat_get_voltage(vbat, &uv);
	if (ret)
		return dev_err_probe(dev, ret, "BATADC read failed\n");

	psy_cfg.drv_data = vbat;
	psy_cfg.fwnode = dev_fwnode(dev);
	psy_cfg.no_wakeup_source = true;
	psy = devm_power_supply_register(dev, &mt6358_vbat_desc, &psy_cfg);
	if (IS_ERR(psy))
		return dev_err_probe(dev, PTR_ERR(psy),
				     "failed to register power supply\n");

	dev_info(dev, "voltage-estimated battery: %u.%03u V, %d%% (estimate)\n",
		 uv / 1000000, (uv / 1000) % 1000, mt6358_vbat_capacity(uv));
	return 0;
}

/*
 * Matched by name against the MFD cell, not by compatible. The stock DTB's
 * "mediatek,mt6358-gauge" node would be the natural binding, but it declares
 * nvmem-cells in the RTC's spare registers that nothing provides, so
 * fw_devlink holds any consumer of it in deferred probe permanently. See the
 * cell in drivers/mfd/mt6397-core.c.
 */
static struct platform_driver mt6358_vbat_driver = {
	.driver = {
		.name = "mt6358-vbat-battery",
	},
	.probe = mt6358_vbat_probe,
};
module_platform_driver(mt6358_vbat_driver);

MODULE_ALIAS("platform:mt6358-vbat-battery");

MODULE_DESCRIPTION("MT6358/MT6366 voltage-estimated battery");
MODULE_LICENSE("GPL");
