// SPDX-License-Identifier: GPL-2.0-only
/*
 * Coulomb-counting fuel gauge for the MT6358/MT6366 PMIC's FGADC block.
 *
 * The PMIC measures pack current through its sense element and integrates it
 * in hardware (the CAR accumulator). This driver reports that current, tracks
 * charge from the accumulator, and derives a state of charge from it. That is
 * a real measurement chain, not a voltage lookup: the percentage does not move
 * when the CPU load does.
 *
 * MediaTek's own stack for this block (mtk_battery.c + mt6358-gauge.c, ~6k
 * lines) implements a full SoC algorithm driven by ~80 DT tuning properties.
 * This is deliberately not that. It is the smallest thing that produces honest
 * numbers:
 *
 *   - CURRENT_NOW and CHARGE_COUNTER are measured, and carry no modelling.
 *   - CAPACITY is coulomb-counted from a boot-time seed. The seed is the one
 *     estimated ingredient, and it is unavoidable: a gauge that has never seen
 *     a full charge/discharge cycle has to start somewhere, which is why real
 *     gauges also seed from open-circuit voltage. Once seeded, the reading
 *     tracks charge actually moved.
 *   - CHARGE_FULL_DESIGN is the pack's design capacity from DT. It is not a
 *     learned full-charge capacity, so an aged pack reads optimistically.
 *
 * Why this exists at all: on the Daylight DC-1 the pack's BQ78Z100 -- which
 * does all of the above properly, in the pack, across reboots -- NAKs its own
 * address on i2c-7. The vendor kernel disables this PMIC gauge
 * (DISABLE_MTKBATTERY) precisely because that TI part is meant to do the job.
 * This driver is what is left when it cannot be reached.
 *
 * Register semantics and constants are from the vendor's mt6358-gauge.c:
 * current LSB 381.47 uA at RG_FGADC_CUR_CON0, charge LSB 0.108507 uAh in the
 * CAR pair, both behind a latch handshake on RG_FGADC_CON1.
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
#include <linux/workqueue.h>

/* AUXADC: BATADC, request bit 0 of RQST0, 128 samples, 3/1 divider, 1800 mV */
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

/* FGADC */
#define RG_FGADC_CON1			0xd0a
#define FG_LATCHDATA_ST			BIT(15)
#define FG_SW_CLEAR			BIT(3)
#define FG_SW_READ_PRE			BIT(0)
#define RG_FGADC_CAR_CON0		0xd14
#define RG_FGADC_CAR_CON1		0xd16
#define RG_FGADC_CUR_CON0		0xd8a
#define FG_LATCH_RETRIES		1000

/*
 * Current LSB is 381.47 uA, expressed in nA to stay in integer arithmetic.
 *
 * The charge LSB is the vendor's documented fundamental "CHARGE_LSB =
 * 190.735 uAs, unit 2^0 LSB", in nA*s here. Note this is NOT the vendor's
 * UNIT_FGCAR (108507): that constant scales their 20-bit NCAR register, a
 * different accumulator from the 32-bit CAR pair read here, and using it makes
 * the charge drain about twice as fast as the measured current can account
 * for. Checked on a DC-1 over 40 s at 124 mA: the CAR advanced 26 238 counts
 * against a true 1379 uAh, i.e. 0.0526 uAh per count, which is this constant
 * to within 0.8%.
 */
#define UNIT_FGCURRENT_NA		381470
#define UNIT_FGCAR_NAS			190735
#define NAS_PER_UAH			3600000

/*
 * Sign, established by measurement on a DC-1 rather than taken from the vendor
 * decode (which reads a positive register as charging): with no cable attached,
 * idle draws raw ~350 and eight spinning cores raw ~700, and the CAR
 * accumulator advances at a proportionally higher rate. So on this board a
 * positive register value is discharge, and the power-supply convention
 * (negative CURRENT_NOW = leaving the battery) is the negation of it.
 */
#define FG_RAW_IS_DISCHARGE		1

/* Integrate at least this often so charge does not jump in large steps. */
#define FG_POLL_INTERVAL_MS		10000
/* Don't drive an AUXADC conversion for every property read. */
#define VBAT_CACHE_MS			2000

#define DEFAULT_CHARGE_FULL_DESIGN_UAH	8000000
#define DEFAULT_VOLTAGE_MIN_DESIGN_UV	3400000

/*
 * Pack + path resistance, used only to undo IR drop when seeding from OCV.
 * Measured on a DC-1: 19 mV of sag for a 120 mA step (135 mA idle -> 255 mA
 * with eight cores busy) is ~158 mOhm. Rounded down slightly, since seeding
 * high is worse than seeding low.
 */
#define FG_PACK_RESISTANCE_MOHM		150

/*
 * Open-circuit voltage to state of charge, single-cell Li-ion, descending.
 * Used ONLY for the boot-time seed -- every subsequent reading comes from the
 * coulomb counter. Endpoints are the board's own numbers (4.35 V charger CV,
 * 3.4 V voltage-min-design); between them this is the generic Li-ion shape.
 */
struct fg_ocv_point {
	unsigned int uv;
	unsigned int permille;
};

static const struct fg_ocv_point fg_ocv_table[] = {
	{ 4350000, 1000 },
	{ 4250000,  930 },
	{ 4150000,  850 },
	{ 4060000,  750 },
	{ 3980000,  650 },
	{ 3920000,  550 },
	{ 3870000,  450 },
	{ 3830000,  350 },
	{ 3790000,  250 },
	{ 3750000,  170 },
	{ 3700000,  110 },
	{ 3650000,   70 },
	{ 3600000,   40 },
	{ 3500000,   10 },
	{ 3400000,    0 },
};

struct mt6358_fg {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	struct delayed_work poll;
	struct mutex lock;		/* everything below */

	unsigned long vbat_sampled_at;	/* jiffies; 0 = never */
	unsigned int voltage_uv;

	bool car_valid;
	s32 last_car;			/* raw accumulator, previous sample */
	s64 charge_uah;			/* tracked remaining charge */

	int current_ua;			/* signed, power-supply convention */
	unsigned int charge_full_design_uah;
	unsigned int energy_full_design_uwh;
	unsigned int voltage_min_design_uv;
};

/* ---------------------------------------------------------------- AUXADC */

static int mt6358_fg_sample_vbat(struct mt6358_fg *fg, unsigned int *uv)
{
	unsigned int val;
	int ret;

	ret = regmap_write(fg->regmap, MT6358_AUXADC_RQST0,
			   MT6358_BATADC_RQST_BIT);
	if (ret)
		return ret;

	fsleep(MT6358_BATADC_SAMPLES * MT6358_BATADC_AVG_TIME_US);

	ret = regmap_read_poll_timeout(fg->regmap, MT6358_AUXADC_ADC0, val,
				       val & AUXADC_RDY_BIT,
				       AUXADC_POLL_DELAY_US, AUXADC_TIMEOUT_US);
	if (ret)
		return ret;

	/* 64-bit: a 15-bit code times a 5 400 000 uV full scale is ~1.8e11. */
	val &= AUXADC_VALUE_MASK;
	*uv = div_u64((u64)val * AUXADC_FULL_SCALE_MV * 1000, AUXADC_RESOLUTION);
	return 0;
}

static int mt6358_fg_voltage(struct mt6358_fg *fg, unsigned int *uv)
{
	unsigned int sample;
	int ret;

	if (fg->vbat_sampled_at &&
	    time_before(jiffies,
			fg->vbat_sampled_at + msecs_to_jiffies(VBAT_CACHE_MS)))
		goto out;

	ret = mt6358_fg_sample_vbat(fg, &sample);
	if (ret) {
		if (!fg->vbat_sampled_at)
			return ret;
		goto out;	/* a stale volt beats no reading */
	}

	fg->voltage_uv = sample;
	fg->vbat_sampled_at = jiffies ? jiffies : 1;
out:
	*uv = fg->voltage_uv;
	return 0;
}

/* ----------------------------------------------------------------- FGADC */

/*
 * Current and charge are only coherent behind the latch handshake: assert
 * FG_SW_READ_PRE, wait for FG_LATCHDATA_ST, read, then release. The vendor
 * driver does this for every access and so does this one.
 */
static int mt6358_fg_read_latched(struct mt6358_fg *fg, s16 *raw_current,
				  s32 *car)
{
	unsigned int val, cur, car0, car1;
	int retries = 0, ret;

	ret = regmap_write(fg->regmap, RG_FGADC_CON1, FG_SW_READ_PRE);
	if (ret)
		return ret;

	do {
		if (++retries > FG_LATCH_RETRIES)
			return -ETIMEDOUT;
		ret = regmap_read(fg->regmap, RG_FGADC_CON1, &val);
		if (ret)
			return ret;
	} while (!(val & FG_LATCHDATA_ST));

	ret = regmap_read(fg->regmap, RG_FGADC_CUR_CON0, &cur);
	if (!ret)
		ret = regmap_read(fg->regmap, RG_FGADC_CAR_CON0, &car0);
	if (!ret)
		ret = regmap_read(fg->regmap, RG_FGADC_CAR_CON1, &car1);

	regmap_update_bits(fg->regmap, RG_FGADC_CON1,
			   FG_SW_CLEAR | FG_SW_READ_PRE, FG_SW_CLEAR);
	retries = 0;
	do {
		if (++retries > FG_LATCH_RETRIES)
			break;
		if (regmap_read(fg->regmap, RG_FGADC_CON1, &val))
			break;
	} while (val & FG_LATCHDATA_ST);
	regmap_update_bits(fg->regmap, RG_FGADC_CON1, FG_SW_CLEAR, 0);

	if (ret)
		return ret;

	*raw_current = (s16)(cur & 0xffff);
	*car = (s32)((car1 << 16) | (car0 & 0xffff));
	return 0;
}

static int mt6358_fg_ocv_to_permille(unsigned int uv)
{
	const struct fg_ocv_point *hi, *lo;
	unsigned int i;

	if (uv >= fg_ocv_table[0].uv)
		return 1000;

	for (i = 1; i < ARRAY_SIZE(fg_ocv_table); i++) {
		if (uv < fg_ocv_table[i].uv)
			continue;

		hi = &fg_ocv_table[i - 1];
		lo = &fg_ocv_table[i];
		return lo->permille + mult_frac(uv - lo->uv,
						hi->permille - lo->permille,
						hi->uv - lo->uv);
	}

	return 0;
}

/*
 * Seed the accumulator from open-circuit voltage, undoing the IR drop that the
 * measured current is causing right now. Only ever called once.
 */
static void mt6358_fg_seed(struct mt6358_fg *fg, unsigned int uv, int current_ua)
{
	int permille;
	s64 ocv_uv;

	ocv_uv = (s64)uv - div_s64((s64)current_ua * FG_PACK_RESISTANCE_MOHM,
				   1000);
	if (ocv_uv < 0)
		ocv_uv = 0;

	permille = mt6358_fg_ocv_to_permille(ocv_uv);
	fg->charge_uah = div_s64((s64)fg->charge_full_design_uah * permille,
				 1000);

	dev_info(fg->dev,
		 "seeded from OCV: %u uV terminal, %lld uV open-circuit at %d uA -> %d.%d%%\n",
		 uv, ocv_uv, current_ua, permille / 10, permille % 10);
}

/* Caller holds fg->lock. */
static int mt6358_fg_update(struct mt6358_fg *fg)
{
	unsigned int uv;
	s16 raw;
	s32 car;
	int ret;

	ret = mt6358_fg_read_latched(fg, &raw, &car);
	if (ret)
		return ret;

	/*
	 * FG_RAW_IS_DISCHARGE: the register counts up as the pack drains, and
	 * CURRENT_NOW is negative while discharging, so negate.
	 */
	fg->current_ua = -div_s64((s64)raw * UNIT_FGCURRENT_NA, 1000);

	ret = mt6358_fg_voltage(fg, &uv);
	if (ret)
		return ret;

	if (!fg->car_valid) {
		mt6358_fg_seed(fg, uv, fg->current_ua);
		fg->last_car = car;
		fg->car_valid = true;
		return 0;
	}

	/*
	 * Magnitude from the hardware accumulator, direction from the measured
	 * current. Taking the sign from the current rather than from the delta
	 * keeps this correct whether the accumulator is a signed net count or
	 * counts only in one direction -- which cannot be established here
	 * without a charger, and which this does not need to depend on.
	 */
	if (car != fg->last_car) {
		s64 delta = abs((s64)car - (s64)fg->last_car);

		delta = div_s64(delta * UNIT_FGCAR_NAS, NAS_PER_UAH);
		if (fg->current_ua < 0)
			fg->charge_uah -= delta;
		else
			fg->charge_uah += delta;

		fg->last_car = car;
	}

	fg->charge_uah = clamp_t(s64, fg->charge_uah, 0,
				 fg->charge_full_design_uah);
	return 0;
}

static int mt6358_fg_capacity(struct mt6358_fg *fg)
{
	if (!fg->charge_full_design_uah)
		return 0;

	return div64_s64(fg->charge_uah * 100, fg->charge_full_design_uah);
}

static void mt6358_fg_poll_work(struct work_struct *work)
{
	struct mt6358_fg *fg = container_of(work, struct mt6358_fg, poll.work);
	int before, after;

	scoped_guard(mutex, &fg->lock) {
		before = mt6358_fg_capacity(fg);
		mt6358_fg_update(fg);
		after = mt6358_fg_capacity(fg);
	}

	/* Push a uevent only when the number a user would see actually moves. */
	if (before != after)
		power_supply_changed(fg->psy);

	schedule_delayed_work(&fg->poll,
			      msecs_to_jiffies(FG_POLL_INTERVAL_MS));
}

static void mt6358_fg_cancel_poll(void *data)
{
	cancel_delayed_work_sync(data);
}

/* --------------------------------------------------------- power supply */

static int mt6358_fg_status(struct mt6358_fg *fg, int capacity)
{
	/*
	 * Direction is measured, so it does not depend on the charger driver
	 * being present. The charger is consulted only to tell "full" from
	 * "sitting at rest with no cable", which current alone cannot.
	 */
	if (fg->current_ua < 0)
		return POWER_SUPPLY_STATUS_DISCHARGING;

	if (fg->current_ua > 0)
		return capacity >= 100 ? POWER_SUPPLY_STATUS_FULL :
					 POWER_SUPPLY_STATUS_CHARGING;

	return POWER_SUPPLY_STATUS_NOT_CHARGING;
}

static int mt6358_fg_capacity_level(int capacity)
{
	if (capacity >= 100)
		return POWER_SUPPLY_CAPACITY_LEVEL_FULL;
	if (capacity > 15)
		return POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
	if (capacity > 5)
		return POWER_SUPPLY_CAPACITY_LEVEL_LOW;
	return POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
}

static int mt6358_fg_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	struct mt6358_fg *fg = power_supply_get_drvdata(psy);
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
		val->intval = fg->charge_full_design_uah;
		return 0;
	case POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN:
		if (!fg->energy_full_design_uwh)
			return -ENODATA;
		val->intval = fg->energy_full_design_uwh;
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = fg->voltage_min_design_uv;
		return 0;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "PMIC coulomb counter";
		return 0;
	default:
		break;
	}

	guard(mutex)(&fg->lock);

	ret = mt6358_fg_update(fg);
	if (ret)
		return ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = fg->voltage_uv;
		return 0;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = fg->current_ua;
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		val->intval = fg->charge_uah;
		return 0;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = mt6358_fg_capacity(fg);
		return 0;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = mt6358_fg_capacity_level(mt6358_fg_capacity(fg));
		return 0;
	case POWER_SUPPLY_PROP_STATUS:
		capacity = mt6358_fg_capacity(fg);
		val->intval = mt6358_fg_status(fg, capacity);
		return 0;
	default:
		return -EINVAL;
	}
}

static enum power_supply_property mt6358_fg_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_SCOPE,
};

static const struct power_supply_desc mt6358_fg_desc = {
	.name = "mt6358-fg",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = mt6358_fg_properties,
	.num_properties = ARRAY_SIZE(mt6358_fg_properties),
	.get_property = mt6358_fg_get_property,
};

/*
 * Design data from the board's own simple-battery node. Located by compatible
 * rather than by phandle because this driver's device is an MFD cell with no
 * firmware node, and the DTB it would point into comes from a vendor_boot
 * partition the port does not rewrite.
 */
static void mt6358_fg_read_design(struct mt6358_fg *fg)
{
	struct device_node *np;
	u32 val;

	fg->charge_full_design_uah = DEFAULT_CHARGE_FULL_DESIGN_UAH;
	fg->voltage_min_design_uv = DEFAULT_VOLTAGE_MIN_DESIGN_UV;

	np = of_find_compatible_node(NULL, NULL, "simple-battery");
	if (!np) {
		dev_info(fg->dev, "no simple-battery node, using defaults\n");
		return;
	}

	if (!of_property_read_u32(np, "charge-full-design-microamp-hours", &val))
		fg->charge_full_design_uah = val;
	if (!of_property_read_u32(np, "energy-full-design-microwatt-hours", &val))
		fg->energy_full_design_uwh = val;
	if (!of_property_read_u32(np, "voltage-min-design-microvolt", &val))
		fg->voltage_min_design_uv = val;

	of_node_put(np);
}

/*
 * A pack gauge that can be reached is strictly better than this one: it counts
 * coulombs in the pack, keeps its state across reboots, and knows the pack's
 * learned capacity. bq27xxx reports present = 0 while its reads fail, so a
 * present gauge means working communication and this driver should stay out of
 * the way. A NULL lookup means it has not probed yet, which is not evidence
 * either way -- register, and let the operator remove one.
 */
static bool mt6358_fg_pack_gauge_alive(void)
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

static int mt6358_fg_probe(struct platform_device *pdev)
{
	struct power_supply_config psy_cfg = {};
	struct device *dev = &pdev->dev;
	struct mt6358_fg *fg;
	int ret;

	if (mt6358_fg_pack_gauge_alive())
		return dev_err_probe(dev, -ENODEV,
				     "pack fuel gauge is answering, deferring to it\n");

	fg = devm_kzalloc(dev, sizeof(*fg), GFP_KERNEL);
	if (!fg)
		return -ENOMEM;

	fg->dev = dev;

	/*
	 * Same two cases as mt6359-auxadc: under SPMI the regmap belongs to the
	 * MT6397 MFD itself, under the SoC PMIC wrapper to the MFD's parent
	 * (pwrap). MT6366 on the DC-1 is the pwrap case.
	 */
	fg->regmap = dev_get_regmap(dev->parent, NULL);
	if (!fg->regmap && dev->parent->parent)
		fg->regmap = dev_get_regmap(dev->parent->parent, NULL);
	if (!fg->regmap)
		return dev_err_probe(dev, -ENODEV, "no PMIC regmap\n");

	ret = devm_mutex_init(dev, &fg->lock);
	if (ret)
		return ret;

	mt6358_fg_read_design(fg);

	/* Prove both measurement paths, and seed, before publishing anything. */
	mutex_lock(&fg->lock);
	ret = mt6358_fg_update(fg);
	mutex_unlock(&fg->lock);
	if (ret)
		return dev_err_probe(dev, ret, "FGADC/AUXADC read failed\n");

	psy_cfg.drv_data = fg;
	psy_cfg.fwnode = dev_fwnode(dev);
	psy_cfg.no_wakeup_source = true;
	fg->psy = devm_power_supply_register(dev, &mt6358_fg_desc, &psy_cfg);
	if (IS_ERR(fg->psy))
		return dev_err_probe(dev, PTR_ERR(fg->psy),
				     "failed to register power supply\n");

	INIT_DELAYED_WORK(&fg->poll, mt6358_fg_poll_work);
	ret = devm_add_action_or_reset(dev, mt6358_fg_cancel_poll, &fg->poll);
	if (ret)
		return ret;
	schedule_delayed_work(&fg->poll, msecs_to_jiffies(FG_POLL_INTERVAL_MS));

	dev_info(dev, "coulomb counter: %d uA, %lld uAh of %u uAh (%d%%)\n",
		 fg->current_ua, fg->charge_uah, fg->charge_full_design_uah,
		 mt6358_fg_capacity(fg));
	return 0;
}

/*
 * Matched by name against the MFD cell, not by compatible. The stock DTB's
 * "mediatek,mt6358-gauge" node would be the natural binding, but it declares
 * nvmem-cells in the RTC's spare registers that nothing provides, so fw_devlink
 * holds any consumer of it in deferred probe permanently. See the cell in
 * drivers/mfd/mt6397-core.c.
 */
static struct platform_driver mt6358_fg_driver = {
	.driver = {
		.name = "mt6358-fg",
	},
	.probe = mt6358_fg_probe,
};
module_platform_driver(mt6358_fg_driver);

MODULE_ALIAS("platform:mt6358-fg");
MODULE_DESCRIPTION("MT6358/MT6366 PMIC coulomb-counting fuel gauge");
MODULE_LICENSE("GPL");
