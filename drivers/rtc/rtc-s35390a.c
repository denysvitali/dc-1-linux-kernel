// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Seiko Instruments S-35390A RTC Driver
 *
 * Copyright (c) 2007 Byron Bradley
 */

#include <linux/module.h>
#include <linux/rtc.h>
#include <linux/i2c.h>
#include <linux/bitrev.h>
#include <linux/bcd.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>

#include "rtc-s35390a-internal.h"

#define S35390A_CMD_STATUS1	0
#define S35390A_CMD_STATUS2	1
#define S35390A_CMD_TIME1	2
#define S35390A_CMD_TIME2	3
#define S35390A_CMD_INT2_REG1	5
#define S35390A_CMD_FREE_REG    7

#define S35390A_BYTE_YEAR	0
#define S35390A_BYTE_MONTH	1
#define S35390A_BYTE_DAY	2
#define S35390A_BYTE_WDAY	3
#define S35390A_BYTE_HOURS	4
#define S35390A_BYTE_MINS	5
#define S35390A_BYTE_SECS	6

#define S35390A_ALRM_BYTE_WDAY	0
#define S35390A_ALRM_BYTE_HOURS	1
#define S35390A_ALRM_BYTE_MINS	2

/* flags for STATUS1 */
#define S35390A_FLAG_POC	BIT(0)
#define S35390A_FLAG_BLD	BIT(1)
#define S35390A_FLAG_24H	BIT(6)
#define S35390A_FLAG_RESET	BIT(7)

/* flag for STATUS2 */
#define S35390A_FLAG_TEST	BIT(0)

/* INT2 pin output mode */
#define S35390A_INT2_MODE_PMIN_EDG	BIT(2) /* INT2ME */
#define S35390A_INT2_MODE_FREQ		BIT(3) /* INT2FE */
#define S35390A_INT2_MODE_PMIN		(BIT(3) | BIT(2)) /* INT2FE | INT2ME */

static const struct i2c_device_id s35390a_id[] = {
	{ .name = "s35390a" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, s35390a_id);

static const __maybe_unused struct of_device_id s35390a_of_match[] = {
	{ .compatible = "sii,s35390a" },
	{ }
};
MODULE_DEVICE_TABLE(of, s35390a_of_match);

struct s35390a {
	struct i2c_client *client[8];
	struct rtc_device *rtc;
	/* Serialize STATUS/alarm transactions with threaded IRQ acknowledge. */
	struct mutex lock;
	int twentyfourhour;
};

static int s35390a_set_reg(struct s35390a *s35390a, int reg, u8  *buf, int len)
{
	struct i2c_client *client = s35390a->client[reg];
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.len = len,
			.buf = buf
		},
	};

	if ((i2c_transfer(client->adapter, msg, 1)) != 1)
		return -EIO;

	return 0;
}

static int s35390a_get_reg(struct s35390a *s35390a, int reg, u8 *buf, int len)
{
	struct i2c_client *client = s35390a->client[reg];
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = buf
		},
	};

	if ((i2c_transfer(client->adapter, msg, 1)) != 1)
		return -EIO;

	return 0;
}

static int s35390a_init(struct s35390a *s35390a)
{
	u8 buf;
	int ret;
	unsigned initcount = 0;

	/*
	 * At least one of POC and BLD are set, so reinitialise chip. Keeping
	 * this information in the hardware to know later that the time isn't
	 * valid is unfortunately not possible because POC and BLD are cleared
	 * on read. So the reset is best done now.
	 *
	 * The 24H bit is kept over reset, so set it already here.
	 */
initialize:
	buf = S35390A_FLAG_RESET | S35390A_FLAG_24H;
	ret = s35390a_set_reg(s35390a, S35390A_CMD_STATUS1, &buf, 1);

	if (ret < 0)
		return ret;

	ret = s35390a_get_reg(s35390a, S35390A_CMD_STATUS1, &buf, 1);
	if (ret < 0)
		return ret;

	if (buf & (S35390A_FLAG_POC | S35390A_FLAG_BLD)) {
		/* Try up to five times to reset the chip */
		if (initcount < 5) {
			++initcount;
			goto initialize;
		} else
			return -EIO;
	}

	/*
	 * The INT-pin output modes are battery-backed: a configuration left
	 * by whatever ran before us -- an armed alarm, a per-minute edge,
	 * a user-frequency output -- survives into this boot with the INT
	 * pin asserting while we know nothing about why.  A level-triggered
	 * alarm IRQ then storms at handler rate from probe onward (measured
	 * at ~2 kHz / 50% duty on hardware).  The RESET above only runs on
	 * POC/BLD, so it does not cover this; clear the modes explicitly.
	 *
	 * Linux owns both INT pins from here on -- userspace re-arms
	 * anything it wants through the RTC core.  Write all-zero rather
	 * than only the mode bits this driver models: parts in the field
	 * differ in which STATUS2 bit gates which output, and zero disables
	 * every one of them (plus TEST, which the datasheet says must be 0).
	 */
	buf = S35390A_INT2_MODE_NOINTR;
	ret = s35390a_set_reg(s35390a, S35390A_CMD_STATUS2, &buf, 1);
	if (ret < 0)
		return ret;

	return 1;
}

/*
 * Returns <0 on error, 0 if rtc is setup fine and 1 if the chip was reset.
 * To keep the information if an irq is pending, pass the value read from
 * STATUS1 to the caller.
 */
static int s35390a_read_status(struct s35390a *s35390a, u8 *status1)
{
	int ret;

	ret = s35390a_get_reg(s35390a, S35390A_CMD_STATUS1, status1, 1);
	if (ret < 0)
		return ret;

	if (*status1 & S35390A_FLAG_POC) {
		/*
		 * Do not communicate for 0.5 seconds since the power-on
		 * detection circuit is in operation.
		 */
		msleep(500);
		return 1;
	} else if (*status1 & S35390A_FLAG_BLD)
		return 1;
	/*
	 * If both POC and BLD are unset everything is fine.
	 */
	return 0;
}

static int s35390a_disable_test_mode(struct s35390a *s35390a)
{
	u8 buf[1];

	if (s35390a_get_reg(s35390a, S35390A_CMD_STATUS2, buf, sizeof(buf)) < 0)
		return -EIO;

	if (!(buf[0] & S35390A_FLAG_TEST))
		return 0;

	buf[0] &= ~S35390A_FLAG_TEST;
	return s35390a_set_reg(s35390a, S35390A_CMD_STATUS2, buf, sizeof(buf));
}

static char s35390a_hr2reg(struct s35390a *s35390a, int hour)
{
	if (s35390a->twentyfourhour)
		return bin2bcd(hour);

	if (hour < 12)
		return bin2bcd(hour);

	return 0x40 | bin2bcd(hour - 12);
}

static int s35390a_reg2hr(struct s35390a *s35390a, char reg)
{
	unsigned hour;

	if (s35390a->twentyfourhour)
		return bcd2bin(reg & 0x3f);

	hour = bcd2bin(reg & 0x3f);
	if (reg & 0x40)
		hour += 12;

	return hour;
}

static int s35390a_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct s35390a	*s35390a = i2c_get_clientdata(client);
	int i, err;
	u8 buf[7], status;

	dev_dbg(&client->dev, "%s: tm is secs=%d, mins=%d, hours=%d mday=%d, "
		"mon=%d, year=%d, wday=%d\n", __func__, tm->tm_sec,
		tm->tm_min, tm->tm_hour, tm->tm_mday, tm->tm_mon, tm->tm_year,
		tm->tm_wday);

	mutex_lock(&s35390a->lock);

	err = s35390a_read_status(s35390a, &status);
	if (err == 1)
		err = s35390a_init(s35390a);
	if (err < 0)
		goto unlock;

	buf[S35390A_BYTE_YEAR] = bin2bcd(tm->tm_year - 100);
	buf[S35390A_BYTE_MONTH] = bin2bcd(tm->tm_mon + 1);
	buf[S35390A_BYTE_DAY] = bin2bcd(tm->tm_mday);
	buf[S35390A_BYTE_WDAY] = bin2bcd(tm->tm_wday);
	buf[S35390A_BYTE_HOURS] = s35390a_hr2reg(s35390a, tm->tm_hour);
	buf[S35390A_BYTE_MINS] = bin2bcd(tm->tm_min);
	buf[S35390A_BYTE_SECS] = bin2bcd(tm->tm_sec);

	/* This chip expects the bits of each byte to be in reverse order */
	for (i = 0; i < 7; ++i)
		buf[i] = bitrev8(buf[i]);

	err = s35390a_set_reg(s35390a, S35390A_CMD_TIME1, buf, sizeof(buf));

unlock:
	mutex_unlock(&s35390a->lock);

	return err;
}

static int s35390a_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct s35390a *s35390a = i2c_get_clientdata(client);
	u8 buf[7], status;
	int i, err;

	mutex_lock(&s35390a->lock);

	err = s35390a_read_status(s35390a, &status);
	if (err == 1) {
		err = -EINVAL;
		goto unlock;
	}
	if (err < 0)
		goto unlock;

	err = s35390a_get_reg(s35390a, S35390A_CMD_TIME1, buf, sizeof(buf));
	if (err < 0)
		goto unlock;

	/* This chip returns the bits of each byte in reverse order */
	for (i = 0; i < 7; ++i)
		buf[i] = bitrev8(buf[i]);

	tm->tm_sec = bcd2bin(buf[S35390A_BYTE_SECS]);
	tm->tm_min = bcd2bin(buf[S35390A_BYTE_MINS]);
	tm->tm_hour = s35390a_reg2hr(s35390a, buf[S35390A_BYTE_HOURS]);
	tm->tm_wday = bcd2bin(buf[S35390A_BYTE_WDAY]);
	tm->tm_mday = bcd2bin(buf[S35390A_BYTE_DAY]);
	tm->tm_mon = bcd2bin(buf[S35390A_BYTE_MONTH]) - 1;
	tm->tm_year = bcd2bin(buf[S35390A_BYTE_YEAR]) + 100;

	dev_dbg(&client->dev, "%s: tm is secs=%d, mins=%d, hours=%d, mday=%d, "
		"mon=%d, year=%d, wday=%d\n", __func__, tm->tm_sec,
		tm->tm_min, tm->tm_hour, tm->tm_mday, tm->tm_mon, tm->tm_year,
		tm->tm_wday);

	err = 0;

unlock:
	mutex_unlock(&s35390a->lock);

	return err;
}

static int s35390a_alarm_irq_enable_locked(struct s35390a *s35390a,
					   bool enabled)
{
	u8 status1, status2;
	int err;

	err = s35390a_get_reg(s35390a, S35390A_CMD_STATUS2, &status2,
			      sizeof(status2));
	if (err < 0)
		return err;

	/* Disable INT2 before clearing its latched STATUS1 condition. */
	status2 = s35390a_status2_alarm_mode(status2, false);
	err = s35390a_set_reg(s35390a, S35390A_CMD_STATUS2, &status2,
			      sizeof(status2));
	if (err < 0)
		return err;

	err = s35390a_get_reg(s35390a, S35390A_CMD_STATUS1, &status1,
			      sizeof(status1));
	if (err < 0 || !enabled)
		return err;

	status2 = s35390a_status2_alarm_mode(status2, true);
	return s35390a_set_reg(s35390a, S35390A_CMD_STATUS2, &status2,
				 sizeof(status2));
}

static int s35390a_rtc_alarm_irq_enable(struct device *dev,
					unsigned int enabled)
{
	struct s35390a *s35390a = dev_get_drvdata(dev);
	int err;

	mutex_lock(&s35390a->lock);
	err = s35390a_alarm_irq_enable_locked(s35390a, enabled);
	mutex_unlock(&s35390a->lock);

	return err;
}

static int s35390a_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct s35390a *s35390a = i2c_get_clientdata(client);
	u8 buf[3];
	int err, i;

	dev_dbg(&client->dev, "%s: alm is secs=%d, mins=%d, hours=%d mday=%d, "\
		"mon=%d, year=%d, wday=%d\n", __func__, alm->time.tm_sec,
		alm->time.tm_min, alm->time.tm_hour, alm->time.tm_mday,
		alm->time.tm_mon, alm->time.tm_year, alm->time.tm_wday);

	mutex_lock(&s35390a->lock);

	err = s35390a_alarm_irq_enable_locked(s35390a, false);
	if (err < 0)
		goto unlock;
	if (alm->enabled) {
		err = s35390a_alarm_irq_enable_locked(s35390a, true);
		if (err < 0)
			goto unlock;
	}

	if (alm->time.tm_wday != -1)
		buf[S35390A_ALRM_BYTE_WDAY] = bin2bcd(alm->time.tm_wday) | 0x80;
	else
		buf[S35390A_ALRM_BYTE_WDAY] = 0;

	buf[S35390A_ALRM_BYTE_HOURS] = s35390a_hr2reg(s35390a,
			alm->time.tm_hour) | 0x80;
	buf[S35390A_ALRM_BYTE_MINS] = bin2bcd(alm->time.tm_min) | 0x80;

	if (alm->time.tm_hour >= 12)
		buf[S35390A_ALRM_BYTE_HOURS] |= 0x40;

	for (i = 0; i < 3; ++i)
		buf[i] = bitrev8(buf[i]);

	err = s35390a_set_reg(s35390a, S35390A_CMD_INT2_REG1, buf,
				 sizeof(buf));

unlock:
	mutex_unlock(&s35390a->lock);

	return err;
}

static int s35390a_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct s35390a *s35390a = i2c_get_clientdata(client);
	u8 buf[3], sts;
	int i, err;

	mutex_lock(&s35390a->lock);

	err = s35390a_get_reg(s35390a, S35390A_CMD_STATUS2, &sts, sizeof(sts));
	if (err < 0)
		goto unlock;

	if ((sts & S35390A_INT2_MODE_MASK) != S35390A_INT2_MODE_ALARM) {
		/*
		 * When the alarm isn't enabled, the register to configure
		 * the alarm time isn't accessible.
		 */
		alm->enabled = 0;
		err = 0;
		goto unlock;
	} else {
		alm->enabled = 1;
	}

	err = s35390a_get_reg(s35390a, S35390A_CMD_INT2_REG1, buf, sizeof(buf));
	if (err < 0)
		goto unlock;

	/* This chip returns the bits of each byte in reverse order */
	for (i = 0; i < 3; ++i)
		buf[i] = bitrev8(buf[i]);

	/*
	 * B0 of the three matching registers is an enable flag. Iff it is set
	 * the configured value is used for matching.
	 */
	if (buf[S35390A_ALRM_BYTE_WDAY] & 0x80)
		alm->time.tm_wday =
			bcd2bin(buf[S35390A_ALRM_BYTE_WDAY] & ~0x80);

	if (buf[S35390A_ALRM_BYTE_HOURS] & 0x80)
		alm->time.tm_hour =
			s35390a_reg2hr(s35390a,
				       buf[S35390A_ALRM_BYTE_HOURS] & ~0x80);

	if (buf[S35390A_ALRM_BYTE_MINS] & 0x80)
		alm->time.tm_min = bcd2bin(buf[S35390A_ALRM_BYTE_MINS] & ~0x80);

	/* alarm triggers always at s=0 */
	alm->time.tm_sec = 0;

	dev_dbg(&client->dev, "%s: alm is mins=%d, hours=%d, wday=%d\n",
			__func__, alm->time.tm_min, alm->time.tm_hour,
			alm->time.tm_wday);

	err = 0;

unlock:
	mutex_unlock(&s35390a->lock);

	return err;
}

static int s35390a_rtc_ioctl(struct device *dev, unsigned int cmd,
			     unsigned long arg)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct s35390a *s35390a = i2c_get_clientdata(client);
	u8 sts;
	int err;

	mutex_lock(&s35390a->lock);

	switch (cmd) {
	case RTC_VL_READ:
		/* s35390a_reset set lowvoltage flag and init RTC if needed */
		err = s35390a_read_status(s35390a, &sts);
		if (err < 0)
			goto unlock;
		if (copy_to_user((void __user *)arg, &err, sizeof(int)))
			err = -EFAULT;
		break;
	case RTC_VL_CLR:
		/* update flag and clear register */
		err = s35390a_init(s35390a);
		if (err < 0)
			goto unlock;
		break;
	default:
		err = -ENOIOCTLCMD;
	}

unlock:
	mutex_unlock(&s35390a->lock);

	return err < 0 ? err : 0;
}

static irqreturn_t s35390a_irq(int irq, void *dev_id)
{
	struct s35390a *s35390a = dev_id;
	unsigned long events;
	u8 status1;
	int err;

	/*
	 * The INT2 flag is read-only in hardware and auto-clears on read, but
	 * clearing it does NOT release the INT2 pin: while INT2AE stays set
	 * and the alarm condition stands (an armed alarm whose time already
	 * passed re-satisfies immediately), the pin keeps asserting.  Reading
	 * STATUS1 alone therefore turns every alarm into an endless
	 * level-triggered storm.  Per the datasheet the acknowledge is "read
	 * the flag, then set INT2AE to 0"; userspace re-arms through
	 * set_alarm().  Do both inside one critical section while IRQF_ONESHOT
	 * still masks the line, so the pin is released before it can retrigger.
	 */
	mutex_lock(&s35390a->lock);
	err = s35390a_get_reg(s35390a, S35390A_CMD_STATUS1, &status1,
			      sizeof(status1));
	if (err == 0)
		err = s35390a_alarm_irq_enable_locked(s35390a, false);
	mutex_unlock(&s35390a->lock);
	if (err < 0) {
		dev_err_ratelimited(&s35390a->client[0]->dev,
				    "failed to clear alarm IRQ: %d\n", err);
		return IRQ_NONE;
	}

	events = s35390a_status1_alarm_events(status1);
	if (!events) {
		/*
		 * Line asserted with no latched event: nothing to report, but
		 * the disarm above is what releases it.  Report HANDLED so a
		 * stuck or noisy line does not feed the spurious-IRQ counter.
		 */
		dev_warn_ratelimited(&s35390a->client[0]->dev,
				     "alarm IRQ with INT2 flag clear; disarmed INT2\n");
		return IRQ_HANDLED;
	}

	rtc_update_irq(s35390a->rtc, 1, events);

	return IRQ_HANDLED;
}

static const struct rtc_class_ops s35390a_rtc_ops = {
	.read_time	= s35390a_rtc_read_time,
	.set_time	= s35390a_rtc_set_time,
	.set_alarm	= s35390a_rtc_set_alarm,
	.read_alarm	= s35390a_rtc_read_alarm,
	.alarm_irq_enable = s35390a_rtc_alarm_irq_enable,
	.ioctl          = s35390a_rtc_ioctl,
};

static int s35390a_nvmem_read(void *priv, unsigned int offset, void *val,
			      size_t bytes)
{
	struct s35390a *s35390a = priv;

	/* The offset is ignored because the NVMEM region is only 1 byte */
	return s35390a_get_reg(s35390a, S35390A_CMD_FREE_REG, val, bytes);
}

static int s35390a_nvmem_write(void *priv, unsigned int offset, void *val,
			       size_t bytes)
{
	struct s35390a *s35390a = priv;

	return s35390a_set_reg(s35390a, S35390A_CMD_FREE_REG, val, bytes);
}

static int s35390a_probe(struct i2c_client *client)
{
	int err, err_read;
	unsigned int i;
	struct s35390a *s35390a;
	struct rtc_device *rtc;
	u8 buf, status1;
	struct device *dev = &client->dev;
	struct nvmem_config nvmem_cfg = {
		.name = "s35390a_nvram",
		.type = NVMEM_TYPE_BATTERY_BACKED,
		.word_size = 1,
		.stride = 1,
		.size = 1,
		.reg_read = s35390a_nvmem_read,
		.reg_write = s35390a_nvmem_write,
	};

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	s35390a = devm_kzalloc(dev, sizeof(struct s35390a), GFP_KERNEL);
	if (!s35390a)
		return -ENOMEM;

	s35390a->client[0] = client;
	mutex_init(&s35390a->lock);
	i2c_set_clientdata(client, s35390a);

	/* This chip uses multiple addresses, use dummy devices for them */
	for (i = 1; i < 8; ++i) {
		s35390a->client[i] = devm_i2c_new_dummy_device(dev,
							       client->adapter,
							       client->addr + i);
		if (IS_ERR(s35390a->client[i])) {
			dev_err(dev, "Address %02x unavailable\n",
				client->addr + i);
			return PTR_ERR(s35390a->client[i]);
		}
	}

	rtc = devm_rtc_allocate_device(dev);
	if (IS_ERR(rtc))
		return PTR_ERR(rtc);
	s35390a->rtc = rtc;

	err_read = s35390a_read_status(s35390a, &status1);
	if (err_read < 0) {
		dev_err(dev, "error resetting chip\n");
		return err_read;
	}

	if (status1 & S35390A_FLAG_24H)
		s35390a->twentyfourhour = 1;
	else
		s35390a->twentyfourhour = 0;

	if (status1 & S35390A_FLAG_INT2) {
		/* disable alarm (and maybe test mode) */
		buf = 0;
		err = s35390a_set_reg(s35390a, S35390A_CMD_STATUS2, &buf, 1);
		if (err < 0) {
			dev_err(dev, "error disabling alarm");
			return err;
		}
	} else {
		err = s35390a_disable_test_mode(s35390a);
		if (err < 0) {
			dev_err(dev, "error disabling test mode\n");
			return err;
		}
	}

	rtc->ops = &s35390a_rtc_ops;
	rtc->range_min = RTC_TIMESTAMP_BEGIN_2000;
	rtc->range_max = RTC_TIMESTAMP_END_2099;

	set_bit(RTC_FEATURE_ALARM_RES_MINUTE, rtc->features);
	clear_bit(RTC_FEATURE_UPDATE_INTERRUPT, rtc->features);
	if (client->irq > 0) {
		err = devm_request_threaded_irq(dev, client->irq, NULL,
						s35390a_irq, IRQF_ONESHOT,
						dev_name(dev), s35390a);
		if (err)
			return dev_err_probe(dev, err,
					     "unable to request alarm IRQ\n");
	} else {
		clear_bit(RTC_FEATURE_ALARM, rtc->features);
	}

	if (status1 & S35390A_FLAG_INT2)
		rtc_update_irq(rtc, 1, RTC_IRQF | RTC_AF);

	nvmem_cfg.priv = s35390a;
	err = devm_rtc_nvmem_register(rtc, &nvmem_cfg);
	if (err)
		return err;

	return devm_rtc_register_device(rtc);
}

static struct i2c_driver s35390a_driver = {
	.driver		= {
		.name	= "rtc-s35390a",
		.of_match_table = of_match_ptr(s35390a_of_match),
	},
	.probe		= s35390a_probe,
	.id_table	= s35390a_id,
};

module_i2c_driver(s35390a_driver);

MODULE_AUTHOR("Byron Bradley <byron.bbradley@gmail.com>");
MODULE_DESCRIPTION("S35390A RTC driver");
MODULE_LICENSE("GPL");
