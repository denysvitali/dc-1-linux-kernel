// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Wacom Penabled Driver for I2C
 *
 * Copyright (c) 2011 - 2013 Tatsunosuke Tobita, Wacom.
 * <tobita.tatsunosuke@wacom.co.jp>
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/input/touchscreen.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include <linux/unaligned.h>

/* Time to wait between feature query attempts */
#define WACOM_QUERY_RETRY_MS	100
#define WACOM_QUERY_RETRIES	10

/* Pressure limit assumed when neither firmware nor DT provides one */
#define WACOM_DEFAULT_PRESSURE_MAX	1023

/* Bitmasks (for data[3]) */
#define WACOM_TIP_SWITCH	BIT(0)
#define WACOM_BARREL_SWITCH	BIT(1)
#define WACOM_ERASER		BIT(2)
#define WACOM_INVERT		BIT(3)
#define WACOM_BARREL_SWITCH_2	BIT(4)
#define WACOM_IN_PROXIMITY	BIT(5)

/* Registers */
#define WACOM_COMMAND_LSB	0x04
#define WACOM_COMMAND_MSB	0x00

#define WACOM_DATA_LSB		0x05
#define WACOM_DATA_MSB		0x00

/* Report types */
#define REPORT_FEATURE		0x30

/* Requests / operations */
#define OPCODE_GET_REPORT	0x02

#define WACOM_QUERY_REPORT	3
#define WACOM_QUERY_SIZE	19

struct wacom_features {
	int x_max;
	int y_max;
	int pressure_max;
	char fw_version;
};

struct wacom_i2c {
	struct i2c_client *client;
	struct input_dev *input;
	u8 data[WACOM_QUERY_SIZE];
	bool prox;
	int tool;
	struct touchscreen_properties prop;
};

static int wacom_query_device(struct i2c_client *client,
			      struct wacom_features *features)
{
	u8 get_query_data_cmd[] = {
		WACOM_COMMAND_LSB,
		WACOM_COMMAND_MSB,
		REPORT_FEATURE | WACOM_QUERY_REPORT,
		OPCODE_GET_REPORT,
		WACOM_DATA_LSB,
		WACOM_DATA_MSB,
	};
	u8 data[WACOM_QUERY_SIZE];
	int ret;

	struct i2c_msg msgs[] = {
		/* Request reading of feature ReportID: 3 (Pen Query Data) */
		{
			.addr = client->addr,
			.flags = 0,
			.len = sizeof(get_query_data_cmd),
			.buf = get_query_data_cmd,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = sizeof(data),
			.buf = data,
		},
	};

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0)
		return ret;
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	features->x_max = get_unaligned_le16(&data[3]);
	features->y_max = get_unaligned_le16(&data[5]);
	features->pressure_max = get_unaligned_le16(&data[11]);
	features->fw_version = get_unaligned_le16(&data[13]);

	dev_dbg(&client->dev,
		"x_max:%d, y_max:%d, pressure:%d, fw:%d\n",
		features->x_max, features->y_max,
		features->pressure_max, features->fw_version);

	return 0;
}

static irqreturn_t wacom_i2c_irq(int irq, void *dev_id)
{
	struct wacom_i2c *wac_i2c = dev_id;
	struct input_dev *input = wac_i2c->input;
	u8 *data = wac_i2c->data;
	unsigned int x, y, pressure;
	unsigned char tsw, f1, f2, ers;
	int error;

	error = i2c_master_recv(wac_i2c->client,
				wac_i2c->data, sizeof(wac_i2c->data));
	if (error < 0)
		goto out;

	tsw = data[3] & WACOM_TIP_SWITCH;
	ers = data[3] & WACOM_ERASER;
	f1 = data[3] & WACOM_BARREL_SWITCH;
	f2 = data[3] & WACOM_BARREL_SWITCH_2;
	x = le16_to_cpup((__le16 *)&data[4]);
	y = le16_to_cpup((__le16 *)&data[6]);
	pressure = le16_to_cpup((__le16 *)&data[8]);

	if (!wac_i2c->prox)
		wac_i2c->tool = (data[3] & (WACOM_ERASER | WACOM_INVERT)) ?
			BTN_TOOL_RUBBER : BTN_TOOL_PEN;

	wac_i2c->prox = data[3] & WACOM_IN_PROXIMITY;

	input_report_key(input, BTN_TOUCH, tsw || ers);
	input_report_key(input, wac_i2c->tool, wac_i2c->prox);
	input_report_key(input, BTN_STYLUS, f1);
	input_report_key(input, BTN_STYLUS2, f2);
	touchscreen_report_pos(input, &wac_i2c->prop, x, y, false);
	input_report_abs(input, ABS_PRESSURE, pressure);
	input_sync(input);

out:
	return IRQ_HANDLED;
}

static int wacom_i2c_open(struct input_dev *dev)
{
	struct wacom_i2c *wac_i2c = input_get_drvdata(dev);
	struct i2c_client *client = wac_i2c->client;

	enable_irq(client->irq);

	return 0;
}

static void wacom_i2c_close(struct input_dev *dev)
{
	struct wacom_i2c *wac_i2c = input_get_drvdata(dev);
	struct i2c_client *client = wac_i2c->client;

	disable_irq(client->irq);
}

static int wacom_i2c_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct wacom_i2c *wac_i2c;
	struct input_dev *input;
	struct gpio_desc *reset;
	struct wacom_features features = { 0 };
	unsigned int retries;
	int error;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(dev, "i2c_check_functionality error\n");
		return -EIO;
	}

	/* Hold the controller in reset while its supply settles. */
	reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(reset))
		return dev_err_probe(dev, PTR_ERR(reset),
				     "Failed to request reset GPIO\n");

	error = devm_regulator_get_enable_optional(dev, "vdd");
	if (error && error != -ENODEV)
		return dev_err_probe(dev, error,
				     "Failed to enable vdd supply\n");

	usleep_range(10000, 20000);
	gpiod_set_value_cansleep(reset, 0);

	/*
	 * Some firmwares take a while after reset release before they answer
	 * the feature query, and some do not implement it at all. Poll for a
	 * while, then fall back to the limits described in the device tree.
	 */
	for (retries = 0; retries < WACOM_QUERY_RETRIES; retries++) {
		error = wacom_query_device(client, &features);
		if (!error && features.x_max && features.y_max)
			break;
		msleep(WACOM_QUERY_RETRY_MS);
	}

	if (error || !features.x_max || !features.y_max)
		dev_warn(dev,
			 "Feature query failed (%d), using DT limits\n",
			 error);

	wac_i2c = devm_kzalloc(dev, sizeof(*wac_i2c), GFP_KERNEL);
	if (!wac_i2c)
		return -ENOMEM;

	wac_i2c->client = client;

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	wac_i2c->input = input;

	input->name = "Wacom I2C Digitizer";
	input->id.bustype = BUS_I2C;
	input->id.vendor = 0x56a;
	input->id.version = features.fw_version;
	input->open = wacom_i2c_open;
	input->close = wacom_i2c_close;

	input->evbit[0] |= BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);

	__set_bit(BTN_TOOL_PEN, input->keybit);
	__set_bit(BTN_TOOL_RUBBER, input->keybit);
	__set_bit(BTN_STYLUS, input->keybit);
	__set_bit(BTN_STYLUS2, input->keybit);
	__set_bit(BTN_TOUCH, input->keybit);

	if (!features.pressure_max)
		features.pressure_max = WACOM_DEFAULT_PRESSURE_MAX;

	input_set_abs_params(input, ABS_X, 0, features.x_max, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, features.y_max, 0, 0);
	input_set_abs_params(input, ABS_PRESSURE,
			     0, features.pressure_max, 0, 0);

	touchscreen_parse_properties(input, false, &wac_i2c->prop);

	/*
	 * Where the feature report carried real axis limits those are the
	 * part's own geometry and outrank the board's fallback numbers.
	 */
	if (features.x_max)
		input_abs_set_max(input, ABS_X, features.x_max);
	if (features.y_max)
		input_abs_set_max(input, ABS_Y, features.y_max);

	if (!input_abs_get_max(input, ABS_X) ||
	    !input_abs_get_max(input, ABS_Y)) {
		dev_err(dev, "No X/Y limits from firmware or DT\n");
		return -EINVAL;
	}

	input_set_drvdata(input, wac_i2c);

	error = devm_request_threaded_irq(dev, client->irq, NULL, wacom_i2c_irq,
					  IRQF_ONESHOT, "wacom_i2c", wac_i2c);
	if (error) {
		dev_err(dev, "Failed to request IRQ: %d\n", error);
		return error;
	}

	/* Disable the IRQ, we'll enable it in wacom_i2c_open() */
	disable_irq(client->irq);

	error = input_register_device(wac_i2c->input);
	if (error) {
		dev_err(dev, "Failed to register input device: %d\n", error);
		return error;
	}

	return 0;
}

static int wacom_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	disable_irq(client->irq);

	return 0;
}

static int wacom_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	enable_irq(client->irq);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(wacom_i2c_pm, wacom_i2c_suspend, wacom_i2c_resume);

static const struct i2c_device_id wacom_i2c_id[] = {
	{ .name = "WAC_I2C_EMR" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, wacom_i2c_id);

static const struct of_device_id wacom_i2c_of_match[] = {
	{ .compatible = "wacom,wacom-i2c" },
	{ }
};
MODULE_DEVICE_TABLE(of, wacom_i2c_of_match);

static struct i2c_driver wacom_i2c_driver = {
	.driver	= {
		.name	= "wacom_i2c",
		.pm	= pm_sleep_ptr(&wacom_i2c_pm),
		.of_match_table = wacom_i2c_of_match,
	},

	.probe		= wacom_i2c_probe,
	.id_table	= wacom_i2c_id,
};
module_i2c_driver(wacom_i2c_driver);

MODULE_AUTHOR("Tatsunosuke Tobita <tobita.tatsunosuke@wacom.co.jp>");
MODULE_DESCRIPTION("WACOM EMR I2C Driver");
MODULE_LICENSE("GPL");
