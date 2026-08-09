// SPDX-License-Identifier: GPL-2.0-only
/*
 * Input driver for legacy hall-switch device-tree nodes.
 *
 * The binding predates the standard gpio-keys representation and stores the
 * GPIO specifier directly in "linux,gpio-int".
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/gpio/legacy.h>
#include <linux/gpio/machine.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeirq.h>
#include <linux/pm_wakeup.h>
#include <linux/property.h>

#define HALL_SWITCH_GPIO_PROPERTY "linux,gpio-int"

struct hall_switch {
	struct gpio_desc *gpio;
	struct input_dev *input;
	bool active_low;
};

static struct gpio_desc *hall_switch_get_gpio(struct device *dev,
					      bool *active_low)
{
	struct fwnode_reference_args args;
	struct gpio_device *gdev;
	struct gpio_desc *gpio;
	int gpio_num;
	int error;

	error = fwnode_property_get_reference_args(dev_fwnode(dev),
						   HALL_SWITCH_GPIO_PROPERTY,
						   "#gpio-cells", 0, 0,
						   &args);
	if (error)
		return ERR_PTR(error);

	if (args.nargs != 2 || args.args[1] & ~GPIO_ACTIVE_LOW) {
		error = -EINVAL;
		goto out_put_fwnode;
	}

	gdev = gpio_device_find_by_fwnode(args.fwnode);
	if (!gdev) {
		error = -EPROBE_DEFER;
		goto out_put_fwnode;
	}

	gpio = gpio_device_get_desc(gdev, args.args[0]);
	if (IS_ERR(gpio)) {
		error = PTR_ERR(gpio);
		goto out_put_gdev;
	}

	gpio_num = desc_to_gpio(gpio);
	if (gpio_num < 0) {
		error = gpio_num;
		goto out_put_gdev;
	}

	error = devm_gpio_request_one(dev, gpio_num, GPIOF_IN,
				      "hall-switch");
	if (error)
		goto out_put_gdev;

	*active_low = args.args[1] == GPIO_ACTIVE_LOW;
	gpio = gpio_to_desc(gpio_num);
	if (!gpio) {
		error = -ENODEV;
		goto out_put_gdev;
	}

	gpio_device_put(gdev);
	fwnode_handle_put(args.fwnode);

	return gpio;

out_put_gdev:
	gpio_device_put(gdev);
out_put_fwnode:
	fwnode_handle_put(args.fwnode);

	return ERR_PTR(error);
}

static int hall_switch_report(struct hall_switch *hall)
{
	int closed;

	closed = gpiod_get_raw_value_cansleep(hall->gpio);
	if (closed < 0)
		return closed;

	closed = !!closed ^ hall->active_low;
	input_report_switch(hall->input, SW_LID, closed);
	input_sync(hall->input);

	return 0;
}

static irqreturn_t hall_switch_irq(int irq, void *data)
{
	struct hall_switch *hall = data;
	int error;

	error = hall_switch_report(hall);
	if (error)
		dev_err_ratelimited(hall->input->dev.parent,
				    "failed to read hall GPIO: %d\n", error);

	return IRQ_HANDLED;
}

static int hall_switch_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct hall_switch *hall;
	struct input_dev *input;
	bool wakeup;
	int error;
	int irq;

	hall = devm_kzalloc(dev, sizeof(*hall), GFP_KERNEL);
	if (!hall)
		return -ENOMEM;

	hall->gpio = hall_switch_get_gpio(dev, &hall->active_low);
	if (IS_ERR(hall->gpio))
		return dev_err_probe(dev, PTR_ERR(hall->gpio),
				     "failed to acquire %s\n",
				     HALL_SWITCH_GPIO_PROPERTY);

	irq = gpiod_to_irq(hall->gpio);
	if (irq < 0)
		return dev_err_probe(dev, irq, "failed to map hall GPIO IRQ\n");

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	hall->input = input;
	input->name = "hall-switch";
	input->phys = "hall-switch/input0";
	input->id.bustype = BUS_HOST;
	input_set_capability(input, EV_SW, SW_LID);

	error = devm_request_threaded_irq(dev, irq, NULL, hall_switch_irq,
					  IRQF_TRIGGER_RISING |
					  IRQF_TRIGGER_FALLING |
					  IRQF_ONESHOT | IRQF_NO_AUTOEN,
					  dev_name(dev), hall);
	if (error)
		return dev_err_probe(dev, error,
				     "failed to request hall GPIO IRQ\n");

	error = input_register_device(input);
	if (error)
		return dev_err_probe(dev, error,
				     "failed to register input device\n");

	error = hall_switch_report(hall);
	if (error)
		return dev_err_probe(dev, error,
				     "failed to read initial hall state\n");

	wakeup = device_property_read_bool(dev, "linux,wakeup");
	if (wakeup) {
		error = devm_device_init_wakeup(dev);
		if (error)
			return dev_err_probe(dev, error,
					     "failed to enable wakeup\n");

		error = devm_pm_set_wake_irq(dev, irq);
		if (error)
			return dev_err_probe(dev, error,
					     "failed to set wake IRQ\n");
	}

	enable_irq(irq);
	platform_set_drvdata(pdev, hall);

	return 0;
}

static const struct of_device_id hall_switch_of_match[] = {
	{ .compatible = "hall-switch" },
	{ }
};
MODULE_DEVICE_TABLE(of, hall_switch_of_match);

static struct platform_driver hall_switch_driver = {
	.probe = hall_switch_probe,
	.driver = {
		.name = "hall-switch",
		.of_match_table = hall_switch_of_match,
	},
};
module_platform_driver(hall_switch_driver);

MODULE_DESCRIPTION("Legacy hall-switch lid sensor driver");
MODULE_LICENSE("GPL");
