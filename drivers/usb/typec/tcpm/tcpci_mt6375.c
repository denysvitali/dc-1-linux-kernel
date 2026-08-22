// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek MT6375 Type-C Port Controller driver.
 *
 * The MT6375 PMU is a multi-address device; the TCPCI bank hangs off a
 * secondary I2C address and is otherwise a plain TCPCI register map, so this
 * driver leans on the generic TCPCI/TCPM cores and only applies the vendor's
 * PHY and timing patch at init -- those registers moved relative to the
 * MT6370 generation, and the software reset lives in SYSCTRL3.
 *
 * This board boots a signed bootloader device tree that cannot describe the
 * chip: the charger driver at the primary address spawns this one with the
 * secondary client, and the Type-C connector is described by a software node
 * carrying sink-only capabilities. Fixed PDOs stop at 12 V so every
 * negotiable voltage stays inside the charger's OVP buckets. No interrupt
 * line is described either, so alerts are polled; PD sources retransmit
 * their capabilities, which absorbs the poll latency while a contract forms.
 *
 * Settled contracts reach the charger through TCPM's per-port power supply
 * ("tcpm-source-psy-*"): its ONLINE/VOLTAGE_NOW/CURRENT_MAX mirror whatever
 * was negotiated, so a power-supply notifier is the sanctioned bridge for a
 * driver that cannot reach inside the opaque struct tcpci.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/string.h>
#include <linux/workqueue.h>
#include <linux/usb/tcpci.h>

/*
 * Vendor registers behind the standard TCPCI map; offsets and values from
 * the BSP's tcpc_mt6375.c. The bootloader leaves the port presenting Rd on
 * both CC pins -- a sink -- so role pulls need no special care here.
 */
#define MT6375_TCPC_PHYCTRL1		0x80
#define MT6375_TCPC_PHYCTRL2		0x81
#define MT6375_TCPC_PHYCTRL3		0x82
#define MT6375_TCPC_PHYCTRL7		0x86
#define MT6375_TCPC_PHYCTRL9		0xAC
#define MT6375_TCPC_VCONCTRL3		0x8C
#define MT6375_TCPC_SYSCTRL1		0x8F
#define MT6375_TCPC_SYSCTRL3		0xB0
#define MT6375_TCPC_TCPCCTRL1		0xB1
#define MT6375_TCPC_TCPCCTRL2		0xB2
#define MT6375_TCPC_TCPCCTRL3		0xB3
#define MT6375_TCPC_HILOCTRL9		0xC8
#define MT6375_TCPC_SHIELDCTRL1		0xCA
#define MT6375_TCPC_FOD			0xCF

#define MT6375_TCPC_VID			0x29cf
#define MT6375_TCPC_PID			0x6375

/* SYSCTRL1: route PD_IRQB through the 3 MHz path, no shipping mode, auto-idle */
#define MT6375_SYSCTRL1_INIT		0xA8
/* SHIELDCTRL1: treat a 40 ms CC open during system UVLO as detach */
#define MT6375_SHIELD_CC_OPEN_UVLO	BIT(4)
/* FOD enable lives in bit 6; the foreign-object feature stays off */
#define MT6375_FOD_EN			BIT(6)

#define MT6375_TCPC_POLL_MS		15

int mt6375_charger_program_input(u32 mv, u32 ma);

struct mt6375_tcpc_pdata {
	struct i2c_client *client;
};

struct mt6375_tcpc {
	struct device *dev;
	struct tcpci_data data;
	struct tcpci *tcpci;
	struct notifier_block psy_nb;
	struct delayed_work poll;
};

static const struct regmap_config mt6375_tcpc_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
	.cache_type = REGCACHE_NONE,
};

static const struct reg_sequence mt6375_tcpc_init_regs[] = {
	/* tTCPCFilter 250 us, DRP cycle 76.8 ms, DRP duty 33% */
	{ .reg = MT6375_TCPC_TCPCCTRL1,		.def = 0x0a },
	{ .reg = MT6375_TCPC_TCPCCTRL2,		.def = 0x04 },
	{ .reg = MT6375_TCPC_TCPCCTRL3,		.def = 0x4a },
	{ .reg = MT6375_TCPC_TCPCCTRL3 + 1,	.def = 0x01 },
	/* BMC PHY tuning: toggle count, CDR threshold, transition window,
	 * idle time, retry period */
	{ .reg = MT6375_TCPC_PHYCTRL1,		.def = 0x74 },
	{ .reg = MT6375_TCPC_PHYCTRL2,		.def = 0x3a },
	{ .reg = MT6375_TCPC_PHYCTRL3,		.def = 0x82 },
	{ .reg = MT6375_TCPC_PHYCTRL7,		.def = 0x36 },
	{ .reg = MT6375_TCPC_PHYCTRL9,		.def = 0x3c },
	/* Vconn current-limit mode, OCP 100 mA, analog OVP */
	{ .reg = MT6375_TCPC_VCONCTRL3,		.def = 0x11 },
	/* CC filter 250 us */
	{ .reg = MT6375_TCPC_HILOCTRL9,		.def = 0x0a },
};

/*
 * Sink capabilities: fixed PDOs only, 12 V ceiling. Everything above the
 * 11 V mark has no matching OVP bucket yet, and PPS would want continuous
 * input-voltage steering the policy does not implement.
 */
static const u32 mt6375_tcpc_sink_pdos[] = {
	0x0684b000,	/* Fixed 5 V, 3 A */
	0x0b84b000,	/* Fixed 9 V, 3 A */
	0x0f04b000,	/* Fixed 12 V, 3 A */
};

static const struct property_entry mt6375_tcpc_connector_props[] = {
	PROPERTY_ENTRY_STRING("compatible", "usb-c-connector"),
	PROPERTY_ENTRY_STRING("power-role", "sink"),
	PROPERTY_ENTRY_STRING("try-power-role", "sink"),
	PROPERTY_ENTRY_STRING("data-role", "device"),
	PROPERTY_ENTRY_U32_ARRAY("sink-pdos", mt6375_tcpc_sink_pdos),
	PROPERTY_ENTRY_U32("op-sink-microwatt", 10000000),
	{ }
};

static const struct software_node mt6375_tcpc_root_node = { };

static const struct software_node mt6375_tcpc_connector_node = {
	.name = "connector",
	.properties = mt6375_tcpc_connector_props,
	.parent = &mt6375_tcpc_root_node,
};

static void mt6375_tcpc_poll(struct work_struct *work)
{
	struct mt6375_tcpc *tcpc =
		container_of(work, struct mt6375_tcpc, poll.work);

	tcpci_irq(tcpc->tcpci);
	schedule_delayed_work(&tcpc->poll, msecs_to_jiffies(MT6375_TCPC_POLL_MS));
}

static int mt6375_tcpc_psy_notify(struct notifier_block *nb,
				  unsigned long action, void *v)
{
	struct power_supply *psy = v;
	union power_supply_propval val;
	u32 mv = 0, ma = 0;
	int ret;

	if (action != PSY_EVENT_PROP_CHANGED || !psy || !psy->desc ||
	    !psy->desc->name || !strstarts(psy->desc->name, "tcpm-source-psy"))
		return NOTIFY_DONE;

	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_ONLINE, &val);
	if (!ret && !val.intval) {
		mt6375_charger_program_input(0, 0);
		return NOTIFY_OK;
	}
	if (ret)
		return NOTIFY_DONE;

	if (!power_supply_get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val))
		mv = val.intval / 1000;
	if (!power_supply_get_property(psy, POWER_SUPPLY_PROP_CURRENT_MAX, &val))
		ma = val.intval / 1000;
	if (mv && ma)
		mt6375_charger_program_input(mv, ma);

	return NOTIFY_OK;
}

static int mt6375_tcpc_init(struct tcpci *tcpci, struct tcpci_data *data)
{
	struct regmap *regmap = data->regmap;
	int ret;

	ret = regmap_write(regmap, MT6375_TCPC_SYSCTRL3, 0x01);
	if (ret)
		return ret;
	usleep_range(1000, 2000);

	ret = regmap_multi_reg_write(regmap, mt6375_tcpc_init_regs,
				     ARRAY_SIZE(mt6375_tcpc_init_regs));
	if (ret)
		return ret;

	ret = regmap_set_bits(regmap, MT6375_TCPC_SYSCTRL1,
			      MT6375_SYSCTRL1_INIT);
	if (!ret)
		ret = regmap_set_bits(regmap, TCPC_TCPC_CTRL,
				      TCPC_TCPC_CTRL_EN_LK4CONN_ALRT);
	if (!ret)
		ret = regmap_set_bits(regmap, MT6375_TCPC_SHIELDCTRL1,
				      MT6375_SHIELD_CC_OPEN_UVLO);
	if (!ret)
		ret = regmap_clear_bits(regmap, MT6375_TCPC_FOD,
					MT6375_FOD_EN);
	if (ret)
		return ret;

	/* The watchdog would need kicking from TCPM; keep it off. */
	return regmap_read(regmap, TCPC_POWER_STATUS, &(unsigned int){ 0 });
}

static int mt6375_tcpc_probe(struct platform_device *pdev)
{
	const struct mt6375_tcpc_pdata *pdata = dev_get_platdata(&pdev->dev);
	struct i2c_client *client;
	struct mt6375_tcpc *tcpc;
	unsigned int lo, hi;
	int ret;

	if (!pdata || !pdata->client)
		return -EINVAL;
	client = pdata->client;

	tcpc = devm_kzalloc(&pdev->dev, sizeof(*tcpc), GFP_KERNEL);
	if (!tcpc)
		return -ENOMEM;
	tcpc->dev = &pdev->dev;

	tcpc->data.regmap = devm_regmap_init_i2c(client,
						 &mt6375_tcpc_regmap_config);
	if (IS_ERR(tcpc->data.regmap))
		return dev_err_probe(&pdev->dev, PTR_ERR(tcpc->data.regmap),
				     "failed to initialize regmap\n");

	ret = regmap_read(tcpc->data.regmap, TCPC_VENDOR_ID, &lo);
	if (!ret)
		ret = regmap_read(tcpc->data.regmap, TCPC_VENDOR_ID + 1, &hi);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "failed to read VID\n");
	if ((hi << 8 | lo) != MT6375_TCPC_VID)
		return dev_err_probe(&pdev->dev, -ENODEV,
				     "unexpected vendor ID 0x%02x%02x\n", hi, lo);

	ret = regmap_read(tcpc->data.regmap, TCPC_PRODUCT_ID, &lo);
	if (!ret)
		ret = regmap_read(tcpc->data.regmap, TCPC_PRODUCT_ID + 1, &hi);
	if (!ret && (hi << 8 | lo) != MT6375_TCPC_PID)
		dev_warn(&pdev->dev, "unexpected product ID 0x%02x%02x\n",
			 hi, lo);

	INIT_DELAYED_WORK(&tcpc->poll, mt6375_tcpc_poll);

	/* The TX buffer is contiguous behind the byte count. */
	tcpc->data.TX_BUF_BYTE_x_hidden = true;
	tcpc->data.init = mt6375_tcpc_init;

	ret = device_add_software_node(&pdev->dev, &mt6375_tcpc_root_node);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to add the connector swnode\n");
	ret = software_node_register(&mt6375_tcpc_connector_node);
	if (ret)
		goto err_remove_swnode;

	tcpc->tcpci = tcpci_register_port(&pdev->dev, &tcpc->data);
	if (IS_ERR(tcpc->tcpci)) {
		ret = dev_err_probe(&pdev->dev, PTR_ERR(tcpc->tcpci),
				    "failed to register TCPCI port\n");
		goto err_unregister_connector;
	}

	tcpc->psy_nb.notifier_call = mt6375_tcpc_psy_notify;
	ret = power_supply_reg_notifier(&tcpc->psy_nb);
	if (ret)
		goto err_unregister_port;

	platform_set_drvdata(pdev, tcpc);
	schedule_delayed_work(&tcpc->poll,
			      msecs_to_jiffies(MT6375_TCPC_POLL_MS));

	dev_info(&pdev->dev, "MT6375 Type-C port controller, polling alerts\n");
	return 0;

err_unregister_port:
	tcpci_unregister_port(tcpc->tcpci);
err_unregister_connector:
	software_node_unregister(&mt6375_tcpc_connector_node);
err_remove_swnode:
	device_remove_software_node(&pdev->dev);
	return ret;
}

static void mt6375_tcpc_remove(struct platform_device *pdev)
{
	struct mt6375_tcpc *tcpc = platform_get_drvdata(pdev);

	power_supply_unreg_notifier(&tcpc->psy_nb);
	cancel_delayed_work_sync(&tcpc->poll);
	tcpci_unregister_port(tcpc->tcpci);
	software_node_unregister(&mt6375_tcpc_connector_node);
	device_remove_software_node(&pdev->dev);
}

static const struct platform_device_id mt6375_tcpc_id[] = {
	{ "mt6375-tcpc", },
	{ }
};
MODULE_DEVICE_TABLE(platform, mt6375_tcpc_id);

static struct platform_driver mt6375_tcpc_driver = {
	.driver = {
		.name = "mt6375-tcpc",
	},
	.probe = mt6375_tcpc_probe,
	.remove = mt6375_tcpc_remove,
	.id_table = mt6375_tcpc_id,
};
module_platform_driver(mt6375_tcpc_driver);

MODULE_DESCRIPTION("MediaTek MT6375 Type-C Port Controller driver");
MODULE_LICENSE("GPL");
