// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek MT6789/MT8781 tinysys SCP (sensor co-processor) driver.
 *
 * The SCP firmware (scp_a.img) is loaded by the bootloader, so this driver
 * has no loader. It maps the SCP's control and mailbox registers, reports
 * liveness, and wires up the five tinysys mailboxes (mtk-mbox) that the
 * sensorhub IPI path (Phase 2) will ride on.
 *
 * This is deliberately minimal: no DVFS, logger, L1-cache, exception or
 * hwvoter support yet. Those come later and only if the sensor path needs
 * them -- the SCP is already running and clocked by the bootloader.
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/soc/mediatek/mtk-mbox.h>
#include <linux/soc/mediatek/mt6789-scp.h>

static struct scp_regs scpreg;

/* Five tinysys mailboxes; recv/send pin tables are added in Phase 2. */
static struct mtk_mbox_info scp_mbox_info[SCP_MBOX_TOTAL];

struct mtk_mbox_device scp_mboxdev = {
	.name = "scp_mboxdev",
	.info_table = scp_mbox_info,
	.count = SCP_MBOX_TOTAL,
	.recv_count = 0,
	.send_count = 0,
};
EXPORT_SYMBOL_GPL(scp_mboxdev);

static int mt6789_scp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	u32 status, pc;
	int i, ret;

	scpreg.cfg = devm_platform_ioremap_resource_byname(pdev, "scp_cfgreg");
	scpreg.clkctrl = devm_platform_ioremap_resource_byname(pdev, "scp_clkreg");
	scpreg.cfg_core0 = devm_platform_ioremap_resource_byname(pdev, "scp_cfgreg_core0");
	scpreg.cfg_core1 = devm_platform_ioremap_resource_byname(pdev, "scp_cfgreg_core1");
	scpreg.bus_tracker = devm_platform_ioremap_resource_byname(pdev, "scp_bus_tracker");
	scpreg.l1cctrl = devm_platform_ioremap_resource_byname(pdev, "scp_l1creg");
	scpreg.cfg_sec = devm_platform_ioremap_resource_byname(pdev, "scp_cfgreg_sec");

	if (IS_ERR(scpreg.cfg) || IS_ERR(scpreg.cfg_core0)) {
		dev_err(dev, "missing SCP control registers\n");
		return -ENODEV;
	}

	/* Liveness: the bootloader-loaded firmware parks in WFI until the AP
	 * sends a wake IPI, so HALT/GATED clear and MON_PC==0 is the expected
	 * idle state, not a fault.
	 */
	status = readl(scpreg.cfg_core0 + SCP_CORE_STATUS);
	pc = readl(scpreg.cfg_core0 + SCP_CORE_MON_PC);
	dev_info(dev, "SCP core0 status=0x%x (halt=%d gated=%d) mon_pc=0x%x\n",
		 status, !!(status & SCP_CORE_HALT),
		 !!(status & SCP_CORE_GATED), pc);

	for (i = 0; i < SCP_MBOX_TOTAL; i++) {
		ret = mtk_mbox_probe(pdev, &scp_mboxdev, i);
		if (ret)
			dev_warn(dev, "mbox %d probe failed ret=%d\n", i, ret);
	}

	dev_info(dev, "SCP mailboxes wired (count=%d)\n", SCP_MBOX_TOTAL);
	return 0;
}

static const struct of_device_id mt6789_scp_of_match[] = {
	{ .compatible = "mediatek,mt6789-tinysys-scp" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6789_scp_of_match);

static struct platform_driver mt6789_scp_driver = {
	.probe = mt6789_scp_probe,
	.driver = {
		.name = "mt6789-scp",
		.of_match_table = mt6789_scp_of_match,
	},
};
module_platform_driver(mt6789_scp_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek MT6789 tinysys SCP driver");
MODULE_AUTHOR("dc-1-pmos");
