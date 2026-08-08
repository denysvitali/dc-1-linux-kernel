// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6789 legacy power-domain binding support
 *
 * Some production MT6789 firmware passes a flat SCPSYS node using the old
 * one-cell binding.  The boot firmware leaves the display domain powered,
 * but without a provider Linux defers every display SMI and IOMMU
 * consumer.
 *
 * Register the bootloader-owned display domain as always-on.  This is a
 * deliberately conservative handoff: it makes no SCPSYS register writes.
 * Full power sequencing can replace it once the remaining vendor sequences
 * have been validated against the generic MediaTek PM-domain framework.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/slab.h>

#include <dt-bindings/power/mediatek,mt6789-power.h>

#define MT6789_POWER_DOMAIN_NR	(MT6789_POWER_DOMAIN_CAM_RAWB + 1)

struct mt6789_pm_domain {
	struct generic_pm_domain display;
	struct generic_pm_domain *domains[MT6789_POWER_DOMAIN_NR];
	struct genpd_onecell_data onecell;
};

static int mt6789_pm_domain_probe(struct platform_device *pdev)
{
	struct mt6789_pm_domain *priv;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->display.name = "disp";
	priv->display.flags = GENPD_FLAG_ALWAYS_ON;

	/* The display domain is handed over by the boot firmware in the on state. */
	ret = pm_genpd_init(&priv->display, NULL, false);
	if (ret)
		return ret;

	priv->domains[MT6789_POWER_DOMAIN_DISP] = &priv->display;
	priv->onecell.domains = priv->domains;
	priv->onecell.num_domains = ARRAY_SIZE(priv->domains);

	ret = of_genpd_add_provider_onecell(pdev->dev.of_node,
					    &priv->onecell);
	if (ret) {
		pm_genpd_remove(&priv->display);
		return dev_err_probe(&pdev->dev, ret,
				     "failed to register display power domain\n");
	}

	platform_set_drvdata(pdev, priv);
	dev_info(&pdev->dev,
		 "registered bootloader-owned display power domain\n");

	return 0;
}

static void mt6789_pm_domain_remove(struct platform_device *pdev)
{
	struct mt6789_pm_domain *priv = platform_get_drvdata(pdev);

	of_genpd_del_provider(pdev->dev.of_node);
	pm_genpd_remove(&priv->display);
}

static const struct of_device_id mt6789_pm_domain_of_match[] = {
	{ .compatible = "mediatek,mt6789-scpsys" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6789_pm_domain_of_match);

static struct platform_driver mt6789_pm_domain_driver = {
	.probe = mt6789_pm_domain_probe,
	.remove = mt6789_pm_domain_remove,
	.driver = {
		.name = "mt6789-pm-domain",
		.suppress_bind_attrs = true,
		.of_match_table = mt6789_pm_domain_of_match,
	},
};
builtin_platform_driver(mt6789_pm_domain_driver);

MODULE_DESCRIPTION("MediaTek MT6789 legacy power-domain handoff");
MODULE_LICENSE("GPL");
