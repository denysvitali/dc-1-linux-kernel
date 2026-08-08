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

#include <dt-bindings/memory/mtk-memory-port.h>
#include <dt-bindings/power/mediatek,mt6789-power.h>

#define MT6789_POWER_DOMAIN_NR	(MT6789_POWER_DOMAIN_CAM_RAWB + 1)

struct mt6789_pm_domain {
	struct generic_pm_domain display;
	struct generic_pm_domain *domains[MT6789_POWER_DOMAIN_NR];
	struct genpd_onecell_data onecell;
};

static __be32 mt6789_ovl_iommus_cells[4];
static struct property mt6789_ovl_iommus = {
	.name = "iommus",
	.length = sizeof(mt6789_ovl_iommus_cells),
	.value = mt6789_ovl_iommus_cells,
};

static int __init mt6789_legacy_display_iommu_fixup(void)
{
	struct device_node *iommu_node;
	struct device_node *ovl_node;
	int ret;

	/*
	 * Daylight's production DT predates the upstream IOMMU binding and
	 * omits OVL0's iommus property.  OVL0 is the DMA device selected by
	 * MediaTek DRM, so the omission makes framebuffer allocation bypass
	 * the IOMMU and fail above the 32-bit DMA aperture.
	 *
	 * Apply the firmware compatibility fix before the generic OF platform
	 * population arch_initcall.  The normal driver core can then defer OVL0
	 * until the IOMMU provider is ready and attach it exactly once.  Do not
	 * try to reconfigure an already-bound DMA device here.
	 */
	if (!of_machine_is_compatible("mediatek,MT6789"))
		return 0;

	iommu_node = of_find_compatible_node(NULL, NULL,
					     "mediatek,mt6789-disp-iommu");
	ovl_node = of_find_compatible_node(NULL, NULL, "mediatek,disp_ovl0");
	if (!iommu_node || !ovl_node) {
		pr_warn("MT6789: cannot find legacy display IOMMU nodes\n");
		ret = 0;
		goto out_put_nodes;
	}

	if (of_property_present(ovl_node, "iommus")) {
		ret = 0;
		goto out_put_nodes;
	}

	if (!iommu_node->phandle) {
		pr_warn("MT6789: display IOMMU has no phandle\n");
		ret = 0;
		goto out_put_nodes;
	}

	mt6789_ovl_iommus_cells[0] = cpu_to_be32(iommu_node->phandle);
	mt6789_ovl_iommus_cells[1] = cpu_to_be32(MTK_M4U_ID(0, 1));
	mt6789_ovl_iommus_cells[2] = cpu_to_be32(iommu_node->phandle);
	mt6789_ovl_iommus_cells[3] = cpu_to_be32(MTK_M4U_ID(0, 2));

	ret = of_add_property(ovl_node, &mt6789_ovl_iommus);
	if (ret)
		pr_warn("MT6789: failed to add OVL0 IOMMU ports: %d\n", ret);
	else
		pr_info("MT6789: added legacy OVL0 IOMMU ports before device population\n");

out_put_nodes:
	of_node_put(ovl_node);
	of_node_put(iommu_node);
	return ret;
}
core_initcall(mt6789_legacy_display_iommu_fixup);

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
