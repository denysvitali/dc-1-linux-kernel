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

#include <linux/device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>

#include <dt-bindings/memory/mtk-memory-port.h>
#include <dt-bindings/power/mediatek,mt6789-power.h>

#define MT6789_POWER_DOMAIN_NR	(MT6789_POWER_DOMAIN_CAM_RAWB + 1)

static struct generic_pm_domain mt6789_display_domain = {
	.name = "disp",
	.flags = GENPD_FLAG_ALWAYS_ON,
};

static struct generic_pm_domain *mt6789_domains[MT6789_POWER_DOMAIN_NR] = {
	[MT6789_POWER_DOMAIN_DISP] = &mt6789_display_domain,
};

static struct genpd_onecell_data mt6789_onecell = {
	.domains = mt6789_domains,
	.num_domains = ARRAY_SIZE(mt6789_domains),
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

static int __init mt6789_pm_domain_init(void)
{
	struct device_node *node;
	int ret;

	if (!of_machine_is_compatible("mediatek,MT6789"))
		return 0;

	node = of_find_compatible_node(NULL, NULL, "mediatek,mt6789-scpsys");
	if (!node) {
		pr_warn("MT6789: cannot find legacy power-domain node\n");
		return 0;
	}

	/* The display domain is handed over by the boot firmware in the on state. */
	ret = pm_genpd_init(&mt6789_display_domain, NULL, false);
	if (ret)
		goto out_put_node;

	ret = of_genpd_add_provider_onecell(node, &mt6789_onecell);
	if (ret) {
		pm_genpd_remove(&mt6789_display_domain);
		goto out_put_node;
	}

	/* The provider retains its own node reference for the boot lifetime. */
	pr_info("MT6789: registered bootloader-owned display power domain early\n");
	of_node_put(node);
	return 0;

out_put_node:
	of_node_put(node);
	return ret;
}
postcore_initcall(mt6789_pm_domain_init);

static int __init mt6789_attach_legacy_supplier(const char *path)
{
	struct platform_device *pdev;
	struct device_node *node;
	int ret;

	node = of_find_node_by_path(path);
	if (!node)
		return -ENODEV;

	pdev = of_find_device_by_node(node);
	of_node_put(node);
	if (!pdev)
		return -ENODEV;

	ret = pdev->dev.driver ? 1 : device_attach(&pdev->dev);
	platform_device_put(pdev);
	return ret > 0 ? 0 : ret ?: -ENODEV;
}

static int __init mt6789_legacy_display_supplier_init(void)
{
	static const char * const paths[] = {
		"/soc/dispsys_config@14000000",
		"/soc/smi_disp_comm@14002000",
		"/soc/smi_larb0@14003000",
		"/soc/smi_larb1@14004000",
		"/soc/iommu@14016000",
	};
	int i, ret;

	if (!of_machine_is_compatible("mediatek,MT6789"))
		return 0;

	/*
	 * The production DT's display supplier chain is deeper than the single
	 * pass made before built-in deferred probing gives up on missing drivers.
	 * At this point all ordinary device initcalls (including MMSYS clocks and
	 * SMI) have run, but initcalls_done is still false.  Probe the chain in
	 * dependency order so the generic late deferred sweep sees a registered
	 * IOMMU when it retries OVL0.
	 */
	for (i = 0; i < ARRAY_SIZE(paths); i++) {
		ret = mt6789_attach_legacy_supplier(paths[i]);
		if (ret) {
			pr_warn("MT6789: failed to attach display supplier %s: %d\n",
				paths[i], ret);
			return 0;
		}
	}

	pr_info("MT6789: attached legacy display suppliers before deferred probe\n");
	return 0;
}
late_initcall(mt6789_legacy_display_supplier_init);

MODULE_DESCRIPTION("MediaTek MT6789 legacy power-domain handoff");
MODULE_LICENSE("GPL");
