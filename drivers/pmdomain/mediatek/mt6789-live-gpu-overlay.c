// SPDX-License-Identifier: GPL-2.0-only
/*
 * Daylight DC-1 MT6789 shipped-DT GPU bridge
 *
 * LK passes a legacy flat SCPSYS node and a downstream Mali node. Load the
 * exact reviewed overlay only after the cold-boot software display has been
 * observed working. Keep both rails at the hardware-observed 850 mV while
 * the nested MFG provider and Panfrost are in use.
 *
 * This module is intentionally permanent once initialized. Removing an OF
 * overlay beneath live genpd consumers is unsafe; reboot is the rollback.
 */

#include <crypto/hash.h>
#include <crypto/sha2.h>

#include <linux/bits.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/string.h>

#define DC1_GPU_OVERLAY_FIRMWARE \
	"mediatek/mt8781-daylight-jagar-live-gpu-probe.dtbo"
#define DC1_GPU_OVERLAY_SIZE	3015

#define DC1_SCPSYS_PATH		"/power-controller@10006000"
#define DC1_GPU_PATH		"/soc/mali@13000000"
#define DC1_MFG_CONTROLLER_NAME	"power-controller"

#define DC1_GPU_UV		850000
#define DC1_SPM_PWR_STATUS	0x16c
#define DC1_SPM_PWR_STATUS_2ND	0x170
#define DC1_MFG_STATUS_MASK	GENMASK(5, 2)

/* CI artifact from kernel commit 2098713f7e1d; source is unchanged here. */
static const u8 dc1_gpu_overlay_sha256[SHA256_DIGEST_SIZE] = {
	0x1e, 0x57, 0xe8, 0x39, 0x1e, 0x55, 0xe1, 0xa7,
	0xef, 0xbb, 0x9d, 0xdc, 0xac, 0x8a, 0x0b, 0xa6,
	0x26, 0x75, 0x08, 0xd4, 0xf7, 0x10, 0x32, 0x18,
	0x23, 0xa8, 0x45, 0xa3, 0xec, 0x2a, 0x70, 0xbd,
};

static const char * const dc1_scpsys_compat[] = {
	"mediatek,mt6789-scpsys",
	"syscon",
};

static const char * const dc1_scpsys_overlay_compat[] = {
	"mediatek,mt6789-scpsys",
	"syscon",
	"simple-mfd",
};

static const char * const dc1_gpu_legacy_compat[] = {
	"mediatek,mali",
	"arm,mali-valhall",
};

static const char * const dc1_gpu_panfrost_compat[] = {
	"mediatek,mt6789-mali",
	"arm,mali-valhall-jm",
};

/* Retained for the lifetime of this deliberately non-removable module. */
static struct regulator *dc1_vgpu;
static struct regulator *dc1_vsram_gpu;
static int dc1_gpu_overlay_id;

static bool dc1_string_list_is(const struct device_node *node,
			       const char *property,
			       const char * const *expected,
			       size_t count)
{
	const char *value;
	int actual_count;
	size_t i;

	actual_count = of_property_count_strings(node, property);
	if (actual_count < 0 || (size_t)actual_count != count)
		return false;

	for (i = 0; i < count; i++) {
		if (of_property_read_string_index(node, property, i, &value) ||
		    strcmp(value, expected[i]))
			return false;
	}

	return true;
}

static int dc1_require_domain(struct device_node *node, u32 expected_id)
{
	u32 id;

	if (!node || of_property_read_u32(node, "reg", &id) ||
	    id != expected_id ||
	    !of_find_property(node, "domain-supply", NULL))
		return -EINVAL;

	return 0;
}

static int dc1_require_leaf_domain(struct device_node *node, u32 expected_id)
{
	u32 id;

	if (!node || of_property_read_u32(node, "reg", &id) ||
	    id != expected_id)
		return -EINVAL;

	return 0;
}

static struct device_node *dc1_get_domain_child(struct device_node *parent,
						 u32 expected_id)
{
	struct device_node *child;
	u32 id;

	/*
	 * child->name is "power-domain", without the unit address.  Looking up
	 * "power-domain@2" with of_get_child_by_name() therefore misses a node
	 * whose full path visibly contains that exact string.  The binding's reg
	 * value is the stable identity and is validated again below.
	 */
	for_each_child_of_node(parent, child) {
		if (!of_property_read_u32(child, "reg", &id) && id == expected_id)
			return child;
	}

	return NULL;
}

static int dc1_check_cold_base(struct device_node **scpsys_out,
			       struct device_node **gpu_out,
			       struct platform_device **scpsys_pdev_out,
			       struct platform_device **gpu_pdev_out)
{
	struct platform_device *scpsys_pdev = NULL;
	struct platform_device *gpu_pdev = NULL;
	struct device_node *scpsys;
	struct device_node *gpu;
	struct device_node *child;
	struct regmap *spm;
	u32 status, status_2nd;
	int ret;

	if (!of_machine_is_compatible("mediatek,MT6789"))
		return -ENODEV;

	scpsys = of_find_node_by_path(DC1_SCPSYS_PATH);
	gpu = of_find_node_by_path(DC1_GPU_PATH);
	if (!scpsys || !gpu) {
		ret = -ENODEV;
		goto out_put_nodes;
	}

	if (!dc1_string_list_is(scpsys, "compatible", dc1_scpsys_compat,
				ARRAY_SIZE(dc1_scpsys_compat)) ||
	    !dc1_string_list_is(gpu, "compatible", dc1_gpu_legacy_compat,
				ARRAY_SIZE(dc1_gpu_legacy_compat))) {
		ret = -EINVAL;
		goto out_put_nodes;
	}

	/* Rollback may depopulate this node, so require an empty shipped base. */
	child = of_get_next_child(scpsys, NULL);
	if (child) {
		of_node_put(child);
		ret = -EALREADY;
		goto out_put_nodes;
	}

	scpsys_pdev = of_find_device_by_node(scpsys);
	gpu_pdev = of_find_device_by_node(gpu);
	if (!scpsys_pdev || !gpu_pdev) {
		ret = -ENODEV;
		goto out_put_devices;
	}
	if (device_is_bound(&gpu_pdev->dev)) {
		ret = -EBUSY;
		goto out_put_devices;
	}

	spm = syscon_node_to_regmap(scpsys);
	if (IS_ERR(spm)) {
		ret = PTR_ERR(spm);
		goto out_put_devices;
	}
	ret = regmap_read(spm, DC1_SPM_PWR_STATUS, &status);
	if (ret)
		goto out_put_devices;
	ret = regmap_read(spm, DC1_SPM_PWR_STATUS_2ND, &status_2nd);
	if (ret)
		goto out_put_devices;
	if ((status | status_2nd) & DC1_MFG_STATUS_MASK) {
		pr_err("DC-1 GPU: refusing non-cold MFG state: %08x/%08x\n",
		       status, status_2nd);
		ret = -EBUSY;
		goto out_put_devices;
	}

	*scpsys_out = scpsys;
	*gpu_out = gpu;
	*scpsys_pdev_out = scpsys_pdev;
	*gpu_pdev_out = gpu_pdev;
	return 0;

out_put_devices:
	if (gpu_pdev)
		platform_device_put(gpu_pdev);
	if (scpsys_pdev)
		platform_device_put(scpsys_pdev);
out_put_nodes:
	of_node_put(gpu);
	of_node_put(scpsys);
	return ret;
}

static int dc1_check_overlay_firmware(const struct firmware *firmware)
{
	struct crypto_shash *tfm;
	struct shash_desc *desc;
	u8 digest[SHA256_DIGEST_SIZE];
	size_t desc_size;
	int ret;

	if (firmware->size != DC1_GPU_OVERLAY_SIZE)
		return -EINVAL;

	tfm = crypto_alloc_shash("sha256", 0, 0);
	if (IS_ERR(tfm))
		return PTR_ERR(tfm);
	if (crypto_shash_digestsize(tfm) != sizeof(digest)) {
		ret = -EINVAL;
		goto out_free_tfm;
	}

	desc_size = sizeof(*desc) + crypto_shash_descsize(tfm);
	desc = kzalloc(desc_size, GFP_KERNEL);
	if (!desc) {
		ret = -ENOMEM;
		goto out_free_tfm;
	}
	desc->tfm = tfm;
	ret = crypto_shash_digest(desc, firmware->data, firmware->size, digest);
	kfree(desc);
	if (ret)
		goto out_free_tfm;

	if (memcmp(digest, dc1_gpu_overlay_sha256, sizeof(digest))) {
		pr_err("DC-1 GPU: refusing unreviewed overlay firmware\n");
		ret = -EKEYREJECTED;
		goto out_free_tfm;
	}

	ret = 0;
out_free_tfm:
	crypto_free_shash(tfm);
	return ret;
}

static int dc1_set_enable_rail(struct regulator *regulator, const char *name)
{
	int voltage;
	int ret;

	ret = regulator_is_enabled(regulator);
	if (ret < 0) {
		pr_err("DC-1 GPU: %s is-enabled query failed: %d\n", name, ret);
		return ret;
	}
	if (ret)
		pr_info("DC-1 GPU: %s inherited enabled; acquiring a consumer hold\n",
			name);

	voltage = regulator_get_voltage(regulator);
	if (voltage < 0) {
		pr_err("DC-1 GPU: %s voltage query failed: %d\n", name,
		       voltage);
		return voltage;
	}
	if (voltage != DC1_GPU_UV) {
		pr_err("DC-1 GPU: refusing %s at unexpected %d uV\n", name,
		       voltage);
		return -ERANGE;
	}

	ret = regulator_set_voltage(regulator, DC1_GPU_UV, DC1_GPU_UV);
	if (ret) {
		pr_err("DC-1 GPU: %s voltage set failed: %d\n", name, ret);
		return ret;
	}
	voltage = regulator_get_voltage(regulator);
	if (voltage != DC1_GPU_UV) {
		pr_err("DC-1 GPU: %s voltage verify failed: %d\n", name,
		       voltage);
		return -ERANGE;
	}

	ret = regulator_enable(regulator);
	if (ret) {
		pr_err("DC-1 GPU: %s enable failed: %d\n", name, ret);
		return ret;
	}
	ret = regulator_is_enabled(regulator);
	if (ret <= 0) {
		pr_err("DC-1 GPU: %s enable verify failed: %d\n", name, ret);
		regulator_disable(regulator);
		return ret ? ret : -EIO;
	}

	return 0;
}

static void dc1_put_nodes(struct device_node *mfg3,
			  struct device_node *mfg2,
			  struct device_node *mfg1,
			  struct device_node *mfg0,
			  struct device_node *controller,
			  struct device_node *gpu,
			  struct device_node *scpsys)
{
	of_node_put(mfg3);
	of_node_put(mfg2);
	of_node_put(mfg1);
	of_node_put(mfg0);
	of_node_put(controller);
	of_node_put(gpu);
	of_node_put(scpsys);
}

static void dc1_log_children(struct device_node *parent, const char *label)
{
	struct device_node *child;

	pr_err("DC-1 GPU: children below %s (%pOF):\n", label, parent);
	for_each_child_of_node(parent, child)
		pr_err("DC-1 GPU:   %pOF\n", child);
}

static int __init dc1_gpu_overlay_init(void)
{
	struct platform_device *controller_pdev = NULL;
	struct platform_device *scpsys_pdev = NULL;
	struct platform_device *gpu_pdev = NULL;
	struct device_node *controller = NULL;
	struct device_node *scpsys = NULL;
	struct device_node *mfg0 = NULL;
	struct device_node *mfg1 = NULL;
	struct device_node *mfg2 = NULL;
	struct device_node *mfg3 = NULL;
	struct device_node *gpu = NULL;
	struct regulator *vgpu = NULL;
	struct regulator *vsram = NULL;
	const struct firmware *firmware;
	bool populated = false;
	bool vgpu_enabled = false;
	bool vsram_enabled = false;
	int overlay_id = -1;
	int rollback_ret;
	int ret;

	ret = dc1_check_cold_base(&scpsys, &gpu, &scpsys_pdev, &gpu_pdev);
	if (ret) {
		pr_err("DC-1 GPU: cold-DT preflight failed: %d\n", ret);
		return ret;
	}
	pr_info("DC-1 GPU: cold-DT preflight passed\n");

	ret = request_firmware_direct(&firmware, DC1_GPU_OVERLAY_FIRMWARE,
				      &scpsys_pdev->dev);
	if (ret) {
		pr_err("DC-1 GPU: cannot load %s: %d\n",
		       DC1_GPU_OVERLAY_FIRMWARE, ret);
		goto out_put_base;
	}
	ret = dc1_check_overlay_firmware(firmware);
	if (ret)
		pr_err("DC-1 GPU: overlay firmware validation failed: %d\n", ret);
	if (!ret)
		ret = of_overlay_fdt_apply(firmware->data, firmware->size,
					   &overlay_id, NULL);
	release_firmware(firmware);
	if (ret) {
		pr_err("DC-1 GPU: overlay apply failed: %d\n", ret);
		goto out_remove_overlay;
	}
	pr_info("DC-1 GPU: overlay applied as id %d\n", overlay_id);

	controller = of_get_child_by_name(scpsys, DC1_MFG_CONTROLLER_NAME);
	if (!controller) {
		pr_err("DC-1 GPU: overlay controller child is missing\n");
		dc1_log_children(scpsys, "SCPSYS");
		ret = -EINVAL;
		goto out_bad_overlay;
	}
	mfg0 = dc1_get_domain_child(controller, 2);
	if (!mfg0) {
		pr_err("DC-1 GPU: overlay MFG0 child is missing\n");
		dc1_log_children(controller, "MFG controller");
		ret = -EINVAL;
		goto out_bad_overlay;
	}
	mfg1 = dc1_get_domain_child(mfg0, 3);
	if (!mfg1) {
		pr_err("DC-1 GPU: overlay MFG1 child is missing\n");
		dc1_log_children(mfg0, "MFG0");
		ret = -EINVAL;
		goto out_bad_overlay;
	}
	mfg2 = dc1_get_domain_child(mfg1, 4);
	mfg3 = dc1_get_domain_child(mfg1, 5);
	if (!of_device_is_compatible(controller,
				     "mediatek,mt6789-power-controller")) {
		pr_err("DC-1 GPU: overlay controller compatible mismatch\n");
		ret = -EINVAL;
		goto out_bad_overlay;
	}
	ret = dc1_require_domain(mfg0, 2);
	if (ret) {
		pr_err("DC-1 GPU: overlay MFG0 semantic mismatch: %d\n", ret);
		goto out_bad_overlay;
	}
	ret = dc1_require_domain(mfg1, 3);
	if (ret) {
		pr_err("DC-1 GPU: overlay MFG1 semantic mismatch: %d\n", ret);
		goto out_bad_overlay;
	}
	ret = dc1_require_leaf_domain(mfg2, 4);
	if (ret) {
		pr_err("DC-1 GPU: overlay MFG2 semantic mismatch: %d\n", ret);
		goto out_bad_overlay;
	}
	ret = dc1_require_leaf_domain(mfg3, 5);
	if (ret) {
		pr_err("DC-1 GPU: overlay MFG3 semantic mismatch: %d\n", ret);
		goto out_bad_overlay;
	}
	if (!dc1_string_list_is(scpsys, "compatible",
				dc1_scpsys_overlay_compat,
				ARRAY_SIZE(dc1_scpsys_overlay_compat))) {
		pr_err("DC-1 GPU: overlay SCPSYS compatible mismatch (count %d)\n",
		       of_property_count_strings(scpsys, "compatible"));
		ret = -EINVAL;
		goto out_bad_overlay;
	}
	if (!dc1_string_list_is(gpu, "compatible", dc1_gpu_panfrost_compat,
				ARRAY_SIZE(dc1_gpu_panfrost_compat))) {
		pr_err("DC-1 GPU: overlay Mali compatible mismatch (count %d)\n",
		       of_property_count_strings(gpu, "compatible"));
		ret = -EINVAL;
		goto out_bad_overlay;
	}

	vgpu = of_regulator_get(&scpsys_pdev->dev, mfg0, "domain");
	if (IS_ERR(vgpu)) {
		ret = PTR_ERR(vgpu);
		pr_err("DC-1 GPU: VGPU regulator lookup failed: %d\n", ret);
		vgpu = NULL;
		goto out_remove_overlay;
	}
	vsram = of_regulator_get(&scpsys_pdev->dev, mfg1, "domain");
	if (IS_ERR(vsram)) {
		ret = PTR_ERR(vsram);
		pr_err("DC-1 GPU: VSRAM_GPU regulator lookup failed: %d\n", ret);
		vsram = NULL;
		goto out_put_regulators;
	}

	ret = dc1_set_enable_rail(vgpu, "VGPU");
	if (ret)
		goto out_put_regulators;
	vgpu_enabled = true;
	pr_info("DC-1 GPU: VGPU enabled at 850 mV\n");

	ret = dc1_set_enable_rail(vsram, "VSRAM_GPU");
	if (ret)
		goto out_disable_rails;
	vsram_enabled = true;
	pr_info("DC-1 GPU: VSRAM_GPU enabled at 850 mV\n");

	/* This is the last check from which complete rollback is possible. */
	if (device_is_bound(&gpu_pdev->dev)) {
		ret = -EBUSY;
		pr_err("DC-1 GPU: GPU bound before bridge population\n");
		goto out_disable_rails;
	}

	/*
	 * The shipped node was not a populated bus when platform enumeration ran.
	 * Merely adding simple-mfd at runtime therefore cannot create this child;
	 * populate the one reviewed child explicitly.
	 */
	populated = true;
	ret = of_platform_populate(scpsys, NULL, NULL, &scpsys_pdev->dev);
	if (ret) {
		pr_err("DC-1 GPU: nested controller population failed: %d\n",
		       ret);
		goto out_depopulate;
	}
	pr_info("DC-1 GPU: nested controller populated\n");

	controller_pdev = of_find_device_by_node(controller);
	if (!controller_pdev || !device_is_bound(&controller_pdev->dev)) {
		ret = -ENODEV;
		pr_err("DC-1 GPU: nested MFG power controller did not bind\n");
		goto out_depopulate;
	}

	/*
	 * A bound controller has no remove callback, so this is an irreversible
	 * commit point.  The exact compatible has only the expected driver; if
	 * that invariant is ever broken, retain every resource and require reboot.
	 */
	if (!controller_pdev->dev.driver ||
	    strcmp(controller_pdev->dev.driver->name, "mtk-power-controller")) {
		pr_crit("DC-1 GPU: unexpected bound controller; resources retained, reboot required\n");
		dc1_vgpu = vgpu;
		dc1_vsram_gpu = vsram;
		dc1_gpu_overlay_id = overlay_id;
		ret = -EUCLEAN;
		goto out_irreversible;
	}

	dc1_vgpu = vgpu;
	dc1_vsram_gpu = vsram;
	dc1_gpu_overlay_id = overlay_id;
	ret = 0;
	pr_info("DC-1 GPU: live-DT bridge ready; rails held at 850 mV, overlay id %d\n",
		overlay_id);

out_irreversible:
	platform_device_put(controller_pdev);
	platform_device_put(gpu_pdev);
	platform_device_put(scpsys_pdev);
	dc1_put_nodes(mfg3, mfg2, mfg1, mfg0, controller, gpu, scpsys);
	return ret;

out_bad_overlay:
	pr_err("DC-1 GPU: applied overlay failed semantic validation\n");
out_depopulate:
	if (controller_pdev)
		platform_device_put(controller_pdev);
	if (populated)
		of_platform_depopulate(&scpsys_pdev->dev);
out_disable_rails:
	if (vsram_enabled)
		regulator_disable(vsram);
	if (vgpu_enabled)
		regulator_disable(vgpu);
out_put_regulators:
	if (vsram)
		regulator_put(vsram);
	if (vgpu)
		regulator_put(vgpu);
out_remove_overlay:
	/* Drop every reference to overlay-created nodes before removal. */
	of_node_put(mfg3);
	mfg3 = NULL;
	of_node_put(mfg2);
	mfg2 = NULL;
	of_node_put(mfg1);
	mfg1 = NULL;
	of_node_put(mfg0);
	mfg0 = NULL;
	of_node_put(controller);
	controller = NULL;
	if (overlay_id >= 0) {
		rollback_ret = of_overlay_remove(&overlay_id);
		if (rollback_ret)
			pr_err("DC-1 GPU: overlay rollback failed: %d\n",
			       rollback_ret);
	}
out_put_base:
	platform_device_put(gpu_pdev);
	platform_device_put(scpsys_pdev);
	dc1_put_nodes(mfg3, mfg2, mfg1, mfg0, controller, gpu, scpsys);
	return ret;
}
module_init(dc1_gpu_overlay_init);

MODULE_DESCRIPTION("Daylight DC-1 MT6789 shipped-DT GPU overlay loader");
MODULE_AUTHOR("Daylight Linux bring-up contributors");
MODULE_FIRMWARE(DC1_GPU_OVERLAY_FIRMWARE);
MODULE_LICENSE("GPL");
