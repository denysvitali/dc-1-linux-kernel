// SPDX-License-Identifier: GPL-2.0-only
/*
 * Daylight DC-1 MT6789 shipped-DT audio power-domain bridge
 *
 * LK passes a flat legacy SCPSYS node, and mt6789-pm-domain.c registers only
 * the bootloader-owned DISP domain on it. The shipped AFE node asks that same
 * node for MT6789_POWER_DOMAIN_AUDIO, finds no provider, and gives up with
 * -ETIMEDOUT once the deferred-probe window closes -- which is why the device
 * has no sound card at all.
 *
 * AUDIO cannot be handed over the way DISP is: SPM PWR_STATUS and
 * PWR_STATUS_2ND bit 22 both read 0 on a booted device, so the domain really is
 * off and really has to be sequenced. Load the packaged overlay, which adds a
 * nested provider carrying only that one domain -- driven by the generic
 * MediaTek PM-domain driver and the reviewed MT6789 register data -- and points
 * the AFE at it. The domain is registered off and the AFE's runtime PM is what
 * powers it up, so loading this module does not by itself energise anything.
 *
 * This is the same shape as the MFG bridge in mt6789-live-gpu-overlay.c, and
 * the two are independent: each refuses to run twice by looking for its own
 * controller child, so either order works.
 *
 * Like that bridge, this module is permanent once it succeeds: removing an OF
 * overlay from beneath a live genpd consumer is unsafe, and reboot is the
 * rollback.
 */

#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/string.h>

#include <dt-bindings/power/mediatek,mt6789-power.h>

#define DC1_AUDIO_OVERLAY_FIRMWARE \
	"mediatek/mt8781-daylight-jagar-live-audio-probe.dtbo"

#define DC1_SCPSYS_PATH			"/power-controller@10006000"
#define DC1_AFE_PATH			"/soc/mt6789-afe-pcm@11210000"
#define DC1_AUDIO_CONTROLLER_NAME	"power-controller-audio"
#define DC1_PM_DOMAIN_DRIVER		"mtk-power-controller"

#define DC1_SPM_PWR_STATUS	0x16c
#define DC1_SPM_PWR_STATUS_2ND	0x170
#define DC1_AUDIO_STATUS_MASK	BIT(22)

/* Retained for the lifetime of this deliberately non-removable module. */
static int dc1_audio_overlay_id = -1;

/* Does the AFE ask @target for the AUDIO domain? */
static bool dc1_afe_targets(struct device_node *afe, struct device_node *target)
{
	struct of_phandle_args pd_args;
	bool match;

	if (of_parse_phandle_with_args(afe, "power-domains",
				       "#power-domain-cells", 0, &pd_args))
		return false;

	match = pd_args.np == target && pd_args.args_count == 1 &&
		pd_args.args[0] == MT6789_POWER_DOMAIN_AUDIO;
	of_node_put(pd_args.np);

	return match;
}

/*
 * Everything the overlay assumes about the shipped tree, checked before it is
 * applied: an SCPSYS node without our controller (with it, the overlay is
 * already in), an AFE that still asks the flat node for AUDIO, and an AFE that
 * has not been bound -- a bound AFE means something already solved this, and
 * retargeting a live consumer's power domain is not something to do blind.
 */
static int dc1_check_base(struct device_node **scpsys_out,
			  struct device_node **afe_out,
			  struct platform_device **scpsys_pdev_out)
{
	struct platform_device *scpsys_pdev = NULL;
	struct platform_device *afe_pdev = NULL;
	struct device_node *scpsys;
	struct device_node *afe;
	struct device_node *child;
	int ret;

	if (!of_machine_is_compatible("mediatek,MT6789"))
		return -ENODEV;

	scpsys = of_find_node_by_path(DC1_SCPSYS_PATH);
	afe = of_find_node_by_path(DC1_AFE_PATH);
	if (!scpsys || !afe) {
		ret = -ENODEV;
		goto out_put_nodes;
	}

	child = of_get_child_by_name(scpsys, DC1_AUDIO_CONTROLLER_NAME);
	if (child) {
		of_node_put(child);
		ret = -EALREADY;
		goto out_put_nodes;
	}

	if (!dc1_afe_targets(afe, scpsys)) {
		pr_err("DC-1 audio: the AFE does not ask the flat SCPSYS for AUDIO\n");
		ret = -EINVAL;
		goto out_put_nodes;
	}

	scpsys_pdev = of_find_device_by_node(scpsys);
	afe_pdev = of_find_device_by_node(afe);
	if (!scpsys_pdev || !afe_pdev) {
		ret = -ENODEV;
		goto out_put_devices;
	}
	if (device_is_bound(&afe_pdev->dev)) {
		ret = -EBUSY;
		goto out_put_devices;
	}

	platform_device_put(afe_pdev);
	*scpsys_out = scpsys;
	*afe_out = afe;
	*scpsys_pdev_out = scpsys_pdev;
	return 0;

out_put_devices:
	if (afe_pdev)
		platform_device_put(afe_pdev);
	if (scpsys_pdev)
		platform_device_put(scpsys_pdev);
out_put_nodes:
	of_node_put(afe);
	of_node_put(scpsys);
	return ret;
}

/*
 * Report, but do not gate on, the domain's handoff state. Both the "off"
 * expected here and an unexpected "on" are handled by the generic driver, which
 * sequences the domain from whichever state it finds; the value is worth having
 * in the log next to any later AFE failure.
 */
static void dc1_report_handoff_state(struct device_node *scpsys)
{
	struct regmap *spm;
	u32 status, status_2nd;

	spm = syscon_node_to_regmap(scpsys);
	if (IS_ERR(spm))
		return;

	if (regmap_read(spm, DC1_SPM_PWR_STATUS, &status) ||
	    regmap_read(spm, DC1_SPM_PWR_STATUS_2ND, &status_2nd))
		return;

	pr_info("DC-1 audio: AUDIO domain at handoff: %s (%08x/%08x)\n",
		((status & status_2nd) & DC1_AUDIO_STATUS_MASK) ? "on" : "off",
		status, status_2nd);
}

/*
 * Confirm the applied overlay is the one this module was written for: the
 * nested controller exists with the expected compatible, carries exactly the
 * AUDIO domain, and the AFE now asks it -- not the flat node -- for that
 * domain. This stands in for the GPU bridge's hash pinning: the blob and this
 * module ship in the same package, so what matters is not which blob it is but
 * that the tree it produced is the one the AFE can probe against.
 */
static int dc1_check_applied(struct device_node *scpsys, struct device_node *afe,
			     struct device_node **controller_out)
{
	struct device_node *controller;
	struct device_node *domain;
	u32 id;
	int ret = -EINVAL;

	controller = of_get_child_by_name(scpsys, DC1_AUDIO_CONTROLLER_NAME);
	if (!controller) {
		pr_err("DC-1 audio: the overlay added no controller child\n");
		return -ENODEV;
	}

	if (!of_device_is_compatible(controller,
				     "mediatek,mt6789-power-controller")) {
		pr_err("DC-1 audio: controller compatible mismatch\n");
		goto out_put_controller;
	}

	domain = of_get_next_child(controller, NULL);
	if (!domain) {
		pr_err("DC-1 audio: controller carries no domain\n");
		ret = -ENODEV;
		goto out_put_controller;
	}
	if (of_property_read_u32(domain, "reg", &id) ||
	    id != MT6789_POWER_DOMAIN_AUDIO ||
	    !of_property_present(domain, "mediatek,infracfg")) {
		pr_err("DC-1 audio: domain child is not the reviewed AUDIO domain\n");
		of_node_put(domain);
		goto out_put_controller;
	}
	of_node_put(domain);

	if (!dc1_afe_targets(afe, controller)) {
		pr_err("DC-1 audio: the AFE was not retargeted at the new controller\n");
		goto out_put_controller;
	}

	*controller_out = controller;
	return 0;

out_put_controller:
	of_node_put(controller);
	return ret;
}

static int __init dc1_audio_overlay_init(void)
{
	struct platform_device *controller_pdev = NULL;
	struct platform_device *scpsys_pdev = NULL;
	struct device_node *controller = NULL;
	struct device_node *scpsys = NULL;
	struct device_node *afe = NULL;
	const struct firmware *overlay;
	int overlay_id = -1;
	int ret;

	ret = dc1_check_base(&scpsys, &afe, &scpsys_pdev);
	if (ret) {
		/*
		 * -EALREADY is the second modprobe of an applied bridge, which
		 * is a no-op and not a failure.
		 */
		pr_info("DC-1 audio: not applying the overlay: %d\n", ret);
		return ret == -EALREADY ? 0 : ret;
	}

	dc1_report_handoff_state(scpsys);

	ret = request_firmware_direct(&overlay, DC1_AUDIO_OVERLAY_FIRMWARE,
				      &scpsys_pdev->dev);
	if (ret) {
		pr_err("DC-1 audio: cannot load %s: %d\n",
		       DC1_AUDIO_OVERLAY_FIRMWARE, ret);
		goto out_put_base;
	}
	ret = of_overlay_fdt_apply(overlay->data, overlay->size, &overlay_id,
				   NULL);
	release_firmware(overlay);
	if (ret) {
		pr_err("DC-1 audio: overlay apply failed: %d\n", ret);
		goto out_put_base;
	}
	pr_info("DC-1 audio: overlay applied as id %d\n", overlay_id);

	ret = dc1_check_applied(scpsys, afe, &controller);
	if (ret)
		goto out_remove_overlay;

	/*
	 * The shipped SCPSYS node was not a populated bus when platform
	 * enumeration ran, so adding simple-mfd at runtime does not by itself
	 * create this child -- unless the GPU bridge already populated the node,
	 * in which case the OF reconfiguration notifier has created it for us
	 * and this call skips it. Either way the device exists afterwards.
	 *
	 * Note this is also the point of no return: never depopulate the SCPSYS
	 * node on a later error, because its children may include the GPU
	 * bridge's live MFG provider.
	 */
	ret = of_platform_populate(scpsys, NULL, NULL, &scpsys_pdev->dev);
	if (ret) {
		pr_err("DC-1 audio: nested controller population failed: %d\n",
		       ret);
		goto out_put_base;
	}

	controller_pdev = of_find_device_by_node(controller);
	if (!controller_pdev || !device_is_bound(&controller_pdev->dev)) {
		pr_err("DC-1 audio: the nested AUDIO power controller did not bind\n");
		ret = -ENODEV;
		goto out_put_base;
	}

	/*
	 * A bound controller owns a live genpd, which makes this the
	 * irreversible commit point: keep the overlay whatever happens next.
	 */
	if (!controller_pdev->dev.driver ||
	    strcmp(controller_pdev->dev.driver->name, DC1_PM_DOMAIN_DRIVER)) {
		pr_crit("DC-1 audio: unexpected bound controller; resources retained, reboot required\n");
		dc1_audio_overlay_id = overlay_id;
		ret = -EUCLEAN;
		goto out_put_base;
	}

	dc1_audio_overlay_id = overlay_id;
	ret = 0;
	pr_info("DC-1 audio: AUDIO power domain provider ready; re-probe %s to bring up the AFE\n",
		DC1_AFE_PATH);
	goto out_put_base;

out_remove_overlay:
	if (overlay_id >= 0) {
		int rollback = of_overlay_remove(&overlay_id);

		if (rollback)
			pr_err("DC-1 audio: overlay rollback failed: %d\n",
			       rollback);
	}
out_put_base:
	if (controller_pdev)
		platform_device_put(controller_pdev);
	of_node_put(controller);
	platform_device_put(scpsys_pdev);
	of_node_put(afe);
	of_node_put(scpsys);
	return ret;
}

module_init(dc1_audio_overlay_init);

MODULE_DESCRIPTION("MediaTek MT6789 shipped-DT audio power-domain bridge");
MODULE_FIRMWARE(DC1_AUDIO_OVERLAY_FIRMWARE);
MODULE_LICENSE("GPL");
