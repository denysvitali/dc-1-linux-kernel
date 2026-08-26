// SPDX-License-Identifier: GPL-2.0-only
/*
 * Novatek NT36523 DriverIC panels driver
 *
 * Copyright (c) 2022, 2023 Jianhua Lu <lujianhua000@gmail.com>
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/display/drm_dsc.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#define DSI_NUM_MIN 1

struct panel_info {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi[2];
	const struct panel_desc *desc;
	enum drm_panel_orientation orientation;

	struct gpio_desc *reset_gpio;
	struct backlight_device *backlight;
	struct regulator *vddio;
	struct regulator_bulk_data bias_supplies[2];
	struct drm_dsc_config dsc;
	bool jagar_bias_enabled;
};

struct panel_desc {
	unsigned int width_mm;
	unsigned int height_mm;

	unsigned int bpc;
	unsigned int lanes;
	unsigned long hs_rate;
	unsigned long mode_flags;
	enum mipi_dsi_pixel_format format;

	const struct drm_display_mode *modes;
	unsigned int num_modes;
	const struct mipi_dsi_device_info dsi_info;
	int (*init_sequence)(struct panel_info *pinfo);

	bool is_dual_dsi;
	bool has_dcs_backlight;
	bool has_jagar_power_sequence;
};

static bool sharp_nt36523n_production_sequence;
module_param_named(jagar_production_sequence,
		   sharp_nt36523n_production_sequence, bool, 0644);
MODULE_PARM_DESC(jagar_production_sequence,
		 "Use the DC-1 production panel power and initialization sequence");

/*
 * The shipped FDT lets the jagar panel probe advance through several pieces of
 * hardware ownership at once: inherited VDDI, the two bias regulators, and the
 * reset GPIO. The staging exists so a live console can advance one resource
 * group at a time, and a failed stage can be recovered through the other A/B
 * slot without conflating those operations.
 *
 * Defaulting this to 3 was tried on 2026-08-09 and DOES NOT BOOT: the full
 * sequence completes fine when advanced from a live console at t=275s, but
 * running it from probe during boot never reaches userspace, and LK falls back
 * to the other slot. Keep the default at the boot-proven hold and advance it at
 * runtime (device/userspace/sway-test/panel-up.sh).
 */
/*
 * The production MP panel runs only in the vendor's DSC mode family.  Its
 * default 60 Hz mode retains the 120 Hz active-line rate and lowers the frame
 * rate with a 1716-line VFP.  The corresponding factory D-PHY rate is
 * 672 Mbps/lane.  Keep this settable before jagar_probe_stage reaches 3 so a
 * diagnostic image can still override it.
 */
static unsigned long sharp_nt36523n_hs_rate = 672000000;
module_param_named(jagar_hs_rate, sharp_nt36523n_hs_rate, ulong, 0644);
MODULE_PARM_DESC(jagar_hs_rate,
		 "DC-1 peripheral HS data rate in Hz (0 = derive from the mode)");

static unsigned int sharp_nt36523n_probe_stage;
module_param_named(jagar_probe_stage, sharp_nt36523n_probe_stage, uint, 0644);
MODULE_PARM_DESC(jagar_probe_stage,
		 "DC-1 panel probe stage: 0=hold, 1=VDDI, 2=bias, 3=reset/attach");

static inline struct panel_info *to_panel_info(struct drm_panel *panel)
{
	return container_of(panel, struct panel_info, panel);
}

static int elish_boe_init_sequence(struct panel_info *pinfo)
{
	struct mipi_dsi_device *dsi0 = pinfo->dsi[0];
	struct mipi_dsi_device *dsi1 = pinfo->dsi[1];
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = NULL };
	/* No datasheet, so write magic init sequence directly */
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0x10);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xb9, 0x05);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0x20);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x18, 0x40);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0x10);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xb9, 0x02);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0x23);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x00, 0x80);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x01, 0x84);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x05, 0x2d);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x06, 0x00);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x07, 0x00);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x08, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x09, 0x45);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x11, 0x02);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x12, 0x80);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x15, 0x83);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x16, 0x0c);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x29, 0x0a);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x30, 0xff);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x31, 0xfe);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x32, 0xfd);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x33, 0xfb);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x34, 0xf8);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x35, 0xf5);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x36, 0xf3);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x37, 0xf2);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x38, 0xf2);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x39, 0xf2);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x3a, 0xef);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x3b, 0xec);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x3d, 0xe9);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x3f, 0xe5);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x40, 0xe5);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x41, 0xe5);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x2a, 0x13);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x45, 0xff);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x46, 0xf4);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x47, 0xe7);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x48, 0xda);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x49, 0xcd);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x4a, 0xc0);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x4b, 0xb3);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x4c, 0xb2);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x4d, 0xb2);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x4e, 0xb2);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x4f, 0x99);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x50, 0x80);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x51, 0x68);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x52, 0x66);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x53, 0x66);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x54, 0x66);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x2b, 0x0e);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x58, 0xff);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x59, 0xfb);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x5a, 0xf7);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x5b, 0xf3);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x5c, 0xef);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x5d, 0xe3);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x5e, 0xda);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x5f, 0xd8);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x60, 0xd8);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x61, 0xd8);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x62, 0xcb);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x63, 0xbf);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x64, 0xb3);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x65, 0xb2);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x66, 0xb2);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x67, 0xb2);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0x2a);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x25, 0x47);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x30, 0x47);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x39, 0x47);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0x26);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x19, 0x10);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x1a, 0xe0);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x1b, 0x10);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x1c, 0x00);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x2a, 0x10);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x2b, 0xe0);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0x10);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0xf0);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x84, 0x08);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x85, 0x0c);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0x20);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x51, 0x00);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0x25);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x91, 0x1f);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x92, 0x0f);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x93, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x94, 0x18);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x95, 0x03);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x96, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0x10);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xb0, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0x25);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x19, 0x1f);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x1b, 0x1b);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0x24);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xb8, 0x28);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0x27);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xd0, 0x31);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xd1, 0x20);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xd2, 0x30);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xd4, 0x08);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xde, 0x80);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xdf, 0x02);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0x26);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x00, 0x81);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x01, 0xb0);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0x22);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x9f, 0x50);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x6f, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x70, 0x11);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x73, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x74, 0x49);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x76, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x77, 0x49);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xa0, 0x3f);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xa9, 0x50);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xaa, 0x28);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xab, 0x28);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xad, 0x10);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xb8, 0x00);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xb9, 0x49);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xba, 0x49);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xbb, 0x49);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xbe, 0x04);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xbf, 0x49);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xc0, 0x04);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xc1, 0x59);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xc2, 0x00);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xc5, 0x00);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xc6, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xc7, 0x48);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xca, 0x43);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xcb, 0x3c);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xce, 0x00);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xcf, 0x43);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xd0, 0x3c);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xd3, 0x43);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xd4, 0x3c);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xd7, 0x00);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xdc, 0x43);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xdd, 0x3c);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xe1, 0x43);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xe2, 0x3c);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xf2, 0x00);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xf3, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xf4, 0x48);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0x25);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x13, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x14, 0x23);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xbc, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xbd, 0x23);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0x2a);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x97, 0x3c);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x98, 0x02);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x99, 0x95);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x9a, 0x03);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x9b, 0x00);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x9c, 0x0b);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x9d, 0x0a);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x9e, 0x90);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0x22);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x9f, 0x50);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0x23);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xa3, 0x50);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0xe0);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x14, 0x60);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x16, 0xc0);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x4f, 0x02);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0xf0);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x3a, 0x08);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0xd0);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x02, 0xaf);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x09, 0xee);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x1c, 0x99);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x1d, 0x09);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0x10);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x51, 0x0f, 0xff);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x53, 0x2c);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x35, 0x00);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xbb, 0x13);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x3b, 0x03, 0xac, 0x1a, 0x04, 0x04);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x11);
	mipi_dsi_msleep(&dsi_ctx, 70);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x29);

	return dsi_ctx.accum_err;
}

static int elish_csot_init_sequence(struct panel_info *pinfo)
{
	struct mipi_dsi_device *dsi0 = pinfo->dsi[0];
	struct mipi_dsi_device *dsi1 = pinfo->dsi[1];
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = NULL };
	/* No datasheet, so write magic init sequence directly */
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0x10);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xb9, 0x05);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0x20);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x18, 0x40);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0x10);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xb9, 0x02);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0xd0);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x02, 0xaf);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x00, 0x30);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x09, 0xee);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x1c, 0x99);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x1d, 0x09);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0xf0);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x3a, 0x08);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0xe0);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x4f, 0x02);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0x20);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x58, 0x40);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0x10);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x35, 0x00);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0x23);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x00, 0x80);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x01, 0x84);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x05, 0x2d);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x06, 0x00);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x07, 0x00);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x08, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x09, 0x45);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x11, 0x02);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x12, 0x80);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x15, 0x83);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x16, 0x0c);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x29, 0x0a);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x30, 0xff);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x31, 0xfe);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x32, 0xfd);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x33, 0xfb);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x34, 0xf8);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x35, 0xf5);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x36, 0xf3);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x37, 0xf2);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x38, 0xf2);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x39, 0xf2);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x3a, 0xef);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x3b, 0xec);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x3d, 0xe9);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x3f, 0xe5);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x40, 0xe5);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x41, 0xe5);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x2a, 0x13);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x45, 0xff);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x46, 0xf4);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x47, 0xe7);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x48, 0xda);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x49, 0xcd);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x4a, 0xc0);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x4b, 0xb3);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x4c, 0xb2);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x4d, 0xb2);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x4e, 0xb2);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x4f, 0x99);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x50, 0x80);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x51, 0x68);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x52, 0x66);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x53, 0x66);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x54, 0x66);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x2b, 0x0e);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x58, 0xff);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x59, 0xfb);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x5a, 0xf7);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x5b, 0xf3);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x5c, 0xef);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x5d, 0xe3);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x5e, 0xda);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x5f, 0xd8);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x60, 0xd8);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x61, 0xd8);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x62, 0xcb);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x63, 0xbf);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x64, 0xb3);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x65, 0xb2);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x66, 0xb2);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x67, 0xb2);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0x10);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x51, 0x0f, 0xff);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x53, 0x2c);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x55, 0x00);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xbb, 0x13);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x3b, 0x03, 0xac, 0x1a, 0x04, 0x04);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0x2a);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x25, 0x46);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x30, 0x46);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x39, 0x46);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0x26);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x01, 0xb0);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x19, 0x10);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x1a, 0xe0);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x1b, 0x10);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x1c, 0x00);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x2a, 0x10);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x2b, 0xe0);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0xf0);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x84, 0x08);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x85, 0x0c);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0x20);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x51, 0x00);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0x25);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x91, 0x1f);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x92, 0x0f);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x93, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x94, 0x18);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x95, 0x03);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x96, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0x10);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xb0, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0x25);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x19, 0x1f);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x1b, 0x1b);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0x24);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xb8, 0x28);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0x27);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xd0, 0x31);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xd1, 0x20);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xd4, 0x08);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xde, 0x80);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xdf, 0x02);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0x26);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x00, 0x81);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x01, 0xb0);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0x22);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x6f, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x70, 0x11);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x73, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x74, 0x4d);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xa0, 0x3f);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xa9, 0x50);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xaa, 0x28);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xab, 0x28);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xad, 0x10);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xb8, 0x00);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xb9, 0x4b);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xba, 0x96);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xbb, 0x4b);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xbe, 0x07);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xbf, 0x4b);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xc0, 0x07);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xc1, 0x5c);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xc2, 0x00);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xc5, 0x00);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xc6, 0x3f);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xc7, 0x00);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xca, 0x08);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xcb, 0x40);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xce, 0x00);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xcf, 0x08);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xd0, 0x40);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xd3, 0x08);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xd4, 0x40);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0x25);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xbc, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xbd, 0x1c);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0x2a);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x9a, 0x03);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0x10);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x11);
	mipi_dsi_msleep(&dsi_ctx, 70);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0x29);

	return dsi_ctx.accum_err;
}

static int j606f_boe_init_sequence(struct panel_info *pinfo)
{
	struct mipi_dsi_device *dsi = pinfo->dsi[0];
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi };

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x05, 0xd9);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x07, 0x78);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x08, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0d, 0x63);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0e, 0x91);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0f, 0x73);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x95, 0xeb);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x96, 0xeb);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_PARTIAL_ROWS, 0x11);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6d, 0x66);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x75, 0xa2);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x77, 0xb3);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x00, 0x08, 0x00, 0x23, 0x00, 0x4d, 0x00, 0x6d,
				     0x00, 0x89, 0x00, 0xa1, 0x00, 0xb6, 0x00, 0xc9);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb1, 0x00, 0xda, 0x01, 0x13, 0x01, 0x3c, 0x01, 0x7e,
				     0x01, 0xab, 0x01, 0xf7, 0x02, 0x2f, 0x02, 0x31);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb2, 0x02, 0x67, 0x02, 0xa6, 0x02, 0xd1, 0x03, 0x08,
				     0x03, 0x2e, 0x03, 0x5b, 0x03, 0x6b, 0x03, 0x7b);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb3, 0x03, 0x8e, 0x03, 0xa2, 0x03, 0xb7, 0x03, 0xe7,
				     0x03, 0xfd, 0x03, 0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb4, 0x00, 0x08, 0x00, 0x23, 0x00, 0x4d, 0x00, 0x6d,
				     0x00, 0x89, 0x00, 0xa1, 0x00, 0xb6, 0x00, 0xc9);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb5, 0x00, 0xda, 0x01, 0x13, 0x01, 0x3c, 0x01, 0x7e,
				     0x01, 0xab, 0x01, 0xf7, 0x02, 0x2f, 0x02, 0x31);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb6, 0x02, 0x67, 0x02, 0xa6, 0x02, 0xd1, 0x03, 0x08,
				     0x03, 0x2e, 0x03, 0x5b, 0x03, 0x6b, 0x03, 0x7b);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb7, 0x03, 0x8e, 0x03, 0xa2, 0x03, 0xb7, 0x03, 0xe7,
				     0x03, 0xfd, 0x03, 0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb8, 0x00, 0x08, 0x00, 0x23, 0x00, 0x4d, 0x00, 0x6d,
				     0x00, 0x89, 0x00, 0xa1, 0x00, 0xb6, 0x00, 0xc9);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb9, 0x00, 0xda, 0x01, 0x13, 0x01, 0x3c, 0x01, 0x7e,
				     0x01, 0xab, 0x01, 0xf7, 0x02, 0x2f, 0x02, 0x31);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xba, 0x02, 0x67, 0x02, 0xa6, 0x02, 0xd1, 0x03, 0x08,
				     0x03, 0x2e, 0x03, 0x5b, 0x03, 0x6b, 0x03, 0x7b);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbb, 0x03, 0x8e, 0x03, 0xa2, 0x03, 0xb7, 0x03, 0xe7,
				     0x03, 0xfd, 0x03, 0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x21);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x00, 0x00, 0x00, 0x1b, 0x00, 0x45, 0x00, 0x65,
				     0x00, 0x81, 0x00, 0x99, 0x00, 0xae, 0x00, 0xc1);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb1, 0x00, 0xd2, 0x01, 0x0b, 0x01, 0x34, 0x01, 0x76,
				     0x01, 0xa3, 0x01, 0xef, 0x02, 0x27, 0x02, 0x29);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb2, 0x02, 0x5f, 0x02, 0x9e, 0x02, 0xc9, 0x03, 0x00,
				     0x03, 0x26, 0x03, 0x53, 0x03, 0x63, 0x03, 0x73);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb3, 0x03, 0x86, 0x03, 0x9a, 0x03, 0xaf, 0x03, 0xdf,
				     0x03, 0xf5, 0x03, 0xf7);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb4, 0x00, 0x00, 0x00, 0x1b, 0x00, 0x45, 0x00, 0x65,
				     0x00, 0x81, 0x00, 0x99, 0x00, 0xae, 0x00, 0xc1);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb5, 0x00, 0xd2, 0x01, 0x0b, 0x01, 0x34, 0x01, 0x76,
				     0x01, 0xa3, 0x01, 0xef, 0x02, 0x27, 0x02, 0x29);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb6, 0x02, 0x5f, 0x02, 0x9e, 0x02, 0xc9, 0x03, 0x00,
				     0x03, 0x26, 0x03, 0x53, 0x03, 0x63, 0x03, 0x73);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb7, 0x03, 0x86, 0x03, 0x9a, 0x03, 0xaf, 0x03, 0xdf,
				     0x03, 0xf5, 0x03, 0xf7);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb8, 0x00, 0x00, 0x00, 0x1b, 0x00, 0x45, 0x00, 0x65,
				     0x00, 0x81, 0x00, 0x99, 0x00, 0xae, 0x00, 0xc1);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb9, 0x00, 0xd2, 0x01, 0x0b, 0x01, 0x34, 0x01, 0x76,
				     0x01, 0xa3, 0x01, 0xef, 0x02, 0x27, 0x02, 0x29);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xba, 0x02, 0x5f, 0x02, 0x9e, 0x02, 0xc9, 0x03, 0x00,
				     0x03, 0x26, 0x03, 0x53, 0x03, 0x63, 0x03, 0x73);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbb, 0x03, 0x86, 0x03, 0x9a, 0x03, 0xaf, 0x03, 0xdf,
				     0x03, 0xf5, 0x03, 0xf7);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x23);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x80);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x07, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x11, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x12, 0x77);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x15, 0x07);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x16, 0x07);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x24);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x01, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x02, 0x1c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x03, 0x1c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x04, 0x1d);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x05, 0x1d);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x06, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x07, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x08, 0x0f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x09, 0x0f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0a, 0x0e);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0b, 0x0e);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0c, 0x0d);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0d, 0x0d);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0e, 0x0c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0f, 0x0c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x10, 0x08);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x11, 0x08);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x12, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x13, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x14, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x15, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x16, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x17, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x18, 0x1c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x19, 0x1c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1a, 0x1d);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1b, 0x1d);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1c, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1d, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1e, 0x0f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1f, 0x0f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x20, 0x0e);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x21, 0x0e);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x22, 0x0d);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x23, 0x0d);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x24, 0x0c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x25, 0x0c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_GAMMA_CURVE, 0x08);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x27, 0x08);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x28, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x29, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x2a, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x2b, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_LUT, 0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x2f, 0x0a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_PARTIAL_ROWS, 0x44);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x33, 0x0c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x34, 0x32);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x37, 0x44);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x38, 0x40);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x39, 0x00);

	mipi_dsi_dcs_set_pixel_format_multi(&dsi_ctx, 0x9a);

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x3b, 0xa0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_3D_CONTROL, 0x42);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x3f, 0x06);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x43, 0x06);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x47, 0x66);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4a, 0x9a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4b, 0xa0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4c, 0x91);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4d, 0x21);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4e, 0x43);

	mipi_dsi_dcs_set_display_brightness_multi(&dsi_ctx, 18);

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x52, 0x34);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x55, 0x82, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x56, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x58, 0x21);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x59, 0x30);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5a, 0xba);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5b, 0xa0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_CABC_MIN_BRIGHTNESS, 0x00, 0x06);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5f, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x65, 0x82);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x7e, 0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x7f, 0x3c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x82, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x97, 0xc0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb6,
				     0x05, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05,
				     0x05, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x92, 0xc4);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x93, 0x1a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x94, 0x5f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd7, 0x55);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xda, 0x0a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xde, 0x08);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xdb, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xdc, 0xc4);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xdd, 0x22);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xdf, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe0, 0xc4);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe1, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe2, 0xc4);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe3, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe4, 0xc4);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe5, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe6, 0xc4);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5c, 0x88);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5d, 0x08);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x8d, 0x88);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x8e, 0x08);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb5, 0x90);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x25);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x05, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x19, 0x07);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1f, 0xba);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x20, 0xa0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_GAMMA_CURVE, 0xba);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x27, 0xa0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x33, 0xba);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x34, 0xa0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x3f, 0xe0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_VSYNC_TIMING, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x44, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_GET_SCANLINE, 0x40);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x48, 0xba);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x49, 0xa0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5b, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5c, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5d, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_CABC_MIN_BRIGHTNESS, 0xd0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x61, 0xba);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x62, 0xa0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf1, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x2a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x64, 0x16);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x67, 0x16);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6a, 0x16);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x70, 0x30);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_READ_PPS_START, 0xf3);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xa3, 0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xa4, 0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xa5, 0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd6, 0x08);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x26);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xa1);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0a, 0xf2);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x04, 0x28);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x06, 0x30);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0c, 0x13);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0d, 0x0a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0f, 0x0a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x11, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x12, 0x50);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x13, 0x51);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x14, 0x65);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x15, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x16, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x17, 0xa0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x18, 0x86);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x19, 0x11);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1a, 0x7b);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1b, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1c, 0xbb);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x22, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x23, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x2a, 0x11);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x2b, 0x7b);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1d, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1e, 0xc3);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1f, 0xc3);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x24, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x25, 0xc3);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x2f, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_PARTIAL_ROWS, 0xc3);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_PARTIAL_COLUMNS, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x32, 0xc3);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x39, 0x00);

	mipi_dsi_dcs_set_pixel_format_multi(&dsi_ctx, 0xc3);

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x20, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x33, 0x11);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x34, 0x78);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x35, 0x16);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc8, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc9, 0x82);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xca, 0x4e);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcb, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_READ_PPS_CONTINUE, 0x4c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xaa, 0x47);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x27);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x56, 0x06);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x58, 0x80);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x59, 0x53);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5a, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5b, 0x14);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5c, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5d, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_CABC_MIN_BRIGHTNESS, 0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5f, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x60, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x61, 0x1d);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x62, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x63, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x64, 0x24);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x65, 0x1c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x66, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x67, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x68, 0x25);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x78, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc3, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd1, 0x24);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd2, 0x30);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x2a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x22, 0x2f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x23, 0x08);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x24, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x25, 0xc3);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_GAMMA_CURVE, 0xf8);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x27, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x28, 0x1a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x29, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x2a, 0x1a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x2b, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_LUT, 0x1a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0xe0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x14, 0x60);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x16, 0xc0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0xf0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);

	mipi_dsi_dcs_set_pixel_format_multi(&dsi_ctx, 0x08);

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x24);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);

	mipi_dsi_dcs_set_pixel_format_multi(&dsi_ctx, 0x5d);

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x3b, 0x60);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4a, 0x5d);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4b, 0x60);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5a, 0x70);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5b, 0x60);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x91, 0x44);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x92, 0x75);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xdb, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xdc, 0x75);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xdd, 0x22);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xdf, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe0, 0x75);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe1, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe2, 0x75);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe3, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe4, 0x75);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe5, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe6, 0x75);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5c, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5d, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x8d, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x8e, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x25);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1f, 0x70);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x20, 0x60);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_GAMMA_CURVE, 0x70);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x27, 0x60);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x33, 0x70);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x34, 0x60);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x48, 0x70);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x49, 0x60);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5b, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x61, 0x70);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x62, 0x60);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x26);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x02, 0x31);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x19, 0x0a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1a, 0x7f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1b, 0x0a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1c, 0x0c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x2a, 0x0a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x2b, 0x7f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1e, 0x75);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1f, 0x75);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x25, 0x75);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_PARTIAL_ROWS, 0x75);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_PARTIAL_COLUMNS, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x32, 0x8d);

	mipi_dsi_dcs_set_pixel_format_multi(&dsi_ctx, 0x75);

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x2a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x25, 0x75);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb9, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x18, 0x40);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb9, 0x02);

	mipi_dsi_dcs_set_tear_on_multi(&dsi_ctx, MIPI_DSI_DCS_TEAR_MODE_VBLANK);

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbb, 0x13);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x3b, 0x03, 0x5f, 0x1a, 0x04, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x10);
	mipi_dsi_usleep_range(&dsi_ctx, 10000, 11000);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);

	mipi_dsi_dcs_set_display_brightness_multi(&dsi_ctx, 0);

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY, 0x2c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_POWER_SAVE, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x68, 0x05, 0x01);

	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 100);

	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 30);

	return dsi_ctx.accum_err;
}

static const struct drm_display_mode elish_boe_modes[] = {
	{
		.clock = (1600 + 60 + 8 + 60) * (2560 + 26 + 4 + 168) * 120 / 1000,
		.hdisplay = 1600,
		.hsync_start = 1600 + 60,
		.hsync_end = 1600 + 60 + 8,
		.htotal = 1600 + 60 + 8 + 60,
		.vdisplay = 2560,
		.vsync_start = 2560 + 26,
		.vsync_end = 2560 + 26 + 4,
		.vtotal = 2560 + 26 + 4 + 168,
	},
};

static const struct drm_display_mode elish_csot_modes[] = {
	{
		.clock = (1600 + 200 + 40 + 52) * (2560 + 26 + 4 + 168) * 120 / 1000,
		.hdisplay = 1600,
		.hsync_start = 1600 + 200,
		.hsync_end = 1600 + 200 + 40,
		.htotal = 1600 + 200 + 40 + 52,
		.vdisplay = 2560,
		.vsync_start = 2560 + 26,
		.vsync_end = 2560 + 26 + 4,
		.vtotal = 2560 + 26 + 4 + 168,
	},
};

static const struct drm_display_mode j606f_boe_modes[] = {
	{
		.clock = (1200 + 58 + 2 + 60) * (2000 + 26 + 2 + 93) * 60 / 1000,
		.hdisplay = 1200,
		.hsync_start = 1200 + 58,
		.hsync_end = 1200 + 58 + 2,
		.htotal = 1200 + 58 + 2 + 60,
		.vdisplay = 2000,
		.vsync_start = 2000 + 26,
		.vsync_end = 2000 + 26 + 2,
		.vtotal = 2000 + 26 + 2 + 93,
		.width_mm = 143,
		.height_mm = 235,
	},
};

static const struct panel_desc elish_boe_desc = {
	.modes = elish_boe_modes,
	.num_modes = ARRAY_SIZE(elish_boe_modes),
	.dsi_info = {
		.type = "BOE-elish",
		.channel = 0,
		.node = NULL,
	},
	.width_mm = 127,
	.height_mm = 203,
	.bpc = 8,
	.lanes = 3,
	.format = MIPI_DSI_FMT_RGB888,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_CLOCK_NON_CONTINUOUS | MIPI_DSI_MODE_LPM,
	.init_sequence = elish_boe_init_sequence,
	.is_dual_dsi = true,
};

static const struct panel_desc elish_csot_desc = {
	.modes = elish_csot_modes,
	.num_modes = ARRAY_SIZE(elish_csot_modes),
	.dsi_info = {
		.type = "CSOT-elish",
		.channel = 0,
		.node = NULL,
	},
	.width_mm = 127,
	.height_mm = 203,
	.bpc = 8,
	.lanes = 3,
	.format = MIPI_DSI_FMT_RGB888,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_CLOCK_NON_CONTINUOUS | MIPI_DSI_MODE_LPM,
	.init_sequence = elish_csot_init_sequence,
	.is_dual_dsi = true,
};

static const struct panel_desc j606f_boe_desc = {
	.modes = j606f_boe_modes,
	.num_modes = ARRAY_SIZE(j606f_boe_modes),
	.width_mm = 143,
	.height_mm = 235,
	.bpc = 8,
	.lanes = 4,
	.format = MIPI_DSI_FMT_RGB888,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
		      MIPI_DSI_CLOCK_NON_CONTINUOUS | MIPI_DSI_MODE_LPM,
	.init_sequence = j606f_boe_init_sequence,
	.has_dcs_backlight = true,
};

/*
 * Daylight DC-1 (jagar) panel: "sharp,nt36523n,vdo,120hz", 1200x1600 single-DSI.
 *
 * Timings and the init sequence were recovered from the unstripped vendor
 * module panel-sharp-nt36523n-vdo-120hz.ko, NOT guessed:
 *   - drm_display_mode at .rodata+0xcb0 (display_mode_60hz_no_dsc)
 *   - production init table at .data+0x182834, selected by the shipped
 *     sample-id1 value 0x30 / lcm_config 0x7ff802
 *   - pre-production init table at .data+0x183fd4, 190 entries of
 *     { u32 cmd; u32 count; u8 data[64]; } with stride 0x48, as decoded from
 *     the module's push_table() helper. Markers 0xFFFC/0xFFFB are ms/us delays.
 * The 60Hz non-DSC mode is used deliberately: the 120Hz mode needs DSC, which
 * is not wired up here yet.
 */
static int sharp_nt36523n_pre_ts_init_sequence(struct panel_info *pinfo)
{
	struct mipi_dsi_device *dsi = pinfo->dsi[0];
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi };

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x05, 0xb1);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x06, 0xc0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x07, 0x69);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x08, 0x55);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0e, 0x87);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0f, 0x73);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x58, 0x60);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6e, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x88, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x89, 0x83);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x8a, 0x83);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x94, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x95, 0xd7);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x96, 0xd7);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x18, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x19, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x62, 0x36);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x63, 0xa6);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x09, 0xab);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0a, 0x88);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0b, 0x17);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0c, 0x07);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x33, 0x35);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x10, 0x0e);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x60, 0x99);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x58, 0x40);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x24);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x60, 0x96);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x61, 0x40);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x63, 0x60);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x91, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x92, 0xb7);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x93, 0x1c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x94, 0x08);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x08);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x01, 0x26);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x02, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x03, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x04, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x05, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x06, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x07, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x08, 0x0d);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x09, 0x0f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0a, 0x0c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0b, 0x0e);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0c, 0x27);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0d, 0x27);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0e, 0x27);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0f, 0x27);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x10, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x11, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x12, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x13, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x14, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x15, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x16, 0x08);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x17, 0x26);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x18, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x19, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1a, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1b, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1c, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1d, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1e, 0x0d);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1f, 0x0f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x20, 0x0c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x21, 0x0e);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x22, 0x27);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x23, 0x27);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x24, 0x27);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x25, 0x27);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x26, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x27, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x28, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x29, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x2a, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x2b, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x98, 0x80);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x2f, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x30, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x33, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x34, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x37, 0x22);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x39, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x3a, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x3b, 0xb0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x3d, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xab, 0x22);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x3f, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x43, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x47, 0x22);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x49, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4a, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4b, 0xb0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4c, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf7, 0x22);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4d, 0x21);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4e, 0x43);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4f, 0x65);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x50, 0x87);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x51, 0x78);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x52, 0x56);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x53, 0x34);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x54, 0x12);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x55, 0x03, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x56, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x58, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x59, 0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5a, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5b, 0xa0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5c, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5d, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5e, 0x00, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5f, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x96, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xa5, 0xaa);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb6, 0x05, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x05, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbb, 0x80);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbc, 0x00, 0x00, 0x03, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc2, 0xc2, 0x50, 0x50);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xdb, 0x72);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xdc, 0xb7);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x25);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x05, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0d, 0x18);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x19, 0x07);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1b, 0x11);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc2, 0xc0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc5, 0x17);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc8, 0x07);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc0, 0x0c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf4, 0xd8);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x13, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x14, 0x4d);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xdb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xdc, 0xd0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x26);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xa0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x04, 0x50);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x27);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc0, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x2a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x64, 0x96);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x67, 0x96);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x68, 0x63);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6a, 0x96);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x7f, 0x96);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x80, 0x63);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x82, 0x96);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xa2, 0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xa3, 0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xa4, 0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xa5, 0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xa6, 0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf1, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x97, 0x3c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x98, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x99, 0x95);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x9a, 0x06);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x9b, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x9c, 0x0b);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x9d, 0x0a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x9e, 0x90);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf2, 0x3c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf3, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf4, 0x95);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf5, 0x06);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf6, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf7, 0x0b);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf8, 0x0a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf9, 0x90);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x35, 0x00);
	/* no-DSC tail (init_pre_ts_60hz_no_dsc); this table carried the 3x DSC one. */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x90, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x91, 0x89, 0xa8, 0x00, 0x0c, 0xd2, 0x00, 0x02, 0x25, 0x01, 0x14, 0x00, 0x07, 0x09, 0x75, 0x08, 0x7a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x92, 0x10, 0xf0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbb, 0x13);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x11);
	mipi_dsi_msleep(&dsi_ctx, 122);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x29);

	return dsi_ctx.accum_err;
}

static int sharp_nt36523n_production_init_sequence(struct panel_info *pinfo)
{
	struct mipi_dsi_device *dsi = pinfo->dsi[0];
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi };

	/* lcm_config 0x7ff802 / sample-id1 0x30 in the shipped DC-1. */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x18, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x19, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x35, 0x00);

	/*
	 * Production MP panels leave DSC enabled in MTP.  The no-DSC tail below
	 * exists only for the vendor's pre-TS sample.  Retain it as a fallback, but
	 * never mix it into the production DSC path.
	 */
	if (!dsi->dsc) {
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x90, 0x00);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x91, 0x89, 0xa8, 0x00,
					     0x0c, 0xd2, 0x00, 0x02, 0x25, 0x01,
					     0x14, 0x00, 0x07, 0x09, 0x75, 0x08,
					     0x7a);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x92, 0x10, 0xf0);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbb, 0x13);
	}

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x11);
	mipi_dsi_msleep(&dsi_ctx, 122);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x29);

	return dsi_ctx.accum_err;
}

/*
 * Which init table fits is a property of the panel silicon, which userspace
 * cannot see -- but userspace does know whether this boot wants the
 * production path at all, because the display gate wrote
 * jagar_production_sequence=Y before probing us. Y therefore selects the
 * production table by itself: its DCS bytes are the vendor module's own
 * init_pre_ts_60hz_no_dsc, so a pre-TS panel tolerates them, while an
 * MP-family panel keeps its MTP default of DSC ON unless they run, and then
 * garbles the first uncompressed commit into the full-screen comb. A DT
 * sample-id1, when present, can still veto Y for a known non-MP panel
 * revision; absent it stays absent, which used to strand every MP panel on
 * the wrong table.
 */
static int sharp_nt36523n_init_sequence(struct panel_info *pinfo)
{
	struct device *dev = pinfo->panel.dev;
	u32 sample_id1;

	if (!sharp_nt36523n_production_sequence)
		return sharp_nt36523n_pre_ts_init_sequence(pinfo);

	if (!of_property_read_u32(pinfo->panel.dev->of_node, "sample-id1",
				  &sample_id1) &&
	    (sample_id1 & 0xf0) != 0x30) {
		dev_warn(dev,
			 "sample-id1 0x%02x is not MP-family; using pre-TS table despite jagar_production_sequence\n",
			 sample_id1);
		return sharp_nt36523n_pre_ts_init_sequence(pinfo);
	}

	return sharp_nt36523n_production_init_sequence(pinfo);
}

static const struct drm_display_mode sharp_nt36523n_modes[] = {
	{
		/* Vendor MP default: 60 Hz by long VFP at the 120 Hz line rate. */
		.clock = 261267,
		.hdisplay = 1200,
		.hsync_start = 1230,
		.hsync_end = 1250,
		.htotal = 1310,
		.vdisplay = 1600,
		.vsync_start = 3316,
		.vsync_end = 3318,
		.vtotal = 3324,
	},
	{
		/* Native vertical timing for the same production DSC family. */
		.clock = 406993,
		.hdisplay = 1200,
		.hsync_start = 1230,
		.hsync_end = 1250,
		.htotal = 1310,
		.vdisplay = 1600,
		.vsync_start = 1602,
		.vsync_end = 1604,
		.vtotal = 1610,
	},
};

static const struct panel_desc sharp_nt36523n_desc = {
	.modes = sharp_nt36523n_modes,
	.num_modes = ARRAY_SIZE(sharp_nt36523n_modes),
	.width_mm = 160,
	.height_mm = 213,
	.bpc = 8,
	.lanes = 4,
	.hs_rate = 850000000,
	.format = MIPI_DSI_FMT_RGB888,
	/*
	 * The shipped panel module writes 0xe05 to dsi->mode_flags in
	 * sharp_probe(), but the shipped MT6789 host's video-mode RX/TX setup
	 * ignores bit 9 (called MIPI_DSI_MODE_EOT_PACKET in that tree).  Setting
	 * the modern NO_EOT_PACKET spelling would therefore change the actual
	 * wire framing by setting DIS_EOT.  Reproduce the factory host behavior:
	 * sync-pulse video with EoT packets, a non-continuous clock, and low-power
	 * command transfers.
	 */
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
		      MIPI_DSI_CLOCK_NON_CONTINUOUS | MIPI_DSI_MODE_LPM,
	.init_sequence = sharp_nt36523n_init_sequence,
	.has_jagar_power_sequence = true,
};

static void nt36523_reset(struct panel_info *pinfo)
{
	gpiod_set_value_cansleep(pinfo->reset_gpio, 1);
	usleep_range(12000, 13000);
	gpiod_set_value_cansleep(pinfo->reset_gpio, 0);
	usleep_range(12000, 13000);
	gpiod_set_value_cansleep(pinfo->reset_gpio, 1);
	usleep_range(12000, 13000);
	gpiod_set_value_cansleep(pinfo->reset_gpio, 0);
	usleep_range(12000, 13000);
}

static int sharp_nt36523n_power_on(struct panel_info *pinfo)
{
	struct device *dev = pinfo->panel.dev;
	int ret;

	/*
	 * Preserve the already boot-proven pre-TS path until userspace opts into
	 * the production sequence. This lets the DC-1 reach its serial and USB
	 * consoles before the first experiment that takes ownership of VPOS/VNEG.
	 */
	if (!sharp_nt36523n_production_sequence) {
		nt36523_reset(pinfo);
		return 0;
	}

	if (pinfo->vddio) {
		ret = regulator_enable(pinfo->vddio);
		if (ret) {
			dev_err(dev, "failed to enable vddi regulator: %d\n", ret);
			return ret;
		}
	}

	usleep_range(2500, 2600);

	ret = regulator_set_voltage(pinfo->bias_supplies[0].consumer,
				    5400000, 5400000);
	if (ret)
		goto err_disable_vddi;

	ret = regulator_set_voltage(pinfo->bias_supplies[1].consumer,
				    5400000, 5400000);
	if (ret)
		goto err_disable_vddi;

	/* The shipped board sequence requires VPOS before VNEG. */
	ret = regulator_enable(pinfo->bias_supplies[0].consumer);
	if (ret)
		goto err_disable_vddi;

	ret = regulator_enable(pinfo->bias_supplies[1].consumer);
	if (ret)
		goto err_disable_vpos;

	usleep_range(5000, 5100);
	usleep_range(11000, 11055);

	/*
	 * Exact production reset from the shipped driver. GPIO85 is active-high in
	 * the factory FDT: the old generic cadence ended low and left the controller
	 * physically held in reset while every DCS command was sent.
	 */
	gpiod_set_raw_value_cansleep(pinfo->reset_gpio, 1);
	usleep_range(12, 13);
	gpiod_set_raw_value_cansleep(pinfo->reset_gpio, 0);
	usleep_range(12, 13);
	gpiod_set_raw_value_cansleep(pinfo->reset_gpio, 1);
	msleep(92);
	dev_info(dev, "production power sequence complete; reset released\n");
	pinfo->jagar_bias_enabled = true;

	return 0;

err_disable_vpos:
	regulator_disable(pinfo->bias_supplies[0].consumer);
err_disable_vddi:
	if (pinfo->vddio)
		regulator_disable(pinfo->vddio);
	dev_err(dev, "failed to enable panel bias supplies: %d\n", ret);
	return ret;
}

static void sharp_nt36523n_power_off(struct panel_info *pinfo)
{
	if (!pinfo->jagar_bias_enabled) {
		gpiod_set_value_cansleep(pinfo->reset_gpio, 1);
		return;
	}

	gpiod_set_raw_value_cansleep(pinfo->reset_gpio, 0);
	regulator_disable(pinfo->bias_supplies[1].consumer);
	regulator_disable(pinfo->bias_supplies[0].consumer);
	if (pinfo->vddio)
		regulator_disable(pinfo->vddio);
	pinfo->jagar_bias_enabled = false;
}

static int nt36523_prepare(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);
	int ret;

	if (pinfo->desc->has_jagar_power_sequence) {
		ret = sharp_nt36523n_power_on(pinfo);
		if (ret)
			return ret;
	} else {
		ret = regulator_enable(pinfo->vddio);
		if (ret) {
			dev_err(panel->dev,
				"failed to enable vddio regulator: %d\n", ret);
			return ret;
		}

		nt36523_reset(pinfo);
	}

	ret = pinfo->desc->init_sequence(pinfo);
	if (ret < 0) {
		if (pinfo->desc->has_jagar_power_sequence)
			sharp_nt36523n_power_off(pinfo);
		else
			regulator_disable(pinfo->vddio);
		dev_err(panel->dev, "failed to initialize panel: %d\n", ret);
		return ret;
	}

	return 0;
}

static int nt36523_disable(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);
	int i;

	for (i = 0; i < DSI_NUM_MIN + pinfo->desc->is_dual_dsi; i++) {
		struct mipi_dsi_multi_context dsi_ctx = { .dsi = pinfo->dsi[i]};

		mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	}

	for (i = 0; i < DSI_NUM_MIN + pinfo->desc->is_dual_dsi; i++) {
		struct mipi_dsi_multi_context dsi_ctx = { .dsi = pinfo->dsi[i]};

		mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	}

	msleep(70);

	return 0;
}

static int nt36523_unprepare(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);

	if (pinfo->desc->has_jagar_power_sequence) {
		sharp_nt36523n_power_off(pinfo);
		return 0;
	}

	gpiod_set_value_cansleep(pinfo->reset_gpio, 1);
	regulator_disable(pinfo->vddio);

	return 0;
}

static void nt36523_remove(struct mipi_dsi_device *dsi)
{
	struct panel_info *pinfo = mipi_dsi_get_drvdata(dsi);

	drm_panel_remove(&pinfo->panel);
}

static int nt36523_get_modes(struct drm_panel *panel,
			       struct drm_connector *connector)
{
	struct panel_info *pinfo = to_panel_info(panel);
	int i;

	for (i = 0; i < pinfo->desc->num_modes; i++) {
		const struct drm_display_mode *m = &pinfo->desc->modes[i];
		struct drm_display_mode *mode;

		mode = drm_mode_duplicate(connector->dev, m);
		if (!mode) {
			dev_err(panel->dev, "failed to add mode %ux%u@%u\n",
				m->hdisplay, m->vdisplay, drm_mode_vrefresh(m));
			return -ENOMEM;
		}

		mode->type = DRM_MODE_TYPE_DRIVER;
		if (i == 0)
			mode->type |= DRM_MODE_TYPE_PREFERRED;

		drm_mode_set_name(mode);
		drm_mode_probed_add(connector, mode);
	}

	connector->display_info.width_mm = pinfo->desc->width_mm;
	connector->display_info.height_mm = pinfo->desc->height_mm;
	connector->display_info.bpc = pinfo->desc->bpc;

	return pinfo->desc->num_modes;
}

static enum drm_panel_orientation nt36523_get_orientation(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);

	return pinfo->orientation;
}

static const struct drm_panel_funcs nt36523_panel_funcs = {
	.disable = nt36523_disable,
	.prepare = nt36523_prepare,
	.unprepare = nt36523_unprepare,
	.get_modes = nt36523_get_modes,
	.get_orientation = nt36523_get_orientation,
};

static int nt36523_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness = backlight_get_brightness(bl);
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_brightness_large(dsi, brightness);
	if (ret < 0)
		return ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return 0;
}

static int nt36523_bl_get_brightness(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness;
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_get_display_brightness_large(dsi, &brightness);
	if (ret < 0)
		return ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return brightness;
}

static const struct backlight_ops nt36523_bl_ops = {
	.update_status = nt36523_bl_update_status,
	.get_brightness = nt36523_bl_get_brightness,
};

static struct backlight_device *nt36523_create_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = 512,
		.max_brightness = 4095,
		.scale = BACKLIGHT_SCALE_NON_LINEAR,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &nt36523_bl_ops, &props);
}

static int nt36523_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *dsi1;
	struct mipi_dsi_host *dsi1_host;
	struct panel_info *pinfo;
	const struct mipi_dsi_device_info *info;
	const char *vddio_supply;
	enum gpiod_flags reset_flags;
	int i, ret;

	pinfo = devm_drm_panel_alloc(dev, struct panel_info, panel,
				     &nt36523_panel_funcs,
				     DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(pinfo))
		return PTR_ERR(pinfo);

	pinfo->desc = of_device_get_match_data(dev);
	if (!pinfo->desc)
		return -ENODEV;

	if (pinfo->desc->has_jagar_power_sequence &&
	    sharp_nt36523n_probe_stage < 1)
		return dev_err_probe(dev, -EPROBE_DEFER,
				     "jagar panel probe held before VDDI\n");

	vddio_supply = pinfo->desc->has_jagar_power_sequence ? "vddi" : "vddio";
	pinfo->vddio = devm_regulator_get(dev, vddio_supply);
	if (pinfo->desc->has_jagar_power_sequence &&
	    PTR_ERR_OR_ZERO(pinfo->vddio) == -EPROBE_DEFER) {
		/*
		 * The shipped FDT calls this regulator child "ldo_vrf18" while
		 * the MT6366 driver matches "vrf18". Consequently the phandle
		 * cannot resolve even though VRF18 is a boot-on 1.8 V rail and is
		 * already enabled by LK. Do not let that naming mismatch block
		 * ownership of the separately controllable panel bias rails.
		 */
		dev_warn(dev, "using the inherited boot-on VRF18 panel supply\n");
		pinfo->vddio = NULL;
	} else if (IS_ERR(pinfo->vddio)) {
		return dev_err_probe(dev, PTR_ERR(pinfo->vddio),
				     "failed to get panel I/O regulator\n");
	}

	if (pinfo->desc->has_jagar_power_sequence &&
	    sharp_nt36523n_probe_stage < 2)
		return dev_err_probe(dev, -EPROBE_DEFER,
				     "jagar panel probe held before bias regulators\n");

	if (pinfo->desc->has_jagar_power_sequence) {
		pinfo->bias_supplies[0].supply = "vpos";
		pinfo->bias_supplies[1].supply = "vneg";
		ret = devm_regulator_bulk_get(dev,
					      ARRAY_SIZE(pinfo->bias_supplies),
					      pinfo->bias_supplies);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to get panel bias supplies\n");
	}

	if (pinfo->desc->has_jagar_power_sequence &&
	    sharp_nt36523n_probe_stage < 3)
		return dev_err_probe(dev, -EPROBE_DEFER,
				     "jagar panel probe held before reset GPIO\n");

	reset_flags = GPIOD_OUT_HIGH;
	pinfo->reset_gpio = devm_gpiod_get(dev, "reset", reset_flags);
	if (IS_ERR(pinfo->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(pinfo->reset_gpio), "failed to get reset gpio\n");

	/* If the panel is dual dsi, register DSI1 */
	if (pinfo->desc->is_dual_dsi) {
		info = &pinfo->desc->dsi_info;

		dsi1 = of_graph_get_remote_node(dsi->dev.of_node, 1, -1);
		if (!dsi1) {
			dev_err(dev, "cannot get secondary DSI node.\n");
			return -ENODEV;
		}

		dsi1_host = of_find_mipi_dsi_host_by_node(dsi1);
		of_node_put(dsi1);
		if (!dsi1_host)
			return dev_err_probe(dev, -EPROBE_DEFER, "cannot get secondary DSI host\n");

		pinfo->dsi[1] = devm_mipi_dsi_device_register_full(dev, dsi1_host, info);
		if (IS_ERR(pinfo->dsi[1])) {
			dev_err(dev, "cannot get secondary DSI device\n");
			return PTR_ERR(pinfo->dsi[1]);
		}
	}

	pinfo->dsi[0] = dsi;
	mipi_dsi_set_drvdata(dsi, pinfo);

	ret = of_drm_get_panel_orientation(dev->of_node, &pinfo->orientation);
	if (ret < 0) {
		dev_err(dev, "%pOF: failed to get orientation %d\n", dev->of_node, ret);
		return ret;
	}

	pinfo->panel.prepare_prev_first = true;

	if (pinfo->desc->has_dcs_backlight) {
		pinfo->panel.backlight = nt36523_create_backlight(dsi);
		if (IS_ERR(pinfo->panel.backlight))
			return dev_err_probe(dev, PTR_ERR(pinfo->panel.backlight),
					     "Failed to create backlight\n");
	} else {
		ret = drm_panel_of_backlight(&pinfo->panel);
		if (ret)
			return dev_err_probe(dev, ret, "Failed to get backlight\n");
	}

	drm_panel_add(&pinfo->panel);

	if (pinfo->desc->has_jagar_power_sequence) {
		pinfo->dsc.dsc_version_major = 1;
		pinfo->dsc.dsc_version_minor = 2;
		pinfo->dsc.line_buf_depth = 9;
		pinfo->dsc.bits_per_component = 8;
		pinfo->dsc.bits_per_pixel = 8 << 4;
		pinfo->dsc.convert_rgb = true;
		pinfo->dsc.block_pred_enable = true;
		pinfo->dsc.pic_width = 1200;
		pinfo->dsc.pic_height = 1600;
		pinfo->dsc.slice_count = 2;
		pinfo->dsc.slice_width = 600;
		pinfo->dsc.slice_height = 20;
		/* Exact values decoded from the shipped MP panel parameters. */
		pinfo->dsc.slice_chunk_size = 600;
		pinfo->dsc.initial_xmit_delay = 512;
		pinfo->dsc.initial_dec_delay = 581;
		pinfo->dsc.first_line_bpg_offset = 13;
		pinfo->dsc.initial_offset = 6144;
		pinfo->dsc.final_offset = 4336;
		pinfo->dsc.initial_scale_value = 32;
		pinfo->dsc.scale_increment_interval = 492;
		pinfo->dsc.scale_decrement_interval = 8;
		pinfo->dsc.nfl_bpg_offset = 1402;
		pinfo->dsc.slice_bpg_offset = 1172;
		pinfo->dsc.rc_model_size = 8192;
		pinfo->dsc.rc_edge_factor = 6;
		pinfo->dsc.rc_quant_incr_limit0 = 11;
		pinfo->dsc.rc_quant_incr_limit1 = 11;
		pinfo->dsc.rc_tgt_offset_high = 3;
		pinfo->dsc.rc_tgt_offset_low = 3;
		pinfo->dsc.flatness_min_qp = 3;
		pinfo->dsc.flatness_max_qp = 12;
	}

	for (i = 0; i < DSI_NUM_MIN + pinfo->desc->is_dual_dsi; i++) {
		pinfo->dsi[i]->lanes = pinfo->desc->lanes;
		pinfo->dsi[i]->hs_rate = pinfo->desc->has_jagar_power_sequence
					 ? sharp_nt36523n_hs_rate
					 : pinfo->desc->hs_rate;
		pinfo->dsi[i]->format = pinfo->desc->format;
		pinfo->dsi[i]->mode_flags = pinfo->desc->mode_flags;
		if (pinfo->desc->has_jagar_power_sequence)
			pinfo->dsi[i]->dsc = &pinfo->dsc;

		ret = devm_mipi_dsi_attach(dev, pinfo->dsi[i]);
		if (ret < 0)
			return dev_err_probe(dev, ret, "cannot attach to DSI%d host.\n", i);
	}

	return 0;
}

static const struct of_device_id nt36523_of_match[] = {
	{
		.compatible = "sharp,nt36523n,vdo,120hz",
		.data = &sharp_nt36523n_desc,
	},
	{
		.compatible = "lenovo,j606f-boe-nt36523w",
		.data = &j606f_boe_desc,
	},
	{
		.compatible = "xiaomi,elish-boe-nt36523",
		.data = &elish_boe_desc,
	},
	{
		.compatible = "xiaomi,elish-csot-nt36523",
		.data = &elish_csot_desc,
	},
	{},
};
MODULE_DEVICE_TABLE(of, nt36523_of_match);

static struct mipi_dsi_driver nt36523_driver = {
	.probe = nt36523_probe,
	.remove = nt36523_remove,
	.driver = {
		.name = "panel-novatek-nt36523",
		.of_match_table = nt36523_of_match,
	},
};
module_mipi_dsi_driver(nt36523_driver);

MODULE_AUTHOR("Jianhua Lu <lujianhua000@gmail.com>");
MODULE_DESCRIPTION("DRM driver for Novatek NT36523 based MIPI DSI panels");
MODULE_LICENSE("GPL");
