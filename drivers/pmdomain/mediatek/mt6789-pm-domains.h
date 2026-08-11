/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MediaTek MT6789 power domains
 *
 * Register offsets, status bits and bus-protection sequences are taken from
 * MediaTek's downstream MT6789 SCPSYS driver.  Keep the GPU domains in their
 * firmware state while registering the provider: unused domains must only be
 * enabled through a real genpd consumer.
 */

#ifndef __PMDOMAIN_MEDIATEK_MT6789_PM_DOMAINS_H
#define __PMDOMAIN_MEDIATEK_MT6789_PM_DOMAINS_H

#include <dt-bindings/power/mediatek,mt6789-power.h>

#include "mtk-pm-domains.h"

#define MT6789_TOP_AXI_PROT_EN_SET		0x2a0
#define MT6789_TOP_AXI_PROT_EN_CLR		0x2a4
#define MT6789_TOP_AXI_PROT_EN_STA1		0x228
#define MT6789_TOP_AXI_PROT_EN_1_SET		0x2a8
#define MT6789_TOP_AXI_PROT_EN_1_CLR		0x2ac
#define MT6789_TOP_AXI_PROT_EN_1_STA1		0x258
#define MT6789_TOP_AXI_PROT_EN_2_SET		0x714
#define MT6789_TOP_AXI_PROT_EN_2_CLR		0x718
#define MT6789_TOP_AXI_PROT_EN_2_STA1		0x724
#define MT6789_TOP_AXI_PROT_EN_MM_SET		0x2d4
#define MT6789_TOP_AXI_PROT_EN_MM_CLR		0x2d8
#define MT6789_TOP_AXI_PROT_EN_MM_STA1		0x2ec
#define MT6789_TOP_AXI_PROT_EN_MM_2_SET		0xdcc
#define MT6789_TOP_AXI_PROT_EN_MM_2_CLR		0xdd0
#define MT6789_TOP_AXI_PROT_EN_MM_2_STA1	0xdd8
#define MT6789_TOP_AXI_PROT_EN_VDNR_SET		0xb84
#define MT6789_TOP_AXI_PROT_EN_VDNR_CLR		0xb88
#define MT6789_TOP_AXI_PROT_EN_VDNR_STA1	0xb90

#define MT6789_PROT_MD			BIT(7)
#define MT6789_PROT_VDNR_MD		(BIT(0) | BIT(10) | BIT(15))
#define MT6789_PROT_CONN		(BIT(13) | BIT(18))
#define MT6789_PROT_CONN_2ND		BIT(14)
#define MT6789_PROT_1_CONN		BIT(10)
#define MT6789_PROT_1_MFG1		BIT(21)
#define MT6789_PROT_2_MFG1		(BIT(5) | BIT(6))
#define MT6789_PROT_MFG1		(BIT(21) | BIT(22))
#define MT6789_PROT_2_MFG1_2ND		BIT(7)
#define MT6789_PROT_MM_2_ISP		BIT(8)
#define MT6789_PROT_MM_2_ISP_2ND	BIT(9)
#define MT6789_PROT_MM_IPE		BIT(16)
#define MT6789_PROT_MM_IPE_2ND		BIT(17)
#define MT6789_PROT_MM_VDEC		BIT(24)
#define MT6789_PROT_MM_VDEC_2ND		BIT(25)
#define MT6789_PROT_MM_VENC		BIT(26)
#define MT6789_PROT_MM_VENC_2ND		BIT(27)
#define MT6789_PROT_MM_DISP		(BIT(10) | BIT(12))
#define MT6789_PROT_DISP		(BIT(6) | BIT(23))
#define MT6789_PROT_2_AUDIO		BIT(4)
#define MT6789_PROT_MM_CAM		(BIT(0) | BIT(2))
#define MT6789_PROT_MM_CAM_2ND		(BIT(1) | BIT(3))

#define MT6789_DOMAIN(_name, _bit, _ctl) \
	.name = (_name), \
	.sta_mask = BIT(_bit), \
	.ctl_offs = (_ctl), \
	.pwr_sta_offs = 0x16c, \
	.pwr_sta2nd_offs = 0x170, \
	.pwr_on_delay_us = 100, \
	.sram_pdn_bits = BIT(8), \
	.sram_pdn_ack_bits = BIT(12), \
	.caps = MTK_SCPD_KEEP_DEFAULT_OFF | MTK_SCPD_PWR_OFF_ON_FIRST

static const struct scpsys_domain_data scpsys_domain_data_mt6789[] = {
	[MT6789_POWER_DOMAIN_MD] = {
		.name = "md",
		.sta_mask = BIT(0),
		.ctl_offs = 0x300,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.ext_buck_iso_offs = 0x398,
		.ext_buck_iso_mask = GENMASK(1, 0),
		.bp_cfg = {
			BUS_PROT_WR_IGN(INFRA, MT6789_PROT_MD,
					MT6789_TOP_AXI_PROT_EN_SET,
					MT6789_TOP_AXI_PROT_EN_CLR,
					MT6789_TOP_AXI_PROT_EN_STA1),
			BUS_PROT_WR_IGN(INFRA, MT6789_PROT_VDNR_MD,
					MT6789_TOP_AXI_PROT_EN_VDNR_SET,
					MT6789_TOP_AXI_PROT_EN_VDNR_CLR,
					MT6789_TOP_AXI_PROT_EN_VDNR_STA1),
		},
		.caps = MTK_SCPD_KEEP_DEFAULT_OFF | MTK_SCPD_EXT_BUCK_ISO |
			MTK_SCPD_MODEM_PWRSEQ,
	},
	[MT6789_POWER_DOMAIN_CONN] = {
		.name = "conn",
		.sta_mask = BIT(1),
		.ctl_offs = 0x304,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.pwr_on_delay_us = 100,
		.bp_cfg = {
			BUS_PROT_WR_IGN(INFRA, MT6789_PROT_CONN,
					MT6789_TOP_AXI_PROT_EN_SET,
					MT6789_TOP_AXI_PROT_EN_CLR,
					MT6789_TOP_AXI_PROT_EN_STA1),
			BUS_PROT_WR_IGN(INFRA, MT6789_PROT_CONN_2ND,
					MT6789_TOP_AXI_PROT_EN_SET,
					MT6789_TOP_AXI_PROT_EN_CLR,
					MT6789_TOP_AXI_PROT_EN_STA1),
			BUS_PROT_WR_IGN(INFRA, MT6789_PROT_1_CONN,
					MT6789_TOP_AXI_PROT_EN_1_SET,
					MT6789_TOP_AXI_PROT_EN_1_CLR,
					MT6789_TOP_AXI_PROT_EN_1_STA1),
		},
		.caps = MTK_SCPD_KEEP_DEFAULT_OFF | MTK_SCPD_PWR_OFF_ON_FIRST,
	},
	[MT6789_POWER_DOMAIN_MFG0] = {
		.name = "mfg0",
		.sta_mask = BIT(2),
		.ctl_offs = 0x308,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.pwr_on_delay_us = 100,
		.sram_pdn_bits = BIT(8),
		.sram_pdn_ack_bits = BIT(12),
		.caps = MTK_SCPD_KEEP_DEFAULT_OFF | MTK_SCPD_DOMAIN_SUPPLY |
			MTK_SCPD_PWR_OFF_ON_FIRST,
	},
	[MT6789_POWER_DOMAIN_MFG1] = {
		.name = "mfg1",
		.sta_mask = BIT(3),
		.ctl_offs = 0x30c,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.pwr_on_delay_us = 100,
		.sram_pdn_bits = BIT(8),
		.sram_pdn_ack_bits = BIT(12),
		.bp_cfg = {
			BUS_PROT_WR(INFRA,
				    MT6789_PROT_1_MFG1,
				    MT6789_TOP_AXI_PROT_EN_1_SET,
				    MT6789_TOP_AXI_PROT_EN_1_CLR,
				    MT6789_TOP_AXI_PROT_EN_1_STA1),
			BUS_PROT_WR(INFRA,
				    MT6789_PROT_2_MFG1,
				    MT6789_TOP_AXI_PROT_EN_2_SET,
				    MT6789_TOP_AXI_PROT_EN_2_CLR,
				    MT6789_TOP_AXI_PROT_EN_2_STA1),
			BUS_PROT_WR(INFRA,
				    MT6789_PROT_MFG1,
				    MT6789_TOP_AXI_PROT_EN_SET,
				    MT6789_TOP_AXI_PROT_EN_CLR,
				    MT6789_TOP_AXI_PROT_EN_STA1),
			BUS_PROT_WR(INFRA,
				    MT6789_PROT_2_MFG1_2ND,
				    MT6789_TOP_AXI_PROT_EN_2_SET,
				    MT6789_TOP_AXI_PROT_EN_2_CLR,
				    MT6789_TOP_AXI_PROT_EN_2_STA1),
		},
		.caps = MTK_SCPD_KEEP_DEFAULT_OFF | MTK_SCPD_DOMAIN_SUPPLY |
			MTK_SCPD_PWR_OFF_ON_FIRST,
	},
	[MT6789_POWER_DOMAIN_MFG2] = {
		.name = "mfg2",
		.sta_mask = BIT(4),
		.ctl_offs = 0x310,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.pwr_on_delay_us = 100,
		.sram_pdn_bits = BIT(8),
		.sram_pdn_ack_bits = BIT(12),
		.caps = MTK_SCPD_KEEP_DEFAULT_OFF | MTK_SCPD_PWR_OFF_ON_FIRST,
	},
	[MT6789_POWER_DOMAIN_MFG3] = {
		.name = "mfg3",
		.sta_mask = BIT(5),
		.ctl_offs = 0x314,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.pwr_on_delay_us = 100,
		.sram_pdn_bits = BIT(8),
		.sram_pdn_ack_bits = BIT(12),
		.caps = MTK_SCPD_KEEP_DEFAULT_OFF | MTK_SCPD_PWR_OFF_ON_FIRST,
	},
	[MT6789_POWER_DOMAIN_ISP] = {
		MT6789_DOMAIN("isp", 13, 0x334),
		.bp_cfg = {
			BUS_PROT_WR_IGN(INFRA, MT6789_PROT_MM_2_ISP,
					MT6789_TOP_AXI_PROT_EN_MM_2_SET,
					MT6789_TOP_AXI_PROT_EN_MM_2_CLR,
					MT6789_TOP_AXI_PROT_EN_MM_2_STA1),
			BUS_PROT_WR_IGN(INFRA, MT6789_PROT_MM_2_ISP_2ND,
					MT6789_TOP_AXI_PROT_EN_MM_2_SET,
					MT6789_TOP_AXI_PROT_EN_MM_2_CLR,
					MT6789_TOP_AXI_PROT_EN_MM_2_STA1),
		},
	},
	[MT6789_POWER_DOMAIN_IPE] = {
		MT6789_DOMAIN("ipe", 15, 0x33c),
		.bp_cfg = {
			BUS_PROT_WR_IGN(INFRA, MT6789_PROT_MM_IPE,
					MT6789_TOP_AXI_PROT_EN_MM_SET,
					MT6789_TOP_AXI_PROT_EN_MM_CLR,
					MT6789_TOP_AXI_PROT_EN_MM_STA1),
			BUS_PROT_WR_IGN(INFRA, MT6789_PROT_MM_IPE_2ND,
					MT6789_TOP_AXI_PROT_EN_MM_SET,
					MT6789_TOP_AXI_PROT_EN_MM_CLR,
					MT6789_TOP_AXI_PROT_EN_MM_STA1),
		},
	},
	[MT6789_POWER_DOMAIN_VDEC] = {
		MT6789_DOMAIN("vdec", 16, 0x340),
		.bp_cfg = {
			BUS_PROT_WR_IGN(INFRA, MT6789_PROT_MM_VDEC,
					MT6789_TOP_AXI_PROT_EN_MM_SET,
					MT6789_TOP_AXI_PROT_EN_MM_CLR,
					MT6789_TOP_AXI_PROT_EN_MM_STA1),
			BUS_PROT_WR_IGN(INFRA, MT6789_PROT_MM_VDEC_2ND,
					MT6789_TOP_AXI_PROT_EN_MM_SET,
					MT6789_TOP_AXI_PROT_EN_MM_CLR,
					MT6789_TOP_AXI_PROT_EN_MM_STA1),
		},
	},
	[MT6789_POWER_DOMAIN_VENC] = {
		MT6789_DOMAIN("venc", 18, 0x348),
		.bp_cfg = {
			BUS_PROT_WR_IGN(INFRA, MT6789_PROT_MM_VENC,
					MT6789_TOP_AXI_PROT_EN_MM_SET,
					MT6789_TOP_AXI_PROT_EN_MM_CLR,
					MT6789_TOP_AXI_PROT_EN_MM_STA1),
			BUS_PROT_WR_IGN(INFRA, MT6789_PROT_MM_VENC_2ND,
					MT6789_TOP_AXI_PROT_EN_MM_SET,
					MT6789_TOP_AXI_PROT_EN_MM_CLR,
					MT6789_TOP_AXI_PROT_EN_MM_STA1),
		},
	},
	[MT6789_POWER_DOMAIN_DISP] = {
		MT6789_DOMAIN("disp", 21, 0x354),
		.bp_cfg = {
			BUS_PROT_WR_IGN(INFRA, MT6789_PROT_MM_DISP,
					MT6789_TOP_AXI_PROT_EN_MM_SET,
					MT6789_TOP_AXI_PROT_EN_MM_CLR,
					MT6789_TOP_AXI_PROT_EN_MM_STA1),
			BUS_PROT_WR_IGN(INFRA, MT6789_PROT_DISP,
					MT6789_TOP_AXI_PROT_EN_SET,
					MT6789_TOP_AXI_PROT_EN_CLR,
					MT6789_TOP_AXI_PROT_EN_STA1),
		},
	},
	[MT6789_POWER_DOMAIN_AUDIO] = {
		MT6789_DOMAIN("audio", 22, 0x358),
		.bp_cfg = {
			BUS_PROT_WR_IGN(INFRA, MT6789_PROT_2_AUDIO,
					MT6789_TOP_AXI_PROT_EN_2_SET,
					MT6789_TOP_AXI_PROT_EN_2_CLR,
					MT6789_TOP_AXI_PROT_EN_2_STA1),
		},
	},
	[MT6789_POWER_DOMAIN_CAM] = {
		MT6789_DOMAIN("cam", 23, 0x35c),
		.bp_cfg = {
			BUS_PROT_WR_IGN(INFRA, MT6789_PROT_MM_CAM,
					MT6789_TOP_AXI_PROT_EN_MM_SET,
					MT6789_TOP_AXI_PROT_EN_MM_CLR,
					MT6789_TOP_AXI_PROT_EN_MM_STA1),
			BUS_PROT_WR_IGN(INFRA, MT6789_PROT_MM_CAM_2ND,
					MT6789_TOP_AXI_PROT_EN_MM_SET,
					MT6789_TOP_AXI_PROT_EN_MM_CLR,
					MT6789_TOP_AXI_PROT_EN_MM_STA1),
		},
	},
	[MT6789_POWER_DOMAIN_CAM_RAWA] = {
		MT6789_DOMAIN("cam_rawa", 24, 0x360),
	},
	[MT6789_POWER_DOMAIN_CAM_RAWB] = {
		MT6789_DOMAIN("cam_rawb", 25, 0x364),
	},
};

static const struct scpsys_soc_data mt6789_scpsys_data = {
	.domains_data = scpsys_domain_data_mt6789,
	.num_domains = ARRAY_SIZE(scpsys_domain_data_mt6789),
};

#endif /* __PMDOMAIN_MEDIATEK_MT6789_PM_DOMAINS_H */
