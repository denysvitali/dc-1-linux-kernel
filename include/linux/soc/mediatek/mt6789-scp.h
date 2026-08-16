/* SPDX-License-Identifier: GPL-2.0 */
/*
 * MediaTek MT6789/MT8781 tinysys SCP (sensor co-processor) register map.
 *
 * The SCP is a RISC-V "tinysys" core. Its firmware (scp_a.img) is loaded by
 * the bootloader, so there is no remoteproc loader here; this driver only
 * maps the SCP's control/mailbox registers so the AP can exchange IPIs with
 * it over the tinysys mbox framework (mtk-mbox).
 *
 * Register bases are taken from the running stock FDT (scp@10500000 node):
 *   scp_sram_base  0x10500000  (0xc0000)  TCM / firmware image
 *   scp_cfgreg     0x10724000             shared control
 *   scp_clkreg     0x10721000             clock control
 *   scp_cfgreg_core0 0x10730000 (0x3000)  core0 control/status
 *   scp_cfgreg_core1 0x10740000           core1 control/status
 *   scp_bus_tracker  0x10752000           bus tracker
 *   scp_l1creg      0x10760000 (0x40000)  L1 cache control
 *   scp_cfgreg_sec   0x107a5000           secure control
 */
#ifndef __MTK6789_SCP_H__
#define __MTK6789_SCP_H__

#define SCP_MBOX_TOTAL 5

struct scp_regs {
	void __iomem *cfg;		/* scp_cfgreg */
	void __iomem *cfg_core0;	/* scp_cfgreg_core0 */
	void __iomem *cfg_core1;	/* scp_cfgreg_core1 */
	void __iomem *clkctrl;		/* scp_clkreg */
	void __iomem *l1cctrl;		/* scp_l1creg */
	void __iomem *bus_tracker;	/* scp_bus_tracker */
	void __iomem *cfg_sec;		/* scp_cfgreg_sec */
};

/* Offsets relative to scp_regs.cfg (scp_cfgreg) */
#define SCP_SEMAPHORE			0x0018
#define SCP_3WAY_SEMAPHORE		0x001c

/* Offsets relative to scp_regs.cfg_core0 (scp_cfgreg_core0) */
#define SCP_CORE_SW_RSTN_CLR		0x0000
#define SCP_CORE_SW_RSTN_SET		0x0004
#define SCP_CORE_DBG_CTRL		0x0010
#define SCP_CORE_WDT_IRQ		0x0030
#define SCP_CORE_WDT_CFG		0x0034
#define SCP_CORE_STATUS			0x0070
#define SCP_CORE_MON_PC			0x0080
#define SCP_CORE_MON_LR			0x0084
#define SCP_CORE_MON_SP			0x0088
#define SCP_CORE_MON_PC_LATCH		0x00d0
#define SCP_CORE_MON_LR_LATCH		0x00d4
#define SCP_CORE_MON_SP_LATCH		0x00d8

/* SCP_CORE_STATUS bits */
#define SCP_CORE_GATED			BIT(0)
#define SCP_CORE_HALT			BIT(1)

/* Offsets relative to scp_regs.clkctrl (scp_clkreg) */
#define SCP_A_SLEEP_DEBUG_REG		0x0028
#define SCP_CLK_CTRL_L1_SRAM_PD		0x002c
#define SCP_CLK_HIGH_CORE_CG		0x005c
#define SCP_CPU0_SRAM_PD		0x0080
#define SCP_CPU1_SRAM_PD		0x0084
#define SCP_POWER_STATUS		0x0124
#define SCP_SLP_PWR_CTRL		0x0128

struct mtk_mbox_device;
extern struct mtk_mbox_device scp_mboxdev;

#endif /* __MTK6789_SCP_H__ */
