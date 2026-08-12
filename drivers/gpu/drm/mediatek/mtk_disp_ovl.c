// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015 MediaTek Inc.
 */

#include <drm/drm_blend.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/soc/mediatek/mtk-cmdq.h>

#include <asm/barrier.h>

#include "mtk_crtc.h"
#include "mtk_ddp_comp.h"
#include "mtk_disp_drv.h"
#include "mtk_drm_drv.h"

#define DISP_REG_OVL_INTEN			0x0004
#define OVL_FME_CPL_INT					BIT(1)
#define OVL_FME_UND_INT					BIT(2)
#define OVL_SWRST_DONE_INT				BIT(3)
#define OVL_RDMA_EOF_ABNORMAL_INT			GENMASK(8, 5)
#define OVL_RDMA_SMI_UNDERFLOW_INT			GENMASK(12, 9)
#define OVL_ABNORMAL_SOF_INT				BIT(13)
#define OVL_HANDOFF_ABNORMAL_INT			(OVL_FME_UND_INT | \
							 OVL_RDMA_EOF_ABNORMAL_INT | \
							 OVL_RDMA_SMI_UNDERFLOW_INT | \
							 OVL_ABNORMAL_SOF_INT)
#define OVL_INT_STATUS_MASK				GENMASK(14, 0)
#define DISP_REG_OVL_INTSTA			0x0008
#define DISP_REG_OVL_EN				0x000c
#define OVL_EN_BYPASS_SHADOW			BIT(22)
#define DISP_REG_OVL_RST			0x0014
#define DISP_REG_OVL_ROI_SIZE			0x0020
#define DISP_REG_OVL_DATAPATH_CON		0x0024
#define OVL_LAYER_SMI_ID_EN				BIT(0)
#define OVL_BGCLR_SEL_IN				BIT(2)
#define OVL_LAYER_AFBC_EN(n)				BIT(4+n)
#define DISP_REG_OVL_ROI_BGCLR			0x0028
#define DISP_REG_OVL_SRC_CON			0x002c
#define DISP_REG_OVL_CON(n)			(0x0030 + 0x20 * (n))
#define DISP_REG_OVL_SRC_SIZE(n)		(0x0038 + 0x20 * (n))
#define DISP_REG_OVL_OFFSET(n)			(0x003c + 0x20 * (n))
#define DISP_REG_OVL_PITCH_MSB(n)		(0x0040 + 0x20 * (n))
#define OVL_PITCH_MSB_2ND_SUBBUF			BIT(16)
#define DISP_REG_OVL_PITCH(n)			(0x0044 + 0x20 * (n))
#define OVL_CONST_BLEND					BIT(28)
#define DISP_REG_OVL_RDMA_CTRL(n)		(0x00c0 + 0x20 * (n))
#define DISP_REG_OVL_RDMA_GMC(n)		(0x00c8 + 0x20 * (n))
#define DISP_REG_OVL_ADDR_MT2701		0x0040
#define DISP_REG_OVL_CLRFMT_EXT			0x02d0
#define DISP_REG_OVL_FLOW_CTRL_DBG		0x0240
#define DISP_REG_OVL_RDMA_DBG(n)		(0x024c + 0x4 * (n))
#define OVL_FLOW_FSM_MASK			GENMASK(9, 0)
#define OVL_FLOW_OUT_IDLE			BIT(15)
#define OVL_FLOW_H_W_RST			0x100
#define OVL_RDMA_DBG_SMI_BUSY			BIT(30)
#define OVL_RDMA_DBG_SMI_GREQ			BIT(31)
#define OVL_RDMA_DBG_LAYER_GREQ			BIT(3)
#define OVL_RDMA_DBG_OUT_VALID			BIT(29)
#define OVL_RDMA_DBG_WARM_RST_MASK		GENMASK(2, 0)
#define OVL_RDMA_DBG_WARM_RST_IDLE		1
#define OVL_CON_CLRFMT_BIT_DEPTH_MASK(n)		(GENMASK(1, 0) << (4 * (n)))
#define OVL_CON_CLRFMT_BIT_DEPTH(depth, n)		((depth) << (4 * (n)))
#define OVL_CON_CLRFMT_8_BIT				(0)
#define OVL_CON_CLRFMT_10_BIT				(1)
#define DISP_REG_OVL_ADDR_MT8173		0x0f40
#define DISP_REG_OVL_ADDR(ovl, n)		((ovl)->data->addr + 0x20 * (n))
#define DISP_REG_OVL_HDR_ADDR(ovl, n)		((ovl)->data->addr + 0x20 * (n) + 0x04)
#define DISP_REG_OVL_HDR_PITCH(ovl, n)		((ovl)->data->addr + 0x20 * (n) + 0x08)

#define GMC_THRESHOLD_BITS	16
#define GMC_THRESHOLD_HIGH	((1 << GMC_THRESHOLD_BITS) / 4)
#define GMC_THRESHOLD_LOW	((1 << GMC_THRESHOLD_BITS) / 8)

#define OVL_CON_CLRFMT_MAN	BIT(23)
#define OVL_CON_BYTE_SWAP	BIT(24)

/* OVL_CON_RGB_SWAP works only if OVL_CON_CLRFMT_MAN is enabled */
#define OVL_CON_RGB_SWAP	BIT(25)

#define OVL_CON_CLRFMT_RGB	(1 << 12)
#define OVL_CON_CLRFMT_ARGB8888	(2 << 12)
#define OVL_CON_CLRFMT_RGBA8888	(3 << 12)
#define OVL_CON_CLRFMT_ABGR8888	(OVL_CON_CLRFMT_ARGB8888 | OVL_CON_BYTE_SWAP)
#define OVL_CON_CLRFMT_BGRA8888	(OVL_CON_CLRFMT_RGBA8888 | OVL_CON_BYTE_SWAP)
#define OVL_CON_CLRFMT_UYVY	(4 << 12)
#define OVL_CON_CLRFMT_YUYV	(5 << 12)
#define OVL_CON_MTX_YUV_TO_RGB	(6 << 16)
#define OVL_CON_CLRFMT_PARGB8888 ((3 << 12) | OVL_CON_CLRFMT_MAN)
#define OVL_CON_CLRFMT_PABGR8888 (OVL_CON_CLRFMT_PARGB8888 | OVL_CON_RGB_SWAP)
#define OVL_CON_CLRFMT_PBGRA8888 (OVL_CON_CLRFMT_PARGB8888 | OVL_CON_BYTE_SWAP)
#define OVL_CON_CLRFMT_PRGBA8888 (OVL_CON_CLRFMT_PABGR8888 | OVL_CON_BYTE_SWAP)
#define OVL_CON_CLRFMT_RGB565(ovl)	((ovl)->data->fmt_rgb565_is_0 ? \
					0 : OVL_CON_CLRFMT_RGB)
#define OVL_CON_CLRFMT_RGB888(ovl)	((ovl)->data->fmt_rgb565_is_0 ? \
					OVL_CON_CLRFMT_RGB : 0)
#define	OVL_CON_AEN		BIT(8)
#define	OVL_CON_ALPHA		0xff
#define	OVL_CON_VIRT_FLIP	BIT(9)
#define	OVL_CON_HORZ_FLIP	BIT(10)

#define OVL_COLOR_ALPHA		GENMASK(31, 24)

static inline bool is_10bit_rgb(u32 fmt)
{
	switch (fmt) {
	case DRM_FORMAT_XRGB2101010:
	case DRM_FORMAT_ARGB2101010:
	case DRM_FORMAT_RGBX1010102:
	case DRM_FORMAT_RGBA1010102:
	case DRM_FORMAT_XBGR2101010:
	case DRM_FORMAT_ABGR2101010:
	case DRM_FORMAT_BGRX1010102:
	case DRM_FORMAT_BGRA1010102:
		return true;
	}
	return false;
}

static const u32 mt8173_formats[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_BGRX8888,
	DRM_FORMAT_BGRA8888,
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_RGB888,
	DRM_FORMAT_BGR888,
	DRM_FORMAT_RGB565,
	DRM_FORMAT_UYVY,
	DRM_FORMAT_YUYV,
};

static const u32 mt8195_formats[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_XRGB2101010,
	DRM_FORMAT_ARGB2101010,
	DRM_FORMAT_BGRX8888,
	DRM_FORMAT_BGRA8888,
	DRM_FORMAT_BGRX1010102,
	DRM_FORMAT_BGRA1010102,
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_XBGR2101010,
	DRM_FORMAT_ABGR2101010,
	DRM_FORMAT_RGBX8888,
	DRM_FORMAT_RGBA8888,
	DRM_FORMAT_RGBX1010102,
	DRM_FORMAT_RGBA1010102,
	DRM_FORMAT_RGB888,
	DRM_FORMAT_BGR888,
	DRM_FORMAT_RGB565,
	DRM_FORMAT_UYVY,
	DRM_FORMAT_YUYV,
};

struct mtk_disp_ovl_data {
	unsigned int addr;
	unsigned int gmc_bits;
	unsigned int layer_nr;
	bool fmt_rgb565_is_0;
	bool smi_id_en;
	bool supports_afbc;
	const u32 blend_modes;
	const u32 *formats;
	size_t num_formats;
	bool supports_clrfmt_ext;
	bool bypass_shadow;
	bool skip_config_reset;
	bool reset_on_stop;
	bool defer_irq_enable;
};

/*
 * struct mtk_disp_ovl - DISP_OVL driver structure
 * @crtc: associated crtc to report vblank events to
 * @data: platform data
 */
struct mtk_disp_ovl {
	struct device			*dev;
	int				irq;
	struct drm_crtc			*crtc;
	struct clk			*clk;
	void __iomem			*regs;
	struct cmdq_client_reg		cmdq_reg;
	const struct mtk_disp_ovl_data	*data;
	void				(*vblank_cb)(void *data);
	void				*vblank_cb_data;
	atomic_t			fme_seq;
	atomic_t			handoff_irq_status;
	wait_queue_head_t		fme_wait_queue;
	bool				irq_enabled;
};

static bool mtk_ovl_flow_operational(u32 flow)
{
	u32 fsm = flow & OVL_FLOW_FSM_MASK;

	/* The non-reset states are one-hot from idle through engine-active. */
	return fsm && !(fsm & (fsm - 1)) && fsm <= 0x20;
}

static bool mtk_ovl_flow_terminal(u32 flow)
{
	u32 fsm = flow & OVL_FLOW_FSM_MASK;

	return (fsm == 0x1 || fsm == 0x2) &&
	       (flow & OVL_FLOW_OUT_IDLE) &&
	       !(flow & (BIT(12) | BIT(21) | BIT(28) | BIT(29) |
			 BIT(30) | BIT(31)));
}

static bool mtk_ovl_flow_stopped(u32 flow)
{
	return (flow & OVL_FLOW_FSM_MASK) == 0x1 &&
	       (flow & OVL_FLOW_OUT_IDLE) &&
	       !(flow & (BIT(12) | BIT(21) | BIT(28) | BIT(29) |
			 BIT(30) | BIT(31)));
}

static irqreturn_t mtk_disp_ovl_irq_handler(int irq, void *dev_id)
{
	struct mtk_disp_ovl *priv = dev_id;
	u32 pending, status;

	status = readl(priv->regs + DISP_REG_OVL_INTSTA);
	pending = status & OVL_INT_STATUS_MASK;
	if (!pending)
		return IRQ_NONE;

	/* OVL_INTSTA is write-zero-to-clear; preserve newly arriving events. */
	writel(~pending, priv->regs + DISP_REG_OVL_INTSTA);
	atomic_or(pending, &priv->handoff_irq_status);
	if (pending & OVL_ABNORMAL_SOF_INT)
		dev_err_ratelimited(priv->dev,
				    "OVL abnormal SOF: status=%#x flow=%#x\n",
				    status,
				    readl(priv->regs + DISP_REG_OVL_FLOW_CTRL_DBG));

	if (!(pending & OVL_FME_CPL_INT))
		return IRQ_HANDLED;

	atomic_inc(&priv->fme_seq);
	wake_up(&priv->fme_wait_queue);

	/*
	 * The bootloader can leave the OVL scanning out and its interrupt
	 * asserted, so this fires long before the DRM device is bound and a
	 * vblank callback exists. We did acknowledge the source above, so
	 * report the interrupt as handled: returning IRQ_NONE here lets the
	 * spurious-interrupt detector disable the line permanently, and the
	 * CRTC then never gets a vblank once it finally comes up.
	 */
	if (!priv->vblank_cb)
		return IRQ_HANDLED;

	priv->vblank_cb(priv->vblank_cb_data);

	return IRQ_HANDLED;
}

void mtk_ovl_register_vblank_cb(struct device *dev,
				void (*vblank_cb)(void *),
				void *vblank_cb_data)
{
	struct mtk_disp_ovl *ovl = dev_get_drvdata(dev);

	ovl->vblank_cb = vblank_cb;
	ovl->vblank_cb_data = vblank_cb_data;
}

void mtk_ovl_unregister_vblank_cb(struct device *dev)
{
	struct mtk_disp_ovl *ovl = dev_get_drvdata(dev);

	ovl->vblank_cb = NULL;
	ovl->vblank_cb_data = NULL;
}

void mtk_ovl_enable_vblank(struct device *dev)
{
	struct mtk_disp_ovl *ovl = dev_get_drvdata(dev);

	writel(0x0, ovl->regs + DISP_REG_OVL_INTSTA);
	writel_relaxed(OVL_FME_CPL_INT, ovl->regs + DISP_REG_OVL_INTEN);
}

void mtk_ovl_disable_vblank(struct device *dev)
{
	struct mtk_disp_ovl *ovl = dev_get_drvdata(dev);

	writel_relaxed(0x0, ovl->regs + DISP_REG_OVL_INTEN);
}

u32 mtk_ovl_get_blend_modes(struct device *dev)
{
	struct mtk_disp_ovl *ovl = dev_get_drvdata(dev);

	return ovl->data->blend_modes;
}

const u32 *mtk_ovl_get_formats(struct device *dev)
{
	struct mtk_disp_ovl *ovl = dev_get_drvdata(dev);

	return ovl->data->formats;
}

size_t mtk_ovl_get_num_formats(struct device *dev)
{
	struct mtk_disp_ovl *ovl = dev_get_drvdata(dev);

	return ovl->data->num_formats;
}

bool mtk_ovl_is_afbc_supported(struct device *dev)
{
	struct mtk_disp_ovl *ovl = dev_get_drvdata(dev);

	return ovl->data->supports_afbc;
}

int mtk_ovl_clk_enable(struct device *dev)
{
	struct mtk_disp_ovl *ovl = dev_get_drvdata(dev);

	return clk_prepare_enable(ovl->clk);
}

void mtk_ovl_clk_disable(struct device *dev)
{
	struct mtk_disp_ovl *ovl = dev_get_drvdata(dev);

	if (ovl->data->defer_irq_enable && ovl->irq_enabled) {
		writel(0, ovl->regs + DISP_REG_OVL_INTEN);
		readl(ovl->regs + DISP_REG_OVL_INTEN);
		disable_irq(ovl->irq);
		ovl->irq_enabled = false;
		writel(0, ovl->regs + DISP_REG_OVL_INTSTA);
		readl(ovl->regs + DISP_REG_OVL_INTSTA);
	}
	clk_disable_unprepare(ovl->clk);
}

void mtk_ovl_start(struct device *dev)
{
	struct mtk_disp_ovl *ovl = dev_get_drvdata(dev);
	u32 en = BIT(0);

	if (ovl->data->smi_id_en) {
		unsigned int reg;

		reg = readl(ovl->regs + DISP_REG_OVL_DATAPATH_CON);
		reg = reg | OVL_LAYER_SMI_ID_EN;
		writel_relaxed(reg, ovl->regs + DISP_REG_OVL_DATAPATH_CON);
	}
	if (ovl->data->bypass_shadow)
		en |= OVL_EN_BYPASS_SHADOW;
	writel_relaxed(en, ovl->regs + DISP_REG_OVL_EN);
}

static u32 mtk_ovl_handoff_dma_busy(struct mtk_disp_ovl *ovl)
{
	u32 flow = readl(ovl->regs + DISP_REG_OVL_FLOW_CTRL_DBG);
	u32 busy = 0;
	unsigned int i;

	if (!mtk_ovl_flow_terminal(flow))
		busy |= BIT(31);
	if (!(flow & OVL_FLOW_OUT_IDLE))
		busy |= BIT(30);
	for (i = 0; i < ovl->data->layer_nr; i++) {
		u32 rdma_dbg = readl(ovl->regs + DISP_REG_OVL_RDMA_DBG(i));

		/* MT6789 reports L0..L3 idle in FLOW bits 19..16. */
		if (!(flow & BIT(19 - i)))
			busy |= BIT(i);
		if ((rdma_dbg & OVL_RDMA_DBG_WARM_RST_MASK) !=
		    OVL_RDMA_DBG_WARM_RST_IDLE ||
		    (rdma_dbg & (OVL_RDMA_DBG_LAYER_GREQ |
				 OVL_RDMA_DBG_OUT_VALID |
				 OVL_RDMA_DBG_SMI_BUSY |
				 OVL_RDMA_DBG_SMI_GREQ)))
			busy |= BIT(i + 4);
	}

	return busy;
}

static u32 mtk_ovl_handoff_layer_busy(struct mtk_disp_ovl *ovl)
{
	u32 flow = readl(ovl->regs + DISP_REG_OVL_FLOW_CTRL_DBG);
	u32 busy = 0;
	unsigned int i;

	for (i = 0; i < ovl->data->layer_nr; i++) {
		u32 rdma_dbg = readl(ovl->regs + DISP_REG_OVL_RDMA_DBG(i));

		if (!(flow & BIT(19 - i)))
			busy |= BIT(i);
		if ((rdma_dbg & OVL_RDMA_DBG_WARM_RST_MASK) !=
		    OVL_RDMA_DBG_WARM_RST_IDLE ||
		    (rdma_dbg & (OVL_RDMA_DBG_LAYER_GREQ |
				 OVL_RDMA_DBG_OUT_VALID |
				 OVL_RDMA_DBG_SMI_BUSY |
				 OVL_RDMA_DBG_SMI_GREQ)))
			busy |= BIT(i + 4);
	}

	return busy;
}

static u32 mtk_ovl_handoff_stopped_busy(struct mtk_disp_ovl *ovl)
{
	u32 flow = readl(ovl->regs + DISP_REG_OVL_FLOW_CTRL_DBG);
	u32 busy = mtk_ovl_handoff_layer_busy(ovl);

	if (!mtk_ovl_flow_stopped(flow))
		busy |= BIT(31);

	return busy;
}

static int mtk_ovl_handoff_wait_stopped_stable(struct mtk_disp_ovl *ovl)
{
	u32 busy;
	int ret;

	ret = read_poll_timeout(mtk_ovl_handoff_stopped_busy, busy, !busy,
				10, 100000, false, ovl);
	if (ret)
		return ret;

	/* EN and reset FSM transitions can lag their MMIO readback. */
	usleep_range(100, 200);
	return mtk_ovl_handoff_stopped_busy(ovl) ? -EBUSY : 0;
}

static void mtk_ovl_handoff_irq_enable(struct mtk_disp_ovl *ovl)
{
	if (!ovl->data->defer_irq_enable || ovl->irq_enabled)
		return;

	/* INTEN and stale status were cleared while the OVL clock was live. */
	enable_irq(ovl->irq);
	ovl->irq_enabled = true;
}

static void mtk_ovl_handoff_log_state(struct mtk_disp_ovl *ovl,
				      const char *stage)
{
	dev_info(ovl->dev,
		 "OVL handoff %s: t=%llu en=%#x rst=%#x intsta=%#x src=%#x flow=%#x rdma=%#x/%#x/%#x/%#x dbg=%#x/%#x/%#x/%#x\n",
		 stage, (unsigned long long)ktime_get_ns(),
		 readl(ovl->regs + DISP_REG_OVL_EN),
		 readl(ovl->regs + DISP_REG_OVL_RST),
		 readl(ovl->regs + DISP_REG_OVL_INTSTA),
		 readl(ovl->regs + DISP_REG_OVL_SRC_CON),
		 readl(ovl->regs + DISP_REG_OVL_FLOW_CTRL_DBG),
		 readl(ovl->regs + DISP_REG_OVL_RDMA_CTRL(0)),
		 readl(ovl->regs + DISP_REG_OVL_RDMA_CTRL(1)),
		 readl(ovl->regs + DISP_REG_OVL_RDMA_CTRL(2)),
		 readl(ovl->regs + DISP_REG_OVL_RDMA_CTRL(3)),
		 readl(ovl->regs + DISP_REG_OVL_RDMA_DBG(0)),
		 readl(ovl->regs + DISP_REG_OVL_RDMA_DBG(1)),
		 readl(ovl->regs + DISP_REG_OVL_RDMA_DBG(2)),
		 readl(ovl->regs + DISP_REG_OVL_RDMA_DBG(3)));
}

static int mtk_ovl_stop_checked(struct mtk_disp_ovl *ovl)
{
	u32 en = ovl->data->bypass_shadow ? OVL_EN_BYPASS_SHADOW : 0;
	u32 busy, val;
	unsigned int i;
	int ret;

	writel(0, ovl->regs + DISP_REG_OVL_INTEN);
	val = readl(ovl->regs + DISP_REG_OVL_INTEN);
	if (val)
		return -EIO;
	synchronize_irq(ovl->irq);

	/*
	 * A clean DSI EOF has removed further SOF triggers, but the inherited
	 * source must still be proven idle before EN changes.  Clearing EN while
	 * LK layer DMA is active is what traps MT6789 in h_w_rst.
	 */
	ret = read_poll_timeout(mtk_ovl_handoff_dma_busy, busy, !busy,
				10, 100000, false, ovl);
	if (ret)
		return ret;
	mtk_ovl_handoff_log_state(ovl, "pre-source-disable");

	/* Remove every inherited LK source while the mutex and DSI are stopped. */
	writel(0, ovl->regs + DISP_REG_OVL_SRC_CON);
	if (readl(ovl->regs + DISP_REG_OVL_SRC_CON))
		return -EIO;
	for (i = 0; i < ovl->data->layer_nr; i++) {
		writel(0, ovl->regs + DISP_REG_OVL_RDMA_CTRL(i));
		if (readl(ovl->regs + DISP_REG_OVL_RDMA_CTRL(i)))
			return -EIO;
	}
	ret = read_poll_timeout(mtk_ovl_handoff_layer_busy, busy, !busy,
				10, 100000, false, ovl);
	if (ret)
		return ret;
	ret = read_poll_timeout(mtk_ovl_handoff_dma_busy, busy, !busy,
				10, 100000, false, ovl);
	if (ret)
		return ret;
	mtk_ovl_handoff_log_state(ovl, "sources-disabled-idle");

	writel(en, ovl->regs + DISP_REG_OVL_EN);
	ret = readl_poll_timeout(ovl->regs + DISP_REG_OVL_EN, val,
				 val == en, 1, 1000);
	if (ret)
		return ret;
	mtk_ovl_handoff_log_state(ovl, "engine-disabled");
	ret = mtk_ovl_handoff_wait_stopped_stable(ovl);
	if (ret)
		return ret;
	mtk_ovl_handoff_log_state(ovl, "engine-disabled-idle");
	if (mtk_ovl_handoff_stopped_busy(ovl))
		return -EBUSY;

	writel(0, ovl->regs + DISP_REG_OVL_INTSTA);
	val = readl(ovl->regs + DISP_REG_OVL_INTSTA);
	if (val & OVL_SWRST_DONE_INT)
		return -EIO;
	writel(1, ovl->regs + DISP_REG_OVL_RST);
	ret = readl_poll_timeout(ovl->regs + DISP_REG_OVL_RST, val,
				 val & 1, 1, 1000);
	if (ret)
		return ret;
	udelay(1);
	writel(0, ovl->regs + DISP_REG_OVL_RST);
	ret = readl_poll_timeout(ovl->regs + DISP_REG_OVL_RST, val,
				 !(val & 1), 1, 1000);
	if (ret)
		return ret;
	ret = readl_poll_timeout(ovl->regs + DISP_REG_OVL_INTSTA, val,
				 val & OVL_SWRST_DONE_INT, 1, 10000);
	if (ret)
		return ret;
	writel(0, ovl->regs + DISP_REG_OVL_INTSTA);
	val = readl(ovl->regs + DISP_REG_OVL_INTSTA);
	if (val & OVL_INT_STATUS_MASK)
		return -EIO;

	ret = mtk_ovl_handoff_wait_stopped_stable(ovl);
	if (!ret)
		mtk_ovl_handoff_log_state(ovl, "reset-complete");

	return ret;
}

int mtk_ovl_handoff_frame_arm(struct device *dev, u32 *fme_seq)
{
	struct mtk_disp_ovl *ovl = dev_get_drvdata(dev);

	if (!ovl->data->reset_on_stop || !fme_seq)
		return -EINVAL;

	writel(0, ovl->regs + DISP_REG_OVL_INTEN);
	if (readl(ovl->regs + DISP_REG_OVL_INTEN))
		return -EIO;
	if (ovl->irq_enabled)
		synchronize_irq(ovl->irq);
	writel(0, ovl->regs + DISP_REG_OVL_INTSTA);
	readl(ovl->regs + DISP_REG_OVL_INTSTA);
	atomic_set(&ovl->handoff_irq_status, 0);

	*fme_seq = atomic_read(&ovl->fme_seq);
	mtk_ovl_handoff_irq_enable(ovl);
	writel(OVL_FME_CPL_INT, ovl->regs + DISP_REG_OVL_INTEN);
	if (readl(ovl->regs + DISP_REG_OVL_INTEN) != OVL_FME_CPL_INT)
		return -EIO;

	return 0;
}

int mtk_ovl_handoff_frame_wait(struct device *dev, u32 fme_seq)
{
	struct mtk_disp_ovl *ovl = dev_get_drvdata(dev);
	u32 irq_status;
	long ret;

	ret = wait_event_timeout(ovl->fme_wait_queue,
				 atomic_read(&ovl->fme_seq) != fme_seq,
				 msecs_to_jiffies(100));
	if (!ret) {
		dev_err(dev,
			"OVL inherited frame timeout: seq=%u now=%u flow=%#x\n",
			fme_seq, atomic_read(&ovl->fme_seq),
			readl(ovl->regs + DISP_REG_OVL_FLOW_CTRL_DBG));
		return -ETIMEDOUT;
	}
	irq_status = atomic_xchg(&ovl->handoff_irq_status, 0);
	if (irq_status & OVL_HANDOFF_ABNORMAL_INT) {
		dev_err(dev,
			"OVL inherited frame abnormal: status=%#x flow=%#x\n",
			irq_status,
			readl(ovl->regs + DISP_REG_OVL_FLOW_CTRL_DBG));
		return -EIO;
	}

	return 0;
}

void mtk_ovl_handoff_frame_cancel(struct device *dev)
{
	struct mtk_disp_ovl *ovl = dev_get_drvdata(dev);
	u32 inten;

	writel(0, ovl->regs + DISP_REG_OVL_INTEN);
	inten = readl(ovl->regs + DISP_REG_OVL_INTEN);
	if (ovl->irq_enabled)
		synchronize_irq(ovl->irq);
	if (inten && ovl->irq_enabled) {
		disable_irq(ovl->irq);
		ovl->irq_enabled = false;
		dev_err(dev, "OVL IRQ mask failed during cancel: inten=%#x\n",
			inten);
	}
}

int mtk_ovl_handoff_stop(struct device *dev)
{
	struct mtk_disp_ovl *ovl = dev_get_drvdata(dev);
	int ret;

	if (!ovl->data->reset_on_stop)
		return -EOPNOTSUPP;

	ret = mtk_ovl_stop_checked(ovl);
	if (ret) {
		dev_err(dev,
			"OVL checked stop failed: %d en=%#x rst=%#x intsta=%#x flow=%#x dma=%#x rdma0=%#x rdma1=%#x rdma2=%#x rdma3=%#x\n",
			ret, readl(ovl->regs + DISP_REG_OVL_EN),
			readl(ovl->regs + DISP_REG_OVL_RST),
			readl(ovl->regs + DISP_REG_OVL_INTSTA),
			readl(ovl->regs + DISP_REG_OVL_FLOW_CTRL_DBG),
			mtk_ovl_handoff_dma_busy(ovl),
			readl(ovl->regs + DISP_REG_OVL_RDMA_DBG(0)),
			readl(ovl->regs + DISP_REG_OVL_RDMA_DBG(1)),
			readl(ovl->regs + DISP_REG_OVL_RDMA_DBG(2)),
			readl(ovl->regs + DISP_REG_OVL_RDMA_DBG(3)));
		return ret;
	}

	mtk_ovl_handoff_irq_enable(ovl);

	return 0;
}

void mtk_ovl_stop(struct device *dev)
{
	struct mtk_disp_ovl *ovl = dev_get_drvdata(dev);
	u32 en = ovl->data->bypass_shadow ? OVL_EN_BYPASS_SHADOW : 0;

	if (ovl->data->reset_on_stop) {
		/*
		 * Match the MT6789 vendor stop sequence while the caller has the
		 * display mutex quiesced: mask interrupts, stop, reset, clear latched
		 * status, then release reset.  Keep BYPASS_SHADOW set while EN itself
		 * is clear.
		 */
		if (mtk_ovl_stop_checked(ovl))
			dev_err(dev, "OVL stop/reset readback failed\n");
	} else {
		writel_relaxed(en, ovl->regs + DISP_REG_OVL_EN);
	}
	if (ovl->data->smi_id_en) {
		unsigned int reg;

		reg = readl(ovl->regs + DISP_REG_OVL_DATAPATH_CON);
		reg = reg & ~OVL_LAYER_SMI_ID_EN;
		writel_relaxed(reg, ovl->regs + DISP_REG_OVL_DATAPATH_CON);
	}
}

static void mtk_ovl_set_afbc(struct mtk_disp_ovl *ovl, struct cmdq_pkt *cmdq_pkt,
			     int idx, bool enabled)
{
	mtk_ddp_write_mask(cmdq_pkt, enabled ? OVL_LAYER_AFBC_EN(idx) : 0,
			   &ovl->cmdq_reg, ovl->regs,
			   DISP_REG_OVL_DATAPATH_CON, OVL_LAYER_AFBC_EN(idx));
}

static void mtk_ovl_set_bit_depth(struct device *dev, int idx, u32 format,
				  struct cmdq_pkt *cmdq_pkt)
{
	struct mtk_disp_ovl *ovl = dev_get_drvdata(dev);
	unsigned int bit_depth = OVL_CON_CLRFMT_8_BIT;

	if (!ovl->data->supports_clrfmt_ext)
		return;

	if (is_10bit_rgb(format))
		bit_depth = OVL_CON_CLRFMT_10_BIT;

	mtk_ddp_write_mask(cmdq_pkt, OVL_CON_CLRFMT_BIT_DEPTH(bit_depth, idx),
			   &ovl->cmdq_reg, ovl->regs, DISP_REG_OVL_CLRFMT_EXT,
			   OVL_CON_CLRFMT_BIT_DEPTH_MASK(idx));
}

void mtk_ovl_config(struct device *dev, unsigned int w,
		    unsigned int h, unsigned int vrefresh,
		    unsigned int bpc, struct cmdq_pkt *cmdq_pkt)
{
	struct mtk_disp_ovl *ovl = dev_get_drvdata(dev);

	if (w != 0 && h != 0)
		mtk_ddp_write_relaxed(cmdq_pkt, h << 16 | w, &ovl->cmdq_reg, ovl->regs,
				      DISP_REG_OVL_ROI_SIZE);

	/*
	 * The background color must be opaque black (ARGB),
	 * otherwise the alpha blending will have no effect
	 */
	mtk_ddp_write_relaxed(cmdq_pkt, OVL_COLOR_ALPHA, &ovl->cmdq_reg,
			      ovl->regs, DISP_REG_OVL_ROI_BGCLR);

	if (!ovl->data->skip_config_reset) {
		mtk_ddp_write(cmdq_pkt, 0x1, &ovl->cmdq_reg, ovl->regs,
			      DISP_REG_OVL_RST);
		mtk_ddp_write(cmdq_pkt, 0x0, &ovl->cmdq_reg, ovl->regs,
			      DISP_REG_OVL_RST);
	}
}

unsigned int mtk_ovl_layer_nr(struct device *dev)
{
	struct mtk_disp_ovl *ovl = dev_get_drvdata(dev);

	return ovl->data->layer_nr;
}

unsigned int mtk_ovl_supported_rotations(struct device *dev)
{
	return DRM_MODE_ROTATE_0 | DRM_MODE_ROTATE_180 |
	       DRM_MODE_REFLECT_X | DRM_MODE_REFLECT_Y;
}

int mtk_ovl_layer_check(struct device *dev, unsigned int idx,
			struct mtk_plane_state *mtk_state)
{
	struct drm_plane_state *state = &mtk_state->base;

	/* check if any unsupported rotation is set */
	if (state->rotation & ~mtk_ovl_supported_rotations(dev))
		return -EINVAL;

	/*
	 * TODO: Rotating/reflecting YUV buffers is not supported at this time.
	 *	 Only RGB[AX] variants are supported.
	 *	 Since DRM_MODE_ROTATE_0 means "no rotation", we should not
	 *	 reject layers with this property.
	 */
	if (state->fb->format->is_yuv && (state->rotation & ~DRM_MODE_ROTATE_0))
		return -EINVAL;

	return 0;
}

void mtk_ovl_layer_on(struct device *dev, unsigned int idx,
		      struct cmdq_pkt *cmdq_pkt)
{
	unsigned int gmc_thrshd_l;
	unsigned int gmc_thrshd_h;
	unsigned int gmc_value;
	struct mtk_disp_ovl *ovl = dev_get_drvdata(dev);

	mtk_ddp_write(cmdq_pkt, 0x1, &ovl->cmdq_reg, ovl->regs,
		      DISP_REG_OVL_RDMA_CTRL(idx));
	gmc_thrshd_l = GMC_THRESHOLD_LOW >>
		      (GMC_THRESHOLD_BITS - ovl->data->gmc_bits);
	gmc_thrshd_h = GMC_THRESHOLD_HIGH >>
		      (GMC_THRESHOLD_BITS - ovl->data->gmc_bits);
	if (ovl->data->gmc_bits == 10)
		gmc_value = gmc_thrshd_h | gmc_thrshd_h << 16;
	else
		gmc_value = gmc_thrshd_l | gmc_thrshd_l << 8 |
			    gmc_thrshd_h << 16 | gmc_thrshd_h << 24;
	mtk_ddp_write(cmdq_pkt, gmc_value,
		      &ovl->cmdq_reg, ovl->regs, DISP_REG_OVL_RDMA_GMC(idx));
	mtk_ddp_write_mask(cmdq_pkt, BIT(idx), &ovl->cmdq_reg, ovl->regs,
			   DISP_REG_OVL_SRC_CON, BIT(idx));
}

void mtk_ovl_layer_off(struct device *dev, unsigned int idx,
		       struct cmdq_pkt *cmdq_pkt)
{
	struct mtk_disp_ovl *ovl = dev_get_drvdata(dev);

	mtk_ddp_write_mask(cmdq_pkt, 0, &ovl->cmdq_reg, ovl->regs,
			   DISP_REG_OVL_SRC_CON, BIT(idx));
	mtk_ddp_write(cmdq_pkt, 0, &ovl->cmdq_reg, ovl->regs,
		      DISP_REG_OVL_RDMA_CTRL(idx));
}

static unsigned int mtk_ovl_fmt_convert(struct mtk_disp_ovl *ovl,
					struct mtk_plane_state *state)
{
	unsigned int fmt = state->pending.format;
	unsigned int blend_mode = DRM_MODE_BLEND_COVERAGE;

	/*
	 * For the platforms where OVL_CON_CLRFMT_MAN is defined in the hardware data sheet
	 * and supports premultiplied color formats, such as OVL_CON_CLRFMT_PARGB8888.
	 *
	 * Check blend_modes in the driver data to see if premultiplied mode is supported.
	 * If not, use coverage mode instead to set it to the supported color formats.
	 *
	 * Current DRM assumption is that alpha is default premultiplied, so the bitmask of
	 * blend_modes must include BIT(DRM_MODE_BLEND_PREMULTI). Otherwise, mtk_plane_init()
	 * will get an error return from drm_plane_create_blend_mode_property() and
	 * state->base.pixel_blend_mode should not be used.
	 */
	if (ovl->data->blend_modes & BIT(DRM_MODE_BLEND_PREMULTI))
		blend_mode = state->base.pixel_blend_mode;

	switch (fmt) {
	default:
	case DRM_FORMAT_RGB565:
		return OVL_CON_CLRFMT_RGB565(ovl);
	case DRM_FORMAT_BGR565:
		return OVL_CON_CLRFMT_RGB565(ovl) | OVL_CON_BYTE_SWAP;
	case DRM_FORMAT_RGB888:
		return OVL_CON_CLRFMT_RGB888(ovl);
	case DRM_FORMAT_BGR888:
		return OVL_CON_CLRFMT_RGB888(ovl) | OVL_CON_BYTE_SWAP;
	case DRM_FORMAT_RGBX8888:
	case DRM_FORMAT_RGBA8888:
	case DRM_FORMAT_RGBX1010102:
	case DRM_FORMAT_RGBA1010102:
		return blend_mode == DRM_MODE_BLEND_COVERAGE ?
		       OVL_CON_CLRFMT_RGBA8888 :
		       OVL_CON_CLRFMT_PRGBA8888;
	case DRM_FORMAT_BGRX8888:
	case DRM_FORMAT_BGRA8888:
	case DRM_FORMAT_BGRX1010102:
	case DRM_FORMAT_BGRA1010102:
		return blend_mode == DRM_MODE_BLEND_COVERAGE ?
		       OVL_CON_CLRFMT_BGRA8888 :
		       OVL_CON_CLRFMT_PBGRA8888;
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_ARGB8888:
	case DRM_FORMAT_XRGB2101010:
	case DRM_FORMAT_ARGB2101010:
		return blend_mode == DRM_MODE_BLEND_COVERAGE ?
		       OVL_CON_CLRFMT_ARGB8888 :
		       OVL_CON_CLRFMT_PARGB8888;
	case DRM_FORMAT_XBGR8888:
	case DRM_FORMAT_ABGR8888:
	case DRM_FORMAT_XBGR2101010:
	case DRM_FORMAT_ABGR2101010:
		return blend_mode == DRM_MODE_BLEND_COVERAGE ?
		       OVL_CON_CLRFMT_ABGR8888 :
		       OVL_CON_CLRFMT_PABGR8888;
	case DRM_FORMAT_UYVY:
		return OVL_CON_CLRFMT_UYVY | OVL_CON_MTX_YUV_TO_RGB;
	case DRM_FORMAT_YUYV:
		return OVL_CON_CLRFMT_YUYV | OVL_CON_MTX_YUV_TO_RGB;
	}
}

static void mtk_ovl_layer_values(struct mtk_disp_ovl *ovl,
				 struct mtk_plane_state *state,
				 u32 *con, u32 *pitch, u32 *addr)
{
	struct mtk_plane_pending_state *pending = &state->pending;
	unsigned int blend_mode = state->base.pixel_blend_mode;
	unsigned int rotation = pending->rotation;
	unsigned int ignore_pixel_alpha = 0;

	*con = mtk_ovl_fmt_convert(ovl, state);
	*addr = pending->addr;
	if (state->base.fb) {
		*con |= state->base.alpha & OVL_CON_ALPHA;
		if (blend_mode || state->base.fb->format->has_alpha)
			*con |= OVL_CON_AEN;
		if (blend_mode == DRM_MODE_BLEND_PIXEL_NONE ||
		    !state->base.fb->format->has_alpha)
			ignore_pixel_alpha = OVL_CONST_BLEND;
	}

	if (rotation & DRM_MODE_ROTATE_180)
		rotation ^= DRM_MODE_REFLECT_X | DRM_MODE_REFLECT_Y;
	if (rotation & DRM_MODE_REFLECT_Y) {
		*con |= OVL_CON_VIRT_FLIP;
		*addr += (pending->height - 1) * pending->pitch;
	}
	if (rotation & DRM_MODE_REFLECT_X) {
		*con |= OVL_CON_HORZ_FLIP;
		*addr += pending->pitch - 1;
	}

	*pitch = (pending->pitch & GENMASK(15, 0)) | ignore_pixel_alpha;
}

static void mtk_ovl_afbc_layer_config(struct mtk_disp_ovl *ovl,
				      unsigned int idx,
				      struct mtk_plane_pending_state *pending,
				      struct cmdq_pkt *cmdq_pkt)
{
	unsigned int pitch_msb = pending->pitch >> 16;
	unsigned int hdr_pitch = pending->hdr_pitch;
	unsigned int hdr_addr = pending->hdr_addr;

	if (pending->modifier != DRM_FORMAT_MOD_LINEAR) {
		mtk_ddp_write_relaxed(cmdq_pkt, hdr_addr, &ovl->cmdq_reg, ovl->regs,
				      DISP_REG_OVL_HDR_ADDR(ovl, idx));
		mtk_ddp_write_relaxed(cmdq_pkt,
				      OVL_PITCH_MSB_2ND_SUBBUF | pitch_msb,
				      &ovl->cmdq_reg, ovl->regs, DISP_REG_OVL_PITCH_MSB(idx));
		mtk_ddp_write_relaxed(cmdq_pkt, hdr_pitch, &ovl->cmdq_reg, ovl->regs,
				      DISP_REG_OVL_HDR_PITCH(ovl, idx));
	} else {
		mtk_ddp_write_relaxed(cmdq_pkt, pitch_msb,
				      &ovl->cmdq_reg, ovl->regs, DISP_REG_OVL_PITCH_MSB(idx));
	}
}

void mtk_ovl_layer_config(struct device *dev, unsigned int idx,
			  struct mtk_plane_state *state,
			  struct cmdq_pkt *cmdq_pkt)
{
	struct mtk_disp_ovl *ovl = dev_get_drvdata(dev);
	struct mtk_plane_pending_state *pending = &state->pending;
	unsigned int addr;
	unsigned int pitch;
	unsigned int fmt = pending->format;
	unsigned int offset = (pending->y << 16) | pending->x;
	unsigned int src_size = (pending->height << 16) | pending->width;
	unsigned int con;

	if (!pending->enable) {
		mtk_ovl_layer_off(dev, idx, cmdq_pkt);
		return;
	}

	mtk_ovl_layer_values(ovl, state, &con, &pitch, &addr);

	if (ovl->data->supports_afbc)
		mtk_ovl_set_afbc(ovl, cmdq_pkt, idx,
				 pending->modifier != DRM_FORMAT_MOD_LINEAR);

	mtk_ddp_write_relaxed(cmdq_pkt, con, &ovl->cmdq_reg, ovl->regs,
			      DISP_REG_OVL_CON(idx));
	mtk_ddp_write_relaxed(cmdq_pkt, pitch,
			      &ovl->cmdq_reg, ovl->regs, DISP_REG_OVL_PITCH(idx));
	mtk_ddp_write_relaxed(cmdq_pkt, src_size, &ovl->cmdq_reg, ovl->regs,
			      DISP_REG_OVL_SRC_SIZE(idx));
	mtk_ddp_write_relaxed(cmdq_pkt, offset, &ovl->cmdq_reg, ovl->regs,
			      DISP_REG_OVL_OFFSET(idx));
	mtk_ddp_write_relaxed(cmdq_pkt, addr, &ovl->cmdq_reg, ovl->regs,
			      DISP_REG_OVL_ADDR(ovl, idx));

	if (ovl->data->supports_afbc)
		mtk_ovl_afbc_layer_config(ovl, idx, pending, cmdq_pkt);

	mtk_ovl_set_bit_depth(dev, idx, fmt, cmdq_pkt);
	mtk_ovl_layer_on(dev, idx, cmdq_pkt);
}

int mtk_ovl_handoff_layer_validate(struct device *dev, unsigned int idx,
				   struct mtk_plane_state *state)
{
	struct mtk_disp_ovl *ovl = dev_get_drvdata(dev);
	struct mtk_plane_pending_state *pending = &state->pending;
	u32 src_con;
	u32 rdma;
	u32 con, pitch, addr, size, offset;

	if (idx >= ovl->data->layer_nr)
		return -EINVAL;
	src_con = readl(ovl->regs + DISP_REG_OVL_SRC_CON);
	rdma = readl(ovl->regs + DISP_REG_OVL_RDMA_CTRL(idx));
	if (!pending->enable) {
		if (!(src_con & BIT(idx)) && !rdma)
			return 0;
		goto mismatch;
	}

	mtk_ovl_layer_values(ovl, state, &con, &pitch, &addr);
	size = (pending->height << 16) | pending->width;
	offset = (pending->y << 16) | pending->x;
	if ((src_con & BIT(idx)) && rdma == 1 &&
	    readl(ovl->regs + DISP_REG_OVL_CON(idx)) == con &&
	    readl(ovl->regs + DISP_REG_OVL_ADDR(ovl, idx)) == addr &&
	    readl(ovl->regs + DISP_REG_OVL_PITCH(idx)) == pitch &&
	    readl(ovl->regs + DISP_REG_OVL_SRC_SIZE(idx)) == size &&
	    readl(ovl->regs + DISP_REG_OVL_OFFSET(idx)) == offset)
		return 0;

mismatch:
	dev_err(dev,
		"OVL L%u readback mismatch: enable=%u src=%#x con=%#x addr=%#x pitch=%#x size=%#x offset=%#x rdma=%#x\n",
		idx, pending->enable, src_con,
		readl(ovl->regs + DISP_REG_OVL_CON(idx)),
		readl(ovl->regs + DISP_REG_OVL_ADDR(ovl, idx)),
		readl(ovl->regs + DISP_REG_OVL_PITCH(idx)),
		readl(ovl->regs + DISP_REG_OVL_SRC_SIZE(idx)),
		readl(ovl->regs + DISP_REG_OVL_OFFSET(idx)), rdma);
	return -EIO;
}

int mtk_ovl_handoff_prepare(struct device *dev, unsigned int width,
			    unsigned int height, u32 *fme_seq)
{
	struct mtk_disp_ovl *ovl = dev_get_drvdata(dev);
	u32 en = BIT(0) | OVL_EN_BYPASS_SHADOW;
	u32 inten = OVL_FME_CPL_INT | OVL_HANDOFF_ABNORMAL_INT;
	u32 roi = height << 16 | width;
	u32 src_con, val;

	if (!ovl->data->bypass_shadow)
		return -EOPNOTSUPP;

	src_con = readl(ovl->regs + DISP_REG_OVL_SRC_CON);
	if (readl(ovl->regs + DISP_REG_OVL_EN) != en ||
	    readl(ovl->regs + DISP_REG_OVL_ROI_SIZE) != roi ||
	    !(src_con & BIT(0))) {
		dev_err(dev,
			"OVL handoff invalid: en=%#x expected=%#x roi=%#x expected=%#x src=%#x flow=%#x\n",
			readl(ovl->regs + DISP_REG_OVL_EN), en,
			readl(ovl->regs + DISP_REG_OVL_ROI_SIZE), roi,
			src_con,
			readl(ovl->regs + DISP_REG_OVL_FLOW_CTRL_DBG));
		return -EIO;
	}

	/* Drain all preceding relaxed OVL writes before the cross-block trigger. */
	readl(ovl->regs + DISP_REG_OVL_RDMA_CTRL(ovl->data->layer_nr - 1));
	mb(); /* Complete OVL writes before enabling the display mutex. */
	writel(0, ovl->regs + DISP_REG_OVL_INTEN);
	if (readl(ovl->regs + DISP_REG_OVL_INTEN))
		return -EIO;
	mtk_ovl_handoff_irq_enable(ovl);
	synchronize_irq(ovl->irq);
	writel(0, ovl->regs + DISP_REG_OVL_INTSTA);
	val = readl(ovl->regs + DISP_REG_OVL_INTSTA);
	if (val & OVL_INT_STATUS_MASK)
		return -EIO;
	atomic_set(&ovl->handoff_irq_status, 0);
	*fme_seq = atomic_read(&ovl->fme_seq);
	writel(inten, ovl->regs + DISP_REG_OVL_INTEN);
	if (readl(ovl->regs + DISP_REG_OVL_INTEN) != inten)
		return -EIO;

	return 0;
}

int mtk_ovl_handoff_wait_for_fme(struct device *dev, u32 fme_seq)
{
	struct mtk_disp_ovl *ovl = dev_get_drvdata(dev);
	u32 flow, inten, irq_status;
	int flow_ret;
	long ret;

	ret = wait_event_timeout(ovl->fme_wait_queue,
				 atomic_read(&ovl->fme_seq) != fme_seq,
				 msecs_to_jiffies(100));
	writel(0, ovl->regs + DISP_REG_OVL_INTEN);
	inten = readl(ovl->regs + DISP_REG_OVL_INTEN);
	synchronize_irq(ovl->irq);
	irq_status = atomic_xchg(&ovl->handoff_irq_status, 0);
	flow_ret = readl_poll_timeout(ovl->regs + DISP_REG_OVL_FLOW_CTRL_DBG,
				      flow,
				      mtk_ovl_flow_operational(flow),
				      10, 20000);
	irq_status |= readl(ovl->regs + DISP_REG_OVL_INTSTA) &
		      OVL_HANDOFF_ABNORMAL_INT;
	if (inten || !ret || flow_ret ||
	    (irq_status & OVL_HANDOFF_ABNORMAL_INT)) {
		dev_err(dev,
			"OVL first-frame failure: seq=%u now=%u status=%#x flow=%#x en=%#x rst=%#x inten=%#x intsta=%#x src=%#x roi=%#x bg=%#x l0con=%#x l0addr=%#x l0pitch=%#x l0size=%#x l0offset=%#x l0rdma=%#x\n",
			fme_seq, atomic_read(&ovl->fme_seq), irq_status,
			flow,
			readl(ovl->regs + DISP_REG_OVL_EN),
			readl(ovl->regs + DISP_REG_OVL_RST),
			readl(ovl->regs + DISP_REG_OVL_INTEN),
			readl(ovl->regs + DISP_REG_OVL_INTSTA),
			readl(ovl->regs + DISP_REG_OVL_SRC_CON),
			readl(ovl->regs + DISP_REG_OVL_ROI_SIZE),
			readl(ovl->regs + DISP_REG_OVL_ROI_BGCLR),
			readl(ovl->regs + DISP_REG_OVL_CON(0)),
			readl(ovl->regs + DISP_REG_OVL_ADDR(ovl, 0)),
			readl(ovl->regs + DISP_REG_OVL_PITCH(0)),
			readl(ovl->regs + DISP_REG_OVL_SRC_SIZE(0)),
			readl(ovl->regs + DISP_REG_OVL_OFFSET(0)),
			readl(ovl->regs + DISP_REG_OVL_RDMA_CTRL(0)));
		if (inten)
			return -EIO;
		if (!ret)
			return -ETIMEDOUT;
		return flow_ret ?: -EIO;
	}

	writel(OVL_FME_CPL_INT, ovl->regs + DISP_REG_OVL_INTEN);
	if (readl(ovl->regs + DISP_REG_OVL_INTEN) != OVL_FME_CPL_INT)
		return -EIO;

	dev_info(dev, "OVL first frame complete: seq=%u status=%#x flow=%#x\n",
		 atomic_read(&ovl->fme_seq), irq_status, flow);
	return 0;
}

void mtk_ovl_bgclr_in_on(struct device *dev)
{
	struct mtk_disp_ovl *ovl = dev_get_drvdata(dev);
	unsigned int reg;

	reg = readl(ovl->regs + DISP_REG_OVL_DATAPATH_CON);
	reg = reg | OVL_BGCLR_SEL_IN;
	writel(reg, ovl->regs + DISP_REG_OVL_DATAPATH_CON);
}

void mtk_ovl_bgclr_in_off(struct device *dev)
{
	struct mtk_disp_ovl *ovl = dev_get_drvdata(dev);
	unsigned int reg;

	reg = readl(ovl->regs + DISP_REG_OVL_DATAPATH_CON);
	reg = reg & ~OVL_BGCLR_SEL_IN;
	writel(reg, ovl->regs + DISP_REG_OVL_DATAPATH_CON);
}

static int mtk_disp_ovl_bind(struct device *dev, struct device *master,
			     void *data)
{
	return 0;
}

static void mtk_disp_ovl_unbind(struct device *dev, struct device *master,
				void *data)
{
}

static const struct component_ops mtk_disp_ovl_component_ops = {
	.bind	= mtk_disp_ovl_bind,
	.unbind = mtk_disp_ovl_unbind,
};

static int mtk_disp_ovl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_disp_ovl *priv;
	unsigned long irq_flags = IRQF_TRIGGER_NONE;
	int irq;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->dev = dev;
	atomic_set(&priv->fme_seq, 0);
	atomic_set(&priv->handoff_irq_status, 0);
	init_waitqueue_head(&priv->fme_wait_queue);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	priv->irq = irq;

	priv->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(priv->clk))
		return dev_err_probe(dev, PTR_ERR(priv->clk),
				     "failed to get ovl clk\n");

	priv->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->regs))
		return dev_err_probe(dev, PTR_ERR(priv->regs),
				     "failed to ioremap ovl\n");
#if IS_REACHABLE(CONFIG_MTK_CMDQ)
	ret = cmdq_dev_get_client_reg(dev, &priv->cmdq_reg, 0);
	if (ret)
		dev_dbg(dev, "get mediatek,gce-client-reg fail!\n");
#endif

	priv->data = of_device_get_match_data(dev);
	platform_set_drvdata(pdev, priv);
	if (priv->data->defer_irq_enable)
		irq_flags |= IRQF_NO_AUTOEN;

	ret = devm_request_irq(dev, irq, mtk_disp_ovl_irq_handler,
			       irq_flags, dev_name(dev), priv);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to request irq %d\n", irq);
	priv->irq_enabled = !priv->data->defer_irq_enable;

	pm_runtime_enable(dev);

	ret = component_add(dev, &mtk_disp_ovl_component_ops);
	if (ret) {
		pm_runtime_disable(dev);
		return dev_err_probe(dev, ret, "Failed to add component\n");
	}

	return 0;
}

static void mtk_disp_ovl_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &mtk_disp_ovl_component_ops);
	pm_runtime_disable(&pdev->dev);
}

static const struct mtk_disp_ovl_data mt2701_ovl_driver_data = {
	.addr = DISP_REG_OVL_ADDR_MT2701,
	.gmc_bits = 8,
	.layer_nr = 4,
	.fmt_rgb565_is_0 = false,
	.formats = mt8173_formats,
	.num_formats = ARRAY_SIZE(mt8173_formats),
};

static const struct mtk_disp_ovl_data mt8167_ovl_driver_data = {
	.addr = DISP_REG_OVL_ADDR_MT8173,
	.gmc_bits = 8,
	.layer_nr = 4,
	.fmt_rgb565_is_0 = true,
	.smi_id_en = true,
	.formats = mt8173_formats,
	.num_formats = ARRAY_SIZE(mt8173_formats),
};

static const struct mtk_disp_ovl_data mt8173_ovl_driver_data = {
	.addr = DISP_REG_OVL_ADDR_MT8173,
	.gmc_bits = 8,
	.layer_nr = 4,
	.fmt_rgb565_is_0 = true,
	.formats = mt8173_formats,
	.num_formats = ARRAY_SIZE(mt8173_formats),
};

static const struct mtk_disp_ovl_data mt8183_ovl_driver_data = {
	.addr = DISP_REG_OVL_ADDR_MT8173,
	.gmc_bits = 10,
	.layer_nr = 4,
	.fmt_rgb565_is_0 = true,
	.formats = mt8173_formats,
	.num_formats = ARRAY_SIZE(mt8173_formats),
};

static const struct mtk_disp_ovl_data mt8183_ovl_2l_driver_data = {
	.addr = DISP_REG_OVL_ADDR_MT8173,
	.gmc_bits = 10,
	.layer_nr = 2,
	.fmt_rgb565_is_0 = true,
	.formats = mt8173_formats,
	.num_formats = ARRAY_SIZE(mt8173_formats),
};

static const struct mtk_disp_ovl_data mt6789_ovl_driver_data = {
	.addr = DISP_REG_OVL_ADDR_MT8173,
	.gmc_bits = 10,
	.layer_nr = 4,
	.fmt_rgb565_is_0 = true,
	.smi_id_en = true,
	.formats = mt8173_formats,
	.num_formats = ARRAY_SIZE(mt8173_formats),
	.bypass_shadow = true,
	.skip_config_reset = true,
	.reset_on_stop = true,
	.defer_irq_enable = true,
};

static const struct mtk_disp_ovl_data mt6789_ovl_2l_driver_data = {
	.addr = DISP_REG_OVL_ADDR_MT8173,
	.gmc_bits = 10,
	.layer_nr = 2,
	.fmt_rgb565_is_0 = true,
	.smi_id_en = true,
	.formats = mt8173_formats,
	.num_formats = ARRAY_SIZE(mt8173_formats),
	.bypass_shadow = true,
	.skip_config_reset = true,
	.reset_on_stop = true,
	.defer_irq_enable = true,
};

static const struct mtk_disp_ovl_data mt8192_ovl_driver_data = {
	.addr = DISP_REG_OVL_ADDR_MT8173,
	.gmc_bits = 10,
	.layer_nr = 4,
	.fmt_rgb565_is_0 = true,
	.smi_id_en = true,
	.blend_modes = BIT(DRM_MODE_BLEND_PREMULTI) |
		       BIT(DRM_MODE_BLEND_COVERAGE) |
		       BIT(DRM_MODE_BLEND_PIXEL_NONE),
	.formats = mt8173_formats,
	.num_formats = ARRAY_SIZE(mt8173_formats),
};

static const struct mtk_disp_ovl_data mt8192_ovl_2l_driver_data = {
	.addr = DISP_REG_OVL_ADDR_MT8173,
	.gmc_bits = 10,
	.layer_nr = 2,
	.fmt_rgb565_is_0 = true,
	.smi_id_en = true,
	.blend_modes = BIT(DRM_MODE_BLEND_PREMULTI) |
		       BIT(DRM_MODE_BLEND_COVERAGE) |
		       BIT(DRM_MODE_BLEND_PIXEL_NONE),
	.formats = mt8173_formats,
	.num_formats = ARRAY_SIZE(mt8173_formats),
};

static const struct mtk_disp_ovl_data mt8195_ovl_driver_data = {
	.addr = DISP_REG_OVL_ADDR_MT8173,
	.gmc_bits = 10,
	.layer_nr = 4,
	.fmt_rgb565_is_0 = true,
	.smi_id_en = true,
	.supports_afbc = true,
	.blend_modes = BIT(DRM_MODE_BLEND_PREMULTI) |
		       BIT(DRM_MODE_BLEND_COVERAGE) |
		       BIT(DRM_MODE_BLEND_PIXEL_NONE),
	.formats = mt8195_formats,
	.num_formats = ARRAY_SIZE(mt8195_formats),
	.supports_clrfmt_ext = true,
};

static const struct of_device_id mtk_disp_ovl_driver_dt_match[] = {
	{ .compatible = "mediatek,mt6789-disp-ovl",
	  .data = &mt6789_ovl_driver_data},
	{ .compatible = "mediatek,mt6789-disp-ovl-2l",
	  .data = &mt6789_ovl_2l_driver_data},
	{ .compatible = "mediatek,mt2701-disp-ovl",
	  .data = &mt2701_ovl_driver_data},
	{ .compatible = "mediatek,mt8167-disp-ovl",
	  .data = &mt8167_ovl_driver_data},
	{ .compatible = "mediatek,mt8173-disp-ovl",
	  .data = &mt8173_ovl_driver_data},
	{ .compatible = "mediatek,mt8183-disp-ovl",
	  .data = &mt8183_ovl_driver_data},
	{ .compatible = "mediatek,mt8183-disp-ovl-2l",
	  .data = &mt8183_ovl_2l_driver_data},
	{ .compatible = "mediatek,mt8192-disp-ovl",
	  .data = &mt8192_ovl_driver_data},
	{ .compatible = "mediatek,mt8192-disp-ovl-2l",
	  .data = &mt8192_ovl_2l_driver_data},
	{ .compatible = "mediatek,mt8195-disp-ovl",
	  .data = &mt8195_ovl_driver_data},
	{},
};
MODULE_DEVICE_TABLE(of, mtk_disp_ovl_driver_dt_match);

struct platform_driver mtk_disp_ovl_driver = {
	.probe		= mtk_disp_ovl_probe,
	.remove		= mtk_disp_ovl_remove,
	.driver		= {
		.name	= "mediatek-disp-ovl",
		.of_match_table = mtk_disp_ovl_driver_dt_match,
	},
};
