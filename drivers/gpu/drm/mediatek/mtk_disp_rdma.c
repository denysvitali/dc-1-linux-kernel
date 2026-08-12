// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015 MediaTek Inc.
 */

#include <drm/drm_fourcc.h>

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/soc/mediatek/mtk-cmdq.h>

#include "mtk_crtc.h"
#include "mtk_ddp_comp.h"
#include "mtk_disp_drv.h"
#include "mtk_drm_drv.h"

#define DISP_REG_RDMA_INT_ENABLE		0x0000
#define DISP_REG_RDMA_INT_STATUS		0x0004
#define RDMA_TARGET_LINE_INT				BIT(5)
#define RDMA_FIFO_UNDERFLOW_INT				BIT(4)
#define RDMA_EOF_ABNORMAL_INT				BIT(3)
#define RDMA_FRAME_END_INT				BIT(2)
#define RDMA_FRAME_START_INT				BIT(1)
#define RDMA_REG_UPDATE_INT				BIT(0)
#define RDMA_INT_STATUS_MASK				GENMASK(5, 0)
#define DISP_REG_RDMA_GLOBAL_CON		0x0010
#define RDMA_ENGINE_EN					BIT(0)
#define RDMA_MODE_MEMORY				BIT(1)
#define RDMA_SOFT_RESET					BIT(4)
#define RDMA_SMI_BUSY					BIT(12)
#define RDMA_RESET_STATE_MASK				GENMASK(10, 8)
#define RDMA_RESET_STATE_IDLE				0x100
#define DISP_REG_RDMA_SIZE_CON_0		0x0014
#define RDMA_MATRIX_ENABLE				BIT(17)
#define RDMA_MATRIX_INT_MTX_SEL				GENMASK(23, 20)
#define RDMA_MATRIX_INT_MTX_BT601_to_RGB		(6 << 20)
#define DISP_REG_RDMA_SIZE_CON_1		0x0018
#define DISP_REG_RDMA_TARGET_LINE		0x001c
#define DISP_RDMA_MEM_CON			0x0024
#define MEM_MODE_INPUT_FORMAT_RGB565			(0x000 << 4)
#define MEM_MODE_INPUT_FORMAT_RGB888			(0x001 << 4)
#define MEM_MODE_INPUT_FORMAT_RGBA8888			(0x002 << 4)
#define MEM_MODE_INPUT_FORMAT_ARGB8888			(0x003 << 4)
#define MEM_MODE_INPUT_FORMAT_UYVY			(0x004 << 4)
#define MEM_MODE_INPUT_FORMAT_YUYV			(0x005 << 4)
#define MEM_MODE_INPUT_SWAP				BIT(8)
#define DISP_RDMA_MEM_SRC_PITCH			0x002c
#define DISP_RDMA_MEM_GMC_SETTING_0		0x0030
#define DISP_REG_RDMA_FIFO_CON			0x0040
#define RDMA_FIFO_UNDERFLOW_EN				BIT(31)
#define RDMA_FIFO_PSEUDO_SIZE(bytes)			(((bytes) / 16) << 16)
#define RDMA_OUTPUT_VALID_FIFO_THRESHOLD(bytes)		((bytes) / 16)
#define RDMA_FIFO_SIZE(rdma)			((rdma)->data->fifo_size)
#define DISP_RDMA_MEM_START_ADDR		0x0f00
#define DISP_REG_RDMA_DBG_OUT			0x0100
#define DISP_REG_RDMA_DBG_OUT1			0x010c
#define DISP_REG_RDMA_DBG_OUT2			0x0110
#define DISP_REG_RDMA_IN_P_CNT			0x0120
#define DISP_REG_RDMA_IN_LINE_CNT		0x0124
#define DISP_REG_RDMA_OUT_P_CNT			0x0128
#define DISP_REG_RDMA_OUT_LINE_CNT		0x012c

#define RDMA_MEM_GMC				0x40402020

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

struct mtk_disp_rdma_data {
	unsigned int fifo_size;
	const u32 *formats;
	size_t num_formats;
	bool checked_handoff;
	bool zero_output_valid_threshold;
};

/*
 * struct mtk_disp_rdma - DISP_RDMA driver structure
 * @data: local driver data
 */
struct mtk_disp_rdma {
	struct device			*dev;
	struct clk			*clk;
	void __iomem			*regs;
	struct cmdq_client_reg		cmdq_reg;
	const struct mtk_disp_rdma_data	*data;
	int				irq;
	bool				irq_enabled;
	atomic_t			frame_end_seq;
	atomic_t			handoff_irq_status;
	wait_queue_head_t		frame_end_wait_queue;
	u32				handoff_idle_counters[4];
	bool				handoff_idle_proven;
	void				(*vblank_cb)(void *data);
	void				*vblank_cb_data;
	u32				fifo_size;
};

static irqreturn_t mtk_disp_rdma_irq_handler(int irq, void *dev_id)
{
	struct mtk_disp_rdma *priv = dev_id;
	u32 pending;

	pending = readl(priv->regs + DISP_REG_RDMA_INT_STATUS) &
		  RDMA_INT_STATUS_MASK;
	if (!pending)
		return IRQ_NONE;

	/* RDMA_INT_STATUS is write-zero-to-clear. Preserve unrelated bits. */
	writel(~pending, priv->regs + DISP_REG_RDMA_INT_STATUS);
	atomic_or(pending, &priv->handoff_irq_status);

	if (pending & (RDMA_FIFO_UNDERFLOW_INT | RDMA_EOF_ABNORMAL_INT))
		dev_err_ratelimited(priv->dev,
				    "RDMA abnormal interrupt: status=%#x\n",
				    pending);

	if (pending & RDMA_FRAME_END_INT) {
		atomic_inc(&priv->frame_end_seq);
		wake_up(&priv->frame_end_wait_queue);
		if (priv->vblank_cb)
			priv->vblank_cb(priv->vblank_cb_data);
	}

	return IRQ_HANDLED;
}

static void rdma_update_bits(struct device *dev, unsigned int reg,
			     unsigned int mask, unsigned int val)
{
	struct mtk_disp_rdma *rdma = dev_get_drvdata(dev);
	unsigned int tmp = readl(rdma->regs + reg);

	tmp = (tmp & ~mask) | (val & mask);
	writel(tmp, rdma->regs + reg);
}

void mtk_rdma_register_vblank_cb(struct device *dev,
				 void (*vblank_cb)(void *),
				 void *vblank_cb_data)
{
	struct mtk_disp_rdma *rdma = dev_get_drvdata(dev);

	rdma->vblank_cb = vblank_cb;
	rdma->vblank_cb_data = vblank_cb_data;
}

void mtk_rdma_unregister_vblank_cb(struct device *dev)
{
	struct mtk_disp_rdma *rdma = dev_get_drvdata(dev);

	rdma->vblank_cb = NULL;
	rdma->vblank_cb_data = NULL;
}

void mtk_rdma_enable_vblank(struct device *dev)
{
	rdma_update_bits(dev, DISP_REG_RDMA_INT_ENABLE, RDMA_FRAME_END_INT,
			 RDMA_FRAME_END_INT);
}

void mtk_rdma_disable_vblank(struct device *dev)
{
	rdma_update_bits(dev, DISP_REG_RDMA_INT_ENABLE, RDMA_FRAME_END_INT, 0);
}

const u32 *mtk_rdma_get_formats(struct device *dev)
{
	struct mtk_disp_rdma *rdma = dev_get_drvdata(dev);

	return rdma->data->formats;
}

size_t mtk_rdma_get_num_formats(struct device *dev)
{
	struct mtk_disp_rdma *rdma = dev_get_drvdata(dev);

	return rdma->data->num_formats;
}

int mtk_rdma_clk_enable(struct device *dev)
{
	struct mtk_disp_rdma *rdma = dev_get_drvdata(dev);

	return clk_prepare_enable(rdma->clk);
}

void mtk_rdma_clk_disable(struct device *dev)
{
	struct mtk_disp_rdma *rdma = dev_get_drvdata(dev);

	if (rdma->data->checked_handoff && rdma->irq_enabled) {
		writel(0, rdma->regs + DISP_REG_RDMA_INT_ENABLE);
		readl(rdma->regs + DISP_REG_RDMA_INT_ENABLE);
		disable_irq(rdma->irq);
		rdma->irq_enabled = false;
		writel(0, rdma->regs + DISP_REG_RDMA_INT_STATUS);
		readl(rdma->regs + DISP_REG_RDMA_INT_STATUS);
	}
	clk_disable_unprepare(rdma->clk);
}

void mtk_rdma_start(struct device *dev)
{
	rdma_update_bits(dev, DISP_REG_RDMA_GLOBAL_CON, RDMA_ENGINE_EN,
			 RDMA_ENGINE_EN);
}

void mtk_rdma_stop(struct device *dev)
{
	rdma_update_bits(dev, DISP_REG_RDMA_GLOBAL_CON, RDMA_ENGINE_EN, 0);
}

int mtk_rdma_handoff_stop(struct device *dev)
{
	struct mtk_disp_rdma *rdma = dev_get_drvdata(dev);
	u32 val;
	int ret;

	if (!rdma->data->checked_handoff)
		return -EOPNOTSUPP;

	writel(0, rdma->regs + DISP_REG_RDMA_INT_ENABLE);
	val = readl(rdma->regs + DISP_REG_RDMA_INT_ENABLE);
	if (val) {
		ret = -EIO;
		goto fail;
	}
	if (rdma->irq_enabled)
		synchronize_irq(rdma->irq);
	rdma_update_bits(dev, DISP_REG_RDMA_GLOBAL_CON, RDMA_ENGINE_EN, 0);
	ret = readl_poll_timeout(rdma->regs + DISP_REG_RDMA_GLOBAL_CON, val,
				 !(val & (RDMA_ENGINE_EN | RDMA_SMI_BUSY)),
				 10, 100000);
	if (ret)
		goto fail;

	writel(RDMA_SOFT_RESET, rdma->regs + DISP_REG_RDMA_GLOBAL_CON);
	ret = readl_poll_timeout(rdma->regs + DISP_REG_RDMA_GLOBAL_CON, val,
				 (val & RDMA_SOFT_RESET) &&
				 (val & RDMA_RESET_STATE_MASK) !=
				 RDMA_RESET_STATE_IDLE, 1, 1000);
	if (ret)
		goto fail;
	udelay(1);
	writel(0, rdma->regs + DISP_REG_RDMA_GLOBAL_CON);
	ret = readl_poll_timeout(rdma->regs + DISP_REG_RDMA_GLOBAL_CON, val,
				 !(val & RDMA_SOFT_RESET) &&
				 (val & RDMA_RESET_STATE_MASK) ==
				 RDMA_RESET_STATE_IDLE, 1, 1000);
	if (!ret) {
		writel(0, rdma->regs + DISP_REG_RDMA_INT_STATUS);
		val = readl(rdma->regs + DISP_REG_RDMA_INT_STATUS);
		if (val & RDMA_INT_STATUS_MASK) {
			ret = -EIO;
			goto fail;
		}
		if (!rdma->irq_enabled) {
			enable_irq(rdma->irq);
			rdma->irq_enabled = true;
		}
		return 0;
	}

fail:
	/* Never quarantine with a level IRQ whose hardware mask is unproven. */
	if (rdma->irq_enabled) {
		disable_irq(rdma->irq);
		rdma->irq_enabled = false;
	}
	dev_err(dev,
		"RDMA checked stop failed: %d global=%#x inten=%#x intsta=%#x\n",
		ret, readl(rdma->regs + DISP_REG_RDMA_GLOBAL_CON),
		readl(rdma->regs + DISP_REG_RDMA_INT_ENABLE),
		readl(rdma->regs + DISP_REG_RDMA_INT_STATUS));
	return ret;
}

int mtk_rdma_handoff_wait_idle(struct device *dev)
{
	struct mtk_disp_rdma *rdma = dev_get_drvdata(dev);
	u64 deadline = ktime_get_ns() + 100 * NSEC_PER_MSEC;
	u64 stable_since = 0;
	u32 counters[4] = { 0 };
	u32 global, inten, intsta, irq_status;
	bool have_counters = false;

	rdma->handoff_idle_proven = false;
	writel(0, rdma->regs + DISP_REG_RDMA_INT_ENABLE);
	inten = readl(rdma->regs + DISP_REG_RDMA_INT_ENABLE);
	if (rdma->irq_enabled)
		synchronize_irq(rdma->irq);
	irq_status = atomic_xchg(&rdma->handoff_irq_status, 0);
	intsta = readl(rdma->regs + DISP_REG_RDMA_INT_STATUS);
	if (inten || ((irq_status | intsta) &
		      (RDMA_FIFO_UNDERFLOW_INT | RDMA_EOF_ABNORMAL_INT)))
		goto fail;

	for (;;) {
		u32 sample[4];
		u64 now = ktime_get_ns();
		bool idle;

		global = readl(rdma->regs + DISP_REG_RDMA_GLOBAL_CON);
		intsta = readl(rdma->regs + DISP_REG_RDMA_INT_STATUS);
		if (intsta & (RDMA_FIFO_UNDERFLOW_INT |
			      RDMA_EOF_ABNORMAL_INT))
			goto fail;
		sample[0] = readl(rdma->regs + DISP_REG_RDMA_IN_P_CNT);
		sample[1] = readl(rdma->regs + DISP_REG_RDMA_IN_LINE_CNT);
		sample[2] = readl(rdma->regs + DISP_REG_RDMA_OUT_P_CNT);
		sample[3] = readl(rdma->regs + DISP_REG_RDMA_OUT_LINE_CNT);
		idle = (global & (RDMA_ENGINE_EN | RDMA_SOFT_RESET |
				  RDMA_SMI_BUSY | RDMA_RESET_STATE_MASK)) ==
		       (RDMA_ENGINE_EN | RDMA_RESET_STATE_IDLE);
		if (!idle ||
		    (have_counters && memcmp(counters, sample,
					     sizeof(counters)))) {
			stable_since = 0;
			have_counters = false;
		} else if (!stable_since) {
			memcpy(counters, sample, sizeof(counters));
			have_counters = true;
			stable_since = now;
			dev_info(dev,
				 "RDMA idle proof start: t=%llu global=%#x count=%#x/%#x/%#x/%#x\n",
				 (unsigned long long)now, global, sample[0],
				 sample[1], sample[2], sample[3]);
		} else if (now - stable_since >= 20 * NSEC_PER_MSEC) {
			dev_info(dev,
				 "RDMA idle proof complete: t=%llu global=%#x count=%#x/%#x/%#x/%#x\n",
				 (unsigned long long)now, global, sample[0],
				 sample[1], sample[2], sample[3]);
			memcpy(rdma->handoff_idle_counters, sample,
			       sizeof(rdma->handoff_idle_counters));
			rdma->handoff_idle_proven = true;
			return 0;
		}
		if (now >= deadline)
			break;
		usleep_range(100, 200);
	}

fail:
	dev_err(dev,
		"RDMA idle proof failed: global=%#x inten=%#x intsta=%#x status=%#x count=%#x/%#x/%#x/%#x\n",
		readl(rdma->regs + DISP_REG_RDMA_GLOBAL_CON), inten, intsta,
		irq_status, readl(rdma->regs + DISP_REG_RDMA_IN_P_CNT),
		readl(rdma->regs + DISP_REG_RDMA_IN_LINE_CNT),
		readl(rdma->regs + DISP_REG_RDMA_OUT_P_CNT),
		readl(rdma->regs + DISP_REG_RDMA_OUT_LINE_CNT));
	return -EBUSY;
}

int mtk_rdma_handoff_validate_idle(struct device *dev)
{
	struct mtk_disp_rdma *rdma = dev_get_drvdata(dev);
	u32 sample[4];
	u32 global = readl(rdma->regs + DISP_REG_RDMA_GLOBAL_CON);
	u32 intsta = readl(rdma->regs + DISP_REG_RDMA_INT_STATUS);
	bool idle;

	sample[0] = readl(rdma->regs + DISP_REG_RDMA_IN_P_CNT);
	sample[1] = readl(rdma->regs + DISP_REG_RDMA_IN_LINE_CNT);
	sample[2] = readl(rdma->regs + DISP_REG_RDMA_OUT_P_CNT);
	sample[3] = readl(rdma->regs + DISP_REG_RDMA_OUT_LINE_CNT);
	idle = (global & (RDMA_ENGINE_EN | RDMA_SOFT_RESET | RDMA_SMI_BUSY |
			  RDMA_RESET_STATE_MASK)) ==
	       (RDMA_ENGINE_EN | RDMA_RESET_STATE_IDLE);
	if (!rdma->handoff_idle_proven || !idle ||
	    memcmp(rdma->handoff_idle_counters, sample, sizeof(sample)) ||
	    (intsta & (RDMA_FIFO_UNDERFLOW_INT | RDMA_EOF_ABNORMAL_INT))) {
		dev_err(dev,
			"RDMA idle revalidation failed: proven=%u global=%#x intsta=%#x count=%#x/%#x/%#x/%#x\n",
			rdma->handoff_idle_proven, global, intsta, sample[0],
			sample[1], sample[2], sample[3]);
		return -EBUSY;
	}

	return 0;
}

int mtk_rdma_handoff_frame_arm(struct device *dev, u32 *frame_end_seq)
{
	struct mtk_disp_rdma *rdma = dev_get_drvdata(dev);
	u32 val;

	if (!rdma->data->checked_handoff || !frame_end_seq)
		return -EINVAL;

	writel(0, rdma->regs + DISP_REG_RDMA_INT_ENABLE);
	if (readl(rdma->regs + DISP_REG_RDMA_INT_ENABLE))
		return -EIO;
	if (rdma->irq_enabled)
		synchronize_irq(rdma->irq);
	writel(0, rdma->regs + DISP_REG_RDMA_INT_STATUS);
	val = readl(rdma->regs + DISP_REG_RDMA_INT_STATUS);
	if (val & RDMA_INT_STATUS_MASK)
		return -EIO;
	atomic_set(&rdma->handoff_irq_status, 0);

	*frame_end_seq = atomic_read(&rdma->frame_end_seq);
	if (!rdma->irq_enabled) {
		enable_irq(rdma->irq);
		rdma->irq_enabled = true;
	}
	writel(RDMA_FRAME_END_INT | RDMA_FIFO_UNDERFLOW_INT |
	       RDMA_EOF_ABNORMAL_INT,
	       rdma->regs + DISP_REG_RDMA_INT_ENABLE);
	if (readl(rdma->regs + DISP_REG_RDMA_INT_ENABLE) !=
	    (RDMA_FRAME_END_INT | RDMA_FIFO_UNDERFLOW_INT |
	     RDMA_EOF_ABNORMAL_INT))
		return -EIO;

	return 0;
}

int mtk_rdma_handoff_frame_wait(struct device *dev, u32 frame_end_seq)
{
	struct mtk_disp_rdma *rdma = dev_get_drvdata(dev);
	u32 inten, irq_status, val;
	int idle_ret;
	long ret;

	ret = wait_event_timeout(rdma->frame_end_wait_queue,
				 atomic_read(&rdma->frame_end_seq) != frame_end_seq,
				 msecs_to_jiffies(100));
	writel(0, rdma->regs + DISP_REG_RDMA_INT_ENABLE);
	inten = readl(rdma->regs + DISP_REG_RDMA_INT_ENABLE);
	if (rdma->irq_enabled)
		synchronize_irq(rdma->irq);
	irq_status = atomic_xchg(&rdma->handoff_irq_status, 0);
	if (inten) {
		dev_err(dev, "RDMA frame mask failed: inten=%#x status=%#x\n",
			inten, irq_status);
		return -EIO;
	}
	if (!ret) {
		dev_err(dev,
			"RDMA frame timeout: seq=%u now=%u status=%#x global=%#x intsta=%#x\n",
			frame_end_seq, atomic_read(&rdma->frame_end_seq),
			irq_status,
			readl(rdma->regs + DISP_REG_RDMA_GLOBAL_CON),
			readl(rdma->regs + DISP_REG_RDMA_INT_STATUS));
		return -ETIMEDOUT;
	}
	if (irq_status & (RDMA_FIFO_UNDERFLOW_INT | RDMA_EOF_ABNORMAL_INT)) {
		dev_err(dev, "RDMA frame abnormal: status=%#x\n",
			irq_status);
		return -EIO;
	}
	idle_ret = readl_poll_timeout(rdma->regs + DISP_REG_RDMA_GLOBAL_CON,
				      val,
				      (val & RDMA_ENGINE_EN) &&
				      !(val & RDMA_SMI_BUSY) &&
				      (val & RDMA_RESET_STATE_MASK) ==
				      RDMA_RESET_STATE_IDLE,
				      10, 10000);
	if (idle_ret) {
		dev_err(dev, "RDMA inherited frame did not become idle: global=%#x\n",
			val);
		return idle_ret;
	}
	val = readl(rdma->regs + DISP_REG_RDMA_INT_STATUS);
	if (val & (RDMA_FIFO_UNDERFLOW_INT | RDMA_EOF_ABNORMAL_INT)) {
		dev_err(dev, "RDMA post-frame abnormal: intsta=%#x\n", val);
		return -EIO;
	}

	return 0;
}

void mtk_rdma_handoff_frame_cancel(struct device *dev)
{
	struct mtk_disp_rdma *rdma = dev_get_drvdata(dev);
	u32 inten;

	writel(0, rdma->regs + DISP_REG_RDMA_INT_ENABLE);
	inten = readl(rdma->regs + DISP_REG_RDMA_INT_ENABLE);
	if (rdma->irq_enabled)
		synchronize_irq(rdma->irq);
	if (inten && rdma->irq_enabled) {
		disable_irq(rdma->irq);
		rdma->irq_enabled = false;
		dev_err(dev, "RDMA IRQ mask failed during cancel: inten=%#x\n",
			inten);
	}
}

void mtk_rdma_handoff_dump(struct device *dev, const char *stage)
{
	struct mtk_disp_rdma *rdma = dev_get_drvdata(dev);

	if (!rdma->data->checked_handoff)
		return;

	dev_err(dev,
		"handoff %s: global=%#x inten=%#x intsta=%#x size=%#x/%#x fifo=%#x count=%#x/%#x/%#x/%#x dbg=%#x/%#x/%#x\n",
		stage,
		readl(rdma->regs + DISP_REG_RDMA_GLOBAL_CON),
		readl(rdma->regs + DISP_REG_RDMA_INT_ENABLE),
		readl(rdma->regs + DISP_REG_RDMA_INT_STATUS),
		readl(rdma->regs + DISP_REG_RDMA_SIZE_CON_0),
		readl(rdma->regs + DISP_REG_RDMA_SIZE_CON_1),
		readl(rdma->regs + DISP_REG_RDMA_FIFO_CON),
		readl(rdma->regs + DISP_REG_RDMA_IN_P_CNT),
		readl(rdma->regs + DISP_REG_RDMA_IN_LINE_CNT),
		readl(rdma->regs + DISP_REG_RDMA_OUT_P_CNT),
		readl(rdma->regs + DISP_REG_RDMA_OUT_LINE_CNT),
		readl(rdma->regs + DISP_REG_RDMA_DBG_OUT),
		readl(rdma->regs + DISP_REG_RDMA_DBG_OUT1),
		readl(rdma->regs + DISP_REG_RDMA_DBG_OUT2));
}

void mtk_rdma_config(struct device *dev, unsigned int width,
		     unsigned int height, unsigned int vrefresh,
		     unsigned int bpc, struct cmdq_pkt *cmdq_pkt)
{
	unsigned int threshold;
	unsigned int reg;
	struct mtk_disp_rdma *rdma = dev_get_drvdata(dev);
	u32 rdma_fifo_size;

	mtk_ddp_write_mask(cmdq_pkt, width, &rdma->cmdq_reg, rdma->regs,
			   DISP_REG_RDMA_SIZE_CON_0, 0xfff);
	mtk_ddp_write_mask(cmdq_pkt, height, &rdma->cmdq_reg, rdma->regs,
			   DISP_REG_RDMA_SIZE_CON_1, 0xfffff);

	if (rdma->fifo_size)
		rdma_fifo_size = rdma->fifo_size;
	else
		rdma_fifo_size = RDMA_FIFO_SIZE(rdma);

	/*
	 * Enable FIFO underflow since DSI and DPI can't be blocked.
	 * Keep the configured FIFO pseudo size. Most generations use a 70%
	 * output threshold, while MT6789 video mode follows the vendor setting
	 * of zero so an empty FIFO can feed the first active line.
	 */
	threshold = rdma->data->zero_output_valid_threshold ? 0 :
		    rdma_fifo_size * 7 / 10;
	reg = RDMA_FIFO_UNDERFLOW_EN |
	      RDMA_FIFO_PSEUDO_SIZE(rdma_fifo_size) |
	      RDMA_OUTPUT_VALID_FIFO_THRESHOLD(threshold);
	mtk_ddp_write(cmdq_pkt, reg, &rdma->cmdq_reg, rdma->regs, DISP_REG_RDMA_FIFO_CON);
}

int mtk_rdma_handoff_validate_config(struct device *dev, unsigned int width,
				     unsigned int height)
{
	struct mtk_disp_rdma *rdma = dev_get_drvdata(dev);
	u32 fifo_size, expected_fifo, fifo, size0, size1;

	if (!rdma->data->checked_handoff)
		return -EOPNOTSUPP;

	fifo_size = rdma->fifo_size ?: RDMA_FIFO_SIZE(rdma);
	expected_fifo = RDMA_FIFO_UNDERFLOW_EN |
			RDMA_FIFO_PSEUDO_SIZE(fifo_size);
	size0 = readl(rdma->regs + DISP_REG_RDMA_SIZE_CON_0);
	size1 = readl(rdma->regs + DISP_REG_RDMA_SIZE_CON_1);
	fifo = readl(rdma->regs + DISP_REG_RDMA_FIFO_CON);
	if ((size0 & 0xfff) == width && (size1 & 0xfffff) == height &&
	    fifo == expected_fifo)
		return 0;

	dev_err(dev,
		"RDMA handoff config mismatch: size=%#x/%#x fifo=%#x expected=%#x\n",
		size0, size1, fifo, expected_fifo);
	return -EIO;
}

static unsigned int rdma_fmt_convert(struct mtk_disp_rdma *rdma,
				     unsigned int fmt)
{
	/* The return value in switch "MEM_MODE_INPUT_FORMAT_XXX"
	 * is defined in mediatek HW data sheet.
	 * The alphabet order in XXX is no relation to data
	 * arrangement in memory.
	 */
	switch (fmt) {
	default:
	case DRM_FORMAT_RGB565:
		return MEM_MODE_INPUT_FORMAT_RGB565;
	case DRM_FORMAT_BGR565:
		return MEM_MODE_INPUT_FORMAT_RGB565 | MEM_MODE_INPUT_SWAP;
	case DRM_FORMAT_RGB888:
		return MEM_MODE_INPUT_FORMAT_RGB888;
	case DRM_FORMAT_BGR888:
		return MEM_MODE_INPUT_FORMAT_RGB888 | MEM_MODE_INPUT_SWAP;
	case DRM_FORMAT_RGBX8888:
	case DRM_FORMAT_RGBA8888:
		return MEM_MODE_INPUT_FORMAT_ARGB8888;
	case DRM_FORMAT_BGRX8888:
	case DRM_FORMAT_BGRA8888:
		return MEM_MODE_INPUT_FORMAT_ARGB8888 | MEM_MODE_INPUT_SWAP;
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_ARGB8888:
		return MEM_MODE_INPUT_FORMAT_RGBA8888;
	case DRM_FORMAT_XBGR8888:
	case DRM_FORMAT_ABGR8888:
		return MEM_MODE_INPUT_FORMAT_RGBA8888 | MEM_MODE_INPUT_SWAP;
	case DRM_FORMAT_UYVY:
		return MEM_MODE_INPUT_FORMAT_UYVY;
	case DRM_FORMAT_YUYV:
		return MEM_MODE_INPUT_FORMAT_YUYV;
	}
}

unsigned int mtk_rdma_layer_nr(struct device *dev)
{
	return 1;
}

void mtk_rdma_layer_config(struct device *dev, unsigned int idx,
			   struct mtk_plane_state *state,
			   struct cmdq_pkt *cmdq_pkt)
{
	struct mtk_disp_rdma *rdma = dev_get_drvdata(dev);
	struct mtk_plane_pending_state *pending = &state->pending;
	unsigned int addr = pending->addr;
	unsigned int pitch = pending->pitch & 0xffff;
	unsigned int fmt = pending->format;
	unsigned int con;

	con = rdma_fmt_convert(rdma, fmt);
	mtk_ddp_write_relaxed(cmdq_pkt, con, &rdma->cmdq_reg, rdma->regs, DISP_RDMA_MEM_CON);

	if (fmt == DRM_FORMAT_UYVY || fmt == DRM_FORMAT_YUYV) {
		mtk_ddp_write_mask(cmdq_pkt, RDMA_MATRIX_ENABLE, &rdma->cmdq_reg, rdma->regs,
				   DISP_REG_RDMA_SIZE_CON_0,
				   RDMA_MATRIX_ENABLE);
		mtk_ddp_write_mask(cmdq_pkt, RDMA_MATRIX_INT_MTX_BT601_to_RGB,
				   &rdma->cmdq_reg, rdma->regs, DISP_REG_RDMA_SIZE_CON_0,
				   RDMA_MATRIX_INT_MTX_SEL);
	} else {
		mtk_ddp_write_mask(cmdq_pkt, 0, &rdma->cmdq_reg, rdma->regs,
				   DISP_REG_RDMA_SIZE_CON_0,
				   RDMA_MATRIX_ENABLE);
	}
	mtk_ddp_write_relaxed(cmdq_pkt, addr, &rdma->cmdq_reg, rdma->regs,
			      DISP_RDMA_MEM_START_ADDR);
	mtk_ddp_write_relaxed(cmdq_pkt, pitch, &rdma->cmdq_reg, rdma->regs,
			      DISP_RDMA_MEM_SRC_PITCH);
	mtk_ddp_write(cmdq_pkt, RDMA_MEM_GMC, &rdma->cmdq_reg, rdma->regs,
		      DISP_RDMA_MEM_GMC_SETTING_0);
	mtk_ddp_write_mask(cmdq_pkt, RDMA_MODE_MEMORY, &rdma->cmdq_reg, rdma->regs,
			   DISP_REG_RDMA_GLOBAL_CON, RDMA_MODE_MEMORY);

}

static int mtk_disp_rdma_bind(struct device *dev, struct device *master,
			      void *data)
{
	return 0;

}

static void mtk_disp_rdma_unbind(struct device *dev, struct device *master,
				 void *data)
{
}

static const struct component_ops mtk_disp_rdma_component_ops = {
	.bind	= mtk_disp_rdma_bind,
	.unbind = mtk_disp_rdma_unbind,
};

static int mtk_disp_rdma_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_disp_rdma *priv;
	unsigned long irq_flags = IRQF_TRIGGER_NONE;
	int irq;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->dev = dev;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	priv->irq = irq;

	priv->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(priv->clk))
		return dev_err_probe(dev, PTR_ERR(priv->clk),
				     "failed to get rdma clk\n");

	priv->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->regs))
		return dev_err_probe(dev, PTR_ERR(priv->regs),
				     "failed to ioremap rdma\n");
#if IS_REACHABLE(CONFIG_MTK_CMDQ)
	ret = cmdq_dev_get_client_reg(dev, &priv->cmdq_reg, 0);
	if (ret)
		dev_dbg(dev, "get mediatek,gce-client-reg fail!\n");
#endif

	ret = of_property_read_u32(dev->of_node,
				   "mediatek,rdma-fifo-size",
				   &priv->fifo_size);
	if (ret && (ret != -EINVAL))
		return dev_err_probe(dev, ret, "Failed to get rdma fifo size\n");

	priv->data = of_device_get_match_data(dev);
	platform_set_drvdata(pdev, priv);
	atomic_set(&priv->frame_end_seq, 0);
	atomic_set(&priv->handoff_irq_status, 0);
	init_waitqueue_head(&priv->frame_end_wait_queue);

	if (priv->data->checked_handoff) {
		/* The inherited block is not clock-owned until component enable. */
		irq_flags |= IRQF_NO_AUTOEN;
	} else {
		/* Disable and clear pending interrupts on cold-start platforms. */
		writel(0, priv->regs + DISP_REG_RDMA_INT_ENABLE);
		writel(0, priv->regs + DISP_REG_RDMA_INT_STATUS);
	}

	ret = devm_request_irq(dev, irq, mtk_disp_rdma_irq_handler,
			       irq_flags, dev_name(dev), priv);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to request irq %d\n", irq);
	priv->irq_enabled = !priv->data->checked_handoff;

	pm_runtime_enable(dev);

	ret = component_add(dev, &mtk_disp_rdma_component_ops);
	if (ret) {
		pm_runtime_disable(dev);
		return dev_err_probe(dev, ret, "Failed to add component\n");
	}

	return 0;
}

static void mtk_disp_rdma_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &mtk_disp_rdma_component_ops);

	pm_runtime_disable(&pdev->dev);
}

static const struct mtk_disp_rdma_data mt2701_rdma_driver_data = {
	.fifo_size = SZ_4K,
	.formats = mt8173_formats,
	.num_formats = ARRAY_SIZE(mt8173_formats),
};

static const struct mtk_disp_rdma_data mt6789_rdma_driver_data = {
	.fifo_size = SZ_32K + SZ_8K + SZ_4K + SZ_2K + SZ_1K +
		     SZ_512 + SZ_256 + SZ_64 + SZ_32,
	.formats = mt8173_formats,
	.num_formats = ARRAY_SIZE(mt8173_formats),
	.checked_handoff = true,
	.zero_output_valid_threshold = true,
};

static const struct mtk_disp_rdma_data mt8173_rdma_driver_data = {
	.fifo_size = SZ_8K,
	.formats = mt8173_formats,
	.num_formats = ARRAY_SIZE(mt8173_formats),
};

static const struct mtk_disp_rdma_data mt8183_rdma_driver_data = {
	.fifo_size = 5 * SZ_1K,
	.formats = mt8173_formats,
	.num_formats = ARRAY_SIZE(mt8173_formats),
};

static const struct mtk_disp_rdma_data mt8195_rdma_driver_data = {
	.fifo_size = 1920,
	.formats = mt8173_formats,
	.num_formats = ARRAY_SIZE(mt8173_formats),
};

static const struct of_device_id mtk_disp_rdma_driver_dt_match[] = {
	{ .compatible = "mediatek,mt2701-disp-rdma",
	  .data = &mt2701_rdma_driver_data},
	{ .compatible = "mediatek,mt6789-disp-rdma",
	  .data = &mt6789_rdma_driver_data},
	{ .compatible = "mediatek,mt8173-disp-rdma",
	  .data = &mt8173_rdma_driver_data},
	{ .compatible = "mediatek,mt8183-disp-rdma",
	  .data = &mt8183_rdma_driver_data},
	{ .compatible = "mediatek,mt8195-disp-rdma",
	  .data = &mt8195_rdma_driver_data},
	{},
};
MODULE_DEVICE_TABLE(of, mtk_disp_rdma_driver_dt_match);

struct platform_driver mtk_disp_rdma_driver = {
	.probe		= mtk_disp_rdma_probe,
	.remove		= mtk_disp_rdma_remove,
	.driver		= {
		.name	= "mediatek-disp-rdma",
		.of_match_table = mtk_disp_rdma_driver_dt_match,
	},
};
