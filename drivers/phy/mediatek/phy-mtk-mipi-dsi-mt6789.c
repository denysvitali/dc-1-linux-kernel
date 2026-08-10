// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include "phy-mtk-io.h"
#include "phy-mtk-mipi-dsi.h"

#define MIPITX_LANE_CON		0x000c
#define MIPITX_VOLTAGE_SEL	0x0010
#define RG_DSI_HSTX_LDO_REF_SEL	GENMASK(9, 6)
#define MIPITX_PRESERVED	0x0014
#define MIPITX_PLL_PWR		0x0028
#define MIPITX_PLL_CON0		0x002c
#define MIPITX_PLL_CON1		0x0030
#define MIPITX_PLL_CON4		0x003c
#define MIPITX_SW_CTRL_CON4	0x0060

#define MIPITX_D2_SW_CTL_EN	0x0144
#define MIPITX_D0_SW_CTL_EN	0x0244
#define MIPITX_CK_SW_CTL_EN	0x0344
#define MIPITX_D1_SW_CTL_EN	0x0444

#define AD_DSI_PLL_SDM_PWR_ON	BIT(0)
#define AD_DSI_PLL_SDM_ISO_EN	BIT(1)
#define RG_DSI_PLL_EN		BIT(4)
#define RG_DSI_PLL_POSDIV	GENMASK(10, 8)
#define DSI_SW_CTL_EN		BIT(0)
#define MIPI_TX_SW_ANA_CK_EN	BIT(8)

static int mtk_mipi_tx_pll_prepare(struct clk_hw *hw)
{
	struct mtk_mipi_tx *mipi_tx = mtk_mipi_tx_from_clk_hw(hw);
	void __iomem *base = mipi_tx->regs;
	unsigned int txdiv, txdiv0;
	u64 pcw;

	dev_dbg(mipi_tx->dev, "prepare: %u Hz\n", mipi_tx->data_rate);

	/* The factory firmware leaves its optional voltage override at zero. */
	if (!mipi_tx->voltage_initialized) {
		mtk_phy_update_field(base + MIPITX_VOLTAGE_SEL,
				     RG_DSI_HSTX_LDO_REF_SEL, 0);
		mipi_tx->voltage_initialized = true;
	}

	if (readl(base + MIPITX_PLL_CON1) & RG_DSI_PLL_EN) {
		/* LK can hand Linux an already-running PLL; still take ownership
		 * of the analog clock gate that belongs to this prepare cycle.
		 */
		mtk_phy_set_bits(base + MIPITX_SW_CTRL_CON4,
				 MIPI_TX_SW_ANA_CK_EN);
		dev_dbg(mipi_tx->dev, "MIPI TX PLL already enabled\n");
		return 0;
	}

	if (mipi_tx->data_rate >= 2000000000) {
		txdiv = 1;
		txdiv0 = 0;
	} else if (mipi_tx->data_rate >= 1000000000) {
		txdiv = 2;
		txdiv0 = 1;
	} else if (mipi_tx->data_rate >= 500000000) {
		txdiv = 4;
		txdiv0 = 2;
	} else if (mipi_tx->data_rate > 250000000) {
		txdiv = 8;
		txdiv0 = 3;
	} else if (mipi_tx->data_rate >= 125000000) {
		txdiv = 16;
		txdiv0 = 4;
	} else {
		return -EINVAL;
	}

	/* Keep this ordering and the full register values in sync with MT6789. */
	writel(0, base + MIPITX_PRESERVED);
	writel(0x00ff12e0, base + MIPITX_PLL_CON4);
	writel(0x3fff0180, base + MIPITX_LANE_CON);
	usleep_range(500, 600);
	writel(0x3fff0080, base + MIPITX_LANE_CON);

	mtk_phy_set_bits(base + MIPITX_D0_SW_CTL_EN, DSI_SW_CTL_EN);
	mtk_phy_set_bits(base + MIPITX_D1_SW_CTL_EN, DSI_SW_CTL_EN);
	mtk_phy_set_bits(base + MIPITX_D2_SW_CTL_EN, DSI_SW_CTL_EN);
	/* The factory MT6789 data maps its D3 selector to the D2 register. */
	mtk_phy_set_bits(base + MIPITX_D2_SW_CTL_EN, DSI_SW_CTL_EN);
	mtk_phy_set_bits(base + MIPITX_CK_SW_CTL_EN, DSI_SW_CTL_EN);

	mtk_phy_set_bits(base + MIPITX_PLL_PWR, AD_DSI_PLL_SDM_PWR_ON);
	usleep_range(30, 100);
	mtk_phy_clear_bits(base + MIPITX_PLL_PWR, AD_DSI_PLL_SDM_ISO_EN);

	pcw = div_u64(((u64)mipi_tx->data_rate * txdiv) << 24, 26000000);
	writel((u32)pcw, base + MIPITX_PLL_CON0);
	mtk_phy_update_field(base + MIPITX_PLL_CON1, RG_DSI_PLL_POSDIV,
			     txdiv0);
	mtk_phy_set_bits(base + MIPITX_PLL_CON1, RG_DSI_PLL_EN);

	usleep_range(50, 100);
	/* MT6789 places the software analog-clock enable at bit 8. */
	mtk_phy_set_bits(base + MIPITX_SW_CTRL_CON4, MIPI_TX_SW_ANA_CK_EN);

	return 0;
}

static void mtk_mipi_tx_pll_unprepare(struct clk_hw *hw)
{
	struct mtk_mipi_tx *mipi_tx = mtk_mipi_tx_from_clk_hw(hw);
	void __iomem *base = mipi_tx->regs;

	dev_dbg(mipi_tx->dev, "unprepare\n");

	mtk_phy_clear_bits(base + MIPITX_PLL_CON1, RG_DSI_PLL_EN);
	mtk_phy_clear_bits(base + MIPITX_SW_CTRL_CON4,
			   MIPI_TX_SW_ANA_CK_EN);
	mtk_phy_set_bits(base + MIPITX_PLL_PWR, AD_DSI_PLL_SDM_ISO_EN);
	mtk_phy_clear_bits(base + MIPITX_PLL_PWR, AD_DSI_PLL_SDM_PWR_ON);

	writel(0x3fff0180, base + MIPITX_LANE_CON);
	writel(0x3fff0100, base + MIPITX_LANE_CON);
}

static int mtk_mipi_tx_pll_determine_rate(struct clk_hw *hw,
					  struct clk_rate_request *req)
{
	req->rate = clamp_val(req->rate, 125000000, 1600000000);

	return 0;
}

static const struct clk_ops mtk_mipi_tx_pll_ops = {
	.prepare = mtk_mipi_tx_pll_prepare,
	.unprepare = mtk_mipi_tx_pll_unprepare,
	.determine_rate = mtk_mipi_tx_pll_determine_rate,
	.set_rate = mtk_mipi_tx_pll_set_rate,
	.recalc_rate = mtk_mipi_tx_pll_recalc_rate,
};

const struct mtk_mipitx_data mt6789_mipitx_data = {
	.mipi_tx_clk_ops = &mtk_mipi_tx_pll_ops,
};
