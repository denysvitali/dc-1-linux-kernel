// SPDX-License-Identifier: GPL-2.0-only

#include <linux/clk-provider.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>

#include <dt-bindings/clock/mediatek,mt6789-clk.h>

#include "clk-gate.h"
#include "clk-mtk.h"

static const struct mtk_gate_regs imp_iic_wrap_cg_regs = {
	.set_ofs = 0xe08,
	.clr_ofs = 0xe04,
	.sta_ofs = 0xe00,
};

/*
 * NOTE: every gate here originally named its parent "top_i2c". That is the
 * MT8186/MT8188 name for this clock; MT6789's topckgen registers it as
 * "i2c_ck" (a FACTOR off the "i2c_sel" mux, clk-mt6789-topckgen.c:95,711) and
 * defines no "top_i2c" at all. The parent therefore never resolved, so every
 * i2c gate came up with rate 0:
 *   imp_iic_wrap_en_ap_clock_i2c2  prepare=1  rate=0  hw=N  11eb0000.i2c main
 * while i2c_sel/i2c_ck ran at 124.8MHz. i2c-mt65xx then derived its timing
 * divider from a zero clock and EVERY transfer on the bus timed out (-110) --
 * which presented as "the TPS65132 panel regulator does not answer", and in
 * turn parked the DSI and the whole display stack.
 */
#define GATE_IMP_IIC_WRAP(_id, _name, _parent, _shift)			\
	GATE_MTK_FLAGS(_id, _name, _parent, &imp_iic_wrap_cg_regs, _shift,			\
		&mtk_clk_gate_ops_setclr, CLK_OPS_PARENT_ENABLE)

static const struct mtk_gate imp_iic_wrap_c_clks[] = {
	GATE_IMP_IIC_WRAP(CLK_IMP_IIC_WRAP_C_AP_CLOCK_I2C3, "imp_iic_wrap_c_ap_clock_i2c3", "i2c_ck", 0),
	GATE_IMP_IIC_WRAP(CLK_IMP_IIC_WRAP_C_AP_CLOCK_I2C5, "imp_iic_wrap_c_ap_clock_i2c5", "i2c_ck", 1),
	GATE_IMP_IIC_WRAP(CLK_IMP_IIC_WRAP_C_AP_CLOCK_I2C6, "imp_iic_wrap_c_ap_clock_i2c6", "i2c_ck", 2),
};

static const struct mtk_gate imp_iic_wrap_w_clks[] = {
	GATE_IMP_IIC_WRAP(CLK_IMP_IIC_WRAP_W_AP_CLOCK_I2C0, "imp_iic_wrap_w_ap_clock_i2c0", "i2c_ck", 0),
	GATE_IMP_IIC_WRAP(CLK_IMP_IIC_WRAP_W_AP_CLOCK_I2C1, "imp_iic_wrap_w_ap_clock_i2c1", "i2c_ck", 1),
};

static const struct mtk_gate imp_iic_wrap_n_clks[] = {
	GATE_IMP_IIC_WRAP(CLK_IMP_IIC_WRAP_N_AP_CLOCK_I2C7, "imp_iic_wrap_n_ap_clock_i2c7", "i2c_ck", 0),
};

static const struct mtk_gate imp_iic_wrap_en_clks[] = {
	GATE_IMP_IIC_WRAP(CLK_IMP_IIC_WRAP_EN_AP_CLOCK_I2C2, "imp_iic_wrap_en_ap_clock_i2c2", "i2c_ck", 0),
	GATE_IMP_IIC_WRAP(CLK_IMP_IIC_WRAP_EN_AP_CLOCK_I2C4, "imp_iic_wrap_en_ap_clock_i2c4", "i2c_ck", 1),
	GATE_IMP_IIC_WRAP(CLK_IMP_IIC_WRAP_EN_AP_CLOCK_I2C8, "imp_iic_wrap_en_ap_clock_i2c8", "i2c_ck", 2),
	GATE_IMP_IIC_WRAP(CLK_IMP_IIC_WRAP_EN_AP_CLOCK_I2C9, "imp_iic_wrap_en_ap_clock_i2c9", "i2c_ck", 3),
};


static const struct mtk_clk_desc imp_iic_wrap_c_desc = {
	.clks = imp_iic_wrap_c_clks,
	.num_clks = ARRAY_SIZE(imp_iic_wrap_c_clks),
};

static const struct mtk_clk_desc imp_iic_wrap_w_desc = {
	.clks = imp_iic_wrap_w_clks,
	.num_clks = ARRAY_SIZE(imp_iic_wrap_w_clks),
};

static const struct mtk_clk_desc imp_iic_wrap_n_desc = {
	.clks = imp_iic_wrap_n_clks,
	.num_clks = ARRAY_SIZE(imp_iic_wrap_n_clks),
};

static const struct mtk_clk_desc imp_iic_wrap_en_desc = {
	.clks = imp_iic_wrap_en_clks,
	.num_clks = ARRAY_SIZE(imp_iic_wrap_en_clks),
};

static const struct of_device_id of_match_clk_mt6789_imp_iic_wrap[] = {
	{ .compatible = "mediatek,mt6789-imp-iic-wrap-c", .data = &imp_iic_wrap_c_desc },
	{ .compatible = "mediatek,mt6789-imp-iic-wrap-w", .data = &imp_iic_wrap_w_desc },
	{ .compatible = "mediatek,mt6789-imp-iic-wrap-n", .data = &imp_iic_wrap_n_desc },
	{ .compatible = "mediatek,mt6789-imp-iic-wrap-en", .data = &imp_iic_wrap_en_desc },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_match_clk_mt6789_imp_iic_wrap);

static struct platform_driver clk_mt6789_imp_iic_wrap_drv = {
	.probe = mtk_clk_simple_probe,
	.remove = mtk_clk_simple_remove,
	.driver = {
		.name = "clk-mt6789-imp_iic_wrap",
		.of_match_table = of_match_clk_mt6789_imp_iic_wrap,
	},
};

module_platform_driver(clk_mt6789_imp_iic_wrap_drv);

MODULE_DESCRIPTION("MediaTek MT6789 I2C Wrapper clocks driver");
MODULE_LICENSE("GPL");
