// SPDX-License-Identifier: GPL-2.0
/*
 *  mt6789-mt6358.c  --  mt6789 mt6358 ALSA SoC machine driver
 *
 *  Copyright (c) 2021 MediaTek Inc.
 *  Author: Yujie Xiao <yujie.xiao@mediatek.com>
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "../common/mtk-afe-platform-driver.h"
#include "mt6789-afe-common.h"
#include "mt6789-afe-clk.h"
#include "mt6789-afe-gpio.h"
#include "../../codecs/mt6358.h"

#if IS_ENABLED(CONFIG_SND_SOC_MT6366_ACCDET)
#include "../../codecs/mt6358-accdet.h"
#endif

#if IS_ENABLED(CONFIG_CM_CUST_GPIOS_SUPPORT)
#include <mt-plat/cust_gpios.h>
#endif

/*
 * if need additional control for the ext spk amp that is connected
 * after Lineout Buffer / HP Buffer on the codec, put the control in
 * mt6789_mt6366_spk_amp_event()
 */
#define EXT_SPK_AMP_W_NAME "Ext_Speaker_Amp"

/*
 * Settling time between raising the amplifier enable pins and letting audio
 * through, taken from the factory sequence (enable rail -> pins high -> wait ->
 * audio). The RT9101 needs its enable pins stable before the buffers drive it;
 * shortening this is what turns a clean start into an audible pop.
 */
#define EXT_SPK_AMP_ENABLE_DELAY_MS	22

struct mt6789_mt6366_priv {
	struct regulator *extamp_reg;
	struct pinctrl *pinctrl;
	struct pinctrl_state *extamp_high;
	struct pinctrl_state *extamp_low;
	bool extamp_on;
};

/*
 * The DC-1's speakers hang off an RT9101 behind the codec's headphone buffers.
 * The amplifier takes its power from the PMIC VIBR rail (fixed 2.8 V in DT) and
 * is enabled by a coupled GPIO158/GPIO159 pair, exposed here as the
 * "extamp-pullhigh"/"extamp-pulllow" pinctrl states. Order matters in both
 * directions: rail up, then pins high, then wait, and on the way down pins low
 * before the rail, so the amplifier is never enabled into an unpowered or
 * collapsing supply.
 */
static int mt6789_mt6366_spk_amp_event(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *kcontrol,
					int event)
{
	struct snd_soc_card *card = snd_soc_dapm_to_card(w->dapm);
	struct mt6789_mt6366_priv *priv = snd_soc_card_get_drvdata(card);
	int ret;

	dev_dbg(card->dev, "%s(), event %d\n", __func__, event);

	if (!priv || !priv->extamp_reg || !priv->extamp_high || !priv->extamp_low)
		return 0;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		if (priv->extamp_on)
			return 0;

		ret = regulator_enable(priv->extamp_reg);
		if (ret) {
			dev_err(card->dev, "%s(), enable extamp supply failed: %d\n",
				__func__, ret);
			return ret;
		}

		ret = pinctrl_select_state(priv->pinctrl, priv->extamp_high);
		if (ret) {
			dev_err(card->dev, "%s(), extamp-pullhigh failed: %d\n",
				__func__, ret);
			regulator_disable(priv->extamp_reg);
			return ret;
		}

		msleep(EXT_SPK_AMP_ENABLE_DELAY_MS);
		priv->extamp_on = true;
		break;
	case SND_SOC_DAPM_PRE_PMD:
		if (!priv->extamp_on)
			return 0;

		ret = pinctrl_select_state(priv->pinctrl, priv->extamp_low);
		if (ret)
			dev_err(card->dev, "%s(), extamp-pulllow failed: %d\n",
				__func__, ret);

		regulator_disable(priv->extamp_reg);
		priv->extamp_on = false;
		break;
	default:
		break;
	}

	return 0;
}

static const struct snd_soc_dapm_widget mt6789_mt6366_widgets[] = {
	SND_SOC_DAPM_SPK(EXT_SPK_AMP_W_NAME, mt6789_mt6366_spk_amp_event),
};

static const struct snd_soc_dapm_route mt6789_mt6366_routes[] = {
	{EXT_SPK_AMP_W_NAME, NULL, "LINEOUT L"},
	{EXT_SPK_AMP_W_NAME, NULL, "Headphone L Ext Spk Amp"},
	{EXT_SPK_AMP_W_NAME, NULL, "Headphone R Ext Spk Amp"},
};

static const struct snd_kcontrol_new mt6789_mt6366_controls[] = {
	SOC_DAPM_PIN_SWITCH(EXT_SPK_AMP_W_NAME),
};


/*
 * define mtk_spk_i2s_mck node in dts when need mclk,
 * BE i2s need assign snd_soc_ops = mt6789_mt6366_i2s_ops
 */
static int mt6789_mt6366_i2s_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	unsigned int rate = params_rate(params);
	unsigned int mclk_fs_ratio = 128;
	unsigned int mclk_fs = rate * mclk_fs_ratio;
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);

	return snd_soc_dai_set_sysclk(cpu_dai,
				      0, mclk_fs, SND_SOC_CLOCK_OUT);
}

static const struct snd_soc_ops mt6789_mt6366_i2s_ops = {
	.hw_params = mt6789_mt6366_i2s_hw_params,
};

static int mt6789_mt6366_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_component *component =
		snd_soc_rtdcom_lookup(rtd, AFE_PCM_NAME);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(component);
	struct mt6789_afe_private *afe_priv = afe->platform_priv;
	struct snd_soc_component *codec_component =
		snd_soc_rtd_to_codec(rtd, 0)->component;
	struct snd_soc_dapm_context *dapm = snd_soc_card_to_dapm(rtd->card);

	/* set mtkaif protocol */
	mt6358_set_mtkaif_protocol(codec_component,
				   MT6358_MTKAIF_PROTOCOL_1);
	afe_priv->mtkaif_protocol = MT6358_MTKAIF_PROTOCOL_1;

	/* disable ext amp connection */
	snd_soc_dapm_disable_pin(dapm, EXT_SPK_AMP_W_NAME);
	return 0;
}

static int mt6789_i2s_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
				      struct snd_pcm_hw_params *params)
{
	dev_info(rtd->dev, "%s(), fix format to 32bit\n", __func__);

	/* fix BE i2s format to 32bit, clean param mask first */
	snd_mask_reset_range(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT),
			     0, SNDRV_PCM_FORMAT_LAST);

	params_set_format(params, SNDRV_PCM_FORMAT_S32_LE);
	return 0;
}

#if IS_ENABLED(CONFIG_MTK_VOW_SUPPORT) && !defined(CONFIG_FPGA_EARLY_PORTING)
#if !defined(SKIP_SB_VOW)
static const struct snd_pcm_hardware mt6789_mt6366_vow_hardware = {
	.info = (SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED |
		 SNDRV_PCM_INFO_MMAP_VALID),
	.period_bytes_min = 256,
	.period_bytes_max = 2 * 1024,
	.periods_min = 2,
	.periods_max = 4,
	.buffer_bytes_max = 2 * 2 * 1024,
};

static int mt6789_mt6366_vow_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_component *component =
			snd_soc_rtdcom_lookup(rtd, AFE_PCM_NAME);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(component);
	int i;

	dev_info(afe->dev, "%s(), start\n", __func__);
	snd_soc_set_runtime_hwparams(substream, &mt6789_mt6366_vow_hardware);

	mt6789_afe_gpio_request(afe, true, MT6789_DAI_VOW, 0);

	/* ASoC will call pm_runtime_get, but vow don't need */
	for_each_rtd_components(rtd, i, component) {
		pm_runtime_put_autosuspend(component->dev);
	}

	return 0;
}

static void mt6789_mt6366_vow_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_component *component =
			snd_soc_rtdcom_lookup(rtd, AFE_PCM_NAME);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(component);
	int i;

	dev_info(afe->dev, "%s(), end\n", __func__);
	mt6789_afe_gpio_request(afe, false, MT6789_DAI_VOW, 0);

	/* restore to fool ASoC */
	for_each_rtd_components(rtd, i, component) {
		pm_runtime_get_sync(component->dev);
	}
}

static const struct snd_soc_ops mt6789_mt6366_vow_ops = {
	.startup = mt6789_mt6366_vow_startup,
	.shutdown = mt6789_mt6366_vow_shutdown,
};
#endif  // #if IS_ENABLED(CONFIG_MTK_VOW_SUPPORT)
#endif

/* FE */
SND_SOC_DAILINK_DEFS(playback1,
	DAILINK_COMP_ARRAY(COMP_CPU("DL1")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(playback12,
	DAILINK_COMP_ARRAY(COMP_CPU("DL12")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(playback2,
	DAILINK_COMP_ARRAY(COMP_CPU("DL2")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(playback3,
	DAILINK_COMP_ARRAY(COMP_CPU("DL3")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(playback4,
	DAILINK_COMP_ARRAY(COMP_CPU("DL4")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(playback5,
	DAILINK_COMP_ARRAY(COMP_CPU("DL5")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(playback6,
	DAILINK_COMP_ARRAY(COMP_CPU("DL6")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(playback7,
	DAILINK_COMP_ARRAY(COMP_CPU("DL7")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(playback8,
	DAILINK_COMP_ARRAY(COMP_CPU("DL8")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(capture1,
	DAILINK_COMP_ARRAY(COMP_CPU("UL1")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(capture2,
	DAILINK_COMP_ARRAY(COMP_CPU("UL2")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(capture3,
	DAILINK_COMP_ARRAY(COMP_CPU("UL3")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(capture4,
	DAILINK_COMP_ARRAY(COMP_CPU("UL4")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(capture5,
	DAILINK_COMP_ARRAY(COMP_CPU("UL5")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(capture6,
	DAILINK_COMP_ARRAY(COMP_CPU("UL6")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(capture7,
	DAILINK_COMP_ARRAY(COMP_CPU("UL7")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(capture8,
	DAILINK_COMP_ARRAY(COMP_CPU("UL8")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(capture_mono_1,
	DAILINK_COMP_ARRAY(COMP_CPU("UL_MONO_1")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(capture_mono_2,
	DAILINK_COMP_ARRAY(COMP_CPU("UL_MONO_2")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(capture_mono_3,
	DAILINK_COMP_ARRAY(COMP_CPU("UL_MONO_3")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

/* hostless */
SND_SOC_DAILINK_DEFS(hostless_lpbk,
	DAILINK_COMP_ARRAY(COMP_CPU("Hostless LPBK DAI")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(hostless_fm,
	DAILINK_COMP_ARRAY(COMP_CPU("Hostless FM DAI")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(hostless_speech,
	DAILINK_COMP_ARRAY(COMP_CPU("Hostless Speech DAI")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(hostless_sph_echo_ref,
	DAILINK_COMP_ARRAY(COMP_CPU("Hostless_Sph_Echo_Ref_DAI")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(hostless_spk_init,
	DAILINK_COMP_ARRAY(COMP_CPU("Hostless_Spk_Init_DAI")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(hostless_adda_dl_i2s_out,
	DAILINK_COMP_ARRAY(COMP_CPU("Hostless_ADDA_DL_I2S_OUT DAI")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(hostless_src1,
	DAILINK_COMP_ARRAY(COMP_CPU("Hostless_SRC_1_DAI")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(hostless_src_bargein,
	DAILINK_COMP_ARRAY(COMP_CPU("Hostless_SRC_Bargein_DAI")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

/* BE */
SND_SOC_DAILINK_DEFS(adda,
	DAILINK_COMP_ARRAY(COMP_CPU("ADDA")),
	DAILINK_COMP_ARRAY(COMP_CODEC("mt6358-sound",
				      "mt6358-snd-codec-aif1")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(ap_dmic,
	DAILINK_COMP_ARRAY(COMP_CPU("AP_DMIC")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(i2s0,
	DAILINK_COMP_ARRAY(COMP_CPU("I2S0")),
#if IS_ENABLED(CONFIG_SND_SOC_AW882XX)
	DAILINK_COMP_ARRAY(COMP_CODEC("aw882xx_smartpa.7-0034","aw882xx-aif-7-34"),
						COMP_CODEC("aw882xx_smartpa.7-0035","aw882xx-aif-7-35")),
#else
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
#endif
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(i2s1,
	DAILINK_COMP_ARRAY(COMP_CPU("I2S1")),
#if IS_ENABLED(CONFIG_SND_SOC_AW882XX)
	DAILINK_COMP_ARRAY(COMP_CODEC("aw882xx_smartpa.7-0034","aw882xx-aif-7-34"),
						COMP_CODEC("aw882xx_smartpa.7-0035","aw882xx-aif-7-35")),
#else
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
#endif
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(i2s2,
	DAILINK_COMP_ARRAY(COMP_CPU("I2S2")),
#if IS_ENABLED(CONFIG_SND_SOC_AW882XX)
	DAILINK_COMP_ARRAY(COMP_CODEC("aw882xx_smartpa.7-0034","aw882xx-aif-7-34"),
						COMP_CODEC("aw882xx_smartpa.7-0035","aw882xx-aif-7-35")),
#else
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
#endif

	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(i2s3,
	DAILINK_COMP_ARRAY(COMP_CPU("I2S3")),
#if IS_ENABLED(CONFIG_SND_SOC_AW882XX)
	DAILINK_COMP_ARRAY(COMP_CODEC("aw882xx_smartpa.7-0034","aw882xx-aif-7-34"),
						COMP_CODEC("aw882xx_smartpa.7-0035","aw882xx-aif-7-35")),
#else
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
#endif
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(hw_gain1,
	DAILINK_COMP_ARRAY(COMP_CPU("HW Gain 1")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(hw_gain2,
	DAILINK_COMP_ARRAY(COMP_CPU("HW Gain 2")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(hw_src1,
	DAILINK_COMP_ARRAY(COMP_CPU("HW_SRC_1")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(hw_src2,
	DAILINK_COMP_ARRAY(COMP_CPU("HW_SRC_2")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(connsys_i2s,
	DAILINK_COMP_ARRAY(COMP_CPU("CONNSYS_I2S")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(pcm2,
	DAILINK_COMP_ARRAY(COMP_CPU("PCM 2")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

/* hostless */
SND_SOC_DAILINK_DEFS(hostless_ul1,
	DAILINK_COMP_ARRAY(COMP_CPU("Hostless_UL1 DAI")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(hostless_ul2,
	DAILINK_COMP_ARRAY(COMP_CPU("Hostless_UL2 DAI")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(hostless_ul3,
	DAILINK_COMP_ARRAY(COMP_CPU("Hostless_UL3 DAI")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(hostless_ul6,
	DAILINK_COMP_ARRAY(COMP_CPU("Hostless_UL6 DAI")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(hostless_hw_gain_aaudio,
	DAILINK_COMP_ARRAY(COMP_CPU("Hostless HW Gain AAudio DAI")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(hostless_src_aaudio,
	DAILINK_COMP_ARRAY(COMP_CPU("Hostless SRC AAudio DAI")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
#if IS_ENABLED(CONFIG_SND_SOC_MTK_BTCVSD) && !defined(CONFIG_FPGA_EARLY_PORTING)
SND_SOC_DAILINK_DEFS(btcvsd,
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("18050000.mtk-btcvsd-snd")));
#endif
#if IS_ENABLED(CONFIG_MTK_VOW_SUPPORT)  && !defined(CONFIG_FPGA_EARLY_PORTING)
#if !defined(SKIP_SB_VOW)
SND_SOC_DAILINK_DEFS(vow,
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_CODEC(DEVICE_MT6358_NAME,
						"mt6358-snd-codec-vow")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
#endif
#endif
#if IS_ENABLED(CONFIG_MTK_ULTRASND_PROXIMITY) && !defined(CONFIG_FPGA_EARLY_PORTING)
SND_SOC_DAILINK_DEFS(ultra,
		DAILINK_COMP_ARRAY(COMP_DUMMY()),
		DAILINK_COMP_ARRAY(COMP_DUMMY()),
		DAILINK_COMP_ARRAY(COMP_PLATFORM("snd_scp_ultra")));
#endif

#if IS_ENABLED(CONFIG_MTK_SCP_AUDIO)
SND_SOC_DAILINK_DEFS(scpspkprocess,
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_DUMMY()));
#endif

#if IS_ENABLED(CONFIG_SND_SOC_AW882XX)
/**************** Awinic Start***********************/
struct snd_soc_dai_link_component awinic_codecs[] = {
	{
		.of_node = NULL,
		.dai_name = "aw882xx-aif-7-34",
		.name = "aw882xx_smartpa.7-0034",

	},
	{
		.of_node = NULL,
		.dai_name = "aw882xx-aif-7-35",
		.name = "aw882xx_smartpa.7-0035",

	},
};
/**************** Awinic End***********************/
#endif


static struct snd_soc_dai_link mt6789_mt6366_dai_links[] = {
	/* Front End DAI links */
	{
		.name = "Playback_1",
		.stream_name = "Playback_1",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		SND_SOC_DAILINK_REG(playback1),
	},
	{
		.name = "Playback_12",
		.stream_name = "Playback_12",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		SND_SOC_DAILINK_REG(playback12),
	},
	{
		.name = "Playback_2",
		.stream_name = "Playback_2",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		SND_SOC_DAILINK_REG(playback2),
	},
	{
		.name = "Playback_3",
		.stream_name = "Playback_3",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		SND_SOC_DAILINK_REG(playback3),
	},
	{
		.name = "Playback_4",
		.stream_name = "Playback_4",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		SND_SOC_DAILINK_REG(playback4),
	},
	{
		.name = "Playback_5",
		.stream_name = "Playback_5",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		SND_SOC_DAILINK_REG(playback5),
	},
	{
		.name = "Playback_6",
		.stream_name = "Playback_6",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		SND_SOC_DAILINK_REG(playback6),
	},
	{
		.name = "Playback_7",
		.stream_name = "Playback_7",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		SND_SOC_DAILINK_REG(playback7),
	},
	{
		.name = "Playback_8",
		.stream_name = "Playback_8",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		SND_SOC_DAILINK_REG(playback8),
	},
	{
		.name = "Capture_1",
		.stream_name = "Capture_1",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		SND_SOC_DAILINK_REG(capture1),
	},
	{
		.name = "Capture_2",
		.stream_name = "Capture_2",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		SND_SOC_DAILINK_REG(capture2),
	},
	{
		.name = "Capture_3",
		.stream_name = "Capture_3",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		SND_SOC_DAILINK_REG(capture3),
	},
	{
		.name = "Capture_4",
		.stream_name = "Capture_4",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		SND_SOC_DAILINK_REG(capture4),
	},
	{
		.name = "Capture_5",
		.stream_name = "Capture_5",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		SND_SOC_DAILINK_REG(capture5),
	},
	{
		.name = "Capture_6",
		.stream_name = "Capture_6",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		SND_SOC_DAILINK_REG(capture6),
	},
	{
		.name = "Capture_7",
		.stream_name = "Capture_7",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		SND_SOC_DAILINK_REG(capture7),
	},
	{
		.name = "Capture_8",
		.stream_name = "Capture_8",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		SND_SOC_DAILINK_REG(capture8),
	},
	{
		.name = "Capture_Mono_1",
		.stream_name = "Capture_Mono_1",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		SND_SOC_DAILINK_REG(capture_mono_1),
	},
	{
		.name = "Capture_Mono_2",
		.stream_name = "Capture_Mono_2",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		SND_SOC_DAILINK_REG(capture_mono_2),
	},
	{
		.name = "Capture_Mono_3",
		.stream_name = "Capture_Mono_3",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		SND_SOC_DAILINK_REG(capture_mono_3),
	},
	{
		.name = "Hostless_LPBK",
		.stream_name = "Hostless_LPBK",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(hostless_lpbk),
	},
	{
		.name = "Hostless_FM",
		.stream_name = "Hostless_FM",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(hostless_fm),
	},
	{
		.name = "Hostless_Speech",
		.stream_name = "Hostless_Speech",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(hostless_speech),
	},
	{
		.name = "Hostless_Sph_Echo_Ref",
		.stream_name = "Hostless_Sph_Echo_Ref",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(hostless_sph_echo_ref),
	},
	{
		.name = "Hostless_Spk_Init",
		.stream_name = "Hostless_Spk_Init",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(hostless_spk_init),
	},
	{
		.name = "Hostless_ADDA_DL_I2S_OUT",
		.stream_name = "Hostless_ADDA_DL_I2S_OUT",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(hostless_adda_dl_i2s_out),
	},
	{
		.name = "Hostless_SRC_1",
		.stream_name = "Hostless_SRC_1",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(hostless_src1),
	},
	{
		.name = "Hostless_SRC_Bargein",
		.stream_name = "Hostless_SRC_Bargein",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(hostless_src_bargein),
	},
	/* Back End DAI links */
	{
		.name = "Primary Codec",
		.no_pcm = 1,
		.ignore_suspend = 1,
		.init = mt6789_mt6366_init,
		SND_SOC_DAILINK_REG(adda),
	},
	{
		.name = "AP_DMIC",
		.no_pcm = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(ap_dmic),
	},
	{
		.name = "I2S0",
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBC_CFC
			| SND_SOC_DAIFMT_GATED,
		.ops = &mt6789_mt6366_i2s_ops,
#if IS_ENABLED(CONFIG_SND_SOC_AW882XX)
		.num_codecs = ARRAY_SIZE(awinic_codecs),
		.codecs = awinic_codecs,
#endif
		.no_pcm = 1,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.be_hw_params_fixup = mt6789_i2s_hw_params_fixup,
		SND_SOC_DAILINK_REG(i2s0),
	},
	{
		.name = "I2S1",
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBC_CFC
			| SND_SOC_DAIFMT_GATED,
		.ops = &mt6789_mt6366_i2s_ops,
#if IS_ENABLED(CONFIG_SND_SOC_AW882XX)
		.num_codecs = ARRAY_SIZE(awinic_codecs),
		.codecs = awinic_codecs,
#endif
		.no_pcm = 1,
		.ignore_suspend = 1,
		.be_hw_params_fixup = mt6789_i2s_hw_params_fixup,
		SND_SOC_DAILINK_REG(i2s1),
	},
	{
		.name = "I2S2",
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBC_CFC
			| SND_SOC_DAIFMT_GATED,
		.ops = &mt6789_mt6366_i2s_ops,
#if IS_ENABLED(CONFIG_SND_SOC_AW882XX)
		.num_codecs = ARRAY_SIZE(awinic_codecs),
		.codecs = awinic_codecs,
#endif
		.no_pcm = 1,
		.ignore_suspend = 1,
		.be_hw_params_fixup = mt6789_i2s_hw_params_fixup,
		SND_SOC_DAILINK_REG(i2s2),
	},
	{
		.name = "I2S3",
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBC_CFC
			| SND_SOC_DAIFMT_GATED,
		.ops = &mt6789_mt6366_i2s_ops,
#if IS_ENABLED(CONFIG_SND_SOC_AW882XX)
		.num_codecs = ARRAY_SIZE(awinic_codecs),
		.codecs = awinic_codecs,
#endif
		.no_pcm = 1,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.be_hw_params_fixup = mt6789_i2s_hw_params_fixup,
		SND_SOC_DAILINK_REG(i2s3),
	},
	{
		.name = "HW Gain 1",
		.no_pcm = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(hw_gain1),
	},
	{
		.name = "HW Gain 2",
		.no_pcm = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(hw_gain2),
	},
	{
		.name = "HW_SRC_1",
		.no_pcm = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(hw_src1),
	},
	{
		.name = "HW_SRC_2",
		.no_pcm = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(hw_src2),
	},
	{
		.name = "CONNSYS_I2S",
		.no_pcm = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(connsys_i2s),
	},
	{
		.name = "PCM 2",
		.no_pcm = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(pcm2),
	},
	/* dummy BE for ul memif to record from dl memif */
	{
		.name = "Hostless_UL1",
		.no_pcm = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(hostless_ul1),
	},
	{
		.name = "Hostless_UL2",
		.no_pcm = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(hostless_ul2),
	},
	{
		.name = "Hostless_UL3",
		.no_pcm = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(hostless_ul3),
	},
	{
		.name = "Hostless_UL6",
		.no_pcm = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(hostless_ul6),
	},
	{
		.name = "Hostless_HW_Gain_AAudio",
		.stream_name = "Hostless_HW_Gain_AAudio",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(hostless_hw_gain_aaudio),
	},
	{
		.name = "Hostless_SRC_AAudio",
		.stream_name = "Hostless_SRC_AAudio",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(hostless_src_aaudio),
	},
	/* BTCVSD */
#if IS_ENABLED(CONFIG_SND_SOC_MTK_BTCVSD) && !defined(CONFIG_FPGA_EARLY_PORTING)
	{
		.name = "BTCVSD",
		.stream_name = "BTCVSD",
		SND_SOC_DAILINK_REG(btcvsd),
	},
#endif
	/* VoW */
#if IS_ENABLED(CONFIG_MTK_VOW_SUPPORT) && !defined(CONFIG_FPGA_EARLY_PORTING)
#if !defined(SKIP_SB_VOW)
	{
		.name = "VOW_Capture",
		.stream_name = "VOW_Capture",
		.ignore_suspend = 1,
		.ops = &mt6789_mt6366_vow_ops,
		SND_SOC_DAILINK_REG(vow),
	},
#endif
#endif
#if IS_ENABLED(CONFIG_MTK_ULTRASND_PROXIMITY) && !defined(CONFIG_FPGA_EARLY_PORTING)
	{
		.name = "SCP_ULTRA_Playback",
		.stream_name = "SCP_ULTRA_Playback",
		SND_SOC_DAILINK_REG(ultra),
	},
#endif
#if IS_ENABLED(CONFIG_MTK_SCP_AUDIO)
	{
		.name = "SCP_SPK_Process",
		.stream_name = "SCP_SPK_Process",
		SND_SOC_DAILINK_REG(scpspkprocess),
	},
#endif
};

static struct snd_soc_card mt6789_mt6366_soc_card = {
	.name = "mt6789-mt6366",
	.owner = THIS_MODULE,
	.dai_link = mt6789_mt6366_dai_links,
	.num_links = ARRAY_SIZE(mt6789_mt6366_dai_links),

	.controls = mt6789_mt6366_controls,
	.num_controls = ARRAY_SIZE(mt6789_mt6366_controls),
	.dapm_widgets = mt6789_mt6366_widgets,
	.num_dapm_widgets = ARRAY_SIZE(mt6789_mt6366_widgets),
	.dapm_routes = mt6789_mt6366_routes,
	.num_dapm_routes = ARRAY_SIZE(mt6789_mt6366_routes),
};

#if IS_ENABLED(CONFIG_MTK_SCP_AUDIO)
int mtk_update_scp_audio_info(struct snd_soc_card *card,
			struct platform_device *pdev)
{
	struct snd_soc_dai_link *dai_link;
	int i = 0;

	/* find dai link of SCP_SPK_Process */
	for_each_card_prelinks(card, i, dai_link) {
		if (strcmp(dai_link->name, "SCP_SPK_Process") == 0) {
			dai_link->cpus->name = NULL;
			dai_link->cpus->dai_name = "audio_task_spk_process";
			dai_link->platforms->name = "snd_scp_audio";
			dai_link->platforms->dai_name = NULL;
			dev_info(&pdev->dev, "scp audio updated\n");
		}
	}

	return 0;
}
#endif

/*
 * Look up the amplifier rail and the two pinctrl states it is gated by. Every
 * piece is optional: a board that wires no external amplifier (or a DT that has
 * not described one yet) still gets a working card, it just leaves the
 * Ext_Speaker_Amp widget as the no-op it was before.
 */
static int mt6789_mt6366_extamp_init(struct platform_device *pdev,
				     struct snd_soc_card *card)
{
	struct mt6789_mt6366_priv *priv;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	snd_soc_card_set_drvdata(card, priv);

	priv->extamp_reg = devm_regulator_get_optional(&pdev->dev, "extamp");
	if (IS_ERR(priv->extamp_reg)) {
		int ret = PTR_ERR(priv->extamp_reg);

		priv->extamp_reg = NULL;
		if (ret == -EPROBE_DEFER)
			return ret;
		dev_info(&pdev->dev, "no extamp supply (%d), speakers stay off\n",
			 ret);
		return 0;
	}

	priv->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(priv->pinctrl)) {
		int ret = PTR_ERR(priv->pinctrl);

		priv->pinctrl = NULL;
		if (ret == -EPROBE_DEFER)
			return ret;
		dev_info(&pdev->dev, "no pinctrl (%d), speakers stay off\n", ret);
		return 0;
	}

	priv->extamp_high = pinctrl_lookup_state(priv->pinctrl,
						 "extamp-pullhigh");
	priv->extamp_low = pinctrl_lookup_state(priv->pinctrl, "extamp-pulllow");
	if (IS_ERR(priv->extamp_high) || IS_ERR(priv->extamp_low)) {
		dev_info(&pdev->dev,
			 "no extamp pinctrl states, speakers stay off\n");
		priv->extamp_high = NULL;
		priv->extamp_low = NULL;
		return 0;
	}

	/*
	 * Leave the amplifier down until DAPM asks for it: "default" is the
	 * pulled-low state, and pinctrl has already applied it by now.
	 */
	return 0;
}

static int mt6789_mt6366_dev_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &mt6789_mt6366_soc_card;
	struct device_node *platform_node, *spk_node;
	int ret, i;
	struct snd_soc_dai_link *dai_link;
#if IS_ENABLED(CONFIG_MTK_SCP_AUDIO)
	struct device_node *scp_audio_node;
	int spkProcessEnable = 0;
#endif

	dev_info(&pdev->dev, "%s()\n", __func__);

	/* get platform node */
	platform_node = of_parse_phandle(pdev->dev.of_node,
					 "mediatek,platform", 0);
	if (!platform_node) {
		dev_err(&pdev->dev, "Property 'platform' missing or invalid\n");
		return -EINVAL;
	}

	/* get speaker codec node */
	spk_node = of_get_child_by_name(pdev->dev.of_node,
					"mediatek,speaker-codec");
	if (!spk_node) {
		dev_err(&pdev->dev,
			"spk_node of_get_child_by_name fail\n");
		return -EINVAL;
	}

	for_each_card_prelinks(card, i, dai_link) {
		if (!dai_link->platforms->name)
			dai_link->platforms->of_node = platform_node;

		if (!strcmp(dai_link->name, "Speaker Codec")) {
			ret = snd_soc_of_get_dai_link_codecs(
						&pdev->dev, spk_node, dai_link);
			if (ret < 0) {
				dev_err(&pdev->dev,
					"Speaker Codec get_dai_link fail: %d\n", ret);
				return -EINVAL;
			}
		} else if (!strcmp(dai_link->name, "Speaker Codec Ref")) {
			ret = snd_soc_of_get_dai_link_codecs(
						&pdev->dev, spk_node, dai_link);
			if (ret < 0) {
				dev_err(&pdev->dev,
					"Speaker Codec Ref get_dai_link fail: %d\n", ret);
				return -EINVAL;
			}
		}
	}

#if IS_ENABLED(CONFIG_MTK_SCP_AUDIO)
	/* get scp audio node */
	scp_audio_node = of_parse_phandle(pdev->dev.of_node,
					 "mediatek,scp-audio", 0);
	if (scp_audio_node) {
		dev_err(&pdev->dev, "got scp audio node\n");

		ret = of_property_read_u32(scp_audio_node,
					   "scp_spk_process_enable",
					   &spkProcessEnable);
		if (ret != 0) {
			pr_info("%s cannot get spkProcessEnable\n", __func__);
			spkProcessEnable = 0;
		}
		if (spkProcessEnable)
			mtk_update_scp_audio_info(card, pdev);
		pr_info("%s spkProcessEnable %d\n", __func__, spkProcessEnable);
	} else {
		dev_err(&pdev->dev, "can't find scp audio node\n");
	}
#endif

	card->dev = &pdev->dev;

	ret = mt6789_mt6366_extamp_init(pdev, card);
	if (ret)
		return ret;

	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret)
		dev_err(&pdev->dev, "%s snd_soc_register_card fail %d\n",
			__func__, ret);

	return ret;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id mt6789_mt6366_dt_match[] = {
	{.compatible = "mediatek,mt6789-mt6366-sound",},
	{}
};
#endif

static const struct dev_pm_ops mt6789_mt6366_pm_ops = {
	.poweroff = snd_soc_poweroff,
	.restore = snd_soc_resume,
};

static struct platform_driver mt6789_mt6366_driver = {
	.driver = {
		.name = "mt6789-mt6366",
#if IS_ENABLED(CONFIG_OF)
		.of_match_table = mt6789_mt6366_dt_match,
#endif
		.pm = &mt6789_mt6366_pm_ops,
	},
	.probe = mt6789_mt6366_dev_probe,
};

module_platform_driver(mt6789_mt6366_driver);

/* Module information */
MODULE_DESCRIPTION("MT6789 MT6366 ALSA SoC machine driver");
MODULE_AUTHOR("Yujie Xiao <yujie.xiao@mediatek.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("mt6789 mt6366 soc card");
