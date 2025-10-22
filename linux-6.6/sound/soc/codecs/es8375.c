// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Based on code by Du lianghan
 * Copyright (C) 2025 Everest Semiconductor Co., Ltd
 */

#include <linux/gpio/consumer.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/tlv.h>
#include <sound/soc.h>
#include <linux/acpi.h>
#include "es8375.h"

#if IS_ENABLED(CONFIG_SND_SOC_SPACEMIT)
#define SPACEMIT_CONFIG_CODEC_ES8375
#endif

/* codec private data */

struct es8375_priv {
	struct snd_soc_component *codec;
	struct regmap *regmap;
	struct clk *mclk;
	unsigned long  mclk_freq;
	int mastermode;
	bool sclkinv;
	bool mclkinv;
	bool dmic_enable;
	u8 dmic_pol;
	u8 mclk_src;
	u8 vdda;
	u8 vddd;
	enum snd_soc_bias_level bias_level;

	struct gpio_desc *pa_ctl_gpio;

	u8 stream_status;
};

struct snd_soc_component *es8375_RegMap_codec;

/*
static int es8375_set_gpio(struct es8375_priv *es8375, bool level)
{
	printk("enter into %s, level = %d\n", __func__, level);
	gpiod_set_value(es8375->pa_ctl_gpio, level);

	return 0;
}
*/

static const DECLARE_TLV_DB_SCALE(es8375_adc_osr_gain_tlv, -3100, 100, 0);
static const DECLARE_TLV_DB_SCALE(es8375_adc_volume_tlv, -9550, 50, 0);
static const DECLARE_TLV_DB_SCALE(es8375_adc_automute_attn_tlv, 0, 100, 0);
static const DECLARE_TLV_DB_SCALE(es8375_dac_volume_tlv, -9550, 50, 0);
static const DECLARE_TLV_DB_SCALE(es8375_dac_vppscale_tlv, -388, 12, 0);
static const DECLARE_TLV_DB_SCALE(es8375_dac_automute_attn_tlv, 0, 400, 0);

static const char *const es8375_dmic_gain_txt[] = {
	"x1",
	"x2",
	"x4",
	"x8",
};
static SOC_ENUM_SINGLE_DECL(es8375_dmic_gain, ES8375_ADC1_0x17,
		DMIC_GAIN_SHIFT_2, es8375_dmic_gain_txt);

static const char *const es8375_ramprate_txt[] = {
	"0.125dB/LRCK",
	"0.125dB/2LRCK",
	"0.125dB/4LRCK",
	"0.125dB/8LRCK",
	"0.125dB/16LRCK",
	"0.125dB/32LRCK",
	"0.125dB/64LRCK",
	"0.125dB/128LRCK",
	"disable softramp",
	"disable softramp",
	"disable softramp",
	"disable softramp",
	"disable softramp",
	"disable softramp",
	"disable softramp",
	"disable softramp",
};
static SOC_ENUM_SINGLE_DECL(es8375_adc_ramprate, ES8375_ADC2_0x18,
		ADC_RAMPRATE_SHIFT_0, es8375_ramprate_txt);
static SOC_ENUM_SINGLE_DECL(es8375_dac_ramprate, ES8375_DAC2_0x20,
		DAC_RAMPRATE_SHIFT_0, es8375_ramprate_txt);

static const char *const es8375_automute_ws_txt[] = {
	"256 samples",
	"512 samples",
	"1024 samples",
	"2048 samples",
	"4096 samples",
	"8192 samples",
	"16384 samples",
	"32768 samples",
};
static SOC_ENUM_SINGLE_DECL(es8375_adc_automute_ws, ES8375_ADC_AUTOMUTE_0x1B,
		ADC_AUTOMUTE_WS_SHIFT_3, es8375_automute_ws_txt);
static SOC_ENUM_SINGLE_DECL(es8375_dac_automute_ws, ES8375_DAC_AUTOMUTE_0x24,
		DAC_AUTOMUTE_WS_SHIFT_5, es8375_automute_ws_txt);

static const char *const es8375_adc_automute_ng_txt[] = {
	"-96dB (default)",
	"-90dB",
	"-84dB",
	"-78dB",
	"-72dB",
	"-66dB",
	"-60dB",
	"-54dB",
};
static SOC_ENUM_SINGLE_DECL(es8375_adc_automute_ng, ES8375_ADC_AUTOMUTE_0x1B,
		ADC_AUTOMUTE_NG_SHIFT_0, es8375_adc_automute_ng_txt);

static const char *const es8375_dac_automute_ng_txt[] = {
	"-144dB (default)",
	"-90dB",
	"-84dB",
	"-78dB",
	"-72dB",
	"-66dB",
	"-60dB",
	"-54dB",
};
static SOC_ENUM_SINGLE_DECL(es8375_dac_automute_ng, ES8375_DAC_AUTOMUTE1_0x23,
		DAC_AUTOMUTE_NG_SHIFT_0, es8375_dac_automute_ng_txt);

static const char *const es8375_adc_src_txt[] = {
	"ADC data source select as AMIC",
	"ADC data source select as DMICL",
};
static SOC_ENUM_SINGLE_DECL(es8375_adc_src, ES8375_ADC1_0x17,
		ADC_SRC_SHIFT_7, es8375_adc_src_txt);

static const char *const es8375_dmic_pol_txt[] = {
	"Low = Left Channel ; High = Right Channel. (default)",
	"Low = Right Channel ; High = Left Channel.",
};
static SOC_ENUM_SINGLE_DECL(es8375_dmic_pol, ES8375_ADC1_0x17,
		DMIC_POL_SHIFT_4, es8375_dmic_pol_txt);


static const char *const es8375_adc_hpf_txt[] = {
	"Freeze Offset",
	"Dynamic HPF (default)",
};
static SOC_ENUM_SINGLE_DECL(es8375_adc_hpf, ES8375_HPF1_0x1D,
		ADC_HPF_SHIFT_5, es8375_adc_hpf_txt);


static const struct snd_kcontrol_new es8375_snd_controls[] = {
	/* Capture Path */
	SOC_SINGLE_TLV("ADC OSR GAIN", ES8375_ADC_OSR_GAIN_0x19,
			ADC_OSR_GAIN_SHIFT_0, ES8375_ADC_OSR_GAIN_MAX, 0,
			es8375_adc_osr_gain_tlv),
	SOC_ENUM("ADC Source Select", es8375_adc_src),
	SOC_SINGLE("ADC Invert", ES8375_ADC1_0x17, ADC_INV_SHIFT_6, 1, 0),
	SOC_SINGLE("ADC RAM Clear", ES8375_ADC1_0x17, ADC_RAMCLR_SHIFT_5, 1, 0),
	SOC_ENUM("DMIC Polarity", es8375_dmic_pol),
	SOC_ENUM("DMIC Gain Select", es8375_dmic_gain),
	SOC_ENUM("ADC Ramp Rate", es8375_adc_ramprate),
	SOC_SINGLE_TLV("ADC Volume", ES8375_ADC_VOLUME_0x1A,
			ADC_VOLUME_SHIFT_0, ES8375_ADC_VOLUME_MAX,
			0, es8375_adc_volume_tlv),
	SOC_SINGLE("ADC Automute Enable", ES8375_ADC_AUTOMUTE_0x1B,
			ADC_AUTOMUTE_SHIFT_7, 1, 0),
	SOC_ENUM("ADC Automute Winsize", es8375_adc_automute_ws),
	SOC_ENUM("ADC Automute Noise Gate", es8375_adc_automute_ng),
	SOC_SINGLE_TLV("ADC Automute Attenuation", ES8375_ADC_AUTOMUTE_ATTN_0x1C,
			ADC_AUTOMUTE_ATTN_SHIFT_0, ES8375_ADC_AUTOMUTE_ATTN_MAX,
			0, es8375_adc_automute_attn_tlv),
	SOC_ENUM("ADC HPF", es8375_adc_hpf),

	/* Playback Path */
	SOC_SINGLE("DAC DSM Mute", ES8375_DAC1_0x1F, DAC_DSMMUTE_SHIFT_7, 1, 0),
	SOC_SINGLE("DAC DEM Mute", ES8375_DAC1_0x1F, DAC_DEMMUTE_SHIFT_6, 1, 0),
	SOC_SINGLE("DAC Invert", ES8375_DAC1_0x1F, DAC_INV_SHIFT_5, 1, 0),
	SOC_SINGLE("DAC RAM Clear", ES8375_DAC1_0x1F, DAC_RAMCLR_SHIFT_4, 1, 0),
	SOC_ENUM("DAC Ramp Rate", es8375_dac_ramprate),
	SOC_SINGLE_TLV("DAC Volume", ES8375_DAC_VOLUME_0x21,
			DAC_VOLUME_SHIFT_0, ES8375_DAC_VOLUME_MAX,
			0, es8375_dac_volume_tlv),
	SOC_SINGLE_TLV("DAC VPP Scale", ES8375_DAC_VPPSCALE_0x22,
			DAC_VPPSCALE_SHIFT_0, ES8375_DAC_VPPSCALE_MAX,
			0, es8375_dac_vppscale_tlv),
	SOC_SINGLE("DAC Automute Enable", ES8375_DAC_AUTOMUTE1_0x23,
			DAC_AUTOMUTE_EN_SHIFT_7, 1, 0),
	SOC_ENUM("DAC Automute Noise Gate", es8375_dac_automute_ng),
	SOC_ENUM("DAC Automute Winsize", es8375_dac_automute_ws),
	SOC_SINGLE_TLV("DAC Automute Attenuation", ES8375_DAC_AUTOMUTE_0x24,
			DAC_AUTOMUTE_ATTN_SHIFT_0, ES8375_DAC_AUTOMUTE_ATTN_MAX,
			0, es8375_dac_automute_attn_tlv),
};

static const struct snd_soc_dapm_widget es8375_dapm_widgets[] = {

	/* Capture Path */
	SND_SOC_DAPM_INPUT("MIC1"),
	SND_SOC_DAPM_PGA("PGA", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_ADC("Mono ADC", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("AIF1TX", "AIF1 Capture", 0, ES8375_SDP2_0x16,
			ES8375_ADC_P2S_MUTE_SHIFT_5, 1),

	/* Playback Path */
	SND_SOC_DAPM_AIF_IN("AIF1RX", "AIF1 Playback", 0, ES8375_SDP_0x15,
			ES8375_DAC_S2P_MUTE_SHIFT_6, 1),
	SND_SOC_DAPM_DAC("Mono DAC", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUTPUT("OUT"),

};

static const struct snd_soc_dapm_route es8375_dapm_routes[] = {

	/* Capture Path */
	{ "PGA", NULL, "MIC1" },
	{ "Mono ADC", NULL, "PGA" },
	{ "AIF1TX", NULL, "Mono ADC" },

	/* Playback Path */
	{ "Mono DAC", NULL, "AIF1RX" },
	{ "OUT", NULL, "Mono DAC" },

};

struct _coeff_div {
	u16 mclk_lrck_ratio;
	u32 mclk;
	u32 rate;
	u8 Reg0x04;
	u8 Reg0x05;
	u8 Reg0x06;
	u8 Reg0x07;
	u8 Reg0x08;
	u8 Reg0x09;
	u8 Reg0x0A;
	u8 Reg0x0B;
	u8 Reg0x19;
	u8 dvdd_vol;
	u8 dmic_sel;
};

static const struct _coeff_div coeff_div[] = {
	{32,	256000,		8000,	0x05,	0x34,	0xDD,	0x55,	0x1F,	0x00,	0x95,	0x00,	0x1F,	2,	2},
	{32,	512000,		16000,	0x05,	0x34,	0xDD,	0x55,	0x1F,	0x00,	0x94,	0x00,	0x1F,	2,	2},
	{32,	1536000,	48000,	0x05,	0x33,	0xD5,	0x55,	0x1F,	0x00,	0x93,	0x00,	0x1F,	2,	2},
	{36,	288000,		8000,	0x05,	0x34,	0xDD,	0x55,	0x23,	0x08,	0x95,	0x00,	0x1F,	2,	2},
	{36,	576000,		16000,	0x05,	0x34,	0xDD,	0x55,	0x23,	0x08,	0x94,	0x00,	0x1F,	2,	2},
	{36,	1728000,	48000,	0x05,	0x33,	0xD5,	0x55,	0x23,	0x08,	0x93,	0x00,	0x1F,	2,	2},
	{48,	384000,		8000,	0x05,	0x14,	0x5D,	0x55,	0x17,	0x20,	0x94,	0x00,	0x28,	2,	2},
	{48,	768000,		16000,	0x05,	0x14,	0x5D,	0x55,	0x17,	0x20,	0x94,	0x00,	0x28,	2,	2},
	{48,	2304000,	48000,	0x05,	0x11,	0x53,	0x55,	0x17,	0x20,	0x92,	0x00,	0x28,	2,	2},
	{50,	400000,		8000,	0x05,	0x14,	0x5D,	0x55,	0x18,	0x24,	0x94,	0x00,	0x27,	2,	2},
	{50,	800000,		16000,	0x05,	0x14,	0x5D,	0x55,	0x18,	0x24,	0x94,	0x00,	0x27,	2,	2},
	{50,	2400000,	48000,	0x05,	0x11,	0x53,	0x55,	0x18,	0x24,	0x92,	0x00,	0x27,	2,	2},
	{64,	512000,		8000,	0x05,	0x14,	0x5D,	0x33,	0x1F,	0x00,	0x94,	0x00,	0x1F,	2,	2},
	{64,	1024000,	16000,	0x05,	0x13,	0x55,	0x33,	0x1F,	0x00,	0x93,	0x00,	0x1F,	2,	2},
	{64,	3072000,	48000,	0x05,	0x11,	0x53,	0x33,	0x1F,	0x00,	0x92,	0x00,	0x1F,	2,	2},
	{72,	576000,		8000,	0x05,	0x14,	0x5D,	0x33,	0x23,	0x08,	0x94,	0x00,	0x1F,	2,	2},
	{72,	1152000,	16000,	0x05,	0x13,	0x55,	0x33,	0x23,	0x08,	0x93,	0x00,	0x1F,	2,	2},
	{72,	3456000,	48000,	0x05,	0x11,	0x53,	0x33,	0x23,	0x08,	0x92,	0x00,	0x1F,	2,	2},
	{96,	768000,		8000,	0x15,	0x34,	0xDD,	0x55,	0x1F,	0x00,	0x94,	0x00,	0x1F,	2,	2},
	{96,	1536000,	16000,	0x15,	0x34,	0xDD,	0x55,	0x1F,	0x00,	0x93,	0x00,	0x1F,	2,	2},
	{96,	4608000,	48000,	0x15,	0x33,	0xD5,	0x55,	0x1F,	0x00,	0x92,	0x00,	0x1F,	2,	2},
	{100,	800000,		8000,	0x05,	0x03,	0x35,	0x33,	0x18,	0x24,	0x94,	0x00,	0x27,	2,	2},
	{100,	1600000,	16000,	0x05,	0x03,	0x35,	0x33,	0x18,	0x24,	0x93,	0x00,	0x27,	2,	2},
	{100,	4800000,	48000,	0x03,	0x00,	0x31,	0x33,	0x18,	0x24,	0x92,	0x00,	0x27,	2,	2},
	{128,	1024000,	8000,	0x05,	0x03,	0x35,	0x11,	0x1F,	0x00,	0x93,	0x01,	0x1F,	2,	2},
	{128,	2048000,	16000,	0x03,	0x01,	0x33,	0x11,	0x1F,	0x00,	0x92,	0x01,	0x1F,	2,	2},
	{128,	6144000,	48000,	0x03,	0x00,	0x31,	0x11,	0x1F,	0x00,	0x92,	0x01,	0x1F,	2,	2},
	{144,	1152000,	8000,	0x05,	0x03,	0x35,	0x11,	0x23,	0x08,	0x93,	0x01,	0x1F,	2,	2},
	{144,	2304000,	16000,	0x03,	0x01,	0x33,	0x11,	0x23,	0x08,	0x92,	0x01,	0x1F,	2,	2},
	{144,	6912000,	48000,	0x03,	0x00,	0x31,	0x11,	0x23,	0x08,	0x92,	0x01,	0x1F,	2,	2},
	{192,	1536000,	8000,	0x15,	0x14,	0x5D,	0x33,	0x1F,	0x00,	0x93,	0x02,	0x1F,	2,	2},
	{192,	3072000,	16000,	0x15,	0x13,	0x55,	0x33,	0x1F,	0x00,	0x92,	0x02,	0x1F,	2,	2},
	{192,	9216000,	48000,	0x15,	0x11,	0x53,	0x33,	0x1F,	0x00,	0x92,	0x02,	0x1F,	2,	2},
	{250,	12000000,	48000,	0x25,	0x11,	0x53,	0x55,	0x18,	0x24,	0x92,	0x04,	0x27,	2,	2},
	{256,	2048000,	8000,	0x0D,	0x03,	0x35,	0x11,	0x1F,	0x00,	0x92,	0x03,	0x1F,	2,	2},
	{256,	4096000,	16000,	0x0B,	0x01,	0x33,	0x11,	0x1F,	0x00,	0x92,	0x03,	0x1F,	2,	2},
	{256,	12288000,	48000,	0x0B,	0x00,	0x31,	0x11,	0x1F,	0x00,	0x92,	0x03,	0x1F,	2,	2},
	{384,	3072000,	8000,	0x15,	0x03,	0x35,	0x11,	0x1F,	0x00,	0x92,	0x05,	0x1F,	2,	2},
	{384,	6144000,	16000,	0x13,	0x01,	0x33,	0x11,	0x1F,	0x00,	0x92,	0x05,	0x1F,	2,	2},
	{384,	18432000,	48000,	0x13,	0x00,	0x31,	0x11,	0x1F,	0x00,	0x92,	0x05,	0x1F,	2,	2},
	{400,	19200000,	48000,	0x1B,	0x00,	0x31,	0x33,	0x18,	0x24,	0x92,	0x04,	0x27,	2,	2},
	{500,	24000000,	48000,	0x23,	0x00,	0x31,	0x33,	0x18,	0x24,	0x92,	0x04,	0x27,	2,	2},
	{512,	4096000,	8000,	0x1D,	0x03,	0x35,	0x11,	0x1F,	0x00,	0x92,	0x07,	0x1F,	2,	2},
	{512,	8192000,	16000,	0x1B,	0x01,	0x33,	0x11,	0x1F,	0x00,	0x92,	0x07,	0x1F,	2,	2},
	{512,	24576000,	48000,	0x1B,	0x00,	0x31,	0x11,	0x1F,	0x00,	0x92,	0x07,	0x1F,	2,	2},
	{768,	6144000,	8000,	0x2D,	0x03,	0x35,	0x11,	0x1F,	0x00,	0x92,	0x0B,	0x1F,	2,	2},
	{768,	12288000,	16000,	0x2B,	0x01,	0x33,	0x11,	0x1F,	0x00,	0x92,	0x0B,	0x1F,	2,	2},
	{1024,	8192000,	8000,	0x3D,	0x03,	0x35,	0x11,	0x1F,	0x00,	0x92,	0x0F,	0x1F,	2,	2},
	{1024,	16384000,	16000,	0x3B,	0x01,	0x33,	0x11,	0x1F,	0x00,	0x92,	0x0F,	0x1F,	2,	2},
	{1152,	9216000,	8000,	0x45,	0x03,	0x35,	0x11,	0x1F,	0x00,	0x92,	0x0F,	0x1F,	2,	2},
	{1152,	18432000,	16000,	0x43,	0x01,	0x33,	0x11,	0x1F,	0x00,	0x92,	0x0F,	0x1F,	2,	2},
	{1200,	9600000,	8000,	0x5D,	0x03,	0x35,	0x33,	0x18,	0x24,	0x92,	0x11,	0x27,	2,	2},
	{1200,	19200000,	16000,	0x5D,	0x03,	0x35,	0x33,	0x18,	0x24,	0x92,	0x11,	0x27,	2,	2},
	{1536,	12288000,	8000,	0x5D,	0x03,	0x35,	0x11,	0x1F,	0x00,	0x92,	0x17,	0x1F,	2,	2},
	{1536,	24576000,	16000,	0x5B,	0x01,	0x33,	0x11,	0x1F,	0x00,	0x92,	0x17,	0x1F,	2,	2},
	{2048,	16384000,	8000,	0x7D,	0x03,	0x35,	0x11,	0x1F,	0x00,	0x92,	0x1F,	0x1F,	2,	2},
	{2304,	18432000,	8000,	0x8D,	0x03,	0x35,	0x11,	0x1F,	0x00,	0x92,	0x23,	0x1F,	2,	2},
	{2400,	19200000,	8000,	0xBD,	0x03,	0x35,	0x33,	0x18,	0x24,	0x92,	0x25,	0x27,	2,	2},
	{3072,	24576000,	8000,	0xBD,	0x03,	0x35,	0x11,	0x1F,	0x00,	0x92,	0x2F,	0x1F,	2,	2},
	{32,	3072000,	96000,	0x05,	0x11,	0x53,	0x55,	0x0F,	0x00,	0x92,	0x00,	0x37,	2,	2},
	{64,	6144000,	96000,	0x03,	0x00,	0x31,	0x33,	0x0F,	0x00,	0x92,	0x00,	0x37,	2,	2},
	{96,	9216000,	96000,	0x15,	0x11,	0x53,	0x55,	0x0F,	0x00,	0x92,	0x00,	0x37,	2,	2},
	{128,	12288000,	96000,	0x0B,	0x00,	0x31,	0x33,	0x0F,	0x00,	0x92,	0x01,	0x37,	2,	2},
};

static inline int get_coeff(u8 vddd, u8 dmic, int mclk, int rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(coeff_div); i++) {
		if (coeff_div[i].rate == rate && coeff_div[i].mclk == mclk) {
			if ((vddd == ES8375_1V8) && (dmic == 1) &&
			(coeff_div[i].dvdd_vol != 1) &&
			(coeff_div[i].dmic_sel != 0)) {
				return i;
			} else if ((vddd == ES8375_1V8) && (dmic == 0) &&
			(coeff_div[i].dvdd_vol != 1) &&
			(coeff_div[i].dmic_sel != 1)) {
				return i;
			} else if ((vddd == ES8375_3V3) && (dmic == 1) &&
			(coeff_div[i].dvdd_vol != 0) &&
			(coeff_div[i].dmic_sel != 0)) {
				return i;
			} else if ((vddd == ES8375_3V3) && (dmic == 0) &&
			(coeff_div[i].dvdd_vol != 0) &&
			(coeff_div[i].dmic_sel != 1)) {
				return i;
			}
		}
	}

	return -EINVAL;
}

static int es8375_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_component *codec = dai->component;
	struct es8375_priv *es8375 = snd_soc_component_get_drvdata(codec);
	int par_width = params_width(params);
	u16 iface = 0;
	int coeff;

	printk("Enter into %s()\n", __func__);

	if (es8375->mclk_src == ES8375_BCLK_PIN) {
		if (es8375->mastermode) {
			dev_err(codec->dev, "no mclk, cannot as master\n");
			return -EINVAL;
		}
		snd_soc_component_update_bits(codec,
			ES8375_MCLK_SEL_0x01, 0x80, 0x80);

		es8375->mclk_freq = 2 * (unsigned int)par_width * params_rate(params);
	}

	printk("%s, mclk = %lu, lrck = %u\n", __func__, es8375->mclk_freq,
		params_rate(params));

	coeff = get_coeff(es8375->vddd, es8375->dmic_enable,
		es8375->mclk_freq, params_rate(params));
	if (coeff < 0) {
		printk("Unable to configure sample rate %uHz with %luHz MCLK\n",
			params_rate(params), es8375->mclk_freq);
		return -EINVAL;
	}
	/*
	 * set clock parammeters
	 */
	snd_soc_component_write(codec, ES8375_CLK_MGR_0x04,
			coeff_div[coeff].Reg0x04);
	snd_soc_component_write(codec, ES8375_CLK_MGR_0x05,
			coeff_div[coeff].Reg0x05);
	snd_soc_component_write(codec, ES8375_CLK_MGR_0x06,
			coeff_div[coeff].Reg0x06);
	snd_soc_component_write(codec, ES8375_CLK_MGR_0x07,
			coeff_div[coeff].Reg0x07);
	snd_soc_component_write(codec, ES8375_CLK_MGR_0x08,
			coeff_div[coeff].Reg0x08);
	snd_soc_component_write(codec, ES8375_CLK_MGR_0x09,
			coeff_div[coeff].Reg0x09);
	snd_soc_component_write(codec, ES8375_CLK_MGR_0x0A,
			coeff_div[coeff].Reg0x0A);
	snd_soc_component_write(codec, ES8375_CLK_MGR_0x0B,
			coeff_div[coeff].Reg0x0B);
	snd_soc_component_write(codec, ES8375_ADC_OSR_GAIN_0x19,
			coeff_div[coeff].Reg0x19);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		iface |= 0x0c;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		iface |= 0x04;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		iface |= 0x10;
		break;
	}

	/* set iface */
	snd_soc_component_update_bits(codec, ES8375_SDP_0x15, 0x1c, iface);

	return 0;
}

static int es8375_set_sysclk(struct snd_soc_dai *dai, int clk_id,
		unsigned int freq, int dir)
{
	struct snd_soc_component *codec = dai->component;
	struct es8375_priv *es8375 = snd_soc_component_get_drvdata(codec);

	es8375->mclk_freq = freq;

	return 0;
}

static int es8375_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_component *codec = codec_dai->component;
	struct es8375_priv *es8375 = snd_soc_component_get_drvdata(codec);
	u8 iface = 0;
	u8 codeciface = 0;

	dev_dbg(codec->dev, "Enter into %s()\n", __func__);

	codeciface = snd_soc_component_read(codec, ES8375_SDP_0x15);

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:    /* MASTER MODE */
		es8375->mastermode = 1;
		dev_dbg(codec->dev, "ES8375 in Master mode\n");
		snd_soc_component_update_bits(codec, ES8375_RESET1_0x00,
				0x80, 0x80);
		break;
	case SND_SOC_DAIFMT_CBS_CFS:    /* SLAVE MODE */
		es8375->mastermode = 0;
		dev_dbg(codec->dev, "ES8375 in Slave mode\n");
		snd_soc_component_update_bits(codec, ES8375_RESET1_0x00,
				0x80, 0x00);
		break;
	default:
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		dev_dbg(codec->dev, "ES8375 in I2S Format\n");
		codeciface &= 0xFC;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		return -EINVAL;
	case SND_SOC_DAIFMT_LEFT_J:
		dev_dbg(codec->dev, "ES8375 in LJ Format\n");
		codeciface &= 0xFC;
		codeciface |= 0x01;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		dev_dbg(codec->dev, "ES8375 in DSP-A Format\n");
		codeciface &= 0xDC;
		codeciface |= 0x03;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		dev_dbg(codec->dev, "ES8375 in DSP-B Format\n");
		codeciface &= 0xDC;
		codeciface |= 0x23;
		break;
	default:
		return -EINVAL;
	}

	iface = snd_soc_component_read(codec, ES8375_CLK_MGR_0x03);

#ifdef SPACEMIT_CONFIG_CODEC_ES8375
	snd_soc_component_write(codec, ES8375_SDP_0x15, codeciface);
#endif

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		iface		&= 0xFE;
		codeciface	&= 0xDF;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		iface		|= 0x01;
		codeciface	|= 0x20;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		iface		|= 0x01;
		codeciface	&= 0xDF;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		iface		&= 0xFE;
		codeciface	|= 0x20;
		break;
	default:
		return -EINVAL;
	}

	snd_soc_component_write(codec, ES8375_CLK_MGR_0x03, iface);

#ifndef SPACEMIT_CONFIG_CODEC_ES8375
	snd_soc_component_write(codec, ES8375_SDP_0x15, codeciface);
#else
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_DSP_B:
		break;
	default:
		snd_soc_component_write(codec, ES8375_SDP_0x15, codeciface);
		break;
	}
#endif
	return 0;
}

static int es8375_set_bias_level(struct snd_soc_component *codec,
		enum snd_soc_bias_level level)
{
	struct es8375_priv *es8375 = snd_soc_component_get_drvdata(codec);
	int ret;

	switch (level) {
	case SND_SOC_BIAS_ON:
		printk("%s on\n", __func__);
		ret = clk_prepare_enable(es8375->mclk);
		if (ret) {
			dev_err(codec->dev, "unable to prepare mclk\n");
			return ret;
		}
		snd_soc_component_write(codec, ES8375_CSM1_0x0F, 0xA6);
		break;
	case SND_SOC_BIAS_PREPARE:
		printk("%s prepare\n", __func__);
		break;
	case SND_SOC_BIAS_STANDBY:
		printk("%s standby\n", __func__);
		snd_soc_component_write(codec, ES8375_CSM1_0x0F, 0x96);
		clk_disable_unprepare(es8375->mclk);
		break;
	case SND_SOC_BIAS_OFF:
		printk("%s off\n", __func__);
		/* power down analog to get minimum power consumption */
		snd_soc_component_write(codec, ES8375_CSM1_0x0F, 0x3C);
		snd_soc_component_write(codec, ES8375_CLK_MGR_0x03, 0x48);
		snd_soc_component_write(codec, ES8375_CSM2_0x10, 0x80);
		snd_soc_component_write(codec, ES8375_CSM1_0x0F, 0x3E);
		snd_soc_component_write(codec, ES8375_CLK_MGR_0x0A, 0x15);
		snd_soc_component_write(codec, ES8375_SYS_CTRL2_0xF9, 0x0C);
		snd_soc_component_write(codec, ES8375_RESET1_0x00, 0x00);
		break;
	}
	return 0;
}

static int es8375_mute(struct snd_soc_dai *dai, int mute, int stream)
{
	struct snd_soc_component *codec = dai->component;
	struct es8375_priv *es8375 = snd_soc_component_get_drvdata(codec);

	printk("Enter into %s(), mute = %d, stream_status = %d\n",
			__func__, mute, es8375->stream_status);

	return 0;
}

#define es8375_RATES SNDRV_PCM_RATE_8000_96000

#define es8375_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
		SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_S32_LE)

static const struct snd_soc_dai_ops es8375_ops = {
	.hw_params = es8375_hw_params,
	.mute_stream = es8375_mute,
	.set_sysclk = es8375_set_sysclk,
	.set_fmt = es8375_set_dai_fmt,
};

static struct snd_soc_dai_driver es8375_dai = {
	.name = "ES8375 HiFi",
	.playback = {
		.stream_name = "AIF1 Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = es8375_RATES,
		.formats = es8375_FORMATS,
	},
	.capture = {
		.stream_name = "AIF1 Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = es8375_RATES,
		.formats = es8375_FORMATS,
	},
	.ops = &es8375_ops,
	.symmetric_rate = 1,
};

static void es8375_init(struct snd_soc_component *codec)
{
	struct es8375_priv *es8375 = snd_soc_component_get_drvdata(codec);

	printk("Enter into %s()\n", __func__);

	snd_soc_component_write(codec, ES8375_CLK_MGR_0x0A, 0x95);
	snd_soc_component_write(codec, ES8375_CLK_MGR_0x03, 0x48);
	snd_soc_component_write(codec, ES8375_DIV_SPKCLK_0x0E, 0x18);
	snd_soc_component_write(codec, ES8375_CLK_MGR_0x04, 0x02);
	snd_soc_component_write(codec, ES8375_CLK_MGR_0x05, 0x05);
	snd_soc_component_write(codec, ES8375_CSM1_0x0F, 0x82);
	snd_soc_component_write(codec, ES8375_VMID_CHARGE2_T_0x11, 0x20);
	snd_soc_component_write(codec, ES8375_VMID_CHARGE3_T_0x12, 0x20);
	snd_soc_component_write(codec, ES8375_DAC_CAL_0x25, 0x28);
	snd_soc_component_write(codec, ES8375_ANALOG_SPK1_0x28, 0xFC);
	snd_soc_component_write(codec, ES8375_ANALOG_SPK2_0x29, 0xE0);
	snd_soc_component_write(codec, ES8375_VMID_SEL_0x2D, 0xFE);
	snd_soc_component_write(codec, ES8375_ANALOG_0x2E, 0xB8);
	snd_soc_component_write(codec, ES8375_SYS_CTRL2_0xF9, 0x03);
	snd_soc_component_write(codec, ES8375_CLK_MGR_0x02, 0x16);
	snd_soc_component_write(codec, ES8375_RESET1_0x00, 0x00);
	msleep(80);
	snd_soc_component_write(codec, ES8375_CLK_MGR_0x03, 0x00);
	snd_soc_component_write(codec, ES8375_CSM1_0x0F, 0x86);
	snd_soc_component_write(codec, ES8375_CLK_MGR_0x04, 0x0B);
	snd_soc_component_write(codec, ES8375_CLK_MGR_0x05, 0x00);
	snd_soc_component_write(codec, ES8375_CLK_MGR_0x06, 0x31);
	snd_soc_component_write(codec, ES8375_CLK_MGR_0x07, 0x11);
	snd_soc_component_write(codec, ES8375_CLK_MGR_0x08, 0x1F);
	snd_soc_component_write(codec, ES8375_CLK_MGR_0x09, 0x00);
	snd_soc_component_write(codec, ES8375_ADC_OSR_GAIN_0x19, 0x1F);
	snd_soc_component_write(codec, ES8375_ADC2_0x18, 0x00);
	snd_soc_component_write(codec, ES8375_DAC2_0x20, 0x00);
	snd_soc_component_write(codec, ES8375_ADC_VOLUME_0x1A, 0xBF);
	snd_soc_component_write(codec, ES8375_DAC_VOLUME_0x21, 0xBF);
	snd_soc_component_write(codec, ES8375_DAC_OTP_0x27, 0x88);
	snd_soc_component_write(codec, ES8375_ANALOG_SPK2_0x29, 0xE7);
	snd_soc_component_write(codec, ES8375_ANALOG_0x32, 0xF0);
	snd_soc_component_write(codec, ES8375_ANALOG_0x37, 0x40);
	snd_soc_component_write(codec, ES8375_CLK_MGR_0x02, 0xFE);

	es8375->stream_status = 0;
}

static int es8375_suspend(struct snd_soc_component *codec)
{
	return 0;
}

static int es8375_resume(struct snd_soc_component *codec)
{
	return 0;
}

static int es8375_codec_probe(struct snd_soc_component *codec)
{
	struct es8375_priv *es8375 = snd_soc_component_get_drvdata(codec);

	printk("Enter into %s()\n", __func__);

	es8375->codec = codec;
	es8375->mastermode = 0;

	es8375_RegMap_codec = codec;

	es8375_init(codec);

	return 0;
}

static bool es8375_writeable_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case ES8375_CHIP_VERSION_0xFF:
	case ES8375_CHIP_ID0_0xFE:
	case ES8375_CHIP_ID1_0xFD:
	case ES8375_SPK_OFFSET_0xFC:
	case ES8375_FLAGS2_0xFB:
		return false;
	default:
		return true;
	}
}

static struct regmap_config es8375_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = ES8375_REG_MAX,
	.cache_type = REGCACHE_RBTREE,
	.use_single_read = true,
	.use_single_write = true,
	.writeable_reg = es8375_writeable_register,
};

static struct snd_soc_component_driver es8375_codec_driver = {
	.probe = es8375_codec_probe,
	.suspend = es8375_suspend,
	.resume = es8375_resume,
	.set_bias_level = es8375_set_bias_level,
	.controls = es8375_snd_controls,
	.num_controls = ARRAY_SIZE(es8375_snd_controls),
	.dapm_widgets = es8375_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(es8375_dapm_widgets),
	.dapm_routes = es8375_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(es8375_dapm_routes),

	.idle_bias_on = 1,
	.suspend_bias_off = 1,
};

static int es8375_read_device_properities(struct device *dev, struct es8375_priv *es8375)
{
	int ret;

	ret = device_property_read_u8(dev, "everest,mclk-src", &es8375->mclk_src);
	if (ret != 0) {
		dev_dbg(dev, "mclk-src return %d", ret);
		es8375->mclk_src = ES8375_MCLK_SOURCE;
	}
	dev_dbg(dev, "mclk-src %x", es8375->mclk_src);

	ret = device_property_read_u8(dev, "everest,vddd-voltage", &es8375->vddd);
	if (ret != 0) {
		dev_dbg(dev, "vddd-voltage return %d", ret);
		es8375->vddd = ES8375_DVDD;
	}
	dev_dbg(dev, "vddd-voltage %x", es8375->vddd);

	es8375->dmic_enable = device_property_read_bool(dev, "everest,dmic-enabled");
	dev_dbg(dev, "dmic_enable %x", es8375->dmic_enable);

	ret = device_property_read_u8(dev, "everest,dmic-pol", &es8375->dmic_pol);
	if (ret != 0) {
		dev_dbg(dev, "dmic-pol return %d", ret);
		es8375->dmic_pol = DMIC_POL;
	}
	dev_dbg(dev, "dmic-pol %x", es8375->dmic_pol);

	es8375->mclkinv = device_property_read_bool(dev, "everest,mclk-inverted");
	dev_dbg(dev, "mclk-inverted %x", es8375->mclkinv);

	es8375->sclkinv = device_property_read_bool(dev, "everest,sclk-inverted");
	dev_dbg(dev, "sclk-inverted %x", es8375->sclkinv);

#ifndef SPACEMIT_CONFIG_CODEC_ES8375
	es8375->mclk = devm_clk_get(dev, "mclk");
	if (IS_ERR(es8375->mclk)) {
		dev_err(dev, "unable to get mclk\n");
		return PTR_ERR(es8375->mclk);
	}
	if (!es8375->mclk)
		dev_warn(dev, "assuming static mclk\n");

	ret = clk_prepare_enable(es8375->mclk);
	if (ret) {
		dev_err(dev, "unable to enable mclk\n");
		return ret;
	}
#endif
	return 0;
}

static u32 cur_reg = 0;

static ssize_t es8375_show(struct device *dev, struct device_attribute *attr, char *_buf)
{
	int ret;

	ret = sprintf(_buf, "%s(): get 0x%04x=0x%04x\n", __func__, cur_reg,
			snd_soc_component_read(es8375_RegMap_codec, cur_reg));
	return ret;
}

static ssize_t es8375_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int val = 0, flag = 0;
	u8 i = 0, reg, num, value_w, value_r;

	val = simple_strtol(buf, NULL, 16);
	flag = (val >> 16) & 0xFF;

	if (flag) {
		reg = (val >> 8) & 0xFF;
		value_w = val & 0xFF;
		printk("\nWrite: start REG:0x%02x,val:0x%02x,count:0x%02x\n",
				reg, value_w, flag);
		while (flag--) {
			snd_soc_component_write(es8375_RegMap_codec, reg, value_w);
			printk("Write 0x%02x to REG:0x%02x\n", value_w, reg);
			reg++;
		}
	} else {
		reg = (val >> 8) & 0xFF;
		num = val & 0xff;
		printk("\nRead: start REG:0x%02x,count:0x%02x\n", reg, num);
		do {
			value_r = 0;
			value_r = snd_soc_component_read(es8375_RegMap_codec, reg);
			printk("REG[0x%02x]: 0x%02x;  \n", reg, value_r);
			reg++;
			i++;
		} while (i < num);
	}

	return count;
}

static DEVICE_ATTR(es8375, 0664, es8375_show, es8375_store);

static struct attribute *es8375_debug_attrs[] = {
	&dev_attr_es8375.attr,
	NULL,
};

static struct attribute_group es8375_debug_attr_group = {
	.name   = "es8375_debug",
	.attrs  = es8375_debug_attrs,
};

static int es8375_i2c_probe(struct i2c_client *i2c_client)
{
	struct es8375_priv *es8375;
	struct device *dev = &i2c_client->dev;
	int ret = -1;
#ifndef SPACEMIT_CONFIG_CODEC_ES8375
	unsigned int val;
#endif

	printk("Enter into %s\n", __func__);
	es8375 = devm_kzalloc(&i2c_client->dev, sizeof(*es8375), GFP_KERNEL);
	if (!es8375)
		return -ENOMEM;

	es8375->regmap = devm_regmap_init_i2c(i2c_client,
			&es8375_regmap_config);
	if (IS_ERR(es8375->regmap)) {
		dev_err(&i2c_client->dev, "regmap_init() failed: %d\n", ret);
		return PTR_ERR(es8375->regmap);
	}

	i2c_set_clientdata(i2c_client, es8375);
#ifndef SPACEMIT_CONFIG_CODEC_ES8375
	/* verify that we have an es8375 */
	ret = regmap_read(es8375->regmap, ES8375_CHIP_ID1_0xFD, &val);
	if (ret < 0) {
		dev_err(&i2c_client->dev, "failed to read i2c at addr %X\n",
				i2c_client->addr);
		return ret;
	}

	/* The ES8375_CHIP_ID1 should be 0x83 */
	if (val != 0x83) {
		dev_err(&i2c_client->dev, "device at addr %X is not an es8375\n",
				i2c_client->addr);
		return -ENODEV;
	}

	ret = regmap_read(es8375->regmap, ES8375_CHIP_ID0_0xFE, &val);
	/* The ES8375_CHIP_ID0 should be 0x75 */
	if (val != 0x75) {
		dev_err(&i2c_client->dev, "device at addr %X is not an es8375\n",
				i2c_client->addr);
		return -ENODEV;
	}

#endif
	ret = es8375_read_device_properities(dev, es8375);
	if (ret != 0) {
		dev_err(&i2c_client->dev, "get an error from dts info %X\n", ret);
		return ret;
	}

	//es8375_set_gpio(es8375, PA_SHUTDOWN);

	ret = sysfs_create_group(&i2c_client->dev.kobj, &es8375_debug_attr_group);
	if (ret) {
		pr_err("failed to create attr group\n");
	}

	return devm_snd_soc_register_component(&i2c_client->dev, &es8375_codec_driver,
			&es8375_dai, 1);
}

static void es8375_i2c_shutdown(struct i2c_client *i2c)
{
	struct es8375_priv *es8375;

	es8375 = i2c_get_clientdata(i2c);

	//es8375_set_gpio(es8375, PA_SHUTDOWN);
}

static const struct i2c_device_id es8375_id[] = {
	{ "es8375" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, es8375_id);

#ifdef CONFIG_ACPI
static const struct acpi_device_id es8375_acpi_match[] = {
	{"ESSX8375", 0},
	{},
};

MODULE_DEVICE_TABLE(acpi, es8375_acpi_match);
#endif

#ifdef CONFIG_OF
static const struct of_device_id es8375_of_match[] = {
	{.compatible = "everest,es8375",},
	{}
};

MODULE_DEVICE_TABLE(of, es8375_of_match);
#endif

static struct i2c_driver es8375_i2c_driver = {
	.driver = {
		.name	= "es8375",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(es8375_of_match),
		.acpi_match_table = ACPI_PTR(es8375_acpi_match),
	},
	.shutdown = es8375_i2c_shutdown,
	.probe = es8375_i2c_probe,
	.id_table = es8375_id,
};
module_i2c_driver(es8375_i2c_driver);
MODULE_DESCRIPTION("ASoC ES8375 driver");
MODULE_AUTHOR("Dulianghan <Dulianghan@everest-semi.com>");
MODULE_LICENSE("GPL");





