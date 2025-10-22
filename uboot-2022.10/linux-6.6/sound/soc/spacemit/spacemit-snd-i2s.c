// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2024 SPACEMIT
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/pm_runtime.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/dmaengine.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/dmaengine_pcm.h>
#include "spacemit-snd-i2s.h"

//APB Clock/Reset Control Register
#define APB_CLK_BASE        0xD4015000
#define APB_SSP0_CLK_RST    0x80
#define APB_SSP1_CLK_RST    0x84
#define APB_AIB_CLK_RST     0x3C

#define FNCLKSEL_6p5M       (0x0 << 4)
#define FNCLKSEL_13M        (0x1 << 4)
#define FNCLKSEL_26M        (0x2 << 4)
#define FNCLKSEL_52M        (0x3 << 4)
#define FNCLKSEL_3p25M      (0x4 << 4)
#define FNCLKSEL_1p625M     (0x5 << 4)
#define FNCLKSEL_812p5M     (0x6 << 4)
#define FNCLKSEL_AUDIO      (0x7 << 4)
#define CLK_ON_NORST        (0x3 << 0)
#define CLK_ON_RST          (0x7 << 0)
#define CLK_ON_26M          (CLK_ON_NORST | FNCLKSEL_26M)
#define CLK_ON_6p5M         (CLK_ON_NORST | FNCLKSEL_6p5M)
#define CLK_ON_I2S          ((0x1<<3) | CLK_ON_NORST | FNCLKSEL_AUDIO)

//I2S CLK
#define  PMUMAIN_BASE       0xD4050000
#define  ISCCR1             0x44
#define  SYSCLK_EN          (0x1 << 31)
#define  BITCLK_EN          (0x1 << 29)
#define  SYSCLK_BASE_156M   (0x1 << 30)
#define  SYSCLK_BASE_26M    (0x0 << 30)
#define  BITCLK_DIV_468     (0x0 << 27)
#define  FRAME_48K_I2S      (0x4 << 15)

#define  SYSCLK_PRE_CTRL    0x08

/*
 * ssp:sspa audio private data
 */
 struct ssp_device {
	struct platform_device *pdev;
	struct list_head	node;

	struct clk	*clk;
	void __iomem	*mmio_base;
	void __iomem	*mmio_ctrl_base;
	void __iomem	*apb_clk_base;
	void __iomem	*pmumain;
	unsigned long	phys_base;

	const char	*label;
	int		port_id;
	int		type;
	int		use_count;
	int		irq;

	struct device_node	*of_node;
};

struct sspa_priv {
	struct ssp_device *sspa;
	struct snd_dmaengine_dai_dma_data *dma_params;
	struct reset_control *sspa_rst;
	int dai_fmt;
	int dai_id_pre;
	int running_cnt;
	struct platform_device *i2splatdev;
	unsigned int sysclk;
	unsigned int mclk_fs;
};

static void i2s_sspa_write_reg(struct ssp_device *sspa, u32 reg, u32 val)
{
	__raw_writel(val, sspa->mmio_base + reg);
}

static u32 i2s_sspa_read_reg(struct ssp_device *sspa, u32 reg)
{
	return __raw_readl(sspa->mmio_base + reg);
}

static int i2s_sspa_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct sspa_priv *sspa_priv = snd_soc_dai_get_drvdata(dai);
	struct ssp_device *sspa = sspa_priv->sspa;
	unsigned int ssp_top_cfg = 0, ssp_fifo_cfg = 0, ssp_int_en_cfg = 0;
	unsigned int ssp_to_cfg = 0, ssp_psp_cfg = 0, ssp_net_work_ctrl = 0;
	int dai_id = dai->id;

	pm_runtime_get_sync(&sspa_priv->i2splatdev->dev);

	if ((sspa_priv->dai_id_pre == dai_id) & (i2s_sspa_read_reg(sspa, PSP_CTRL)))
		return 0;

	ssp_top_cfg  = TOP_TRAIL_DMA | DW_32BYTE | TOP_SFRMDIR_M | TOP_SCLKDIR_M | TOP_FRF_PSP;
	ssp_fifo_cfg = FIFO_RSRE | FIFO_TSRE | FIFO_RX_THRES_15 | FIFO_TX_THRES_15;

	if ((i2s_sspa_read_reg(sspa, TOP_CTRL) & TOP_SSE)) {
		pr_debug("no need to change hardware dai format: stream is in use\n");
		return 0;
	}

	switch (sspa_priv->dai_fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) {
	case SND_SOC_DAIFMT_CBP_CFP:
		ssp_top_cfg |= TOP_SFRMDIR_M;
		ssp_top_cfg |= TOP_SCLKDIR_M;
		pr_debug("%s,%d------------SND_SOC_DAIFMT_CBP_CFP\n", __func__, __LINE__);
		break;
	case SND_SOC_DAIFMT_CBC_CFC:
		pr_debug("%s,%d------------SND_SOC_DAIFMT_CBC_CFC\n", __func__, __LINE__);
		break;
	default:
		return -EINVAL;
	}

	switch (sspa_priv->dai_fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		pr_debug("%s,%d------------mode i2s\n", __func__, __LINE__);
		ssp_top_cfg |= TOP_FRF_PSP;
		ssp_psp_cfg = (0x10<<12) | (0x1<<3) | PSP_SFRMP;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		pr_debug("%s,%d------------mode B\n", __func__, __LINE__);
		ssp_top_cfg |= TOP_FRF_PSP;
		ssp_psp_cfg = (0x1<<12) | PSP_SFRMP;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		pr_debug("%s,%d------------mode A\n", __func__, __LINE__);
		ssp_top_cfg |= TOP_FRF_PSP;
		ssp_psp_cfg = (0x1<<12) | (0x1<<3) | PSP_SFRMP;
		break;
	default:
		pr_debug("%s, unexpected format type\n", __func__);
		return -EINVAL;
	}

	i2s_sspa_write_reg(sspa, TOP_CTRL, ssp_top_cfg);
	i2s_sspa_write_reg(sspa, PSP_CTRL, ssp_psp_cfg);
	i2s_sspa_write_reg(sspa, INT_EN, ssp_int_en_cfg);
	i2s_sspa_write_reg(sspa, TO, ssp_to_cfg);
	i2s_sspa_write_reg(sspa, FIFO_CTRL, ssp_fifo_cfg);
	i2s_sspa_write_reg(sspa, NET_WORK_CTRL, ssp_net_work_ctrl);

	pr_debug("TOP_CTRL=0x%x,\n PSP_CTRL=0x%x,\n INT_EN=0x%x,\n TO=0x%x,\n FIFO_CTRL=0x%x,\n,NET_WORK_CTRL=0x%x",
				i2s_sspa_read_reg(sspa, TOP_CTRL),
				i2s_sspa_read_reg(sspa, PSP_CTRL),
				i2s_sspa_read_reg(sspa, INT_EN),
				i2s_sspa_read_reg(sspa, TO),
				i2s_sspa_read_reg(sspa, FIFO_CTRL),
				i2s_sspa_read_reg(sspa, NET_WORK_CTRL));
	return 0;
}

static void i2s_sspa_shutdown(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct sspa_priv *priv = snd_soc_dai_get_drvdata(dai);
	pm_runtime_put_sync(&priv->i2splatdev->dev);
}

static int i2s_sspa_set_dai_sysclk(struct snd_soc_dai *cpu_dai,
				    int clk_id, unsigned int freq, int dir)
{
	struct sspa_priv *sspa_priv = snd_soc_dai_get_drvdata(cpu_dai);

	if (sspa_priv->running_cnt)
		return 0;

	if (sspa_priv->sysclk == freq)
		return 0;

	sspa_priv->sysclk = freq;
	return 0;
}

static int i2s_sspa_set_dai_pll(struct snd_soc_dai *cpu_dai, int pll_id,
				 int source, unsigned int freq_in,
				 unsigned int freq_out)
{
	return 0;
}

/*
 * Set up the sspa dai format. The sspa port must be inactive
 * before calling this function as the physical
 * interface format is changed.
 */
static int i2s_sspa_set_dai_fmt(struct snd_soc_dai *cpu_dai,
				 unsigned int fmt)
{
	struct sspa_priv *sspa_priv = snd_soc_dai_get_drvdata(cpu_dai);
	int dai_id = cpu_dai->id;

	pr_debug("%s, fmt=0x%x, dai_id=0x%x\n", __FUNCTION__, fmt, dai_id);

	sspa_priv->dai_fmt = fmt;
	sspa_priv->dai_id_pre = dai_id;

	switch (sspa_priv->dai_fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		pr_debug("%s,%d------------mode i2s\n", __func__, __LINE__);
		cpu_dai->driver->playback.formats = SNDRV_PCM_FMTBIT_S16_LE;
		cpu_dai->driver->capture.formats = SNDRV_PCM_FMTBIT_S16_LE;
		break;
	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_DSP_B:
		pr_debug("%s,%d------------mode A/B\n", __func__, __LINE__);
		cpu_dai->driver->playback.channels_min = 1;
		cpu_dai->driver->playback.channels_max = 1;
		cpu_dai->driver->capture.channels_min = 1;
		cpu_dai->driver->capture.channels_max = 1;
		cpu_dai->driver->playback.formats = SNDRV_PCM_FMTBIT_S32_LE;
		cpu_dai->driver->capture.formats = SNDRV_PCM_FMTBIT_S32_LE;
		break;
	default:
		pr_debug("%s, unexpected format type\n", __func__);
		return -EINVAL;
	}
	return 0;
}

/*
 * Set the SSPA audio DMA parameters and sample size.
 * Can be called multiple times.
 */
static int i2s_sspa_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params,
			       struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	struct sspa_priv *sspa_priv = snd_soc_dai_get_drvdata(dai);
	struct ssp_device *sspa = sspa_priv->sspa;
	struct snd_dmaengine_dai_dma_data *dma_params;
	unsigned int val, target;
	unsigned int ssp_top_cfg = 0, data_width = 0, data_bits = 0;

	if (sspa_priv->running_cnt)
		return 0;

	dma_params = &sspa_priv->dma_params[substream->stream];
	dma_params->addr = (sspa->phys_base + DATAR);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		data_bits = 8;
		data_width = DW_8BYTE;
		dma_params->maxburst = 8;
		dma_params->addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		data_bits = 16;
		data_width = DW_16BYTE;
		dma_params->maxburst = 16;
		dma_params->addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
		if ((sspa_priv->dai_fmt & SND_SOC_DAIFMT_FORMAT_MASK) == SND_SOC_DAIFMT_I2S) {
			data_width = DW_32BYTE;
			dma_params->maxburst = 32;
			dma_params->addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		}
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		data_bits = 32;
		data_width = DW_32BYTE;
		dma_params->maxburst = 32;
		dma_params->addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		break;
	default:
		pr_debug("%s, unexpected data width type\n", __func__);
		return -EINVAL;
	}

	ssp_top_cfg = i2s_sspa_read_reg(sspa, TOP_CTRL);
	ssp_top_cfg &= ~DW_32BYTE;
	ssp_top_cfg |= data_width;
	i2s_sspa_write_reg(sspa, TOP_CTRL, ssp_top_cfg);

	snd_soc_dai_set_dma_data(cpu_dai, substream, dma_params);

	sspa_priv->mclk_fs = sspa_priv->sysclk / (params_rate(params));
	switch (sspa_priv->mclk_fs) {
	case 64:
		target = SYSCLK_BASE_156M | 326 << 15 | 32600; //64fs
		break;
	case 128:
		target = SYSCLK_BASE_156M | 652 << 15 | 32600; //128fs
		break;
	case 256:
		target = SYSCLK_BASE_156M | 1304 << 15 | 32600; //256fs
		break;
	default:
		target = SYSCLK_BASE_156M | 1304 << 15 | 32600; //256fs
		break;
	}

	switch (sspa_priv->sysclk / (params_channels(params) * params_rate(params) * data_bits)) {
	case 2:
		target |= 0 << 27;
		break;
	case 4:
		target |= 1 << 27;
		break;
	case 6:
		target |= 2 << 27;
		break;
	case 8:
		target |= 3 << 27;
		break;
	default:
		target |= 3 << 27;
		break;
	}

	val = __raw_readl(sspa->pmumain + ISCCR1);
	val = val & ~0x5FFFFFFF;
	__raw_writel(val | target, sspa->pmumain + ISCCR1);

	val = __raw_readl(sspa->pmumain + SYSCLK_PRE_CTRL);
	val |= 1 << 29;
	__raw_writel(val, sspa->pmumain + SYSCLK_PRE_CTRL);
	return 0;
}

static int i2s_sspa_trigger(struct snd_pcm_substream *substream, int cmd,
			     struct snd_soc_dai *dai)
{
	struct sspa_priv *sspa_priv = snd_soc_dai_get_drvdata(dai);
	struct ssp_device *sspa = sspa_priv->sspa;
	int ret = 0;
	unsigned int ssp_top_cfg;

	pr_debug("%s cmd=%d, cnt=%d\n", __FUNCTION__, cmd, sspa_priv->running_cnt);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (sspa_priv->running_cnt == 0) {
			ssp_top_cfg = i2s_sspa_read_reg(sspa, TOP_CTRL);
			pr_debug("TOP_CTRL:0x%x", ssp_top_cfg);
			ssp_top_cfg |= TOP_SSE;
			i2s_sspa_write_reg(sspa, TOP_CTRL, ssp_top_cfg);   //SSP_enable
		}
		sspa_priv->running_cnt++;

		pr_debug("triger::TOP_CTRL=0x%x,\n PSP_CTRL=0x%x,\n INT_EN=0x%x,\n TO=0x%x,\n FIFO_CTRL=0x%x,\n",
			i2s_sspa_read_reg(sspa, TOP_CTRL), i2s_sspa_read_reg(sspa, PSP_CTRL),
			i2s_sspa_read_reg(sspa, INT_EN),
			i2s_sspa_read_reg(sspa, TO),
			i2s_sspa_read_reg(sspa, FIFO_CTRL));
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (sspa_priv->running_cnt > 0)
			sspa_priv->running_cnt--;
		if (sspa_priv->running_cnt == 0) {
			ssp_top_cfg = i2s_sspa_read_reg(sspa, TOP_CTRL);
			ssp_top_cfg &= (~TOP_SSE);
			i2s_sspa_write_reg(sspa, TOP_CTRL, ssp_top_cfg);
			pr_debug("TOP_CTRL=0x%x, dai->id=%d \n", i2s_sspa_read_reg(sspa, TOP_CTRL), dai->id);
		}
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			pr_debug("%s ignore playback tx\n", __FUNCTION__);
		}

		/* have no capture stream, disable rx port */
		if (!sspa_priv->running_cnt) {
			pr_debug("%s sspa_priv->running_cnt=%d\n", __FUNCTION__, sspa_priv->running_cnt);
		}
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

static int i2s_sspa_probe(struct snd_soc_dai *dai)
{
	struct sspa_priv *priv = dev_get_drvdata(dai->dev);
	struct ssp_device *sspa = priv->sspa;
	unsigned int sspa_clk = 0;

	if (dai->id == 0)
	{
		//i2s0
		sspa_clk = __raw_readl(sspa->apb_clk_base + APB_SSP0_CLK_RST);
		__raw_writel((1 << 3)|sspa_clk, sspa->apb_clk_base + APB_SSP0_CLK_RST);
	} else {
		//i2s1
		sspa_clk = __raw_readl(sspa->apb_clk_base + APB_SSP1_CLK_RST);
		__raw_writel((1 << 3)|sspa_clk, sspa->apb_clk_base + APB_SSP1_CLK_RST);
	}
	reset_control_deassert(priv->sspa_rst);
	snd_soc_dai_set_drvdata(dai, priv);
	return 0;

}

#define I2S_SSPA_RATES SNDRV_PCM_RATE_8000_192000
#define I2S_SSPA_FORMATS (SNDRV_PCM_FMTBIT_S8 | \
		SNDRV_PCM_FMTBIT_S16_LE | \
		SNDRV_PCM_FMTBIT_S24_LE | \
		SNDRV_PCM_FMTBIT_S32_LE)

static const struct snd_soc_dai_ops i2s_sspa_dai_ops = {
	.probe		= i2s_sspa_probe,
	.startup	= i2s_sspa_startup,
	.shutdown	= i2s_sspa_shutdown,
	.trigger	= i2s_sspa_trigger,
	.hw_params	= i2s_sspa_hw_params,
	.set_sysclk	= i2s_sspa_set_dai_sysclk,
	.set_pll	= i2s_sspa_set_dai_pll,
	.set_fmt	= i2s_sspa_set_dai_fmt,
};

static struct snd_soc_dai_driver i2s0_sspa_dai = {
	.name = "i2s0-dai",
	.id = 0,
	.playback = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = I2S_SSPA_FORMATS,
	},
	.capture = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = I2S_SSPA_FORMATS,
	},
	.ops = &i2s_sspa_dai_ops,
};

static struct snd_soc_dai_driver i2s1_sspa_dai = {
	.name = "i2s1-dai",
	.id = 1,
	.playback = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = I2S_SSPA_FORMATS,
	},
	.capture = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = I2S_SSPA_FORMATS,
	},
	.ops = &i2s_sspa_dai_ops,
};

static void i2s_sspa_init(struct sspa_priv *priv)
{
	struct ssp_device *sspa = priv->sspa;
	unsigned int target, val;

	i2s_sspa_write_reg(sspa, TOP_CTRL, TOP_TRAIL_DMA | DW_32BYTE | TOP_SFRMDIR_M | TOP_SCLKDIR_M | TOP_FRF_PSP);
	i2s_sspa_write_reg(sspa, FIFO_CTRL, FIFO_RSRE | FIFO_TSRE | FIFO_RX_THRES_15 | FIFO_TX_THRES_15);
	i2s_sspa_write_reg(sspa, INT_EN, 0);
	i2s_sspa_write_reg(sspa, PSP_CTRL, (0x10 << 12) | (0x1 << 3) | PSP_SFRMP);

	switch (priv->mclk_fs) {
	case 64:
		target = SYSCLK_BASE_156M | 0 << 27 | 326 << 15 | 32600; //64fs
		break;
	case 128:
		target = SYSCLK_BASE_156M | 1 << 27 | 652 << 15 | 32600; //128fs
		break;
	case 256:
		target = SYSCLK_BASE_156M | 3 << 27 | 1304 << 15 | 32600; //256fs
		break;
	default:
		target = SYSCLK_BASE_156M | 3 << 27 | 1304 << 15 | 32600; //256fs
		break;
	}
	val = __raw_readl(sspa->pmumain + ISCCR1);
	val = val & ~0x5FFFFFFF;
	__raw_writel(val | target, sspa->pmumain + ISCCR1);

	val = __raw_readl(sspa->pmumain + SYSCLK_PRE_CTRL);
	val |= 1 << 29;
	__raw_writel(val, sspa->pmumain + SYSCLK_PRE_CTRL);

}

static int i2s_sspa_suspend(struct device *dev)
{
	struct sspa_priv *priv = dev_get_drvdata(dev);

	reset_control_assert(priv->sspa_rst);
	return 0;
}

static int i2s_sspa_resume(struct device *dev)
{
	struct sspa_priv *priv = dev_get_drvdata(dev);
	struct ssp_device *sspa = priv->sspa;
	unsigned int sspa_clk = 0;

	if (priv->dai_id_pre == 0)
	{
		//i2s0
		sspa_clk = __raw_readl(sspa->apb_clk_base + APB_SSP0_CLK_RST);
		__raw_writel((1 << 3)|sspa_clk, sspa->apb_clk_base + APB_SSP0_CLK_RST);
	} else {
		//i2s1
		sspa_clk = __raw_readl(sspa->apb_clk_base + APB_SSP1_CLK_RST);
		__raw_writel((1 << 3)|sspa_clk, sspa->apb_clk_base + APB_SSP1_CLK_RST);
	}
	reset_control_deassert(priv->sspa_rst);

	if (i2s_sspa_read_reg(sspa, TOP_CTRL) == 0) {
		i2s_sspa_init(priv);
	}
	return 0;
}

static const struct dev_pm_ops spacemit_i2s_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(i2s_sspa_suspend, i2s_sspa_resume)
};
static const struct snd_soc_component_driver i2s_sspa_component = {
	.name 			= "spacemit-dmasspa-dai",
};

static int asoc_i2s_sspa_probe(struct platform_device *pdev)
{
	struct sspa_priv *priv;
	struct resource *res;
	struct snd_soc_dai_driver *dai;

	pr_debug("%s enter: dev name %s\n", __func__, dev_name(&pdev->dev));
	priv = devm_kzalloc(&pdev->dev,
				sizeof(struct sspa_priv), GFP_KERNEL);
	if (!priv) {
		pr_err("%s priv alloc failed\n", __FUNCTION__);
		return -ENOMEM;
	}
	priv->sspa = devm_kzalloc(&pdev->dev,
				sizeof(struct ssp_device), GFP_KERNEL);
	if (priv->sspa == NULL) {
		pr_err("%s sspa alloc failed\n", __FUNCTION__);
		return -ENOMEM;
	}

	priv->dma_params = devm_kcalloc(&pdev->dev,
			2, sizeof(struct snd_dmaengine_dai_dma_data),
			GFP_KERNEL);
	if (priv->dma_params == NULL) {
		pr_err("%s dma_params alloc failed\n", __FUNCTION__);
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pr_debug("%s, start=0x%lx, end=0x%lx\n", __FUNCTION__, (unsigned long)res->start, (unsigned long)res->end);
	priv->sspa->mmio_base = devm_ioremap_resource(&pdev->dev, res);

	if (IS_ERR(priv->sspa->mmio_base)) {
		pr_err("%s mmio_base alloc failed\n", __FUNCTION__);
		return PTR_ERR(priv->sspa->mmio_base);
	}

	priv->sspa->phys_base = res->start;

	if ((priv->sspa->apb_clk_base = ioremap(APB_CLK_BASE, 0x100)) == NULL) {
		pr_err("sspa ioremap err\n");
		return -1;
	}
	if ((priv->sspa->pmumain = ioremap(PMUMAIN_BASE, 0x100)) == NULL) {
		pr_err("sspa pmumain ioremap err\n");
		return -1;
	}
	//get reset
	priv->sspa_rst = devm_reset_control_get(&pdev->dev, "sspa-rst");
	if (IS_ERR(priv->sspa_rst))
		return PTR_ERR(priv->sspa_rst);

	pm_runtime_enable(&pdev->dev);
	priv->i2splatdev = pdev;

	priv->dai_fmt = (unsigned int) -1;
	platform_set_drvdata(pdev, priv);
	pr_debug("exit %s\n", __FUNCTION__);
	if (of_device_is_compatible(pdev->dev.of_node, "spacemit,spacemit-i2s0")) {
		dai = &i2s0_sspa_dai;
	} else {
		dai = &i2s1_sspa_dai;
	}
	return devm_snd_soc_register_component(&pdev->dev, &i2s_sspa_component, dai, 1);
}

static int asoc_i2s_sspa_remove(struct platform_device *pdev)
{
	struct sspa_priv *priv = platform_get_drvdata(pdev);

	pm_runtime_disable(&pdev->dev);
	reset_control_assert(priv->sspa_rst);
	snd_soc_unregister_component(&pdev->dev);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id spacemit_i2s_ids[] = {
	{ .compatible = "spacemit,spacemit-i2s0", },
	{ .compatible = "spacemit,spacemit-i2s1", },
	{ /* sentinel */ }
};
#endif

static struct platform_driver asoc_i2s_sspa_driver = {
	.driver = {
		.name = "spacemit-snd-i2s",
		.of_match_table = of_match_ptr(spacemit_i2s_ids),
		.pm = &spacemit_i2s_pm_ops,
	},
	.probe = asoc_i2s_sspa_probe,
	.remove = asoc_i2s_sspa_remove,
};

module_platform_driver(asoc_i2s_sspa_driver);

MODULE_DESCRIPTION("I2S SSPA SoC driver");
MODULE_LICENSE("GPL");
