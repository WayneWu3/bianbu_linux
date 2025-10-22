// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for Spacemit Mobile Storage Host Controller
 *
 * Copyright (C) 2023 Spacemit
 */

#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/reset.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/pstore_blk.h>
#include "sdhci-k1-panic.h"

extern struct bdev_info k1_bdev_info;
extern int k1_pstore_blk_get_bdev_info(struct bdev_info *bdev);

#define MWR_TO_NS		(250*1000*1000)
#define MWR_RFAIL	-1
#define MWR_ROK	0

#define k1_SMHC_BASE  0xD4281000
#define k1_PMU_BASE		0xD4282800

static char *ghost_base_reg;

static u8 __maybe_unused cur_com_reg[960]; /* 8 line, 120  character  per line */
static u8 __maybe_unused cur_pri_reg[960];

static int spacemit_reg[] = {
	0x100, 0x104, 0x108, 0x10c, 0x110, 0x114, 0x118, 0x11c,
	0x120, 0x124, 0x128, 0x12c, 0x130, 0x134, 0x160, 0x164,
	0x168, 0x16c, 0x170, 0x174, 0x178, 0x17c, 0x180, 0x184,
	0x188, 0x18c, 0x190, 0x1f0, 0x1f4, 0xFFF,
};

static void __maybe_unused dump_sdh_regs(char *host, u8 *com_reg, u8 *pri_reg)
{
	int val;
	int offset;
	int i;
	int len;
	u8 *buf;

	buf = com_reg;
	len = 0;
	i = 0;
	for (offset = 0; offset < 0x70; offset += 4) {
		val = sdhci_readl(host, offset);
		if (i % 4 == 0)
			len += sprintf(buf + len, "\n");
		len += sprintf(buf + len, "\toffset:0x%03x 0x%08x\t", offset, val);
		i++;
	}

	if (i % 4 == 0)
		len += sprintf(buf + len, "\n");
	val = sdhci_readl(host, 0xe0);
	len += sprintf(buf + len, "\toffset:0x%03x 0x%08x\t", 0xe0, val);
	val = sdhci_readl(host, 0xfc);
	len += sprintf(buf + len, "\toffset:0x%03x 0x%08x\t\n", 0xfc, val);

	buf = pri_reg;
	len = 0;
	i = 0;
	do {
		if (spacemit_reg[i] > 0x134)
			break;
		val = sdhci_readl(host, spacemit_reg[i]);
		if (i % 4 == 0)
			len += sprintf(buf + len, "\n");
		len += sprintf(buf + len, "\toffset:0x%03x 0x%08x\t", spacemit_reg[i], val);
		i++;
	} while (spacemit_reg[i] != 0xFFF);
	len += sprintf(buf + len, "\n");
}

static void sdhci_reset(char *host, u8 mask)
{
	unsigned long timeout;

	/* Wait max 100 ms */
	timeout = 100;
	sdhci_writeb(host, mask, SDHCI_SOFTWARE_RESET);
	while (sdhci_readb(host, SDHCI_SOFTWARE_RESET) & mask) {
		if (timeout == 0) {
			pr_err("%s: Reset 0x%x never completed.\n",
			       __func__, (int)mask);
			return;
		}
		timeout--;
		udelay(1000);
	}
}

static void sdhci_cmd_done(char *host, struct mmc_cmd *cmd)
{
	int i;
	if (cmd->resp_type & MMC_RSP_136) {
		/* CRC is stripped so we need to do some shifting. */
		for (i = 0; i < 4; i++) {
			cmd->response[i] = sdhci_readl(host,
					SDHCI_RESPONSE + (3-i)*4) << 8;
			if (i != 3)
				cmd->response[i] |= sdhci_readb(host,
						SDHCI_RESPONSE + (3-i)*4-1);
		}
	} else {
		cmd->response[0] = sdhci_readl(host, SDHCI_RESPONSE);
	}
}

static void sdhci_transfer_pio(char *host, struct mmc_data *data)
{
	int i;
	char *offs;
	for (i = 0; i < data->blocksize; i += 4) {
		offs = data->dest + i;
		if (data->flags == MMC_DATA_READ)
			*(u32 *)offs = sdhci_readl(host, SDHCI_BUFFER);
		else
			sdhci_writel(host, *(u32 *)offs, SDHCI_BUFFER);
	}
}

static int sdhci_transfer_data(char *host, struct mmc_data *data)
{
	unsigned int stat, rdy, mask, timeout, block = 0;
	bool transfer_done = false;

	timeout = 1000000;
	rdy = SDHCI_INT_SPACE_AVAIL | SDHCI_INT_DATA_AVAIL;
	mask = SDHCI_DATA_AVAILABLE | SDHCI_SPACE_AVAILABLE;
	do {
		stat = sdhci_readl(host, SDHCI_INT_STATUS);
		if (stat & SDHCI_INT_ERROR) {
			pr_debug("%s: Error detected in status(0x%X)!\n",
				 __func__, stat);
			return -EIO;
		}
		if (!transfer_done && (stat & rdy)) {
			if (!(sdhci_readl(host, SDHCI_PRESENT_STATE) & mask))
				continue;
			sdhci_writel(host, rdy, SDHCI_INT_STATUS);
			sdhci_transfer_pio(host, data);
			data->dest += data->blocksize;
			if (++block >= data->blocks) {
				/* Keep looping until the SDHCI_INT_DATA_END is
				 * cleared, even if we finished sending all the
				 * blocks.
				 */
				transfer_done = true;
				continue;
			}
		}

		if (timeout-- > 0)
			udelay(10);
		else {
			pr_err("%s: Transfer data timeout\n", __func__);
			return -ETIMEDOUT;
		}
	} while (!(stat & SDHCI_INT_DATA_END));

	return 0;
}

static int sdhci_wait_dat0(char *host, int state,
			   int timeout_us)
{
	int tmp;
	unsigned long timeout = jiffies + usecs_to_jiffies(timeout_us);

	// readx_poll_timeout is unsuitable because sdhci_readl accepts
	// two arguments
	do {
		tmp = sdhci_readl(host, SDHCI_PRESENT_STATE);
		if (!!(tmp & SDHCI_DATA_0_LVL_MASK) == !!state)
			return 0;
	} while (!timeout_us || !time_after(jiffies, timeout));

	return -ETIMEDOUT;
}

#define SDHCI_CMD_MAX_TIMEOUT			3200
#define SDHCI_CMD_DEFAULT_TIMEOUT		100
#define SDHCI_READ_STATUS_TIMEOUT		1000

static int sdhci_send_command(char *host, struct mmc_cmd *cmd,
			      struct mmc_data *data)
{
	unsigned int stat = 0;
	int ret = 0;
	int trans_bytes = 0;
	u32 mask, flags, mode;
	unsigned int time = 0;
	ulong start;

	/* Timeout unit - ms */
	static unsigned int cmd_timeout = SDHCI_CMD_DEFAULT_TIMEOUT;

	mask = SDHCI_CMD_INHIBIT | SDHCI_DATA_INHIBIT;

	/* We shouldn't wait for data inihibit for stop commands, even
	   though they might use busy signaling */
	if (cmd->cmdidx == MMC_CMD_STOP_TRANSMISSION ||
	    ((cmd->cmdidx == MMC_CMD_SEND_TUNING_BLOCK ||
	      cmd->cmdidx == MMC_CMD_SEND_TUNING_BLOCK_HS200) && !data))
		mask &= ~SDHCI_DATA_INHIBIT;

	while (sdhci_readl(host, SDHCI_PRESENT_STATE) & mask) {
		if (time >= cmd_timeout) {
			pr_err("%s: MMC busy ", __func__);
			if (2 * cmd_timeout <= SDHCI_CMD_MAX_TIMEOUT) {
				cmd_timeout += cmd_timeout;
				pr_err("timeout increasing to: %u ms.\n",
				       cmd_timeout);
			} else {
				return -ECOMM;
			}
		}
		time++;
		udelay(1000);
	}

	sdhci_writel(host, SDHCI_INT_ALL_MASK, SDHCI_INT_STATUS);

	mask = SDHCI_INT_RESPONSE;
	if ((cmd->cmdidx == MMC_CMD_SEND_TUNING_BLOCK ||
	     cmd->cmdidx == MMC_CMD_SEND_TUNING_BLOCK_HS200) && !data)
		mask = SDHCI_INT_DATA_AVAIL;

	if (!(cmd->resp_type & MMC_RSP_PRESENT))
		flags = SDHCI_CMD_RESP_NONE;
	else if (cmd->resp_type & MMC_RSP_136)
		flags = SDHCI_CMD_RESP_LONG;
	else if (cmd->resp_type & MMC_RSP_BUSY) {
		flags = SDHCI_CMD_RESP_SHORT_BUSY;
		mask |= SDHCI_INT_DATA_END;
	} else
		flags = SDHCI_CMD_RESP_SHORT;

	if (cmd->resp_type & MMC_RSP_CRC)
		flags |= SDHCI_CMD_CRC;
	if (cmd->resp_type & MMC_RSP_OPCODE)
		flags |= SDHCI_CMD_INDEX;
	if (data || cmd->cmdidx ==  MMC_CMD_SEND_TUNING_BLOCK ||
	    cmd->cmdidx == MMC_CMD_SEND_TUNING_BLOCK_HS200)
		flags |= SDHCI_CMD_DATA;

	/* Set Transfer mode regarding to data flag */
	if (data) {
		sdhci_writeb(host, 0xe, SDHCI_TIMEOUT_CONTROL);
		mode = SDHCI_TRNS_BLK_CNT_EN;
		trans_bytes = data->blocks * data->blocksize;
		if (data->blocks > 1)
			mode |= SDHCI_TRNS_MULTI;

		if (data->flags == MMC_DATA_READ)
			mode |= SDHCI_TRNS_READ;

		sdhci_writew(host, SDHCI_MAKE_BLKSZ(SDHCI_DEFAULT_BOUNDARY_ARG,
				data->blocksize),
				SDHCI_BLOCK_SIZE);
		sdhci_writew(host, data->blocks, SDHCI_BLOCK_COUNT);
		sdhci_writew(host, mode, SDHCI_TRANSFER_MODE);
	} else if (cmd->resp_type & MMC_RSP_BUSY) {
		sdhci_writeb(host, 0xe, SDHCI_TIMEOUT_CONTROL);
	}

	sdhci_writel(host, cmd->cmdarg, SDHCI_ARGUMENT);
	sdhci_writew(host, SDHCI_MAKE_CMD(cmd->cmdidx, flags), SDHCI_COMMAND);
	start = jiffies;
	do {
		stat = sdhci_readl(host, SDHCI_INT_STATUS);
		if (stat & SDHCI_INT_ERROR)
			break;

		if (time_after(jiffies, start + usecs_to_jiffies(SDHCI_READ_STATUS_TIMEOUT))) {
			pr_err("%s: Timeout for status update!\n",
					__func__);
			return -ETIMEDOUT;
		}
	} while ((stat & mask) != mask);

	if ((stat & (SDHCI_INT_ERROR | mask)) == mask) {
		sdhci_cmd_done(host, cmd);
		sdhci_writel(host, mask, SDHCI_INT_STATUS);
	} else
		ret = -1;

	if (!ret && data)
		ret = sdhci_transfer_data(host, data);

	stat = sdhci_readl(host, SDHCI_INT_STATUS);
	sdhci_writel(host, SDHCI_INT_ALL_MASK, SDHCI_INT_STATUS);
	if (!ret) {
		return 0;
	}

	sdhci_reset(host, SDHCI_RESET_CMD);
	sdhci_reset(host, SDHCI_RESET_DATA);
	if (stat & SDHCI_INT_TIMEOUT)
		return -ETIMEDOUT;
	else
		return -ECOMM;
}

static int sdhci_send_cmd_retry(char *host, struct mmc_cmd *cmd,
			      struct mmc_data *data, uint retries)
{
	int ret;

	do {
		ret = sdhci_send_command(host, cmd, data);
	} while (ret && retries--);

	return ret;
}

static int sdhci_send_status(char *host, unsigned int *status)
{
	struct mmc_cmd cmd;
	int ret;

	cmd.cmdidx = MMC_CMD_SEND_STATUS;
	cmd.resp_type = MMC_RSP_R1;
	cmd.cmdarg = 0x1 << 16;

	ret = sdhci_send_cmd_retry(host, &cmd, NULL, 4);
	if (!ret)
		*status = cmd.response[0];

	return ret;
}

static int sdhci_poll_for_busy(char *host, int timeout_ms)
{
	unsigned int status;
	int err;

	err = sdhci_wait_dat0(host, 1, timeout_ms * 1000);
	if (err != -ENOSYS)
		return err;

	while (1) {
		err = sdhci_send_status(host, &status);
		if (err)
			return err;

		if ((status & MMC_STATUS_RDY_FOR_DATA) &&
		    (status & MMC_STATUS_CURR_STATE) !=
		     MMC_STATE_PRG)
			break;

		if (status & MMC_STATUS_MASK) {
			return -ECOMM;
		}

		if (timeout_ms-- <= 0)
			break;

		udelay(1000);
	}

	if (timeout_ms <= 0) {
		return -ETIMEDOUT;
	}

	return 0;
}

static int __maybe_unused sdhci_set_blocklen(char *host, int len)
{
	struct mmc_cmd cmd;

	cmd.cmdidx = MMC_CMD_SET_BLOCKLEN;
	cmd.resp_type = MMC_RSP_R1;
	cmd.cmdarg = len;

	return sdhci_send_cmd_retry(host, &cmd, NULL, 4);
}

static ulong sdhci_write_blocks(char *host, ulong start,
		ulong blkcnt, const void *src, bool high_capacity)
{
	struct mmc_cmd cmd;
	struct mmc_data data;
	int timeout_ms = 1000;

	printk("sdhci_write_blocks start:0x%lx blkcnt:0x%lx src:0x%p\n",
			start, blkcnt, src);

	if (blkcnt > 1) {
		cmd.cmdidx = MMC_CMD_SET_BLOCK_COUNT;
		cmd.cmdarg = blkcnt & 0x0000FFFF;
		cmd.resp_type = MMC_RSP_R1;
		if (sdhci_send_command(host, &cmd, NULL)) {
			printk("fail to set block count\n");
			return 0;
		}
	}

	if (blkcnt == 0)
		return 0;
	else if (blkcnt == 1)
		cmd.cmdidx = MMC_CMD_WRITE_SINGLE_BLOCK;
	else
		cmd.cmdidx = MMC_CMD_WRITE_MULTIPLE_BLOCK;

	if (high_capacity)
		cmd.cmdarg = start;
	else
		cmd.cmdarg = start * MMC_BLOCK_SIZE;

	cmd.resp_type = MMC_RSP_R1;

	data.src = src;
	data.blocks = blkcnt;
	data.blocksize = MMC_BLOCK_SIZE;
	data.flags = MMC_DATA_WRITE;

	if (sdhci_send_command(host, &cmd, &data)) {
		printk("write failed\n");
		return 0;
	}

	/* Waiting for the ready status */
	if (sdhci_poll_for_busy(host, timeout_ms)) {
		printk("poll for busy failed\n");
		return 0;
	}

	return blkcnt;
}

static ulong sdhci_bwrite(char *host, ulong start,
		ulong blkcnt, const void *src)
{
	ulong cur, blocks_todo = blkcnt;
	int b_max = MMC_MAX_BLK_COUNT;

	do {
		cur = (blocks_todo > b_max) ? b_max : blocks_todo;
		if (sdhci_write_blocks(host, start, cur, src, true) != cur)
			return 0;
		blocks_todo -= cur;
		start += cur;
		src += cur * MMC_BLOCK_SIZE;
	} while (blocks_todo > 0);

	return blkcnt;
}

static int sdhci_init(char *host)
{
	/* Enable only interrupts served by the SD controller */
	sdhci_writel(host, SDHCI_INT_DATA_MASK | SDHCI_INT_CMD_MASK,
		     SDHCI_INT_ENABLE);
	/* Mask all sdhci interrupt sources */
	sdhci_writel(host, 0x0, SDHCI_SIGNAL_ENABLE);

	return 0;
}

/**use for panic situation,no irq,no lock,no dma**/
int k1_mmc_panic_write(u32 sec_addr, u32 sec_cnt, const char *inbuf)
{
	char *reg = ghost_base_reg;

	BUG_ON(inbuf == NULL);
	if (!ghost_base_reg) {
		printk("hostreg has not init\n");
		return MWR_RFAIL;
	}

	sdhci_init(reg);
	return sdhci_bwrite(reg, sec_addr, sec_cnt, inbuf);
}

ssize_t k1_mmc_panic_write_ps(const char *buf, size_t bytes, loff_t pos)
{
	ssize_t ret;
	printk("%s, %d\n", __func__, __LINE__);
	if (((pos + bytes) >> SECTOR_SHIFT) > k1_bdev_info.nr_sects) {
		printk("%s size + off (%llu + %llu) exceed (%llu)\n", __func__,
				(unsigned long long)bytes, (unsigned long long)pos, k1_bdev_info.nr_sects);
		return -EOVERFLOW;
	}
	ghost_base_reg = ioremap(k1_SMHC_BASE, 0x1F4);
	if (!ghost_base_reg) {
		printk("*iormap host failed*\n");
		return -1;
	}

	ret =  k1_mmc_panic_write(k1_bdev_info.start_sect + (pos >> SECTOR_SHIFT), bytes >> SECTOR_SHIFT, buf);
	if (ret)
		return ret;
	return 0;
}
#if 0
int k1_mmc_panic_init_ps(void *data)
{
	int ret = 0;
	struct pstore_zone_info zone = {};
	//struct pstore_device_info dev = {};

	printk("%s, %d\n", __func__, __LINE__);


	ret = k1_mmc_panic_init();
	if (ret <= MWR_RFAIL)
		return ret;

	zone.owner = THIS_MODULE;
	zone.max_reason = CONFIG_PSTORE_BLK_MAX_REASON;
	zone.panic_write = k1_mmc_panic_write_ps;
	ret = register_pstore_zone(&zone);
	if (ret == -ENODEV) {
		printk("%s, %d\n", __func__, __LINE__);
	}
	if (ret)
		goto exit_panic;

	return 0;

exit_panic:
	k1_mmc_panic_exit();
	return ret;
}
#endif

static void k1_mmc_panic_resource_release(void)
{
	iounmap(ghost_base_reg);
}

static int k1_mmc_register_pstore_panic(enum pstore_blk_notifier_type type,
		struct pstore_device_info *pdi)
{
	switch (type) {
	case PSTORE_BLK_BACKEND_REGISTER:
	case PSTORE_BLK_BACKEND_PANIC_DRV_REGISTER:
		pdi->zone.panic_write = k1_mmc_panic_write_ps;
		pr_info("register pstore backend %s panic write\n", pdi->zone.name);
		break;

	case PSTORE_BLK_BACKEND_UNREGISTER:
	case PSTORE_BLK_BACKEND_PANIC_DRV_UNREGISTER:
		pdi->zone.panic_write = NULL;

		break;
	default:
		goto err;

	}

	return NOTIFY_OK;

err:
	return NOTIFY_BAD;
}

static struct pstore_blk_notifier k1_mmc_panic = {
	.notifier_call = k1_mmc_register_pstore_panic,
};

static int __init k1_mmc_panic_init(void)
{
	if (register_pstore_blk_panic_notifier(&k1_mmc_panic) == NOTIFY_OK)
		return 0;
	else
		return -1;
}


static void __exit k1_mmc_panic_exit(void)
{
	unregister_pstore_blk_panic_notifier(&k1_mmc_panic);

	k1_mmc_panic_resource_release();
}


module_init(k1_mmc_panic_init);
module_exit(k1_mmc_panic_exit);

MODULE_DESCRIPTION("Spacemit SD/MMC Card Panic Pstore backend Support");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0.0");
