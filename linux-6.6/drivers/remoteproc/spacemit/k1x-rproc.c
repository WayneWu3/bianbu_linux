// SPDX-License-Identifier: GPL-2.0

#include <linux/limits.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/pm_runtime.h>
#include <linux/remoteproc.h>
#include <linux/reset.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/kthread.h>
#include <linux/clk-provider.h>
#include <linux/mailbox_client.h>
#include <linux/completion.h>
#include <linux/freezer.h>
#include <linux/firmware.h>
#include <linux/elf.h>
#include <uapi/linux/sched/types.h>
#include <uapi/linux/sched.h>
#include <linux/sched/prio.h>
#include <linux/rpmsg.h>
#include <linux/pm_qos.h>
#include <linux/delay.h>
#include <linux/syscore_ops.h>
#include <linux/fs.h>
#include <linux/pm_domain.h>
#include <linux/dma-map-ops.h>
#include <linux/dma-direction.h>
#include <linux/spacemit/platform_pm_ops.h>
#include <linux/suspend.h>
#include "../remoteproc_internal.h"
#include "../remoteproc_elf_helpers.h"

#define MAX_MEM_BASE	2
#define MAX_MBOX	2

#define K1X_MBOX_VQ0_ID	0
#define K1X_MBOX_VQ1_ID	1

#define BOOTC_MEM_BASE_OFFSET	0
#define SYSCTRL_MEM_BASE_OFFSET	1

#define ESOS_BOOT_ENTRY_REG_OFFSET	0x88
#define ESOS_BOOTUP_REG_OFFSET		0x30
#define ESOS_AON_PER_CLK_RST_CTL_REG	0x2c
#define AUDIO_PMU_VOTE_REG_OFFSET	0x20

#define ESOS_DDR_REGMAP_BASE_REG_OFFSET	0xc0

#define APMU_AUDIO_CLK_RES_CTRL	0x14c
#define APMU_AUDIO_POWER_STATUS_OFFSET	23

#define DEV_PM_QOS_CLK_GATE            1
#define DEV_PM_QOS_REGULATOR_GATE      2
#define DEV_PM_QOS_PM_DOMAIN_GATE      4
#define DEV_PM_QOS_DEFAULT             7

struct dev_pm_qos_request greq;

#ifdef CONFIG_HIBERNATION
void *hibernate_rcpu_snapshot;
void *hibernate_rcpu_runtime_vbase;
void *hibernate_rcpu_runtime_pbase;
unsigned int hibernal_snapshot_size;
#endif

struct spacemit_mbox {
	const char name[10];
	struct mbox_chan *chan;
	struct mbox_client client;
	struct task_struct *mb_thread;
	bool kthread_running;
	struct completion mb_comp;
	int vq_id;
};

struct spacemit_rproc {
	struct device *dev;
	unsigned int apb_clk_rate, apb_clk_rate_default;
	struct reset_control *core_rst;
	struct clk *core_clk, *apb_clk;
	unsigned int ddr_remap_base;
	void __iomem *base[MAX_MEM_BASE];
	struct spacemit_mbox *mb;
	char *verid;
	unsigned int size;
#ifdef CONFIG_PM_SLEEP
	struct rpmsg_device *rpdev;
#ifdef CONFIG_HIBERNATION
	struct notifier_block pm_notifier;
	int hinernate_restore;
	int hibernate_freeze;
#endif
#endif
};

static int spacemit_rproc_mem_alloc(struct rproc *rproc,
				 struct rproc_mem_entry *mem)
{
	void __iomem *va = NULL;

	dev_dbg(&rproc->dev, "map memory: %pa+%zx\n", &mem->dma, mem->len);
	va = ioremap(mem->dma, mem->len);
	if (!va) {
		dev_err(&rproc->dev, "Unable to map memory region: %pa+%zx\n",
			&mem->dma, mem->len);
		return -ENOMEM;
	}

	memset(va, 0, mem->len);

	/* Update memory entry va */
	mem->va = va;

	return 0;
}

static int spacemit_rproc_mem_release(struct rproc *rproc,
				   struct rproc_mem_entry *mem)
{
	dev_dbg(&rproc->dev, "unmap memory: %pa\n", &mem->dma);

	iounmap(mem->va);

	return 0;
}

static int spacemit_rproc_prepare(struct rproc *rproc)
{
	struct spacemit_rproc *priv = rproc->priv;
	struct device *dev = rproc->dev.parent;
	struct device_node *np = dev->of_node;
	struct of_phandle_iterator it;
	struct rproc_mem_entry *mem;
	struct reserved_mem *rmem;
	u32 da;
	int ret, index = 0;

	/* de-assert the audio module */
	reset_control_deassert(priv->core_rst);

	/* open the clk & pm-switch using pm-domain framework */
	dev_pm_qos_add_request(priv->dev, &greq, DEV_PM_QOS_MAX_FREQUENCY,
			DEV_PM_QOS_CLK_GATE | DEV_PM_QOS_PM_DOMAIN_GATE);

	/* enable the power-switch and the clk */
	pm_runtime_get_sync(priv->dev);

	priv->apb_clk_rate_default = clk_get_rate(priv->apb_clk);

	/* set apb clk rate */
	clk_set_rate(priv->apb_clk, priv->apb_clk_rate);

	/* Register associated reserved memory regions */
	of_phandle_iterator_init(&it, np, "memory-region", NULL, 0);
	while (of_phandle_iterator_next(&it) == 0) {
		rmem = of_reserved_mem_lookup(it.node);
		if (!rmem) {
			dev_err(&rproc->dev,
				"unable to acquire memory-region\n");
			return -EINVAL;
		}

		if (rmem->base > U64_MAX) {
			dev_err(&rproc->dev,
				"the rmem base is overflow\n");
			return -EINVAL;
		}

		/* find the da */
		ret = of_property_read_u32(it.node, "da_base", &da);
		if (ret) {
			/* no da_base; means that the da = dma */
			da = rmem->base;
		}

		mem = rproc_mem_entry_init(dev, NULL,
					   rmem->base,
					   rmem->size, da,
					   spacemit_rproc_mem_alloc,
					   spacemit_rproc_mem_release,
					   it.node->name);
		if (!mem)
			return -ENOMEM;

		rproc_add_carveout(rproc, mem);
		index++;
	}

	return 0;
}

static const void *find_version_id_section(struct device *dev, const struct firmware *fw)
{
	const void *shdr, *name_table_shdr;
	int i;
	const char *name_table;
	const u8 *elf_data = (void *)fw->data;
	u8 class = fw_elf_get_class(fw);
	const void *ehdr = elf_data;
	u16 shnum = elf_hdr_get_e_shnum(class, ehdr);
	u32 elf_shdr_get_size = elf_size_of_shdr(class);
	u16 shstrndx = elf_hdr_get_e_shstrndx(class, ehdr);

	/* First, get the section header according to the elf class */
	shdr = elf_data + elf_hdr_get_e_shoff(class, ehdr);
	/* Compute name table section header entry in shdr array */
	name_table_shdr = shdr + (shstrndx * elf_shdr_get_size);
	/* Finally, compute the name table section address in elf */
	name_table = elf_data + elf_shdr_get_sh_offset(class, name_table_shdr);

	for (i = 0; i < shnum; i++, shdr += elf_shdr_get_size) {
		u32 name = elf_shdr_get_sh_name(class, shdr);

		if (strcmp(name_table + name, ".version_id_table"))
			continue;

		/* make sure we have the entire table */
		return shdr;
	}

	return NULL;
}

char *rproc_elf_find_version_id_table(struct rproc *rproc, const struct firmware *fw, u64 *sh_size_p)
{
	const void *shdr;
	u64 sh_addr, sh_size;
	u8 class = fw_elf_get_class(fw);
	struct device *dev = &rproc->dev;

	shdr = find_version_id_section(&rproc->dev, fw);
	if (!shdr)
		return NULL;

	sh_addr = elf_shdr_get_sh_addr(class, shdr);
	sh_size = elf_shdr_get_sh_size(class, shdr);

	if (!rproc_u64_fit_in_size_t(sh_size)) {
		dev_err(dev, "size (%llx) does not fit in size_t type\n",
			sh_size);
		return NULL;
	}

	*sh_size_p = sh_size;

	return (char *)sh_addr;
}

static int spacemit_rproc_start(struct rproc *rproc)
{
	struct spacemit_rproc *priv = rproc->priv;

	if (priv->verid != NULL)
		pr_notice("the firmare version id is:%s\n", rproc_da_to_va(rproc, (u64)priv->verid, priv->size, NULL));

	/* enable ipc2ap clk & reset--> rcpu side */
	writel(0xff, priv->base[BOOTC_MEM_BASE_OFFSET] + ESOS_AON_PER_CLK_RST_CTL_REG);

	/* set the boot-entry */
	writel(rproc->bootaddr, priv->base[SYSCTRL_MEM_BASE_OFFSET] + ESOS_BOOT_ENTRY_REG_OFFSET);

	/* set ddr map */
	writel(priv->ddr_remap_base, priv->base[SYSCTRL_MEM_BASE_OFFSET] + ESOS_DDR_REGMAP_BASE_REG_OFFSET);

	/* lanching up esos */
	writel(1, priv->base[BOOTC_MEM_BASE_OFFSET] + ESOS_BOOTUP_REG_OFFSET);

	return 0;
}

static int spacemit_rproc_stop(struct rproc *rproc)
{
	struct spacemit_rproc *priv = rproc->priv;

	/* hold the rcpu */
	writel(0, priv->base[BOOTC_MEM_BASE_OFFSET] + ESOS_BOOTUP_REG_OFFSET);

	pm_runtime_put_sync(priv->dev);

	reset_control_assert(priv->core_rst);

	return 0;
}

static int spacemit_rproc_parse_fw(struct rproc *rproc, const struct firmware *fw)
{
	int ret;
	u64 sh_size;
	struct spacemit_rproc *ddata = rproc->priv;
	char *version_id_table;

	ddata->verid = NULL;
	/* find the firmare id */
	version_id_table = rproc_elf_find_version_id_table(rproc, fw, &sh_size);
	if (!version_id_table) {
		dev_info(&rproc->dev, "Can not find version id table\n");
	} else {
		ddata->verid = version_id_table;
		ddata->size = sh_size;
	}

	ret = rproc_elf_load_rsc_table(rproc, fw);
	if (ret)
		dev_info(&rproc->dev, "No resource table in elf\n");

	return 0;
}

static u64 spacemit_get_boot_addr(struct rproc *rproc, const struct firmware *fw)
{
	int err;
	unsigned int entry_point;
	struct device *dev = rproc->dev.parent;

	/* get the entry point */
	err = of_property_read_u32(dev->of_node, "esos-entry-point", &entry_point);
	if (err) {
		 dev_err(dev, "failed to get entry point\n");
		 return 0;
	}

	return entry_point;
}

static void spacemit_rproc_kick(struct rproc *rproc, int vqid)
{
	struct spacemit_rproc *ddata = rproc->priv;
	unsigned int i;
	int err;

	if (WARN_ON(vqid >= MAX_MBOX))
		return;

	for (i = 0; i < MAX_MBOX; i++) {
		if (vqid != ddata->mb[i].vq_id)
			continue;
		if (!ddata->mb[i].chan)
			return;
		err = mbox_send_message(ddata->mb[i].chan, "kick");
		if (err < 0)
			dev_err(&rproc->dev, "%s: failed (%s, err:%d)\n",
				__func__, ddata->mb[i].name, err);
		return;
	}
}

static struct rproc_ops spacemit_rproc_ops = {
	.prepare	= spacemit_rproc_prepare,
	.start		= spacemit_rproc_start,
	.stop		= spacemit_rproc_stop,
	.load		= rproc_elf_load_segments,
	.parse_fw	= spacemit_rproc_parse_fw,
	.kick		= spacemit_rproc_kick,
	.find_loaded_rsc_table = rproc_elf_find_loaded_rsc_table,
	.sanity_check	= rproc_elf_sanity_check,
	.get_boot_addr	= spacemit_get_boot_addr,
};

static int __process_theread(void *arg)
{
	int ret;
	struct mbox_client *cl = arg;
	struct rproc *rproc = dev_get_drvdata(cl->dev);
	struct spacemit_mbox *mb = container_of(cl, struct spacemit_mbox, client);
	struct sched_param param = {.sched_priority = 0 };

	mb->kthread_running = true;
	ret = sched_setscheduler(current, SCHED_FIFO, &param);
	set_freezable();

	do {
		try_to_freeze();
		wait_for_completion_timeout(&mb->mb_comp, 10);
		if (rproc_vq_interrupt(rproc, mb->vq_id) == IRQ_NONE)
			dev_dbg(&rproc->dev, "no message found in vq%d\n", mb->vq_id);
	} while (!kthread_should_stop());

	mb->kthread_running = false;

	return 0;
}
static void k1x_rproc_mb_callback(struct mbox_client *cl, void *data)
{
	struct spacemit_mbox *mb = container_of(cl, struct spacemit_mbox, client);

	complete(&mb->mb_comp);
}

static struct spacemit_mbox k1x_rpoc_mbox[] = {
	{
		.name = "vq0",
		.vq_id = K1X_MBOX_VQ0_ID,
		.client = {
			.rx_callback = k1x_rproc_mb_callback,
			.tx_block = true,
		},
	},
	{
		.name = "vq1",
		.vq_id = K1X_MBOX_VQ1_ID,
		.client = {
			.rx_callback = k1x_rproc_mb_callback,
			.tx_block = true,
		},
	},
};

#ifdef CONFIG_PM_SLEEP

#define STARTUP_MSG "pwr_management"

static struct rpmsg_device_id rpmsg_rcpu_pwr_management_id_table[] = {
	{ .name	= "rcpu-pwr-management-service", .driver_data = 0 },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, rpmsg_rcpu_pwr_management_id_table);

static int rpmsg_rcpu_pwr_cb(struct rpmsg_device *rpdev, void *data,
		int len, void *priv, u32 src)
{
	struct spacemit_rproc *srproc;

	if (strcmp(data, "pwr_management_ok") == 0) {
		pr_err("Connection create success\n");
		return 0;
	}

	srproc = dev_get_drvdata(&rpdev->dev);

	/* do something */

	return 0;
}

static int rpmsg_rcpu_pwr_manage_probe(struct rpmsg_device *rpdev)
{
	int ret;
	struct rproc *rproc;
	struct spacemit_rproc *srproc;
	struct platform_device *pdev;

	pdev = (struct platform_device *)rpmsg_rcpu_pwr_management_id_table[0].driver_data;

	rproc = platform_get_drvdata(pdev);
	srproc = rproc->priv;
	srproc->rpdev = rpdev;

	dev_set_drvdata(&rpdev->dev, srproc);

	dev_info(&rpdev->dev, "new channel: 0x%x -> 0x%x!\n",
					rpdev->src, rpdev->dst);

	ret = rpmsg_send(rpdev->ept, STARTUP_MSG, strlen(STARTUP_MSG));

	return 0;
}

static void rpmsg_rcpu_pwr_manage_romove(struct rpmsg_device *rpdev)
{
	dev_info(&rpdev->dev, "rpmsg rcpu power management driver is removed\n");
}

/* here we should register a endpoint for power-management */
static struct rpmsg_driver rpmsg_rcpu_pm_client = {
	.drv.name	= KBUILD_MODNAME,
	.id_table	= rpmsg_rcpu_pwr_management_id_table,
	.probe		= rpmsg_rcpu_pwr_manage_probe,
	.callback	= rpmsg_rcpu_pwr_cb,
	.remove		= rpmsg_rcpu_pwr_manage_romove,
};

module_rpmsg_driver(rpmsg_rcpu_pm_client);

#define RCPU_ENTER_LOW_PWR_MODE		"$"

static int rproc_platform_late(void)
{
	int ret;
	unsigned int val;
	struct rproc *rproc;
	struct rproc_mem_entry *src_table_mem;
	struct spacemit_rproc *srproc;
	struct platform_device *pdev;
	struct generic_pm_domain *genpd;

	pdev = (struct platform_device *)rpmsg_rcpu_pwr_management_id_table[0].driver_data;

	rproc = dev_get_drvdata(&pdev->dev);
	srproc = rproc->priv;

	/* send msg to rcpu to let it enter low power mode */
	ret = rpmsg_send(srproc->rpdev->ept, RCPU_ENTER_LOW_PWR_MODE,
			strlen(RCPU_ENTER_LOW_PWR_MODE));

	src_table_mem = rproc_find_carveout_by_name(rproc, "rsc_table");
	if (!src_table_mem) {
		pr_err("Failed to find the rcpu_mem_snapshots\n");
		return -1;
	}

	while (1) {
		/* will be wrotten by rpcu, using the reserved entry of resource table */
		val = readl(src_table_mem->va + 8);
		if (val == 1)
			break;
	}

	/* wait the rcpu enter wfi */
	mdelay(10);

	clk_set_rate(srproc->apb_clk, srproc->apb_clk_rate_default);

	genpd = pd_to_genpd(pdev->dev.pm_domain);

	pdev->dev.power.wakeup_path = false;

#ifdef CONFIG_HIBERNATION
	if (srproc->hinernate_restore)
		return 0;

	if (srproc->hibernate_freeze) {
		/* store the runtime memory */
		memcpy((void *)hibernate_rcpu_snapshot,
				(void *)hibernate_rcpu_runtime_vbase,
				hibernal_snapshot_size);

		arch_sync_dma_for_device(virt_to_phys(hibernate_rcpu_snapshot),
				hibernal_snapshot_size,
				DMA_TO_DEVICE);

	}
#endif
	val = readl(srproc->base[BOOTC_MEM_BASE_OFFSET] + AUDIO_PMU_VOTE_REG_OFFSET);
	/**
	 * set the vote first
	 * 0x6E: bit1:standbyen; bit2:sleepen; bit3:vctcxosd; bit5: ddrsd; bit6: axisd
	 */
	writel(val | 0x6e, srproc->base[BOOTC_MEM_BASE_OFFSET] + AUDIO_PMU_VOTE_REG_OFFSET);

	/* close the clk & power-switch */
	genpd->domain.ops.suspend_noirq(&pdev->dev);
	
	return 0;
}

static void rproc_platfrom_wake(void)
{
	unsigned int val;
	struct rproc *rproc;
	struct spacemit_rproc *srproc;
	struct rproc_mem_entry *src_table_mem;
	struct platform_device *pdev;
	struct generic_pm_domain *genpd;

	pdev = (struct platform_device *)rpmsg_rcpu_pwr_management_id_table[0].driver_data;

	rproc = dev_get_drvdata(&pdev->dev);
	srproc = rproc->priv;

	/* de-assert the clk */
	reset_control_assert(srproc->core_rst);
	reset_control_deassert(srproc->core_rst);

	genpd = pd_to_genpd(pdev->dev.pm_domain);
	/* enable the clk & power-switch */
	genpd->domain.ops.resume_noirq(&pdev->dev);

	/* set apb clk rate */
	clk_set_rate(srproc->apb_clk, srproc->apb_clk_rate);

	/* enable ipc2ap clk & reset--> rcpu side */
	writel(0xff, srproc->base[BOOTC_MEM_BASE_OFFSET] + ESOS_AON_PER_CLK_RST_CTL_REG);

	/* set the boot-entry and let the rcpu run to vector0 */
	writel(rproc->bootaddr & ~0xffff, srproc->base[SYSCTRL_MEM_BASE_OFFSET] + ESOS_BOOT_ENTRY_REG_OFFSET);

	/* set ddr map */
	writel(srproc->ddr_remap_base, srproc->base[SYSCTRL_MEM_BASE_OFFSET] + ESOS_DDR_REGMAP_BASE_REG_OFFSET);

#ifdef CONFIG_HIBERNATION
	if (srproc->hinernate_restore || srproc->hibernate_freeze) {
		arch_sync_dma_for_device(virt_to_phys(hibernate_rcpu_snapshot),
				hibernal_snapshot_size,
				DMA_FROM_DEVICE);

		/* copy the runtime memory */
		memcpy((void *)hibernate_rcpu_runtime_vbase,
				(void *)hibernate_rcpu_snapshot,
				hibernal_snapshot_size);
	}
#endif

	val = readl(srproc->base[BOOTC_MEM_BASE_OFFSET] + AUDIO_PMU_VOTE_REG_OFFSET);
	/**
	 * clear the vote first
	 * 0x6E: bit1:standbyen; bit2:sleepen; bit3:vctcxosd; bit5: ddrsd; bit6: axisd
	 */
	writel(val & ~0x6e, srproc->base[BOOTC_MEM_BASE_OFFSET] + AUDIO_PMU_VOTE_REG_OFFSET);

	/* luaching up rpcu */
	writel(1, srproc->base[BOOTC_MEM_BASE_OFFSET] + ESOS_BOOTUP_REG_OFFSET);

	src_table_mem = rproc_find_carveout_by_name(rproc, "rsc_table");
	if (!src_table_mem) {
		pr_err("Failed to find the rcpu_mem_snapshots\n");
		return;
	}

	while (1) {
		/* will be wrotten by rpcu: using the reserved entry of resource table */
		val = readl(src_table_mem->va + 8);
		if (val == 2)
			break;
	}
}

#ifdef CONFIG_HIBERNATION
static int rproc_platform_hibernate_pre_snapshot(void)
{
	return rproc_platform_late();
}

static void rproc_platform_hibernate_finish(void)
{
	rproc_platfrom_wake();
}
#endif

static struct platfrom_pm_ops rproc_platform_pm_ops = {
	.prepare_late = rproc_platform_late,
	.wake = rproc_platfrom_wake,
};

#ifdef CONFIG_HIBERNATION
static struct platfrom_pm_ops rproc_platform_hibernate_ops = {
	.pre_snapshot = rproc_platform_hibernate_pre_snapshot,
	.finish = rproc_platform_hibernate_finish,
};

static int hibernate_pm_notify(struct notifier_block *nb,
			     unsigned long mode, void *_unused)
{
	struct spacemit_rproc *priv = 
		container_of(nb, struct spacemit_rproc,
				pm_notifier);

	switch (mode) {
	case PM_RESTORE_PREPARE:
		priv->hinernate_restore = 1;
		break;
	case PM_POST_RESTORE:
		priv->hinernate_restore = 0;
		break;
	case PM_HIBERNATION_PREPARE:
		priv->hibernate_freeze = 1;
		break;
	case PM_POST_HIBERNATION:
		priv->hibernate_freeze = 0;
		break;
	default:
		break;
	}
	return 0;
}
#endif

static int spacemit_rproc_suspend(struct device *dev)
{
	/* this code do nothing but pretect the power & clk of audio 
	 * from closing in noirq process when system suspend
	 * */
	device_set_wakeup_path(dev);

	return 0;
}

static int spacemit_rproc_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops spacemit_rproc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(spacemit_rproc_suspend,
				spacemit_rproc_resume)
};
#endif

static int spacemit_rproc_probe(struct platform_device *pdev)
{
	int ret, i;
	const char *name;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	const char *fw_name = "esos.elf";
	struct spacemit_rproc *priv;
	struct mbox_client *cl;
	struct rproc *rproc;
#ifdef CONFIG_HIBERNATION
	struct of_phandle_iterator it;
	struct reserved_mem *rmem;
#endif

	ret = rproc_of_parse_firmware(dev, 0, &fw_name);
	if (ret < 0 && ret != -EINVAL)
		return ret;

	rproc = devm_rproc_alloc(dev, np->name, &spacemit_rproc_ops,
				fw_name, sizeof(*priv));
	if (!rproc)
		return -ENOMEM;

	priv = rproc->priv;
	priv->dev = dev;

	priv->base[BOOTC_MEM_BASE_OFFSET] = devm_platform_ioremap_resource(pdev, BOOTC_MEM_BASE_OFFSET);
	if (IS_ERR(priv->base[BOOTC_MEM_BASE_OFFSET])) {
		ret = PTR_ERR(priv->base[BOOTC_MEM_BASE_OFFSET]);
		dev_err(dev, "failed to get reg base\n");
		ret = -EINVAL;
		goto err_0;
	}

	priv->base[SYSCTRL_MEM_BASE_OFFSET] = devm_platform_ioremap_resource(pdev, SYSCTRL_MEM_BASE_OFFSET);
	if (IS_ERR(priv->base[SYSCTRL_MEM_BASE_OFFSET])) {
		ret = PTR_ERR(priv->base[SYSCTRL_MEM_BASE_OFFSET]);
		dev_err(dev, "failed to get reg base\n");
		ret = -EINVAL;
		goto err_0;
	}

	priv->core_rst = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(priv->core_rst)) {
		ret = PTR_ERR(priv->core_rst);
		dev_err_probe(dev, ret, "fail to acquire rproc reset\n");
		ret = -EINVAL;
		goto err_0;
	}

	priv->core_clk = devm_clk_get(dev, "core");
	if (IS_ERR(priv->core_clk)) {
		ret = PTR_ERR(priv->core_clk);
		dev_err(dev, "failed to acquire rpoc core\n");
		ret = -EINVAL;
		goto err_0;
	}

	priv->apb_clk = devm_clk_get(dev, "apb");
	if (IS_ERR(priv->apb_clk)) {
		ret = PTR_ERR(priv->apb_clk);
		dev_err(dev, "failed to acquire rpoc apb clk\n");
		ret = -EINVAL;
		goto err_0;
	}

	/* get the apb clk rate */
	ret = of_property_read_u32(np, "apb-clk-rate", &priv->apb_clk_rate);
	if (ret) {
		dev_err(dev, "failed to acquire rpoc apb clk rate\n");
		ret = -EINVAL;
		goto err_0;
	}

	/* get the ddr-remap base */
	ret = of_property_read_u32(pdev->dev.of_node, "ddr-remap-base", &priv->ddr_remap_base);

	pm_runtime_enable(dev);

	platform_set_drvdata(pdev, rproc);

	/* get the mailbox */
	priv->mb = k1x_rpoc_mbox;

	for (i = 0; i < MAX_MBOX; ++i) {
		name = priv->mb[i].name;

		cl = &priv->mb[i].client;
		cl->dev = dev;
		init_completion(&priv->mb[i].mb_comp);

		priv->mb[i].chan = mbox_request_channel_byname(cl, name);
		if (IS_ERR(priv->mb[i].chan)) {
			dev_err(dev, "failed to request mbox channel\n");
			ret = -EINVAL;
			goto err_1;
		}

		if (priv->mb[i].vq_id >= 0) {
			priv->mb[i].mb_thread = kthread_run(__process_theread, (void *)cl, name);
			if (IS_ERR(priv->mb[i].mb_thread)) {
				ret = PTR_ERR(priv->mb[i].mb_thread);
				goto err_1;
			}
		}
	}

#ifdef CONFIG_PM_SLEEP
	rpmsg_rcpu_pwr_management_id_table[0].driver_data = (unsigned long long)pdev;

	register_platform_pm_ops(&rproc_platform_pm_ops);
#ifdef CONFIG_HIBERNATION
	register_platform_hibernate_pm_ops(&rproc_platform_hibernate_ops);

	priv->pm_notifier.notifier_call = hibernate_pm_notify; 
	ret = register_pm_notifier(&priv->pm_notifier);
#endif
#endif

	ret = devm_rproc_add(dev, rproc);
	if (ret) {
		dev_err(dev, "rproc_add failed\n");
		ret = -EINVAL;
		goto err_1;
	}

	ret = rproc_boot(rproc);
	if (ret) {
		dev_err(dev, "rproc_boot failed\n");
		ret = -EINVAL;
		goto err_1;
	}

#ifdef CONFIG_HIBERNATION
	of_phandle_iterator_init(&it, np, "memory-region", NULL, 0);
	while (of_phandle_iterator_next(&it) == 0) {
		rmem = of_reserved_mem_lookup(it.node);
		if (!rmem) {
			dev_err(dev,
				"unable to acquire memory-region\n");
			ret = -EINVAL;
			goto err_2;
		}

		if (strcmp(it.node->name, "vdev0buffer") == 0 ||
				strcmp(it.node->name, "vdev0vring1") == 0 ||
				strcmp(it.node->name, "vdev0vring0") == 0 ||
				strcmp(it.node->name, "rcpu_mem_heap") == 0 ||
				strcmp(it.node->name, "rsc_table") == 0 ||
				strcmp(it.node->name, "rcpu_mem_0") == 0 ) {

			hibernal_snapshot_size += rmem->size;
			if (strcmp(it.node->name, "rcpu_mem_heap") == 0)
				hibernate_rcpu_runtime_pbase = (void *)rmem->base;

		}
	}

	if (hibernal_snapshot_size != 0) {
		/* this is used for store the memory snapshot for rcpu runtime */
		hibernate_rcpu_snapshot = kmalloc(hibernal_snapshot_size, GFP_KERNEL);
		if (!hibernate_rcpu_snapshot) {
			dev_err(dev, "malloc hibernate mem snapshot failed\n");
			ret = -ENOMEM;
			goto err_2;
		}

		/* the rcpu memory runtime */
		hibernate_rcpu_runtime_vbase = ioremap((phys_addr_t)hibernate_rcpu_runtime_pbase, hibernal_snapshot_size);
		if (!hibernate_rcpu_runtime_vbase) {
			dev_err(dev, "map hibernate run time memory error\n");
			ret = -ENOMEM;
			goto err_3;
		}
	}
#endif
	return 0;
#ifdef CONFIG_HIBERNATION
err_3:
	kfree(hibernate_rcpu_snapshot);
err_2:
#endif
	rproc_shutdown(rproc);
err_1:
	while (--i >= 0) {
		if (priv->mb[i].chan)
			mbox_free_channel(priv->mb[i].chan);
		if (priv->mb[i].mb_thread)
			kthread_stop(priv->mb[i].mb_thread);
	}
err_0:
	rproc_free(rproc);

	return ret;
}

static void k1x_rproc_free_mbox(struct rproc *rproc)
{
	struct spacemit_rproc *ddata = rproc->priv;
	unsigned int i;

	for (i = 0; i < MAX_MBOX; i++) {
		if (ddata->mb[i].chan)
			mbox_free_channel(ddata->mb[i].chan);
		ddata->mb[i].chan = NULL;
	}
}

static int spacemit_rproc_remove(struct platform_device *pdev)
{
	int i = 0;
	struct rproc *rproc = platform_get_drvdata(pdev);
	struct spacemit_rproc *ddata = rproc->priv;

	for (i = 0; i < MAX_MBOX; ++i)
		if (ddata->mb[i].kthread_running)
			kthread_stop(ddata->mb[i].mb_thread);

	rproc_del(rproc);
	k1x_rproc_free_mbox(rproc);
	rproc_free(rproc);

#ifdef CONFIG_PM_SLEEP
	unregister_rpmsg_driver(&rpmsg_rcpu_pm_client);
	unregister_platform_pm_ops(&rproc_platform_pm_ops);
#ifdef CONFIG_HIBERNATION
	unregister_pm_notifier(&ddata->pm_notifier);
	unregister_platform_hibernate_pm_ops(&rproc_platform_hibernate_ops);
	iounmap(hibernate_rcpu_runtime_vbase);
	kfree(hibernate_rcpu_snapshot);
#endif
#endif
	return 0;
}

static const struct of_device_id spacemit_rproc_of_match[] = {
	{ .compatible = "spacemit,k1-x-rproc" },
	{},
};

MODULE_DEVICE_TABLE(of, spacemit_rproc_of_match);

static void spacemit_rproc_shutdown(struct platform_device *pdev)
{
	int i;
	struct rproc *rproc;
	struct spacemit_rproc *priv;

	rproc = dev_get_drvdata(&pdev->dev);
	priv = rproc->priv;

	for (i = 0; i < MAX_MBOX; ++i) {
		/* release the resource of rt thread */
		if (priv->mb[i].kthread_running) {
			if (!frozen((priv->mb[i].mb_thread)))
				kthread_stop(priv->mb[i].mb_thread);
		}
		/* mbox_free_channel(priv->mb[i].chan); */
	}
}

static struct platform_driver spacemit_rproc_driver = {
	.probe = spacemit_rproc_probe,
	.remove = spacemit_rproc_remove,
	.shutdown = spacemit_rproc_shutdown,
	.driver = {
		.name = "spacemit-rproc",
#ifdef CONFIG_PM_SLEEP
		.pm	= &spacemit_rproc_pm_ops,
#endif
		.of_match_table = spacemit_rproc_of_match,
	},
};

static __init int spacemit_rproc_driver_init(void)
{
	return platform_driver_register(&spacemit_rproc_driver);
}
device_initcall(spacemit_rproc_driver_init);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("sapcemit remote processor control driver");
