/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __PSTORE_BLK_H_
#define __PSTORE_BLK_H_

#include <linux/types.h>
#include <linux/pstore.h>
#include <linux/pstore_zone.h>

/**
 * struct pstore_device_info - back-end pstore/blk driver structure.
 *
 * @flags:	Refer to macro starting with PSTORE_FLAGS defined in
 *		linux/pstore.h. It means what front-ends this device support.
 *		Zero means all backends for compatible.
 * @zone:	The struct pstore_zone_info details.
 *
 */
struct pstore_device_info {
	unsigned int flags;
	struct pstore_zone_info zone;
};

int  register_pstore_device(struct pstore_device_info *dev);
void unregister_pstore_device(struct pstore_device_info *dev);

/**
 * struct pstore_blk_config - the pstore_blk backend configuration
 *
 * @device:		Name of the desired block device
 * @max_reason:		Maximum kmsg dump reason to store to block device
 * @kmsg_size:		Total size of for kmsg dumps
 * @pmsg_size:		Total size of the pmsg storage area
 * @console_size:	Total size of the console storage area
 * @ftrace_size:	Total size for ftrace logging data (for all CPUs)
 */
struct pstore_blk_config {
	char device[80];
	enum kmsg_dump_reason max_reason;
	unsigned long kmsg_size;
	unsigned long pmsg_size;
	unsigned long console_size;
	unsigned long ftrace_size;
};

#ifdef CONFIG_MMC_SDHCI_OF_K1X_PANIC
struct bdev_info {
	struct block_device *bdev;
	dev_t devt;
	sector_t nr_sects;
	sector_t start_sect;
};
#endif
/**
 * pstore_blk_get_config - get a copy of the pstore_blk backend configuration
 *
 * @info:	The sturct pstore_blk_config to be filled in
 *
 * Failure returns negative error code, and success returns 0.
 */
int pstore_blk_get_config(struct pstore_blk_config *info);
enum pstore_blk_notifier_type {
	PSTORE_BLK_BACKEND_REGISTER = 1,
	PSTORE_BLK_BACKEND_PANIC_DRV_REGISTER,
	PSTORE_BLK_BACKEND_UNREGISTER,
	PSTORE_BLK_BACKEND_PANIC_DRV_UNREGISTER,
};

typedef	int (*pstore_blk_notifier_fn_t)(enum pstore_blk_notifier_type type,
		struct pstore_device_info *dev);
struct pstore_blk_notifier {
	struct notifier_block nb;
	pstore_blk_notifier_fn_t notifier_call;
};

extern int register_pstore_device(struct pstore_device_info *dev);
extern void unregister_pstore_device(struct pstore_device_info *dev);

extern int register_pstore_blk_panic_notifier(struct pstore_blk_notifier *pbn);
extern void unregister_pstore_blk_panic_notifier(struct pstore_blk_notifier *nb);
#endif
