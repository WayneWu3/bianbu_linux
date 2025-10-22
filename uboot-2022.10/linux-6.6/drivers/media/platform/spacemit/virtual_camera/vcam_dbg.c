// SPDX-License-Identifier: GPL-2.0
/*
 * vcam_dbg.c - vcamera debug utility
 *
 * Copyright(C) 2025 SPACEMIT Micro Limited
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include "vcam_dbg.h"

static uint debug_mdl = 0x0;
static uint debug_loop_prink = 0x0;

void vcam_printk(const char *vcam_level, const char *kern_level,
		const char *func, int line, const char *format, ...)
{
	struct va_format vaf;
	va_list args;

	va_start(args, format);

	vaf.fmt = format;
	vaf.va = &args;

	printk("%s" "%s: %s %d: %pV\n", kern_level, vcam_level, func, line, &vaf);
	va_end(args);
}

EXPORT_SYMBOL(vcam_printk);

void vcam_debug(const char *vcam_level, const char *func, int line, const char *format, ...)
{
	struct va_format vaf;
	va_list args;

	if (debug_loop_prink == 0)
		return;

	va_start(args, format);

	vaf.fmt = format;
	vaf.va = &args;

	pr_debug("%s: %s %d: %pV\n", vcam_level, func, line, &vaf);
	va_end(args);
}

EXPORT_SYMBOL(vcam_debug);

module_param(debug_mdl, uint, 0644);
module_param(debug_loop_prink, uint, 0644);
