/* SPDX-License-Identifier: GPL-2.0 */
/*
 * vcam_dbg.h - vcamera debug utility
 *
 * Copyright(C) 2025 SPACEMIT Micro Limited
 */
#ifndef __VCAM_DBG_H__
#define __VCAM_DBG_H__

#include <linux/printk.h>

__printf(5, 6)
void vcam_printk(const char *vcam_level, const char *kern_level,
		const char *func, int line, const char *format, ...);

__printf(4, 5)
void vcam_debug(const char *vcam_level, const char *func, int line, const char *format, ...);

/**
 * vcamera error output.
 *
 * @format: printf() like format string.
 */
#define vcam_err(format, ...)                                       \
	vcam_printk("vcam_err", KERN_ERR,                \
			 __func__, __LINE__, format, ##__VA_ARGS__)

/**
 * vcamera warning output.
 *
 * @format: printf() like format string.
 */
#define vcam_warn(format, ...)                                  \
	vcam_printk("vcam_wrn", KERN_WARNING,        \
			 __func__, __LINE__, format, ##__VA_ARGS__)

/**
 * vcamera notice output.
 *
 * @format: printf() like format string.
 */
#define vcam_not(format, ...)                                    \
	vcam_printk("vcam_not", KERN_NOTICE,          \
			 __func__, __LINE__, format, ##__VA_ARGS__)

/**
 * vcamera information output.
 *
 * @format: printf() like format string.
 */
#define vcam_info(format, ...)                                     \
	vcam_printk("vcam_inf", KERN_INFO,              \
			 __func__, __LINE__, format, ##__VA_ARGS__)

/**
 * vcamera debug output.
 *
 * @format: printf() like format string.
 */
#define vcam_dbg(format, ...)                                      \
	vcam_debug("vcam_dbg", __func__, __LINE__, format, ##__VA_ARGS__)

#endif /* ifndef __VCAM_DBG_H__ */
