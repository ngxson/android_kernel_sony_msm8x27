/*
 * drivers/input/touchscreen/doubletap2wake.c
 *
 *
 * Copyright (c) 2013, Dennis Rassmann <showp1984@gmail.com>
 *           (C) 2014 LoungeKatt <twistedumbrella@gmail.com>
 *	     (c) 2014 redlee90 <redlee90@gmail.com> 	
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/input/doubletap2wake.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#ifdef CONFIG_POWERSUSPEND
#include <linux/powersuspend.h>
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/wakelock.h>

/* Version, author, desc, etc */
#define DRIVER_AUTHOR "Ngxson"
#define DRIVER_DESCRIPTION "DoubleTap2Wake and Swipe2wake for Sony Xperia M"
#define DRIVER_VERSION "6.9 just kidding"
#define LOGTAG "[doubletap2wake]: "


/* Resources */
int dt2w_switch = 0;
int dt2w_vib = 1;
int s2w_switch = 0;
int s2m = 0;
bool s2m_reverse = false;
bool scr_suspended = false;
bool nui_report_input = true;
bool dt2w_debug = false;
bool no_suspend_touch = false;
int s2w_oneswipe = 0;

/* Read cmdline for dt2w */
static int __init read_dt2w_cmdline(char *dt2w)
{
	if (strcmp(dt2w, "1") == 0) {
		printk("[cmdline_dt2w]: DoubleTap2Wake enabled. \
			| dt2w='%s'\n", dt2w);
		dt2w_switch = 1;
	} else if (strcmp(dt2w, "2") == 0) {
		printk("[cmdline_dt2w]: DoubleTap2Wake enabled. \
			| dt2w='%s'\n", dt2w);
		dt2w_switch = 2;
	} else if (strcmp(dt2w, "0") == 0) {
		printk("[cmdline_dt2w]: DoubleTap2Wake disabled. \
			| dt2w='%s'\n", dt2w);
		dt2w_switch = 0;
	} else {
		printk("[cmdline_dt2w]: No valid input found. \
			Going with default: | dt2w='%u'\n", dt2w_switch);
	}
	return 0;
}
__setup("dt2w=", read_dt2w_cmdline);

/*
 * SYSFS stuff below here
 */
static ssize_t dt2w_doubletap2wake_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", dt2w_switch);

	return count;
}

static ssize_t dt2w_doubletap2wake_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int value;

	if (sysfs_streq(buf, "0"))
		value = 0;
	else if (sysfs_streq(buf, "1"))
		value = 1;
	else if (sysfs_streq(buf, "2"))
		value = 2;
	else
		return -EINVAL;
	if (dt2w_switch != value) {
		// dt2w_switch is safe to be changed only when !scr_suspended
		if (scr_suspended) {
			//dt2w_reset();
			//doubletap2wake_pwrswitch();
			msleep(400);
		}
		if (!scr_suspended) {
			dt2w_switch = value;
			if((s2w_switch > 0) || (dt2w_switch > 0) || (s2m > 0)) no_suspend_touch = true;
			else no_suspend_touch = false;
		}
	}
	return count;
}

static DEVICE_ATTR(doubletap2wake, (S_IWUGO|S_IRUGO),
	dt2w_doubletap2wake_show, dt2w_doubletap2wake_dump);

static ssize_t doubletap2wake_vib_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", dt2w_vib);

	return count;
}

static ssize_t doubletap2wake_vib_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int value;

	if (sysfs_streq(buf, "0"))
		value = 0;
	else if (sysfs_streq(buf, "1"))
		value = 1;
	else
		return -EINVAL;
	if (dt2w_vib != value) {
		dt2w_vib = value;
	}
	return count;
}

static DEVICE_ATTR(doubletap2wake_vib, (S_IWUGO|S_IRUGO),
	doubletap2wake_vib_show, doubletap2wake_vib_dump);
	
static ssize_t dt2w_debug_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;
	
	return count;
}

static ssize_t dt2w_debug_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	if (sysfs_streq(buf, "0"))
		dt2w_debug = false;
	else if (sysfs_streq(buf, "1"))
		dt2w_debug = true;
	else
		return -EINVAL;

	return count;
}

static DEVICE_ATTR(debug, (S_IWUGO|S_IRUGO),
	dt2w_debug_show, dt2w_debug_dump);

static ssize_t s2w_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", s2w_switch);

	return count;
}

static ssize_t s2w_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int value;

	if (sysfs_streq(buf, "0"))
		value = 0;
	else if (sysfs_streq(buf, "1"))
		value = 1;
	else
		return -EINVAL;
	if (s2w_switch != value) {
		if (!scr_suspended) {
			s2w_switch = value;
			if((s2w_switch > 0) || (dt2w_switch > 0) || (s2m > 0)) no_suspend_touch = true;
			else no_suspend_touch = false;
		}
	}
	return count;
}

static DEVICE_ATTR(swipe2wake, (S_IWUGO|S_IRUGO),
	s2w_show, s2w_dump);
	
//media control
static ssize_t s2m_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;
	count += sprintf(buf, "%d\n", s2m);
	return count;
}

static ssize_t s2m_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{

	if (sysfs_streq(buf, "0"))
		s2m = 0;
	else if (sysfs_streq(buf, "1"))
		s2m = 1;
	else
		return -EINVAL;
		
	if((s2w_switch > 0) || (dt2w_switch > 0) || (s2m > 0)) no_suspend_touch = true;
	else no_suspend_touch = false;

	return count;
}

static DEVICE_ATTR(swipe2media, (S_IWUGO|S_IRUGO),
	s2m_show, s2m_dump);
	
static ssize_t s2m_reverse_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;
	if(s2m_reverse) count += sprintf(buf, "1\n");
	else count += sprintf(buf, "0\n");
	return count;
}

static ssize_t s2m_reverse_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	if (sysfs_streq(buf, "0"))
		s2m_reverse = false;
	else if (sysfs_streq(buf, "1"))
		s2m_reverse = true;
	else
		return -EINVAL;
	return count;
}

static DEVICE_ATTR(swipe2media_reverse, (S_IWUGO|S_IRUGO),
	s2m_reverse_show, s2m_reverse_dump);

/*
 * INIT / EXIT stuff below here
 */
//extern struct kobject *android_touch_kobj;
struct kobject *android_touch_kobj;
EXPORT_SYMBOL_GPL(android_touch_kobj);

static int __init doubletap2wake_init(void)
{
	int rc = 0;
    android_touch_kobj = kobject_create_and_add("android_touch", NULL) ;
    if (android_touch_kobj == NULL) {
        pr_warn("%s: android_touch_kobj create_and_add failed\n", __func__);
    }
    rc = sysfs_create_file(android_touch_kobj, &dev_attr_doubletap2wake.attr);
    if (rc) {
        pr_warn("%s: sysfs_create_file failed for doubletap2wake\n", __func__);
    }
    rc = sysfs_create_file(android_touch_kobj, &dev_attr_swipe2wake.attr);
    if (rc) {
        pr_warn("%s: sysfs_create_file failed for swipe2wake\n", __func__);
    }
    rc = sysfs_create_file(android_touch_kobj, &dev_attr_doubletap2wake_vib.attr);
    if (rc) {
        pr_warn("%s: sysfs_create_file failed for doubletap2wake_vib\n", __func__);
    }
    rc = sysfs_create_file(android_touch_kobj, &dev_attr_swipe2media.attr);
    rc = sysfs_create_file(android_touch_kobj, &dev_attr_swipe2media_reverse.attr);
    if (rc) {
        pr_warn("%s: sysfs_create_file failed for swipe2media\n", __func__);
    }
    rc = sysfs_create_file(android_touch_kobj, &dev_attr_debug.attr);

	return 0;
}

static void __exit doubletap2wake_exit(void)
{
    kobject_del(android_touch_kobj);
	return;
}

module_init(doubletap2wake_init);
module_exit(doubletap2wake_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESCRIPTION);
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPLv2");
