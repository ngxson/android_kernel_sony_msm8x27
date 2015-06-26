/*
 * drivers/accessibility/nuisetting.c
 *
 *
 * Copyright (c) 2013, Nguyen Xuan Son <thichthat@gmail.com>	
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
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/nuisetting.h>

#define DRIVER_AUTHOR "Nguyen Xuan Son <thichthat@gmail.com>"
#define DRIVER_DESCRIPTION "Settings for Nui kernel"
#define DRIVER_VERSION "1.0"
#define LOGTAG "[ngxson]: "

int nbr_switch = 1;
int level_short_switch = 0;
int zero_precent_switch = 0;
int nui_batt_level = 100;

/*
 * SYSFS stuff below here
 */
//brightness tool
static ssize_t nui_brightness_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", nbr_switch);

	return count;
}

static ssize_t nui_brightness_dump(struct device *dev,
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
	if (nbr_switch != value) {
		nbr_switch = value;
	}
	return count;
}
static DEVICE_ATTR(nuibrightness, (S_IWUSR|S_IRUGO),
	nui_brightness_show, nui_brightness_dump);

//short vibration setting
static ssize_t nui_level_short_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", level_short_switch);

	return count;
}

static ssize_t nui_level_short_dump(struct device *dev,
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
	if (level_short_switch != value) {
		level_short_switch = value;
	}
	return count;
}
static DEVICE_ATTR(level_short, (S_IWUSR|S_IRUGO),
	nui_level_short_show, nui_level_short_dump);

//Prevent 0% battery level
static ssize_t nui_zeroprecent_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", zero_precent_switch);

	return count;
}

static ssize_t nui_zeroprecent_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int value;

	if (sysfs_streq(buf, "0"))
		value = 0;
	else if (sysfs_streq(buf, "1"))
		value = 1;
	else
		return -EINVAL;
	if (zero_precent_switch != value) {
		zero_precent_switch = value;
	}
	return count;
}
static DEVICE_ATTR(zeroprecent, (S_IWUSR|S_IRUGO),
	nui_zeroprecent_show, nui_zeroprecent_dump);
/*
 * INIT / EXIT stuff below here
 */
//extern struct kobject *nui_setting_kobj;
struct kobject *nui_setting_kobj;
EXPORT_SYMBOL_GPL(nui_setting_kobj);

static int __init nuisetting_init(void)
{
	int rc = 0;

    nui_setting_kobj = kobject_create_and_add("nui", NULL) ;
    if (nui_setting_kobj == NULL) {
        pr_warn("%s: nui_setting_kobj create_and_add failed\n", __func__);
    }
    rc = sysfs_create_file(nui_setting_kobj, &dev_attr_nuibrightness.attr);
    if (rc) {
        pr_warn("%s: sysfs_create_file failed for nuibrightness\n", __func__);
    }
    rc = sysfs_create_file(nui_setting_kobj, &dev_attr_level_short.attr);
    if (rc) {
        pr_warn("%s: sysfs_create_file failed for level_short\n", __func__);
    }
    rc = sysfs_create_file(nui_setting_kobj, &dev_attr_zeroprecent.attr);
    if (rc) {
        pr_warn("%s: sysfs_create_file failed for zeroprecent\n", __func__);
    }
	pr_info(LOGTAG"%s done\n", __func__);

	return 0;
}

static void __exit nuisetting_exit(void)
{
    kobject_del(nui_setting_kobj);
	return;
}

module_init(nuisetting_init);
module_exit(nuisetting_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESCRIPTION);
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPLv2");
