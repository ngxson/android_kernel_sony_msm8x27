/*
 * drivers/accessibility/nuisetting.c
 *
 * Copyright (c) 2015, Nguyen Xuan Son <thichthat@gmail.com>	
 *
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
#include <../video/msm/msm_fb.h>

#define DRIVER_AUTHOR "Nguyen Xuan Son <thichthat@gmail.com>"
#define DRIVER_DESCRIPTION "Settings for Nui kernel"
#define DRIVER_VERSION "1.0"
#define LOGTAG "[ngxson]: "

int nbr_switch = 1;
int level_short_switch = 0;
int zero_precent_switch = 0;
int nui_batt_level = 100;
int nui_batt_sav = 0;
int camera_key = 766;
int focus_key = 528;
bool key_scroff = false;
int camera_key_scroff = 766;
int focus_key_scroff = 528;
int nui_torch_intensity = 2;
int nui_proximity_sens = 0;
int brlock = 0;
int nui_which_vol = 0;
bool focus2torch = false;
bool hn_enable = false;
bool cam_gain = false;

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
static DEVICE_ATTR(nuibrightness, (S_IWUGO|S_IRUGO),
	nui_brightness_show, nui_brightness_dump);

//Torch intensity
static ssize_t nui_torch_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", nui_torch_intensity);

	return count;
}

static ssize_t nui_torch_dump(struct device *dev,
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
	if (nui_torch_intensity != value) {
		nui_torch_intensity = value;
	}
	return count;
}
static DEVICE_ATTR(torch, (S_IWUGO|S_IRUGO),
	nui_torch_show, nui_torch_dump);

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
static DEVICE_ATTR(level_short, (S_IWUGO|S_IRUGO),
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
static DEVICE_ATTR(zeroprecent, (S_IWUGO|S_IRUGO),
	nui_zeroprecent_show, nui_zeroprecent_dump);
	
//nui_proximity_sensitive
static ssize_t nui_proxi_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", nui_proximity_sens);

	return count;
}

static ssize_t nui_proxi_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int value;

	if (sysfs_streq(buf, "0"))
		value = 0;
	else if (sysfs_streq(buf, "1"))
		value = 1;
	else
		return -EINVAL;
	if (nui_proximity_sens != value) {
		nui_proximity_sens = value;
		nui_proximity_sensitive(value);
	}
	return count;
}
static DEVICE_ATTR(proximitysens, (S_IWUGO|S_IRUGO),
	nui_proxi_show, nui_proxi_dump);
	
//Battery Saving mode
static ssize_t nui_nooc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", nui_batt_sav);

	return count;
}

static ssize_t nui_nooc_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int value;

	if (sysfs_streq(buf, "0"))
		value = 0;
	else if (sysfs_streq(buf, "1"))
		value = 1;
	else if (sysfs_streq(buf, "2"))
		value = 2;
	else if (sysfs_streq(buf, "3"))
		value = 3;
	else
		return -EINVAL;
	if (nui_batt_sav != value) {
		nui_batt_sav = value;
		nui_batt_sav_mode(nui_batt_sav);
	}
	return count;
}
static DEVICE_ATTR(nooc, (S_IWUGO|S_IRUGO),
	nui_nooc_show, nui_nooc_dump);
	
//Logo tool
static ssize_t nui_logo_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;
	count += sprintf(buf, "%d\n", 0);
	return count;
}

static ssize_t nui_logo_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int value;

	if (sysfs_streq(buf, "0"))
		value = 0;
	else if (sysfs_streq(buf, "1"))
		value = 1;
	else if (sysfs_streq(buf, "2"))
		value = 2;
	else if (sysfs_streq(buf, "3"))
		value = 3;
	else
		return -EINVAL;
	
	draw_nui_logo(value);
	
	return count;
}
static DEVICE_ATTR(logo, (S_IWUGO|S_IRUGO),
	nui_logo_show, nui_logo_dump);
	
// camera key setting 
static ssize_t camera_key_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", camera_key);
}

static ssize_t camera_key_dump(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{

	int val;
	int rc;

	rc = kstrtoint(buf, 10, &val);
	if (rc) return -EINVAL;

	if((val == KEY_POWER) ||
		(val == KEY_CAMERA_SNAPSHOT) ||
		(val == KEY_CAMERA_FOCUS) ||
		(val == KEY_BACK) ||
		(val == KEY_NEXTSONG) ||
		(val == KEY_PLAYPAUSE) ||
		(val == KEY_PREVIOUSSONG) ||
		(val == KEY_SEARCH) ||
		(val == KEY_MENU) ||
		(val == KEY_HOMEPAGE) ||
		(val == KEY_VOLUMEDOWN) ||
		(val == KEY_VOLUMEUP) ||
		(val == 580) ||
		(val == 0)) {
				camera_key = val;
		} else return -EINVAL;

	return strnlen(buf, count);
}

static DEVICE_ATTR(camera_key, (S_IWUGO|S_IRUGO),
	camera_key_show, camera_key_dump);

// focus key setting 
static ssize_t focus_key_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", focus_key);
}

static ssize_t focus_key_dump(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{

	int val;
	int rc;

	rc = kstrtoint(buf, 10, &val);
	if (rc) return -EINVAL;
	
	if((val == KEY_POWER) ||
		(val == KEY_CAMERA_SNAPSHOT) ||
		(val == KEY_CAMERA_FOCUS) ||
		(val == KEY_BACK) ||
		(val == KEY_NEXTSONG) ||
		(val == KEY_PLAYPAUSE) ||
		(val == KEY_PREVIOUSSONG) ||
		(val == KEY_SEARCH) ||
		(val == KEY_MENU) ||
		(val == KEY_HOMEPAGE) ||
		(val == KEY_VOLUMEDOWN) ||
		(val == KEY_VOLUMEUP) ||
		(val == 580) ||
		(val == 0)) {
				focus_key = val;
		} else return -EINVAL;

	return strnlen(buf, count);
}

static DEVICE_ATTR(focus_key, (S_IWUGO|S_IRUGO),
	focus_key_show, focus_key_dump);
	
// camera key setting scr off
static ssize_t camera_key_scroff_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", camera_key_scroff);
}

static ssize_t camera_key_scroff_dump(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{

	int val;
	int rc;

	rc = kstrtoint(buf, 10, &val);
	if (rc) return -EINVAL;

	if((val == KEY_POWER) ||
		(val == KEY_CAMERA_SNAPSHOT) ||
		(val == KEY_CAMERA_FOCUS) ||
		(val == KEY_BACK) ||
		(val == KEY_NEXTSONG) ||
		(val == KEY_PLAYPAUSE) ||
		(val == KEY_PREVIOUSSONG) ||
		(val == KEY_SEARCH) ||
		(val == KEY_MENU) ||
		(val == KEY_HOMEPAGE) ||
		(val == 580) ||
		(val == 0)) {
				camera_key_scroff = val;
		} else return -EINVAL;

	return strnlen(buf, count);
}

static DEVICE_ATTR(camera_key_scroff, (S_IWUGO|S_IRUGO),
	camera_key_scroff_show, camera_key_scroff_dump);

// focus key setting scr off
static ssize_t focus_key_scroff_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", focus_key_scroff);
}

static ssize_t focus_key_scroff_dump(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{

	int val;
	int rc;

	rc = kstrtoint(buf, 10, &val);
	if (rc) return -EINVAL;
	
	if((val == KEY_POWER) ||
		(val == KEY_CAMERA_SNAPSHOT) ||
		(val == KEY_CAMERA_FOCUS) ||
		(val == KEY_BACK) ||
		(val == KEY_NEXTSONG) ||
		(val == KEY_PLAYPAUSE) ||
		(val == KEY_PREVIOUSSONG) ||
		(val == KEY_SEARCH) ||
		(val == KEY_MENU) ||
		(val == KEY_HOMEPAGE) ||
		(val == 580) ||
		(val == 0)) {
				focus_key_scroff = val;
		} else return -EINVAL;

	return strnlen(buf, count);
}

static DEVICE_ATTR(focus_key_scroff, (S_IWUGO|S_IRUGO),
	focus_key_scroff_show, focus_key_scroff_dump);
	
// key setting scr off
static ssize_t key_scroff_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", key_scroff);
}

static ssize_t key_scroff_dump(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{

	int val;
	int rc;

	rc = kstrtoint(buf, 10, &val);
	if (rc) return -EINVAL;
	
	if(val == 1) {
			key_scroff = true;
		} else if (val == 0) {
			key_scroff = false;
		} else return -EINVAL;

	return strnlen(buf, count);
}

static DEVICE_ATTR(key_scroff, (S_IWUGO|S_IRUGO),
	key_scroff_show, key_scroff_dump);
	
// lock brightness
static ssize_t brlock_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	
	return snprintf(buf, PAGE_SIZE, "%d\n", brlock);
}

static ssize_t brlock_dump(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{

	int val;
	int rc;

	rc = kstrtoint(buf, 10, &val);
	if (rc) return -EINVAL;
	
	if((val > -1) && (val < 256)) {
			brlock = val;
			if (val != 0) nui_set_brightness(val);
	} else return -EINVAL;

	return strnlen(buf, count);
}
static DEVICE_ATTR(brlock, (S_IWUGO|S_IRUGO),
	brlock_show, brlock_dump);
	
// choose twrp (detect vol key)
static ssize_t which_vol_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", nui_which_vol);
}

static ssize_t which_vol_dump(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{

	int val;
	int rc;

	rc = kstrtoint(buf, 10, &val);
	if (rc) return -EINVAL;
	if(val == 0) {
			nui_which_vol = 0; /* reset the value */
	} else return -EINVAL;
	return strnlen(buf, count);
}

static DEVICE_ATTR(which_vol, (S_IWUGO|S_IRUGO),
	which_vol_show, which_vol_dump);
	
// focus2torch
static ssize_t focus2torch_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", focus2torch);
}

static ssize_t focus2torch_dump(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{

	int val;
	int rc;

	rc = kstrtoint(buf, 10, &val);
	if (rc) return -EINVAL;
	if(val == 1) {
			focus2torch = true;
		} else if (val == 0) {
			focus2torch = false;
		} else return -EINVAL;

	return strnlen(buf, count);
}
static DEVICE_ATTR(focus2torch, (S_IWUGO|S_IRUGO),
	focus2torch_show, focus2torch_dump);
	
// hide navigation bar mode
static ssize_t hn_enable_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", hn_enable);
}

static ssize_t hn_enable_dump(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{

	int val;
	int rc;

	rc = kstrtoint(buf, 10, &val);
	if (rc) return -EINVAL;
	if(val == 1) {
			hn_enable = true;
			hn_update_sett();
		} else if (val == 0) {
			hn_enable = false;
			hn_update_sett();
		} else return -EINVAL;

	return strnlen(buf, count);
}
static DEVICE_ATTR(hn_enable, (S_IWUGO|S_IRUGO),
	hn_enable_show, hn_enable_dump);
	
// camera exposure gain
static ssize_t cam_gain_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", cam_gain);
}

static ssize_t cam_gain_dump(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{

	int val;
	int rc;

	rc = kstrtoint(buf, 10, &val);
	if (rc) return -EINVAL;
	if(val == 1) {
			cam_gain = true;
		} else if (val == 0) {
			cam_gain = false;
		} else return -EINVAL;

	return strnlen(buf, count);
}
static DEVICE_ATTR(cam_gain, (S_IWUGO|S_IRUGO),
	cam_gain_show, cam_gain_dump);
	
// force 30fps video recording
static ssize_t cam_rec_30fps_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", cam_rec_30fps);
}

static ssize_t cam_rec_30fps_dump(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{

	int val;
	int rc;

	rc = kstrtoint(buf, 10, &val);
	if (rc) return -EINVAL;
	if(val == 1) {
			cam_rec_30fps = true;
		} else if (val == 0) {
			cam_rec_30fps = false;
		} else return -EINVAL;

	return strnlen(buf, count);
}
static DEVICE_ATTR(cam_rec_30fps, (S_IWUGO|S_IRUGO),
	cam_rec_30fps_show, cam_rec_30fps_dump);

/*
 * INIT / EXIT stuff below here
 */
//extern struct kobject *nui_setting_kobj;
struct kobject *nui_setting_kobj;
EXPORT_SYMBOL_GPL(nui_setting_kobj);

static int __init nuisetting_init(void)
{
	int rc = 0;

	nui_batt_sav = 0;
	nui_batt_sav_mode(nui_batt_sav);

    nui_setting_kobj = kobject_create_and_add("nui", NULL) ;
    if (nui_setting_kobj == NULL) {
        pr_warn("%s: nui_setting_kobj create_and_add failed\n", __func__);
    }
    rc = sysfs_create_file(nui_setting_kobj, &dev_attr_nuibrightness.attr);
    rc = sysfs_create_file(nui_setting_kobj, &dev_attr_level_short.attr);
    rc = sysfs_create_file(nui_setting_kobj, &dev_attr_zeroprecent.attr);
    rc = sysfs_create_file(nui_setting_kobj, &dev_attr_nooc.attr);
    rc = sysfs_create_file(nui_setting_kobj, &dev_attr_logo.attr);
    rc = sysfs_create_file(nui_setting_kobj, &dev_attr_camera_key.attr);	
	rc = sysfs_create_file(nui_setting_kobj, &dev_attr_focus_key.attr);	
	rc = sysfs_create_file(nui_setting_kobj, &dev_attr_camera_key_scroff.attr);	
	rc = sysfs_create_file(nui_setting_kobj, &dev_attr_focus_key_scroff.attr);	
	rc = sysfs_create_file(nui_setting_kobj, &dev_attr_key_scroff.attr);	
	rc = sysfs_create_file(nui_setting_kobj, &dev_attr_torch.attr);	
	rc = sysfs_create_file(nui_setting_kobj, &dev_attr_proximitysens.attr);
	rc = sysfs_create_file(nui_setting_kobj, &dev_attr_brlock.attr);
	rc = sysfs_create_file(nui_setting_kobj, &dev_attr_which_vol.attr);
	rc = sysfs_create_file(nui_setting_kobj, &dev_attr_focus2torch.attr);
	rc = sysfs_create_file(nui_setting_kobj, &dev_attr_hn_enable.attr);
	rc = sysfs_create_file(nui_setting_kobj, &dev_attr_cam_gain.attr);
	rc = sysfs_create_file(nui_setting_kobj, &dev_attr_cam_rec_30fps.attr);

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
