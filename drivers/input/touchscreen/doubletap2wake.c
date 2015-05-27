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
#define DRIVER_AUTHOR "Dennis Rassmann <showp1984@gmail.com>"
#define DRIVER_DESCRIPTION "DoubleTap2Wake for almost any device"
#define DRIVER_VERSION "1.7"
#define LOGTAG "[doubletap2wake]: "

/* Tuneables */
#define DT2W_PWRKEY_DUR          60
#define DT2W_TIMEOUT_MAX         600
#define DT2W_TIMEOUT_MIN         80
#define DT2W_DELTA               180


/* Resources */
static struct wake_lock dt2w_wake_lock;
int dt2w_switch = 0;
bool scr_suspended = false;
bool touchdown = false;
static bool touch_x_called = false, touch_y_called = false;
static struct input_dev * doubletap2wake_pwrdev;
static DEFINE_MUTEX(pwrkeyworklock);
static struct workqueue_struct *dt2w_input_wq;
static struct work_struct dt2w_input_work;

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

/* PowerKey work func */
static void doubletap2wake_presspwr(struct work_struct *doubletap2wake_presspwr_work) {
	if (!mutex_trylock(&pwrkeyworklock))
                return;
printk("ngxson:debug dt2w pwr press");
	input_event(doubletap2wake_pwrdev, EV_KEY, KEY_POWER, 1);
	input_event(doubletap2wake_pwrdev, EV_SYN, 0, 0);
	msleep(DT2W_PWRKEY_DUR);
	input_event(doubletap2wake_pwrdev, EV_KEY, KEY_POWER, 0);
	input_event(doubletap2wake_pwrdev, EV_SYN, 0, 0);
	msleep(DT2W_PWRKEY_DUR);
        mutex_unlock(&pwrkeyworklock);
	return;
}
static DECLARE_WORK(doubletap2wake_presspwr_work, doubletap2wake_presspwr);

/* PowerKey trigger */
static void doubletap2wake_pwrswitch(void) {
	schedule_work(&doubletap2wake_presspwr_work);
        return;
}

/*reset on finger release*/
static int last_touch_position_x = 0;
static int last_touch_position_y = 0;
static s64 prev_time;
static int prev_x = 0, prev_y = 0;

static void dt2w_reset(void)
{
	prev_time = 0;
	prev_x = 0;
	prev_y = 0;
}

/*main function for dt2w*/
static void detect_dt2w(int x, int y, s64 trigger_time)
{
	//printk("[TP] [DT2W] dt2w x=%d y=%d\n", x, y);
	if (prev_time == 0) {
		prev_time = trigger_time;
		prev_x = x;
		prev_y = y;
	} else if (((trigger_time - prev_time) > DT2W_TIMEOUT_MAX) &&
			((trigger_time - prev_time) < DT2W_TIMEOUT_MIN)) {
		prev_time = trigger_time;
		prev_x = x;
		prev_y = y;
	} else {
		if (((abs(x - prev_x) < DT2W_DELTA) && (abs(y - prev_y) < DT2W_DELTA))
						|| (prev_x == 0 && prev_y == 0)) {
			printk("[TP] [DT2W] dt2w ON\n");
			dt2w_reset();
			doubletap2wake_pwrswitch();
		} else {
			prev_time = trigger_time;
			prev_x = x;
			prev_y = y;
		}
	}
}

static void dt2w_input_callback(struct work_struct *unused) {
msleep(25);
if (scr_suspended) {
	if(touchdown) {
		touchdown = false;
		detect_dt2w(last_touch_position_x, last_touch_position_y, ktime_to_ms(ktime_get()));
	}
}
	return;
}

static void dt2w_input_event(struct input_handle *handle, unsigned int type,
				unsigned int code, int value) {
	if (code == ABS_MT_SLOT) {
		dt2w_reset();
		return;
	}

	if (code == ABS_MT_TRACKING_ID && value == -1) {
		dt2w_reset();
		return;
	}

	if (code == ABS_MT_POSITION_X) {
		last_touch_position_x = value;
		touch_x_called = true;
	}

	if (code == ABS_MT_POSITION_Y) {
		last_touch_position_y = value;
		touch_y_called = true;
	}

	if (touch_x_called && touch_y_called) {
		touch_x_called = false;
		touch_y_called = false;
		queue_work_on(0, dt2w_input_wq, &dt2w_input_work);
	}
}

static int input_dev_filter(struct input_dev *dev) {
	if (strstr(dev->name, "touch") ||
		strstr(dev->name, "synaptics_dsx_i2c") ||
		strstr(dev->name, "synaptics_dsx_rmi4_i2c") ||
		strstr(dev->name, "touchscreen")) {
		//printk("ngxson debug input_dev_filter = 0");
		return 0;
	} else {
		//printk("ngxson debug input_dev_filter = 1");
		return 1;
	}
}

static int dt2w_input_connect(struct input_handler *handler,
				struct input_dev *dev,
				const struct input_device_id *id) {
	struct input_handle *handle;
	int error;

	if (input_dev_filter(dev))
		return -ENODEV;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "dt2w";

	error = input_register_handle(handle);
	if (error)
		goto err2;

	error = input_open_device(handle);
	if (error)
		goto err1;

	return 0;
err1:
	input_unregister_handle(handle);
err2:
	kfree(handle);
	return error;
}

static void dt2w_input_disconnect(struct input_handle *handle) {
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id dt2w_ids[] = {
	{ .driver_info = 1 },
	{ },
};

static struct input_handler dt2w_input_handler = {
	.event		= dt2w_input_event,
	.connect	= dt2w_input_connect,
	.disconnect	= dt2w_input_disconnect,
	.name		= "dt2w_inputreq",
	.id_table	= dt2w_ids,
};

#ifdef CONFIG_POWERSUSPEND
static void dt2w_power_suspend(struct power_suspend *h) {
	scr_suspended = true;
	printk("ngxson: debug POWERSUSPEND pwr off");
}

static void dt2w_power_resume(struct power_suspend *h) {
	scr_suspended = false;
	printk("ngxson: debug POWERSUSPEND pwr on");
}

static struct power_suspend dt2w_power_suspend_handler = {
	.suspend = dt2w_power_suspend,
	.resume = dt2w_power_resume,
};
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
static void dt2w_early_suspend(struct early_suspend *h) {
	if(dt2w_switch > 0) wake_lock(&dt2w_wake_lock);
	printk("ngxson: debug EARLYSUSPEND pwr off");
	scr_suspended = true;
}

static void dt2w_late_resume(struct early_suspend *h) {
	if(dt2w_switch > 0) wake_unlock(&dt2w_wake_lock);
	printk("ngxson: debug EARLYSUSPEND pwr on");
	scr_suspended = false;
}

static struct early_suspend dt2w_early_suspend_handler = {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
	.suspend = dt2w_early_suspend,
	.resume = dt2w_late_resume,
};
#endif
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
			dt2w_reset();
			doubletap2wake_pwrswitch();
			msleep(400);
		}
		if (!scr_suspended) {
			dt2w_switch = value;
		}
	}
	return count;
}

static DEVICE_ATTR(doubletap2wake, (S_IWUSR|S_IRUGO),
	dt2w_doubletap2wake_show, dt2w_doubletap2wake_dump);

static ssize_t dt2w_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%s\n", DRIVER_VERSION);

	return count;
}

static ssize_t dt2w_version_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static DEVICE_ATTR(doubletap2wake_version, (S_IWUSR|S_IRUGO),
	dt2w_version_show, dt2w_version_dump);

/*
 * INIT / EXIT stuff below here
 */
//extern struct kobject *android_touch_kobj;
struct kobject *android_touch_kobj;
EXPORT_SYMBOL_GPL(android_touch_kobj);

static int __init doubletap2wake_init(void)
{
	int rc = 0;
	wake_lock_init(&dt2w_wake_lock, WAKE_LOCK_SUSPEND, "dt2w");

	doubletap2wake_pwrdev = input_allocate_device();
	if (!doubletap2wake_pwrdev) {
		printk("Can't allocate suspend autotest power button\n");
		goto err_alloc_dev;
	}

	input_set_capability(doubletap2wake_pwrdev, EV_KEY, KEY_POWER);
	doubletap2wake_pwrdev->name = "dt2w_pwrkey";
	doubletap2wake_pwrdev->phys = "dt2w_pwrkey/input0";

	rc = input_register_device(doubletap2wake_pwrdev);
	if (rc) {
		printk("%s: dt2w input_register_device err=%d\n", __func__, rc);
		goto err_input_dev;
	}

	dt2w_input_wq = create_workqueue("dt2wiwq");
	if (!dt2w_input_wq) {
		printk("%s: dt2w Failed to create dt2wiwq workqueue\n", __func__);
		return -EFAULT;
	}
	INIT_WORK(&dt2w_input_work, dt2w_input_callback);
	rc = input_register_handler(&dt2w_input_handler);
	if (rc)
		printk("%s: dt2w Failed to register dt2w_input_handler\n", __func__);
    
    android_touch_kobj = kobject_create_and_add("android_touch", NULL) ;
    if (android_touch_kobj == NULL) {
        pr_warn("%s: android_touch_kobj create_and_add failed\n", __func__);
    }
    rc = sysfs_create_file(android_touch_kobj, &dev_attr_doubletap2wake.attr);
    if (rc) {
        pr_warn("%s: sysfs_create_file failed for doubletap2wake\n", __func__);
    }
    rc = sysfs_create_file(android_touch_kobj, &dev_attr_doubletap2wake_version.attr);
    if (rc) {
        pr_warn("%s: sysfs_create_file failed for doubletap2wake_version\n", __func__);
    }

#ifdef CONFIG_POWERSUSPEND
	register_power_suspend(&dt2w_power_suspend_handler);
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&dt2w_early_suspend_handler);
#endif

err_input_dev:
	input_free_device(doubletap2wake_pwrdev);
err_alloc_dev:
	pr_info(LOGTAG"%s done\n", __func__);

	return 0;
}

static void __exit doubletap2wake_exit(void)
{
    kobject_del(android_touch_kobj);
#ifdef CONFIG_POWERSUSPEND
	unregister_power_suspend(&dt2w_power_suspend_handler);
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&dt2w_early_suspend_handler);
#endif
	wake_lock_destroy(&dt2w_wake_lock);
	input_unregister_handler(&dt2w_input_handler);
	destroy_workqueue(dt2w_input_wq);
	input_unregister_device(doubletap2wake_pwrdev);
	input_free_device(doubletap2wake_pwrdev);
	return;
}

module_init(doubletap2wake_init);
module_exit(doubletap2wake_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESCRIPTION);
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPLv2");
