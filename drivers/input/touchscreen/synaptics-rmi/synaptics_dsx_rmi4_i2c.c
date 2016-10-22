/*
 * Synaptics DSX touchscreen driver
 *
 * Copyright (C) 2012 Synaptics Incorporated
 * 
 * ******* ONLY FOR XPERIA M SS/DS *******
 *
 * Blah blah (B) 2015 Nui <thichthat@gmail.com>
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
 * Copyright (C) 2010 Js HA <js.ha@stericsson.com>
 * Copyright (C) 2010 Naveen Kumar G <naveen.gaddipati@stericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/input/synaptics_dsx.h>
#include <linux/input/synaptics_dsx_rmi4_i2c.h>
#ifdef KERNEL_ABOVE_2_6_38
#include <linux/input/mt.h>
#endif

#include <linux/wakelock.h>
#include <linux/nuisetting.h>
#include <linux/input/FIH-tool/config.h>

#define DRIVER_NAME "synaptics_dsx_i2c"
#define INPUT_PHYS_NAME "synaptics_dsx_i2c/input0"

#ifdef KERNEL_ABOVE_2_6_38
#define TYPE_B_PROTOCOL
#endif

#define NO_0D_WHILE_2D

#define RPT_OBJ_TYPE (1 << 0)
#define RPT_X_LSB (1 << 1)
#define RPT_X_MSB (1 << 2)
#define RPT_Y_LSB (1 << 3)
#define RPT_Y_MSB (1 << 4)
#define RPT_Z (1 << 5)
#define RPT_WX (1 << 6)
#define RPT_WY (1 << 7)
#define F12_RPT_DEFAULT (RPT_OBJ_TYPE | RPT_X_LSB | RPT_Y_LSB | RPT_Y_MSB)
#define F12_RPT (F12_RPT_DEFAULT | RPT_WX | RPT_WY)

#define EXP_FN_DET_INTERVAL 1000 /* ms */
#define POLLING_PERIOD 1 /* ms */
#define SYN_I2C_RETRY_TIMES 10
#define MAX_ABS_MT_TOUCH_MAJOR 15

#define F01_STD_QUERY_LEN 21
#define F11_STD_QUERY_LEN 9
#define F11_STD_CTRL_LEN 10
#define F11_STD_DATA_LEN 12

#define NORMAL_OPERATION (0 << 0)
#define SENSOR_SLEEP (1 << 0)
#define NO_SLEEP_OFF (0 << 3)
#define NO_SLEEP_ON (1 << 3)

//ngxson_dt2w
#include <linux/hrtimer.h>
#include <asm-generic/cputime.h>
#include <linux/input/doubletap2wake.h>
#include <linux/mfd/pm8xxx/vibrator.h>

#define D2W_PWRKEY_DUR 30
#define DT2W_TIMEOUT_MAX         400
#define DT2W_TIMEOUT_MIN         80
#define DT2W_DELTA_X               120
#define DT2W_DELTA_Y               70
#define DT2W_SCREEN_MIDDLE         550
#define S2W_TIMEOUT_MAX         1500
#define S2W_DELTA_X               250

static unsigned short data_addr;
static unsigned short ctrl_addr;
static unsigned char data_reg_blk_size;
bool nui_suspend = false;
static unsigned char work_mode = 0;

static cputime64_t tap_time_pre = 0;
static int x_pre = 0, y_pre = 0;

static struct input_dev * doubletap2wake_pwrdev;
static DEFINE_MUTEX(pwrkeyworklock);
static DEFINE_MUTEX(irqdt2w);
static struct wake_lock dt2w_wake_lock;
static bool dt2w_ok = false;
static bool dt2w_pressed = false;
static bool dt2w_got_xy = false;
static unsigned int dt2w_get_x = 0;
static unsigned int dt2w_get_y = 0;
//s2w
static cputime64_t s2w_tap_time_pre;
static unsigned int s2w_x = 0;
static bool s2w_finger = false;
static bool s2w_scron = false;
static bool scr_suspended = false;
//s2m
static cputime64_t s2m_tap_time_pre;
static unsigned int s2m_y = 0;
static bool s2m_right = true;
static bool s2m_finger = false;
//hide nav
static bool hn_active;
static bool hn_detecting;
static unsigned char hn_btn;
static cputime64_t hn_time;
//
static unsigned char reg_index;
static unsigned char finger_shift;
static unsigned char finger_status;
static unsigned char prev_status;
static unsigned char finger_status_reg[3];
static unsigned char data[F11_STD_DATA_LEN];
static unsigned short data_offset;
static int x, y, wx, wy, prev_x, prev_y;
static unsigned char intr_f11_irq[MAX_INTR_REGISTERS];
static unsigned int hn_one = KEY_BACK;
static unsigned int hn_two = KEY_HOMEPAGE;
static unsigned int hn_thr = 580;
static unsigned int hn_m = KEY_MENU;
//static bool nui_enabled_irq = false;
static bool block_gesture = false;
//end

static int synaptics_rmi4_i2c_read(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data,
		unsigned short length);

static int synaptics_rmi4_i2c_write(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data,
		unsigned short length);

static int synaptics_rmi4_reset_device(struct synaptics_rmi4_data *rmi4_data);

#ifdef CONFIG_HAS_EARLYSUSPEND
static ssize_t synaptics_rmi4_full_pm_cycle_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_full_pm_cycle_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
		
static void synaptics_rmi4_early_suspend(struct early_suspend *h);

static void reset_device(struct synaptics_rmi4_data *rmi4_data);

static void synaptics_rmi4_late_resume(struct early_suspend *h);

static int synaptics_rmi4_suspend(struct device *dev);

static int synaptics_rmi4_resume(struct device *dev);
#endif

static ssize_t synaptics_rmi4_f01_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_f01_productinfo_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_f01_flashprog_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_0dbutton_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_0dbutton_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

struct synaptics_rmi4_f01_device_status {
	union {
		struct {
			unsigned char status_code:4;
			unsigned char reserved:2;
			unsigned char flash_prog:1;
			unsigned char unconfigured:1;
		} __packed;
		unsigned char data[1];
	};
};


struct synaptics_rmi4_exp_fn {
	enum exp_fn fn_type;
	bool inserted;
	int (*func_init)(struct synaptics_rmi4_data *rmi4_data);
	void (*func_remove)(struct synaptics_rmi4_data *rmi4_data);
	void (*func_attn)(struct synaptics_rmi4_data *rmi4_data,
			unsigned char intr_mask);
	struct list_head link;
};

static struct device_attribute attrs[] = {
#ifdef CONFIG_HAS_EARLYSUSPEND
	__ATTR(full_pm_cycle, (S_IRUGO | S_IWUSR),
			synaptics_rmi4_full_pm_cycle_show,
			synaptics_rmi4_full_pm_cycle_store),
#endif
	__ATTR(reset, S_IWUSR,
			synaptics_rmi4_show_error,
			synaptics_rmi4_f01_reset_store),
	__ATTR(productinfo, S_IRUGO,
			synaptics_rmi4_f01_productinfo_show,
			synaptics_rmi4_store_error),
	__ATTR(flashprog, S_IRUGO,
			synaptics_rmi4_f01_flashprog_show,
			synaptics_rmi4_store_error),
	__ATTR(0dbutton, (S_IRUGO | S_IWUSR),
			synaptics_rmi4_0dbutton_show,
			synaptics_rmi4_0dbutton_store),
};

static bool exp_fn_inited;
static struct mutex exp_fn_list_mutex;
static struct list_head exp_fn_list;

//for keypress
void inline btn_press(int i, bool b) {
	if(b) {
		input_event(doubletap2wake_pwrdev, EV_KEY, i, 1);
		input_event(doubletap2wake_pwrdev, EV_SYN, 0, 0);
	} else {
		input_event(doubletap2wake_pwrdev, EV_KEY, i, 0);
		input_event(doubletap2wake_pwrdev, EV_SYN, 0, 0);
	}
}

//ngxson_dt2w
static void doubletap2wake_presspwr(struct work_struct * doubletap2wake_presspwr_work) {
	if (!mutex_trylock(&pwrkeyworklock))
		return;
	vibrate(70);
	input_event(doubletap2wake_pwrdev, EV_KEY, KEY_POWER, 1);
	input_event(doubletap2wake_pwrdev, EV_SYN, 0, 0);
	msleep(D2W_PWRKEY_DUR);
	input_event(doubletap2wake_pwrdev, EV_KEY, KEY_POWER, 0);
	input_event(doubletap2wake_pwrdev, EV_SYN, 0, 0);
	//msleep(D2W_PWRKEY_DUR);
	mutex_unlock(&pwrkeyworklock);
	return;
}
static DECLARE_WORK(doubletap2wake_presspwr_work, doubletap2wake_presspwr);

/* PowerKey trigger */
static void inline doubletap2wake_pwrtrigger(void) {
	if(!block_gesture)
		schedule_work(&doubletap2wake_presspwr_work);
	return;
}

void inline doubletap2wake_reset(void) {
	//printk( "ngxson : dt2w doubletap2wake_reset\n");
	tap_time_pre = 0;
	x_pre = 0;
	y_pre = 0;
}

static void detect_doubletap2wake(int x, int y)
{
	//printk("[ngxson] [DT2W] dt2w x=%d y=%d\n", x, y);
	if (tap_time_pre == 0) {
		tap_time_pre = ktime_to_ms(ktime_get());
		x_pre = x;
		y_pre = y;
	} else if (((ktime_to_ms(ktime_get()) - tap_time_pre) > DT2W_TIMEOUT_MAX) ||
			((ktime_to_ms(ktime_get()) - tap_time_pre) < DT2W_TIMEOUT_MIN)) {
		tap_time_pre = ktime_to_ms(ktime_get());
		x_pre = x;
		y_pre = y;
	} else if (dt2w_switch == 2) { /* Detect half screen */
		if ((((abs(x - x_pre) < DT2W_DELTA_X) && (abs(y - y_pre) < DT2W_DELTA_Y))
						|| (x_pre == 0 && y_pre == 0)) 
						&& ((y > DT2W_SCREEN_MIDDLE) && (y_pre > DT2W_SCREEN_MIDDLE))) {
			doubletap2wake_reset();
			doubletap2wake_pwrtrigger();
		} else {
			tap_time_pre = ktime_to_ms(ktime_get());
			x_pre = x;
			y_pre = y;
		}
	} else {
		if (((abs(x - x_pre) < DT2W_DELTA_X) && (abs(y - y_pre) < DT2W_DELTA_Y))
						|| (x_pre == 0 && y_pre == 0)) {
			doubletap2wake_reset();
			doubletap2wake_pwrtrigger();
		} else {
			tap_time_pre = ktime_to_ms(ktime_get());
			x_pre = x;
			y_pre = y;
		}
	}
} //detect_doubletap2wake
//end

//s2w
void inline s2w_reset(void) {
	s2w_tap_time_pre = 0;
	s2w_x = 0;
	s2w_finger = false;
}

//s2m
void inline s2m_reset(void) {
	s2m_finger = false;
	s2m_y = 0;
	s2m_tap_time_pre = 0;
	s2m_right = true;
}

static void mediakey_press(struct work_struct * mediakey_press_work) {
	int key_code_m;
	
	if (!mutex_trylock(&pwrkeyworklock))
		return;
	vibrate(50);
	if(s2m_right) {
		if(!s2m_reverse) key_code_m = KEY_NEXTSONG;
		else key_code_m = KEY_PREVIOUSSONG;
	} else {
		if(!s2m_reverse) key_code_m = KEY_PREVIOUSSONG;
		else key_code_m = KEY_NEXTSONG;
	}
	input_event(doubletap2wake_pwrdev, EV_KEY, key_code_m, 1);
	input_event(doubletap2wake_pwrdev, EV_SYN, 0, 0);
	msleep(D2W_PWRKEY_DUR);
	input_event(doubletap2wake_pwrdev, EV_KEY, key_code_m, 0);
	input_event(doubletap2wake_pwrdev, EV_SYN, 0, 0);
	s2m_reset();
	mutex_unlock(&pwrkeyworklock);
	return;
}
static DECLARE_WORK(mediakey_press_work, mediakey_press);

void s2m_detect(int x, int y) {
	if(s2m_right) {
		if ((abs(y - s2m_y) < S2W_DELTA_X) && (x > 256) &&
				((ktime_to_ms(ktime_get()) - s2m_tap_time_pre) < S2W_TIMEOUT_MAX) ) {
			if (x < 512) {
				s2m_finger = false;
				schedule_work(&mediakey_press_work);
			}
		} else s2m_reset();
	} else {
		if ((abs(y - s2m_y) < S2W_DELTA_X) && (x < 768) &&
				((ktime_to_ms(ktime_get()) - s2m_tap_time_pre) < S2W_TIMEOUT_MAX) ) {
			if (x > 512) {
				s2m_finger = false;
				schedule_work(&mediakey_press_work);
			}
		} else s2m_reset();
	}
}

//end

#ifdef CONFIG_HAS_EARLYSUSPEND
static ssize_t synaptics_rmi4_full_pm_cycle_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			rmi4_data->full_pm_cycle);
}

static ssize_t synaptics_rmi4_full_pm_cycle_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	rmi4_data->full_pm_cycle = input > 0 ? 1 : 0;

	return count;
}
#endif

static ssize_t synaptics_rmi4_f01_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int reset;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	if (sscanf(buf, "%u", &reset) != 1)
		return -EINVAL;

	if (reset != 1)
		return -EINVAL;

	retval = synaptics_rmi4_reset_device(rmi4_data);
	if (retval < 0) {
		printk( "ITUCH : Device(%s), Failed to issue reset command, error(%d)\n", dev_name( dev ), retval );
		return retval;
	}

	return count;
}

static ssize_t synaptics_rmi4_f01_productinfo_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "0x%02x 0x%02x\n",
			(rmi4_data->rmi4_mod_info.product_info[0]),
			(rmi4_data->rmi4_mod_info.product_info[1]));
}

static ssize_t synaptics_rmi4_f01_flashprog_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int retval;
	struct synaptics_rmi4_f01_device_status device_status;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_data_base_addr,
			device_status.data,
			sizeof(device_status.data));
	if (retval < 0) {
		printk( "ITUCH : Device(%s), Failed to read device status, error(%d)\n", dev_name( dev ), retval );
		return retval;
	}

	return snprintf(buf, PAGE_SIZE, "%u\n",
			device_status.flash_prog);
}

static ssize_t synaptics_rmi4_0dbutton_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n",
			rmi4_data->button_0d_enabled);
}

static ssize_t synaptics_rmi4_0dbutton_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int input;
	unsigned char ii;
	unsigned char intr_enable;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	input = input > 0 ? 1 : 0;

	if (rmi4_data->button_0d_enabled == input)
		return count;

	list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
		if (fhandler->fn_number == SYNAPTICS_RMI4_F1A) {
			ii = fhandler->intr_reg_num;

			retval = synaptics_rmi4_i2c_read(rmi4_data,
					rmi4_data->f01_ctrl_base_addr + 1 + ii,
					&intr_enable,
					sizeof(intr_enable));
			if (retval < 0)
				return retval;

			if (input == 1)
				intr_enable |= fhandler->intr_mask;
			else
				intr_enable &= ~fhandler->intr_mask;

			retval = synaptics_rmi4_i2c_write(rmi4_data,
					rmi4_data->f01_ctrl_base_addr + 1 + ii,
					&intr_enable,
					sizeof(intr_enable));
			if (retval < 0)
				return retval;
		}
	}

	rmi4_data->button_0d_enabled = input;

	return count;
}

static void inline nui_rmi4_proc_fngr_press(unsigned int x, unsigned int y)
{
	if (dt2w_switch) {
			dt2w_pressed = true;
			if(((x) <100) || ((x)>900) 
					|| ((y)<50) || ((y) >950)) {
					/* Prevent sliding from screen edge */
				doubletap2wake_reset();
				dt2w_ok = false;
			} else {
				dt2w_get_x = x;
				dt2w_get_y = y;
				dt2w_ok = true;
			}
	}
	//s2w
	if((s2w_switch)) {
		if(!s2w_finger) {
			if(((y) < 950)) {
				s2w_reset();
			} else {
				s2w_finger = true;
				s2w_tap_time_pre = ktime_to_ms(ktime_get());
				s2w_x = x;
				s2w_scron = false;
			}
		}
	}
	
	//media control
	if(s2m) {
		if(!s2m_finger) {
			if(((y) > 256)) {
				s2m_reset();
			} else if (((x) > 128) && ((x) < 896)) {
				s2m_reset();
			} else {
				if((x) > 512) s2m_right = true;
				else s2m_right = false;
				s2m_finger = true;
				s2m_tap_time_pre = ktime_to_ms(ktime_get());
				s2m_y = y;
			}
		}
	}
	return;
}

static void inline nui_rmi4_proc_fngr_release(unsigned int x, unsigned int y)
{
	if (dt2w_switch) {
			dt2w_got_xy = true;
			if (!dt2w_pressed) detect_doubletap2wake((x), (y));
			else if (dt2w_ok) {
				detect_doubletap2wake(dt2w_get_x, dt2w_get_y);
				dt2w_ok = false;
				dt2w_pressed = false;
			}
	}
	//s2w
	if (s2w_finger){ 
		s2w_reset();
	}
	
	//media control
	if (s2m_finger){ 
		s2m_reset();
	}

	return;
}

static void inline nui_rmi4_proc_fngr_move(unsigned int x, unsigned int y)
{
	if(s2w_finger) {
		if(!s2w_scron) {
			if ((abs(x - s2w_x) < S2W_DELTA_X) && (y > 450) &&
					((ktime_to_ms(ktime_get()) - s2w_tap_time_pre) < S2W_TIMEOUT_MAX) ) {
				if (y < 750) {
					s2w_scron = true;
					doubletap2wake_pwrtrigger();
				}
			} else s2w_reset();
		}
	}
	
	if(s2m_finger) s2m_detect(x,y);

	return;
}

 /**
 * synaptics_rmi4_set_page()
 *
 * Called by synaptics_rmi4_i2c_read() and synaptics_rmi4_i2c_write().
 *
 * This function writes to the page select register to switch to the
 * assigned page.
 */
static int synaptics_rmi4_set_page(struct synaptics_rmi4_data *rmi4_data,
		unsigned int address)
{
	int retval = 0;
	unsigned char retry;
	unsigned char buf[PAGE_SELECT_LEN];
	unsigned char page;
	struct i2c_client *i2c = rmi4_data->i2c_client;

	page = ((address >> 8) & MASK_8BIT);
	if (page != rmi4_data->current_page) {
		buf[0] = MASK_8BIT;
		buf[1] = page;
		for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
			retval = i2c_master_send(i2c, buf, PAGE_SELECT_LEN);
			if (retval != PAGE_SELECT_LEN) {
				printk( "ITUCH : Device(%s), I2C retry(%d)\n", dev_name( &i2c->dev ), retry + 1);
				reset_device(rmi4_data);
				synaptics_rmi4_reset_device(rmi4_data);
				reset_device(rmi4_data);
				msleep(20);
			} else {
				rmi4_data->current_page = page;
				break;
			}
		}
	} else {
		retval = PAGE_SELECT_LEN;
	}

	return retval;
}



 /**
 * synaptics_rmi4_i2c_read()
 *
 * Called by various functions in this driver, and also exported to
 * other expansion Function modules such as rmi_dev.
 *
 * This function reads data of an arbitrary length from the sensor,
 * starting from an assigned register address of the sensor, via I2C
 * with a retry mechanism.
 */
static int synaptics_rmi4_i2c_read(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data, unsigned short length)
{
	int retval;
	unsigned char retry;
	unsigned char buf;
	struct i2c_msg msg[] = {
		{
			.addr = rmi4_data->i2c_client->addr,
			.flags = 0,
			.len = 1,
			.buf = &buf,
		},
		{
			.addr = rmi4_data->i2c_client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		},
	};

	buf = addr & MASK_8BIT;

	mutex_lock(&(rmi4_data->rmi4_io_ctrl_mutex));

	retval = synaptics_rmi4_set_page(rmi4_data, addr);
	if (retval != PAGE_SELECT_LEN)
		goto exit;

	for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(rmi4_data->i2c_client->adapter, msg, 2) == 2) {
			retval = length;
			break;
		}
		printk( "ITUCH : Device(%s), I2C retry(%d)\n", dev_name( &rmi4_data->i2c_client->dev ), retry + 1 );
		msleep(20);
	}

	if (retry == SYN_I2C_RETRY_TIMES) {
		printk( "ITUCH : Device(%s), I2C read over retry limit\n", dev_name( &rmi4_data->i2c_client->dev ) );
		reset_device(rmi4_data);
		synaptics_rmi4_reset_device(rmi4_data);
		reset_device(rmi4_data);
		retval = -EIO;
	}

exit:
	mutex_unlock(&(rmi4_data->rmi4_io_ctrl_mutex));

	return retval;
}

 /**
 * synaptics_rmi4_i2c_write()
 *
 * Called by various functions in this driver, and also exported to
 * other expansion Function modules such as rmi_dev.
 *
 * This function writes data of an arbitrary length to the sensor,
 * starting from an assigned register address of the sensor, via I2C with
 * a retry mechanism.
 */
static int synaptics_rmi4_i2c_write(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data, unsigned short length)
{
	int retval;
	unsigned char retry;
	unsigned char buf[length + 1];
	struct i2c_msg msg[] = {
		{
			.addr = rmi4_data->i2c_client->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

	mutex_lock(&(rmi4_data->rmi4_io_ctrl_mutex));

	retval = synaptics_rmi4_set_page(rmi4_data, addr);
	if (retval != PAGE_SELECT_LEN)
		goto exit;

	buf[0] = addr & MASK_8BIT;
	memcpy(&buf[1], &data[0], length);

	for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(rmi4_data->i2c_client->adapter, msg, 1) == 1) {
			retval = length;
			break;
		}
		printk( "ITUCH : Device(%s), I2C retry(%d)\n", dev_name( &rmi4_data->i2c_client->dev ), retry + 1 );
		msleep(20);
	}

	if (retry == SYN_I2C_RETRY_TIMES) {
		printk( "ITUCH : Device(%s), I2C write over retry limit\n", dev_name( &rmi4_data->i2c_client->dev ) );
		reset_device(rmi4_data);
		synaptics_rmi4_reset_device(rmi4_data);
		reset_device(rmi4_data);
		retval = -EIO;
	}

exit:
	mutex_unlock(&(rmi4_data->rmi4_io_ctrl_mutex));

	return retval;
}

static inline void nui_rmi4_f11_abs_report(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char finger;
		
	for (finger = 1; finger < 5; finger++) {
		reg_index = finger / 4;
		finger_shift = (finger % 4) * 2;
		finger_status = (finger_status_reg[reg_index] >> finger_shift)
				& MASK_2BIT;
		if (finger_status) {
			break;
		}
	}
	if(!finger_status) {
		finger = 0;
		reg_index = finger / 4;
		finger_shift = (finger % 4) * 2;
		finger_status = (finger_status_reg[reg_index] >> finger_shift)
				& MASK_2BIT;
		if (finger_status) {
			data_offset = data_addr +
					2 +
					(finger * data_reg_blk_size);
			retval = synaptics_rmi4_i2c_read(rmi4_data,
					data_offset,
					data,
					data_reg_blk_size);
			if (retval < 0)
				return;
			x = (data[0] << 4) | (data[2] & MASK_4BIT);
			y = (data[1] << 4) | ((data[2] >> 4) & MASK_4BIT);
		}
		if(!prev_status && finger_status) nui_rmi4_proc_fngr_press(x, y);
		else if(prev_status && finger_status) nui_rmi4_proc_fngr_move(x, y);
		else if(prev_status && !finger_status) nui_rmi4_proc_fngr_release(x, y);
		prev_status = finger_status;
	} else {
		doubletap2wake_reset();
		s2w_reset();
		s2m_reset();
	}

	return;
}

/**
 * synaptics_rmi4_f11_abs_report()
 *
 * Called by synaptics_rmi4_report_touch() when valid Function $11
 * finger data has been detected.
 *
 * This function reads the Function $11 data registers, determines the
 * status of each finger supported by the Function, processes any
 * necessary coordinate manipulation, reports the finger data to
 * the input subsystem, and returns the number of fingers detected.
 */
static inline void synaptics_rmi4_f11_abs_report(struct synaptics_rmi4_data *rmi4_data,
										unsigned char finger, unsigned char m_reg_index, unsigned char m_finger_shift)
{
	int retval;
	finger_status = (finger_status_reg[m_reg_index] >> m_finger_shift)
			& MASK_2BIT;

	if (!(rmi4_data->finger_state[finger].status) && !finger_status)
		return;

	input_mt_slot(rmi4_data->input_dev, finger);
	input_mt_report_slot_state(rmi4_data->input_dev,
			MT_TOOL_FINGER, finger_status);

	if (finger_status) {
		data_offset = data_addr +
				2 +
				(finger * data_reg_blk_size);
		retval = synaptics_rmi4_i2c_read(rmi4_data,
				data_offset,
				data,
				data_reg_blk_size);
		if (retval < 0)
			return;

		x = (data[0] << 4) | (data[2] & MASK_4BIT);
		y = (data[1] << 4) | ((data[2] >> 4) & MASK_4BIT);
		wx = (data[3] & MASK_4BIT);
		wy = (data[3] >> 4) & MASK_4BIT;
		//z = data[4];
	}

	if(rmi4_data->finger_state[finger].status && !finger_status) {
		rmi4_data->finger_state[finger].status = finger_status;
		return;
	} else {
		rmi4_data->finger_state[finger].status = finger_status;
	}

	input_report_abs(rmi4_data->input_dev,
			ABS_MT_POSITION_X, x);
	input_report_abs(rmi4_data->input_dev,
			ABS_MT_POSITION_Y, y);
	input_report_abs(rmi4_data->input_dev,
			ABS_MT_TOUCH_MAJOR, max(wx, wy));
	input_report_abs(rmi4_data->input_dev,
			ABS_MT_TOUCH_MINOR, min(wx, wy));
	//input_report_abs(rmi4_data->input_dev,
	//		ABS_MT_PRESSURE, z);

	return;
}

static void hn_pressbtn(struct work_struct *hn_pressbtn_work) {
	if (!mutex_trylock(&pwrkeyworklock))
		return;
		
	switch(hn_btn) {
		case 0:
			btn_press(hn_one, true);
			msleep(D2W_PWRKEY_DUR);
			btn_press(hn_one, false);
			break;
		case 1:
			btn_press(hn_two, true);
			msleep(D2W_PWRKEY_DUR);
			btn_press(hn_two, false);
			break;
		case 2:
			btn_press(hn_thr, true);
			msleep(D2W_PWRKEY_DUR);
			btn_press(hn_thr, false);
			break;
		default:
			btn_press(hn_m, true);
			msleep(D2W_PWRKEY_DUR);
			btn_press(hn_m, false);
	}
	vibrate(30);
	mutex_unlock(&pwrkeyworklock);
	return;
}
static DECLARE_WORK(hn_pressbtn_work, hn_pressbtn);

void hn_update_sett(void) {
	if(scr_suspended) {
		work_mode = 1;
	} else {
		if(hn_enable) work_mode = 2;
		else work_mode = 0;
	}
}

static inline void synaptics_rmi4_f11_abs_report_hn(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			data_addr,
			finger_status_reg,
			2);
	if (retval < 0)
		return;

	finger_status = (finger_status_reg[0])
			& MASK_2BIT;
	//prev_status = rmi4_data->finger_state[finger].status;

	input_mt_slot(rmi4_data->input_dev, 0);
	input_mt_report_slot_state(rmi4_data->input_dev,
			MT_TOOL_FINGER, finger_status);

	if (finger_status) {
		data_offset = data_addr +
				2;
		retval = synaptics_rmi4_i2c_read(rmi4_data,
				data_offset,
				data,
				data_reg_blk_size);
		if (retval < 0)
			return;

		x = (data[0] << 4) | (data[2] & MASK_4BIT);
		y = (data[1] << 4) | ((data[2] >> 4) & MASK_4BIT);
		wx = (data[3] & MASK_4BIT);
		wy = (data[3] >> 4) & MASK_4BIT;
	}
	//prev_status = rmi4_data->finger_state[0].status;
	prev_x = rmi4_data->finger_state[0].x;
	prev_y = rmi4_data->finger_state[0].y;
	if (rmi4_data->finger_state[0].status && finger_status) { //on moving
		if(hn_active) {
			if((y < 960) && (hn_detecting)) {
				if(((ktime_to_ms(ktime_get()) - hn_time) > 12) &&
					((ktime_to_ms(ktime_get()) - hn_time) < 250)) //check time 12<t<200 (ms)
				schedule_work(&hn_pressbtn_work);
				hn_detecting = false;
			}
			rmi4_data->finger_state[0].x = x;
			rmi4_data->finger_state[0].y = y;
		} else {
			rmi4_data->finger_state[0].x = x;
			rmi4_data->finger_state[0].y = y;
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_POSITION_X, x);
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_POSITION_Y, y);
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_TOUCH_MAJOR, max(wx, wy));
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_TOUCH_MINOR, min(wx, wy));
		}
	} else if (!(rmi4_data->finger_state[0].status) && finger_status) { //finger press
		if(y < 985) {
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_POSITION_X, x);
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_POSITION_Y, y);
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_TOUCH_MAJOR, max(wx, wy));
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_TOUCH_MINOR, min(wx, wy));
		} else {
			hn_active = true;
			hn_time = ktime_to_ms(ktime_get());
			hn_detecting = true;
			if(x < 341) { //back btn
				hn_btn = 0;
			} else if (x > 683) {
				if(x > 964) hn_btn = 3; //menu btn
				else hn_btn = 2; //recent btn
			} else { //home btn
				hn_btn = 1;
			}
		}
		rmi4_data->finger_state[0].x = x;
		rmi4_data->finger_state[0].y = y;
	} else if (rmi4_data->finger_state[0].status && !finger_status) { //release finger
		if(hn_active && (prev_y > 980)) {
			input_mt_report_slot_state(rmi4_data->input_dev,
				MT_TOOL_FINGER, true);
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_POSITION_X, prev_x);
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_POSITION_Y, prev_y);
			input_sync(rmi4_data->input_dev);
			
			input_mt_report_slot_state(rmi4_data->input_dev,
				MT_TOOL_FINGER, false);
		}
		hn_active = false;
	}
	rmi4_data->finger_state[0].status = finger_status;

	synaptics_rmi4_f11_abs_report(rmi4_data, 1, 0, 2);
	synaptics_rmi4_f11_abs_report(rmi4_data, 2, 0, 4);
	synaptics_rmi4_f11_abs_report(rmi4_data, 3, 0, 6);
	synaptics_rmi4_f11_abs_report(rmi4_data, 4, 1, 0);
	if(!hn_active) input_sync(rmi4_data->input_dev);

	return;
}

static void reset_all_touch(struct synaptics_rmi4_data *rmi4_data) {
	unsigned char finger;

	prev_status = 0;
	prev_x = 0;
	prev_y = 0;
	for (finger = 0; finger < 5; finger++) {
		input_mt_slot(rmi4_data->input_dev, finger);
		input_mt_report_slot_state(rmi4_data->input_dev,
				MT_TOOL_FINGER, true);
		input_report_abs(rmi4_data->input_dev,
				ABS_MT_POSITION_X, 0);
		input_report_abs(rmi4_data->input_dev,
				ABS_MT_POSITION_Y, 0);
		
	}
	for (finger = 0; finger < 5; finger++) {
		rmi4_data->finger_state[finger].x = 0;
		rmi4_data->finger_state[finger].y = 0;
		rmi4_data->finger_state[finger].wx = 0;
		rmi4_data->finger_state[finger].wy = 0;
		rmi4_data->finger_state[finger].status = false;
		input_mt_slot(rmi4_data->input_dev, finger);
		input_mt_report_slot_state(rmi4_data->input_dev,
				MT_TOOL_FINGER, false);
	}
	input_sync(rmi4_data->input_dev);
}

 /**
 * synaptics_rmi4_irq()
 *
 * Called by the kernel when an interrupt occurs (when the sensor
 * asserts the attention irq).
 *
 * This function is the ISR thread and handles the acquisition
 * and the reporting of finger data when the presence of fingers
 * is detected.
 */
 
static irqreturn_t synaptics_rmi4_irq(int irq, void *data)
{
	unsigned int i;
	struct synaptics_rmi4_data *rmi4_data = data;

	switch(work_mode) {
		case 0:
			synaptics_rmi4_i2c_read(rmi4_data,
				rmi4_data->f01_data_base_addr + 1,
				intr_f11_irq,
				rmi4_data->num_of_intr_regs);
			synaptics_rmi4_i2c_read(rmi4_data,
				data_addr,
				finger_status_reg,
				2);
			synaptics_rmi4_f11_abs_report(rmi4_data, 0, 0, 0);
			synaptics_rmi4_f11_abs_report(rmi4_data, 1, 0, 2);
			synaptics_rmi4_f11_abs_report(rmi4_data, 2, 0, 4);
			synaptics_rmi4_f11_abs_report(rmi4_data, 3, 0, 6);
			synaptics_rmi4_f11_abs_report(rmi4_data, 4, 1, 0);
			input_sync(rmi4_data->input_dev);
			break;
			
		case 1:
			wake_lock_timeout(&dt2w_wake_lock, 1000);
			for(i=0; i<5 ; i++) {
				if(!nui_suspend) {
					synaptics_rmi4_i2c_read(rmi4_data,
						rmi4_data->f01_data_base_addr + 1,
						intr_f11_irq,
						rmi4_data->num_of_intr_regs);
					synaptics_rmi4_i2c_read(rmi4_data,
						data_addr,
						finger_status_reg,
						2);
					nui_rmi4_f11_abs_report(rmi4_data);
					msleep(12); //my luck number ;)
					break;
				}
				msleep(12);
			}
			break;
			
		default:
			synaptics_rmi4_i2c_read(rmi4_data,
				rmi4_data->f01_data_base_addr + 1,
				intr_f11_irq,
				rmi4_data->num_of_intr_regs);
			synaptics_rmi4_i2c_read(rmi4_data,
				data_addr,
				finger_status_reg,
				2);
			synaptics_rmi4_f11_abs_report_hn(rmi4_data);

	}
	
	return IRQ_HANDLED;
}

 /**
 * synaptics_rmi4_irq_enable()
 *
 * Called by synaptics_rmi4_probe() and the power management functions
 * in this driver and also exported to other expansion Function modules
 * such as rmi_dev.
 *
 * This function handles the enabling and disabling of the attention
 * irq including the setting up of the ISR thread.
 */
static int synaptics_rmi4_irq_enable(struct synaptics_rmi4_data *rmi4_data,
		bool enable)
{
	int retval = 0;
	unsigned char intr_status;
	int irq_flags;

	if(scr_suspended) {
		work_mode = 1;
	} else {
		if(hn_enable) work_mode = 2;
		else work_mode = 0;
	}
	
	if (enable) {
		if (rmi4_data->irq_enabled)
			return retval;

		/* Clear interrupts first */
		retval = synaptics_rmi4_i2c_read(rmi4_data,
				rmi4_data->f01_data_base_addr + 1,
				&intr_status,
				rmi4_data->num_of_intr_regs);
		if (retval < 0)
			return retval;

		irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_NO_SUSPEND | IRQF_EARLY_RESUME;	

		printk("ngxson: synaptics enable irq\n");
		
		retval = request_threaded_irq(rmi4_data->irq, NULL,
				synaptics_rmi4_irq, irq_flags, DRIVER_NAME, rmi4_data);
		if (retval < 0) {
			printk( "ITUCH : Device(%s), Failed to create irq thread\n", dev_name( &rmi4_data->i2c_client->dev ) );
			return retval;
		}

		rmi4_data->irq_enabled = true;
	} else {
		if (rmi4_data->irq_enabled) {
			printk("ngxson: synaptics disable irq\n");
			disable_irq(rmi4_data->irq);
			free_irq(rmi4_data->irq, rmi4_data);
			rmi4_data->irq_enabled = false;
		}
	}

	return retval;
}

 /**
 * synaptics_rmi4_f11_init()
 *
 * Called by synaptics_rmi4_query_device().
 *
 * This funtion parses information from the Function 11 registers
 * and determines the number of fingers supported, x and y data ranges,
 * offset to the associated interrupt status register, interrupt bit
 * mask, and gathers finger data acquisition capabilities from the query
 * registers.
 */
static int synaptics_rmi4_f11_init(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler,
		struct synaptics_rmi4_fn_desc *fd,
		unsigned int intr_count)
{
	int retval;
	unsigned char ii;
	unsigned char intr_offset;
	unsigned char abs_data_size;
	unsigned char query[F11_STD_QUERY_LEN];
	unsigned char control[F11_STD_CTRL_LEN];

	fhandler->fn_number = fd->fn_number;
	fhandler->num_of_data_sources = fd->intr_src_count;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			fhandler->full_addr.query_base,
			query,
			sizeof(query));
	if (retval < 0)
		return retval;

	/* Maximum number of fingers supported */
	if ((query[1] & MASK_3BIT) <= 4)
		fhandler->num_of_data_points = (query[1] & MASK_3BIT) + 1;
	else if ((query[1] & MASK_3BIT) == 5)
		fhandler->num_of_data_points = 10;

	rmi4_data->num_of_fingers = fhandler->num_of_data_points;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			fhandler->full_addr.ctrl_base,
			control,
			sizeof(control));
	if (retval < 0)
		return retval;

	/* Maximum x and y */
	rmi4_data->sensor_max_x = ((control[6] & MASK_8BIT) << 0) |
			((control[7] & MASK_4BIT) << 8);
	rmi4_data->sensor_max_y = ((control[8] & MASK_8BIT) << 0) |
			((control[9] & MASK_4BIT) << 8);
	dev_dbg(&rmi4_data->i2c_client->dev,
			"%s: Function %02x max x = %d max y = %d\n",
			__func__, fhandler->fn_number,
			rmi4_data->sensor_max_x,
			rmi4_data->sensor_max_y);

	control[0] = 0x9;
	retval = synaptics_rmi4_i2c_write(rmi4_data,
			fhandler->full_addr.ctrl_base,
			control,
			sizeof(control));

	fhandler->intr_reg_num = (intr_count + 7) / 8;
	if (fhandler->intr_reg_num != 0)
		fhandler->intr_reg_num -= 1;

	/* Set an enable bit for each data source */
	intr_offset = intr_count % 8;
	fhandler->intr_mask = 0;
	for (ii = intr_offset;
			ii < ((fd->intr_src_count & MASK_3BIT) +
			intr_offset);
			ii++)
		fhandler->intr_mask |= 1 << ii;

	abs_data_size = query[5] & MASK_2BIT;
	data_reg_blk_size = 3 + (2 * (abs_data_size == 0 ? 1 : 0));

	return retval;
}

static int synaptics_rmi4_alloc_fh(struct synaptics_rmi4_fn **fhandler,
		struct synaptics_rmi4_fn_desc *rmi_fd, int page_number, bool isf11)
{
	*fhandler = kzalloc(sizeof(**fhandler), GFP_KERNEL);
	if (!(*fhandler))
		return -ENOMEM;

	if(isf11) {
		data_addr =
			(rmi_fd->data_base_addr |
			(page_number << 8));
		ctrl_addr =
			(rmi_fd->ctrl_base_addr |
			(page_number << 8));
		(*fhandler)->full_addr.ctrl_base = ctrl_addr;
	}
	(*fhandler)->full_addr.data_base =
			(rmi_fd->data_base_addr |
			(page_number << 8));
	(*fhandler)->full_addr.cmd_base =
			(rmi_fd->cmd_base_addr |
			(page_number << 8));
	(*fhandler)->full_addr.query_base =
			(rmi_fd->query_base_addr |
			(page_number << 8));

	return 0;
}

 /**
 * synaptics_rmi4_query_device()
 *
 * Called by synaptics_rmi4_probe().
 *
 * This funtion scans the page description table, records the offsets
 * to the register types of Function $01, sets up the function handlers
 * for Function $11 and Function $12, determines the number of interrupt
 * sources from the sensor, adds valid Functions with data inputs to the
 * Function linked list, parses information from the query registers of
 * Function $01, and enables the interrupt sources from the valid Functions
 * with data inputs.
 */
static int synaptics_rmi4_query_device(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char ii;
	unsigned char page_number;
	unsigned char intr_count = 0;
	unsigned char data_sources = 0;
	unsigned char f01_query[F01_STD_QUERY_LEN];
	unsigned short pdt_entry_addr;
	unsigned short intr_addr;
	struct synaptics_rmi4_f01_device_status status;
	struct synaptics_rmi4_fn_desc rmi_fd;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	INIT_LIST_HEAD(&rmi->support_fn_list);

	/* Scan the page description tables of the pages to service */
	for (page_number = 0; page_number < PAGES_TO_SERVICE; page_number++) {
		for (pdt_entry_addr = PDT_START; pdt_entry_addr > PDT_END;
				pdt_entry_addr -= PDT_ENTRY_SIZE) {
			pdt_entry_addr |= (page_number << 8);

			retval = synaptics_rmi4_i2c_read(rmi4_data,
					pdt_entry_addr,
					(unsigned char *)&rmi_fd,
					sizeof(rmi_fd));
			if (retval < 0)
				return retval;

			fhandler = NULL;

			if (rmi_fd.fn_number == 0) {
				dev_dbg(&rmi4_data->i2c_client->dev,
						"%s: Reached end of PDT\n",
						__func__);
				break;
			}

			dev_dbg(&rmi4_data->i2c_client->dev,
					"%s: F%02x found (page %d)\n",
					__func__, rmi_fd.fn_number,
					page_number);

			switch (rmi_fd.fn_number) {
			case SYNAPTICS_RMI4_F01:
				rmi4_data->f01_query_base_addr =
						rmi_fd.query_base_addr;
				rmi4_data->f01_ctrl_base_addr =
						rmi_fd.ctrl_base_addr;
				rmi4_data->f01_data_base_addr =
						rmi_fd.data_base_addr;
				rmi4_data->f01_cmd_base_addr =
						rmi_fd.cmd_base_addr;

				retval = synaptics_rmi4_i2c_read(rmi4_data,
						rmi4_data->f01_data_base_addr,
						status.data,
						sizeof(status.data));
				if (retval < 0)
					return retval;

				if (status.flash_prog == 1) {
					pr_notice("%s: In flash prog mode, status = 0x%02x\n",
							__func__,
							status.status_code);
					goto flash_prog_mode;
				}
					break;
			case SYNAPTICS_RMI4_F11:
				if (rmi_fd.intr_src_count == 0)
					break;

				retval = synaptics_rmi4_alloc_fh(&fhandler,
						&rmi_fd, page_number, true);
				if (retval < 0) {
					printk( "ITUCH : Device(%s), Failed to alloc for F%d\n", dev_name( &rmi4_data->i2c_client->dev ), rmi_fd.fn_number );
					return retval;
				}

				retval = synaptics_rmi4_f11_init(rmi4_data,
						fhandler, &rmi_fd, intr_count);
				if (retval < 0)
					return retval;
				break;
			}

			/* Accumulate the interrupt count */
			intr_count += (rmi_fd.intr_src_count & MASK_3BIT);

			if (fhandler && rmi_fd.intr_src_count) {
				list_add_tail(&fhandler->link,
						&rmi->support_fn_list);
			}
		}
	}

flash_prog_mode:
	rmi4_data->num_of_intr_regs = (intr_count + 7) / 8;
	dev_dbg(&rmi4_data->i2c_client->dev,
			"%s: Number of interrupt registers = %d\n",
			__func__, rmi4_data->num_of_intr_regs);

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_query_base_addr,
			f01_query,
			sizeof(f01_query));
	if (retval < 0)
		return retval;

	/* RMI Version 4.0 currently supported */
	rmi->version_major = 4;
	rmi->version_minor = 0;

	rmi->manufacturer_id = f01_query[0];
	rmi->product_props = f01_query[1];
	rmi->product_info[0] = f01_query[2] & MASK_7BIT;
	rmi->product_info[1] = f01_query[3] & MASK_7BIT;
	rmi->date_code[0] = f01_query[4] & MASK_5BIT;
	rmi->date_code[1] = f01_query[5] & MASK_4BIT;
	rmi->date_code[2] = f01_query[6] & MASK_5BIT;
	rmi->tester_id = ((f01_query[7] & MASK_7BIT) << 8) |
			(f01_query[8] & MASK_7BIT);
	rmi->serial_number = ((f01_query[9] & MASK_7BIT) << 8) |
			(f01_query[10] & MASK_7BIT);
	memcpy(rmi->product_id_string, &f01_query[11], 10);

	if (rmi->manufacturer_id != 1) {
		printk( "ITUCH : Device(%s), Non-Synaptics device found, manufacturer ID(%d)\n", dev_name( &rmi4_data->i2c_client->dev ), rmi->manufacturer_id );
	}

	memset(rmi4_data->intr_mask, 0x00, sizeof(rmi4_data->intr_mask));

	/*
	 * Map out the interrupt bit masks for the interrupt sources
	 * from the registered function handlers.
	 */
	list_for_each_entry(fhandler, &rmi->support_fn_list, link)
		data_sources += fhandler->num_of_data_sources;
	if (data_sources) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->num_of_data_sources) {
				rmi4_data->intr_mask[fhandler->intr_reg_num] |=
						fhandler->intr_mask;
			}
		}
	}

	/* Enable the interrupt sources */
	for (ii = 0; ii < rmi4_data->num_of_intr_regs; ii++) {
		if (rmi4_data->intr_mask[ii] != 0x00) {
			dev_dbg(&rmi4_data->i2c_client->dev,
					"%s: Interrupt enable mask %d = 0x%02x\n",
					__func__, ii, rmi4_data->intr_mask[ii]);
			intr_addr = rmi4_data->f01_ctrl_base_addr + 1 + ii;
			retval = synaptics_rmi4_i2c_write(rmi4_data,
					intr_addr,
					&(rmi4_data->intr_mask[ii]),
					sizeof(rmi4_data->intr_mask[ii]));
			if (retval < 0)
				return retval;
		}
	}

	return 0;
}

static int synaptics_rmi4_reset_device(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char command = 0x01;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	retval = synaptics_rmi4_i2c_write(rmi4_data,
			rmi4_data->f01_cmd_base_addr,
			&command,
			sizeof(command));
	if (retval < 0) {
		printk( "ITUCH : Device(%s), Failed to issue reset command, error(%d)\n", dev_name( &rmi4_data->i2c_client->dev ), retval );
		return retval;
	}

	msleep(100);

	list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			kfree(fhandler->data);
		kfree(fhandler);
	}

	retval = synaptics_rmi4_query_device(rmi4_data);
	if (retval < 0) {
		printk( "ITUCH : Device(%s), Failed to query device\n", dev_name( &rmi4_data->i2c_client->dev ) );
		return retval;
	}

	return 0;
}

/**
* synaptics_rmi4_detection_work()
*
* Called by the kernel at the scheduled time.
*
* This function is a self-rearming work thread that checks for the
* insertion and removal of other expansion Function modules such as
* rmi_dev and calls their initialization and removal callback functions
* accordingly.
*/
static void synaptics_rmi4_detection_work(struct work_struct *work)
{
	struct synaptics_rmi4_exp_fn *exp_fhandler, *next_list_entry;
	struct synaptics_rmi4_data *rmi4_data =
			container_of(work, struct synaptics_rmi4_data,
			det_work.work);

	static unsigned	count = 0;

	if( count < 5 )
	{
		queue_delayed_work(rmi4_data->det_workqueue,
				&rmi4_data->det_work,
				msecs_to_jiffies(EXP_FN_DET_INTERVAL));
		++count;
	}

	mutex_lock(&exp_fn_list_mutex);
	if (!list_empty(&exp_fn_list)) {
		list_for_each_entry_safe(exp_fhandler,
				next_list_entry,
				&exp_fn_list,
				link) {
			if ((exp_fhandler->func_init != NULL) &&
					(exp_fhandler->inserted == false)) {
				exp_fhandler->func_init(rmi4_data);
				exp_fhandler->inserted = true;
			} else if ((exp_fhandler->func_init == NULL) &&
					(exp_fhandler->inserted == true)) {
				exp_fhandler->func_remove(rmi4_data);
				list_del(&exp_fhandler->link);
				kfree(exp_fhandler);
			}
		}
	}
	mutex_unlock(&exp_fn_list_mutex);

	return;
}

/**
* synaptics_rmi4_new_function()
*
* Called by other expansion Function modules in their module init and
* module exit functions.
*
* This function is used by other expansion Function modules such as
* rmi_dev to register themselves with the driver by providing their
* initialization and removal callback function pointers so that they
* can be inserted or removed dynamically at module init and exit times,
* respectively.
*/
void synaptics_rmi4_new_function(enum exp_fn fn_type, bool insert,
		int (*func_init)(struct synaptics_rmi4_data *rmi4_data),
		void (*func_remove)(struct synaptics_rmi4_data *rmi4_data),
		void (*func_attn)(struct synaptics_rmi4_data *rmi4_data,
		unsigned char intr_mask))
{
	struct synaptics_rmi4_exp_fn *exp_fhandler;

	if (!exp_fn_inited) {
		mutex_init(&exp_fn_list_mutex);
		INIT_LIST_HEAD(&exp_fn_list);
		exp_fn_inited = 1;
	}

	mutex_lock(&exp_fn_list_mutex);
	if (insert) {
		exp_fhandler = kzalloc(sizeof(*exp_fhandler), GFP_KERNEL);
		if (!exp_fhandler) {
			printk( "ITUCH : Failed to alloc mem for expansion function\n" );
			goto exit;
		}
		exp_fhandler->fn_type = fn_type;
		exp_fhandler->func_init = func_init;
		exp_fhandler->func_attn = func_attn;
		exp_fhandler->func_remove = func_remove;
		exp_fhandler->inserted = false;
		list_add_tail(&exp_fhandler->link, &exp_fn_list);
	} else {
		list_for_each_entry(exp_fhandler, &exp_fn_list, link) {
			if (exp_fhandler->func_init == func_init) {
				exp_fhandler->inserted = false;
				exp_fhandler->func_init = NULL;
				exp_fhandler->func_attn = NULL;
				goto exit;
			}
		}
	}

exit:
	mutex_unlock(&exp_fn_list_mutex);

	return;
}
EXPORT_SYMBOL(synaptics_rmi4_new_function);

 /**
 * synaptics_rmi4_probe()
 *
 * Called by the kernel when an association with an I2C device of the
 * same name is made (after doing i2c_add_driver).
 *
 * This funtion allocates and initializes the resources for the driver
 * as an input driver, turns on the power to the sensor, queries the
 * sensor for its supported Functions and characteristics, registers
 * the driver to the input subsystem, sets up the interrupt, handles
 * the registration of the early_suspend and late_resume functions,
 * and creates a work queue for detection of other expansion Function
 * modules.
 */
static int __devinit synaptics_rmi4_probe(struct i2c_client *client,
		const struct i2c_device_id *dev_id)
{
	int retval;
	unsigned char attr_count;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_data *rmi4_data;
	struct synaptics_rmi4_device_info *rmi;
	const struct synaptics_dsx_platform_data *platform_data =
			client->dev.platform_data;

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		printk( "ITUCH : Device(%s), SMBus byte data not supported\n", dev_name( &client->dev ) );
		return -EIO;
	}

	if (!platform_data) {
		printk( "ITUCH : Device(%s), No platform data found\n", dev_name( &client->dev ) );
		return -EINVAL;
	}

	if( platform_data->enable_touch_power && !platform_data->enable_touch_power() )
	{
		printk( "ITUCH : Enable power failed\n" );
		return	-EFAULT;
	}

	if( platform_data->reset_touch_ic && !platform_data->reset_touch_ic() )
	{
		printk( "ITUCH : Reset touch failed\n" );
		return	-EFAULT;
	}

	rmi4_data = kzalloc(sizeof(*rmi4_data) * 2, GFP_KERNEL);
	if (!rmi4_data) {
		printk( "ITUCH : Device(%s), Failed to alloc mem for rmi4_data\n", dev_name( &client->dev ) );
		return -ENOMEM;
	}

	rmi = &(rmi4_data->rmi4_mod_info);

	rmi4_data->input_dev = input_allocate_device();
	if (rmi4_data->input_dev == NULL) {
		printk( "ITUCH : Device(%s), Failed to allocate input device\n", dev_name( &client->dev ) );
		retval = -ENOMEM;
		goto err_input_device;
	}
/*
	if (platform_data->regulator_en) {
		rmi4_data->regulator = regulator_get(&client->dev, "vdd");
		if (IS_ERR(rmi4_data->regulator)) {
			dev_err(&client->dev,
					"%s: Failed to get regulator\n",
					__func__);
			retval = PTR_ERR(rmi4_data->regulator);
			goto err_regulator;
		}
		regulator_enable(rmi4_data->regulator);
	}
*/
	rmi4_data->i2c_client = client;
	rmi4_data->current_page = MASK_8BIT;
	rmi4_data->board = platform_data;
	rmi4_data->touch_stopped = false;
	rmi4_data->sensor_sleep = false;
	rmi4_data->irq_enabled = false;
	rmi4_data->fingers_on_2d = true;

	rmi4_data->i2c_read = synaptics_rmi4_i2c_read;
	rmi4_data->i2c_write = synaptics_rmi4_i2c_write;
	rmi4_data->irq_enable = synaptics_rmi4_irq_enable;
	rmi4_data->reset_device = synaptics_rmi4_reset_device;

	init_waitqueue_head(&rmi4_data->wait);
	mutex_init(&(rmi4_data->rmi4_io_ctrl_mutex));

	retval = synaptics_rmi4_query_device(rmi4_data);
	if (retval < 0) {
		printk( "ITUCH : Device(%s), Failed to query device\n", dev_name( &client->dev ) );
		goto err_query_device;
	}

	i2c_set_clientdata(client, rmi4_data);

	rmi4_data->input_dev->name = DRIVER_NAME;
	rmi4_data->input_dev->phys = INPUT_PHYS_NAME;
	rmi4_data->input_dev->id.bustype = BUS_I2C;
	rmi4_data->input_dev->dev.parent = &client->dev;
	input_set_drvdata(rmi4_data->input_dev, rmi4_data);

	set_bit(EV_SYN, rmi4_data->input_dev->evbit);
	set_bit(EV_KEY, rmi4_data->input_dev->evbit);
	set_bit(EV_ABS, rmi4_data->input_dev->evbit);

#ifdef INPUT_PROP_DIRECT
	set_bit(INPUT_PROP_DIRECT, rmi4_data->input_dev->propbit);
#endif

	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_POSITION_X, 0,
			rmi4_data->sensor_max_x, 0, 0);
	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_POSITION_Y, 0,
			rmi4_data->sensor_max_y, 0, 0);
	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_TOUCH_MAJOR, 0,
			MAX_ABS_MT_TOUCH_MAJOR, 0, 0);
	//input_set_abs_params(rmi4_data->input_dev,
	//		ABS_MT_PRESSURE, 0,
	//		0xFF, 0, 0);

#ifdef TYPE_B_PROTOCOL
	input_mt_init_slots(rmi4_data->input_dev,
			rmi4_data->num_of_fingers);
#endif

	retval = input_register_device(rmi4_data->input_dev);
	if (retval) {
		printk( "ITUCH : Device(%s), Failed to register input device\n", dev_name( &client->dev ) );
		goto err_register_input;
	}
	
//ngxson_dt2w
	retval = input_register_device(doubletap2wake_pwrdev);
	if (retval < 0) {
		pr_err("%s: Error, failed to register input device r=%d\n",
			dev_name( &client->dev ), retval);
		goto err_register_input;
	}
//end

#ifdef CONFIG_HAS_EARLYSUSPEND
	rmi4_data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	rmi4_data->early_suspend.suspend = synaptics_rmi4_early_suspend;
	rmi4_data->early_suspend.resume = synaptics_rmi4_late_resume;
	register_early_suspend(&rmi4_data->early_suspend);
#endif

	if (!exp_fn_inited) {
		mutex_init(&exp_fn_list_mutex);
		INIT_LIST_HEAD(&exp_fn_list);
		exp_fn_inited = 1;
	}

	rmi4_data->det_workqueue =
			create_singlethread_workqueue("rmi_det_workqueue");
	INIT_DELAYED_WORK(&rmi4_data->det_work,
			synaptics_rmi4_detection_work);
	queue_delayed_work(rmi4_data->det_workqueue,
			&rmi4_data->det_work,
			msecs_to_jiffies(EXP_FN_DET_INTERVAL));

	if (platform_data->gpio_config) {
		retval = platform_data->gpio_config(platform_data->gpio, true);
		if (retval < 0) {
			printk( "ITUCH : Device(%s), Failed to configure GPIO\n", dev_name( &client->dev ) );
			goto err_gpio;
		}
	}

	rmi4_data->irq = gpio_to_irq(platform_data->gpio);

	retval = synaptics_rmi4_irq_enable(rmi4_data, true);
	if (retval < 0) {
		printk( "ITUCH : Device(%s), Failed to enable attention interrupt\n", dev_name( &client->dev ) );
		goto err_enable_irq;
	}

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		retval = sysfs_create_file(&rmi4_data->input_dev->dev.kobj,
				&attrs[attr_count].attr);
		if (retval < 0) {
			printk( "ITUCH : Device(%s), Failed to create sysfs attributes\n", dev_name( &client->dev ) );
			goto err_sysfs;
		}
	}

	printk( "ITUCH : driver is ready\n" );

	REGISTER_TOUCH_TOOL_DRIVER( rmi4_data );

	return retval;

err_sysfs:
	for (attr_count--; attr_count >= 0; attr_count--) {
		sysfs_remove_file(&rmi4_data->input_dev->dev.kobj,
				&attrs[attr_count].attr);
	}

err_enable_irq:
err_gpio:
	input_unregister_device(rmi4_data->input_dev);

err_register_input:
err_query_device:
	if (platform_data->regulator_en) {
		regulator_disable(rmi4_data->regulator);
		regulator_put(rmi4_data->regulator);
	}

	list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if(fhandler->data)
				kfree(fhandler->data);

		kfree(fhandler);

	}

/*
err_regulator:
*/
	input_free_device(rmi4_data->input_dev);
	input_free_device(doubletap2wake_pwrdev);
	rmi4_data->input_dev = NULL;

err_input_device:
	kfree(rmi4_data);

	return retval;
}

 /**
 * synaptics_rmi4_remove()
 *
 * Called by the kernel when the association with an I2C device of the
 * same name is broken (when the driver is unloaded).
 *
 * This funtion terminates the work queue, stops sensor data acquisition,
 * frees the interrupt, unregisters the driver from the input subsystem,
 * turns off the power to the sensor, and frees other allocated resources.
 */
static int __devexit synaptics_rmi4_remove(struct i2c_client *client)
{
	unsigned char attr_count;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_data *rmi4_data = i2c_get_clientdata(client);
	struct synaptics_rmi4_device_info *rmi;
	const struct synaptics_dsx_platform_data *platform_data =
			rmi4_data->board;

	rmi = &(rmi4_data->rmi4_mod_info);

	cancel_delayed_work_sync(&rmi4_data->det_work);
	flush_workqueue(rmi4_data->det_workqueue);
	destroy_workqueue(rmi4_data->det_workqueue);

	rmi4_data->touch_stopped = true;
	wake_up(&rmi4_data->wait);

	synaptics_rmi4_irq_enable(rmi4_data, false);

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		sysfs_remove_file(&rmi4_data->input_dev->dev.kobj,
				&attrs[attr_count].attr);
	}

	input_unregister_device(rmi4_data->input_dev);

	if (platform_data->regulator_en) {
		regulator_disable(rmi4_data->regulator);
		regulator_put(rmi4_data->regulator);
	}

	list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			kfree(fhandler->data);
		kfree(fhandler);
	}

	input_free_device(rmi4_data->input_dev);

	kfree(rmi4_data);

	return 0;
}

#ifdef CONFIG_PM
 /**
 * synaptics_rmi4_sensor_sleep()
 *
 * Called by synaptics_rmi4_early_suspend() and synaptics_rmi4_suspend().
 *
 * This function stops finger data acquisition and puts the sensor to sleep.
 */
static void synaptics_rmi4_sensor_sleep(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char device_ctrl;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		printk( "ITUCH : Device(%s), Failed to enter sleep mode\n", dev_name( &(rmi4_data->input_dev->dev ) ) );
		rmi4_data->sensor_sleep = false;
		return;
	}

	device_ctrl = (device_ctrl & ~MASK_3BIT);
	device_ctrl = (device_ctrl | NO_SLEEP_OFF | SENSOR_SLEEP);

	retval = synaptics_rmi4_i2c_write(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		printk( "ITUCH : Device(%s), Failed to enter sleep mode\n", dev_name( &(rmi4_data->input_dev->dev) ) );
		rmi4_data->sensor_sleep = false;
		return;
	} else {
		rmi4_data->sensor_sleep = true;
	}

	return;
}

 /**
 * synaptics_rmi4_sensor_wake()
 *
 * Called by synaptics_rmi4_resume() and synaptics_rmi4_late_resume().
 *
 * This function wakes the sensor from sleep.
 */

static bool ngxson_touch_slept = false;

static void synaptics_rmi4_sensor_wake(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char device_ctrl;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		printk( "ITUCH : Device(%s), Failed to wake from sleep mode\n", dev_name( &(rmi4_data->input_dev->dev) ) );
		rmi4_data->sensor_sleep = true;
		return;
	}

	device_ctrl = (device_ctrl & ~MASK_3BIT);
	device_ctrl = (device_ctrl | NO_SLEEP_OFF | NORMAL_OPERATION);

	retval = synaptics_rmi4_i2c_write(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		printk( "ITUCH : Device(%s), Failed to wake from sleep mode\n", dev_name( &(rmi4_data->input_dev->dev) ) );
		rmi4_data->sensor_sleep = true;
		return;
	} else {
		rmi4_data->sensor_sleep = false;
	}

	return;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
 /**
 * synaptics_rmi4_early_suspend()
 *
 * Called by the kernel during the early suspend phase when the system
 * enters suspend.
 *
 * This function calls synaptics_rmi4_sensor_sleep() to stop finger
 * data acquisition and put the sensor to sleep.
 */
 
static void reset_device(struct synaptics_rmi4_data *rmi4_data) {

		rmi4_data->touch_stopped = true;
		wake_up(&rmi4_data->wait);
		synaptics_rmi4_irq_enable(rmi4_data, false);
		synaptics_rmi4_sensor_sleep(rmi4_data);
	
		if (rmi4_data->full_pm_cycle) {
			synaptics_rmi4_suspend(&(rmi4_data->input_dev->dev));
		}
		msleep(2);
		reset_all_touch(rmi4_data);
		msleep(2);
		if (rmi4_data->full_pm_cycle) {
			synaptics_rmi4_resume(&(rmi4_data->input_dev->dev));
		}
		synaptics_rmi4_sensor_wake(rmi4_data);
		rmi4_data->touch_stopped = false;
		synaptics_rmi4_irq_enable(rmi4_data, true);

		return;
}

static void synaptics_rmi4_early_suspend(struct early_suspend *h)
{
	struct synaptics_rmi4_data *rmi4_data =
			container_of(h, struct synaptics_rmi4_data,
			early_suspend);

	//printk( "ngxson: debug synaptics_rmi4_early_suspend\n");
	scr_suspended = true;
	if((dt2w_switch>0)||(s2w_switch>0)) nui_report_input = false;
	
	if (no_suspend_touch) {
		s2w_reset();
		s2m_reset();
		doubletap2wake_reset();
		reset_device(rmi4_data);
		synaptics_rmi4_irq_enable(rmi4_data, true);
		//enable_irq(rmi4_data->irq);
		enable_irq_wake(rmi4_data->irq);
		//printk( "ngxson: debug dt2w on\n");
		if (rmi4_data->full_pm_cycle)
			synaptics_rmi4_resume(&(rmi4_data->input_dev->dev));
	
		if (rmi4_data->sensor_sleep == true) {
			synaptics_rmi4_sensor_wake(rmi4_data);
			rmi4_data->touch_stopped = false;
		}
	} else {
		//printk( "ngxson: debug synaptics_rmi4_early_suspend\n");
		ngxson_touch_slept = true;
		rmi4_data->touch_stopped = true;
		wake_up(&rmi4_data->wait);
		synaptics_rmi4_irq_enable(rmi4_data, false);
		synaptics_rmi4_sensor_sleep(rmi4_data);
	
		if (rmi4_data->full_pm_cycle)
			synaptics_rmi4_suspend(&(rmi4_data->input_dev->dev));
	}
	return;
}


 /**
 * synaptics_rmi4_late_resume()
 *
 * Called by the kernel during the late resume phase when the system
 * wakes up from suspend.
 *
 * This function goes through the sensor wake process if the system wakes
 * up from early suspend (without going into suspend).
 */
static void synaptics_rmi4_late_resume(struct early_suspend *h)
{
	struct synaptics_rmi4_data *rmi4_data =
			container_of(h, struct synaptics_rmi4_data,
			early_suspend);

	//printk( "ngxson: debug synaptics_rmi4_late_resume\n");
	scr_suspended = false;
	if(!nui_report_input) nui_report_input = true;
	if ((no_suspend_touch) && (!ngxson_touch_slept)) {
		//printk( "ngxson: debug dt2w on\n");
		doubletap2wake_reset();
		//if(s2w_oneswipe == 0)
		reset_device(rmi4_data);
		disable_irq_wake(rmi4_data->irq);
	} else {
		//printk( "ngxson: debug synaptics_rmi4_late_resume\n");
		ngxson_touch_slept = false;
		if (rmi4_data->full_pm_cycle)
			synaptics_rmi4_resume(&(rmi4_data->input_dev->dev));
	
		if (rmi4_data->sensor_sleep == true) {
			synaptics_rmi4_sensor_wake(rmi4_data);
			rmi4_data->touch_stopped = false;
			synaptics_rmi4_irq_enable(rmi4_data, true);
		}
	}
	return;
}
#endif

 /**
 * synaptics_rmi4_suspend()
 *
 * Called by the kernel during the suspend phase when the system
 * enters suspend.
 *
 * This function stops finger data acquisition and puts the sensor to
 * sleep (if not already done so during the early suspend phase),
 * disables the interrupt, and turns off the power to the sensor.
 */
static int synaptics_rmi4_suspend(struct device *dev)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	const struct synaptics_dsx_platform_data *platform_data =
			rmi4_data->board;

	if (!rmi4_data->sensor_sleep) {
		rmi4_data->touch_stopped = true;
		wake_up(&rmi4_data->wait);
		synaptics_rmi4_irq_enable(rmi4_data, false);
		synaptics_rmi4_sensor_sleep(rmi4_data);
	}

	if (platform_data->regulator_en)
		regulator_disable(rmi4_data->regulator);

	return 0;
}

 /**
 * synaptics_rmi4_resume()
 *
 * Called by the kernel during the resume phase when the system
 * wakes up from suspend.
 *
 * This function turns on the power to the sensor, wakes the sensor
 * from sleep, enables the interrupt, and starts finger data
 * acquisition.
 */
static int synaptics_rmi4_resume(struct device *dev)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	const struct synaptics_dsx_platform_data *platform_data =
			rmi4_data->board;

	if (platform_data->regulator_en)
		regulator_enable(rmi4_data->regulator);

	synaptics_rmi4_sensor_wake(rmi4_data);
	rmi4_data->touch_stopped = false;
	synaptics_rmi4_irq_enable(rmi4_data, true);

	return 0;
}

/*
static const struct dev_pm_ops synaptics_rmi4_dev_pm_ops = {
	.suspend = synaptics_rmi4_suspend,
	.resume  = synaptics_rmi4_resume,


};
*/
#endif

static const struct i2c_device_id synaptics_rmi4_id_table[] = {
	{DRIVER_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, synaptics_rmi4_id_table);

static struct i2c_driver synaptics_rmi4_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		// .pm = &synaptics_rmi4_dev_pm_ops,
#endif
	},
	.probe = synaptics_rmi4_probe,
	.remove = __devexit_p(synaptics_rmi4_remove),
	.id_table = synaptics_rmi4_id_table,
};

 /**
 * synaptics_rmi4_init()
 *
 * Called by the kernel during do_initcalls (if built-in)
 * or when the driver is loaded (if a module).
 *
 * This function registers the driver to the I2C subsystem.
 *
 */
static int __init synaptics_rmi4_init(void)
{
//ngxson-dt2w
	doubletap2wake_pwrdev = input_allocate_device();
	if (!doubletap2wake_pwrdev) {
		pr_err("Can't allocate suspend autotest power button\n");
	}
	doubletap2wake_pwrdev->name = "dt2w_pwrkey";
	doubletap2wake_pwrdev->phys = "dt2w_pwrkey/input0";
	input_set_capability(doubletap2wake_pwrdev, EV_KEY, KEY_POWER);
	input_set_capability(doubletap2wake_pwrdev, EV_KEY, KEY_CAMERA_SNAPSHOT);
	input_set_capability(doubletap2wake_pwrdev, EV_KEY, KEY_CAMERA_FOCUS);
	input_set_capability(doubletap2wake_pwrdev, EV_KEY, KEY_BACK);
	input_set_capability(doubletap2wake_pwrdev, EV_KEY, KEY_NEXTSONG);
	input_set_capability(doubletap2wake_pwrdev, EV_KEY, KEY_PLAYPAUSE);
	input_set_capability(doubletap2wake_pwrdev, EV_KEY, KEY_PREVIOUSSONG);
	input_set_capability(doubletap2wake_pwrdev, EV_KEY, KEY_SEARCH);
	input_set_capability(doubletap2wake_pwrdev, EV_KEY, KEY_MENU);
	input_set_capability(doubletap2wake_pwrdev, EV_KEY, KEY_HOMEPAGE);
	input_set_capability(doubletap2wake_pwrdev, EV_KEY, KEY_VOLUMEDOWN);
	input_set_capability(doubletap2wake_pwrdev, EV_KEY, KEY_VOLUMEUP);
	input_set_capability(doubletap2wake_pwrdev, EV_KEY, 580);
	wake_lock_init(&dt2w_wake_lock, WAKE_LOCK_SUSPEND, "dt2w");
//end
	return i2c_add_driver(&synaptics_rmi4_driver);
}

 /**
 * synaptics_rmi4_exit()
 *
 * Called by the kernel when the driver is unloaded.
 *
 * This funtion unregisters the driver from the I2C subsystem.
 *
 */
static void __exit synaptics_rmi4_exit(void)
{
	wake_lock_destroy(&dt2w_wake_lock);
	i2c_del_driver(&synaptics_rmi4_driver);
}

module_param(hn_one, int, 0644);
module_param(hn_two, int, 0644);
module_param(hn_thr, int, 0644);
module_param(hn_m, int, 0644);
module_param(block_gesture, bool, 0644);

module_init(synaptics_rmi4_init);
module_exit(synaptics_rmi4_exit);

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics RMI4 I2C Touch Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(SYNAPTICS_RMI4_DRIVER_VERSION);
