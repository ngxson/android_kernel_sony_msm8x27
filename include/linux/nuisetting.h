/*
 * include/linux/nuisetting.h
 *
 * Copyright (c) 2015, Nguyen Xuan Son
 *
 * SETTINGS FOR NUI KERNEL
 *
 */

#ifndef _LINUX_NUISETTING_H
#define _LINUX_NUISETTING_H
#define NUI_BATT_SAV_FREQ_LEV	7
#define NUI_BATT_SAV_FREQ_LEV_TWO	9
#define NUI_BATT_SAV_FREQ_LEV_THREE	11

/*
 * For reading battery level
 */
extern int nui_batt_level;

/*
 * Nui brightness settings
 * 0 for stock value
 * 1 for darker (default)
 * 2 for very dark
 */
extern int nbr_switch;

/*
 * Voltage level for short vibration (haptic, navkay, etc) setting
 * 0 for disable (default)
 * 1 for enable
 */
extern int level_short_switch;

/*
 * Prevent 0% battery level settings
 * 0 for disable (default)
 * 1 for enable
 */
extern int zero_precent_switch;

/*
 * Battery saving mode (no overclock)
 * 0 for disable
 * 1 for enable (default)
 * 2 for 1.4GHz
 * 3 for 1.2GHz
 */
extern int nui_batt_sav;
extern int nui_old_freq;
void nui_batt_sav_mode(int m);

/*
 * Camera/focus key settings
 */
extern int camera_key;
extern int focus_key;
extern bool key_scroff;
extern int camera_key_scroff;
extern int focus_key_scroff;
void btn_press(int i, bool b);

//logo tool
void draw_nui_logo(int logo);

/*
 * Torch intensity
 * 0 for TORCH_46P88_MA
 * 1 for TORCH_93P74_MA
 * 2 for TORCH_140P63_MA (default)
 */
extern int nui_torch_intensity;
void nui_fire_torch(bool on);

/*
 * Proximity sensor sensitive
 * 0 for 2cm
 * 1 for 5cm
 */
extern int nui_proximity_sens;
void nui_proximity_sensitive(int i);

/*
 * Lock brightness
 */
extern int brlock;

/*
 * Detect which of 2 volume keys was preaaed
 * Use for choosing which recovery to boot
 * 0 for none (waiting)
 * 1 for vol down
 * 2 for vol up
 */
extern int nui_which_vol;

// Press focus to turn on torch
extern bool nui_call;

//hide navigation bar mode
extern bool hn_enable;
void hn_update_sett(void);

//camera exposure gain
extern bool cam_gain;
extern bool nui_cam_rec;
extern bool cam_rec_30fps;
extern bool nui_camera_on;

//breathing light
extern unsigned int breathing_interval;

#endif	/* _LINUX_NUISETTING_H */
