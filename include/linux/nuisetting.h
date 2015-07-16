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

extern int nui_proximity_sens;
void nui_proximity_sensitive(int i);

/*
 * Lock brightness
 */
extern int brlock;

#endif	/* _LINUX_NUISETTING_H */
