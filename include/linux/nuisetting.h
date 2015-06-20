/*
 * include/linux/nuisetting.h
 *
 * Copyright (c) 2013, Dennis Rassmann <showp1984@gmail.com>
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
 *
 * SETTINGS FOR NUI KERNEL
 *
 */

#ifndef _LINUX_NUISETTING_H
#define _LINUX_NUISETTING_H

extern int nui_batt_level;

/*
 * Nui brightness settings
 * 0 for stock value
 * 1 for darker
 * 2 for very dark
 */
extern int nbr_switch;

/*
 * Voltage level for short vibration (haptic, navkay, etc) setting
 * 0 for disable
 * 1 for enable
 */
extern int level_short_switch;

/*
 * Prevent 0% battery level settings
 * 0 for disable
 * 1 for enable
 */
extern int zero_precent_switch;

#endif	/* _LINUX_NUISETTING_H */
