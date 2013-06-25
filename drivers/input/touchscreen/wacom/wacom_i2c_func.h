/*
 *  wacom_i2c_func.h - Wacom G5 Digitizer Controller (I2C bus)
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

 #ifndef _LINUX_WACOM_I2C_FUNC_H
#define _LINUX_WACOM_I2C_FUNC_H

#ifdef CONFIG_SEC_TOUCHSCREEN_DVFS_LOCK
#include <mach/cpufreq.h>
#define SEC_DVFS_LOCK_TIMEOUT 3
#endif

#ifdef SEC_BUS_LOCK
#include <mach/dev.h>
#define SEC_DVFS_LOCK_TIMEOUT_MS	200
#define SEC_BUS_LOCK_FREQ		267160
#define SEC_BUS_LOCK_FREQ2	400200
#endif

#ifdef COOR_WORK_AROUND
extern unsigned char screen_rotate;
extern unsigned char user_hand;
#endif

extern unsigned int sttg_boxfilter_threshold;
extern unsigned int sttg_boxfilter_threshold_hover;
extern unsigned int sttg_boost_freq;
extern unsigned int sttg_min_pressure;
extern unsigned int sttg_side_button_mode;
extern unsigned int sttg_fixed_pressure;
extern unsigned int sttg_avg_strength;
extern int sttg_offset_override_x;
extern int sttg_offset_override_y;
extern bool sttg_offset_override_enabled;
extern unsigned int sttg_touchkey_block_length;

#define WACOM_I2C_STOP		0x30
#define WACOM_I2C_START		0x31
#define WACOM_I2C_GRID_CHECK	0xC9
#define WACOM_STATUS			0xD8

extern int g_aveLevel_C[];
extern int g_aveLevel_X[];
extern int g_aveLevel_Y[];
extern int g_aveLevel_Trs[];
extern int g_aveLevel_Cor[];
extern int g_aveShift;

extern int wacom_i2c_send(struct wacom_i2c *wac_i2c,
			  const char *buf, int count, bool mode);
extern int wacom_i2c_recv(struct wacom_i2c *wac_i2c,
			char *buf, int count, bool mode);
extern int wacom_i2c_test(struct wacom_i2c *wac_i2c);
extern int wacom_i2c_coord(struct wacom_i2c *wac_i2c);
extern int wacom_i2c_query(struct wacom_i2c *wac_i2c);
extern int wacom_checksum(struct wacom_i2c *wac_i2c);
extern void forced_release(struct wacom_i2c *wac_i2c);
#ifdef WACOM_PDCT_WORK_AROUND
extern void forced_hover(struct wacom_i2c *wac_i2c);
#endif

#ifdef WACOM_IRQ_WORK_AROUND
extern void wacom_i2c_pendct_work(struct work_struct *work);
#endif

#ifdef CONFIG_SEC_TOUCHSCREEN_DVFS_LOCK
extern void free_dvfs_lock(struct work_struct *work);
#endif

extern void epen_activity_lockout_free(struct work_struct *work);
extern int epen_cpufreq_level;

#endif	/* _LINUX_WACOM_I2C_FUNC_H */
