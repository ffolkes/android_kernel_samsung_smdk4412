/*
 * Copyright (C) 2012 The CyanogenMod Project
 *
 * Authors: Daniel Hillenbrand <codeworkx@cyanogenmod.com>
 *          Marco Hillenbrand <marco.hillenbrand@googlemail.com>
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

void touchscreen_state_report(int state);

bool tsp_check_touched_flag(unsigned int mode);

void set_mali_asv(unsigned int level);
unsigned int get_mali_asv(void);
unsigned int get_mali_cur_freq(void);

void set_mali_step_freq(unsigned int level, unsigned int freq);
void set_mali_step_volt(unsigned int level, unsigned int volt);

unsigned int get_mali_freq_by_step(unsigned int level);
unsigned int get_mali_volt_by_step(unsigned int level);

unsigned int get_mali_up_threshold_by_step(unsigned int level);
unsigned int get_mali_down_threshold_by_step(unsigned int level);

void set_mali_up_threshold_by_step(unsigned int level, unsigned int threshold);
void set_mali_down_threshold_by_step(unsigned int level, unsigned int threshold);

void set_mali_step_lock(unsigned int level);
unsigned int get_mali_step_lock(void);

void set_mali_step_limit(unsigned int level);
unsigned int get_mali_step_limit(void);

void set_mali_time_in_state(unsigned int mode);
u64 get_mali_time_in_state(unsigned int level);

unsigned int get_mali_utilization_timeout(void);
void set_mali_utilization_timeout(unsigned int ms);
