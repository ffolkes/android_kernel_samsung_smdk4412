/*
 *  Copyright (C) 2012, Samsung Electronics Co. Ltd. All Rights Reserved.
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
 */
#include "ssp.h"

#ifdef CONFIG_TOUCH_WAKE
#include <linux/touch_wake.h>
#endif

extern void press_button(int keycode, bool delayed, bool force, bool elastic);
extern unsigned int flg_ctr_cpuboost;

static long ary_kw_gyro_y_history[25] = { 50, 50, 50, 50, 50,
										50, 50, 50, 50, 50,
										50, 50, 50, 50, 50,
										50, 50, 50, 50, 50,
										50, 50, 50, 50, 50 };
static unsigned int ctr_kw_gyro_y_history = 0;
int ctr_kw_gyro_skip = 0;
static int ctr_kw_gyro_recheck = 0;
static unsigned int ctr_max_unstable = 0;
static int mag_z = 0;
static bool flg_primethetime = true;
unsigned int ctr_knocks;
static struct timeval time_kw_knock1;
static struct timeval time_kw_knock2;
static struct timeval time_kw_knock3;
static struct timeval time_kw_knock4;

bool flg_kw_pressedpower = false;

static void deferred_knock2_work(struct work_struct * work_deferred_knock2);
static DECLARE_DELAYED_WORK(work_deferred_knock2, deferred_knock2_work);

static void deferred_knock2_work(struct work_struct * work_deferred_knock2)
{
	ssp_dbg("[SSP/ka] KNOCK 2 ------------------------ !!! TWO 2 DEFERRED starting -----------\n", NULL);
	
	// check to see if there have been any more knocks.
	if (ctr_knocks == 2 && sttg_ka_mode && flg_screen_on) {
		// we're still at 2.
		// now we know there won't be a three, so do 2's action now.
		
		ssp_dbg("[SSP/ka] KNOCK 2 ------------------------ !!! TWO 2 DEFERRED valid! -----------\n", NULL);
			
		ctr_knocks = 0;
		
		press_button(sttg_ka_knock2_keycode, sttg_ka_knock2_keydelay, true, false);

	}
	
	return;
}

/*************************************************************************/
/* SSP Kernel -> HAL input evnet function                                */
/*************************************************************************/
void convert_acc_data(s16 *iValue)
{
	if (*iValue > 2048)
		*iValue = ((4096 - *iValue)) * (-1);
}

void report_acc_data(struct ssp_data *data, struct sensor_value *accdata)
{
	convert_acc_data(&accdata->x);
	convert_acc_data(&accdata->y);
	convert_acc_data(&accdata->z);

	data->buf[ACCELEROMETER_SENSOR].x = accdata->x - data->accelcal.x;
	data->buf[ACCELEROMETER_SENSOR].y = accdata->y - data->accelcal.y;
	data->buf[ACCELEROMETER_SENSOR].z = accdata->z - data->accelcal.z;
	
	//ssp_dbg("[SSP] accel y: %ld\n",
	//		data->buf[ACCELEROMETER_SENSOR].y);

	input_report_rel(data->acc_input_dev, REL_X,
		data->buf[ACCELEROMETER_SENSOR].x);
	input_report_rel(data->acc_input_dev, REL_Y,
		data->buf[ACCELEROMETER_SENSOR].y);
	input_report_rel(data->acc_input_dev, REL_Z,
		data->buf[ACCELEROMETER_SENSOR].z);
	input_sync(data->acc_input_dev);
}

void report_gyro_data(struct ssp_data *data, struct sensor_value *gyrodata)
{
	long lTemp[3] = {0,};
	long tmp_sum = 0;
	int tmp_avg = 0;
	unsigned int i;
	unsigned int ctr_unstable = 0;
	bool flg_stable = true;
	struct timeval time_kw_knocknow;
	unsigned int time_since_knock1;
	unsigned int time_since_knock2;
	unsigned int time_since_knock3;
	
	data->buf[GYROSCOPE_SENSOR].x = gyrodata->x - data->gyrocal.x;
	data->buf[GYROSCOPE_SENSOR].y = gyrodata->y - data->gyrocal.y;
	data->buf[GYROSCOPE_SENSOR].z = gyrodata->z - data->gyrocal.z;

	if (data->uGyroDps == GYROSCOPE_DPS250)	{
		lTemp[0] = (long)data->buf[GYROSCOPE_SENSOR].x >> 1;
		lTemp[1] = (long)data->buf[GYROSCOPE_SENSOR].y >> 1;
		lTemp[2] = (long)data->buf[GYROSCOPE_SENSOR].z >> 1;
	} else if (data->uGyroDps == GYROSCOPE_DPS2000)	{
		lTemp[0] = (long)data->buf[GYROSCOPE_SENSOR].x << 2;
		lTemp[1] = (long)data->buf[GYROSCOPE_SENSOR].y << 2;
		lTemp[2] = (long)data->buf[GYROSCOPE_SENSOR].z << 2;
	} else {
		lTemp[0] = (long)data->buf[GYROSCOPE_SENSOR].x;
		lTemp[1] = (long)data->buf[GYROSCOPE_SENSOR].y;
		lTemp[2] = (long)data->buf[GYROSCOPE_SENSOR].z;
	}
	
	/*if (mag_z > 2035 || mag_z < 1950) {
		ctr_kw_gyro_skip = 1;
		ctr_kw_gyro_recheck = 0;
		ssp_dbg("[SSP/kw] gyro MAG SKIP mag_z: %i\n", mag_z);
	}*/
	
	// screen-off check. also run on 2nd knock, to check for 3rd knock.
	if (sttg_kw_mode > 0 && (sttg_ka_mode || !flg_screen_on || ctr_knocks > 1)) {
		
		if (ctr_kw_gyro_skip > 0) {
			
			// skip
			//////////////////////////////////////////////
			
			//kw_status_code = 60;
			
			if (sttg_kw_debug)
				ssp_dbg("[SSP/kw] SKIP #%i gyro x: %ld, y: %ld, z: %ld\n", ctr_kw_gyro_skip,
						lTemp[0], lTemp[1], lTemp[2]);
			
			if (ctr_kw_gyro_skip == 1 && ctr_max_unstable >= sttg_kw_extreme_instab_tolerance) {
				// if we're here, it means we were sent here most likely by the extreme instability counter.
				// so we're still in panic mode, and this is the last cycle of the ignore it requested.
				// see if the axis has stabilized, and if not, request more ignores.
				
				ssp_dbg("[SSP/kw] SKIP - CHECKING STABILITY\n", ctr_kw_gyro_skip);
				
				if (lTemp[0] <= sttg_kw_gyro_x_max_threshold && lTemp[0] >= sttg_kw_gyro_x_min_threshold
					&& lTemp[1] <= sttg_kw_gyro_y_max_threshold && lTemp[1] >= sttg_kw_gyro_y_min_threshold
					&& lTemp[2] <= sttg_kw_gyro_z_max_threshold && lTemp[2] >= sttg_kw_gyro_z_min_threshold) {
					// axis back within calm range, cancel panic mode.
					
					ssp_dbg("[SSP/kw] SKIP - STABILIZED\n", ctr_kw_gyro_skip);
					
					// reset the history so we don't have to spend 25 cycles clearing the unstable data out.
					ctr_max_unstable = 0;
					for (i = 0; i < 25; i++) {
						ary_kw_gyro_y_history[i] = 50;
					}
					
				} else {
					// maintain panic mode, axis still unstable.
					
					kw_status_code = 62;
					
					// reset the skip counter to the panic value.
					ctr_kw_gyro_skip = sttg_kw_extreme_instab_suspensions;
				}
			}// else {
			//	if (sttg_kw_debug)
			//		ssp_dbg("[SSP/kw] SKIP - DIDN'T CHECK STABILITY %d %d %d\n", ctr_kw_gyro_skip, ctr_max_unstable, sttg_kw_extreme_instab_suspensions);
			//}
			
			ctr_kw_gyro_skip--;
			
		} else if (lTemp[0] < sttg_kw_gyro_x_min_threshold || lTemp[0] > sttg_kw_gyro_x_max_threshold || lTemp[2] < sttg_kw_gyro_z_min_threshold || lTemp[2] > sttg_kw_gyro_z_max_threshold || abs(lTemp[1]) > 3000) {
			
			// massive instability on y or z-axis
			//////////////////////////////////////////////
			
			kw_status_code = 61;
			
			if (sttg_kw_debug)
				ssp_dbg("[SSP/kw] SKIP X/Y/Z %i XXXXXXXXXXXXXXXX massive instability on y or z. gyro x: %ld (%d-%d), y: %ld (>3000), z: %ld (%d-%d), resetting!\n",
						ctr_kw_gyro_skip, lTemp[0], sttg_kw_gyro_x_min_threshold, sttg_kw_gyro_x_max_threshold, lTemp[1], lTemp[2], sttg_kw_gyro_z_min_threshold, sttg_kw_gyro_z_max_threshold);
			ctr_knocks = 0;
			ctr_kw_gyro_recheck = 0;
			
			// TODO: sttg_kw_extreme_instab_suspensions_retry.
			// wait some time for the z-axis instability to stop.
			ctr_kw_gyro_skip = sttg_kw_z_instab_suspensions;
			
		} else {
			
			if (sttg_kw_debug)
				ssp_dbg("[SSP/kw] EVENT   gyro x: %ld, y: %ld, z: %ld\n", lTemp[0], lTemp[1], lTemp[2]);
			
			if (kw_status_code > 20) {
				kw_status_code = 1;
			}
			
			for (i = 0; i < sttg_kw_gyro_y_history; i++) {
				
				if (ary_kw_gyro_y_history[i] < sttg_kw_gyro_y_max_threshold && ary_kw_gyro_y_history[i] > sttg_kw_gyro_y_min_threshold) {
					// stable.
					
					tmp_sum += ary_kw_gyro_y_history[i];
					
				} else {
					// this item exceeded the idle thresholds.
					
					ctr_unstable++;
					
					ssp_dbg("[SSP/kw] gyro y: %ld unstable %i time(s)\n", ary_kw_gyro_y_history[i], ctr_unstable);
					
					if (ctr_unstable > sttg_kw_instab_tolerance) {
						// we been unstable enough times to give up.
						
						flg_stable = false;
						ctr_max_unstable++;
						
						if (ctr_max_unstable >= sttg_kw_extreme_instab_tolerance) {
							// way too much instability, stop everything for a few cycles.
							
							ctr_kw_gyro_skip = sttg_kw_extreme_instab_suspensions;
							kw_status_code = 63;
							ssp_dbg("[SSP/kw] EXTREME instability gyro y: %ld unstable %i time(s)\n", ary_kw_gyro_y_history[i], ctr_unstable);
						}
						
						ctr_knocks = 0;
						
						ssp_dbg("[SSP/kw] gyro y: %ld totally unstable, failed %i time(s), totally unstable counter: %i\n", ary_kw_gyro_y_history[i], ctr_unstable, ctr_max_unstable);
						
						break;
					}
				}
			}
			
			if (ctr_max_unstable > 0 && ctr_unstable == 0) {
				// we've achieved some stability.
				ctr_max_unstable = 0;
				ssp_dbg("[SSP/kw] last average count had no spikes, we're stable. gyro y: %ld ----\n", lTemp[1]);
			}
			
			if (flg_stable) {
				
				if (ctr_kw_gyro_recheck > 0) {
					
					ssp_dbg("[SSP/kw] RECHECK #%d starting... gyro y: %ld ----\n", ctr_kw_gyro_recheck, lTemp[1]);
					
					ctr_kw_gyro_recheck--;
					
					if (!ctr_kw_gyro_recheck && lTemp[1] <= sttg_kw_gyro_y_max_threshold && lTemp[1] >= sttg_kw_gyro_y_min_threshold) {
						// we're done with the rechecks.
						// check current value, then trigger!
						
						ssp_dbg("[SSP/kw] RECHECK #%d ACCEPTED! gyro y: %ld getting nowtime ----\n", (ctr_kw_gyro_recheck + 1), lTemp[1]);
						
						/////////////////////////////////////////////////
						// KNOCKED!
						/////////////////////////////////////////////////
						
						do_gettimeofday(&time_kw_knocknow);
						
						if (flg_primethetime) {
							
							do_gettimeofday(&time_kw_knock1);
							do_gettimeofday(&time_kw_knock2);
							do_gettimeofday(&time_kw_knock3);
							do_gettimeofday(&time_kw_knock4);
							
							kw_status_code = 11;
							time_since_knock1 = 1;
							flg_primethetime = false;
							ctr_knocks = 1;
							
							ssp_dbg("[SSP/kw] KNOCK gyro y: %ld ------------------------ !!! ONE 1 FIRST -----------\n", lTemp[1]);
							
						} else {
							
							time_since_knock1 = (time_kw_knocknow.tv_sec - time_kw_knock1.tv_sec) * MSEC_PER_SEC +
												(time_kw_knocknow.tv_usec - time_kw_knock1.tv_usec) / USEC_PER_MSEC;
							
							time_since_knock2 = (time_kw_knocknow.tv_sec - time_kw_knock2.tv_sec) * MSEC_PER_SEC +
												(time_kw_knocknow.tv_usec - time_kw_knock2.tv_usec) / USEC_PER_MSEC;
							
							time_since_knock3 = (time_kw_knocknow.tv_sec - time_kw_knock3.tv_sec) * MSEC_PER_SEC +
												(time_kw_knocknow.tv_usec - time_kw_knock3.tv_usec) / USEC_PER_MSEC;
							
							if (ctr_knocks == 3) {
								// this is possibly our fourth knock.
								
								if (time_since_knock3 >= sttg_kw_knock_min_speed && time_since_knock3 <= sttg_kw_knock4_window) {
									
									kw_status_code = 14;
									
									ssp_dbg("[SSP/kw] KNOCK 4? - gyro y: %ld --- ctr_knocks was: %d ----\n", lTemp[1], ctr_knocks);
									
									if (flg_kw_pressedpower) {
										// our fourth knock.
										// save the time.
										
										flg_ctr_cpuboost = 25;
										
										do_gettimeofday(&time_kw_knock4);
										
										// no more knocks after this.
										ctr_knocks = 0;
										
										/////////////////////////////////////////////////
										// KNOCK #4 (FOUR)
										/////////////////////////////////////////////////
										
										ssp_dbg("[SSP/kw] KNOCK 4 - gyro y: %ld ------------------------ !!! FOUR 4 -----------\n",
												lTemp[1]);
										// knockwake only, no knockactive.
										
										// last knock, just press the button.
										
										ssp_dbg("[SSP/kw] KNOCK 4 - gyro y: %ld ------------------------ !!! FOUR 4 - PRESSING %d -----------\n",
												lTemp[1], sttg_kw_knock3_keycode);
										
										press_button(sttg_kw_knock4_keycode, sttg_kw_knock4_keydelay, true, false);
										
									}
									
								} else if (sttg_kw_strict_validation) {
									// this knock failed its validation.
									
									kw_status_code = 67;
									ctr_knocks = 0;
									ctr_kw_gyro_skip = 10;
								}
								
							} else if (ctr_knocks == 2 && time_since_knock2 <= sttg_kw_knock3_window) {
								// this is possibly our third knock.
								
								ssp_dbg("[SSP/kw] KNOCK 3? - gyro y: %ld --- ctr_knocks was: %d ----\n", lTemp[1], ctr_knocks);
								
								if (((!flg_screen_on || flg_kw_pressedpower) && time_since_knock2 >= sttg_kw_knock_min_speed && time_since_knock2 <= sttg_kw_knock3_window)
									|| (sttg_ka_mode && flg_screen_on && time_since_knock2 >= sttg_ka_knock3_min_speed && time_since_knock2 <= sttg_ka_knock3_window)) {
									// our third knock.
									// save the time.
									
									kw_status_code = 13;
									do_gettimeofday(&time_kw_knock3);
									ctr_knocks++;
									
									/////////////////////////////////////////////////
									// KNOCK #3 (THREE)
									/////////////////////////////////////////////////
									
									ssp_dbg("[SSP/kw] KNOCK 3 - gyro y: %ld ------------------------ !!! THREE 3 -----------\n",
											lTemp[1]);
									
									if (sttg_kw_mode < 4) {
										// no more knocks after this.
										ctr_knocks = 0;
									}
									
									if (!flg_kw_pressedpower && sttg_ka_mode && flg_screen_on) {
										// knockactive.
										
										ssp_dbg("[SSP/kw] KNOCK 3 - gyro y: %ld ------------------------ !!! THREE 3 - PRESSING %d -----------\n",
												lTemp[1], sttg_ka_knock3_keycode);
										
										press_button(sttg_ka_knock3_keycode, sttg_ka_knock3_keydelay, true, false);
										
									} else if (!flg_screen_on || flg_kw_pressedpower) {
										// knockwake.
										
										if (sttg_kw_mode == 2) {
											// only wake from 2 knocks, do nothing more.
											
											// we shouldn't be here.
											
										} else if (sttg_kw_mode == 1 || sttg_kw_mode == 5) {
											// wake from 2 knocks, then action for 3rd or 4th.
											
											flg_ctr_cpuboost = 25;
											
											ssp_dbg("[SSP/kw] KNOCK 3 - gyro y: %ld ------------------------ !!! THREE 3 - PRESSING %d -----------\n",
													lTemp[1], sttg_kw_knock3_keycode);
											
											press_button(sttg_kw_knock3_keycode, sttg_kw_knock3_keydelay, true, false);
											
										} else if (sttg_kw_mode == 3) {
											// only wake from 3 knocks, so wake.
											
											flg_ctr_cpuboost = 25;
											ctr_knocks = 0;
											flg_kw_pressedpower = true;
											press_button(KEY_POWER, false, true, false);
										
										} else if (sttg_kw_mode == 4) {
											// wake from 3 knocks, then action for 4th knock.
											
											flg_ctr_cpuboost = 25;
											flg_kw_pressedpower = true;
											press_button(KEY_POWER, false, true, false);
											
										}
									}
									
								} else if (sttg_kw_strict_validation) {
									// this knock failed its validation.
									
									kw_status_code = 67;
									ctr_knocks = 0;
									ctr_kw_gyro_skip = 10;
								}
								
							} else if (ctr_knocks == 1 && time_since_knock1 <= sttg_kw_knock2_window)  {
								
								ssp_dbg("[SSP/kw] KNOCK 2? - gyro y: %ld --- ctr_knocks was: %d ----\n", lTemp[1], ctr_knocks);
							
								if ((sttg_ka_mode && flg_screen_on && time_since_knock1 >= sttg_ka_knock2_min_speed && time_since_knock1 <= sttg_ka_knock2_window)
									||
									(!flg_screen_on && time_since_knock1 >= sttg_kw_knock_min_speed && time_since_knock1 <= sttg_kw_knock2_window)) {
									// our second knock.
									// save the time.
									
									do_gettimeofday(&time_kw_knock2);
									ctr_knocks++;
									kw_status_code = 12;
									
									/////////////////////////////////////////////////
									// KNOCK #2 (TWO)
									/////////////////////////////////////////////////
									
									ssp_dbg("[SSP/kw] KNOCK 2 - gyro y: %ld ------------------------ !!! TWO 2 -----------\n", lTemp[1]);
									
									if (sttg_ka_mode && flg_screen_on) {
										// knockactive.
										
										ssp_dbg("[SSP/kw] KNOCK 2 - gyro y: %ld ------------------------ !!! TWO 2 knockACTIVE mode-----------\n", lTemp[1]);
										
										// will there be a third knock? does knock2 even have anything to press?
										if (sttg_ka_knock3_keycode && sttg_ka_knock2_keycode) {
											// there might be a 3rd knock.
											// this means we can't do anything until
											// the 3rd knock times out.
											
											ssp_dbg("[SSP/kw] KNOCK 2 - gyro y: %ld ------------------------ !!! TWO 2 scheduling work -----------\n", lTemp[1]);
											
											// schedule work.
											schedule_delayed_work(&work_deferred_knock2, msecs_to_jiffies(sttg_ka_knock3_window));
											
										} else if (sttg_ka_knock2_keycode) {
											// there won't be a 3rd knock, do work now.
											
											press_button(sttg_ka_knock2_keycode, sttg_ka_knock2_keydelay, true, false);
											
											// knockactive isn't using knock3, reset the knock counter.
											ctr_knocks = 0;
										}
										
									} else if (!flg_screen_on) {
										// knockwake.
										
										kw_status_code = 11;
										ssp_dbg("[SSP/kw] KNOCK 2 - gyro y: %ld ------------------------ !!! TWO 2 knockWAKE mode-----------\n", lTemp[1]);
									
										if (sttg_kw_mode == 2) {
											// only wake from 2 knocks, do nothing more.
											
											flg_ctr_cpuboost = 25;
											ctr_knocks = 0;
											flg_kw_pressedpower = true;
											press_button(KEY_POWER, false, true, false);
											
										} else if (sttg_kw_mode == 1 || sttg_kw_mode == 5) {
											// wake from 2 knocks, then action for 3rd or 4th knock.
											
											flg_ctr_cpuboost = 25;
											flg_kw_pressedpower = true;
											press_button(KEY_POWER, false, true, false);
											
										} else if (sttg_kw_mode == 3) {
											// modes 3 and 4 always need at least 2 knocks,
											// so do nothing here. no need to check for 4.
											
										}
									}
									
								} else if (sttg_kw_strict_validation) {
									// this knock failed its validation.
									
									kw_status_code = 67;
									ctr_knocks = 0;
									ctr_kw_gyro_skip = 10;
								}
								
							} else {
								// our first knock.
								// save the time.
								
								kw_status_code = 11;
								do_gettimeofday(&time_kw_knock1);
								ctr_knocks = 1;
								
								ssp_dbg("[SSP/kw] KNOCK 1 - gyro y: %ld ------------------------ !!! ONE 1 -----------\n", lTemp[1]);
							}
						}
						
					} else if (sttg_kw_strict_validation && (lTemp[1] > sttg_kw_gyro_y_max_threshold || lTemp[1] < sttg_kw_gyro_y_min_threshold)) {
						// this recheck has failed.
						// give up on this entire event.
						
						ssp_dbg("[SSP/kw] exceeded y min or max. strict mode set, discarding entire event. gyro y: %ld ----\n", lTemp[1]);
						ctr_kw_gyro_recheck = 0;
						ctr_knocks = 0;
						
					} else if (lTemp[1] <= sttg_kw_gyro_y_max_threshold && lTemp[1] >= sttg_kw_gyro_y_min_threshold) {
						// this recheck has succeeded.
						// this exists for debugging purposes.
						
						ssp_dbg("[SSP/kw] RECHECK #%d PASSED! gyro y: %ld ----\n", (ctr_kw_gyro_recheck + 1), lTemp[1]);
						
					} else {
						kw_status_code = 64;
						ctr_kw_gyro_skip = 25;
						ssp_dbg("[SSP/kw] RECHECK #%d FAILED! gyro y: %ld ----\n", (ctr_kw_gyro_recheck + 1), lTemp[1]);
					}
					
				} else {
					
					tmp_avg = tmp_sum / sttg_kw_gyro_y_history;
					
					if (tmp_avg <= sttg_kw_gyro_y_max_average
						&& tmp_avg >= sttg_kw_gyro_y_min_average
						&& abs(lTemp[1] - tmp_avg) >= sttg_kw_gyro_y_trigger_min_threshold
						&& abs(lTemp[1] - tmp_avg) <= sttg_kw_gyro_y_trigger_max_threshold) {
						
						ssp_dbg("[SSP/kw] EARLYKNOCK      gyro y: %ld, z: %ld, avg: %d -----------\n", lTemp[1], lTemp[2], tmp_avg);
						
						if (kw_status_code > 20 && kw_status_code < 10)
							kw_status_code = 65;
						ctr_kw_gyro_skip = sttg_kw_init_detection_suspensions;
						ctr_kw_gyro_recheck = sttg_kw_post_suspension_validations;
						
					} else if (tmp_avg > sttg_kw_gyro_y_max_average || tmp_avg < sttg_kw_gyro_y_min_average) {
						
						//ctr_kw_gyro_skip = 25;
						ssp_dbg("[SSP/kw] EARLYFAIL AVG   gyro y: %ld, z: %ld, avg: %d -----------\n", lTemp[1], lTemp[2], tmp_avg);
												
					}// else if (abs(lTemp[1] - tmp_avg) < sttg_kw_gyro_y_trigger_min_threshold || abs(lTemp[1] - tmp_avg) > sttg_kw_gyro_y_trigger_max_threshold) {
						
					//	ssp_dbg("[SSP/kw] EARLYFAIL Y     gyro y: %ld, z: %ld, avg: %d -----------\n", lTemp[1], lTemp[2], tmp_avg);
					//}
				}
				
			} else {
				ctr_kw_gyro_recheck = 0;
			}
			
			if (ctr_kw_gyro_skip < 1) {
				// TODO: it wouldn't be good if someone set sttg_kw_init_detection_suspensions to 0,
				// because then we'd trigger this IF which would add the trigger event to the history.
				
				if (ctr_kw_gyro_y_history >= sttg_kw_gyro_y_history) {
					ctr_kw_gyro_y_history = 0;
				}
				
				ary_kw_gyro_y_history[ctr_kw_gyro_y_history] = lTemp[1];
				
				ctr_kw_gyro_y_history++;
			}
		}
	} else {
		kw_status_code = 999;
	}
	
	if (sttg_kw_debug)
		pr_info("[SSP/kw/gyro] %ld,%ld,%ld\n", lTemp[0], lTemp[1], lTemp[2]);

	input_report_rel(data->gyro_input_dev, REL_RX, lTemp[0]);
	input_report_rel(data->gyro_input_dev, REL_RY, lTemp[1]);
	input_report_rel(data->gyro_input_dev, REL_RZ, lTemp[2]);
	input_sync(data->gyro_input_dev);
}

void ignoreKnocks(unsigned int suspensions)
{
	
	/*if (sttg_ka_mode && (ctr_knocks == 1 || ctr_knocks == 2)) {
		ssp_dbg("[SSP/kw] ignoreKnocks ignored. knocks: %d ----\n", ctr_knocks);
		return;
	}*/
	
	ctr_kw_gyro_recheck = 0;
	
	if (sttg_kw_mode != 1 || sttg_ka_mode) {
		// we can't clear the knock counter in mode 1, because
		// that mode relies on ctr_knock = 2 to hold open the
		// entire detection algorithm, otherwise it disables itself
		// upon screen-on. this is irrelevant if knockactive is enabled.
		
		ctr_knocks = 0;
	}
	
	kw_status_code = 66;
	ctr_kw_gyro_skip = suspensions;
	
	if (sttg_kw_debug)
		ssp_dbg("[SSP/kw] ignoreKnocks called. suspensions: %d ----\n", suspensions);
	
}
EXPORT_SYMBOL(ignoreKnocks);

void report_mag_data(struct ssp_data *data, struct sensor_value *magdata)
{
	data->buf[GEOMAGNETIC_SENSOR].x = magdata->x;
	data->buf[GEOMAGNETIC_SENSOR].y = magdata->y;
	data->buf[GEOMAGNETIC_SENSOR].z = magdata->z;
	
	mag_z = data->buf[GEOMAGNETIC_SENSOR].z;
	
	//ssp_dbg("[SSP] mag x: %i, y: %i, z: %i\n",
	//		data->buf[GEOMAGNETIC_SENSOR].x,
	//		data->buf[GEOMAGNETIC_SENSOR].y,
	//		data->buf[GEOMAGNETIC_SENSOR].z);
	
}

void report_gesture_data(struct ssp_data *data, struct sensor_value *gesdata)
{
	/* todo: remove func*/
}

void report_pressure_data(struct ssp_data *data, struct sensor_value *predata)
{
	data->buf[PRESSURE_SENSOR].pressure[0] =
		predata->pressure[0] - data->iPressureCal;
	data->buf[PRESSURE_SENSOR].pressure[1] = predata->pressure[1];

	/* pressure */
	input_report_rel(data->pressure_input_dev, REL_HWHEEL,
		data->buf[PRESSURE_SENSOR].pressure[0]);
	/* temperature */
	input_report_rel(data->pressure_input_dev, REL_WHEEL,
		data->buf[PRESSURE_SENSOR].pressure[1]);
	input_sync(data->pressure_input_dev);
}

void report_light_data(struct ssp_data *data, struct sensor_value *lightdata)
{
	data->buf[LIGHT_SENSOR].r = lightdata->r;
	data->buf[LIGHT_SENSOR].g = lightdata->g;
	data->buf[LIGHT_SENSOR].b = lightdata->b;
	data->buf[LIGHT_SENSOR].w = lightdata->w;

	input_report_rel(data->light_input_dev, REL_HWHEEL,
		data->buf[LIGHT_SENSOR].r + 1);
	input_report_rel(data->light_input_dev, REL_DIAL,
		data->buf[LIGHT_SENSOR].g + 1);
	input_report_rel(data->light_input_dev, REL_WHEEL,
		data->buf[LIGHT_SENSOR].b + 1);
	input_report_rel(data->light_input_dev, REL_MISC,
		data->buf[LIGHT_SENSOR].w + 1);
	input_sync(data->light_input_dev);
}

void report_prox_data(struct ssp_data *data, struct sensor_value *proxdata)
{
	ssp_dbg("[SSP] Proximity Sensor Detect : %u, raw : %u\n",
		proxdata->prox[0], proxdata->prox[1]);

#ifdef CONFIG_TOUCH_WAKE
	if (proxdata->prox[0]) { // true if proximity detected
		proximity_detected();
	} else {
    		proximity_off();
  	}
#endif

	data->buf[PROXIMITY_SENSOR].prox[0] = proxdata->prox[0];
	data->buf[PROXIMITY_SENSOR].prox[1] = proxdata->prox[1];

	input_report_abs(data->prox_input_dev, ABS_DISTANCE,
		(!proxdata->prox[0]));
	input_sync(data->prox_input_dev);

	wake_lock_timeout(&data->ssp_wake_lock, 3 * HZ);
}

void report_prox_raw_data(struct ssp_data *data,
	struct sensor_value *proxrawdata)
{
	if (data->uFactoryProxAvg[0]++ >= PROX_AVG_READ_NUM) {
		data->uFactoryProxAvg[2] /= PROX_AVG_READ_NUM;
		data->buf[PROXIMITY_RAW].prox[1] = (u8)data->uFactoryProxAvg[1];
		data->buf[PROXIMITY_RAW].prox[2] = (u8)data->uFactoryProxAvg[2];
		data->buf[PROXIMITY_RAW].prox[3] = (u8)data->uFactoryProxAvg[3];

		data->uFactoryProxAvg[0] = 0;
		data->uFactoryProxAvg[1] = 0;
		data->uFactoryProxAvg[2] = 0;
		data->uFactoryProxAvg[3] = 0;
	} else {
		data->uFactoryProxAvg[2] += proxrawdata->prox[0];

		if (data->uFactoryProxAvg[0] == 1)
			data->uFactoryProxAvg[1] = proxrawdata->prox[0];
		else if (proxrawdata->prox[0] < data->uFactoryProxAvg[1])
			data->uFactoryProxAvg[1] = proxrawdata->prox[0];

		if (proxrawdata->prox[0] > data->uFactoryProxAvg[3])
			data->uFactoryProxAvg[3] = proxrawdata->prox[0];
	}

	data->buf[PROXIMITY_RAW].prox[0] = proxrawdata->prox[0];
}

int initialize_event_symlink(struct ssp_data *data)
{
	int iRet = 0;
	struct class *event_class = NULL;

	event_class = class_create(THIS_MODULE, "sensor_event");
	data->sen_dev = device_create(event_class, NULL, 0, NULL,
		"%s", "symlink");

	iRet = sysfs_create_link(&data->sen_dev->kobj,
		&data->acc_input_dev->dev.kobj,
		data->acc_input_dev->name);
	if (iRet < 0)
		goto iRet_acc_sysfs_create_link;

	iRet = sysfs_create_link(&data->sen_dev->kobj,
		&data->gyro_input_dev->dev.kobj,
		data->gyro_input_dev->name);
	if (iRet < 0)
		goto iRet_gyro_sysfs_create_link;

	iRet = sysfs_create_link(&data->sen_dev->kobj,
		&data->pressure_input_dev->dev.kobj,
		data->pressure_input_dev->name);
	if (iRet < 0)
		goto iRet_prs_sysfs_create_link;

	iRet = sysfs_create_link(&data->sen_dev->kobj,
		&data->light_input_dev->dev.kobj,
		data->light_input_dev->name);
	if (iRet < 0)
		goto iRet_light_sysfs_create_link;

	iRet = sysfs_create_link(&data->sen_dev->kobj,
		&data->prox_input_dev->dev.kobj,
		data->prox_input_dev->name);
	if (iRet < 0)
		goto iRet_prox_sysfs_create_link;

	return SUCCESS;

iRet_prox_sysfs_create_link:
	sysfs_delete_link(&data->sen_dev->kobj,
		&data->light_input_dev->dev.kobj,
		data->light_input_dev->name);
iRet_light_sysfs_create_link:
	sysfs_delete_link(&data->sen_dev->kobj,
		&data->pressure_input_dev->dev.kobj,
		data->pressure_input_dev->name);
iRet_prs_sysfs_create_link:
	sysfs_delete_link(&data->sen_dev->kobj,
		&data->gyro_input_dev->dev.kobj,
		data->gyro_input_dev->name);
iRet_gyro_sysfs_create_link:
	sysfs_delete_link(&data->sen_dev->kobj,
		&data->acc_input_dev->dev.kobj,
		data->acc_input_dev->name);
iRet_acc_sysfs_create_link:
	pr_err("[SSP]: %s - could not create event symlink\n", __func__);

	return FAIL;
}

void remove_event_symlink(struct ssp_data *data)
{
	sysfs_delete_link(&data->sen_dev->kobj,
		&data->acc_input_dev->dev.kobj,
		data->acc_input_dev->name);
	sysfs_delete_link(&data->sen_dev->kobj,
		&data->gyro_input_dev->dev.kobj,
		data->gyro_input_dev->name);
	sysfs_delete_link(&data->sen_dev->kobj,
		&data->pressure_input_dev->dev.kobj,
		data->pressure_input_dev->name);
	sysfs_delete_link(&data->sen_dev->kobj,
		&data->light_input_dev->dev.kobj,
		data->light_input_dev->name);
	sysfs_delete_link(&data->sen_dev->kobj,
		&data->prox_input_dev->dev.kobj,
		data->prox_input_dev->name);
}

int initialize_input_dev(struct ssp_data *data)
{
	int iRet = 0;
	struct input_dev *acc_input_dev, *gyro_input_dev, *pressure_input_dev,
		*light_input_dev, *prox_input_dev;

	/* allocate input_device */
	acc_input_dev = input_allocate_device();
	if (acc_input_dev == NULL)
		goto iRet_acc_input_free_device;

	gyro_input_dev = input_allocate_device();
	if (gyro_input_dev == NULL)
		goto iRet_gyro_input_free_device;

	pressure_input_dev = input_allocate_device();
	if (pressure_input_dev == NULL)
		goto iRet_pressure_input_free_device;

	light_input_dev = input_allocate_device();
	if (light_input_dev == NULL)
		goto iRet_light_input_free_device;

	prox_input_dev = input_allocate_device();
	if (prox_input_dev == NULL)
		goto iRet_proximity_input_free_device;

	input_set_drvdata(acc_input_dev, data);
	input_set_drvdata(gyro_input_dev, data);
	input_set_drvdata(pressure_input_dev, data);
	input_set_drvdata(light_input_dev, data);
	input_set_drvdata(prox_input_dev, data);

	acc_input_dev->name = "accelerometer_sensor";
	gyro_input_dev->name = "gyro_sensor";
	pressure_input_dev->name = "pressure_sensor";
	light_input_dev->name = "light_sensor";
	prox_input_dev->name = "proximity_sensor";

	input_set_capability(acc_input_dev, EV_REL, REL_X);
	input_set_capability(acc_input_dev, EV_REL, REL_Y);
	input_set_capability(acc_input_dev, EV_REL, REL_Z);

	input_set_capability(gyro_input_dev, EV_REL, REL_RX);
	input_set_capability(gyro_input_dev, EV_REL, REL_RY);
	input_set_capability(gyro_input_dev, EV_REL, REL_RZ);

	input_set_capability(pressure_input_dev, EV_REL, REL_HWHEEL);
	input_set_capability(pressure_input_dev, EV_REL, REL_DIAL);
	input_set_capability(pressure_input_dev, EV_REL, REL_WHEEL);

	input_set_capability(light_input_dev, EV_REL, REL_HWHEEL);
	input_set_capability(light_input_dev, EV_REL, REL_DIAL);
	input_set_capability(light_input_dev, EV_REL, REL_WHEEL);
	input_set_capability(light_input_dev, EV_REL, REL_MISC);

	input_set_capability(prox_input_dev, EV_ABS, ABS_DISTANCE);
	input_set_abs_params(prox_input_dev, ABS_DISTANCE, 0, 1, 0, 0);

	/* register input_device */
	iRet = input_register_device(acc_input_dev);
	if (iRet < 0)
		goto iRet_acc_input_unreg_device;

	iRet = input_register_device(gyro_input_dev);
	if (iRet < 0) {
		input_free_device(gyro_input_dev);
		goto iRet_gyro_input_unreg_device;
	}

	iRet = input_register_device(pressure_input_dev);
	if (iRet < 0) {
		input_free_device(pressure_input_dev);
		goto iRet_pressure_input_unreg_device;
	}

	iRet = input_register_device(light_input_dev);
	if (iRet < 0) {
		input_free_device(light_input_dev);
		goto iRet_light_input_unreg_device;
	}

	iRet = input_register_device(prox_input_dev);
	if (iRet < 0) {
		input_free_device(prox_input_dev);
		goto iRet_proximity_input_unreg_device;
	}

	data->acc_input_dev = acc_input_dev;
	data->gyro_input_dev = gyro_input_dev;
	data->pressure_input_dev = pressure_input_dev;
	data->light_input_dev = light_input_dev;
	data->prox_input_dev = prox_input_dev;

	return SUCCESS;

iRet_proximity_input_unreg_device:
	input_unregister_device(light_input_dev);
iRet_light_input_unreg_device:
	input_unregister_device(pressure_input_dev);
iRet_pressure_input_unreg_device:
	input_unregister_device(gyro_input_dev);
iRet_gyro_input_unreg_device:
	input_unregister_device(acc_input_dev);
	return ERROR;

iRet_acc_input_unreg_device:
	pr_err("[SSP]: %s - could not register input device\n", __func__);
	input_free_device(prox_input_dev);
iRet_proximity_input_free_device:
	input_free_device(light_input_dev);
iRet_light_input_free_device:
	input_free_device(pressure_input_dev);
iRet_pressure_input_free_device:
	input_free_device(gyro_input_dev);
iRet_gyro_input_free_device:
	input_free_device(acc_input_dev);
iRet_acc_input_free_device:
	pr_err("[SSP]: %s - could not allocate input device\n", __func__);
	return ERROR;
}

void remove_input_dev(struct ssp_data *data)
{
	input_unregister_device(data->acc_input_dev);
	input_unregister_device(data->gyro_input_dev);
	input_unregister_device(data->pressure_input_dev);
	input_unregister_device(data->light_input_dev);
	input_unregister_device(data->prox_input_dev);
}
