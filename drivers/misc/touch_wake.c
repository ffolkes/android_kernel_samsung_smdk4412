/* drivers/misc/touch_wake.c
 *
 * Copyright 2011  Ezekeel
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * --------------------------------------------------------------------------------------
 *
 * Fixed issues with random misbehaving when powering off device via Powerkey
 *
 * Bumped version to 1.1
 *
 *                                         Jean-Pierre Rasquin <yank555.lu@gmail.com>
 */

#include <linux/init.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/touch_wake.h>
#include <linux/workqueue.h>
#include <linux/earlysuspend.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <linux/input.h>

#define HSL2RGB_H

void RGBtoHSL(void);

extern void touchscreen_enable(void);
extern void touchscreen_disable(void);
extern unsigned int flg_ctr_cpuboost;
extern unsigned int flg_ctr_cpuboost_mid;
extern struct wake_lock wavewake_wake_lock;
static struct wake_lock knockwake_wake_lock;
extern unsigned int ctr_knocks;
extern int ctr_kw_gyro_skip;
extern void toggleRearLED(unsigned int level);
extern void controlRearLED(unsigned int level);
extern struct timeval time_ledtriggered;
extern void ssp_manual_suspend(void);
extern void ssp_manual_resume(void);
extern void mdnie_toggle_blackout(void);

static struct timer_list timer_backblink;
bool flg_bb_active = false;
bool flg_bb_testmode = false;
bool flg_bb_magcheck = false;
bool flg_bb_magpassed = false;
unsigned int flg_ctr_bb_magchecks = 0;
static int ctr_bb_rearblink = 0;
static int ctr_bb_rearblinkon = 0;
static int ctr_bb_rearblinkoff = 0;
static int ctr_bb_blinkgroup_on = 0;
static int ctr_bb_blinkgroup_off = 0;
unsigned int bb_color_r = 0;
unsigned int bb_color_g = 0;
unsigned int bb_color_b = 0;

unsigned int sttg_bb_mode = 0; // 0 = off, 1 = on
bool sttg_bb_magcheck = true;
int sttg_bb_magcheck_z_min = 2350;
int sttg_bb_magcheck_z_max = 2600;
unsigned int sttg_bb_dutycycle_on = 50; // duration in ms rear led is on
unsigned int sttg_bb_dutycycle_off = 750; // duration in ms rear led is off
unsigned int sttg_bb_limit = 10; // how many blinks
bool sttg_bb_poweron_clearedobstacle = false; // turn power on when device picked up
unsigned int sttg_bb_brightness = 1; // brightness step 1 to 15
unsigned int sttg_bb_blinkgroup_on_size = 0; // how many flashes-on in each group
unsigned int sttg_bb_blinkgroup_on_delay = 500; // how long between each group
unsigned int sttg_bb_blinkgroup_off_size = 2; // how many flashes-on in each group
unsigned int sttg_bb_blinkgroup_off_delay = 2000; // how long between each group

unsigned int use_sttg_bb_dutycycle_on = 0;
unsigned int use_sttg_bb_dutycycle_off = 0;
unsigned int use_sttg_bb_blinkgroup_on_size = 0;
unsigned int use_sttg_bb_blinkgroup_on_delay = 0;
unsigned int use_sttg_bb_blinkgroup_off_size = 0;
unsigned int use_sttg_bb_blinkgroup_off_delay = 0;

bool flg_kw_gyro_on = false;
bool flg_tw_prox_on = false;
bool flg_bb_mag_on = false;

bool touchwake_enabled = false;
static bool touch_disabled = false;
static bool device_suspended = false;
bool timed_out = true;
bool prox_near = false;
bool ignore_once = false;
bool flg_touchwake_active = false;
static bool flg_touchwake_allowforceunlock = false;
unsigned int touchoff_delay = 10000;
bool sttg_tw_usewavewake = false;
bool sttg_touchwake_swipe_only = false;
bool flg_touchwake_pressed = false;
bool flg_touchwake_swipe_only = false;
bool sttg_touchwake_persistent = false;
bool sttg_touchwake_persistent_wakelock = false;
bool sttg_touchwake_ignoretkeys = true;
bool sttg_touchwake_ignoregestures = false;
bool sttg_touchwake_ignorepowerkey = false;
bool sttg_keys_ignorehomekeywake = false;
bool sttg_touchwake_force_timedout_tapwake = true;
unsigned int sttg_touchwake_delay_afterpoweroff = 0;
unsigned int sttg_touchwake_swipe_y_drift_tolerance = 60;
unsigned int sttg_touchwake_swipe_max_pressure = 35;
unsigned int sttg_touchwake_swipe_finger_angle_tolerance = 30;
bool sttg_touchwake_swipe_fast_trigger = true;
bool sttg_touchwake_swipe_arc_rh = true;
bool sttg_touchwake_swipe_arc_lh = true;
bool sttg_touchwake_longpressoff_enabled = false;
unsigned int sttg_touchwake_longpressoff_timeout = 30000;
unsigned int sttg_touchwake_longpressoff_duration = 75;
unsigned int sttg_touchwake_longpressoff_min_pressure = 22;
unsigned int sttg_touchwake_longpressoff_xy_drift_tolerance = 45;
unsigned int wpmk_min_pressure_fuzzy = 18; // default 15% of min_pressure, will be updated on sysfs change
unsigned int sttg_ww_mode = 0;
unsigned int sttg_ww_linger = 5000; // prox will stay on for 5 seconds after led goes on
unsigned int sttg_ww_waveoff_linger = 10000; // prox wills tay on for 10 seconds after screen comes on
bool sttg_ww_waveoff = false;
unsigned int sttg_ww_dismissled = 0;
bool sttg_ww_noredundancies = true;
bool sttg_ww_lightflow = false;
bool sttg_ww_trigger_noti_only = false;
bool flg_ww_userspace_noti = false;

unsigned int sttg_kw_mode = 0; // disabled.
int64_t sttg_kw_resolution = 60000000;
unsigned int sttg_kw_gyro_y_trigger_min_threshold = 40;
unsigned int sttg_kw_gyro_y_trigger_max_threshold = 3000;
unsigned int sttg_kw_gyro_y_history = 10;
unsigned int sttg_kw_knock2_window = 1500; // ms.
unsigned int sttg_kw_knock3_window = 3000;
unsigned int sttg_kw_knock3_keycode = KEY_POWER;
bool sttg_kw_knock3_keydelay = false;
unsigned int sttg_kw_knock4_window = 7000;
unsigned int sttg_kw_knock4_keycode = KEY_POWER;
bool sttg_kw_knock4_keydelay = false;
unsigned int sttg_kw_gyro_y_min_threshold = 35;
unsigned int sttg_kw_gyro_y_max_threshold = 80;
unsigned int sttg_kw_gyro_y_min_average = 30;
unsigned int sttg_kw_gyro_y_max_average = 80;
int sttg_kw_gyro_x_min_threshold = -260; // don't forget, these are reversed because they're negative.
int sttg_kw_gyro_x_max_threshold = 260;
int sttg_kw_gyro_z_min_threshold = -200;
int sttg_kw_gyro_z_max_threshold = 200;
unsigned int sttg_kw_z_instab_suspensions = 100;
unsigned int sttg_kw_init_detection_suspensions = 4;
unsigned int sttg_kw_post_suspension_validations = 2;
bool sttg_kw_strict_validation = true;
unsigned int sttg_kw_instab_tolerance = 2;
unsigned int sttg_kw_extreme_instab_tolerance = 10;
unsigned int sttg_kw_extreme_instab_suspensions = 20;
unsigned int sttg_kw_knock_min_speed = 700; // ms. keep divisible by 100 for synapse.
unsigned int kw_status_code = 999;
bool sttg_kw_debug = false;
unsigned int sttg_kw_tsp_event_threshold = 4; // this many tsp events to trigger block.
unsigned int sttg_kw_tsp_event_suspensions = 30; // how long to hold block.

unsigned int sttg_ka_mode = 0; // disabled.
unsigned int sttg_ka_knock2_window = 1500;
unsigned int sttg_ka_knock3_window = 1500;
unsigned int sttg_ka_knock2_keycode = 0;
bool sttg_ka_knock2_keydelay = false;
unsigned int sttg_ka_knock3_keycode = 0;
bool sttg_ka_knock3_keydelay = false;
unsigned int sttg_ka_knock2_min_speed = 700; // ms. keep divisible by 100 for synapse.
unsigned int sttg_ka_knock3_min_speed = 700;

//unsigned int sttg_kw_third_timeout = 3000;

struct timeval time_ww_last_screenon;
struct timeval time_ww_last_screenoff;
struct timeval time_ww_last_manualtrigger;

static void knockwake_off_work(struct work_struct * work_knockwake_off);
static DECLARE_DELAYED_WORK(work_knockwake_off, knockwake_off_work);

static void bb_magcheck_off_work(struct work_struct * work_bb_magcheck_off);
static DECLARE_DELAYED_WORK(work_bb_magcheck_off, bb_magcheck_off_work);

static void touchwake_reset_swipeonly(struct work_struct * touchwake_reset_swipeonly_work);
static DECLARE_DELAYED_WORK(touchwake_reset_swipeonly_work, touchwake_reset_swipeonly);
static void touchwake_touchoff(struct work_struct * touchoff_work);
static DECLARE_DELAYED_WORK(touchoff_work, touchwake_touchoff);
static void press_powerkey(struct work_struct * presspower_work);
static DECLARE_WORK(presspower_work, press_powerkey);
static DEFINE_MUTEX(lock);

static struct touchwake_implementation * touchwake_imp = NULL;
struct input_dev * powerkey_device;
struct ssp_data * prox_device;
static struct wake_lock touchwake_wake_lock;
static struct timeval last_powerkeypress;

int ctr_ww_prox_hits = 1;

#define TOUCHWAKE_VERSION "1.1"
#define TIME_LONGPRESS 500
#define POWERPRESS_DELAY 0
#define POWERPRESS_TIMEOUT 0

#define DEBUG_PRINT

void bb_set_blinkpattern_manual(void)
{
	use_sttg_bb_dutycycle_on = sttg_bb_dutycycle_on;
	use_sttg_bb_dutycycle_off = sttg_bb_dutycycle_off;
	use_sttg_bb_blinkgroup_on_size = sttg_bb_blinkgroup_on_size;
	use_sttg_bb_blinkgroup_on_delay = sttg_bb_blinkgroup_on_delay;
	use_sttg_bb_blinkgroup_off_size = sttg_bb_blinkgroup_off_size;
	use_sttg_bb_blinkgroup_off_delay = sttg_bb_blinkgroup_off_delay;
	
	pr_info("[TW/bb_set_blinkpattern_manual] reset to user-defined defaults\n");
}

void bb_set_blinkpattern_autorgb(unsigned int r, unsigned int g, unsigned int b)
{
	unsigned int r_percent = (r / 255);
	unsigned int g_percent = (g / 255);
	unsigned int b_percent = (b / 255);
	unsigned int max_color = 0;
	unsigned int min_color = 0;
	unsigned int h = 0;
	
	if (!r && !g && !b) {
		// found 0, 0, 0.
		// reset to user-defined values and return.
		
		bb_set_blinkpattern_manual();
		return;
	}
	
	if (r == g && r == b) {
		// neutral color.
		
		use_sttg_bb_dutycycle_on = 20;
		use_sttg_bb_dutycycle_off = 100;
		use_sttg_bb_blinkgroup_on_size = 0;
		use_sttg_bb_blinkgroup_on_delay = 0;
		use_sttg_bb_blinkgroup_off_size = 0;
		use_sttg_bb_blinkgroup_off_delay = 0;
		pr_info("[TW/bb_set_blinkpattern_autorgb] NEUTRAL\n");
		
		// we're done.
		return;
	}
	
	// calculate hue from RGB values.
	
	if ((r_percent >= g_percent) && (r_percent >= b_percent)) {
		max_color = r_percent;
	}
	
	if ((g_percent >= r_percent) && (g_percent >= b_percent)) {
		max_color = g_percent;
	}
	
	if ((b_percent >= r_percent) && (b_percent >= g_percent)) {
		max_color = b_percent;
	}
	
	if ((r_percent <= g_percent) && (r_percent <= b_percent)) {
		min_color = r_percent;
	}
	
	if ((g_percent <= r_percent) && (g_percent <= b_percent)) {
		min_color = g_percent;
	}
	
	if ((b_percent <= r_percent) && (b_percent <= g_percent)) {
		min_color = b_percent;
	}
	
	if (max_color == min_color) {
		
		h = 0;
		
	} else {
		
		if (max_color == r_percent) {
			h = (g_percent - b_percent) / (max_color - min_color);
		}
		
		if (max_color == g_percent) {
			h = 2 + (b_percent - r_percent) / (max_color - min_color);
		}
		
		if (max_color == b_percent) {
			h = 4 + (r_percent - g_percent) / (max_color - min_color);
		}
	}
	
	h = h * 60;
	
	if (h < 0) {
		h += 360;
	}
	
	pr_info("[TW/bb_set_blinkpattern_autorgb] h: %d\n", h);
	
	if (h <= 20 || h >= 330) {
		// red.
		
		use_sttg_bb_dutycycle_on = 20;
		use_sttg_bb_dutycycle_off = 100;
		use_sttg_bb_blinkgroup_on_size = 0;
		use_sttg_bb_blinkgroup_on_delay = 0;
		use_sttg_bb_blinkgroup_off_size = 0;
		use_sttg_bb_blinkgroup_off_delay = 0;
		pr_info("[TW/bb_set_blinkpattern_autorgb] RED\n");
		
	} else if (h <= 20 || h <= 50) {
		// orange.
		
		use_sttg_bb_dutycycle_on = 40;
		use_sttg_bb_dutycycle_off = 100;
		use_sttg_bb_blinkgroup_on_size = 0;
		use_sttg_bb_blinkgroup_on_delay = 0;
		use_sttg_bb_blinkgroup_off_size = 0;
		use_sttg_bb_blinkgroup_off_delay = 0;
		pr_info("[TW/bb_set_blinkpattern_autorgb] ORANGE\n");
		
	} else if (h <= 50 || h <= 67) {
		// yellow.
		
		use_sttg_bb_dutycycle_on = 60;
		use_sttg_bb_dutycycle_off = 100;
		use_sttg_bb_blinkgroup_on_size = 0;
		use_sttg_bb_blinkgroup_on_delay = 0;
		use_sttg_bb_blinkgroup_off_size = 0;
		use_sttg_bb_blinkgroup_off_delay = 0;
		pr_info("[TW/bb_set_blinkpattern_autorgb] YELLOW\n");
		
	} else if (h <= 65 || h <= 160) {
		// green.
		
		use_sttg_bb_dutycycle_on = 80;
		use_sttg_bb_dutycycle_off = 100;
		use_sttg_bb_blinkgroup_on_size = 0;
		use_sttg_bb_blinkgroup_on_delay = 0;
		use_sttg_bb_blinkgroup_off_size = 0;
		use_sttg_bb_blinkgroup_off_delay = 0;
		pr_info("[TW/bb_set_blinkpattern_autorgb] GREEN\n");
		
	} else if (h <= 160 || h <= 170) {
		// teal.
		
		use_sttg_bb_dutycycle_on = 100;
		use_sttg_bb_dutycycle_off = 100;
		use_sttg_bb_blinkgroup_on_size = 0;
		use_sttg_bb_blinkgroup_on_delay = 0;
		use_sttg_bb_blinkgroup_off_size = 0;
		use_sttg_bb_blinkgroup_off_delay = 0;
		pr_info("[TW/bb_set_blinkpattern_autorgb] TEAL\n");
		
	} else if (h <= 170 || h <= 190) {
		// cyan.
		
		use_sttg_bb_dutycycle_on = 120;
		use_sttg_bb_dutycycle_off = 100;
		use_sttg_bb_blinkgroup_on_size = 0;
		use_sttg_bb_blinkgroup_on_delay = 0;
		use_sttg_bb_blinkgroup_off_size = 0;
		use_sttg_bb_blinkgroup_off_delay = 0;
		pr_info("[TW/bb_set_blinkpattern_autorgb] CYAN\n");
		
	} else if (h <= 190 || h <= 260) {
		// blue.
		
		use_sttg_bb_dutycycle_on = 140;
		use_sttg_bb_dutycycle_off = 100;
		use_sttg_bb_blinkgroup_on_size = 0;
		use_sttg_bb_blinkgroup_on_delay = 0;
		use_sttg_bb_blinkgroup_off_size = 0;
		use_sttg_bb_blinkgroup_off_delay = 0;
		pr_info("[TW/bb_set_blinkpattern_autorgb] BLUE\n");
		
	} else if (h <= 260 || h <= 290) {
		// purple.
		
		use_sttg_bb_dutycycle_on = 160;
		use_sttg_bb_dutycycle_off = 100;
		use_sttg_bb_blinkgroup_on_size = 0;
		use_sttg_bb_blinkgroup_on_delay = 0;
		use_sttg_bb_blinkgroup_off_size = 0;
		use_sttg_bb_blinkgroup_off_delay = 0;
		pr_info("[TW/bb_set_blinkpattern_autorgb] PURPLE\n");
		
	} else if (h <= 290 || h <= 330) {
		// pink.
		
		use_sttg_bb_dutycycle_on = 180;
		use_sttg_bb_dutycycle_off = 100;
		use_sttg_bb_blinkgroup_on_size = 0;
		use_sttg_bb_blinkgroup_on_delay = 0;
		use_sttg_bb_blinkgroup_off_size = 0;
		use_sttg_bb_blinkgroup_off_delay = 0;
		pr_info("[TW/bb_set_blinkpattern_autorgb] PINK\n");
		
	} else {
		// unknown.
		
		bb_set_blinkpattern_manual();
		pr_info("[TW/bb_set_blinkpattern_autorgb] UNKNOWN - reset to user-defined values\n");
	}
}

static void press_power()
{
	input_event(powerkey_device, EV_KEY, KEY_POWER, 1);
    input_sync(powerkey_device);
    msleep(10);
    input_event(powerkey_device, EV_KEY, KEY_POWER, 0);
    input_sync(powerkey_device);
}

static void touchwake_disable_touch(void)
{
    
#ifdef DEBUG_PRINT
	pr_info("[TOUCHWAKE] Disable touch controls\n");
#endif

	touch_disabled = true;
	
	touchscreen_disable();

	if (touchwake_imp)
	{
	    touchwake_imp->disable();
	}

	return;
}

static void touchwake_enable_touch(void)
{
#ifdef DEBUG_PRINT
	pr_info("[TOUCHWAKE] Enable touch controls\n");
#endif

	touch_disabled = false;
    
    flg_touchwake_active = false;
	
	touchscreen_enable();

	if (touchwake_imp)
	{
	    touchwake_imp->enable();
	}
	
	return;
}

static void touchwake_early_suspend(struct early_suspend * h)
{
#ifdef DEBUG_PRINT
	pr_info("[TOUCHWAKE] Enter early suspend\n");
#endif
	
	flg_kw_pressedpower = false;
	
	ctr_knocks = 0;
	ctr_kw_gyro_skip = 0;
	
	leds_reset_last();
	
	// don't turn the gyro off if we've gone back to sleep before it timed out.
	cancel_delayed_work_sync(&work_knockwake_off);
	
	if (sttg_kw_mode) {
		pr_info("[TW/ssp/kw] stopping GYRO\n");
		forceDisableSensor(1);
		pr_info("[TW/ssp/kw] starting GYRO\n");
		pr_info("[SSP/kw] locking knockwake_wake_lock\n");
		wake_lock(&knockwake_wake_lock);
		forceEnableSensor(1, false);
	}
	
	//flg_ww_prox_on = false;
	ctr_ww_prox_hits = sttg_ww_mode;
	
	//wake_unlock(&wavewake_wake_lock);
	//pr_info("[TW/SSP/ww] screen-off: unlocked wavewake_wake_lock\n");
    
    // if touchwake isn't enabled, abort now.
    if (!touchwake_enabled) {
        flg_touchwake_active = false;
        pr_info("[TOUCHWAKE] Early suspend - disable touch immediately\n");
        touchwake_disable_touch();
        return;
    }

    device_suspended = true;
    flg_touchwake_pressed = false;
    flg_touchwake_active = true;
    flg_touchwake_swipe_only = sttg_touchwake_swipe_only;
	
	if (sttg_touchwake_persistent && !sttg_touchwake_force_timedout_tapwake) {
		// this is a dirty hack because i don't have time to rewrite tw.
		// if we aren't going to schedule work, always enforce the lock.
		// TODO: make this into scheduled work that will change this var
		// x ms after suspend regardless of any other settings.
		flg_touchwake_allowforceunlock = false;
		
	} else {
		
		flg_touchwake_allowforceunlock = true;
	}
    
	pr_info("[TOUCHWAKE] flg_touchwake_allowforceunlock: %d\n", flg_touchwake_allowforceunlock);
	
	// allow persistent mode only if pu is off, or on and s2w allowed.
	//if (sttg_touchwake_persistent && (!sttg_pu_mode || sttg_pu_allow_s2w)) {
    if (sttg_touchwake_persistent) {
        // if we are in persistent mode, don't go beyond this if-statement.
        
#ifdef DEBUG_PRINT
        pr_info("[TOUCHWAKE] Early suspend - slide2wake persistent mode. digitizer always on.\n");
#endif
        
        //if (sttg_touchwake_persistent_wakelock) {  /* TODO: disabled if-block until deepsleep wake works */
        //     start the persistent mode wakelock.
        //    pr_info("[Touchwake] wakelock requested.\n");
            wake_lock(&touchwake_wake_lock);
        //}
		
		flg_ctr_cpuboost_mid = 10;
        
        if (timed_out && sttg_touchwake_force_timedout_tapwake) {
            // the user might want to hold a persistent lock and use slide2wake, but still be able to
            // just single tap to wake up after a timeout event. so we have to schedule delayed work
            // to reset sttg_touchwake_swipe_only back to the real value.
            
            pr_info("[TOUCHWAKE] Early suspend - swipeonly and force_tap set. scheduling work...\n");
            
            //pr_info("[Touchwake] wakelock needed for tap check.\n");
            //wake_lock(&touchwake_wake_lock); /* TODO: disabled until deepsleep wake works */
			
			if (sttg_tw_usewavewake) {
				if (!sensorStatus(5)) {
					pr_info("[TW/ssp] immediately starting PROX\n");
					forceEnableSensor(5, false);
					flg_tw_prox_on = true;
				} else {
					// debug purposes.
					
					pr_info("[TW/ssp] PROX was already on! skipping.\n");
				}
			}
            
			flg_ctr_cpuboost_mid = 25;
            flg_touchwake_swipe_only = false;
            schedule_delayed_work(&touchwake_reset_swipeonly_work, msecs_to_jiffies(touchoff_delay));
            
        }
		
		if (sttg_touchwake_ignoretkeys) {
			// touchkeys are being ignored, so turn the hardware off.
			
			if (touchwake_imp) {
				pr_info("[TW/early_suspend] TKEYS - TURNING OFF\n");
				touchwake_imp->disable();
			}
		}
        
        // always on, so skip the rest.
        return;
    }

	if (!ignore_once) {
        if (timed_out && !prox_near && touchoff_delay > 0) {
            // if touchoff_delay is greater than zero, then hold a wakelock for that time.
            pr_info("[TOUCHWAKE] Early suspend - enable touch delay\n");
			
			if (sttg_tw_usewavewake) {
				if (!sensorStatus(5)) {
					pr_info("[TW/ssp] immediately starting PROX\n");
					forceEnableSensor(5, false);
					flg_tw_prox_on = true;
				} else {
					// debug purposes.
					
					pr_info("[TW/ssp] PROX was already on! skipping.\n");
				}
			}
			
			flg_ctr_cpuboost_mid = 50;
            flg_touchwake_active = true;
            wake_lock(&touchwake_wake_lock);
            schedule_delayed_work(&touchoff_work, msecs_to_jiffies(touchoff_delay));
        } else if (sttg_touchwake_delay_afterpoweroff > 0) {
            // this will hold a wakelock after any power off event.
            // if afterpoweroff delay is greater than zero, then hold a wakelock for that time.
            pr_info("[TOUCHWAKE] Early suspend - enable touch delay\n");
            flg_touchwake_active = true;
            wake_lock(&touchwake_wake_lock);
            schedule_delayed_work(&touchoff_work, msecs_to_jiffies(sttg_touchwake_delay_afterpoweroff));
        } else {
            flg_touchwake_active = false;
            pr_info("[TOUCHWAKE] Early suspend - disable touch immediately\n");
            touchwake_disable_touch();
        }
	} else {
		flg_touchwake_active = false;
#ifdef DEBUG_PRINT
		pr_info("[TOUCHWAKE] Early suspend - disable touch immediately (TouchWake disabled)\n");
#endif
		touchwake_disable_touch();
	}
	
	if (ignore_once) {
		// touchwake cannot differentiate a virtual power key press (via app,
		// script, etc) so it will wrongly activate, thinking the screen has
		// timed out. this setting lets a script tell touchwake via sysfs to
		// disregard the next early_suspend event immediately before the
		// script makes its call for the virtual power button.
		
		// we just used up our one ignore, so reset.
		ignore_once = false;
	}
    
	return;
}

static void touchwake_late_resume(struct early_suspend * h)
{
	unsigned int tmp_kw_workdelay = 0;
	
#ifdef DEBUG_PRINT
	pr_info("[TOUCHWAKE] Enter late resume\n");
#endif
	// important that device_suspended gets set as soon as possible.
	device_suspended = false;
    flg_touchwake_active = false;
    flg_touchwake_pressed = false;
	cancel_delayed_work_sync(&touchoff_work);
    cancel_delayed_work_sync(&touchwake_reset_swipeonly_work);
    flg_touchwake_swipe_only = sttg_touchwake_swipe_only;
	flg_ww_userspace_noti = false;
	flg_ctr_cpuboost_mid = 0;
	//flush_scheduled_work();
	
	touchwake_enable_touch();
	
	// turn off backblink, and led.
	del_timer(&timer_backblink);
	controlRearLED(0);
	
	if (flg_tw_prox_on) {
		pr_info("[TW/SSP] PROX - DISABLE - screen is back on\n");
		flg_tw_prox_on = false;
		forceDisableSensor(5);
	}
	
	if (!sttg_ww_waveoff && flg_ww_prox_on) {
		// if waveoff isn't set and the prox is on, then turn it all off.
		
		pr_info("[TW/ww] screen on, no waveoff, forcing PROX off now\n");
		ww_disable_prox();
		
	} else if (flg_ww_prox_on){
		// if wavewake is on and the prox is on, then turn it all off in a while.
		
		pr_info("[TW/ww] screen on, waveoff, PROX off in %d seconds\n", sttg_ww_waveoff_linger);
		ww_set_disable_prox(sttg_ww_waveoff_linger);
	}
	
	leds_reset_last();
	
	
	/*// stop the booster when the screen comes on.
	// this is just piggybacked in touch_wake.c for my convenience.
	flg_ctr_cpuboost = 0;*/
	
	if ((sttg_kw_mode == 1 || sttg_kw_mode > 3) && flg_kw_pressedpower) {
		// if we're expecting a more knocks, we can't turn the sensor off yet.
		
		if (sttg_kw_mode == 5) {
			// we need to delay the work long enough for knocks 3 and 4.
			
			tmp_kw_workdelay = sttg_kw_knock3_window + sttg_kw_knock4_window;
			
		} else if (sttg_kw_mode == 4) {
			// we need to delay the work long enough for knock 4.
			
			tmp_kw_workdelay = sttg_kw_knock4_window;
			
		} else {
			// we need to delay the work long enough for knock 3.
			
			tmp_kw_workdelay = sttg_kw_knock3_window;
		}
		
		pr_info("[TW/ssp/kw] blocking first press and scheduling work to turn off gyro in %d ms\n", tmp_kw_workdelay);
		
		// block the first touch.
		wpmk_block_touch = 1;
		
		schedule_delayed_work(&work_knockwake_off, msecs_to_jiffies(tmp_kw_workdelay));
		
	} else if (!sttg_kw_mode || (sttg_kw_mode && !sttg_ka_mode)) {
		
		pr_info("[TW/ssp/kw] immediately stopping GYRO\n");
		forceDisableSensor(1);
		
		pr_info("[SSP/kw] unlocking knockwake_wake_lock\n");
		wake_unlock(&knockwake_wake_lock);
		
	} else if (sttg_ka_mode) {
		pr_info("[TW/ssp/ka] stopping GYRO\n");
		forceDisableSensor(1);
		pr_info("[TW/ssp/ka] restarting GYRO because knockactive is active\n");
		forceEnableSensor(1, false);
	}

	wake_unlock(&touchwake_wake_lock);

	timed_out = true;

	return;
}

static struct early_suspend touchwake_suspend_data =
{
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
	.suspend = touchwake_early_suspend,
	.resume = touchwake_late_resume,
};

static void touchwake_touchoff(struct work_struct * touchoff_work)
{
    flg_touchwake_active = false;
	
	// now that tw has timed out, don't allow it to bypass the pu input lock.
	flg_touchwake_allowforceunlock = false;
	
	if (flg_tw_prox_on) {
		pr_info("[TW/SSP] PROX - DISABLE - touchwake has timed out\n");
		flg_tw_prox_on = false;
		forceDisableSensor(5);
	}
	
	touchwake_disable_touch();
	touchwake_droplock();

	return;
}

static void knockwake_off_work(struct work_struct * work_knockwake_off)
{
	
	wpmk_block_touch = 0;
	
	if (!sttg_ka_mode) {
		
		pr_info("[TW/ssp/kw] stopping GYRO\n");
		forceDisableSensor(1);
		
		pr_info("[SSP/kw] unlocking knockwake_wake_lock\n");
		wake_unlock(&knockwake_wake_lock);
		
	} else {
		
		pr_info("[TW/ssp/ka_work] not stopping GYRO because knockactive is active\n");
	}
	
	return;
}

static void touchwake_reset_swipeonly(struct work_struct * touchwake_reset_swipeonly_work)
{
	if (flg_tw_prox_on) {
		pr_info("[TW/SSP] PROX - DISABLE - touchwake has timed out (persistent)\n");
		flg_tw_prox_on = false;
		forceDisableSensor(5);
	}
	flg_touchwake_allowforceunlock = false;
    flg_touchwake_swipe_only = sttg_touchwake_swipe_only;
    pr_info("touchwake: reset_swipeonly set %d to %d\n", flg_touchwake_swipe_only, sttg_touchwake_swipe_only);
    //if (!sttg_touchwake_persistent_wakelock) { /* TODO: disabled IF-block until deepsleep wake works */
        // stop the persistent mode wakelock.
    //    pr_info("[Touchwake] wakelock released.\n");
    //    wake_unlock(&touchwake_wake_lock);
    //}
	return;
}

void touchwake_droplock(void)
{
    // disable the persistent wakelock setting and unlock the wakelock.
    flg_touchwake_active = false; /* TODO: remember to disable this when implementing deepsleep wake */
    sttg_touchwake_persistent = false; /* TODO: remember to rename this (sttg_touchwake_persistent_wakelock) when implementing deepsleep wake */
    wake_unlock(&touchwake_wake_lock);
    
	return;
}
EXPORT_SYMBOL(touchwake_droplock);

static void press_powerkey(struct work_struct * presspower_work)
{
	input_event(powerkey_device, EV_KEY, KEY_POWER, 1);
	input_event(powerkey_device, EV_SYN, 0, 0);
	msleep(POWERPRESS_DELAY);

	input_event(powerkey_device, EV_KEY, KEY_POWER, 0);
	input_event(powerkey_device, EV_SYN, 0, 0);
	msleep(POWERPRESS_DELAY);

	msleep(POWERPRESS_TIMEOUT);

	mutex_unlock(&lock);

	return;
}

static ssize_t touchwake_status_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", (touchwake_enabled ? 1 : 0));
}

static ssize_t touchwake_status_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;

	if(sscanf(buf, "%u\n", &data) == 1) {
		pr_devel("%s: %u \n", __FUNCTION__, data);

		if (data == 1) {
#ifdef DEBUG_PRINT
			pr_info("[TOUCHWAKE] %s: TOUCHWAKE function enabled\n", __FUNCTION__);
#endif
			touchwake_enabled = true;
		} else if (data == 0) {
#ifdef DEBUG_PRINT
			pr_info("[TOUCHWAKE] %s: TOUCHWAKE function disabled\n", __FUNCTION__);
#endif
			touchwake_enabled = false;
#ifdef DEBUG_PRINT
		} else {
			pr_info("[TOUCHWAKE] %s: invalid input range %u\n", __FUNCTION__, data);
#endif
		}
#ifdef DEBUG_PRINT
	} else 	{
		pr_info("[TOUCHWAKE] %s: invalid input\n", __FUNCTION__);
#endif
	}

	return size;
}

static ssize_t touchwake_delay_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", touchoff_delay);
}

static ssize_t touchwake_delay_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;

	if(sscanf(buf, "%u\n", &data) == 1) {
		touchoff_delay = data;
#ifdef DEBUG_PRINT
		pr_info("[TOUCHWAKE] Delay set to %u\n", touchoff_delay);
#endif
#ifdef DEBUG_PRINT
	} else 	{
		pr_info("[TOUCHWAKE] %s: invalid input\n", __FUNCTION__);
#endif
	}

	return size;
}

static ssize_t touchwake_ignore_once_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", ignore_once);
}

static ssize_t touchwake_ignore_once_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if(ret && data == 1) {
		ignore_once = true;
		pr_info("TOUCHWAKE ignore_once has been set\n");
	} else {
		ignore_once = false;
		pr_info("TOUCHWAKE ignore_once has been unset\n");
	}
	
	return size;
}

static ssize_t touchwake_slide2wake_only_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_touchwake_swipe_only);
}

static ssize_t touchwake_slide2wake_only_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if(ret && data == 1) {
		sttg_touchwake_swipe_only = true;
        flg_touchwake_swipe_only = true;
		pr_info("TOUCHWAKE slide2wake_only has been set\n");
	} else {
		sttg_touchwake_swipe_only = false;
        flg_touchwake_swipe_only = false;
		pr_info("TOUCHWAKE slide2wake_only has been unset\n");
	}
	
	return size;
}

static ssize_t touchwake_enable_persistent_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_touchwake_persistent);
}

static ssize_t touchwake_enable_persistent_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if(ret && data == 1) {
		if (!sttg_touchwake_persistent) {
			sttg_touchwake_persistent = true;
			flg_touchwake_active = true;
			flg_touchwake_swipe_only = sttg_touchwake_swipe_only;
			wake_lock(&touchwake_wake_lock);
			touchwake_enable_touch();
			pr_info("TOUCHWAKE enable_persistent has been set\n");
		} else {
			pr_info("TOUCHWAKE enable_persistent was already set\n");
		}
	} else {
		sttg_touchwake_persistent = false;
        wake_unlock(&touchwake_wake_lock);
		pr_info("TOUCHWAKE enable_persistent has been unset\n");
	}
	
	return size;
}

static ssize_t touchwake_enable_persistent_wakelock_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_touchwake_persistent_wakelock);
}

static ssize_t touchwake_enable_persistent_wakelock_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if(ret && data == 1) {
		sttg_touchwake_persistent_wakelock = true;
		pr_info("TOUCHWAKE enable_persistent_wakelock has been set\n");
	} else {
		sttg_touchwake_persistent_wakelock = false;
        wake_unlock(&touchwake_wake_lock);
		pr_info("TOUCHWAKE enable_persistent_wakelock has been unset\n");
	}
	
	return size;
}

static ssize_t touchwake_ignore_tkeys_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_touchwake_ignoretkeys);
}

static ssize_t touchwake_ignore_tkeys_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if(ret && data == 1) {
		sttg_touchwake_ignoretkeys = true;
		pr_info("TOUCHWAKE ignore_tkeys has been set\n");
	} else {
		sttg_touchwake_ignoretkeys = false;
		pr_info("TOUCHWAKE ignore_tkeys has been unset\n");
	}
	
	return size;
}

static ssize_t touchwake_force_timedout_tapwake_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_touchwake_force_timedout_tapwake);
}

static ssize_t touchwake_force_timedout_tapwake_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if(ret && data == 1) {
		sttg_touchwake_force_timedout_tapwake = true;
		pr_info("TOUCHWAKE force_timedout_tapwake has been set\n");
	} else {
		sttg_touchwake_force_timedout_tapwake = false;
		pr_info("TOUCHWAKE force_timedout_tapwake has been unset\n");
	}
	
	return size;
}

static ssize_t touchwake_ignore_gestures_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_touchwake_ignoregestures);
}

static ssize_t touchwake_ignore_gestures_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if(ret && data == 1) {
		sttg_touchwake_ignoregestures = true;
		pr_info("TOUCHWAKE ignore_gestures has been set\n");
	} else {
		sttg_touchwake_ignoregestures = false;
		pr_info("TOUCHWAKE ignore_gestures has been unset\n");
	}
	
	return size;
}

static ssize_t touchwake_ignore_powerkey_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_touchwake_ignorepowerkey);
}

static ssize_t touchwake_ignore_powerkey_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if(ret && data == 1) {
		sttg_touchwake_ignorepowerkey = true;
		pr_info("TOUCHWAKE ignore_powerkey has been set\n");
	} else {
		sttg_touchwake_ignorepowerkey = false;
		pr_info("TOUCHWAKE ignore_powerkey has been unset\n");
	}
	
	return size;
}

static ssize_t touchwake_ignore_homekeywake_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_keys_ignorehomekeywake);
}

static ssize_t touchwake_ignore_homekeywake_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if(ret && data == 1) {
		sttg_keys_ignorehomekeywake = true;
		pr_info("TOUCHWAKE ignore_homekeywake has been set\n");
	} else {
		sttg_keys_ignorehomekeywake = false;
		pr_info("TOUCHWAKE ignore_homekeywake has been unset\n");
	}
	
	return size;
}

static ssize_t touchwake_delay_afterpoweroff_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_touchwake_delay_afterpoweroff);
}

static ssize_t touchwake_delay_afterpoweroff_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_touchwake_delay_afterpoweroff = data;
		pr_info("TOUCHWAKE delay_afterpoweroff has been set to %d\n", sttg_touchwake_delay_afterpoweroff);
	}
	
	return size;
}

static ssize_t touchwake_swipe_y_drift_tolerance_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_touchwake_swipe_y_drift_tolerance);
}

static ssize_t touchwake_swipe_y_drift_tolerance_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_touchwake_swipe_y_drift_tolerance = data;
		pr_info("TOUCHWAKE swipe_y_drift_tolerance has been set to %d\n", sttg_touchwake_swipe_y_drift_tolerance);
	}
	
	return size;
}

static ssize_t touchwake_swipe_max_pressure_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_touchwake_swipe_max_pressure);
}

static ssize_t touchwake_swipe_max_pressure_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_touchwake_swipe_max_pressure = data;
		pr_info("TOUCHWAKE swipe_max_pressure has been set to %d\n", sttg_touchwake_swipe_max_pressure);
	}
	
	return size;
}

static ssize_t touchwake_swipe_finger_angle_tolerance_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_touchwake_swipe_finger_angle_tolerance);
}

static ssize_t touchwake_swipe_finger_angle_tolerance_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_touchwake_swipe_finger_angle_tolerance = data;
		pr_info("TOUCHWAKE swipe_finger_angle_tolerance has been set to %d\n", sttg_touchwake_swipe_finger_angle_tolerance);
	}
	
	return size;
}

static ssize_t touchwake_swipe_fast_trigger_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_touchwake_swipe_fast_trigger);
}

static ssize_t touchwake_swipe_fast_trigger_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if(ret && data == 1) {
		sttg_touchwake_swipe_fast_trigger = true;
		pr_info("TOUCHWAKE swipe_fast_trigger has been set\n");
	} else {
		sttg_touchwake_swipe_fast_trigger = false;
		pr_info("TOUCHWAKE swipe_fast_trigger has been unset\n");
	}
	
	return size;
}

static ssize_t touchwake_swipe_arc_rh_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_touchwake_swipe_arc_rh);
}

static ssize_t touchwake_swipe_arc_rh_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if(ret && data == 1) {
		sttg_touchwake_swipe_arc_rh = true;
		pr_info("TOUCHWAKE swipe_arc_rh has been set\n");
	} else {
		sttg_touchwake_swipe_arc_rh = false;
		pr_info("TOUCHWAKE swipe_arc_rh has been unset\n");
	}
	
	return size;
}

static ssize_t touchwake_swipe_arc_lh_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_touchwake_swipe_arc_lh);
}

static ssize_t touchwake_swipe_arc_lh_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if(ret && data == 1) {
		sttg_touchwake_swipe_arc_lh = true;
		pr_info("TOUCHWAKE swipe_arc_lh has been set\n");
	} else {
		sttg_touchwake_swipe_arc_lh = false;
		pr_info("TOUCHWAKE swipe_arc_lh has been unset\n");
	}
	
	return size;
}

static ssize_t touchwake_longpressoff_duration_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_touchwake_longpressoff_duration);
}

static ssize_t touchwake_longpressoff_duration_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_touchwake_longpressoff_duration = data;
		pr_info("TOUCHWAKE longpress_duration has been set to %d\n", sttg_touchwake_longpressoff_duration);
	}
	
	return size;
}

static ssize_t touchwake_longpressoff_timeout_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_touchwake_longpressoff_timeout / 1000);
}

static ssize_t touchwake_longpressoff_timeout_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_touchwake_longpressoff_timeout = data * 1000;
		pr_info("TOUCHWAKE longpress_timeout has been set to %d\n", sttg_touchwake_longpressoff_timeout);
	}
	
	return size;
}

static ssize_t touchwake_longpressoff_enabled_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_touchwake_longpressoff_enabled);
}

static ssize_t touchwake_longpressoff_enabled_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if(ret && data == 1) {
		sttg_touchwake_longpressoff_enabled = true;
		pr_info("TOUCHWAKE longpressoff_enabled has been set\n");
	} else {
		sttg_touchwake_longpressoff_enabled = false;
		pr_info("TOUCHWAKE longpressoff_enabled has been unset\n");
	}
	
	return size;
}

static ssize_t touchwake_longpressoff_min_pressure_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_touchwake_longpressoff_min_pressure);
}

static ssize_t touchwake_longpressoff_min_pressure_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_touchwake_longpressoff_min_pressure = data;
		wpmk_min_pressure_fuzzy = data - wpmk_min_pressure_entry_tolerance;
		//wpmk_min_pressure_fuzzy = sttg_touchwake_longpressoff_min_pressure - (((sttg_touchwake_longpressoff_min_pressure * 100) * wpmk_fuzzy_percent) / 10000); // take off 15%
		
		pr_info("TOUCHWAKE longpress_min_pressure has been set to %d (fuzzy: %i)\n", sttg_touchwake_longpressoff_min_pressure, wpmk_min_pressure_fuzzy);
	}
	
	return size;
}

static ssize_t touchwake_longpressoff_xy_drift_tolerance_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_touchwake_longpressoff_xy_drift_tolerance);
}

static ssize_t touchwake_longpressoff_xy_drift_tolerance_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_touchwake_longpressoff_xy_drift_tolerance = data;
		pr_info("TOUCHWAKE longpress_xy_drift_tolerance has been set to %d\n", sttg_touchwake_longpressoff_xy_drift_tolerance);
	}
	
	return size;
}

static ssize_t bb_mode_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_bb_mode);
}

static ssize_t bb_mode_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_bb_mode = data;
		pr_info("[TW/bb] sttg_bb_mode has been set to %d\n", sttg_bb_mode);
	}
	
	return size;
}

static ssize_t bb_magcheck_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_bb_magcheck);
}

static ssize_t bb_magcheck_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		if (data > 0) {
			data = 1;
		} else {
			data = 0;
		}
		sttg_bb_magcheck = data;
		pr_info("[TW/bb] sttg_bb_magcheck has been set to %d\n", sttg_bb_magcheck);
	}
	
	return size;
}

static ssize_t bb_magcheck_z_min_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_bb_magcheck_z_min);
}

static ssize_t bb_magcheck_z_min_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_bb_magcheck_z_min = data;
		pr_info("[TW/bb] sttg_bb_magcheck_z_min has been set to %d\n", sttg_bb_magcheck_z_min);
	}
	
	return size;
}

static ssize_t bb_magcheck_z_max_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_bb_magcheck_z_max);
}

static ssize_t bb_magcheck_z_max_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_bb_magcheck_z_max = data;
		pr_info("[TW/bb] sttg_bb_magcheck_z_max has been set to %d\n", sttg_bb_magcheck_z_max);
	}
	
	return size;
}

static ssize_t bb_dutycycle_on_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_bb_dutycycle_on);
}

static ssize_t bb_dutycycle_on_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_bb_dutycycle_on = max((int) data, 1); // no faster than 50ms
		sttg_bb_dutycycle_on = min((int) sttg_bb_dutycycle_on, 300000); // no longer than 5 minutes (300000 ms)
		pr_info("[TW/bb] sttg_bb_dutycycle_on has been set to %d\n", sttg_bb_dutycycle_on);
	}
	
	return size;
}

static ssize_t bb_dutycycle_off_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_bb_dutycycle_off);
}

static ssize_t bb_dutycycle_off_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_bb_dutycycle_off = max((int) data, 1); // no faster than 50ms
		pr_info("[TW/bb] sttg_bb_dutycycle_off has been set to %d\n", sttg_bb_dutycycle_off);
	}
	
	return size;
}

static ssize_t bb_limit_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_bb_limit);
}

static ssize_t bb_limit_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_bb_limit = max((int) data, 1); // no less than 1 blink
		pr_info("[TW/bb] sttg_bb_limit has been set to %d\n", sttg_bb_limit);
	}
	
	return size;
}

static ssize_t bb_poweron_clearedobstacle_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_bb_poweron_clearedobstacle);
}

static ssize_t bb_poweron_clearedobstacle_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		if (data > 0) {
			data = 1;
		} else {
			data = 0;
		}
		sttg_bb_poweron_clearedobstacle = data;
		pr_info("[TW/bb] sttg_bb_poweron_clearedobstacle has been set to %d\n", sttg_bb_poweron_clearedobstacle);
	}
	
	return size;
}

static ssize_t bb_brightness_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_bb_brightness);
}

static ssize_t bb_brightness_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		if (data < 1) {
			data = 1; // no dimmer than step 1.
			
		} else if (data > 15) {
			data = 15; // no brighter than step 15.
		}
		sttg_bb_brightness = data;
		pr_info("[TW/bb] sttg_bb_brightness has been set to %d\n", sttg_bb_brightness);
	}
	
	return size;
}

static ssize_t bb_blinkgroup_on_size_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_bb_blinkgroup_on_size);
}

static ssize_t bb_blinkgroup_on_size_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		if (data != 0 && data < 2) {
			data = 2; // it would be pointless to have a blinkgroup of 1.
			
		}
		sttg_bb_blinkgroup_on_size = data;
		pr_info("[TW/bb] sttg_bb_blinkgroup_on_size has been set to %d\n", sttg_bb_blinkgroup_on_size);
	}
	
	return size;
}

static ssize_t bb_blinkgroup_on_delay_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_bb_blinkgroup_on_delay);
}

static ssize_t bb_blinkgroup_on_delay_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		if (data < 1) {
			data = 1; // no faster than 1 ms.
		}
		sttg_bb_blinkgroup_on_delay = data;
		pr_info("[TW/bb] sttg_bb_blinkgroup_on_delay has been set to %d\n", sttg_bb_blinkgroup_on_delay);
	}
	
	return size;
}

static ssize_t bb_blinkgroup_off_size_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_bb_blinkgroup_off_size);
}

static ssize_t bb_blinkgroup_off_size_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		if (data != 0 && data < 2) {
			data = 2; // it would be pointless to have a blinkgroup of 1.
			
		}
		sttg_bb_blinkgroup_off_size = data;
		pr_info("[TW/bb] sttg_bb_blinkgroup_off_size has been set to %d\n", sttg_bb_blinkgroup_off_size);
	}
	
	return size;
}

static ssize_t bb_blinkgroup_off_delay_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_bb_blinkgroup_off_delay);
}

static ssize_t bb_blinkgroup_off_delay_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		if (data < 1) {
			data = 1; // no faster than 1 ms.
		}
		sttg_bb_blinkgroup_off_delay = data;
		pr_info("[TW/bb] sttg_bb_blinkgroup_off_delay has been set to %d\n", sttg_bb_blinkgroup_off_delay);
	}
	
	return size;
}

static ssize_t bb_test_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		
		if (flg_bb_testmode || data == 0) {
			
			// turn it off this time.
			flg_bb_testmode = false;
			del_timer(&timer_backblink);
			
			// turn off rear led.
			controlRearLED(0);
			
		} else {
			
			// reset some backblink stuff.
			ctr_bb_rearblink = 0;
			ctr_bb_rearblinkon = 0;
			ctr_bb_rearblinkoff = 0;
			ctr_bb_blinkgroup_on = 0;
			ctr_bb_blinkgroup_off = 0;
			
			// use manual pattern.
			bb_set_blinkpattern_manual();
			
			// start blinker.
			flg_bb_testmode = true;
			mod_timer(&timer_backblink,
					  jiffies + msecs_to_jiffies(300));
		}
	}
	
	return size;
}

static ssize_t ww_mode_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_ww_mode);
}

static ssize_t ww_mode_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_ww_mode = data;
		pr_info("[TW/ww] sttg_ww_mode has been set to %d\n", sttg_ww_mode);
	}
	
	return size;
}

static ssize_t ww_linger_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_ww_linger);
}

static ssize_t ww_linger_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_ww_linger = data;
		pr_info("[TW/ww] sttg_ww_linger has been set to %d\n", sttg_ww_linger);
	}
	
	return size;
}

static ssize_t ww_waveoff_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_ww_waveoff);
}

static ssize_t ww_waveoff_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_ww_waveoff = data;
		pr_info("[TW/ww] sttg_ww_waveoff has been set to %d\n", sttg_ww_waveoff);
	}
	
	return size;
}

static ssize_t ww_waveoff_linger_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_ww_waveoff_linger);
}

static ssize_t ww_waveoff_linger_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_ww_waveoff_linger = data;
		pr_info("[TW/ww] sttg_ww_waveoff_linger has been set to %d\n", sttg_ww_waveoff_linger);
	}
	
	return size;
}

static ssize_t ww_dismissled_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_ww_dismissled);
}

static ssize_t ww_dismissled_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_ww_dismissled = data;
		pr_info("[TW/ww] sttg_ww_dismissled has been set to %d\n", sttg_ww_dismissled);
	}
	
	return size;
}

static ssize_t ww_noredundancies_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_ww_noredundancies);
}

static ssize_t ww_noredundancies_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_ww_noredundancies = data;
		pr_info("[TW/ww] sttg_ww_noredundancies has been set to %d\n", sttg_ww_noredundancies);
	}
	
	return size;
}

static ssize_t ww_lightflow_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_ww_lightflow);
}

static ssize_t ww_lightflow_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_ww_lightflow = data;
		pr_info("[TW/ww] sttg_ww_lightflow has been set to %d\n", sttg_ww_lightflow);
	}
	
	return size;
}

static ssize_t ww_trigger_noti_only_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_ww_trigger_noti_only);
}

static ssize_t ww_trigger_noti_only_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_ww_trigger_noti_only = data;
		pr_info("[TW/ww] sttg_ww_trigger_noti_only has been set to %d\n", sttg_ww_trigger_noti_only);
	}
	
	return size;
}

static ssize_t ww_trigger_noti_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	//static unsigned int time_since_manualtrigger;
	//static unsigned int time_since_ledtriggered;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		
		// the purpose of this is to let us know when to ignore the antiredundancy checks
		// in the led hook. otherwise, repeated things (sms, etc) wouldn't know to trigger
		// a new ww/bb event.
		
		// i forgot, this is also used if the user wants to ignore the led hook (perhaps
		// from too many false positives), in which case that needs to be reflected here.
		
		pr_info("[TW/ww/manual] userspace has requested a manual noti\n");
		
		if ((sttg_ww_mode || sttg_bb_mode) && !flg_screen_on) {
			
			pr_info("[TW/ww/manual] saving manual trigger time\n");
			do_gettimeofday(&time_ww_last_manualtrigger);
			
			// boost the cpu.
			if (sttg_ww_linger > 0) {
				// boost cpu. multiply by 5, since sampling rate is probably ~200ms. hardcoded 30 sec max.
				flg_ctr_cpuboost_mid = min((int) (5 * (sttg_ww_linger / 1000)), 150);
			} else {
				// boost cpu. if sttg_ww_linger is set to unlimited it will be 0, so we cannot use it.
				flg_ctr_cpuboost_mid = 50; // ~10 seconds
			}
			
			pr_info("[TW/ww/manual] flg_ctr_cpuboost_mid: %d\n", flg_ctr_cpuboost);
			
			flg_ww_userspace_noti = true;
			pr_info("[TW/ww/manual] userspace has set flg_ww_userspace_noti\n");
			
			// now, what if the led hook is disabled? we need to start it manually.
			if (!flg_ww_prox_on && sttg_ww_trigger_noti_only) {
				
				// if we're here, it means a noti was detected but we're being told to ignore
				// the led hook. so now is our only chance to start ww/bb.
				
				pr_info("[TW/ww/manual] manually triggering ww/bb since there is no led hook\n");
				
				// if we don't use the led hook, we don't know the color, so just set this to default.
				pr_info("[TW/ww/manual] resetting bb_colors\n");
				
				bb_color_r = 0;
				bb_color_g = 0;
				bb_color_b = 0;
				
				if (sttg_ww_linger > 0) {
					// boost cpu. multiply by 5, since sampling rate is probably ~200ms. hardcoded 30 sec max.
					flg_ctr_cpuboost_mid = min((int) (5 * (sttg_ww_linger / 1000)), 150);
				} else {
					// boost cpu. if sttg_ww_linger is set to unlimited it will be 0, so we cannot use it.
					flg_ctr_cpuboost_mid = 50; // ~10 seconds
				}
				
				pr_info("[TW/ww/manual] recalculated flg_ctr_cpuboost_mid: %d\n", flg_ctr_cpuboost);
				
				pr_info("[TW/ww/manual] locking wavewake_wake_lock\n");
				wake_lock(&wavewake_wake_lock);
				pr_info("[TW/ww/manual] immediately starting PROX\n");
				forceEnableSensor(5, false);
				
				if (sttg_ww_mode) {
					// if this is a ww event, schedule normal linger.
					ww_set_disable_prox(1);
					
				} else if (sttg_bb_mode) {
					// if this is a bb event, schedule just a blip.
					ww_set_disable_prox(200);
				}
			}
			
		}
		
		/* // old code
		 if (!sttg_ww_trigger_noti_mode) {
			// user doesn't want manual noti's.
			return size;
		}
		
		pr_info("[TW/ww/manual] userspace has requested a manual noti\n");
		
		// leave this outside of the IF for global benefit.
		if (flg_ctr_cpuboost < 5) {
			// boost cpu for a few samples.
			flg_ctr_cpuboost = 5;
		}
		
		// save current time.
		do_gettimeofday(&time_now);
		
		// calculate the last time we did this.
		// we don't want to trigger this too many times too quickly.
		time_since_manualtrigger = (time_now.tv_sec - time_ww_last_manualtrigger.tv_sec) * MSEC_PER_SEC +
									(time_now.tv_usec - time_ww_last_manualtrigger.tv_usec) / USEC_PER_MSEC;
		
		if ((sttg_ww_mode > 0 || sttg_bb_mode) && !flg_screen_on && !flg_ww_prox_on && time_since_manualtrigger > 3000) {
			
			flg_ww_bypass_antiredundancy = true;
			pr_info("[TW/ww/manual] userspace has set flg_ww_bypass_antiredundancy\n");
			
			// when was bb triggered by the led last?
			time_since_ledtriggered = (time_now.tv_sec - time_ledtriggered.tv_sec) * MSEC_PER_SEC +
										(time_now.tv_usec - time_ledtriggered.tv_usec) / USEC_PER_MSEC;
			
			if (time_since_ledtriggered > 2000) {
				// it has been at least 2 seconds since the led was called.
				// this means this manual noti should use manual timings,
				// as it's consider its own event now.
				
				pr_info("[TW/ww/manual] resetting bb_colors\n");
				
				bb_color_r = 0;
				bb_color_g = 0;
				bb_color_b = 0;
			}
			
			if (sttg_ww_linger > 0) {
				// boost cpu. multiply by 5, since sampling rate is probably ~200ms. hardcoded 30 sec max.
				flg_ctr_cpuboost_mid = min((int) (5 * (sttg_ww_linger / 1000)), 150);
			} else {
				// boost cpu. if sttg_ww_linger is set to unlimited it will be 0, so we cannot use it.
				flg_ctr_cpuboost_mid = 50; // ~10 seconds
			}
			
			pr_info("[TW/ww/manual] flg_ctr_cpuboost: %d\n", flg_ctr_cpuboost);
			
			pr_info("[TW/ww/manual] saving manual trigger time\n");
			do_gettimeofday(&time_ww_last_manualtrigger);
			
			pr_info("[TW/ww/manual] locking wavewake_wake_lock\n");
			wake_lock(&wavewake_wake_lock);
			pr_info("[TW/ww/manual] immediately starting PROX\n");
			forceEnableSensor(5, false);
			
			if (sttg_ww_mode) {
				// if this is a ww event, schedule normal linger.
				ww_set_disable_prox(1);
				
			} else if (sttg_bb_mode) {
				// if this is a bb event, schedule just a blip.
				ww_set_disable_prox(200);
			}
		}*/
	}
	
	return size;
}

// knockwake settings.
/////////////////////////////////

static ssize_t kw_mode_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_kw_mode);
}

static ssize_t kw_mode_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_kw_mode = data;
		pr_info("[TW/kw] sttg_kw_mode has been set to %d\n", sttg_kw_mode);
		if (data == 0) {
			kw_status_code = 999;
			pr_info("[TW/ssp/kw] immediately stopping GYRO\n");
			forceDisableSensor(1);
			pr_info("[SSP/kw] unlocking knockwake_wake_lock\n");
			wake_unlock(&knockwake_wake_lock);
		} else {
			if (sttg_ka_mode) {
				// turn on gyro if kw and ka are on.
				kw_status_code = 1;
				pr_info("[TW/ssp/kw] stopping GYRO\n");
				forceDisableSensor(1);
				pr_info("[TW/ssp/kw] starting GYRO\n");
				pr_info("[SSP/kw] locking knockwake_wake_lock\n");
				wake_lock(&knockwake_wake_lock);
				forceEnableSensor(1, false);
			}
		}
	}
	
	return size;
}

static ssize_t kw_resolution_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%lld\n", sttg_kw_resolution);
}

static ssize_t kw_resolution_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	int64_t data;
	
	if (strict_strtoll(buf, 10, &data) < 0)
		return -1;
	
	sttg_kw_resolution = data;
	forceChangeDelay(1, data);
	pr_info("[TW/kw] sttg_kw_resolution has been set to %lld\n", sttg_kw_resolution);
	
	return size;
}

static ssize_t kw_gyro_y_trigger_min_threshold_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_kw_gyro_y_trigger_min_threshold);
}

static ssize_t kw_gyro_y_trigger_min_threshold_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_kw_gyro_y_trigger_min_threshold = data;
		pr_info("[TW/kw] sttg_kw_gyro_y_trigger_min_threshold has been set to %d\n", sttg_kw_gyro_y_trigger_min_threshold);
	}
	
	return size;
}

static ssize_t kw_gyro_y_trigger_max_threshold_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_kw_gyro_y_trigger_max_threshold);
}

static ssize_t kw_gyro_y_trigger_max_threshold_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_kw_gyro_y_trigger_max_threshold = data;
		pr_info("[TW/kw] sttg_kw_gyro_y_trigger_max_threshold has been set to %d\n", sttg_kw_gyro_y_trigger_max_threshold);
	}
	
	return size;
}

static ssize_t kw_gyro_y_history_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_kw_gyro_y_history);
}

static ssize_t kw_gyro_y_history_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_kw_gyro_y_history = data;
		pr_info("[TW/kw] sttg_kw_gyro_y_history has been set to %d\n", sttg_kw_gyro_y_history);
	}
	
	return size;
}

static ssize_t kw_knock2_window_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_kw_knock2_window);
}

static ssize_t kw_knock2_window_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_kw_knock2_window = data;
		pr_info("[TW/kw] sttg_kw_knock2_window has been set to %d\n", sttg_kw_knock2_window);
	}
	
	return size;
}

static ssize_t kw_knock3_keycode_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_kw_knock3_keycode);
}

static ssize_t kw_knock3_keycode_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_kw_knock3_keycode = data;
		pr_info("[TW/kw] sttg_kw_knock3_keycode has been set to %d\n", sttg_kw_knock3_keycode);
	}
	
	return size;
}

static ssize_t kw_knock3_keydelay_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_kw_knock3_keydelay);
}

static ssize_t kw_knock3_keydelay_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		if (data > 0) {
			data = 1;
		} else {
			data = 0;
		}
		sttg_kw_knock3_keydelay = data;
		pr_info("[TW/kw] sttg_kw_knock3_keydelay has been set to %d\n", sttg_kw_knock3_keydelay);
	}
	
	return size;
}

static ssize_t kw_knock3_window_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_kw_knock3_window);
}

static ssize_t kw_knock3_window_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_kw_knock3_window = data;
		pr_info("[TW/kw] sttg_kw_knock3_window has been set to %d\n", sttg_kw_knock3_window);
	}
	
	return size;
}

static ssize_t kw_knock4_window_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_kw_knock4_window);
}

static ssize_t kw_knock4_window_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_kw_knock4_window = data;
		pr_info("[TW/kw] sttg_kw_knock4_window has been set to %d\n", sttg_kw_knock4_window);
	}
	
	return size;
}

static ssize_t kw_knock4_keycode_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_kw_knock4_keycode);
}

static ssize_t kw_knock4_keycode_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_kw_knock4_keycode = data;
		pr_info("[TW/kw] sttg_kw_knock4_keycode has been set to %d\n", sttg_kw_knock4_keycode);
	}
	
	return size;
}

static ssize_t kw_knock4_keydelay_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_kw_knock4_keydelay);
}

static ssize_t kw_knock4_keydelay_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		if (data > 0) {
			data = 1;
		} else {
			data = 0;
		}
		sttg_kw_knock4_keydelay = data;
		pr_info("[TW/kw] sttg_kw_knock4_keydelay has been set to %d\n", sttg_kw_knock4_keydelay);
	}
	
	return size;
}

static ssize_t kw_gyro_y_min_threshold_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_kw_gyro_y_min_threshold);
}

static ssize_t kw_gyro_y_min_threshold_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_kw_gyro_y_min_threshold = data;
		pr_info("[TW/kw] sttg_kw_gyro_y_min_threshold has been set to %d\n", sttg_kw_gyro_y_min_threshold);
	}
	
	return size;
}

static ssize_t kw_gyro_y_max_threshold_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_kw_gyro_y_max_threshold);
}

static ssize_t kw_gyro_y_max_threshold_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_kw_gyro_y_max_threshold = data;
		pr_info("[TW/kw] sttg_kw_gyro_y_max_threshold has been set to %d\n", sttg_kw_gyro_y_max_threshold);
	}
	
	return size;
}

static ssize_t kw_gyro_y_min_average_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_kw_gyro_y_min_average);
}

static ssize_t kw_gyro_y_min_average_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_kw_gyro_y_min_average = data;
		pr_info("[TW/kw] sttg_kw_gyro_y_min_average has been set to %d\n", sttg_kw_gyro_y_min_average);
	}
	
	return size;
}

static ssize_t kw_gyro_y_max_average_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_kw_gyro_y_max_average);
}

static ssize_t kw_gyro_y_max_average_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_kw_gyro_y_max_average = data;
		pr_info("[TW/kw] sttg_kw_gyro_y_max_average has been set to %d\n", sttg_kw_gyro_y_max_average);
	}
	
	return size;
}

static ssize_t kw_gyro_x_min_threshold_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%d\n", sttg_kw_gyro_x_min_threshold);
}

static ssize_t kw_gyro_x_min_threshold_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	int data;
	
	if(sscanf(buf, "%i\n", &data) == 1) {
		sttg_kw_gyro_x_min_threshold = data;
		pr_info("[TW/kw] sttg_kw_gyro_x_min_threshold has been set to %d\n", sttg_kw_gyro_x_min_threshold);
	}
	
	return size;
}

static ssize_t kw_gyro_x_max_threshold_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%d\n", sttg_kw_gyro_x_max_threshold);
}

static ssize_t kw_gyro_x_max_threshold_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	int data;
	
	if(sscanf(buf, "%i\n", &data) == 1) {
		sttg_kw_gyro_x_max_threshold = data;
		pr_info("[TW/kw] sttg_kw_gyro_x_max_threshold has been set to %d\n", sttg_kw_gyro_x_max_threshold);
	}
	
	return size;
}

static ssize_t kw_gyro_z_min_threshold_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%d\n", sttg_kw_gyro_z_min_threshold);
}

static ssize_t kw_gyro_z_min_threshold_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	int data;
	
	if(sscanf(buf, "%i\n", &data) == 1) {
		sttg_kw_gyro_z_min_threshold = data;
		pr_info("[TW/kw] sttg_kw_gyro_z_min_threshold has been set to %d\n", sttg_kw_gyro_z_min_threshold);
	}
	
	return size;
}

static ssize_t kw_gyro_z_max_threshold_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%d\n", sttg_kw_gyro_z_max_threshold);
}

static ssize_t kw_gyro_z_max_threshold_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	int data;
	
	if(sscanf(buf, "%i\n", &data) == 1) {
		sttg_kw_gyro_z_max_threshold = data;
		pr_info("[TW/kw] sttg_kw_gyro_z_max_threshold has been set to %d\n", sttg_kw_gyro_z_max_threshold);
	}
	
	return size;
}

static ssize_t kw_z_instab_suspensions_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_kw_z_instab_suspensions);
}

static ssize_t kw_z_instab_suspensions_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_kw_z_instab_suspensions = data;
		pr_info("[TW/kw] sttg_kw_z_instab_suspensions has been set to %d\n", sttg_kw_z_instab_suspensions);
	}
	
	return size;
}

static ssize_t kw_init_detection_suspensions_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_kw_init_detection_suspensions);
}

static ssize_t kw_init_detection_suspensions_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_kw_init_detection_suspensions = data;
		pr_info("[TW/kw] sttg_kw_init_detection_suspensions has been set to %d\n", sttg_kw_init_detection_suspensions);
	}
	
	return size;
}

static ssize_t kw_post_suspension_validations_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_kw_post_suspension_validations);
}

static ssize_t kw_post_suspension_validations_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_kw_post_suspension_validations = data;
		pr_info("[TW/kw] sttg_kw_post_suspension_validations has been set to %d\n", sttg_kw_post_suspension_validations);
	}
	
	return size;
}

static ssize_t kw_strict_validation_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_kw_strict_validation);
}

static ssize_t kw_strict_validation_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		if (data > 0) {
			data = 1;
		} else {
			data = 0;
		}
		sttg_kw_strict_validation = data;
		pr_info("[TW/kw] sttg_kw_strict_validation has been set to %d\n", sttg_kw_strict_validation);
	}
	
	return size;
}

static ssize_t kw_instab_tolerance_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_kw_instab_tolerance);
}

static ssize_t kw_instab_tolerance_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_kw_instab_tolerance = data;
		pr_info("[TW/kw] sttg_kw_instab_tolerance has been set to %d\n", sttg_kw_instab_tolerance);
	}
	
	return size;
}

static ssize_t kw_extreme_instab_tolerance_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_kw_extreme_instab_tolerance);
}

static ssize_t kw_extreme_instab_tolerance_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_kw_extreme_instab_tolerance = data;
		pr_info("[TW/kw] sttg_kw_extreme_instab_tolerance has been set to %d\n", sttg_kw_extreme_instab_tolerance);
	}
	
	return size;
}

static ssize_t kw_extreme_instab_suspensions_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_kw_extreme_instab_suspensions);
}

static ssize_t kw_extreme_instab_suspensions_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_kw_extreme_instab_suspensions = data;
		pr_info("[TW/kw] sttg_kw_extreme_instab_suspensions has been set to %d\n", sttg_kw_extreme_instab_suspensions);
	}
	
	return size;
}

static ssize_t kw_knock_min_speed_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_kw_knock_min_speed);
}

static ssize_t kw_knock_min_speed_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_kw_knock_min_speed = data;
		pr_info("[TW/kw] sttg_kw_knock_min_speed has been set to %d\n", sttg_kw_knock_min_speed);
	}
	
	return size;
}

static ssize_t kw_debug_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_kw_debug);
}

static ssize_t kw_debug_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		if (data > 0) {
			data = 1;
		} else {
			data = 0;
		}
		sttg_kw_debug = data;
		pr_info("[TW/kw] sttg_kw_debug has been set to %d\n", sttg_kw_debug);
	}
	
	return size;
}

static ssize_t kw_tsp_event_threshold_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_kw_tsp_event_threshold);
}

static ssize_t kw_tsp_event_threshold_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_kw_tsp_event_threshold = data;
		pr_info("[TW/kw] sttg_kw_tsp_event_threshold has been set to %d\n", sttg_kw_tsp_event_threshold);
	}
	
	return size;
}

static ssize_t kw_tsp_event_suspensions_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_kw_tsp_event_suspensions);
}

static ssize_t kw_tsp_event_suspensions_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_kw_tsp_event_suspensions = data;
		pr_info("[TW/kw] sttg_kw_tsp_event_suspensions has been set to %d\n", sttg_kw_tsp_event_suspensions);
	}
	
	return size;
}

static ssize_t kw_status_code_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", kw_status_code);
	
}

// knockactive
/////////////////////////////////////////

static ssize_t ka_mode_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_ka_mode);
}

static ssize_t ka_mode_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		if (data > 0) {
			data = 1;
			if (sttg_kw_mode) {
				kw_status_code = 1;
				forceDisableSensor(1);
				pr_info("[SSP/kw] locking knockwake_wake_lock\n");
				wake_lock(&knockwake_wake_lock);
				pr_info("[TW/ssp/kw] immediately starting GYRO\n");
				forceEnableSensor(1, false);
			}
		} else {
			data = 0;
			pr_info("[SSP/kw] unlocking knockwake_wake_lock\n");
			wake_unlock(&knockwake_wake_lock);
			pr_info("[TW/ssp/kw] immediately stopping GYRO\n");
			forceDisableSensor(1);
			kw_status_code = 999;
		}
		sttg_ka_mode = data;
		pr_info("[TW/ka] sttg_ka_mode has been set to %d\n", sttg_ka_mode);
	}
	
	return size;
}

static ssize_t ka_knock2_window_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_ka_knock2_window);
}

static ssize_t ka_knock2_window_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_ka_knock2_window = data;
		pr_info("[TW/kw] sttg_ka_knock2_window has been set to %d\n", sttg_ka_knock2_window);
	}
	
	return size;
}

static ssize_t ka_knock3_window_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_ka_knock3_window);
}

static ssize_t ka_knock3_window_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_ka_knock3_window = data;
		pr_info("[TW/kw] sttg_ka_knock3_window has been set to %d\n", sttg_ka_knock3_window);
	}
	
	return size;
}

static ssize_t ka_knock2_keycode_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_ka_knock2_keycode);
}

static ssize_t ka_knock2_keycode_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_ka_knock2_keycode = data;
		pr_info("[TW/kw] sttg_ka_knock2_keycode has been set to %d\n", sttg_ka_knock2_keycode);
	}
	
	return size;
}

static ssize_t ka_knock2_keydelay_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_ka_knock2_keydelay);
}

static ssize_t ka_knock2_keydelay_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		if (data > 0) {
			data = 1;
		} else {
			data = 0;
		}
		sttg_ka_knock2_keydelay = data;
		pr_info("[TW/kw] sttg_ka_knock2_keydelay has been set to %d\n", sttg_ka_knock2_keydelay);
	}
	
	return size;
}

static ssize_t ka_knock3_keycode_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_ka_knock3_keycode);
}

static ssize_t ka_knock3_keycode_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_ka_knock3_keycode = data;
		pr_info("[TW/kw] sttg_ka_knock3_keycode has been set to %d\n", sttg_ka_knock3_keycode);
	}
	
	return size;
}

static ssize_t ka_knock3_keydelay_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_ka_knock3_keydelay);
}

static ssize_t ka_knock3_keydelay_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		if (data > 0) {
			data = 1;
		} else {
			data = 0;
		}
		sttg_ka_knock3_keydelay = data;
		pr_info("[TW/kw] sttg_ka_knock3_keydelay has been set to %d\n", sttg_ka_knock3_keydelay);
	}
	
	return size;
}

static ssize_t ka_knock2_min_speed_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_ka_knock2_min_speed);
}

static ssize_t ka_knock2_min_speed_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_ka_knock2_min_speed = data;
		pr_info("[TW/kw] sttg_ka_knock2_min_speed has been set to %d\n", sttg_ka_knock2_min_speed);
	}
	
	return size;
}

static ssize_t ka_knock3_min_speed_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_ka_knock3_min_speed);
}

static ssize_t ka_knock3_min_speed_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_ka_knock3_min_speed = data;
		pr_info("[TW/kw] sttg_ka_knock3_min_speed has been set to %d\n", sttg_ka_knock3_min_speed);
	}
	
	return size;
}

static ssize_t tw_usewavewake_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_tw_usewavewake);
}

static ssize_t tw_usewavewake_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		if (data > 0) {
			data = 1;
		} else {
			data = 0;
		}
		sttg_tw_usewavewake = data;
		pr_info("[TW/kw] sttg_tw_usewavewake has been set to %d\n", sttg_tw_usewavewake);
	}
	
	return size;
}

int get_touchoff_delay()
{

	return touchoff_delay;
}
EXPORT_SYMBOL(get_touchoff_delay);

static ssize_t touchwake_version(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%s\n", TOUCHWAKE_VERSION);
}

#ifdef DEBUG_PRINT
static ssize_t touchwake_debug(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "timed_out : %u\nprox_near : %u\n", (unsigned int) timed_out, (unsigned int) prox_near);
}
#endif

static DEVICE_ATTR(enabled, S_IRUGO | S_IWUGO, touchwake_status_read, touchwake_status_write);
static DEVICE_ATTR(delay, S_IRUGO | S_IWUGO, touchwake_delay_read, touchwake_delay_write);
static DEVICE_ATTR(ignore_once, S_IRUGO | S_IWUGO, touchwake_ignore_once_read, touchwake_ignore_once_write);
static DEVICE_ATTR(version, S_IRUGO , touchwake_version, NULL);
static DEVICE_ATTR(slide2wake_only, S_IRUGO | S_IWUGO, touchwake_slide2wake_only_read, touchwake_slide2wake_only_write);
static DEVICE_ATTR(enable_persistent, S_IRUGO | S_IWUGO, touchwake_enable_persistent_read, touchwake_enable_persistent_write);
static DEVICE_ATTR(enable_persistent_wakelock, S_IRUGO | S_IWUGO, touchwake_enable_persistent_wakelock_read, touchwake_enable_persistent_wakelock_write);
static DEVICE_ATTR(ignore_tkeys, S_IRUGO | S_IWUGO, touchwake_ignore_tkeys_read, touchwake_ignore_tkeys_write);
static DEVICE_ATTR(ignore_gestures, S_IRUGO | S_IWUGO, touchwake_ignore_gestures_read, touchwake_ignore_gestures_write);
static DEVICE_ATTR(ignore_powerkey, S_IRUGO | S_IWUGO, touchwake_ignore_powerkey_read, touchwake_ignore_powerkey_write);
static DEVICE_ATTR(ignore_homekeywake, S_IRUGO | S_IWUGO, touchwake_ignore_homekeywake_read, touchwake_ignore_homekeywake_write);
static DEVICE_ATTR(delay_afterpoweroff, S_IRUGO | S_IWUGO, touchwake_delay_afterpoweroff_read, touchwake_delay_afterpoweroff_write);
static DEVICE_ATTR(tw_usewavewake, S_IRUGO | S_IWUGO, tw_usewavewake_read, tw_usewavewake_write);
static DEVICE_ATTR(force_timedout_tapwake, S_IRUGO | S_IWUGO, touchwake_force_timedout_tapwake_read, touchwake_force_timedout_tapwake_write);
static DEVICE_ATTR(swipe_y_drift_tolerance, S_IRUGO | S_IWUGO, touchwake_swipe_y_drift_tolerance_read, touchwake_swipe_y_drift_tolerance_write);
static DEVICE_ATTR(swipe_max_pressure, S_IRUGO | S_IWUGO, touchwake_swipe_max_pressure_read, touchwake_swipe_max_pressure_write);
static DEVICE_ATTR(swipe_finger_angle_tolerance, S_IRUGO | S_IWUGO, touchwake_swipe_finger_angle_tolerance_read, touchwake_swipe_finger_angle_tolerance_write);
static DEVICE_ATTR(swipe_fast_trigger, S_IRUGO | S_IWUGO, touchwake_swipe_fast_trigger_read, touchwake_swipe_fast_trigger_write);
static DEVICE_ATTR(swipe_arc_rh, S_IRUGO | S_IWUGO, touchwake_swipe_arc_rh_read, touchwake_swipe_arc_rh_write);
static DEVICE_ATTR(swipe_arc_lh, S_IRUGO | S_IWUGO, touchwake_swipe_arc_lh_read, touchwake_swipe_arc_lh_write);
static DEVICE_ATTR(longpressoff_enabled, S_IRUGO | S_IWUGO, touchwake_longpressoff_enabled_read, touchwake_longpressoff_enabled_write);
static DEVICE_ATTR(longpressoff_duration, S_IRUGO | S_IWUGO, touchwake_longpressoff_duration_read, touchwake_longpressoff_duration_write);
static DEVICE_ATTR(longpressoff_timeout, S_IRUGO | S_IWUGO, touchwake_longpressoff_timeout_read, touchwake_longpressoff_timeout_write);
static DEVICE_ATTR(longpressoff_min_pressure, S_IRUGO | S_IWUGO, touchwake_longpressoff_min_pressure_read, touchwake_longpressoff_min_pressure_write);
static DEVICE_ATTR(longpressoff_xy_drift_tolerance, S_IRUGO | S_IWUGO, touchwake_longpressoff_xy_drift_tolerance_read, touchwake_longpressoff_xy_drift_tolerance_write);
static DEVICE_ATTR(bb_mode, S_IRUGO | S_IWUGO, bb_mode_read, bb_mode_write);
static DEVICE_ATTR(bb_magcheck, S_IRUGO | S_IWUGO, bb_magcheck_read, bb_magcheck_write);
static DEVICE_ATTR(bb_magcheck_z_min, S_IRUGO | S_IWUGO, bb_magcheck_z_min_read, bb_magcheck_z_min_write);
static DEVICE_ATTR(bb_magcheck_z_max, S_IRUGO | S_IWUGO, bb_magcheck_z_max_read, bb_magcheck_z_max_write);
static DEVICE_ATTR(bb_dutycycle_on, S_IRUGO | S_IWUGO, bb_dutycycle_on_read, bb_dutycycle_on_write);
static DEVICE_ATTR(bb_dutycycle_off, S_IRUGO | S_IWUGO, bb_dutycycle_off_read, bb_dutycycle_off_write);
static DEVICE_ATTR(bb_limit, S_IRUGO | S_IWUGO, bb_limit_read, bb_limit_write);
static DEVICE_ATTR(bb_poweron_clearedobstacle, S_IRUGO | S_IWUGO, bb_poweron_clearedobstacle_read, bb_poweron_clearedobstacle_write);
static DEVICE_ATTR(bb_brightness, S_IRUGO | S_IWUGO, bb_brightness_read, bb_brightness_write);
static DEVICE_ATTR(bb_blinkgroup_on_size, S_IRUGO | S_IWUGO, bb_blinkgroup_on_size_read, bb_blinkgroup_on_size_write);
static DEVICE_ATTR(bb_blinkgroup_on_delay, S_IRUGO | S_IWUGO, bb_blinkgroup_on_delay_read, bb_blinkgroup_on_delay_write);
static DEVICE_ATTR(bb_blinkgroup_off_size, S_IRUGO | S_IWUGO, bb_blinkgroup_off_size_read, bb_blinkgroup_off_size_write);
static DEVICE_ATTR(bb_blinkgroup_off_delay, S_IRUGO | S_IWUGO, bb_blinkgroup_off_delay_read, bb_blinkgroup_off_delay_write);
static DEVICE_ATTR(bb_test, S_IWUGO, NULL, bb_test_write);
static DEVICE_ATTR(ww_mode, S_IRUGO | S_IWUGO, ww_mode_read, ww_mode_write);
static DEVICE_ATTR(ww_linger, S_IRUGO | S_IWUGO, ww_linger_read, ww_linger_write);
static DEVICE_ATTR(ww_waveoff, S_IRUGO | S_IWUGO, ww_waveoff_read, ww_waveoff_write);
static DEVICE_ATTR(ww_waveoff_linger, S_IRUGO | S_IWUGO, ww_waveoff_linger_read, ww_waveoff_linger_write);
static DEVICE_ATTR(ww_dismissled, S_IRUGO | S_IWUGO, ww_dismissled_read, ww_dismissled_write);
static DEVICE_ATTR(ww_noredundancies, S_IRUGO | S_IWUGO, ww_noredundancies_read, ww_noredundancies_write);
static DEVICE_ATTR(ww_lightflow, S_IRUGO | S_IWUGO, ww_lightflow_read, ww_lightflow_write);
static DEVICE_ATTR(ww_trigger_noti_only, S_IRUGO | S_IWUGO, ww_trigger_noti_only_read, ww_trigger_noti_only_write);
static DEVICE_ATTR(ww_trigger_noti, S_IWUGO, NULL, ww_trigger_noti_write);
static DEVICE_ATTR(kw_mode, S_IRUGO | S_IWUGO, kw_mode_read, kw_mode_write);
static DEVICE_ATTR(kw_resolution, S_IRUGO | S_IWUGO, kw_resolution_read, kw_resolution_write);
static DEVICE_ATTR(kw_gyro_y_trigger_min_threshold, S_IRUGO | S_IWUGO, kw_gyro_y_trigger_min_threshold_read, kw_gyro_y_trigger_min_threshold_write);
static DEVICE_ATTR(kw_gyro_y_trigger_max_threshold, S_IRUGO | S_IWUGO, kw_gyro_y_trigger_max_threshold_read, kw_gyro_y_trigger_max_threshold_write);
static DEVICE_ATTR(kw_gyro_y_history, S_IRUGO | S_IWUGO, kw_gyro_y_history_read, kw_gyro_y_history_write);
static DEVICE_ATTR(kw_knock2_window, S_IRUGO | S_IWUGO, kw_knock2_window_read, kw_knock2_window_write);
static DEVICE_ATTR(kw_knock3_window, S_IRUGO | S_IWUGO, kw_knock3_window_read, kw_knock3_window_write);
static DEVICE_ATTR(kw_knock3_keycode, S_IRUGO | S_IWUGO, kw_knock3_keycode_read, kw_knock3_keycode_write);
static DEVICE_ATTR(kw_knock3_keydelay, S_IRUGO | S_IWUGO, kw_knock3_keydelay_read, kw_knock3_keydelay_write);
static DEVICE_ATTR(kw_knock4_window, S_IRUGO | S_IWUGO, kw_knock4_window_read, kw_knock4_window_write);
static DEVICE_ATTR(kw_knock4_keycode, S_IRUGO | S_IWUGO, kw_knock4_keycode_read, kw_knock4_keycode_write);
static DEVICE_ATTR(kw_knock4_keydelay, S_IRUGO | S_IWUGO, kw_knock4_keydelay_read, kw_knock4_keydelay_write);
static DEVICE_ATTR(kw_gyro_y_min_threshold, S_IRUGO | S_IWUGO, kw_gyro_y_min_threshold_read, kw_gyro_y_min_threshold_write);
static DEVICE_ATTR(kw_gyro_y_max_threshold, S_IRUGO | S_IWUGO, kw_gyro_y_max_threshold_read, kw_gyro_y_max_threshold_write);
static DEVICE_ATTR(kw_gyro_y_min_average, S_IRUGO | S_IWUGO, kw_gyro_y_min_average_read, kw_gyro_y_min_average_write);
static DEVICE_ATTR(kw_gyro_y_max_average, S_IRUGO | S_IWUGO, kw_gyro_y_max_average_read, kw_gyro_y_max_average_write);
static DEVICE_ATTR(kw_gyro_x_min_threshold, S_IRUGO | S_IWUGO, kw_gyro_x_min_threshold_read, kw_gyro_x_min_threshold_write);
static DEVICE_ATTR(kw_gyro_x_max_threshold, S_IRUGO | S_IWUGO, kw_gyro_x_max_threshold_read, kw_gyro_x_max_threshold_write);
static DEVICE_ATTR(kw_gyro_z_min_threshold, S_IRUGO | S_IWUGO, kw_gyro_z_min_threshold_read, kw_gyro_z_min_threshold_write);
static DEVICE_ATTR(kw_gyro_z_max_threshold, S_IRUGO | S_IWUGO, kw_gyro_z_max_threshold_read, kw_gyro_z_max_threshold_write);
static DEVICE_ATTR(kw_z_instab_suspensions, S_IRUGO | S_IWUGO, kw_z_instab_suspensions_read, kw_z_instab_suspensions_write);
static DEVICE_ATTR(kw_init_detection_suspensions, S_IRUGO | S_IWUGO, kw_init_detection_suspensions_read, kw_init_detection_suspensions_write);
static DEVICE_ATTR(kw_post_suspension_validations, S_IRUGO | S_IWUGO, kw_post_suspension_validations_read, kw_post_suspension_validations_write);
static DEVICE_ATTR(kw_strict_validation, S_IRUGO | S_IWUGO, kw_strict_validation_read, kw_strict_validation_write);
static DEVICE_ATTR(kw_instab_tolerance, S_IRUGO | S_IWUGO, kw_instab_tolerance_read, kw_instab_tolerance_write);
static DEVICE_ATTR(kw_extreme_instab_tolerance, S_IRUGO | S_IWUGO, kw_extreme_instab_tolerance_read, kw_extreme_instab_tolerance_write);
static DEVICE_ATTR(kw_extreme_instab_suspensions, S_IRUGO | S_IWUGO, kw_extreme_instab_suspensions_read, kw_extreme_instab_suspensions_write);
static DEVICE_ATTR(kw_knock_min_speed, S_IRUGO | S_IWUGO, kw_knock_min_speed_read, kw_knock_min_speed_write);
static DEVICE_ATTR(kw_status_code, S_IRUGO, kw_status_code_read, NULL);
static DEVICE_ATTR(kw_debug, S_IRUGO | S_IWUGO, kw_debug_read, kw_debug_write);
static DEVICE_ATTR(ka_mode, S_IRUGO | S_IWUGO, ka_mode_read, ka_mode_write);
static DEVICE_ATTR(kw_tsp_event_threshold, S_IRUGO | S_IWUGO, kw_tsp_event_threshold_read, kw_tsp_event_threshold_write);
static DEVICE_ATTR(kw_tsp_event_suspensions, S_IRUGO | S_IWUGO, kw_tsp_event_suspensions_read, kw_tsp_event_suspensions_write);
static DEVICE_ATTR(ka_knock2_window, S_IRUGO | S_IWUGO, ka_knock2_window_read, ka_knock2_window_write);
static DEVICE_ATTR(ka_knock3_window, S_IRUGO | S_IWUGO, ka_knock3_window_read, ka_knock3_window_write);
static DEVICE_ATTR(ka_knock2_keycode, S_IRUGO | S_IWUGO, ka_knock2_keycode_read, ka_knock2_keycode_write);
static DEVICE_ATTR(ka_knock2_keydelay, S_IRUGO | S_IWUGO, ka_knock2_keydelay_read, ka_knock2_keydelay_write);
static DEVICE_ATTR(ka_knock3_keycode, S_IRUGO | S_IWUGO, ka_knock3_keycode_read, ka_knock3_keycode_write);
static DEVICE_ATTR(ka_knock3_keydelay, S_IRUGO | S_IWUGO, ka_knock3_keydelay_read, ka_knock3_keydelay_write);
static DEVICE_ATTR(ka_knock2_min_speed, S_IRUGO | S_IWUGO, ka_knock2_min_speed_read, ka_knock2_min_speed_write);
static DEVICE_ATTR(ka_knock3_min_speed, S_IRUGO | S_IWUGO, ka_knock3_min_speed_read, ka_knock3_min_speed_write);
#ifdef DEBUG_PRINT
static DEVICE_ATTR(debug, S_IRUGO , touchwake_debug, NULL);
#endif

static struct attribute *touchwake_notification_attributes[] =
{
	&dev_attr_enabled.attr,
	&dev_attr_delay.attr,
	&dev_attr_ignore_once.attr,
	&dev_attr_version.attr,
    &dev_attr_slide2wake_only.attr,
    &dev_attr_enable_persistent.attr,
    &dev_attr_enable_persistent_wakelock.attr,
    &dev_attr_ignore_tkeys.attr,
    &dev_attr_ignore_gestures.attr,
    &dev_attr_ignore_powerkey.attr,
	&dev_attr_ignore_homekeywake.attr,
    &dev_attr_delay_afterpoweroff.attr,
    &dev_attr_force_timedout_tapwake.attr,
	&dev_attr_tw_usewavewake.attr,
    &dev_attr_swipe_y_drift_tolerance.attr,
    &dev_attr_swipe_max_pressure.attr,
    &dev_attr_swipe_finger_angle_tolerance.attr,
    &dev_attr_swipe_fast_trigger.attr,
    &dev_attr_swipe_arc_rh.attr,
    &dev_attr_swipe_arc_lh.attr,
    &dev_attr_longpressoff_enabled.attr,
    &dev_attr_longpressoff_duration.attr,
    &dev_attr_longpressoff_timeout.attr,
    &dev_attr_longpressoff_min_pressure.attr,
    &dev_attr_longpressoff_xy_drift_tolerance.attr,
	&dev_attr_bb_mode.attr,
	&dev_attr_bb_magcheck.attr,
	&dev_attr_bb_magcheck_z_min.attr,
	&dev_attr_bb_magcheck_z_max.attr,
	&dev_attr_bb_dutycycle_on.attr,
	&dev_attr_bb_dutycycle_off.attr,
	&dev_attr_bb_limit.attr,
	&dev_attr_bb_poweron_clearedobstacle.attr,
	&dev_attr_bb_brightness.attr,
	&dev_attr_bb_blinkgroup_on_size.attr,
	&dev_attr_bb_blinkgroup_on_delay.attr,
	&dev_attr_bb_blinkgroup_off_size.attr,
	&dev_attr_bb_blinkgroup_off_delay.attr,
	&dev_attr_bb_test.attr,
	&dev_attr_ww_mode.attr,
	&dev_attr_ww_linger.attr,
	&dev_attr_ww_waveoff.attr,
	&dev_attr_ww_waveoff_linger.attr,
	&dev_attr_ww_dismissled.attr,
	&dev_attr_ww_noredundancies.attr,
	&dev_attr_ww_lightflow.attr,
	&dev_attr_ww_trigger_noti_only.attr,
	&dev_attr_ww_trigger_noti.attr,
	&dev_attr_kw_mode,
	&dev_attr_kw_resolution,
	&dev_attr_kw_gyro_y_trigger_min_threshold,
	&dev_attr_kw_gyro_y_trigger_max_threshold,
	&dev_attr_kw_gyro_y_history,
	&dev_attr_kw_knock2_window,
	&dev_attr_kw_knock3_window,
	&dev_attr_kw_knock3_keycode,
	&dev_attr_kw_knock3_keydelay,
	&dev_attr_kw_knock4_window,
	&dev_attr_kw_knock4_keycode,
	&dev_attr_kw_knock4_keydelay,
	&dev_attr_kw_gyro_y_min_threshold,
	&dev_attr_kw_gyro_y_max_threshold,
	&dev_attr_kw_gyro_y_min_average,
	&dev_attr_kw_gyro_y_max_average,
	&dev_attr_kw_gyro_x_min_threshold,
	&dev_attr_kw_gyro_x_max_threshold,
	&dev_attr_kw_gyro_z_min_threshold,
	&dev_attr_kw_gyro_z_max_threshold,
	&dev_attr_kw_z_instab_suspensions,
	&dev_attr_kw_init_detection_suspensions,
	&dev_attr_kw_post_suspension_validations,
	&dev_attr_kw_strict_validation,
	&dev_attr_kw_instab_tolerance,
	&dev_attr_kw_extreme_instab_tolerance,
	&dev_attr_kw_extreme_instab_suspensions,
	&dev_attr_kw_knock_min_speed,
	&dev_attr_kw_status_code,
	&dev_attr_kw_debug,
	&dev_attr_kw_tsp_event_threshold,
	&dev_attr_kw_tsp_event_suspensions,
	&dev_attr_ka_mode,
	&dev_attr_ka_knock2_window,
	&dev_attr_ka_knock3_window,
	&dev_attr_ka_knock2_keycode,
	&dev_attr_ka_knock2_keydelay,
	&dev_attr_ka_knock3_keycode,
	&dev_attr_ka_knock3_keydelay,
	&dev_attr_ka_knock2_min_speed,
	&dev_attr_ka_knock3_min_speed,
#ifdef DEBUG_PRINT
	&dev_attr_debug.attr,
#endif
	NULL
};

static struct attribute_group touchwake_notification_group =
{
	.attrs  = touchwake_notification_attributes,
};

static struct miscdevice touchwake_device =
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = "touchwake",
};

static void bb_start(void)
{
	unsigned int tmp_set_disable_prox = 0;
	unsigned int tmp_blinkgroups = 0;
	
	pr_info("[TW/bb/bb_start] ACTIVATING BACKBLINK\n");
	
	// reset some backblink blinker stuff.
	ctr_bb_rearblink = 0;
	ctr_bb_rearblinkon = 0;
	ctr_bb_rearblinkoff = 0;
	ctr_bb_blinkgroup_on = 0;
	ctr_bb_blinkgroup_off = 0;
	
	// use auto or manual blink timing?
	if (sttg_bb_mode == 2 && (bb_color_r || bb_color_g || bb_color_b)) {
		// auto mode requested, and colors are set.
		
		// set auto blink pattern.
		pr_info("[TW/bb/bb_start] setting auto blink pattern based on color\n");
		bb_set_blinkpattern_autorgb(bb_color_r, bb_color_g, bb_color_b);
		
	} else {
		
		// use normal blink pattern.
		pr_info("[TW/bb/bb_start] setting user blink pattern\n");
		bb_set_blinkpattern_manual();
	}
	
	// start the blink timer.
	flg_bb_active = true;
	
	mod_timer(&timer_backblink,
			  jiffies + msecs_to_jiffies(300));
	
	// now the rear led is blinking. each blink will count up
	// until the threshold is met, then it will stop on its own.
	// on its last blink, it will turn the prox off.
	
	// however, the prox will turn itself off probably too soon,
	// so reset it to match roughly (bb_dutyon + bb_dutyoff) * bb_limit.
	// remember to subtract one bb_dutyoff, since the limit stops the instant
	// the last bb_dutyoff starts.
	
	tmp_set_disable_prox = ((sttg_bb_dutycycle_on * sttg_bb_limit) + (sttg_bb_dutycycle_off * (sttg_bb_limit - 1)));
	
	pr_info("[TW/bb/bb_start] backblink extending prox timeout to: %d ms\n", tmp_set_disable_prox);
	
	if (sttg_bb_blinkgroup_on_size && sttg_bb_limit > sttg_bb_blinkgroup_on_size) {
		// there is an on-blinkgroup we need to factor in.
		// make sure the limit is greater than the groupsize, otherwise there's no point.
		
		// for example
		// total limit: 10
		// blinkgroup delay: 500ms
		// blinkgroup size: 2
		// total limit / size = 10 / 2 = 5
		// subract one group, since it will be clipped: 5 - 1 = 4
		// 4 groups * delay = 4 * 500 = 2000ms extra needed
		// don't forget: each blinkgroup delay replaces a normal delay, so subtract those. -= 5 groups * dutyoff
		
		// calculate estimated amount of blinkgroups. since all we're doing is adding extended delays,
		// add to the existing value as calculated already.
		tmp_blinkgroups = (sttg_bb_limit / sttg_bb_blinkgroup_on_size);
		
		// calculate new estimated backblink timeout.
		tmp_set_disable_prox += ((tmp_blinkgroups * sttg_bb_blinkgroup_on_delay) - (tmp_blinkgroups * sttg_bb_dutycycle_on));
		
		pr_info("[TW/bb/bb_start] on-blinkgroup detected, recalculated extension to: %d ms\n", tmp_set_disable_prox);
	}
	
	if (sttg_bb_blinkgroup_off_size && sttg_bb_limit > sttg_bb_blinkgroup_off_size) {
		// there is an off-blinkgroup we need to factor in.
		// make sure the limit is greater than the groupsize, otherwise there's no point.
		
		// calculate estimated amount of blinkgroups. since all we're doing is adding extended delays,
		// add to the existing value as calculated already.
		tmp_blinkgroups = (sttg_bb_limit / sttg_bb_blinkgroup_off_size);
		
		// calculate new estimated backblink timeout.
		tmp_set_disable_prox += ((tmp_blinkgroups * sttg_bb_blinkgroup_off_delay) - (tmp_blinkgroups * sttg_bb_dutycycle_off));
		
		// this isn't needed, since on the final run it stops itself before it even thinks to do an extra off-blinkgroup.
		/*// if blinkgroup_off_size fits evenly into bb_limit, we know we will stop before
		 // the final off-blinkgroup, so subtract one.
		 if (sttg_bb_limit % sttg_bb_blinkgroup_off_size == 0) {
		 //
		 pr_info("[TW/ww/backblink] bb_limit: %d divided by off-blinkgroups: %d had no remainder\n", sttg_bb_limit, tmp_blinkgroups);
		 tmp_set_disable_prox -= sttg_bb_blinkgroup_off_delay;
		 } else {
		 pr_info("[TW/ww/backblink] bb_limit: %d divided by off-blinkgroups: %d had a remainder\n", sttg_bb_limit, tmp_blinkgroups);
		 }*/
		
		pr_info("[TW/bb/bb_start] off-blinkgroup detected, recalculated extension to: %d ms. there were %d estimated blinkgroups\n", tmp_set_disable_prox, tmp_blinkgroups);
	}
	
	ww_set_disable_prox(tmp_set_disable_prox);
}

static void bb_magcheck_off_work(struct work_struct * work_bb_magcheck_off)
{
	// turn off the mag sensor if need be.
	if (sttg_bb_magcheck) {
		// if magcheck is enabled, assume the sensor was on.
		
		// set this asap.
		flg_bb_magcheck = false;
		
		// turn off the sensor.
		pr_info("[TW/bb/magcheckoff] TURNING OFF - MAG\n");
		forceDisableSensor(2);
		
		// turn off the MCU.
		ssp_manual_suspend();
	}
	
	// was the magcheck successful?
	if ((sttg_bb_magcheck && (flg_bb_magpassed || !flg_ctr_bb_magchecks))
		|| !sttg_bb_magcheck) {
		// IF magcheck was called and completed
		//    OR magcheck was called and never run (sensor glitch)
		// IF magcheck was never called.
		// anyway, now bb has been validated, so it's time to blink.
		
		bb_start();
		
	} else {
		// the magcheck failed.
		// turn the prox off, and drop the wakelock.
		
		pr_info("[TW/bb/magcheckoff] TURNING OFF - PROX\n");
		forceDisableSensor(5);
		wake_unlock(&wavewake_wake_lock);
		pr_info("[TW/bb/magcheckoff] UNLOCKED wavewake_wake_lock\n");
		
		// anytime we prematurely turn the prox off, we run the risk
		// that prox_near will never be reset, so make sure it's false.
		prox_near = false;
	}
}

static void timerhandler_backblink()
{
	static unsigned int tmp_sttg_bb_dutycycle;
	
	// sanity checks.
	if (!use_sttg_bb_dutycycle_on) {
		use_sttg_bb_dutycycle_on = 1;
	}
	
	if (!use_sttg_bb_dutycycle_off) {
		use_sttg_bb_dutycycle_off = 1;
	}
	
	if (!use_sttg_bb_blinkgroup_on_delay) {
		use_sttg_bb_blinkgroup_on_delay = 1;
	}
	
	if (!use_sttg_bb_blinkgroup_off_delay) {
		use_sttg_bb_blinkgroup_off_delay = 1;
	}
	
	// increment total blink counter.
	ctr_bb_rearblink++;
	
	// check to see if we will be turning it ON or OFF.
	if (ctr_bb_rearblink % 2) {
		// ctr_bb_rearblink is odd, led is about to be turned ON.
		
		// increment total blinks on, and update intended timer duration.
		ctr_bb_rearblinkon++;
		tmp_sttg_bb_dutycycle = use_sttg_bb_dutycycle_on;
		
		// increment blinks on. this variable gets reset when the on-blinkgroup size threshold is met.
		ctr_bb_blinkgroup_on++;
		
		// toggle rearled at brightness level 1.
		toggleRearLED(sttg_bb_brightness);
		
		pr_info("[TW/bb/backblinkhandler] toggled rear led ON. total blinks: %i, blinks on: %i, blinks off: %i\n",
				ctr_bb_rearblink, ctr_bb_rearblinkon, ctr_bb_rearblinkoff);
		
		// blink group-on gets calculated on the ON cycle.
		if (use_sttg_bb_blinkgroup_on_size && ctr_bb_blinkgroup_on >= use_sttg_bb_blinkgroup_on_size) {
			// the user wants to divide the blinks into groups.
			// if we are here, we have met the threshold to inject a delay for ON.
			
			// the blinkgroup mechanism has its own counter.
			// it increments on the rising side until the group size is met,
			// then injects a delay of sttg_bb_blinkgroup_on_delay
			// into the tmp_dutycycle used for mod_timer() on the falling side,
			// then resets itself.
			
			// inject the blinkgroup delay.
			tmp_sttg_bb_dutycycle = use_sttg_bb_blinkgroup_on_delay;
			
			pr_info("[TW/bb/backblinkhandler] blinkgroup ON threshold reached. ctr_bb_blinkgroup_on: %d, sttg_bb_blinkgroup_on_size: %d, new delay: %d\n",
					ctr_bb_blinkgroup_on, use_sttg_bb_blinkgroup_on_size, sttg_bb_blinkgroup_on_delay);
			
			// reset the counter.
			ctr_bb_blinkgroup_on = 0;
		}
		
	} else {
		// otherwise increment the rearblinkoff counter, led is about to be turned OFF.
		
		// increment total blinks off, and update intended timer duration.
		ctr_bb_rearblinkoff++;
		tmp_sttg_bb_dutycycle = use_sttg_bb_dutycycle_off;
		
		// increment blinks on. this variable gets reset when the on-blinkgroup size threshold is met.
		ctr_bb_blinkgroup_off++;
		
		// things are different when turning OFF because on the last OFF command, we
		// want to manually turn the led OFF instead of just toggling it.
		
		if (ctr_bb_rearblinkoff >= sttg_bb_limit) {
			// we have blinked enough, turn off completely.
			
			pr_info("[TW/bb/backblinkhandler] backblink limit reached, turned rear led OFF. total blinks: %i, blinks on: %i, blinks off: %i, limit: %d\n",
					ctr_bb_rearblink, ctr_bb_rearblinkon, ctr_bb_rearblinkoff, sttg_bb_limit);
			
			// reset the test flag, just in case this entire run was a test.
			flg_bb_testmode = false;
			
			controlRearLED(0);
			
			// disable all this. instead, let the scheduled work handle shutting off the prox.
			/*pr_info("[TW/backblinkhandler] turning off prox\n");
			forceDisableSensor(5);
			wake_unlock(&wavewake_wake_lock);
			pr_info("[TW/backblinkhandler/ww] unlocked wavewake_wake_lock\n");
			prox_near = false;*/
			
			// exit before we recycle the timer.
			return;
			
		} else {
			// we still have at least another cycle to go, so toggle it off.
			
			// blink group gets calculated on the OFF cycle.
			if (use_sttg_bb_blinkgroup_off_size && ctr_bb_blinkgroup_off >= use_sttg_bb_blinkgroup_off_size) {
				// the user wants to divide the blinks into groups.
				// if we are here, we have met the threshold to inject a delay for OFF.
				
				// the blinkgroup mechanism has its own counter.
				// it increments on the rising side until the group size is met,
				// then injects a delay of sttg_bb_blinkgroup_off_delay
				// into the tmp_dutycycle used for mod_timer() on the falling side,
				// then resets itself.
				
				// inject the blinkgroup delay.
				tmp_sttg_bb_dutycycle = use_sttg_bb_blinkgroup_off_delay;
				
				pr_info("[TW/bb/backblinkhandler] blinkgroup OFF threshold reached. ctr_bb_blinkgroup_off: %d, sttg_bb_blinkgroup_off_size: %d, new delay: %d\n",
						ctr_bb_blinkgroup_off, use_sttg_bb_blinkgroup_off_size, use_sttg_bb_blinkgroup_off_delay);
				
				// reset the counter.
				ctr_bb_blinkgroup_off = 0;
			}
			
			pr_info("[TW/bb/backblinkhandler] toggled rear led OFF. total blinks: %i, blinks on: %i, blinks off: %i\n",
					ctr_bb_rearblink, ctr_bb_rearblinkon, ctr_bb_rearblinkoff);
			
			// toggle rearled at brightness level 1.
			toggleRearLED(sttg_bb_brightness);
		}
	}
	
	pr_info("[TW/bb/backblinkhandler] recycling timer to start again in %d ms\n", tmp_sttg_bb_dutycycle);
	
	// recycle timer.
	mod_timer(&timer_backblink,
			  jiffies + msecs_to_jiffies(tmp_sttg_bb_dutycycle));
}

void proximity_detected(void)
{
	struct timeval time_now;
	unsigned int time_since_ledtriggered;
	unsigned int time_since_manualtrigger;
	
	// remember to reset prox_near if exiting early, or else tsp will
	// lock itself out because it thinks there is always something there.
	prox_near = true;
	
#ifdef DEBUG_PRINT
	pr_info("[TOUCHWAKE] Proximity enabled\n");
#endif
	
	// do the touchwake waveoff check first.
	if (flg_tw_prox_on) {
		
		pr_info("[TW/proximity_detected] TURNING OFF - PROX - touchwake is pressing power\n");
		forceDisableSensor(5);
		
		// reset some stuff.
		flg_tw_prox_on = false;
		prox_near = false;
		
		// turn on.
		pr_info("[TW/proximity_detected] PRESSPOWER\n");
		touch_press(sttg_pu_allow_tw);
		
		// we're done.
		return;
	}
	
	do_gettimeofday(&time_now);
	
	// calculate time since led hook triggered prox on.
	time_since_ledtriggered = (time_now.tv_sec - time_ledtriggered.tv_sec) * MSEC_PER_SEC +
								(time_now.tv_usec - time_ledtriggered.tv_usec) / USEC_PER_MSEC;
	
	// calculate time since manual hook triggered prox on.
	time_since_manualtrigger = (time_now.tv_sec - time_ww_last_manualtrigger.tv_sec) * MSEC_PER_SEC +
								(time_now.tv_usec - time_ww_last_manualtrigger.tv_usec) / USEC_PER_MSEC;
	
	// we need to know if the sensor was already blocked when it came on,
	// or did something move in the way. don't forget to check for manual time,
	// in case the led hook isn't being used.
	if (time_since_ledtriggered > 400 && time_since_manualtrigger > 400) {
		// if the sensor hasn't said it is blocked by now, assume it's clear.
		
		if (sttg_ww_mode > 0) {
			
			pr_info("[TW/SSP/ww] ww is enabled. time since LED: %d\n", time_since_ledtriggered);
			
			if (flg_ww_prox_on) {
				
				ctr_ww_prox_hits--;
				pr_info("[TW/SSP/ww] CHECKING - flg_ww_prox_on and ctr is: %d\n", ctr_ww_prox_hits);
				
				if (ctr_ww_prox_hits == 0 || (sttg_ww_waveoff && ctr_ww_prox_hits == -1)) {
					// we want this to run if we hit the trigger (0)
					// AND if waveoff is set and we've already hit the trigger (-1)
					
					pr_info("[TW/SSP/ww] ACTIVATED!\n");
					
					do_gettimeofday(&time_ww_last_screenon);
					
					if (ctr_ww_prox_hits == -1) {
						// only way we can be here is if sttg_ww_waveoff was true too.
						
						if (sttg_ww_dismissled == 2) {
							pr_info("[TW/SSP/ww] LEDS - DISABLE (2)\n");
							an30259a_ledsoff();
						}
						
						do_gettimeofday(&time_ww_last_screenoff);
						//flg_ww_prox_on = false;
						pr_info("[TW/SSP/ww] PROX - DISABLE - waveoff is pressing power\n");
						forceDisableSensor(5);
						wake_unlock(&wavewake_wake_lock);
						pr_info("[TW/SSP/ww] unlocked wavewake_wake_lock\n");
						ctr_ww_prox_hits = sttg_ww_mode;
						
					} else if (!sttg_ww_waveoff) {
						// waveoff isn't set, so this is our first and last run.
						// turn off the prox now, since we won't be needing it.
						
						//flg_ww_prox_on = false;
						pr_info("[TW/SSP/ww] PROX - DISABLE - no wave off so we're done\n");
						forceDisableSensor(5);
						wake_unlock(&wavewake_wake_lock);
						pr_info("[TW/SSP/ww] unlocked wavewake_wake_lock\n");
						ctr_ww_prox_hits = sttg_ww_mode;
					}
					
					if (sttg_ww_dismissled == 1) {
						pr_info("[TW/SSP/ww] LEDS - DISABLE (1)\n");
						an30259a_ledsoff();
					}
					
					pr_info("[TW/SSP/ww] ACTIVATED - pressing power!\n");
					press_power();
					
				}
			} else {
				
				pr_info("[TW/SSP/ww] flg_ww_prox_on: false\n");
			}
			
		}
		
	} else {
		// if we're getting a prox detected event this soon,
		// it's probably because there is something there, like a pocket or a desk.
		
		// reset some stuff.
		ctr_ww_prox_hits = sttg_ww_mode;
		flg_ww_userspace_noti = false;
		
		if (flg_ww_prox_on) {
			// only mess with the prox sensor if we turned it on.
			
			if (sttg_bb_mode) {
				// if backblink is enabled, this is our cue to do something.
				// the led has been triggered and prox went on, and found something.
				// let's assume the device is face-down on a desk.
				
				// reset bb stuff.
				flg_bb_magcheck = true;
				flg_ctr_bb_magchecks = 0;
				flg_bb_magpassed = false;
				
				// does the user want a magcheck?
				if (sttg_bb_magcheck) {
					
					pr_info("[TW/ww/bb/proximity_detected] holding out prox, turning on ssp & mag sensor, and scheduling work to check it in %d ms\n", 500);
					
					// power up mcu.
					ssp_manual_resume();
					
					// extend prox timeout to give mag sensor a chance to work.
					ww_set_disable_prox(550);
					
					// turn on mag sensor.
					forceEnableSensor(2, false);
					
					// give it a little time, then schedule work to see
					// if the magcheck passed or failed.
					schedule_delayed_work(&work_bb_magcheck_off, msecs_to_jiffies(500));
					
					// we're done.
					return;
				}
				
				pr_info("[TW/ww/bb/proximity_detected] magcheck not enabled\n");
				
				// no magcheck, so start now.
				bb_start();
				
			} else {
				// prox was blocked, and backblink is disabled, so assume it's in a pocket and turn prox off.
				
				pr_info("[TW/ww/proximity_detected] TURNING OFF - PROX - time since led hook triggered prox: %d\n", time_since_ledtriggered);
				forceDisableSensor(5);
				wake_unlock(&wavewake_wake_lock);
				pr_info("[TW/ww/proximity_detected] UNLOCKED wavewake_wake_lock\n");
				prox_near = false;
			}
		}
	}

	return;
}
EXPORT_SYMBOL(proximity_detected);

void proximity_off(void)
{
	prox_near = false;
	
	if (flg_bb_active) {
		// if backblink is active, then now it looks like someone lifted the device.
		// stop the blinking, and turn prox off.
		
		pr_info("[TW/bb/proximity_off] backblink is active but needs to be turned off. disabling timer, rear led, and prox now\n");
		
		// turn off backblink.
		flg_bb_active = false;
		del_timer(&timer_backblink);
		
		// turn off rear led.
		controlRearLED(0);
		
		// todo: make prox off into a function.
		pr_info("[TW/ww/bb/proximity_off] TURNING OFF - PROX\n");
		forceDisableSensor(5);
		wake_unlock(&wavewake_wake_lock);
		pr_info("[TW/ww/bb/proximity_off] UNLOCKED wavewake_wake_lock\n");
		
		if (!flg_screen_on && sttg_bb_poweron_clearedobstacle) {
			// since the prox block has been cleared (presumably
			// the user has picked the device up), the user
			// has the option to turn the power on now.
			
			pr_info("[TW/ww/bb/proximity_off] PRESSPOWER - obstacle cleared\n");
			press_power();
		}
	}
	
#ifdef DEBUG_PRINT
	pr_info("[TOUCHWAKE] Proximity disabled\n");
#endif

	return;
}
EXPORT_SYMBOL(proximity_off);

void powerkey_pressed(void)
{
#ifdef DEBUG_PRINT
	pr_info("[TOUCHWAKE] Powerkey pressed\n");
#endif
    
	do_gettimeofday(&last_powerkeypress);
	timed_out = false; // Yank555 : consider user is indeed turning off the device

	return;
}
EXPORT_SYMBOL(powerkey_pressed);

void powerkey_released(void)
{
	struct timeval now;
	int time_pressed;
    
#ifdef DEBUG_PRINT
	pr_info("[TOUCHWAKE] Powerkey released\n");
#endif

	do_gettimeofday(&now);

	time_pressed = (now.tv_sec - last_powerkeypress.tv_sec) * MSEC_PER_SEC +
	(now.tv_usec - last_powerkeypress.tv_usec) / USEC_PER_MSEC;

	if (unlikely(time_pressed > TIME_LONGPRESS || device_suspended)) {
		timed_out = true; // Yank555 : OK, user is not turning off device, but long-pressing Powerkey, or turing on device, so back to normal
#ifdef DEBUG_PRINT
		pr_info("[TOUCHWAKE] Powerkey longpress detected released\n");
#endif
#ifdef DEBUG_PRINT
	} else {
		pr_info("[TOUCHWAKE] Device being turned off\n");
        proximity_off();
#endif
	}

	return;
}
EXPORT_SYMBOL(powerkey_released);

void touch_press(bool forceunlock)
{
#ifdef DEBUG_PRINT
	pr_info("[TOUCHWAKE] Touch press detected, timed_out: %d\n", timed_out);
#endif
    
    flg_touchwake_pressed = true;
    
	if (unlikely(device_suspended && touchwake_enabled && !prox_near && mutex_trylock(&lock))) {
		
		// what if someone turns the screen on, doesn't unlock, lets it time out,
		// lets tw start, then taps it. that will bypass pu because we're forcing
		// the lock off here. we need to know the lock status when the screen went
		// off so we can know not to restore it if it was never unlocked.
		
		if (forceunlock && timed_out && flg_touchwake_allowforceunlock && !flg_pu_locktsp_saved_beforesuspend && !flg_touchwake_swipe_only) {
			// drop the pu lock because we were unlocked when the screen went off.
			
			pu_clearAll();
			
			// what about the screenblank?
			if (flg_pu_blackout) {
				
				// unblackout the screen.
				pr_info("[TW/touch_press/pu] unblacking out screen\n");
				flg_pu_blackout = false;
				mdnie_toggle_blackout();
			}
		}
		
		schedule_work(&presspower_work);
	}

	return;
}
EXPORT_SYMBOL(touch_press);

void set_powerkeydev(struct input_dev * input_device)
{
#ifdef DEBUG_PRINT
	pr_info("[TOUCHWAKE] Powerkey device set to: %p\n", input_device);
#endif

	powerkey_device = input_device;

	return;
}
EXPORT_SYMBOL(set_powerkeydev);

bool device_is_suspended(void)
{
	return device_suspended;
}
EXPORT_SYMBOL(device_is_suspended);

void register_touchwake_implementation(struct touchwake_implementation * imp)
{
    touchwake_imp = imp;

    return;
}
EXPORT_SYMBOL(register_touchwake_implementation);
static int __init touchwake_control_init(void)
{
	int ret;

	pr_info("%s misc_register(%s)\n", __FUNCTION__, touchwake_device.name);
	ret = misc_register(&touchwake_device);

	if (ret) {
		pr_err("%s misc_register(%s) fail\n", __FUNCTION__, touchwake_device.name);

		return 1;
	}

	if (sysfs_create_group(&touchwake_device.this_device->kobj, &touchwake_notification_group) < 0) {
		pr_err("%s sysfs_create_group fail\n", __FUNCTION__);
		pr_err("Failed to create sysfs group for device (%s)!\n", touchwake_device.name);
	}

	register_early_suspend(&touchwake_suspend_data);

	wake_lock_init(&touchwake_wake_lock, WAKE_LOCK_SUSPEND, "touchwake_wake");
	wake_lock_init(&knockwake_wake_lock, WAKE_LOCK_SUSPEND, "knockwake_wake");

	do_gettimeofday(&last_powerkeypress);
	
	setup_timer(&timer_backblink, timerhandler_backblink, 0);

	return 0;
}

device_initcall(touchwake_control_init);
