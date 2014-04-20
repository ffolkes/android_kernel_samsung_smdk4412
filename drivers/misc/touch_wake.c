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

extern void touchscreen_enable(void);
extern void touchscreen_disable(void);
extern unsigned int flg_ctr_cpuboost;
extern unsigned int flg_ctr_cpuboost_mid;
extern struct wake_lock wavewake_wake_lock;
static struct wake_lock knockwake_wake_lock;
extern unsigned int ctr_knocks;
extern int ctr_kw_gyro_skip;

bool flg_kw_gyro_on = false;
bool flg_tw_prox_on = false;

bool touchwake_enabled = false;
static bool touch_disabled = false;
static bool device_suspended = false;
static bool timed_out = true;
bool prox_near = false;
bool ignore_once = false;
bool flg_touchwake_active = false;
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
bool flg_ww_trigger_noti = false;

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
        
        if (timed_out && sttg_touchwake_swipe_only && sttg_touchwake_force_timedout_tapwake) {
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
	flg_ww_trigger_noti = false;
	flg_ctr_cpuboost_mid = 0;
	//flush_scheduled_work();
	
	touchwake_enable_touch();
	
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
	
	// stop the booster when the screen comes on.
	// this is just piggybacked in touch_wake.c for my convenience.
	flg_ctr_cpuboost = 0;
	
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
	
	if (data == 100) {
		pr_info("SSPFF turning off...%u\n", data);
		//forceDisableSensor(5);
		return size;
	}
	
	if (data == 10) {
		pr_info("SSPFF turning on...%u\n", data);
		//forceEnableSensor(5);
	}
	
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
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		
		if (flg_ctr_cpuboost < 5) {
			// boost cpu for a few samples.
			flg_ctr_cpuboost = 5;
		}
		
		flg_ww_trigger_noti = true;
		pr_info("[TW/ww] userspace has set flg_ww_trigger_noti\n");
		
		if (sttg_ww_mode > 0 && !flg_screen_on && !flg_ww_prox_on) {
			
			if (sttg_ww_linger > 0) {
				// boost cpu. multiply by 5, since sampling rate is probably ~200ms.
				flg_ctr_cpuboost = 5 * (sttg_ww_linger / 1000);
			} else {
				// boost cpu. if sttg_ww_linger is set to unlimited it will be 0, so we cannot use it.
				flg_ctr_cpuboost = 50;
			}
			
			pr_info("[SSP/ww] saving manual trigger time\n");
			do_gettimeofday(&time_ww_last_manualtrigger);
			
			pr_info("[SSP/ww] locking wavewake_wake_lock\n");
			wake_lock(&wavewake_wake_lock);
			pr_info("[TW/ssp/ww] immediately starting PROX\n");
			forceEnableSensor(5, false);
			//flg_ww_prox_on = true;
			ww_set_disable_prox(1);
		}
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
	return sprintf(buf, "%u\n", sttg_kw_resolution);
}

static ssize_t kw_resolution_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	int64_t data;
	
	if (strict_strtoll(buf, 10, &data) < 0)
		return -1;
	
	sttg_kw_resolution = data;
	forceChangeDelay(1, data);
	pr_info("[TW/kw] sttg_kw_resolution has been set to %u %lld\n", sttg_kw_resolution, sttg_kw_resolution);
	
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

void proximity_detected(void)
{
	struct timeval time_now;
	unsigned int time_since_ledwenton;
	unsigned int time_since_manualtrigger;
	
	prox_near = true;
#ifdef DEBUG_PRINT
	pr_info("[TOUCHWAKE] Proximity enabled\n");
#endif
	
	do_gettimeofday(&time_now);
	
	time_since_ledwenton = (time_now.tv_sec - time_ledwenton.tv_sec) * MSEC_PER_SEC +
							(time_now.tv_usec - time_ledwenton.tv_usec) / USEC_PER_MSEC;
	
	time_since_manualtrigger = (time_now.tv_sec - time_ww_last_manualtrigger.tv_sec) * MSEC_PER_SEC +
							(time_now.tv_usec - time_ww_last_manualtrigger.tv_usec) / USEC_PER_MSEC;
	
	if (flg_tw_prox_on) {
		pr_info("[TW/SSP] ACTIVATED - pressing power!\n");
		pr_info("[TW/SSP] PROX - DISABLE - touchwake is pressing power\n");
		flg_tw_prox_on = false;
		prox_near = false;
		forceDisableSensor(5);
		press_power();
	}
	
	if (sttg_ww_mode > 0 && time_since_ledwenton > 250 && time_since_manualtrigger > 250) {
		
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
	} else {
		// if we're getting a prox detected even this soon,
		// it's probably because there is something there, like a pocket.
		// don't risk it by leaving it on, and turn WW off.
		if (flg_ww_prox_on) {
			//flg_ww_prox_on = false;
			pr_info("[TW/SSP/ww] disable prox for this entire event because LED was called within 500ms\n");
			forceDisableSensor(5);
			wake_unlock(&wavewake_wake_lock);
			pr_info("[SSP/ww] unlocked wavewake_wake_lock\n");
			ctr_ww_prox_hits = sttg_ww_mode;
			// stop the cpu boost, we don't need it anymore.
			flg_ctr_cpuboost = 0;
		}
	}
	
	

	return;
}
EXPORT_SYMBOL(proximity_detected);

void proximity_off(void)
{
	prox_near = false;
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

void touch_press(void)
{
#ifdef DEBUG_PRINT
	pr_info("[TOUCHWAKE] Touch press detected\n");
#endif
    
    flg_touchwake_pressed = true;
    
	if (unlikely(device_suspended && touchwake_enabled && !prox_near && mutex_trylock(&lock)))
		schedule_work(&presspower_work);

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

	return 0;
}

device_initcall(touchwake_control_init);
