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

bool touchwake_enabled = false;
static bool touch_disabled = false;
static bool device_suspended = false;
static bool timed_out = true;
bool prox_near = false;
static bool ignore_once = false;
bool flg_touchwake_active = false;
unsigned int touchoff_delay = 15000;
bool sttg_touchwake_swipe_only = false;
bool flg_touchwake_pressed = false;
bool flg_touchwake_swipe_only = false;
bool sttg_touchwake_persistent = false;
bool sttg_touchwake_persistent_wakelock = false;
bool sttg_touchwake_ignoretkeys = true;
bool sttg_touchwake_ignoregestures = false;
bool sttg_touchwake_ignorepowerkey = false;
bool sttg_touchwake_force_timedout_tapwake = true;
unsigned int sttg_touchwake_delay_afterpoweroff = 0;
unsigned int sttg_touchwake_swipe_y_drift_tolerance = 60;
unsigned int sttg_touchwake_swipe_max_pressure = 35;
unsigned int sttg_touchwake_swipe_finger_angle_tolerance = 30;
bool sttg_touchwake_swipe_fast_trigger = true;
bool sttg_touchwake_swipe_arc_rh = true;
bool sttg_touchwake_swipe_arc_lh = false;
bool sttg_touchwake_longpressoff_enabled = true;
unsigned int sttg_touchwake_longpressoff_timeout = 30000;
unsigned int sttg_touchwake_longpressoff_duration = 300;
unsigned int sttg_touchwake_longpressoff_min_pressure = 20;
unsigned int sttg_touchwake_longpressoff_xy_drift_tolerance = 20;

static void touchwake_reset_swipeonly(struct work_struct * touchwake_reset_swipeonly_work);
static DECLARE_DELAYED_WORK(touchwake_reset_swipeonly_work, touchwake_reset_swipeonly);
static void touchwake_touchoff(struct work_struct * touchoff_work);
static DECLARE_DELAYED_WORK(touchoff_work, touchwake_touchoff);
static void press_powerkey(struct work_struct * presspower_work);
static DECLARE_WORK(presspower_work, press_powerkey);
static DEFINE_MUTEX(lock);

static struct touchwake_implementation * touchwake_imp = NULL;
struct input_dev * powerkey_device;
static struct wake_lock touchwake_wake_lock;
static struct timeval last_powerkeypress;

#define TOUCHWAKE_VERSION "1.1"
#define TIME_LONGPRESS 500
#define POWERPRESS_DELAY 0
#define POWERPRESS_TIMEOUT 0

#define DEBUG_PRINT

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
        
        if (timed_out && sttg_touchwake_swipe_only && sttg_touchwake_force_timedout_tapwake) {
            // the user might want to hold a persistent lock and use slide2wake, but still be able to
            // just single tap to wake up after a timeout event. so we have to schedule delayed work
            // to reset sttg_touchwake_swipe_only back to the real value.
            
            pr_info("[TOUCHWAKE] Early suspend - swipeonly and force_tap set. scheduling work...\n");
            
            //pr_info("[Touchwake] wakelock needed for tap check.\n");
            //wake_lock(&touchwake_wake_lock); /* TODO: disabled until deepsleep wake works */
            
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
	//flush_scheduled_work();

	wake_unlock(&touchwake_wake_lock);

	//if (touch_disabled)
		touchwake_enable_touch();

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
	touchwake_disable_touch();
	touchwake_droplock();

	return;
}

static void touchwake_reset_swipeonly(struct work_struct * touchwake_reset_swipeonly_work)
{
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
	return sprintf(buf, "%u\n", touchoff_delay / 1000);
}

static ssize_t touchwake_delay_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;

	if(sscanf(buf, "%u\n", &data) == 1) {
		touchoff_delay = data * 1000;
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
		sttg_touchwake_persistent = true;
		pr_info("TOUCHWAKE enable_persistent has been set\n");
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

static ssize_t touchwake_delay_afterpoweroff_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", sttg_touchwake_delay_afterpoweroff / 1000);
}

static ssize_t touchwake_delay_afterpoweroff_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int data;
	
	if(sscanf(buf, "%u\n", &data) == 1) {
		sttg_touchwake_delay_afterpoweroff = data * 1000;
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
		pr_info("TOUCHWAKE longpress_min_pressure has been set to %d\n", sttg_touchwake_longpressoff_min_pressure);
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
static DEVICE_ATTR(delay_afterpoweroff, S_IRUGO | S_IWUGO, touchwake_delay_afterpoweroff_read, touchwake_delay_afterpoweroff_write);
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
    &dev_attr_delay_afterpoweroff.attr,
    &dev_attr_force_timedout_tapwake.attr,
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
	prox_near = true;
#ifdef DEBUG_PRINT
	pr_info("[TOUCHWAKE] Proximity enabled\n");
#endif

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

	do_gettimeofday(&last_powerkeypress);

	return 0;
}

device_initcall(touchwake_control_init);
