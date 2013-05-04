/* drivers/misc/touch_wake.c
 *
 * Copyright 2011  Ezekeel
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
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
static bool prox_near = false;
static bool ignore_once = false;
bool flg_touchwake_active = false;
unsigned int touchoff_delay = 45000;

static void touchwake_touchoff(struct work_struct * touchoff_work);
static DECLARE_DELAYED_WORK(touchoff_work, touchwake_touchoff);
static void press_powerkey(struct work_struct * presspower_work);
static DECLARE_WORK(presspower_work, press_powerkey);
static DEFINE_MUTEX(lock);

static struct touchwake_implementation * touchwake_imp = NULL;
static struct input_dev * powerkey_device;
static struct wake_lock touchwake_wake_lock;
static struct timeval last_powerkeypress;

#define TOUCHWAKE_VERSION 1
#define TIME_LONGPRESS 500
#define POWERPRESS_DELAY 100
#define POWERPRESS_TIMEOUT 1000

static void touchwake_disable_touch(void)
{
	pr_info("[Touchwake] disable touch controls\n");
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
	pr_info("[Touchwake] enable touch controls\n");
	touch_disabled = false;
	
	touchscreen_enable();

	if (touchwake_imp)
	{
	    touchwake_imp->enable();
	}
	
	return;
}

static void touchwake_early_suspend(struct early_suspend * h)
{
	device_suspended = true;
	if (touchwake_enabled && !ignore_once) {
		if (touchoff_delay > 0)	{
			if (timed_out) {
				flg_touchwake_active = true;
				wake_lock(&touchwake_wake_lock);
				schedule_delayed_work(&touchoff_work, msecs_to_jiffies(touchoff_delay));
			} else {
                flg_touchwake_active = false;
				touchwake_disable_touch();
			}
		} else {
			wake_lock(&touchwake_wake_lock);
		}
	} else {
		flg_touchwake_active = false;
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
	// important that device_suspended gets set as soon as possible. 
	device_suspended = false;
	cancel_delayed_work_sync(&touchoff_work);
	//flush_scheduled_work();

	wake_unlock(&touchwake_wake_lock);

	if (touch_disabled)
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
	wake_unlock(&touchwake_wake_lock);

	return;
}

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
			pr_info("%s: TOUCHWAKE function enabled\n", __FUNCTION__);
			touchwake_enabled = true;
		} else if (data == 0) {
			pr_info("%s: TOUCHWAKE function disabled\n", __FUNCTION__);
			touchwake_enabled = false;
		} else {
			pr_info("%s: invalid input range %u\n", __FUNCTION__, data);
		}
	} else 	{
		pr_info("%s: invalid input\n", __FUNCTION__);
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
		pr_info("TOUCHWAKE delay set to %u\n", touchoff_delay); 
	} else 	{
		pr_info("%s: invalid input\n", __FUNCTION__);
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

int get_touchoff_delay()
{   
	return touchoff_delay;
}
EXPORT_SYMBOL(get_touchoff_delay);

static ssize_t touchwake_version(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", TOUCHWAKE_VERSION);
}

static DEVICE_ATTR(enabled, S_IRUGO | S_IWUGO, touchwake_status_read, touchwake_status_write);
static DEVICE_ATTR(delay, S_IRUGO | S_IWUGO, touchwake_delay_read, touchwake_delay_write);
static DEVICE_ATTR(ignore_once, S_IRUGO | S_IWUGO, touchwake_ignore_once_read, touchwake_ignore_once_write);
static DEVICE_ATTR(version, S_IRUGO , touchwake_version, NULL);

static struct attribute *touchwake_notification_attributes[] =
{
	&dev_attr_enabled.attr,
	&dev_attr_delay.attr,
	&dev_attr_ignore_once.attr,
	&dev_attr_version.attr,
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
	timed_out = false;
	prox_near = true;
	return;
}
EXPORT_SYMBOL(proximity_detected);

void proximity_off(void)
{   
	timed_out = true;
	prox_near = false;

	return;
}
EXPORT_SYMBOL(proximity_off);

void powerkey_pressed(void)
{
	do_gettimeofday(&last_powerkeypress);

	return;
}
EXPORT_SYMBOL(powerkey_pressed);

void powerkey_released(void)
{
	struct timeval now;
	int time_pressed;

	do_gettimeofday(&now);

	time_pressed = (now.tv_sec - last_powerkeypress.tv_sec) * MSEC_PER_SEC +
	(now.tv_usec - last_powerkeypress.tv_usec) / USEC_PER_MSEC;

	if (time_pressed < TIME_LONGPRESS)
		timed_out = false;

	return;
}
EXPORT_SYMBOL(powerkey_released);

void touch_press(void)
{
		
	if (device_suspended && touchwake_enabled && !prox_near && mutex_trylock(&lock)){
		
		printk("[Touchwake] valid touch_press() received\n");
		schedule_work(&presspower_work);
		
	}

	return;
}
EXPORT_SYMBOL(touch_press);

void set_powerkeydev(struct input_dev * input_device)
{   
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
