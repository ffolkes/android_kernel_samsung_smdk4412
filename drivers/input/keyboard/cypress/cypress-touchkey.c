/*
 * Driver for keys on GPIO lines capable of generating interrupts.
 *
 * Copyright 2005 Phil Blundell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <mach/regs-gpio.h>
#include <plat/gpio-cfg.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/earlysuspend.h>
#include <linux/io.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>

#include "issp_extern.h"
#include "cypress-touchkey.h"

#ifdef CONFIG_TOUCH_WAKE
#include <linux/touch_wake.h>
#endif

#ifdef CONFIG_TOUCHSCREEN_ATMEL_MXT540E
#include <linux/i2c/mxt540e.h>
#else
#include <linux/i2c/mxt224_u1.h>
#endif
#include <linux/i2c/touchkey_i2c.h>

#include <linux/wacom_i2c.h>

bool flg_touchkey_pressed = false;
bool flg_touchkey_was_pressed = false;
bool flg_cypress_suspended = false;

static bool flg_skip_next = false;

static unsigned int tk_key_keycode = 0;
static unsigned int sttg_tk_key1_key_code = 0;
static unsigned int sttg_tk_key2_key_code = 0;

// tk dt (double tap)
static unsigned int sttg_tk_mt_key1_mode = 0; // 0 = off, 1+ = on & number of presses required
static unsigned int sttg_tk_mt_key2_mode = 0; // 0 = off, 1+ = on & number of presses required
static unsigned int sttg_tk_mt_minimum_between = 100;
static unsigned int sttg_tk_mt_maximum_between = 500;
static unsigned int tk_mt_last_keycode_type = 0;
static int ctr_tk_mt_presses = 0;
static struct timeval time_tk_lastpress;
static unsigned int time_since_tk_mt_lastpress = 0;

extern bool epen_is_active;
extern int touch_is_pressed;
extern void press_button(int keycode, bool delayed, bool force, bool elastic, bool powerfirst);
extern int flg_ctr_cpuboost;
extern unsigned int sttg_boost_button_cycles;

// Yank555.lu : Add cleartext status settings for h/w key LED lightup on touchscreen touch
#define TOUCHKEY_LED_DISABLED	0
#define TOUCHKEY_LED_ENABLED	1

// Yank555.lu : Add cleartext status settings for kernel / ROM handling h/w key LED
#define TOUCHKEY_LED_ROM	1
#define TOUCHKEY_LED_KERNEL	0

// Yank555.lu : Add cleartext status settings for h/w key pressed
#define TOUCHKEY_HW_TIMEDOUT	0
#define TOUCHKEY_HW_PRESSED	1

/* M0 Touchkey temporary setting */

#if defined(CONFIG_MACH_M0) || defined(CONFIG_MACH_M3)
#define CONFIG_MACH_Q1_BD
#elif defined(CONFIG_MACH_C1) && !defined(CONFIG_TARGET_LOCALE_KOR)
#define CONFIG_MACH_Q1_BD
#elif defined(CONFIG_MACH_C1) && defined(CONFIG_TARGET_LOCALE_KOR)
/* C1 KOR doesn't use Q1_BD */
#endif

#if defined(CONFIG_TARGET_LOCALE_NAATT_TEMP)
/* Temp Fix NAGSM_SEL_ANDROID_MOHAMMAD_ANSARI_20111224*/
#define CONFIG_TARGET_LOCALE_NAATT
#endif

// ff - galaxy note 2 (L00) uses TK_USE_2KEY_TYPE_M0

static int touchkey_keycode[] = { 0,
#if defined(TK_USE_4KEY_TYPE_ATT)
	KEY_MENU, KEY_ENTER, KEY_BACK, KEY_END,

#elif defined(TK_USE_4KEY_TYPE_NA)
	KEY_SEARCH, KEY_BACK, KEY_HOMEPAGE, KEY_MENU,

#elif defined(TK_USE_2KEY_TYPE_M0)
	KEY_BACK, KEY_MENU, KEY_FIND, KEY_SEARCH, KEY_FINANCE, KEY_F16, KEY_F17, KEY_CAMERA, KEY_RIGHTBRACE, KEY_ANDROIDSEARCH, KEY_HOME, KEY_POWER,
	KEY_F19, KEY_F20, KEY_F21, KEY_F22, KEY_F23, KEY_F24, KEY_F7, KEY_ZENKAKUHANKAKU, KEY_KATAKANAHIRAGANA,

#else
	KEY_MENU, KEY_BACK,
#endif
};
static const int touchkey_count = sizeof(touchkey_keycode) / sizeof(int);

struct touchkey_i2c *tkey_i2c_local;
struct timer_list touch_led_timer;
int touch_led_timeout = 3; // timeout for the touchkey backlight in secs
int touch_led_disabled = 0; // 1= force disable the touchkey backlight
int touch_led_on_screen_touch	= TOUCHKEY_LED_DISABLED;	// Yank555.lu : Light up h/w key on touchscreen touch by default
int touchkey_pressed		= TOUCHKEY_HW_TIMEDOUT;	// Yank555.lu : Consider h/w keys as not pressed on start
int touch_led_handling		= TOUCHKEY_LED_ROM;	// Yank555.lu : Consider h/w keys handled by ROM (newer CM)
bool flg_touchkey_ignore = false;

#if defined(TK_HAS_AUTOCAL)
static u16 raw_data0;
static u16 raw_data1;
static u16 raw_data2;
static u16 raw_data3;
static u8 idac0;
static u8 idac1;
static u8 idac2;
static u8 idac3;
static u8 touchkey_threshold;

static int touchkey_autocalibration(struct touchkey_i2c *tkey_i2c);
#endif

#ifdef CONFIG_TOUCH_WAKE
struct touchkey_i2c *touchwakedevdata;
#endif

struct input_dev *input_dev_tk;

#ifdef CONFIG_TOUCHSCREEN_GESTURES
unsigned int sttg_gesture_delay = 0;
#endif

#if defined(CONFIG_TARGET_LOCALE_KOR)
#ifndef TRUE
#define TRUE	1
#endif

#ifndef FALSE
#define FALSE	0
#endif

#if defined(SEC_TKEY_EVENT_DEBUG)
static bool g_debug_tkey = TRUE;
#else
static bool g_debug_tkey = FALSE;
#endif
#endif

static int touchkey_i2c_check(struct touchkey_i2c *tkey_i2c);

static u16 menu_sensitivity;
static u16 back_sensitivity;
#if defined(TK_USE_4KEY)
static u8 home_sensitivity;
static u8 search_sensitivity;
#endif

static int touchkey_enable;
static bool touchkey_probe = true;

static const struct i2c_device_id sec_touchkey_id[] = {
	{"sec_touchkey", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, sec_touchkey_id);

extern int get_touchkey_firmware(char *version);
static int touchkey_led_status;
static int touchled_cmd_reversed;

static int touchkey_debug_count;
static char touchkey_debug[104];

#ifdef LED_LDO_WITH_REGULATOR
static void change_touch_key_led_voltage(int vol_mv)
{
	struct regulator *tled_regulator;

	tled_regulator = regulator_get(NULL, "touch_led");
	if (IS_ERR(tled_regulator)) {
		pr_err("%s: failed to get resource %s\n", __func__,
		       "touch_led");
		return;
	}
	regulator_set_voltage(tled_regulator, vol_mv * 1000, vol_mv * 1000);
	regulator_put(tled_regulator);
}

static ssize_t brightness_control(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	int data;

	if (sscanf(buf, "%d\n", &data) == 1) {
		printk(KERN_ERR "[TouchKey] touch_led_brightness: %d\n", data);
		change_touch_key_led_voltage(data);
	} else {
		printk(KERN_ERR "[TouchKey] touch_led_brightness Error\n");
	}

	return size;
}
#endif

static void set_touchkey_debug(char value)
{
	if (touchkey_debug_count == 100)
		touchkey_debug_count = 0;

	touchkey_debug[touchkey_debug_count] = value;
	touchkey_debug_count++;
}

static int i2c_touchkey_read(struct i2c_client *client,
		u8 reg, u8 *val, unsigned int len)
{
	int err = 0;
	int retry = 3;
#if !defined(TK_USE_GENERAL_SMBUS)
	struct i2c_msg msg[1];
#endif

	if ((client == NULL) || !(touchkey_enable == 1)
	    || !touchkey_probe) {
		printk(KERN_ERR "[TouchKey] touchkey is not enabled. %d\n",
		       __LINE__);
		return -ENODEV;
	}

	while (retry--) {
#if defined(TK_USE_GENERAL_SMBUS)
		err = i2c_smbus_read_i2c_block_data(client,
				KEYCODE_REG, len, val);
#else
		msg->addr = client->addr;
		msg->flags = I2C_M_RD;
		msg->len = len;
		msg->buf = val;
		err = i2c_transfer(client->adapter, msg, 1);
#endif

		if (err >= 0)
			return 0;
		printk(KERN_ERR "[TouchKey] %s %d i2c transfer error\n",
		       __func__, __LINE__);
		mdelay(10);
	}
	return err;

}

static int i2c_touchkey_write(struct i2c_client *client,
		u8 *val, unsigned int len)
{
	int err = 0;
	int retry = 3;
#if !defined(TK_USE_GENERAL_SMBUS)
	struct i2c_msg msg[1];
#endif

	if ((client == NULL) || !(touchkey_enable == 1)
	    || !touchkey_probe) {
		printk(KERN_ERR "[TouchKey] touchkey is not enabled. %d\n",
		       __LINE__);
		return -ENODEV;
	}

	while (retry--) {
#if defined(TK_USE_GENERAL_SMBUS)
		err = i2c_smbus_write_i2c_block_data(client,
				KEYCODE_REG, len, val);
#else
		msg->addr = client->addr;
		msg->flags = I2C_M_WR;
		msg->len = len;
		msg->buf = val;
		err = i2c_transfer(client->adapter, msg, 1);
#endif

		if (err >= 0)
			return 0;

		printk(KERN_DEBUG "[TouchKey] %s %d i2c transfer error\n",
		       __func__, __LINE__);
		mdelay(10);
	}
	return err;
}

#if defined(TK_HAS_AUTOCAL)
static int touchkey_autocalibration(struct touchkey_i2c *tkey_i2c)
{
	u8 data[6] = { 0, };
	int count = 0;
	int ret = 0;
	unsigned short retry = 0;

#if defined(CONFIG_TARGET_LOCALE_NA)
	if (tkey_i2c->module_ver < 8)
		return -1;
#endif

	while (retry < 3) {
		ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 4);
		if (ret < 0) {
			printk(KERN_ERR "[TouchKey]i2c read fail.\n");
			return ret;
		}
		printk(KERN_DEBUG
				"[TouchKey] data[0]=%x data[1]=%x data[2]=%x data[3]=%x\n",
				data[0], data[1], data[2], data[3]);

		/* Send autocal Command */
		data[0] = 0x50;
		data[3] = 0x01;

		count = i2c_touchkey_write(tkey_i2c->client, data, 4);

		msleep(100);

		/* Check autocal status */
		ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 6);

		if ((data[5] & TK_BIT_AUTOCAL)) {
			printk(KERN_DEBUG "[Touchkey] autocal Enabled\n");
			break;
		} else
			printk(KERN_DEBUG
			       "[Touchkey] autocal disabled, retry %d\n",
			       retry);

		retry = retry + 1;
	}

	if (retry == 3)
		printk(KERN_DEBUG "[Touchkey] autocal failed\n");

	return count;
}
#endif

#ifdef CONFIG_TARGET_LOCALE_NAATT
static ssize_t set_touchkey_autocal_testmode(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t size)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	int count = 0;
	u8 set_data;
	int on_off;

	if (sscanf(buf, "%d\n", &on_off) == 1) {
		printk(KERN_ERR "[TouchKey] Test Mode : %d\n", on_off);

		if (on_off == 1) {
			set_data = 0x40;
			count = i2c_touchkey_write(tkey_i2c->client,
					&set_data, 1);
		} else {
			tkey_i2c->pdata->power_on(0);
			msleep(50);
			tkey_i2c->pdata->power_on(1);
			msleep(50);
#if defined(TK_HAS_AUTOCAL)
			touchkey_autocalibration();
#endif
		}
	} else {
		printk(KERN_ERR "[TouchKey] touch_led_brightness Error\n");
	}

	return count;
}
#endif

#if defined(TK_HAS_AUTOCAL)
static ssize_t touchkey_raw_data0_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	u8 data[26] = { 0, };
	int ret;

	printk(KERN_DEBUG "called %s\n", __func__);
	ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 26);
#if defined(CONFIG_TARGET_LOCALE_NA)
	printk(KERN_DEBUG "called %s data[18] =%d,data[19] = %d\n", __func__,
	       data[18], data[19]);
	raw_data0 = ((0x00FF & data[18]) << 8) | data[19];
#elif defined(CONFIG_MACH_M0) || defined(CONFIG_MACH_C1)\
|| defined(CONFIG_MACH_M3)\
	 || defined(CONFIG_MACH_T0)
	printk(KERN_DEBUG "called %s data[16] =%d,data[17] = %d\n", __func__,
	       data[16], data[17]);
	raw_data0 = ((0x00FF & data[16]) << 8) | data[17]; /* menu*/
#elif defined(CONFIG_MACH_Q1_BD)
	printk(KERN_DEBUG "called %s data[16] =%d,data[17] = %d\n", __func__,
	       data[16], data[17]);
	raw_data0 = ((0x00FF & data[14]) << 8) | data[15];
#else
	printk(KERN_DEBUG "called %s data[18] =%d,data[19] = %d\n", __func__,
	       data[10], data[11]);
	raw_data0 = ((0x00FF & data[10]) << 8) | data[11];
#endif
	return sprintf(buf, "%d\n", raw_data0);
}

static ssize_t touchkey_raw_data1_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	u8 data[26] = { 0, };
	int ret;

	printk(KERN_DEBUG "called %s\n", __func__);
	ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 26);
#if defined(CONFIG_TARGET_LOCALE_NA)
	printk(KERN_DEBUG "called %s data[20] =%d,data[21] = %d\n", __func__,
	       data[20], data[21]);
	raw_data1 = ((0x00FF & data[20]) << 8) | data[21];
#elif defined(CONFIG_MACH_M0) || defined(CONFIG_MACH_C1)\
|| defined(CONFIG_MACH_M3)\
	 || defined(CONFIG_MACH_T0)
	printk(KERN_DEBUG "called %s data[14] =%d,data[15] = %d\n", __func__,
	       data[14], data[15]);
	raw_data1 = ((0x00FF & data[14]) << 8) | data[15]; /*back*/
#elif defined(CONFIG_MACH_Q1_BD)
	printk(KERN_DEBUG "called %s data[14] =%d,data[15] = %d\n", __func__,
			   data[14], data[15]);
	raw_data1 = ((0x00FF & data[16]) << 8) | data[17];
#else
	printk(KERN_DEBUG "called %s data[20] =%d,data[21] = %d\n", __func__,
	       data[12], data[13]);
	raw_data1 = ((0x00FF & data[12]) << 8) | data[13];
#endif				/* CONFIG_TARGET_LOCALE_NA */
	return sprintf(buf, "%d\n", raw_data1);
}

static ssize_t touchkey_raw_data2_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	u8 data[26] = { 0, };
	int ret;

	printk(KERN_DEBUG "called %s\n", __func__);
	ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 26);
#if defined(CONFIG_TARGET_LOCALE_NA)
	printk(KERN_DEBUG "called %s data[22] =%d,data[23] = %d\n", __func__,
	       data[22], data[23]);
	raw_data2 = ((0x00FF & data[22]) << 8) | data[23];
#else
	printk(KERN_DEBUG "called %s data[22] =%d,data[23] = %d\n", __func__,
	       data[14], data[15]);
	raw_data2 = ((0x00FF & data[14]) << 8) | data[15];
#endif				/* CONFIG_TARGET_LOCALE_NA */
	return sprintf(buf, "%d\n", raw_data2);
}

static ssize_t touchkey_raw_data3_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	u8 data[26] = { 0, };
	int ret;

	printk(KERN_DEBUG "called %s\n", __func__);
	ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 26);
#if defined(CONFIG_TARGET_LOCALE_NA)
	printk(KERN_DEBUG "called %s data[24] =%d,data[25] = %d\n", __func__,
	       data[24], data[25]);
	raw_data3 = ((0x00FF & data[24]) << 8) | data[25];
#else
	printk(KERN_DEBUG "called %s data[24] =%d,data[25] = %d\n", __func__,
	       data[16], data[17]);
	raw_data3 = ((0x00FF & data[16]) << 8) | data[17];
#endif				/* CONFIG_TARGET_LOCALE_NA */
	return sprintf(buf, "%d\n", raw_data3);
}

static ssize_t touchkey_idac0_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	u8 data[10];
	int ret;
#ifdef CONFIG_TARGET_LOCALE_NA
	if (tkey_i2c->module_ver < 8)
		return 0;
#endif

	printk(KERN_DEBUG "called %s\n", __func__);
	ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 10);
	printk(KERN_DEBUG "called %s data[6] =%d\n", __func__, data[6]);
	idac0 = data[6];
	return sprintf(buf, "%d\n", idac0);
}

static ssize_t touchkey_idac1_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	u8 data[10];
	int ret;
#ifdef CONFIG_TARGET_LOCALE_NA
	if (tkey_i2c->module_ver < 8)
		return 0;
#endif

	printk(KERN_DEBUG "called %s\n", __func__);
	ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 10);
	printk(KERN_DEBUG "called %s data[7] = %d\n", __func__, data[7]);
	idac1 = data[7];
	return sprintf(buf, "%d\n", idac1);
}

static ssize_t touchkey_idac2_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	u8 data[10];
	int ret;
#ifdef CONFIG_TARGET_LOCALE_NA
	if (tkey_i2c->module_ver < 8)
		return 0;
#endif

	printk(KERN_DEBUG "called %s\n", __func__);
	ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 10);
	printk(KERN_DEBUG "called %s data[8] =%d\n", __func__, data[8]);
	idac2 = data[8];
	return sprintf(buf, "%d\n", idac2);
}

static ssize_t touchkey_idac3_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	u8 data[10];
	int ret;
#ifdef CONFIG_TARGET_LOCALE_NA
	if (tkey_i2c->module_ver < 8)
		return 0;
#endif

	printk(KERN_DEBUG "called %s\n", __func__);
	ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 10);
	printk(KERN_DEBUG "called %s data[9] = %d\n", __func__, data[9]);
	idac3 = data[9];
	return sprintf(buf, "%d\n", idac3);
}

static ssize_t touchkey_threshold_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	u8 data[10];
	int ret;

	printk(KERN_DEBUG "called %s\n", __func__);
	ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 10);
	printk(KERN_DEBUG "called %s data[4] = %d\n", __func__, data[4]);
	touchkey_threshold = data[4];
	return sprintf(buf, "%d\n", touchkey_threshold);
}
#endif

#if defined(TK_HAS_FIRMWARE_UPDATE)
static int touchkey_firmware_update(struct touchkey_i2c *tkey_i2c)
{
	int retry = 3;
	int ret = 0;
	char data[3];

	disable_irq(tkey_i2c->irq);


	ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 3);
	if (ret < 0) {
		printk(KERN_DEBUG
		"[TouchKey] i2c read fail. do not excute firm update.\n");
		data[1] = 0;
		data[2] = 0;
	}

	printk(KERN_ERR "%s F/W version: 0x%x, Module version:0x%x\n", __func__,
	data[1], data[2]);

	tkey_i2c->firmware_ver = data[1];
	tkey_i2c->module_ver = data[2];

#if defined(CONFIG_MACH_M0) || defined(CONFIG_MACH_C1) \
|| defined(CONFIG_MACH_M3) || defined(CONFIG_MACH_T0)
	if ((tkey_i2c->firmware_ver < TK_FIRMWARE_VER) &&
	    (tkey_i2c->module_ver <= TK_MODULE_VER)) {
#else
	if ((tkey_i2c->firmware_ver < TK_FIRMWARE_VER) &&
		(tkey_i2c->module_ver == TK_MODULE_VER)) {
#endif
		printk(KERN_DEBUG "[TouchKey] firmware auto update excute\n");

		tkey_i2c->update_status = TK_UPDATE_DOWN;

		while (retry--) {
			if (ISSP_main(tkey_i2c) == 0) {
				printk(KERN_DEBUG
				       "[TouchKey]firmware update succeeded\n");
				tkey_i2c->update_status = TK_UPDATE_PASS;
				msleep(50);
				break;
			}
			msleep(50);
			printk(KERN_DEBUG
			       "[TouchKey] firmware update failed. retry\n");
		}
		if (retry <= 0) {
			tkey_i2c->pdata->power_on(0);
			tkey_i2c->update_status = TK_UPDATE_FAIL;
			printk(KERN_DEBUG
			       "[TouchKey] firmware update failed.\n");
		}
		ret = touchkey_i2c_check(tkey_i2c);
		if (ret < 0) {
			printk(KERN_DEBUG
				"[TouchKey] i2c read fail.\n");
			return TK_UPDATE_FAIL;
		}
#if defined(CONFIG_TARGET_LOCALE_KOR)
		ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 3);
		if (ret < 0) {
			printk(KERN_DEBUG
			"[TouchKey] i2c read fail. do not excute firm update.\n");
		}
		tkey_i2c->firmware_ver = data[1];
		tkey_i2c->module_ver = data[2];
#endif
		printk(KERN_DEBUG "[TouchKey] firm ver = %d, module ver = %d\n",
			tkey_i2c->firmware_ver, tkey_i2c->module_ver);
	} else {
		printk(KERN_DEBUG
		       "[TouchKey] firmware auto update do not excute\n");
		printk(KERN_DEBUG
		       "[TouchKey] firmware_ver(banary=%d, current=%d)\n",
		       TK_FIRMWARE_VER, tkey_i2c->firmware_ver);
		printk(KERN_DEBUG
		       "[TouchKey] module_ver(banary=%d, current=%d)\n",
		       TK_MODULE_VER, tkey_i2c->module_ver);
	}
	enable_irq(tkey_i2c->irq);
	return TK_UPDATE_PASS;
}
#else
static int touchkey_firmware_update(struct touchkey_i2c *tkey_i2c)
{
	char data[3];
	int retry;
	int ret = 0;

	ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 3);
	if (ret < 0) {
		printk(KERN_DEBUG
		       "[TouchKey] i2c read fail. do not excute firm update.\n");
		return ret;
	}

	printk(KERN_ERR "%s F/W version: 0x%x, Module version:0x%x\n", __func__,
	       data[1], data[2]);
	retry = 3;

	tkey_i2c->firmware_ver = data[1];
	tkey_i2c->module_ver = data[2];

	if (tkey_i2c->firmware_ver < 0x0A) {
		tkey_i2c->update_status = TK_UPDATE_DOWN;
		while (retry--) {
			if (ISSP_main(tkey_i2c) == 0) {
				printk(KERN_ERR
				       "[TOUCHKEY]Touchkey_update succeeded\n");
				tkey_i2c->update_status = TK_UPDATE_PASS;
				break;
			}
			printk(KERN_ERR "touchkey_update failed...retry...\n");
		}
		if (retry <= 0) {
			tkey_i2c->pdata->power_on(0);
			tkey_i2c->update_status = TK_UPDATE_FAIL;
			ret = TK_UPDATE_FAIL;
		}
	} else {
		if (tkey_i2c->firmware_ver >= 0x0A) {
			printk(KERN_ERR
			       "[TouchKey] Not F/W update. Cypess touch-key F/W version is latest\n");
		} else {
			printk(KERN_ERR
			       "[TouchKey] Not F/W update. Cypess touch-key version(module or F/W) is not valid\n");
		}
	}
	return ret;
}
#endif

#ifndef TEST_JIG_MODE
static irqreturn_t touchkey_interrupt(int irq, void *dev_id)
{
	struct touchkey_i2c *tkey_i2c = dev_id;
    static const int ledCmd[] = {TK_CMD_LED_ON, TK_CMD_LED_OFF};
	u8 data[3];
	int ret;
	int retry = 10;
	int keycode_type = 0;
	int pressed;
	static struct timeval time_now_tk_mt;

	set_touchkey_debug('a');

	retry = 3;
	while (retry--) {
		ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 3);
		if (!ret)
			break;
		else {
			printk(KERN_DEBUG
			       "[TouchKey] i2c read failed, ret:%d, retry: %d\n",
			       ret, retry);
			continue;
		}
	}
	if (ret < 0)
		return IRQ_HANDLED;

	set_touchkey_debug(data[0]);

	keycode_type = (data[0] & TK_BIT_KEYCODE);
	pressed = !(data[0] & TK_BIT_PRESS_EV);
	
	if (pressed && touchwake_enabled && flg_touchwake_active) {
		// if touchwake is enabled and active, decide if it should be triggered.
		// either way, exit early so the tap doesn't vibrate.
		
		if (!sttg_touchwake_ignoretkeys) {
			pr_info("[touchkey/touchwake] got touchkey press\n");
			
			// if input is locked, see if we should unlock when we wake.
			if (flg_pu_locktsp) {
				touch_press(sttg_pu_allow_tw);
			} else {
				// input isn't locked or pu isn't being used.
				touch_press(false);
			}
		}
		
		return IRQ_HANDLED;
	}
	
	if (flg_pu_locktsp && pu_valid()) {
		// device is locked out. disable touchkeys.
		printk("[TouchKey] input skipped (flg_pu_locktsp)\n");
		return IRQ_HANDLED;
	}
	
	if (flg_skip_next) {
		// avoid sending the key-up event.
		flg_skip_next = false;
		return IRQ_HANDLED;
	}
	
	if (pu_recording_end()) {
		// pu was recording, drop this press.
		flg_skip_next = true;
		return IRQ_HANDLED;
	}

	if (keycode_type <= 0 || keycode_type >= touchkey_count) {
		printk(KERN_DEBUG "[Touchkey] keycode_type err\n");
		return IRQ_HANDLED;
	}
	
	if (flg_touchkey_ignore || touch_is_pressed || epen_is_active || flg_epen_worryfree_mode) {
		printk("[TouchKey] input skipped (flg_touchkey_ignore)\n");
		return IRQ_HANDLED;
	}
	
	if (pressed  // touchkey down-state
		&& (
			(keycode_type == 1 && sttg_tk_mt_key1_mode)  // key1 is being pressed, is it enabled?
			 || (keycode_type == 2 && sttg_tk_mt_key2_mode)  // key2 is being pressed, is it enabled?
			)
		) {
		
		// get the time.
		do_gettimeofday(&time_now_tk_mt);
		
		time_since_tk_mt_lastpress = (time_now_tk_mt.tv_sec - time_tk_lastpress.tv_sec) * MSEC_PER_SEC +
										(time_now_tk_mt.tv_usec - time_tk_lastpress.tv_usec) / USEC_PER_MSEC;
		
		// save the time now that we've read it.
		do_gettimeofday(&time_tk_lastpress);
		
		if (time_since_tk_mt_lastpress < sttg_tk_mt_minimum_between  // not too fast
			|| time_since_tk_mt_lastpress > sttg_tk_mt_maximum_between   // not too slow
			|| keycode_type != tk_mt_last_keycode_type  // the same keycode must bind them all. right, frodo?
			) {
			// event is invalid, start a new one.
			
			printk(KERN_DEBUG "[TOUCHKEY/mt] first press (lastpress: %d, keycode: %d)\n", time_since_tk_mt_lastpress, keycode_type);
			ctr_tk_mt_presses = 1;
			
			// save which key this event is bound to.
			tk_mt_last_keycode_type = keycode_type;
			
			// ditch this press, and the up-state following it.
			flg_skip_next = true;
			return IRQ_HANDLED;
		}
		
		if (
			(keycode_type == 1 && ctr_tk_mt_presses == sttg_tk_mt_key1_mode)  // check for key1 nth
			|| (keycode_type == 2 && ctr_tk_mt_presses == sttg_tk_mt_key2_mode)  // check for key2 nth
			) {
			// this is the nth press, so let it through.
			
			printk(KERN_DEBUG "[TOUCHKEY/mt] nth press found (lastpress: %d, keycode: %d)\n", time_since_tk_mt_lastpress, keycode_type);
			
			// this shouldn't matter, but just to be safe reset it.
			ctr_tk_mt_presses = 0;
			
		} else {
			// this isn't the nth press. ignore it.
			
			// increment counter.
			ctr_tk_mt_presses++;
			printk(KERN_DEBUG "[TOUCHKEY/mt] ignoring press %d (%d - %d)\n", ctr_tk_mt_presses, keycode_type, pressed);
			
			// ditch the up-state following this press.
			flg_skip_next = true;
			
			// ditch this press.
			return IRQ_HANDLED;
		}
	}

	if (pressed) {
		set_touchkey_debug('P');

		// Yank555.lu : ROM is handling (newer CM)
		if (touch_led_handling == TOUCHKEY_LED_ROM && !touch_led_disabled) {

			// Yank555.lu : enable lights on h/w key pressed
			touchkey_pressed = TOUCHKEY_HW_PRESSED;
			if (touchkey_led_status       == TK_CMD_LED_OFF	       &&
			    touch_led_on_screen_touch == TOUCHKEY_LED_DISABLED   ) {
				pr_debug("[Touchkey] %s: enabling touchled\n", __func__);
				i2c_touchkey_write(tkey_i2c->client, (u8 *) &ledCmd[0], 1);
				touchkey_led_status = TK_CMD_LED_ON;
			}

		} else {

		// Yank555.lu : Kernel is handling (older CM)
		        // enable lights on keydown
			if (touch_led_disabled == 0) {
			    if (touchkey_led_status == TK_CMD_LED_OFF) {
				pr_debug("[Touchkey] %s: keydown - LED ON\n", __func__);
				i2c_touchkey_write(tkey_i2c->client, (u8 *) &ledCmd[0], 1);
				touchkey_led_status = TK_CMD_LED_ON;
			    }
			    if (timer_pending(&touch_led_timer) == 1) {
				mod_timer(&touch_led_timer, jiffies + (HZ * touch_led_timeout));
			    }
			}

		}

	} else {

		// Yank555.lu : Kernel is handling (older CM)
		if (touch_led_handling == TOUCHKEY_LED_KERNEL) {
			// touch led timeout on keyup
			if (touch_led_disabled == 0) {
			    if (timer_pending(&touch_led_timer) == 0) {
				pr_debug("[Touchkey] %s: keyup - add_timer\n", __func__);
				touch_led_timer.expires = jiffies + (HZ * touch_led_timeout);
				add_timer(&touch_led_timer);
			    } else {
				mod_timer(&touch_led_timer, jiffies + (HZ * touch_led_timeout));
			    }
			}
		}
		// Yank555.lu : ROM is handling (newer CM) - nothing to do
	}
	
	if (sttg_kw_mode > 0
		&& (sttg_ka_mode || sttg_kw_mode == 1 || sttg_kw_mode > 3)
		&& flg_screen_on) {
		// if knockwake is enabled, reset it if a touchkey was pressed.
		
		pr_info("[Touchkey/kw] resetting knockwake\n");
		ignoreKnocks(sttg_kw_tsp_event_suspensions);
	}
    
    if (pressed) {
        flg_touchkey_pressed = true;
        if (get_tsp_status()) {
            flg_touchkey_was_pressed = true;
            printk("[TSP/gestures] touchkey_was_pressed = true\n");
            return IRQ_HANDLED;
        } else {
            //flg_touchkey_was_pressed = false;
        }
    } else {
        flg_touchkey_pressed = false;
        if (!get_tsp_status()) {
            if (flg_touchkey_was_pressed) {
                flg_touchkey_was_pressed = false;
            }
        }
    }
	
#ifdef CONFIG_TOUCHSCREEN_GESTURES
	if (!ignore_gestures && pressed && sttg_gesture_delay > 0) {
		tsp_gestures_only(true);
	} else if (sttg_gesture_delay > 0) {
		tsp_gestures_only(false);
	}
#endif
	
	if (get_epen_status()) {
		//pr_info("[Touchkey] touckey pressed but ignored because of epen input\n");
		return IRQ_HANDLED;
	}
	
#ifdef CONFIG_TOUCHSCREEN_GESTURES
	if (!ignore_gestures && pressed && sttg_gesture_delay > 0) {
		// if ignore_gestures is true, then gestures are disabled, so don't even bother.
		//
		// immediately set gesture mode (fast gestures might execute before the msleep() is up)
		// wait x ms and recheck touchscreen.
        
        	if (touchwake_enabled && flg_touchwake_active && sttg_touchwake_ignoretkeys) {
			// if touchwake is enabled, active, and set to ignore tkeys, then we can stop now.
			return IRQ_HANDLED;
        	}
        
        	flg_touchkey_was_pressed = true;
		
		// reset touch_was_pressed flag.
		tsp_check_touched_flag(0);
		
		// preemptively enable gesture mode.
		tsp_gestures_only(true);
		
		// wait to see if we need to block this touchkey press.
		msleep(sttg_gesture_delay);
		
		// check to see if the screen has been touched.
		if (tsp_check_touched_flag(1) || get_tsp_status()) {
			// touchscreen has been touched, or is being touched. presumably a gesture is being drawn. don't report this keypress.
			//pr_info("[Touchkey] looks like this is a gesture.\n");
			return IRQ_HANDLED;
		} else {
			// cancel the preemptive gesture mode, user wasn't doing a gesture after all. carry on.
			tsp_gestures_only(false);
		}

	}
#endif

	if (get_tsp_status() && pressed) {
		printk(KERN_DEBUG "[TouchKey] touchkey pressed but don't send event because touch is pressed.\n");
	} else {
		
#ifdef CONFIG_TOUCH_WAKE

		/*if (pressed && touchwake_enabled && flg_touchwake_active) {
			// if touchwake is enabled and active, decide if it should be triggered.
			// either way, exit early so the tap doesn't vibrate.
			
			if (!sttg_touchwake_ignoretkeys) {
				pr_info("[touchkey/touchwake] got touchkey press\n");
				
				// if input is locked, see if we should unlock when we wake.
				if (flg_pu_locktsp) {
					touch_press(sttg_pu_allow_tw);
				} else {
					// input isn't locked or pu isn't being used.
					touch_press(false);
				}
			}// else {
				//pr_info("[touchkey/touchwake] ignoring touchkey press\n");
			//}
			
			return IRQ_HANDLED;
		}*/

#endif
		if (flg_screen_on)
			flg_ctr_cpuboost = sttg_boost_button_cycles;
		
		tk_key_keycode = touchkey_keycode[keycode_type];

		if (keycode_type == 1) {
			
			if (sttg_tk_key1_key_code) {
				// custom keycode.
				
				tk_key_keycode = sttg_tk_key1_key_code;
				pr_info("[TK/custom] SET - key1 set to: %d\n", tk_key_keycode);
				
				if (pressed && (sttg_tk_key1_key_code >= 900 || sttg_tk_key1_key_code == KEY_POWER)) {
					// user wants to use a special keycode.
					
					press_button(sttg_tk_key1_key_code, false, true, false, false);
					
					// stop now, since we've already pressed a button.
					return IRQ_HANDLED;
				}
			}
		} else if (keycode_type == 2) {
			
			if (sttg_tk_key2_key_code) {
				// custom keycode.
				
				tk_key_keycode = sttg_tk_key2_key_code;
				pr_info("[TK/custom] SET - key2 set to: %d\n", tk_key_keycode);
				
				if (pressed && (sttg_tk_key2_key_code >= 900 || sttg_tk_key2_key_code == KEY_POWER)) {
					// user wants to use a special keycode.
					
					press_button(sttg_tk_key2_key_code, false, true, false, false);
					
					// stop now, since we've already pressed a button.
					return IRQ_HANDLED;
				}
			}
		}
		
		input_report_key(tkey_i2c->input_dev,
				 tk_key_keycode, pressed);
		
		input_sync(tkey_i2c->input_dev);
		
#if !defined(CONFIG_SAMSUNG_PRODUCT_SHIP)
		printk(KERN_DEBUG "[TouchKey] keycode:%d pressed:%d type:%d\n",
		   tk_key_keycode, pressed, keycode_type);
#else
		printk(KERN_DEBUG "[TouchKey] keycode:%d pressed:%d type:%d\n",
			   tk_key_keycode, pressed, keycode_type);
#endif

		#if defined(CONFIG_TARGET_LOCALE_KOR)
		if (g_debug_tkey == true) {
			printk(KERN_DEBUG "[TouchKey] keycode[%d]=%d pressed:%d\n",
			keycode_type, touchkey_keycode[keycode_type], pressed);
		} else {
			printk(KERN_DEBUG "[TouchKey] pressed:%d\n", pressed);
		}
		#endif
	}
	set_touchkey_debug('A');
	return IRQ_HANDLED;
}
#else
static irqreturn_t touchkey_interrupt(int irq, void *dev_id)
{
	struct touchkey_i2c *tkey_i2c = dev_id;
	u8 data[18];
	int ret;
	int retry = 10;
	int keycode_type = 0;
	int pressed;

#if 0
	if (gpio_get_value(_3_GPIO_TOUCH_INT)) {
		printk(KERN_DEBUG "[TouchKey] Unknown state.\n", __func__);
		return IRQ_HANDLED;
	}
#endif

	set_touchkey_debug('a');

#ifdef CONFIG_CPU_FREQ
	/* set_dvfs_target_level(LEV_800MHZ); */
#endif

	retry = 3;
	while (retry--) {
#if defined(CONFIG_TARGET_LOCALE_NA) || defined(CONFIG_MACH_Q1_BD)\
	 || defined(CONFIG_MACH_C1)
		ret = i2c_touchkey_read(tkey_i2c->client,
				KEYCODE_REG, data, 18);
#else
		ret = i2c_touchkey_read(tkey_i2c->client,
				KEYCODE_REG, data, 10);
#endif
		if (!ret)
			break;
		else {
			printk(KERN_DEBUG
			       "[TouchKey] i2c read failed, ret:%d, retry: %d\n",
			       ret, retry);
			continue;
		}
	}
	if (ret < 0)
		return IRQ_HANDLED;

#if defined(CONFIG_TARGET_LOCALE_NA)
#if defined(CONFIG_MACH_C1_NA_SPR_EPIC2_REV00)
	menu_sensitivity = data[11];
	home_sensitivity = data[13];
	search_sensitivity = data[15];
	back_sensitivity = data[17];
#else
	if (tkey_i2c->module_ver >= 8) {
		menu_sensitivity = data[17];
		home_sensitivity = data[15];
		search_sensitivity = data[11];
		back_sensitivity = data[13];
	} else {
		menu_sensitivity = data[6];
		home_sensitivity = data[7];
		search_sensitivity = data[8];
		back_sensitivity = data[9];
	}
#endif
#elif defined(CONFIG_MACH_Q1_BD) || defined(CONFIG_MACH_C1)
	menu_sensitivity = data[13];
	back_sensitivity = data[11];
#else
	menu_sensitivity = data[7];
	back_sensitivity = data[9];
#endif				/* CONFIG_TARGET_LOCALE_NA  */

	set_touchkey_debug(data[0]);

	keycode_type = (data[0] & TK_BIT_KEYCODE);
	pressed = !(data[0] & TK_BIT_PRESS_EV);

	if (keycode_type <= 0 || keycode_type >= touchkey_count) {
		printk(KERN_DEBUG "[Touchkey] keycode_type err\n");
		return IRQ_HANDLED;
	}

	if (pressed)
		set_touchkey_debug('P');

	if (get_tsp_status() && pressed)
		printk(KERN_DEBUG "[TouchKey] touchkey pressed"
		       " but don't send event because touch is pressed.\n");
	else {
		input_report_key(touchkey_driver->input_dev,
				 touchkey_keycode[keycode_type], pressed);
		input_sync(touchkey_driver->input_dev);

#ifdef CONFIG_TOUCH_WAKE
	touch_press(true);
#endif
		/* printk(KERN_DEBUG "[TouchKey] keycode:%d pressed:%d\n",
		   touchkey_keycode[keycode_index], pressed); */
	}

	if (keycode_type == 1)
		printk(KERN_DEBUG "search key sensitivity = %d\n",
		       search_sensitivity);
	if (keycode_type == 2)
		printk(KERN_DEBUG "back key sensitivity = %d\n",
		       back_sensitivity);
#ifdef CONFIG_TARGET_LOCALE_NA
	if (keycode_type == 3)
		printk(KERN_DEBUG "home key sensitivity = %d\n",
		       home_sensitivity);
	if (keycode_type == 4)
		printk(KERN_DEBUG "menu key sensitivity = %d\n",
		       menu_sensitivity);
#endif

	set_touchkey_debug('A');
	return IRQ_HANDLED;
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static int sec_touchkey_early_suspend(struct early_suspend *h)
{
	struct touchkey_i2c *tkey_i2c =
	container_of(h, struct touchkey_i2c, early_suspend);
	
#ifdef CONFIG_TOUCH_WAKE
	tkey_i2c->pdata->led_power_on(0);
    	flg_touchkey_pressed = false;
    	flg_touchkey_was_pressed = false;
#else
	int ret;
	int i;

	disable_irq(tkey_i2c->irq);
	ret = cancel_work_sync(&tkey_i2c->update_work);
	if (ret) {
		printk(KERN_DEBUG "[Touchkey] enable_irq ret=%d\n", ret);
		enable_irq(tkey_i2c->irq);
	}

	/* release keys */
	for (i = 1; i < touchkey_count; ++i) {
		input_report_key(tkey_i2c->input_dev,
				 touchkey_keycode[i], 0);
	}
	input_sync(tkey_i2c->input_dev);

	touchkey_enable = 0;
	set_touchkey_debug('S');
	printk(KERN_DEBUG "[TouchKey] sec_touchkey_early_suspend\n");
	if (touchkey_enable < 0) {
		printk(KERN_DEBUG "[TouchKey] ---%s---touchkey_enable: %d\n",
		       __func__, touchkey_enable);
		return 0;
	}

	/* disable ldo18 */
	tkey_i2c->pdata->led_power_on(0);

	/* disable ldo11 */
	tkey_i2c->pdata->power_on(0);
#endif

	return 0;
}

static int sec_touchkey_late_resume(struct early_suspend *h)
{

	struct touchkey_i2c *tkey_i2c =
	container_of(h, struct touchkey_i2c, early_suspend);
    
    	flg_touchkey_was_pressed = false;
	
#ifdef CONFIG_TOUCH_WAKE
	tkey_i2c->pdata->led_power_on(1);
    	flg_touchkey_pressed = false;
    	flg_touchkey_was_pressed = false;
#else
#ifdef TEST_JIG_MODE
	unsigned char get_touch = 0x40;
#endif

	set_touchkey_debug('R');
	printk(KERN_DEBUG "[TouchKey] sec_touchkey_late_resume\n");

	/* enable ldo11 */
	tkey_i2c->pdata->power_on(1);

	if (touchkey_enable < 0) {
		printk(KERN_DEBUG "[TouchKey] ---%s---touchkey_enable: %d\n",
		       __func__, touchkey_enable);
		return 0;
	}
	msleep(50);
	tkey_i2c->pdata->led_power_on(1);

	touchkey_enable = 1;

#if defined(TK_HAS_AUTOCAL)
	touchkey_autocalibration(tkey_i2c);
#endif

	if (touchled_cmd_reversed) {
		touchled_cmd_reversed = 0;
		// Yank555.lu : touch_led_on_screen_touch : only accept feedback from touchscreen driver if enabled
		if (touch_led_on_screen_touch == TOUCHKEY_LED_DISABLED) {
			touchkey_led_status = TK_CMD_LED_OFF;
		}
		i2c_touchkey_write(tkey_i2c->client,
			(u8 *) &touchkey_led_status, 1);
		printk(KERN_DEBUG "[Touchkey] LED returned to desired state\n");
	}
#ifdef TEST_JIG_MODE
	i2c_touchkey_write(tkey_i2c->client, &get_touch, 1);
#endif

	enable_irq(tkey_i2c->irq);
#endif

	return 0;
}
#endif
	
#ifdef CONFIG_TOUCH_WAKE
static void cypress_touchwake_disable(void)
{
	int ret;
	int i;

	printk(KERN_DEBUG "[Touchkey] touchwake_disable\n");

	disable_irq(touchwakedevdata->irq);
	ret = cancel_work_sync(&touchwakedevdata->update_work);
	
	flg_cypress_suspended = true;
		
	if (ret) {
		printk(KERN_DEBUG "[Touchkey] enable_irq ret=%d\n", ret);
		enable_irq(touchwakedevdata->irq);
	}
	
	/* release keys */
	for (i = 1; i < touchkey_count; ++i) {
		input_report_key(touchwakedevdata->input_dev,
						 touchkey_keycode[i], 0);
	}
	input_sync(touchwakedevdata->input_dev);
		
	touchkey_enable = 0;
	set_touchkey_debug('S');
	printk(KERN_DEBUG "[TouchKey] sec_touchkey_early_suspend\n");
	if (touchkey_enable < 0) {
		printk(KERN_DEBUG "[TouchKey] ---%s---touchkey_enable: %d\n",
			   __func__, touchkey_enable);
		return;
	}
	
	/* disable ldo11 */
	touchwakedevdata->pdata->power_on(0);
	
	return;
	
}
	
static void cypress_touchwake_enable(void)
{
	if (!flg_cypress_suspended) {
		return;
	}
	
	printk(KERN_DEBUG "[Touchkey] touchwake_enable\n");
	
	set_touchkey_debug('R');
	printk(KERN_DEBUG "[TouchKey] sec_touchkey_late_resume\n");
	
	/* enable ldo11 */
	touchwakedevdata->pdata->power_on(1);
	
	if (touchkey_enable < 0) {
		printk(KERN_DEBUG "[TouchKey] ---%s---touchkey_enable: %d\n",
			   __func__, touchkey_enable);
		return;
	}
	msleep(50);
	touchwakedevdata->pdata->led_power_on(1);
	
	touchkey_enable = 1;
	
#if defined(TK_HAS_AUTOCAL)
	touchkey_autocalibration(touchwakedevdata);
#endif
	
	if (touchled_cmd_reversed) {
		touchled_cmd_reversed = 0;
		i2c_touchkey_write(touchwakedevdata->client,
						   (u8 *) &touchkey_led_status, 1);
		printk(KERN_DEBUG "[Touchkey] LED returned on\n");
	}
	
	enable_irq(touchwakedevdata->irq);
	
	flg_cypress_suspended = false;

	return;
	
}
	
static struct touchwake_implementation cypress_touchwake = 
    {
		.enable = cypress_touchwake_enable,
		.disable = cypress_touchwake_disable,
    };
#endif

static int touchkey_i2c_check(struct touchkey_i2c *tkey_i2c)
{
	char data[3] = { 0, };
	int ret = 0;

	ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 3);
	if (ret < 0) {
		printk(KERN_ERR "[TouchKey] module version read fail\n");
		return ret;
	}

	tkey_i2c->firmware_ver = data[1];
	tkey_i2c->module_ver = data[2];

	return ret;
}

ssize_t touchkey_update_read(struct file *filp, char *buf, size_t count,
			     loff_t *f_pos)
{
	char data[3] = { 0, };

	get_touchkey_firmware(data);
	put_user(data[1], buf);

	return 1;
}

static ssize_t touch_version_read(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	char data[3] = { 0, };
	int count;

	i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 3);

	count = sprintf(buf, "0x%x\n", data[1]);

	printk(KERN_DEBUG "[TouchKey] touch_version_read 0x%x\n", data[1]);
	printk(KERN_DEBUG "[TouchKey] module_version_read 0x%x\n", data[2]);

	return count;
}

static ssize_t touch_version_write(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	printk(KERN_DEBUG "[TouchKey] input data --> %s\n", buf);

	return size;
}

void touchkey_update_func(struct work_struct *work)
{
	struct touchkey_i2c *tkey_i2c =
		container_of(work, struct touchkey_i2c, update_work);
	int retry = 3;
#if defined(CONFIG_TARGET_LOCALE_NAATT)
	char data[3];
	i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 3);
	printk(KERN_DEBUG "[%s] F/W version: 0x%x, Module version:0x%x\n",
	       __func__, data[1], data[2]);
#endif
	tkey_i2c->update_status = TK_UPDATE_DOWN;
	printk(KERN_DEBUG "[TouchKey] %s start\n", __func__);
	touchkey_enable = 0;
	while (retry--) {
		if (ISSP_main(tkey_i2c) == 0) {
			printk(KERN_DEBUG
			       "[TouchKey] touchkey_update succeeded\n");
			msleep(50);
			touchkey_enable = 1;
#if defined(TK_HAS_AUTOCAL)
			touchkey_autocalibration(tkey_i2c);
#endif
			tkey_i2c->update_status = TK_UPDATE_PASS;
			enable_irq(tkey_i2c->irq);
			return;
		}
		tkey_i2c->pdata->power_on(0);
	}
	enable_irq(tkey_i2c->irq);
	tkey_i2c->update_status = TK_UPDATE_FAIL;
	printk(KERN_DEBUG "[TouchKey] touchkey_update failed\n");
	return;
}

static ssize_t touch_update_write(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
#ifdef CONFIG_TARGET_LOCALE_NA
	if (tkey_i2c->module_ver < 8) {
		printk(KERN_DEBUG
		       "[TouchKey] Skipping f/w update : module_version =%d\n",
		       tkey_i2c->module_ver);
		tkey_i2c->update_status = TK_UPDATE_PASS;
		return 1;
	} else {
#endif				/* CONFIG_TARGET_LOCALE_NA */
		printk(KERN_DEBUG "[TouchKey] touchkey firmware update\n");

		if (*buf == 'S') {
			disable_irq(tkey_i2c->irq);
			schedule_work(&tkey_i2c->update_work);
		}
		return size;
#ifdef CONFIG_TARGET_LOCALE_NA
	}
#endif				/* CONFIG_TARGET_LOCALE_NA */
}

static ssize_t touch_update_read(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	int count = 0;

	printk(KERN_DEBUG
	       "[TouchKey] touch_update_read: update_status %d\n",
	       tkey_i2c->update_status);

	if (tkey_i2c->update_status == TK_UPDATE_PASS)
		count = sprintf(buf, "PASS\n");
	else if (tkey_i2c->update_status == TK_UPDATE_DOWN)
		count = sprintf(buf, "Downloading\n");
	else if (tkey_i2c->update_status == TK_UPDATE_FAIL)
		count = sprintf(buf, "Fail\n");

	return count;
}

static ssize_t touchkey_led_control_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
    int ret;

    ret = sprintf(buf, "%d\n", touchkey_led_status);
    pr_info("[Touchkey] %s: touchkey_led_status=%d\n", __func__, touchkey_led_status);

    return ret;
}

static ssize_t touchkey_led_control(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t size)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	int data;
	int ret;
	static const int ledCmd[] = {TK_CMD_LED_ON, TK_CMD_LED_OFF};

#if defined(CONFIG_TARGET_LOCALE_KOR)
	if (touchkey_probe == false)
		return size;
#endif
	ret = sscanf(buf, "%d", &data);
	if (ret != 1) {
		printk(KERN_DEBUG "[TouchKey] %s, %d err\n",
			__func__, __LINE__);
		return size;
	}

	if (data != 1 && data != 2) {
		printk(KERN_DEBUG "[TouchKey] %s wrong cmd %x\n",
			__func__, data);
		return size;
	}

	if (data == 2 && touch_led_handling == TOUCHKEY_LED_ROM)
		touchkey_pressed = TOUCHKEY_HW_TIMEDOUT; // Yank555.lu : h/w light disabled, consider timeout reached

	if (touchkey_led_status 	== TK_CMD_LED_OFF	 &&
	    touchkey_pressed 		== TOUCHKEY_HW_TIMEDOUT  &&
	    touch_led_handling		== TOUCHKEY_LED_ROM   &&
	    touch_led_on_screen_touch	== TOUCHKEY_LED_DISABLED    ) {

		data = TK_CMD_LED_OFF;

	} else {

		data = ledCmd[data-1];

	}

	// Yank555.lu : KERNEL is handling (older CM)
	if (touch_led_handling == TOUCHKEY_LED_KERNEL) {
	    if (touch_led_disabled == 0) {
		ret = i2c_touchkey_write(tkey_i2c->client, (u8 *) &data, 1);
	    }

	    if(data == ledCmd[0]) {
		if (touch_led_disabled == 0) {
		    if (timer_pending(&touch_led_timer) == 0) {
			pr_debug("[Touchkey] %s: add_timer\n", __func__);
			touch_led_timer.expires = jiffies + (HZ * touch_led_timeout);
			add_timer(&touch_led_timer);
		    } else {
			mod_timer(&touch_led_timer, jiffies + (HZ * touch_led_timeout));
		    }
		}
	    } else {
		if (timer_pending(&touch_led_timer) == 1) {
		    pr_debug("[Touchkey] %s: del_timer\n", __func__);
		    del_timer(&touch_led_timer);
		}
	    }
	} else {
		// Yank555.lu : ROM is handling (newer CM)
		ret = i2c_touchkey_write(tkey_i2c->client, (u8 *) &data, 1);
	}

	if (ret == -ENODEV) {
		printk(KERN_DEBUG"[Touchkey] error to write i2c\n");
		touchled_cmd_reversed = 1;
	}

	touchkey_led_status = data;

	printk("[Touchkey] %s: new status = %d\n", __func__, touchkey_led_status);

	return size;
}

#ifdef CONFIG_TOUCHSCREEN_GESTURES
static ssize_t gesture_delay_show(struct device *dev,
									  struct device_attribute *attr, char *buf)
{
	int ret;
	
	ret = sprintf(buf, "%d\n", sttg_gesture_delay);
	
	return ret;
}

static ssize_t gesture_delay_store(struct device *dev,
								   struct device_attribute *attr, const char *buf,
								   size_t size)
{
	int data, ret;
	
	ret = sscanf(buf, "%d\n", &data);
	if (unlikely(ret != 1 || data < 0 || data > 2000)) {
		return -EINVAL;
	}
	
	sttg_gesture_delay = data;
	pr_info("[Touchkey] gesture_delay set to %d\n", data);
	
	return size;
}
#endif

static ssize_t touch_led_force_disable_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int ret;

    ret = sprintf(buf, "%d\n", touch_led_disabled);
    pr_info("[Touchkey] %s: touch_led_disabled=%d\n", __func__, touch_led_disabled);

    return ret;
}

static ssize_t touch_led_force_disable_store(struct device *dev,
        struct device_attribute *attr, const char *buf,
        size_t size)
{
    struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	static const int ledCmd[] = {TK_CMD_LED_ON, TK_CMD_LED_OFF};
    int data, ret;

    ret = sscanf(buf, "%d\n", &data);
    if (unlikely(ret != 1)) {
        pr_err("[Touchkey] %s err\n", __func__);
        return -EINVAL;
    }
    pr_info("[Touchkey] %s value=%d\n", __func__, data);
    
    if (data == 1) {
        i2c_touchkey_write(tkey_i2c->client, (u8 *) &ledCmd[1], 1);
        touchkey_led_status = TK_CMD_LED_OFF;
    }
    touch_led_disabled = data;

    return size;
}
static DEVICE_ATTR(force_disable, S_IRUGO | S_IWUSR | S_IWGRP,
        touch_led_force_disable_show, touch_led_force_disable_store);

static ssize_t touch_led_timeout_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int ret;

    ret = sprintf(buf, "%d\n", touch_led_timeout);
    pr_info("[Touchkey] %s: touch_led_timeout=%d\n", __func__, touch_led_timeout);

    return ret;
}

static ssize_t touch_led_timeout_store(struct device *dev,
        struct device_attribute *attr, const char *buf,
        size_t size)
{
    int data;
    int ret;

    ret = sscanf(buf, "%d\n", &data);
    if (unlikely(ret != 1)) {
        pr_err("[TouchKey] %s err\n", __func__);
        return -EINVAL;
    }
    pr_info("[TouchKey] %s new timeout=%d\n", __func__, data);
    touch_led_timeout = data;

    return size;
}
static DEVICE_ATTR(timeout, S_IRUGO | S_IWUSR | S_IWGRP,
        touch_led_timeout_show, touch_led_timeout_store);

void touch_led_timedout(unsigned long ptr)
{
    queue_work(tkey_i2c_local->wq, &tkey_i2c_local->work);
}

void touch_led_timedout_work(struct work_struct *work)
{
    struct touchkey_i2c *tkey_i2c = container_of(work, struct touchkey_i2c, work);
    static const int ledCmd[] = {TK_CMD_LED_ON, TK_CMD_LED_OFF};

    if (touch_led_timeout != 0)
    {
        pr_info("[TouchKey] %s disabling touchled\n", __func__);
        i2c_touchkey_write(tkey_i2c->client, (u8 *) &ledCmd[1], 1);
        touchkey_led_status = TK_CMD_LED_OFF;
    }
}

void touchscreen_state_report(int state)
{
    static const int ledCmd[] = {TK_CMD_LED_ON, TK_CMD_LED_OFF};

	// Yank555.lu : KERNEL is handling (older CM)
	if (touch_led_handling == TOUCHKEY_LED_KERNEL) {

	    // Yank555.lu : touch_led_on_screen_touch : only accept feedback from touchscreen driver if enabled
	    if (touch_led_disabled == 0 && touch_led_on_screen_touch == TOUCHKEY_LED_ENABLED) {
		if (state == 1) {
		    if(touchkey_led_status == TK_CMD_LED_OFF) {
		        pr_debug("[TouchKey] %s enable touchleds\n", __func__);
		        i2c_touchkey_write(tkey_i2c_local->client, (u8 *) &ledCmd[0], 1);
		        touchkey_led_status = TK_CMD_LED_ON;
		    } else {
		        if (timer_pending(&touch_led_timer) == 1) {
		            pr_debug("[TouchKey] %s mod_timer\n", __func__);
		            mod_timer(&touch_led_timer, jiffies + (HZ * touch_led_timeout));
		        }
		    }
		} else if (state == 0) {
		    if (timer_pending(&touch_led_timer) == 1) {
		        pr_debug("[TouchKey] %s mod_timer\n", __func__);
		        mod_timer(&touch_led_timer, jiffies + (HZ * touch_led_timeout));
		    } else if (touchkey_led_status == TK_CMD_LED_ON){
		        pr_debug("[TouchKey] %s add_timer\n", __func__);
		        touch_led_timer.expires = jiffies + (HZ * touch_led_timeout);
		        add_timer(&touch_led_timer);
		    }
		}
	    }
	}
	// Yank555.lu : ROM is handling (newer CM) - nothing to do
}

#if defined(TK_USE_4KEY)
static ssize_t touchkey_menu_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	u8 data[18] = { 0, };
	int ret;

	printk(KERN_DEBUG "called %s\n", __func__);
	ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 18);
#ifdef CONFIG_TARGET_LOCALE_NA
	if (tkey_i2c->module_ver < 8) {
		printk(KERN_DEBUG "called %s data[12] =%d,data[13] = %d\n",
		       __func__, data[12], data[13]);
		menu_sensitivity = ((0x00FF & data[12]) << 8) | data[13];
	} else {
		printk(KERN_DEBUG "called %s data[17] =%d\n", __func__,
		       data[17]);
		menu_sensitivity = data[17];
	}
#else
	printk(KERN_DEBUG "called %s data[10] =%d,data[11] = %d\n", __func__,
	       data[10], data[11]);
	menu_sensitivity = ((0x00FF & data[10]) << 8) | data[11];
#endif				/* CONFIG_TARGET_LOCALE_NA */
	return sprintf(buf, "%d\n", menu_sensitivity);
}

static ssize_t touchkey_home_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	u8 data[18] = { 0, };
	int ret;

	printk(KERN_DEBUG "called %s\n", __func__);
	ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 18);
#ifdef CONFIG_TARGET_LOCALE_NA
	if (tkey_i2c->module_ver < 8) {
		printk(KERN_DEBUG "called %s data[10] =%d,data[11] = %d\n",
		       __func__, data[10], data[11]);
		home_sensitivity = ((0x00FF & data[10]) << 8) | data[11];
	} else {
		printk(KERN_DEBUG "called %s data[15] =%d\n", __func__,
		       data[15]);
		home_sensitivity = data[15];
	}
#else
	printk(KERN_DEBUG "called %s data[12] =%d,data[13] = %d\n", __func__,
	       data[12], data[13]);
	home_sensitivity = ((0x00FF & data[12]) << 8) | data[13];
#endif				/* CONFIG_TARGET_LOCALE_NA */
	return sprintf(buf, "%d\n", home_sensitivity);
}

static ssize_t touchkey_back_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	u8 data[18] = { 0, };
	int ret;

	printk(KERN_DEBUG "called %s\n", __func__);
	ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 18);
#ifdef CONFIG_TARGET_LOCALE_NA
	if (tkey_i2c->module_ver < 8) {
		printk(KERN_DEBUG "called %s data[8] =%d,data[9] = %d\n",
		       __func__, data[8], data[9]);
		back_sensitivity = ((0x00FF & data[8]) << 8) | data[9];
	} else {
		printk(KERN_DEBUG "called %s data[13] =%d\n", __func__,
		       data[13]);
		back_sensitivity = data[13];
	}
#else
	printk(KERN_DEBUG "called %s data[14] =%d,data[15] = %d\n", __func__,
	       data[14], data[15]);
	back_sensitivity = ((0x00FF & data[14]) << 8) | data[15];
#endif				/* CONFIG_TARGET_LOCALE_NA */
	return sprintf(buf, "%d\n", back_sensitivity);
}

static ssize_t touchkey_search_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	u8 data[18] = { 0, };
	int ret;

	printk(KERN_DEBUG "called %s\n", __func__);
	ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 18);
#ifdef CONFIG_TARGET_LOCALE_NA
	if (tkey_i2c->module_ver < 8) {
		printk(KERN_DEBUG "called %s data[6] =%d,data[7] = %d\n",
		       __func__, data[6], data[7]);
		search_sensitivity = ((0x00FF & data[6]) << 8) | data[7];
	} else {
		printk(KERN_DEBUG "called %s data[11] =%d\n", __func__,
		       data[11]);
		search_sensitivity = data[11];
	}
#else
	printk(KERN_DEBUG "called %s data[16] =%d,data[17] = %d\n", __func__,
	       data[16], data[17]);
	search_sensitivity = ((0x00FF & data[16]) << 8) | data[17];
#endif				/* CONFIG_TARGET_LOCALE_NA */
	return sprintf(buf, "%d\n", search_sensitivity);
}
#else
static ssize_t touchkey_menu_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
#if defined(CONFIG_MACH_Q1_BD) \
|| (defined(CONFIG_MACH_C1) && defined(CONFIG_TARGET_LOCALE_KOR))\
	|| defined(CONFIG_MACH_T0)
	u8 data[14] = { 0, };
	int ret;

	ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 14);

	printk(KERN_DEBUG "called %s data[12] = %d, data[13] =%d\n", __func__,
			data[12], data[13]);
	menu_sensitivity = ((0x00FF & data[12]) << 8) | data[13];
	printk(KERN_DEBUG "called %s menu_sensitivity =%d\n", __func__,
			menu_sensitivity);

#else
	u8 data[10];
	int ret;

	printk(KERN_DEBUG "called %s\n", __func__);
	ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 10);
	menu_sensitivity = data[7];
#endif
	return sprintf(buf, "%d\n", menu_sensitivity);
}

static ssize_t touchkey_back_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
#if defined(CONFIG_MACH_Q1_BD) \
	|| (defined(CONFIG_MACH_C1) && defined(CONFIG_TARGET_LOCALE_KOR))\
	|| defined(CONFIG_MACH_T0)
	u8 data[14] = { 0, };
	int ret;

	ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 14);

	printk(KERN_DEBUG "called %s data[10] = %d, data[11] =%d\n", __func__,
			data[10], data[11]);
	back_sensitivity =((0x00FF & data[10]) << 8) | data[11];
	printk(KERN_DEBUG "called %s back_sensitivity =%d\n", __func__,
			back_sensitivity);
#else
	u8 data[10];
	int ret;

	printk(KERN_DEBUG "called %s\n", __func__);
	ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 10);
	back_sensitivity = data[9];
#endif
	return sprintf(buf, "%d\n", back_sensitivity);
}
#endif

// Yank555.lu : touch_led_on_screen_touch : only accept feedback from touchscreen driver if enabled
static ssize_t touch_led_on_screen_touch_show(struct device *dev,
					      struct device_attribute *attr, char *buf)
{
	  return sprintf(buf, "%d\n", touch_led_on_screen_touch);
}

static ssize_t touch_led_on_screen_touch_store(struct device *dev,
					       struct device_attribute *attr, const char *buf, size_t count)
{
	int new_touch_led_on_screen_touch;

	sscanf(buf, "%du", &new_touch_led_on_screen_touch);

	switch (new_touch_led_on_screen_touch) {
	  case TOUCHKEY_LED_DISABLED:
	  case TOUCHKEY_LED_ENABLED:	touch_led_on_screen_touch = new_touch_led_on_screen_touch;
					return count;
	  default:			return -EINVAL;
	}

}

// Yank555.lu : touch_led_handling : have kernel (older CM) or ROM (newer CM) handle h/w keys backlight
static ssize_t touch_led_handling_show(struct device *dev,
					      struct device_attribute *attr, char *buf)
{
	  return sprintf(buf, "%d\n", touch_led_handling);
}

static ssize_t touch_led_handling_store(struct device *dev,
					       struct device_attribute *attr, const char *buf, size_t count)
{
	int new_touch_led_handling;

	sscanf(buf, "%du", &new_touch_led_handling);

	switch (new_touch_led_handling) {
	  case TOUCHKEY_LED_ROM:
	  case TOUCHKEY_LED_KERNEL:	touch_led_handling = new_touch_led_handling;
					return count;
	  default:			return -EINVAL;
	}

}

#if defined(TK_HAS_AUTOCAL)
static ssize_t autocalibration_enable(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t size)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	int data;

	sscanf(buf, "%d\n", &data);

	if (data == 1)
		touchkey_autocalibration(tkey_i2c);

	return size;
}

static ssize_t autocalibration_status(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	u8 data[6];
	int ret;
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);

	printk(KERN_DEBUG "[Touchkey] %s\n", __func__);

	ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 6);
	if ((data[5] & TK_BIT_AUTOCAL))
		return sprintf(buf, "Enabled\n");
	else
		return sprintf(buf, "Disabled\n");

}
#endif				/* CONFIG_TARGET_LOCALE_NA */

static ssize_t touch_sensitivity_control(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t size)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	unsigned char data = 0x40;
	i2c_touchkey_write(tkey_i2c->client, &data, 1);
	return size;
}

static ssize_t set_touchkey_firm_version_show(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	return sprintf(buf, "0x%x\n", TK_FIRMWARE_VER);
}

static ssize_t set_touchkey_update_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	int count = 0;
	int retry = 3;

	tkey_i2c->update_status = TK_UPDATE_DOWN;

	disable_irq(tkey_i2c->irq);

#ifdef TEST_JIG_MODE
	unsigned char get_touch = 0x40;
#endif

	while (retry--) {
		if (ISSP_main(tkey_i2c) == 0) {
			printk(KERN_ERR
			       "[TouchKey]Touchkey_update succeeded\n");
			tkey_i2c->update_status = TK_UPDATE_PASS;
			count = 1;
			msleep(50);
			break;
		}
		printk(KERN_ERR "touchkey_update failed... retry...\n");
	}
	if (retry <= 0) {
		/* disable ldo11 */
		tkey_i2c->pdata->power_on(0);
		count = 0;
		printk(KERN_ERR "[TouchKey]Touchkey_update fail\n");
		tkey_i2c->update_status = TK_UPDATE_FAIL;
		enable_irq(tkey_i2c->irq);
		return count;
	}

#ifdef TEST_JIG_MODE
	i2c_touchkey_write(tkey_i2c->client, &get_touch, 1);
#endif

#if defined(TK_HAS_AUTOCAL)
	touchkey_autocalibration(tkey_i2c);
#endif

	enable_irq(tkey_i2c->irq);

	return count;

}

static ssize_t set_touchkey_firm_version_read_show(struct device *dev,
						   struct device_attribute
						   *attr, char *buf)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	char data[3] = { 0, };
	int count;

	i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 3);
	count = sprintf(buf, "0x%x\n", data[1]);

	printk(KERN_DEBUG "[TouchKey] touch_version_read 0x%x\n", data[1]);
	printk(KERN_DEBUG "[TouchKey] module_version_read 0x%x\n", data[2]);
	return count;
}

static ssize_t set_touchkey_firm_status_show(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	int count = 0;

	printk(KERN_DEBUG
	       "[TouchKey] touch_update_read: update_status %d\n",
	       tkey_i2c->update_status);

	if (tkey_i2c->update_status == TK_UPDATE_PASS)
		count = sprintf(buf, "PASS\n");
	else if (tkey_i2c->update_status == TK_UPDATE_DOWN)
		count = sprintf(buf, "Downloading\n");
	else if (tkey_i2c->update_status == TK_UPDATE_FAIL)
		count = sprintf(buf, "Fail\n");

	return count;
}
	
static ssize_t tk_key1_key_code_show(struct device *dev,
									 struct device_attribute *attr, char *buf)
{
	int ret;
	ret = sprintf(buf, "%d\n", sttg_tk_key1_key_code);
	return ret;
}

static ssize_t tk_key1_key_code_store(struct device *dev,
									  struct device_attribute *attr, const char *buf, size_t size)
{
	int data, ret;
	
	ret = sscanf(buf, "%d\n", &data);
	if (unlikely(ret != 1 || data < 0)) {
		return -EINVAL;
	}
	
	sttg_tk_key1_key_code = data;
	pr_info("[TK/custom] STORE - sttg_tk_key1_key_code has been set to: %d\n", data);
	
	return size;
}
	
static ssize_t tk_key2_key_code_show(struct device *dev,
									 struct device_attribute *attr, char *buf)
{
	int ret;
	ret = sprintf(buf, "%d\n", sttg_tk_key2_key_code);
	return ret;
}

static ssize_t tk_key2_key_code_store(struct device *dev,
									  struct device_attribute *attr, const char *buf, size_t size)
{
	int data, ret;
	
	ret = sscanf(buf, "%d\n", &data);
	if (unlikely(ret != 1 || data < 0)) {
		return -EINVAL;
	}
	
	sttg_tk_key2_key_code = data;
	pr_info("[TK/custom] STORE - sttg_tk_key2_key_code has been set to: %d\n", data);
	
	return size;
}
	
static ssize_t tk_mt_key1_mode_show(struct device *dev,
									struct device_attribute *attr, char *buf)
{
	int ret;
	ret = sprintf(buf, "%d\n", sttg_tk_mt_key1_mode);
	return ret;
}

static ssize_t tk_mt_key1_mode_store(struct device *dev,
									 struct device_attribute *attr, const char *buf, size_t size)
{
	int data, ret;
	
	ret = sscanf(buf, "%d\n", &data);
	if (unlikely(ret != 1 || data < 0 || data > 5)) {
		return -EINVAL;
	}
	
	sttg_tk_mt_key1_mode = data;
	pr_info("[TK/custom] STORE - sttg_tk_mt_key1_mode has been set to: %d\n", data);
	
	return size;
}

static ssize_t tk_mt_key2_mode_show(struct device *dev,
									struct device_attribute *attr, char *buf)
{
	int ret;
	ret = sprintf(buf, "%d\n", sttg_tk_mt_key2_mode);
	return ret;
}

static ssize_t tk_mt_key2_mode_store(struct device *dev,
									 struct device_attribute *attr, const char *buf, size_t size)
{
	int data, ret;
	
	ret = sscanf(buf, "%d\n", &data);
	if (unlikely(ret != 1 || data < 0 || data > 5)) {
		return -EINVAL;
	}
	
	sttg_tk_mt_key2_mode = data;
	pr_info("[TK/custom] STORE - sttg_tk_mt_key2_mode has been set to: %d\n", data);
	
	return size;
}
	
static ssize_t tk_mt_minimum_between_show(struct device *dev,
										  struct device_attribute *attr, char *buf)
{
	int ret;
	ret = sprintf(buf, "%d\n", sttg_tk_mt_minimum_between);
	return ret;
}

static ssize_t tk_mt_minimum_between_store(struct device *dev,
										   struct device_attribute *attr, const char *buf, size_t size)
{
	int data, ret;
	
	ret = sscanf(buf, "%d\n", &data);
	if (unlikely(ret != 1 || data < 0 || data > 5000)) {
		return -EINVAL;
	}
	
	sttg_tk_mt_minimum_between = data;
	pr_info("[TK/custom] STORE - sttg_tk_mt_minimum_between has been set to: %d\n", data);
	
	return size;
}
	
static ssize_t tk_mt_maximum_between_show(struct device *dev,
										  struct device_attribute *attr, char *buf)
{
	int ret;
	ret = sprintf(buf, "%d\n", sttg_tk_mt_maximum_between);
	return ret;
}

static ssize_t tk_mt_maximum_between_store(struct device *dev,
										   struct device_attribute *attr, const char *buf, size_t size)
{
	int data, ret;
	
	ret = sscanf(buf, "%d\n", &data);
	if (unlikely(ret != 1 || data < 0 || data > 5000)) {
		return -EINVAL;
	}
	
	sttg_tk_mt_maximum_between = data;
	pr_info("[TK/custom] STORE - sttg_tk_mt_maximum_between has been set to: %d\n", data);
	
	return size;
}

static ssize_t mali_asv_show(struct device *dev,
									struct device_attribute *attr, char *buf)
{
	int ret;

	ret = sprintf(buf, "%d\n", get_mali_asv());
	
	return ret;
}

static ssize_t mali_asv_store(struct device *dev,
								   struct device_attribute *attr, const char *buf, size_t size)
{
	int data, ret;
	
	ret = sscanf(buf, "%d\n", &data);
	if (unlikely(ret != 1 || data < 0 || data > 13)) {
		return -EINVAL;
	}
	
	set_mali_asv(data);
	pr_info("[Touchkey] mali asv to %d\n", data);
	
	return size;
}
	
static ssize_t mali_cur_freq_show(struct device *dev,
							 struct device_attribute *attr, char *buf)
{
	int ret;
	
	ret = sprintf(buf, "%d\n", get_mali_cur_freq());
	
	return ret;
}
	
static ssize_t mali_s5_freq_show(struct device *dev,
									struct device_attribute *attr, char *buf)
{
	int ret;
	
	ret = sprintf(buf, "%d\n", get_mali_freq_by_step(5));
	
	return ret;
}

static ssize_t mali_s5_freq_store(struct device *dev,
									 struct device_attribute *attr, const char *buf, size_t size)
{
	int data, ret;
	
	ret = sscanf(buf, "%d\n", &data);
	if (unlikely(ret != 1 || data < 0 || data > 900)) {
		return -EINVAL;
	}
	
	set_mali_step_freq(5, data);
	//pr_info("[Touchkey] mali step 5 freq set to %d\n", data);
	
	return size;
}
	
static ssize_t mali_s5_volt_show(struct device *dev,
								 struct device_attribute *attr, char *buf)
{
	int ret;
	
	ret = sprintf(buf, "%d\n", get_mali_volt_by_step(5));
	
	return ret;
}

static ssize_t mali_s5_volt_store(struct device *dev,
								  struct device_attribute *attr, const char *buf, size_t size)
{
	int data, ret;
	
	ret = sscanf(buf, "%d\n", &data);
	if (unlikely(ret != 1 || data < 0 || data > 1300000)) {
		return -EINVAL;
	}
	
	set_mali_step_volt(5, data);
	//pr_info("[Touchkey] mali step 5 volt set to %d\n", data);
	
	return size;
}

static ssize_t mali_s4_freq_show(struct device *dev,
								 struct device_attribute *attr, char *buf)
{
	int ret;
	
	ret = sprintf(buf, "%d\n", get_mali_freq_by_step(4));
	
	return ret;
}

static ssize_t mali_s4_freq_store(struct device *dev,
								  struct device_attribute *attr, const char *buf, size_t size)
{
	int data, ret;
	
	ret = sscanf(buf, "%d\n", &data);
	if (unlikely(ret != 1 || data < 0 || data > 900)) {
		return -EINVAL;
	}
	
	set_mali_step_freq(4, data);
	//pr_info("[Touchkey] mali step 4 freq set to %d\n", data);
	
	return size;
}
	
static ssize_t mali_s4_volt_show(struct device *dev,
								 struct device_attribute *attr, char *buf)
{
	int ret;
	
	ret = sprintf(buf, "%d\n", get_mali_volt_by_step(4));
	
	return ret;
}

static ssize_t mali_s4_volt_store(struct device *dev,
								  struct device_attribute *attr, const char *buf, size_t size)
{
	int data, ret;
	
	ret = sscanf(buf, "%d\n", &data);
	if (unlikely(ret != 1 || data < 0 || data > 1300000)) {
		return -EINVAL;
	}
	
	set_mali_step_volt(4, data);
	//pr_info("[Touchkey] mali step 4 volt set to %d\n", data);
	
	return size;
}
	
static ssize_t mali_s3_freq_show(struct device *dev,
								 struct device_attribute *attr, char *buf)
{
	int ret;
	
	ret = sprintf(buf, "%d\n", get_mali_freq_by_step(3));
	
	return ret;
}

static ssize_t mali_s3_freq_store(struct device *dev,
								  struct device_attribute *attr, const char *buf, size_t size)
{
	int data, ret;
	
	ret = sscanf(buf, "%d\n", &data);
	if (unlikely(ret != 1 || data < 0 || data > 900)) {
		return -EINVAL;
	}
	
	set_mali_step_freq(3, data);
	//pr_info("[Touchkey] mali step 3 freq set to %d\n", data);
	
	return size;
}
	
static ssize_t mali_s3_volt_show(struct device *dev,
								 struct device_attribute *attr, char *buf)
{
	int ret;
	
	ret = sprintf(buf, "%d\n", get_mali_volt_by_step(3));
	
	return ret;
}

static ssize_t mali_s3_volt_store(struct device *dev,
								  struct device_attribute *attr, const char *buf, size_t size)
{
	int data, ret;
	
	ret = sscanf(buf, "%d\n", &data);
	if (unlikely(ret != 1 || data < 0 || data > 1300000)) {
		return -EINVAL;
	}
	
	set_mali_step_volt(3, data);
	//pr_info("[Touchkey] mali step 3 volt set to %d\n", data);
	
	return size;
}

static ssize_t mali_s2_freq_show(struct device *dev,
								 struct device_attribute *attr, char *buf)
{
	int ret;
	
	ret = sprintf(buf, "%d\n", get_mali_freq_by_step(2));
	
	return ret;
}

static ssize_t mali_s2_freq_store(struct device *dev,
								  struct device_attribute *attr, const char *buf, size_t size)
{
	int data, ret;
	
	ret = sscanf(buf, "%d\n", &data);
	if (unlikely(ret != 1 || data < 0 || data > 900)) {
		return -EINVAL;
	}
	
	set_mali_step_freq(2, data);
	//pr_info("[Touchkey] mali step 2 freq set to %d\n", data);
	
	return size;
}
	
static ssize_t mali_s2_volt_show(struct device *dev,
								 struct device_attribute *attr, char *buf)
{
	int ret;
	
	ret = sprintf(buf, "%d\n", get_mali_volt_by_step(2));
	
	return ret;
}

static ssize_t mali_s2_volt_store(struct device *dev,
								  struct device_attribute *attr, const char *buf, size_t size)
{
	int data, ret;
	
	ret = sscanf(buf, "%d\n", &data);
	if (unlikely(ret != 1 || data < 0 || data > 1300000)) {
		return -EINVAL;
	}
	
	set_mali_step_volt(2, data);
	//pr_info("[Touchkey] mali step 2 volt set to %d\n", data);
	
	return size;
}
	
static ssize_t mali_s1_freq_show(struct device *dev,
								 struct device_attribute *attr, char *buf)
{
	int ret;
	
	ret = sprintf(buf, "%d\n", get_mali_freq_by_step(1));
	
	return ret;
}

static ssize_t mali_s1_freq_store(struct device *dev,
								  struct device_attribute *attr, const char *buf, size_t size)
{
	int data, ret;
	
	ret = sscanf(buf, "%d\n", &data);
	if (unlikely(ret != 1 || data < 0 || data > 900)) {
		return -EINVAL;
	}
	
	set_mali_step_freq(1, data);
	//pr_info("[Touchkey] mali step 1 freq set to %d\n", data);
	
	return size;
}
	
static ssize_t mali_s1_volt_show(struct device *dev,
								 struct device_attribute *attr, char *buf)
{
	int ret;
	
	ret = sprintf(buf, "%d\n", get_mali_volt_by_step(1));
	
	return ret;
}

static ssize_t mali_s1_volt_store(struct device *dev,
								  struct device_attribute *attr, const char *buf, size_t size)
{
	int data, ret;
	
	ret = sscanf(buf, "%d\n", &data);
	if (unlikely(ret != 1 || data < 0 || data > 1300000)) {
		return -EINVAL;
	}
	
	set_mali_step_volt(1, data);
	//pr_info("[Touchkey] mali step 1 volt set to %d\n", data);
	
	return size;
}
	
static ssize_t mali_step_lock_show(struct device *dev,
									struct device_attribute *attr, char *buf)
{
	int ret;
	
	ret = sprintf(buf, "%d\n", get_mali_step_lock());
	
	return ret;
}

static ssize_t mali_step_lock_store(struct device *dev,
									 struct device_attribute *attr, const char *buf, size_t size)
{
	int data, ret;
	
	ret = sscanf(buf, "%d\n", &data);
	if (unlikely(ret != 1 || data < 0 || data > 5)) {
		return -EINVAL;
	}
	
	set_mali_step_lock(data);
	pr_info("[Touchkey] mali lock set to %d\n", data);
	
	return size;
}
	
static ssize_t mali_step_limit_show(struct device *dev,
							 struct device_attribute *attr, char *buf)
{
	int ret;
	
	ret = sprintf(buf, "%d\n", get_mali_step_limit());
	
	return ret;
}

static ssize_t mali_step_limit_store(struct device *dev,
							  struct device_attribute *attr, const char *buf, size_t size)
{
	int data, ret;
	
	ret = sscanf(buf, "%d\n", &data);
	if (unlikely(ret != 1 || data < 0 || data > 5)) {
		return -EINVAL;
	}
	
	set_mali_step_limit(data);
	//pr_info("[Touchkey] mali limit set to %d\n", data);
	
	return size;
}

/*
==========================================================================================
==========================================================================================
*/

static ssize_t mali_s5_down_threshold_show(struct device *dev,
									struct device_attribute *attr, char *buf)
{
	int ret;
	
	ret = sprintf(buf, "%d\n", get_mali_down_threshold_by_step(5));
	
	return ret;
}

static ssize_t mali_s5_down_threshold_store(struct device *dev,
									 struct device_attribute *attr, const char *buf, size_t size)
{
	int data, ret;
	
	ret = sscanf(buf, "%d\n", &data);
	if (unlikely(ret != 1 || data < 0 || data > 100)) {
		return -EINVAL;
	}
	
	set_mali_down_threshold_by_step(5, data);
	
	return size;
}

/* --- */

static ssize_t mali_s4_up_threshold_show(struct device *dev,
									struct device_attribute *attr, char *buf)
{
	int ret;
	
	ret = sprintf(buf, "%d\n", get_mali_up_threshold_by_step(4));
	
	return ret;
}

static ssize_t mali_s4_up_threshold_store(struct device *dev,
									 struct device_attribute *attr, const char *buf, size_t size)
{
	int data, ret;
	
	ret = sscanf(buf, "%d\n", &data);
	if (unlikely(ret != 1 || data < 0 || data > 100)) {
		return -EINVAL;
	}
	
	set_mali_up_threshold_by_step(4, data);
	
	return size;
}

static ssize_t mali_s4_down_threshold_show(struct device *dev,
									struct device_attribute *attr, char *buf)
{
	int ret;
	
	ret = sprintf(buf, "%d\n", get_mali_down_threshold_by_step(4));
	
	return ret;
}

static ssize_t mali_s4_down_threshold_store(struct device *dev,
									 struct device_attribute *attr, const char *buf, size_t size)
{
	int data, ret;
	
	ret = sscanf(buf, "%d\n", &data);
	if (unlikely(ret != 1 || data < 0 || data > 100)) {
		return -EINVAL;
	}
	
	set_mali_down_threshold_by_step(4, data);
	
	return size;
}

/* --- */

static ssize_t mali_s3_up_threshold_show(struct device *dev,
									struct device_attribute *attr, char *buf)
{
	int ret;
	
	ret = sprintf(buf, "%d\n", get_mali_up_threshold_by_step(3));
	
	return ret;
}

static ssize_t mali_s3_up_threshold_store(struct device *dev,
									 struct device_attribute *attr, const char *buf, size_t size)
{
	int data, ret;
	
	ret = sscanf(buf, "%d\n", &data);
	if (unlikely(ret != 1 || data < 0 || data > 100)) {
		return -EINVAL;
	}
	
	set_mali_up_threshold_by_step(3, data);
	
	return size;
}

static ssize_t mali_s3_down_threshold_show(struct device *dev,
									struct device_attribute *attr, char *buf)
{
	int ret;
	
	ret = sprintf(buf, "%d\n", get_mali_down_threshold_by_step(3));
	
	return ret;
}

static ssize_t mali_s3_down_threshold_store(struct device *dev,
									 struct device_attribute *attr, const char *buf, size_t size)
{
	int data, ret;
	
	ret = sscanf(buf, "%d\n", &data);
	if (unlikely(ret != 1 || data < 0 || data > 100)) {
		return -EINVAL;
	}
	
	set_mali_down_threshold_by_step(3, data);
	
	return size;
}

/* --- */

static ssize_t mali_s2_up_threshold_show(struct device *dev,
									struct device_attribute *attr, char *buf)
{
	int ret;
	
	ret = sprintf(buf, "%d\n", get_mali_up_threshold_by_step(2));
	
	return ret;
}

static ssize_t mali_s2_up_threshold_store(struct device *dev,
									 struct device_attribute *attr, const char *buf, size_t size)
{
	int data, ret;
	
	ret = sscanf(buf, "%d\n", &data);
	if (unlikely(ret != 1 || data < 0 || data > 100)) {
		return -EINVAL;
	}
	
	set_mali_up_threshold_by_step(2, data);
	
	return size;
}

static ssize_t mali_s2_down_threshold_show(struct device *dev,
									struct device_attribute *attr, char *buf)
{
	int ret;
	
	ret = sprintf(buf, "%d\n", get_mali_down_threshold_by_step(2));
	
	return ret;
}

static ssize_t mali_s2_down_threshold_store(struct device *dev,
									 struct device_attribute *attr, const char *buf, size_t size)
{
	int data, ret;
	
	ret = sscanf(buf, "%d\n", &data);
	if (unlikely(ret != 1 || data < 0 || data > 100)) {
		return -EINVAL;
	}
	
	set_mali_down_threshold_by_step(2, data);
	
	return size;
}

/* --- */

static ssize_t mali_s1_up_threshold_show(struct device *dev,
									struct device_attribute *attr, char *buf)
{
	int ret;
	
	ret = sprintf(buf, "%d\n", get_mali_up_threshold_by_step(1));
	
	return ret;
}

static ssize_t mali_s1_up_threshold_store(struct device *dev,
									 struct device_attribute *attr, const char *buf, size_t size)
{
	int data, ret;
	
	ret = sscanf(buf, "%d\n", &data);
	if (unlikely(ret != 1 || data < 0 || data > 100)) {
		return -EINVAL;
	}
	
	set_mali_up_threshold_by_step(1, data);
	
	return size;
}

/* ---------------------
---------------------------
*/

/*static ssize_t mali_time_in_state_show(struct device *dev,
									struct device_attribute *attr, char *buf)
{
	int i, len = 0;

	for (i = 4; i >= 0; i--) {
		len += sprintf(buf + len, "Step %d:\t%llu\n", (i + 1), get_mali_time_in_state(i + 1));
	}

	return len;
}

static ssize_t mali_time_in_state_store(struct device *dev,
									 struct device_attribute *attr, const char *buf, size_t size)
{
	int data, ret;
	
	ret = sscanf(buf, "%d\n", &data);
	if (unlikely(ret != 1 || data < 0 || data > 2)) {
		return -EINVAL;
	}
	
	set_mali_time_in_state(data);
	
	return size;
}*/

static ssize_t mali_utilization_timeout_show(struct device *dev,
									struct device_attribute *attr, char *buf)
{
	int ret;
	
	ret = sprintf(buf, "%d\n", get_mali_utilization_timeout());
	
	return ret;
}

static ssize_t mali_utilization_timeout_store(struct device *dev,
									 struct device_attribute *attr, const char *buf, size_t size)
{
	int data, ret;
	
	ret = sscanf(buf, "%d\n", &data);
	if (unlikely(ret != 1 || data < 0 || data > 5000)) {
		return -EINVAL;
	}
	
	set_mali_utilization_timeout(data);
	
	return size;
}

static DEVICE_ATTR(recommended_version, S_IRUGO | S_IWUSR | S_IWGRP,
		   touch_version_read, touch_version_write);
static DEVICE_ATTR(updated_version, S_IRUGO | S_IWUSR | S_IWGRP,
		   touch_update_read, touch_update_write);
static DEVICE_ATTR(brightness, S_IRUGO | S_IWUSR | S_IWGRP, touchkey_led_control_show,
		   touchkey_led_control);
static DEVICE_ATTR(touchkey_menu, S_IRUGO | S_IWUSR | S_IWGRP,
		   touchkey_menu_show, NULL);
static DEVICE_ATTR(touchkey_back, S_IRUGO | S_IWUSR | S_IWGRP,
		   touchkey_back_show, NULL);

// Yank555.lu : touch_led_on_screen_touch : only accept feedback from touchscreen driver if enabled
static DEVICE_ATTR(touch_led_on_screen_touch, S_IRUGO | S_IWUSR | S_IWGRP,
		   touch_led_on_screen_touch_show, touch_led_on_screen_touch_store);

// Yank555.lu : touch_led_handling : have kernel (older CM) or ROM (newer CM) handle h/w keys backlight
static DEVICE_ATTR(touch_led_handling, S_IRUGO | S_IWUSR | S_IWGRP,
		   touch_led_handling_show, touch_led_handling_store);

#if defined(TK_USE_4KEY)
static DEVICE_ATTR(touchkey_home, S_IRUGO, touchkey_home_show, NULL);
static DEVICE_ATTR(touchkey_search, S_IRUGO, touchkey_search_show, NULL);
#endif

static DEVICE_ATTR(touch_sensitivity, S_IRUGO | S_IWUSR | S_IWGRP, NULL,
		   touch_sensitivity_control);
static DEVICE_ATTR(touchkey_firm_update, S_IRUGO | S_IWUSR | S_IWGRP,
	set_touchkey_update_show, NULL);
static DEVICE_ATTR(touchkey_firm_update_status, S_IRUGO | S_IWUSR | S_IWGRP,
	set_touchkey_firm_status_show, NULL);
static DEVICE_ATTR(touchkey_firm_version_phone, S_IRUGO | S_IWUSR | S_IWGRP,
	set_touchkey_firm_version_show, NULL);
static DEVICE_ATTR(touchkey_firm_version_panel, S_IRUGO | S_IWUSR | S_IWGRP,
		   set_touchkey_firm_version_read_show, NULL);
#ifdef LED_LDO_WITH_REGULATOR
static DEVICE_ATTR(touchkey_brightness, S_IRUGO | S_IWUSR | S_IWGRP, NULL,
		   brightness_control);
#endif

#if defined(CONFIG_TARGET_LOCALE_NAATT)
static DEVICE_ATTR(touchkey_autocal_start, S_IRUGO | S_IWUSR | S_IWGRP, NULL,
		   set_touchkey_autocal_testmode);
#endif

#if defined(TK_HAS_AUTOCAL)
static DEVICE_ATTR(touchkey_raw_data0, S_IRUGO, touchkey_raw_data0_show, NULL);
static DEVICE_ATTR(touchkey_raw_data1, S_IRUGO, touchkey_raw_data1_show, NULL);
static DEVICE_ATTR(touchkey_raw_data2, S_IRUGO, touchkey_raw_data2_show, NULL);
static DEVICE_ATTR(touchkey_raw_data3, S_IRUGO, touchkey_raw_data3_show, NULL);
static DEVICE_ATTR(touchkey_idac0, S_IRUGO, touchkey_idac0_show, NULL);
static DEVICE_ATTR(touchkey_idac1, S_IRUGO, touchkey_idac1_show, NULL);
static DEVICE_ATTR(touchkey_idac2, S_IRUGO, touchkey_idac2_show, NULL);
static DEVICE_ATTR(touchkey_idac3, S_IRUGO, touchkey_idac3_show, NULL);
static DEVICE_ATTR(touchkey_threshold, S_IRUGO, touchkey_threshold_show, NULL);
static DEVICE_ATTR(autocal_enable, S_IRUGO | S_IWUSR | S_IWGRP, NULL,
		   autocalibration_enable);
static DEVICE_ATTR(autocal_stat, S_IRUGO | S_IWUSR | S_IWGRP,
		   autocalibration_status, NULL);
#endif
#ifdef CONFIG_TOUCHSCREEN_GESTURES
static DEVICE_ATTR(gesture_delay, S_IRUGO | S_IWUSR | S_IWGRP,
					   gesture_delay_show, gesture_delay_store);
#endif
static DEVICE_ATTR(tk_key1_key_code, S_IRUGO | S_IWUSR | S_IWGRP, tk_key1_key_code_show, tk_key1_key_code_store);
static DEVICE_ATTR(tk_key2_key_code, S_IRUGO | S_IWUSR | S_IWGRP, tk_key2_key_code_show, tk_key2_key_code_store);
static DEVICE_ATTR(tk_mt_key1_mode, S_IRUGO | S_IWUSR | S_IWGRP, tk_mt_key1_mode_show, tk_mt_key1_mode_store);
static DEVICE_ATTR(tk_mt_key2_mode, S_IRUGO | S_IWUSR | S_IWGRP, tk_mt_key2_mode_show, tk_mt_key2_mode_store);
static DEVICE_ATTR(tk_mt_minimum_between, S_IRUGO | S_IWUSR | S_IWGRP, tk_mt_minimum_between_show, tk_mt_minimum_between_store);
static DEVICE_ATTR(tk_mt_maximum_between, S_IRUGO | S_IWUSR | S_IWGRP, tk_mt_maximum_between_show, tk_mt_maximum_between_store);
static DEVICE_ATTR(mali_asv, S_IRUGO | S_IWUSR | S_IWGRP,mali_asv_show, mali_asv_store);
static DEVICE_ATTR(mali_cur_freq, S_IRUGO, mali_cur_freq_show, NULL);
static DEVICE_ATTR(mali_step_lock, S_IRUGO | S_IWUSR | S_IWGRP, mali_step_lock_show, mali_step_lock_store);
static DEVICE_ATTR(mali_step_limit, S_IRUGO | S_IWUSR | S_IWGRP, mali_step_limit_show, mali_step_limit_store);
static DEVICE_ATTR(mali_s5_volt, S_IRUGO | S_IWUSR | S_IWGRP, mali_s5_volt_show, mali_s5_volt_store);
static DEVICE_ATTR(mali_s5_freq, S_IRUGO | S_IWUSR | S_IWGRP, mali_s5_freq_show, mali_s5_freq_store);
static DEVICE_ATTR(mali_s4_volt, S_IRUGO | S_IWUSR | S_IWGRP, mali_s4_volt_show, mali_s4_volt_store);
static DEVICE_ATTR(mali_s4_freq, S_IRUGO | S_IWUSR | S_IWGRP, mali_s4_freq_show, mali_s4_freq_store);
static DEVICE_ATTR(mali_s3_volt, S_IRUGO | S_IWUSR | S_IWGRP, mali_s3_volt_show, mali_s3_volt_store);
static DEVICE_ATTR(mali_s3_freq, S_IRUGO | S_IWUSR | S_IWGRP, mali_s3_freq_show, mali_s3_freq_store);
static DEVICE_ATTR(mali_s2_volt, S_IRUGO | S_IWUSR | S_IWGRP, mali_s2_volt_show, mali_s2_volt_store);
static DEVICE_ATTR(mali_s2_freq, S_IRUGO | S_IWUSR | S_IWGRP, mali_s2_freq_show, mali_s2_freq_store);
static DEVICE_ATTR(mali_s1_volt, S_IRUGO | S_IWUSR | S_IWGRP, mali_s1_volt_show, mali_s1_volt_store);
static DEVICE_ATTR(mali_s1_freq, S_IRUGO | S_IWUSR | S_IWGRP, mali_s1_freq_show, mali_s1_freq_store);

static DEVICE_ATTR(mali_s5_down_threshold, S_IRUGO | S_IWUSR | S_IWGRP, mali_s5_down_threshold_show, mali_s5_down_threshold_store);
static DEVICE_ATTR(mali_s4_up_threshold, S_IRUGO | S_IWUSR | S_IWGRP, mali_s4_up_threshold_show, mali_s4_up_threshold_store);
static DEVICE_ATTR(mali_s4_down_threshold, S_IRUGO | S_IWUSR | S_IWGRP, mali_s4_down_threshold_show, mali_s4_down_threshold_store);
static DEVICE_ATTR(mali_s3_up_threshold, S_IRUGO | S_IWUSR | S_IWGRP, mali_s3_up_threshold_show, mali_s3_up_threshold_store);
static DEVICE_ATTR(mali_s3_down_threshold, S_IRUGO | S_IWUSR | S_IWGRP, mali_s3_down_threshold_show, mali_s3_down_threshold_store);
static DEVICE_ATTR(mali_s2_up_threshold, S_IRUGO | S_IWUSR | S_IWGRP, mali_s2_up_threshold_show, mali_s2_up_threshold_store);
static DEVICE_ATTR(mali_s2_down_threshold, S_IRUGO | S_IWUSR | S_IWGRP, mali_s2_down_threshold_show, mali_s2_down_threshold_store);
static DEVICE_ATTR(mali_s1_up_threshold, S_IRUGO | S_IWUSR | S_IWGRP, mali_s1_up_threshold_show, mali_s1_up_threshold_store);

/*static DEVICE_ATTR(mali_time_in_state, S_IRUGO | S_IWUSR | S_IWGRP, mali_time_in_state_show, mali_time_in_state_store);*/
static DEVICE_ATTR(mali_utilization_timeout, S_IRUGO | S_IWUSR | S_IWGRP, mali_utilization_timeout_show, mali_utilization_timeout_store);


static struct attribute *touchkey_attributes[] = {
	&dev_attr_recommended_version.attr,
	&dev_attr_updated_version.attr,
	&dev_attr_brightness.attr,
	&dev_attr_touchkey_menu.attr,
	&dev_attr_touchkey_back.attr,
	&dev_attr_touch_led_on_screen_touch.attr, // Yank555.lu : touch_led_on_screen_touch : only accept feedback from touchscreen driver if enabled
	&dev_attr_touch_led_handling.attr, 	  // Yank555.lu : touch_led_handling : have kernel (older CM) or ROM (newer CM) handle h/w keys backlight
#if defined(TK_USE_4KEY)
	&dev_attr_touchkey_home.attr,
	&dev_attr_touchkey_search.attr,
#endif
	&dev_attr_touch_sensitivity.attr,
	&dev_attr_touchkey_firm_update.attr,
	&dev_attr_touchkey_firm_update_status.attr,
	&dev_attr_touchkey_firm_version_phone.attr,
	&dev_attr_touchkey_firm_version_panel.attr,
#ifdef LED_LDO_WITH_REGULATOR
	&dev_attr_touchkey_brightness.attr,
#endif
#if defined(CONFIG_TARGET_LOCALE_NAATT)
	&dev_attr_touchkey_autocal_start.attr,
#endif
#if defined(TK_HAS_AUTOCAL)
	&dev_attr_touchkey_raw_data0.attr,
	&dev_attr_touchkey_raw_data1.attr,
	&dev_attr_touchkey_raw_data2.attr,
	&dev_attr_touchkey_raw_data3.attr,
	&dev_attr_touchkey_idac0.attr,
	&dev_attr_touchkey_idac1.attr,
	&dev_attr_touchkey_idac2.attr,
	&dev_attr_touchkey_idac3.attr,
	&dev_attr_touchkey_threshold.attr,
	&dev_attr_autocal_enable.attr,
	&dev_attr_autocal_stat.attr,
#endif
	&dev_attr_timeout.attr,
    &dev_attr_force_disable.attr,
#ifdef CONFIG_TOUCHSCREEN_GESTURES
	&dev_attr_gesture_delay.attr,
#endif
	&dev_attr_tk_key1_key_code.attr,
	&dev_attr_tk_key2_key_code.attr,
	&dev_attr_tk_mt_key1_mode.attr,
	&dev_attr_tk_mt_key2_mode.attr,
	&dev_attr_tk_mt_minimum_between.attr,
	&dev_attr_tk_mt_maximum_between.attr,
	&dev_attr_mali_asv.attr,
	&dev_attr_mali_cur_freq.attr,
	&dev_attr_mali_step_lock.attr,
	&dev_attr_mali_step_limit.attr,
	&dev_attr_mali_s5_volt.attr,
	&dev_attr_mali_s5_freq.attr,
	&dev_attr_mali_s4_volt.attr,
	&dev_attr_mali_s4_freq.attr,
	&dev_attr_mali_s3_volt.attr,
	&dev_attr_mali_s3_freq.attr,
	&dev_attr_mali_s2_volt.attr,
	&dev_attr_mali_s2_freq.attr,
	&dev_attr_mali_s1_volt.attr,
	&dev_attr_mali_s1_freq.attr,
	&dev_attr_mali_s5_down_threshold.attr,
	&dev_attr_mali_s4_up_threshold.attr,
	&dev_attr_mali_s4_down_threshold.attr,
	&dev_attr_mali_s3_up_threshold.attr,
	&dev_attr_mali_s3_down_threshold.attr,
	&dev_attr_mali_s2_up_threshold.attr,
	&dev_attr_mali_s2_down_threshold.attr,
	&dev_attr_mali_s1_up_threshold.attr,
	//&dev_attr_mali_time_in_state.attr,
	&dev_attr_mali_utilization_timeout.attr,
	NULL,
};

static struct attribute_group touchkey_attr_group = {
	.attrs = touchkey_attributes,
};

static int i2c_touchkey_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct touchkey_platform_data *pdata = client->dev.platform_data;
	struct touchkey_i2c *tkey_i2c;

	struct input_dev *input_dev;
	int err = 0;
	unsigned char data;
	int i;
	int ret;

	printk(KERN_DEBUG "[TouchKey] i2c_touchkey_probe\n");

	if (pdata == NULL) {
		printk(KERN_ERR "%s: no pdata\n", __func__);
		return -ENODEV;
	}

	/*Check I2C functionality */
	ret = i2c_check_functionality(client->adapter, I2C_FUNC_I2C);
	if (ret == 0) {
		printk(KERN_ERR "[Touchkey] No I2C functionality found\n");
		ret = -ENODEV;
		return ret;
	}

	/*Obtain kernel memory space for touchkey i2c */
	tkey_i2c = kzalloc(sizeof(struct touchkey_i2c), GFP_KERNEL);
	if (NULL == tkey_i2c) {
		printk(KERN_ERR "[Touchkey] failed to allocate tkey_i2c.\n");
		return -ENOMEM;
	}
	tkey_i2c_local = tkey_i2c;

	input_dev = input_allocate_device();

	if (!input_dev) {
		printk(KERN_ERR"[Touchkey] failed to allocate input device\n");
		kfree(tkey_i2c);
		return -ENOMEM;
	}

	input_dev->name = "sec_touchkey";
	input_dev->phys = "sec_touchkey/input0";
	input_dev->id.bustype = BUS_HOST;
	input_dev->dev.parent = &client->dev;

	/*tkey_i2c*/
	tkey_i2c->pdata = pdata;
	tkey_i2c->input_dev = input_dev;
	tkey_i2c->client = client;
	tkey_i2c->irq = client->irq;
	tkey_i2c->name = "sec_touchkey";

	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_LED, input_dev->evbit);
	set_bit(LED_MISC, input_dev->ledbit);
	set_bit(EV_KEY, input_dev->evbit);

	for (i = 1; i < touchkey_count; i++)
		set_bit(touchkey_keycode[i], input_dev->keybit);

	input_set_drvdata(input_dev, tkey_i2c);

	ret = input_register_device(input_dev);
	if (ret) {
		printk(KERN_ERR"[Touchkey] failed to register input device\n");
		input_free_device(input_dev);
		kfree(tkey_i2c);
		return err;
	}

	INIT_WORK(&tkey_i2c->update_work, touchkey_update_func);

	tkey_i2c->pdata->power_on(1);
	msleep(50);

	touchkey_enable = 1;
	data = 1;

	/*sysfs*/
	tkey_i2c->dev = device_create(sec_class, NULL, 0, NULL, "sec_touchkey");

	if (IS_ERR(tkey_i2c->dev)) {
		printk(KERN_ERR "Failed to create device(tkey_i2c->dev)!\n");
		input_unregister_device(input_dev);
	} else {
		dev_set_drvdata(tkey_i2c->dev, tkey_i2c);
		ret = sysfs_create_group(&tkey_i2c->dev->kobj,
					&touchkey_attr_group);
		if (ret) {
			printk(KERN_ERR
				"[TouchKey]: failed to create sysfs group\n");
		}
	}

#if defined(CONFIG_MACH_M0) || defined(CONFIG_MACH_C1)
	gpio_request(GPIO_OLED_DET, "OLED_DET");
	ret = gpio_get_value(GPIO_OLED_DET);
	printk(KERN_DEBUG
	"[TouchKey] OLED_DET = %d\n", ret);

	if (ret == 0) {
		printk(KERN_DEBUG
		"[TouchKey] device wasn't connected to board\n");

		input_unregister_device(input_dev);
		touchkey_probe = false;
		return -EBUSY;
	}
#else
	ret = touchkey_i2c_check(tkey_i2c);
	if (ret < 0) {
		printk(KERN_DEBUG"[TouchKey] probe failed\n");
		input_unregister_device(input_dev);
		touchkey_probe = false;
		return -EBUSY;
	}
#endif

	ret =
		request_threaded_irq(tkey_i2c->irq, NULL, touchkey_interrupt,
				IRQF_DISABLED | IRQF_TRIGGER_FALLING |
				IRQF_ONESHOT, tkey_i2c->name, tkey_i2c);
	if (ret < 0) {
		printk(KERN_ERR
			"[Touchkey]: failed to request irq(%d) - %d\n",
			tkey_i2c->irq, ret);
		input_unregister_device(input_dev);
		touchkey_probe = false;
		return -EBUSY;
	}

	tkey_i2c->pdata->led_power_on(1);

#if defined(TK_HAS_FIRMWARE_UPDATE)
	ret = touchkey_firmware_update(tkey_i2c);
	if (ret < 0) {
		printk(KERN_ERR
			"[Touchkey]: failed firmware updating process (%d)\n",
			ret);
		input_unregister_device(input_dev);
		touchkey_probe = false;
		return -EBUSY;
	}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	tkey_i2c->early_suspend.suspend =
		(void *)sec_touchkey_early_suspend;
	tkey_i2c->early_suspend.resume =
		(void *)sec_touchkey_late_resume;
	register_early_suspend(&tkey_i2c->early_suspend);
#endif

#ifdef CONFIG_TOUCH_WAKE
	touchwakedevdata = tkey_i2c;
	register_touchwake_implementation(&cypress_touchwake);
#endif
	
	input_dev_tk = input_dev;

#if defined(TK_HAS_AUTOCAL)
	touchkey_autocalibration(tkey_i2c);
#endif
	set_touchkey_debug('K');

    // init workqueue
    tkey_i2c->wq = create_singlethread_workqueue("tkey_i2c_wq");
    if (!tkey_i2c->wq) {
        ret = -ENOMEM;
        pr_err("%s: could not create workqueue\n", __func__);
    }

    /* this is the thread function we run on the work queue */
	INIT_WORK(&tkey_i2c->work, touch_led_timedout_work);

	return 0;
}

struct i2c_driver touchkey_i2c_driver = {
	.driver = {
		.name = "sec_touchkey_driver",
	},
	.id_table = sec_touchkey_id,
	.probe = i2c_touchkey_probe,
};

static int __init touchkey_init(void)
{
	int ret = 0;

#if defined(CONFIG_MACH_M0)
	if (system_rev < TOUCHKEY_FW_UPDATEABLE_HW_REV) {
		printk(KERN_DEBUG "[Touchkey] Doesn't support this board rev %d\n",
				system_rev);
		return 0;
	}
#elif defined(CONFIG_MACH_C1)
	if (system_rev < TOUCHKEY_FW_UPDATEABLE_HW_REV) {
		printk(KERN_DEBUG "[Touchkey] Doesn't support this board rev %d\n",
				system_rev);
		return 0;
	}
#endif

#ifdef TEST_JIG_MODE
	unsigned char get_touch = 0x40;
#endif

	ret = i2c_add_driver(&touchkey_i2c_driver);

	if (ret) {
		printk(KERN_ERR
	       "[TouchKey] registration failed, module not inserted.ret= %d\n",
	       ret);
	}
#ifdef TEST_JIG_MODE
	i2c_touchkey_write(tkey_i2c->client, &get_touch, 1);
#endif

    // init the touchled timer
    init_timer(&touch_led_timer);
    touch_led_timer.function = touch_led_timedout;

	return ret;
}

static void __exit touchkey_exit(void)
{
	printk(KERN_DEBUG "[TouchKey] %s\n", __func__);
	i2c_del_driver(&touchkey_i2c_driver);
}

late_initcall(touchkey_init);
module_exit(touchkey_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("@@@");
MODULE_DESCRIPTION("touch keypad");
