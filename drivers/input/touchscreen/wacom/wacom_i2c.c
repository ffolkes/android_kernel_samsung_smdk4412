/*
 *  wacom_i2c.c - Wacom G5 Digitizer Controller (I2C bus)
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

#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/wacom_i2c.h>
#include <linux/earlysuspend.h>
#include <linux/uaccess.h>
#include <linux/firmware.h>
#include "wacom_i2c_func.h"
#include "wacom_i2c_flash.h"
#ifdef WACOM_IMPORT_FW_ALGO
#include "wacom_i2c_coord_tables.h"
#endif

#define WACOM_UMS_UPDATE
#define WACOM_FW_PATH "/sdcard/firmware/wacom_firm.bin"

extern bool flg_tsp_touching;
extern int flg_ctr_tsp_touching;
extern bool flg_epen_pressed;

unsigned char screen_rotate;
unsigned char user_hand = 1;
static bool epen_reset_result;

bool flg_epen_hovering = false;

static bool pen_insert_state;
static bool firmware_updating_state;

unsigned int sttg_boxfilter_threshold = 0;
unsigned int sttg_boxfilter_threshold_hover = 30;
unsigned int sttg_boost_freq = 400000;
unsigned int sttg_min_pressure = 0;
unsigned int sttg_side_button_mode = 0;
unsigned int sttg_fixed_pressure = 0;
unsigned int sttg_avg_strength = 2;
int sttg_offset_override_x = 0;
int sttg_offset_override_y = 0;
bool sttg_offset_override_enabled = false;
unsigned int sttg_touchkey_block_length = 1500;
bool sttg_epen_worryfree_mode = false;
bool flg_epen_worryfree_mode = false;

unsigned int sttg_epen_hover_side_hotspot_height = (WACOM_MAX_COORD_X - 1000);
unsigned int sttg_epen_hover_side_left_keycode = KEY_MENU;
bool sttg_epen_hover_side_left_keydelay = false;
unsigned int sttg_epen_hover_side_center_keycode = KEY_HOME;
bool sttg_epen_hover_side_center_keydelay = false;
unsigned int sttg_epen_hover_side_right_keycode = KEY_BACK;
bool sttg_epen_hover_side_right_keydelay = false;

unsigned int sttg_epen_padding_top = 100;
unsigned int sttg_epen_padding_right = (WACOM_MAX_COORD_Y - 100);
unsigned int sttg_epen_padding_bottom = (WACOM_MAX_COORD_X - 100);
unsigned int sttg_epen_padding_left = 100;

static void wacom_i2c_enable_irq(struct wacom_i2c *wac_i2c, bool enable)
{
	static int depth;

	if (enable) {
		if (depth) {
			--depth;
			enable_irq(wac_i2c->irq);
#ifdef WACOM_PDCT_WORK_AROUND
			enable_irq(wac_i2c->irq_pdct);
#endif
		}
	} else {
		if (!depth) {
			++depth;
			disable_irq(wac_i2c->irq);
#ifdef WACOM_PDCT_WORK_AROUND
			disable_irq(wac_i2c->irq_pdct);
#endif
		}
	}

#ifdef WACOM_IRQ_DEBUG
	printk(KERN_DEBUG"[E-PEN] Enable %d, depth %d\n", (int)enable, depth);
#endif
}

static void wacom_i2c_reset_hw(struct wacom_g5_platform_data *wac_pdata)
{
	/* Reset IC */
	wac_pdata->suspend_platform_hw();
	msleep(200);
	wac_pdata->resume_platform_hw();
	msleep(200);
}

static void wacom_i2c_enable(struct wacom_i2c *wac_i2c)
{
	bool en = true;

#ifdef BATTERY_SAVING_MODE
	if (wac_i2c->battery_saving_mode
		&& wac_i2c->pen_insert)
		en = false;
#endif

	if (en) {
		wac_i2c->wac_pdata->late_resume_platform_hw();
		schedule_delayed_work(&wac_i2c->resume_work, HZ / 5);
	}
}

static void wacom_i2c_disable(struct wacom_i2c *wac_i2c)
{
	if (wac_i2c->power_enable) {
		wacom_i2c_enable_irq(wac_i2c, false);

		/* release pen, if it is pressed */
#if defined(CONFIG_MACH_P4NOTE)
#ifdef WACOM_PDCT_WORK_AROUND
		if (wac_i2c->pen_pdct == PDCT_DETECT_PEN)
#else
		if (wac_i2c->pen_pressed || wac_i2c->side_pressed
			|| wac_i2c->pen_prox)
#endif
#else
		if (wac_i2c->pen_pressed || wac_i2c->side_pressed
			|| wac_i2c->pen_prox)
#endif
			forced_release(wac_i2c);

		if (!wake_lock_active(&wac_i2c->wakelock)) {
			wac_i2c->power_enable = false;
			wac_i2c->wac_pdata->early_suspend_platform_hw();
		}
	}
}

int wacom_i2c_get_ums_data(struct wacom_i2c *wac_i2c, u8 **ums_data)
{
	struct file *fp;
	mm_segment_t old_fs;
	long fsize, nread;
	int ret = 0;
	unsigned int nSize;

#if defined(CONFIG_MACH_P4NOTE)
	nSize = MAX_ADDR_514 + 1;
#else
	nSize = WACOM_FW_SIZE;
#endif

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	fp = filp_open(WACOM_FW_PATH, O_RDONLY, S_IRUSR);

	if (IS_ERR(fp)) {
		printk(KERN_ERR "[E-PEN] failed to open %s.\n", WACOM_FW_PATH);
		ret = -ENOENT;
		set_fs(old_fs);
		return ret;
	}

	fsize = fp->f_path.dentry->d_inode->i_size;
	printk(KERN_NOTICE
		"[E-PEN] start, file path %s, size %ld Bytes\n",
		WACOM_FW_PATH, fsize);

	if (fsize != nSize) {
		printk(KERN_ERR "[E-PEN] UMS firmware size is different\n");
		ret = -EFBIG;
		goto size_error;
	}

	*ums_data = kmalloc(fsize, GFP_KERNEL);
	if (IS_ERR(*ums_data)) {
		printk(KERN_ERR
			"[E-PEN] %s, kmalloc failed\n", __func__);
		ret = -EFAULT;
		goto malloc_error;
	}

	nread = vfs_read(fp, (char __user *)*ums_data,
		fsize, &fp->f_pos);
	printk(KERN_NOTICE "[E-PEN] nread %ld Bytes\n", nread);
	if (nread != fsize) {
		printk(KERN_ERR
			"[E-PEN] failed to read firmware file, nread %ld Bytes\n",
			nread);
		ret = -EIO;
		kfree(*ums_data);
		goto read_err;
	}

	filp_close(fp, current->files);
	set_fs(old_fs);

	return 0;

 read_err:
 malloc_error:
 size_error:
	filp_close(fp, current->files);
 open_err:
	set_fs(old_fs);
	return ret;
}

int wacom_i2c_fw_update_UMS(struct wacom_i2c *wac_i2c)
{
	int ret = 0;
	u8 *ums_data = NULL;

	printk(KERN_ERR "[E-PEN] Start firmware flashing (UMS).\n");

	/*read firmware data*/
	ret = wacom_i2c_get_ums_data(wac_i2c, &ums_data);
	if (ret < 0) {
		printk(KERN_DEBUG"[E-PEN] file op is failed\n");
		return 0;
	}

	/*start firm update*/
	wacom_i2c_set_firm_data(ums_data);

	ret = wacom_i2c_flash(wac_i2c);
	if (ret < 0) {
		printk(KERN_ERR
			"[E-PEN] failed to write firmware(%d)\n", ret);
		kfree(ums_data);
		wacom_i2c_set_firm_data(NULL);
		return ret;
	}

	wacom_i2c_set_firm_data(NULL);
	kfree(ums_data);

	return 0;
}

#if defined(CONFIG_MACH_Q1_BD) || defined(CONFIG_MACH_T0)
int wacom_i2c_firm_update(struct wacom_i2c *wac_i2c)
{
	int ret;
	int retry = 3;
	const struct firmware *firm_data = NULL;

	firmware_updating_state = true;

	while (retry--) {
		ret =
		    request_firmware(&firm_data, firmware_name,
				     &wac_i2c->client->dev);
		if (ret < 0) {
			printk(KERN_ERR
			       "[E-PEN] Unable to open firmware. ret %d retry %d\n",
			       ret, retry);
			continue;
		}
		wacom_i2c_set_firm_data((unsigned char *)firm_data->data);

		ret = wacom_i2c_flash(wac_i2c);

		if (ret == 0) {
			release_firmware(firm_data);
			break;
		}

		printk(KERN_ERR "[E-PEN] failed to write firmware(%d)\n", ret);
		release_firmware(firm_data);

		/* Reset IC */
		wacom_i2c_reset_hw(wac_i2c->wac_pdata);
	}

	firmware_updating_state = false;

	if (ret < 0)
		return -1;

	return 0;
}
#endif

#if defined(CONFIG_MACH_P4NOTE)
static bool epen_check_factory_mode(void)
{
	struct file *fp;
	mm_segment_t old_fs;
	long fsize;
	int err = 0;
	int nread = 0;
	char *buf;
	bool ret = false;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	fp = filp_open("/efs/FactoryApp/factorymode",
			O_RDONLY, S_IRUSR);

	if (IS_ERR(fp)) {
		err = PTR_ERR(fp);
		if (err == -ENOENT)
			pr_err("[E-PEN] There is no file\n");
		else
			pr_err("[E-PEN] File open error(%d)\n", err);

		ret = false;
		goto out;
	}

	fsize = fp->f_path.dentry->d_inode->i_size;

	if (!fsize) {
		pr_err("[E-PEN] File size os zero\n");
		ret = false;
		goto err_filesize;
	}

	buf = kzalloc(fsize, GFP_KERNEL);
	if (!buf) {
		pr_err("[E-PEN] Memory allocate failed\n");
		ret = false;
		goto err_filesize;
	}

	nread = vfs_read(fp, (char __user *)buf, fsize, &fp->f_pos);
	if (nread != fsize) {
		pr_err("[E-PEN] File size[%ld] not same with read one[%d]\n",
				fsize, nread);
		ret = false;
		goto err_readfail;
	}

	/*
	* if the factory mode is disable,
	* do not update the firmware.
	*	factory mode : ON -> disable
	*	factory mode : OFF -> enable
	*/
	if (strncmp("ON", buf, 2))
		ret = true;

	printk(KERN_DEBUG "[E-PEN] Factorymode is %s\n", ret ? "ENG" : "USER");

err_readfail:
	kfree(buf);
err_filesize:
	filp_close(fp, current->files);
out:
	set_fs(old_fs);
	return ret;
}

static void update_fw_p4(struct wacom_i2c *wac_i2c)
{
	int ret = 0;
	int retry = 2;

	/* the firmware should be updated in factory mode durring the boot */
	if (!epen_check_factory_mode())
		retry = 0;

	while (retry--) {
		printk(KERN_DEBUG "[E-PEN] INIT_FIRMWARE_FLASH is enabled.\n");
		ret = wacom_i2c_flash(wac_i2c);
		if (ret == 0)
			break;

		printk(KERN_DEBUG "[E-PEN] update failed. retry %d, ret %d\n",
		       retry, ret);

		/* Reset IC */
		wacom_i2c_reset_hw(wac_i2c->wac_pdata);
	}

	printk(KERN_DEBUG "[E-PEN] flashed.(%d)\n", ret);
}
#endif

static irqreturn_t wacom_interrupt(int irq, void *dev_id)
{
	struct wacom_i2c *wac_i2c = dev_id;
	
	//printk(KERN_DEBUG "[E-PEN] touching: %d, touching ctr: %i\n", flg_tsp_touching, flg_ctr_tsp_touching);
	wacom_i2c_coord(wac_i2c);
	return IRQ_HANDLED;
}

#if defined(WACOM_PDCT_WORK_AROUND)
static irqreturn_t wacom_interrupt_pdct(int irq, void *dev_id)
{
	struct wacom_i2c *wac_i2c = dev_id;

	if (wac_i2c->query_status == false)
		return IRQ_HANDLED;

	wac_i2c->pen_pdct = gpio_get_value(wac_i2c->wac_pdata->gpio_pendct);

	printk(KERN_DEBUG "[E-PEN] pdct %d(%d)\n",
		wac_i2c->pen_pdct, wac_i2c->pen_prox);
	
	if (!wac_i2c->pen_pdct) {
		flg_epen_hovering = true;
	} else {
		flg_epen_hovering = false;
	}

	if (wac_i2c->pen_pdct == PDCT_NOSIGNAL) {
		//printk(KERN_DEBUG "[E-PEN] NOSIGNAL\n");
		/* If rdy is 1, pen is still working*/
		if (wac_i2c->pen_prox == 0){
			printk(KERN_DEBUG "[E-PEN] NOSIGNAL FORCING RELEASE\n");
			forced_release(wac_i2c);
		}
		
	} else if (wac_i2c->pen_prox == 0) {
		//printk(KERN_DEBUG "[E-PEN] NOSIGNAL2\n");
		forced_hover(wac_i2c);
	}

	return IRQ_HANDLED;
}
#endif

#ifdef WACOM_PEN_DETECT

#ifdef CONFIG_MACH_T0
bool wacom_i2c_invert_by_switch_type(void)
{
	if (system_rev < WACOM_DETECT_SWITCH_HWID)
		return true;

	return false;
}
#endif

static void pen_insert_work(struct work_struct *work)
{
	struct wacom_i2c *wac_i2c =
		container_of(work, struct wacom_i2c, pen_insert_dwork.work);

#ifdef WACOM_DEBOUNCEINT_BY_ESD
	pen_insert_state = !gpio_get_value(wac_i2c->gpio_pen_insert);

#if defined(CONFIG_MACH_T0)
	if (wac_i2c->invert_pen_insert)
		pen_insert_state = !pen_insert_state;
#endif
	if (wac_i2c->pen_insert == pen_insert_state) {
		printk(KERN_DEBUG "[E-PEN] %s INT: (%d) was skipped\n",
			__func__, wac_i2c->pen_insert);

		#ifdef BATTERY_SAVING_MODE
		if (wac_i2c->pen_insert) {
			if (wac_i2c->battery_saving_mode)
				wacom_i2c_disable(wac_i2c);
		} else {
			if (firmware_updating_state == true)
				return;
			printk(KERN_DEBUG "[E-PEN] %s call WACOM Reset\n",
				__func__);
			wac_i2c->wac_pdata->suspend_platform_hw();
			msleep(200);
			wacom_i2c_enable(wac_i2c);
		}
		#endif
		return;
	}
	wac_i2c->pen_insert = pen_insert_state;

#else
	wac_i2c->pen_insert = !gpio_get_value(wac_i2c->gpio_pen_insert);
	#if defined(CONFIG_MACH_T0)
	if (wac_i2c->invert_pen_insert)
		wac_i2c->pen_insert = !wac_i2c->pen_insert;
	#endif
#endif
	
	pen_insert_state = !gpio_get_value(wac_i2c->gpio_pen_insert);
	if (wac_i2c->invert_pen_insert)
		pen_insert_state = !pen_insert_state;
		
	//pr_info("wacom: pen_insert_state=%d wac_i2c->invert_pen_insert=%d wac_i2c->pen_insert=%d\n", pen_insert_state, wac_i2c->invert_pen_insert, wac_i2c->pen_insert);

	printk(KERN_DEBUG "[E-PEN] %s : %d\n",
		__func__, wac_i2c->pen_insert);
	//pr_info("wacom: input SW_PEN_INSERT %d\n", !wac_i2c->pen_insert);
	input_report_switch(wac_i2c->input_dev, SW_PEN_INSERT, !wac_i2c->pen_insert);
	input_sync(wac_i2c->input_dev);
	
	// L900/i605: wac_i2c->pen_insert = 0 when pen is out, 1 when pen is in.
	
	// make sure the epen activity lock is reset.
	cancel_delayed_work(&wac_i2c->activitylockout_work);
	epen_is_active = false;
	
	if (flg_epen_worryfree_mode) {
		printk(KERN_DEBUG "[E-PEN] DEBUG flg_epen_worryfree_mode was ON\n");
	} else {
		printk(KERN_DEBUG "[E-PEN] DEBUG flg_epen_worryfree_mode was OFF\n");
	}
	
	if (sttg_epen_worryfree_mode) {
		printk(KERN_DEBUG "[E-PEN] DEBUG sttg_epen_worryfree_mode was ON\n");
	} else {
		printk(KERN_DEBUG "[E-PEN] DEBUG sttg_epen_worryfree_mode was OFF\n");
	}
	
	if (wac_i2c->pen_insert) {
		printk(KERN_DEBUG "[E-PEN] sttg_epen_worryfree_mode OFF\n");
		flg_epen_worryfree_mode = false;
	} else {
		if (sttg_epen_worryfree_mode) {
			printk(KERN_DEBUG "[E-PEN] sttg_epen_worryfree_mode ON\n");
			flg_epen_worryfree_mode = true;
		}
	}

#ifdef BATTERY_SAVING_MODE
	if (wac_i2c->pen_insert) {
		if (wac_i2c->battery_saving_mode)
			wacom_i2c_disable(wac_i2c);
	} else
		wacom_i2c_enable(wac_i2c);
#endif
}
static irqreturn_t wacom_pen_detect(int irq, void *dev_id)
{
	struct wacom_i2c *wac_i2c = dev_id;

	cancel_delayed_work_sync(&wac_i2c->pen_insert_dwork);
	schedule_delayed_work(&wac_i2c->pen_insert_dwork, HZ / 20);
	return IRQ_HANDLED;
}
static int wacom_i2c_input_open(struct input_dev *dev)
{
#ifdef WACOM_PEN_DETECT
	struct wacom_i2c *wac_i2c = input_get_drvdata(dev);
	int ret = 0;
	int irq = gpio_to_irq(wac_i2c->gpio_pen_insert);

	INIT_DELAYED_WORK(&wac_i2c->pen_insert_dwork, pen_insert_work);

	ret =
		request_threaded_irq(
			irq, NULL,
			wacom_pen_detect,
			IRQF_DISABLED | IRQF_TRIGGER_RISING |
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			"pen_insert", wac_i2c);
	if (ret < 0)
		printk(KERN_ERR
			"[E-PEN]: failed to request pen insert irq\n");

	enable_irq_wake(irq);

	/* update the current status */
	schedule_delayed_work(&wac_i2c->pen_insert_dwork, HZ / 2);
#endif

	return 0;
}

static void wacom_i2c_input_close(struct input_dev *dev)
{
	/* TBD */
}

#endif

static void wacom_i2c_set_input_values(struct i2c_client *client,
				       struct wacom_i2c *wac_i2c,
				       struct input_dev *input_dev)
{
	/*Set input values before registering input device */

	input_dev->name = "sec_e-pen";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;
	input_dev->evbit[0] |= BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);

	input_dev->evbit[0] |= BIT_MASK(EV_SW);
	input_set_capability(input_dev, EV_SW, SW_PEN_INSERT);
#ifdef WACOM_PEN_DETECT
	input_dev->open = wacom_i2c_input_open;
	input_dev->close = wacom_i2c_input_close;
#endif

	__set_bit(ABS_X, input_dev->absbit);
	__set_bit(ABS_Y, input_dev->absbit);
	__set_bit(ABS_PRESSURE, input_dev->absbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(BTN_TOOL_PEN, input_dev->keybit);
	__set_bit(BTN_TOOL_RUBBER, input_dev->keybit);
	__set_bit(BTN_STYLUS, input_dev->keybit);
	__set_bit(KEY_UNKNOWN, input_dev->keybit);
	__set_bit(KEY_PEN_PDCT, input_dev->keybit);

	/*  __set_bit(BTN_STYLUS2, input_dev->keybit); */
	/*  __set_bit(ABS_MISC, input_dev->absbit); */
}

static int wacom_check_emr_prox(struct wacom_g5_callbacks *cb)
{
	struct wacom_i2c *wac = container_of(cb, struct wacom_i2c, callbacks);
	printk(KERN_DEBUG "[E-PEN] %s:\n", __func__);

	return wac->pen_prox;
}

static int wacom_i2c_remove(struct i2c_client *client)
{
	struct wacom_i2c *wac_i2c = i2c_get_clientdata(client);
	free_irq(client->irq, wac_i2c);
	input_unregister_device(wac_i2c->input_dev);
	kfree(wac_i2c);

	return 0;
}

static void wacom_i2c_early_suspend(struct early_suspend *h)
{
	struct wacom_i2c *wac_i2c =
	    container_of(h, struct wacom_i2c, early_suspend);
	printk(KERN_DEBUG "[E-PEN] %s.\n", __func__);
#ifdef WACOM_STATE_CHECK
	cancel_delayed_work_sync(&wac_i2c->wac_statecheck_work);
#endif
	wacom_i2c_disable(wac_i2c);
}

static void wacom_i2c_resume_work(struct work_struct *work)
{
	struct wacom_i2c *wac_i2c =
	    container_of(work, struct wacom_i2c, resume_work.work);

#if defined(WACOM_PDCT_WORK_AROUND)
	irq_set_irq_type(wac_i2c->irq_pdct, IRQ_TYPE_EDGE_BOTH);
#endif

#if defined(CONFIG_MACH_P4NOTE)
	irq_set_irq_type(wac_i2c->client->irq, IRQ_TYPE_EDGE_RISING);
#endif

	wac_i2c->power_enable = true;
	wacom_i2c_enable_irq(wac_i2c, true);
#ifdef WACOM_STATE_CHECK
	schedule_delayed_work(&wac_i2c->wac_statecheck_work, HZ * 30);
#endif
	printk(KERN_DEBUG "[E-PEN] %s\n", __func__);
}


#ifdef WACOM_STATE_CHECK
static void wac_statecheck_work(struct work_struct *work)
{
	int ret, i;
	char buf, test[10];
	struct wacom_i2c *wac_i2c =
	    container_of(work, struct wacom_i2c, wac_statecheck_work.work);
	buf = COM_QUERY;
	printk(KERN_DEBUG "[E-PEN] %s\n", __func__);

	if (firmware_updating_state == true)
		return;

#ifdef BATTERY_SAVING_MODE
	if (wac_i2c->battery_saving_mode
		&& wac_i2c->pen_insert) {
		printk(KERN_DEBUG "[E-PEN] escaped from wacom check mode\n");
		printk(KERN_DEBUG "        becase pen has inserted at lpm\n");
		return;
	}
#endif

#if defined(CONFIG_TARGET_LOCALE_KOR)
#if defined(CONFIG_MACH_T0) && defined(CONFIG_TDMB_ANT_DET)
	ret = gpio_get_value(wac_i2c->wac_pdata->gpio_esd_check);
	if (ret == 0) {
		printk(KERN_DEBUG "[E-PEN] skip wacom state checking\n");
		printk(KERN_DEBUG "        becase ANT has closed\n");
		schedule_delayed_work(&wac_i2c->wac_statecheck_work, HZ * 30);
		return;
	}
#endif
#endif

	ret = wacom_i2c_send(wac_i2c, &buf, sizeof(buf), false);
	if (ret > 0)
		printk(KERN_INFO "[E-PEN] buf:%d, sent:%d\n", buf, ret);
	else {
		printk(KERN_ERR "[E-PEN] Digitizer is not active\n");
		wac_i2c->wac_pdata->suspend_platform_hw();
		msleep(200);
		wacom_i2c_enable(wac_i2c);
		printk(KERN_ERR "[E-PEN] wacom reset done\n");
	}

	schedule_delayed_work(&wac_i2c->wac_statecheck_work, HZ * 30);
}
#endif


static void wacom_i2c_late_resume(struct early_suspend *h)
{
	struct wacom_i2c *wac_i2c =
	    container_of(h, struct wacom_i2c, early_suspend);

	printk(KERN_DEBUG "[E-PEN] %s.\n", __func__);
	wacom_i2c_enable(wac_i2c);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
#define wacom_i2c_suspend	NULL
#define wacom_i2c_resume	NULL

#else
static int wacom_i2c_suspend(struct i2c_client *client, pm_message_t mesg)
{
	wacom_i2c_early_suspend();
	printk(KERN_DEBUG "[E-PEN] %s.\n", __func__);
	return 0;
}

static int wacom_i2c_resume(struct i2c_client *client)
{
	wacom_i2c_late_resume();
	printk(KERN_DEBUG "[E-PEN] %s.\n", __func__);
	return 0;
}
#endif

static ssize_t epen_firm_update_status_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct wacom_i2c *wac_i2c = dev_get_drvdata(dev);

	printk(KERN_DEBUG "[E-PEN] %s:(%d)\n", __func__,
	       wac_i2c->wac_feature->firm_update_status);

	if (wac_i2c->wac_feature->firm_update_status == 2)
		return sprintf(buf, "PASS\n");
	else if (wac_i2c->wac_feature->firm_update_status == 1)
		return sprintf(buf, "DOWNLOADING\n");
	else if (wac_i2c->wac_feature->firm_update_status == -1)
		return sprintf(buf, "FAIL\n");
	else
		return 0;
}

static ssize_t epen_firm_version_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct wacom_i2c *wac_i2c = dev_get_drvdata(dev);
	printk(KERN_DEBUG "[E-PEN] %s: 0x%x|0x%X\n", __func__,
	       wac_i2c->wac_feature->fw_version, Firmware_version_of_file);

	return sprintf(buf, "%04X\t%04X\n",
			wac_i2c->wac_feature->fw_version,
			Firmware_version_of_file);
}

#if defined(WACOM_IMPORT_FW_ALGO)
static ssize_t epen_tuning_version_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	printk(KERN_DEBUG "[E-PEN] %s: %s\n", __func__,
			tuning_version);

	return sprintf(buf, "%s_%s\n",
			tuning_model,
			tuning_version);
}
#endif

static bool check_update_condition(struct wacom_i2c *wac_i2c, const char buf)
{
	u32 fw_ic_ver = wac_i2c->wac_feature->fw_version;
	bool bUpdate = false;

	switch (buf) {
	case 'I':
	case 'K':
		bUpdate = true;
		break;
	case 'R':
	case 'W':
		if (fw_ic_ver <
			Firmware_version_of_file)
			bUpdate = true;
#ifdef CONFIG_MACH_Q1_BD
		/* Q1 board rev 0.3 */
		if (system_rev < 6)
			if (fw_ic_ver !=
				Firmware_version_of_file)
				bUpdate = true;
#endif
		break;
	default:
		printk(KERN_ERR "[E-PEN] wrong parameter\n");
		bUpdate = false;
		break;
	}

	return bUpdate;
}

static ssize_t epen_firmware_update_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct wacom_i2c *wac_i2c = dev_get_drvdata(dev);
	int ret = 1;
	u32 fw_ic_ver = wac_i2c->wac_feature->fw_version;
	bool need_update = false;

	printk(KERN_DEBUG "[E-PEN] %s\n", __func__);

	need_update = check_update_condition(wac_i2c, *buf);
	if (need_update == false) {
		printk(KERN_DEBUG"[E-PEN] Pass Update."
			"Cmd %c, IC ver %04x, Ker ver %04x\n",
			*buf, fw_ic_ver, Firmware_version_of_file);
		return count;
	}

	/*start firm update*/
	mutex_lock(&wac_i2c->lock);
	wacom_i2c_enable_irq(wac_i2c, false);
	wac_i2c->wac_feature->firm_update_status = 1;

	switch (*buf) {
	/*ums*/
	case 'I':
		ret = wacom_i2c_fw_update_UMS(wac_i2c);
		break;
	/*kernel*/
	case 'K':
		printk(KERN_ERR
			"[E-PEN] Start firmware flashing (kernel image).\n");
#if defined(CONFIG_MACH_Q1_BD) || defined(CONFIG_MACH_T0)
		ret = wacom_i2c_firm_update(wac_i2c);
#else
		ret = wacom_i2c_flash(wac_i2c);
#endif
		break;

	/*booting*/
#if defined(CONFIG_MACH_Q1_BD) \
	|| defined(CONFIG_MACH_T0)
	case 'R':
		ret = wacom_i2c_firm_update(wac_i2c);
		break;
#elif defined(CONFIG_MACH_P4NOTE)
	case 'R':
		update_fw_p4(wac_i2c);
		break;
#endif
	default:
		/*There's no default case*/
		break;
	}

	if (ret < 0) {
		printk(KERN_ERR "[E-PEN] failed to flash firmware.\n");
		goto failure;
	}

	printk(KERN_ERR "[E-PEN] Finish firmware flashing.\n");

	wacom_i2c_query(wac_i2c);

	wac_i2c->wac_feature->firm_update_status = 2;
	wacom_i2c_enable_irq(wac_i2c, true);
	mutex_unlock(&wac_i2c->lock);

	return count;

 param_err:

 failure:
	wac_i2c->wac_feature->firm_update_status = -1;
	wacom_i2c_enable_irq(wac_i2c, true);
	mutex_unlock(&wac_i2c->lock);
	return -1;
}

#if defined(CONFIG_MACH_P4NOTE)
static ssize_t epen_sampling_rate_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct wacom_i2c *wac_i2c = dev_get_drvdata(dev);
	int value;
	char mode;

	if (sscanf(buf, "%d", &value) == 1) {
		switch (value) {
		case 0:
			mode = COM_SAMPLERATE_STOP;
			break;
		case 40:
			mode = COM_SAMPLERATE_40;
			break;
		case 80:
			mode = COM_SAMPLERATE_80;
			break;
		case 133:
			mode = COM_SAMPLERATE_133;
			break;
		default:
			pr_err("[E-PEN] Invalid sampling rate value\n");
			count = -1;
			goto fail;
		}

		wacom_i2c_enable_irq(wac_i2c, false);
		if (1 == wacom_i2c_send(wac_i2c, &mode, 1, false)) {
			printk(KERN_DEBUG "[E-PEN] sampling rate %d\n", value);
			msleep(100);
		} else {
			pr_err("[E-PEN] I2C write error\n");
			wacom_i2c_enable_irq(wac_i2c, true);
			count = -1;
		}
		wacom_i2c_enable_irq(wac_i2c, true);

	} else {
		pr_err("[E-PEN] can't get sampling rate data\n");
		count = -1;
	}
 fail:
	return count;
}
#else
static ssize_t epen_rotation_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	static bool factory_test;
	static unsigned char last_rotation;
	unsigned int val;

	sscanf(buf, "%u", &val);

	/* Fix the rotation value to 0(Portrait) when factory test(15 mode) */
	if (val == 100 && !factory_test) {
		factory_test = true;
		screen_rotate = 0;
		printk(KERN_DEBUG "[E-PEN] %s, enter factory test mode\n",
		       __func__);
	} else if (val == 200 && factory_test) {
		factory_test = false;
		screen_rotate = last_rotation;
		printk(KERN_DEBUG "[E-PEN] %s, exit factory test mode\n",
		       __func__);
	}

	/* Framework use index 0, 1, 2, 3 for rotation 0, 90, 180, 270 */
	/* Driver use same rotation index */
	if (val >= 0 && val <= 3) {
		if (factory_test)
			last_rotation = val;
		else
			screen_rotate = val;
	}

	/* 0: Portrait 0, 1: Landscape 90, 2: Portrait 180 3: Landscape 270 */
	printk(KERN_DEBUG "[E-PEN] %s: rotate=%d\n", __func__, screen_rotate);

	return count;
}

static ssize_t epen_hand_show(struct device *dev,
							  struct device_attribute *attr,
							  char *buf)
{	
	return sprintf(buf, "%d\n", user_hand);
}

static ssize_t epen_hand_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	unsigned int val;

	sscanf(buf, "%u", &val);

	if (val == 0 || val == 1)
		user_hand = (unsigned char)val;

	/* 0:Left hand, 1:Right Hand */
	printk(KERN_DEBUG "[E-PEN] %s: hand=%u\n", __func__, user_hand);

	return count;
}
#endif

static ssize_t epen_boxfilter_threshold_show(struct device *dev,
							  struct device_attribute *attr,
							  char *buf)
{	
	return sprintf(buf, "%d\n", sttg_boxfilter_threshold);
}

static ssize_t epen_boxfilter_threshold_store(struct device *dev,
							   struct device_attribute *attr, const char *buf,
							   size_t count)
{
	unsigned int val;
	
	sscanf(buf, "%u", &val);
	
	if (val >= 0 && val <= 1000)
		sttg_boxfilter_threshold = val;
	
	printk("[E-PEN] box threshold set to %d\n", val);
	
	return count;
}

static ssize_t epen_boxfilter_threshold_hover_show(struct device *dev,
									   struct device_attribute *attr,
									   char *buf)
{	
	return sprintf(buf, "%d\n", sttg_boxfilter_threshold_hover);
}

static ssize_t epen_boxfilter_threshold_hover_store(struct device *dev,
										struct device_attribute *attr, const char *buf,
										size_t count)
{
	unsigned int val;
	
	sscanf(buf, "%u", &val);
	
	if (val >= 0 && val <= 1000)
		sttg_boxfilter_threshold_hover = val;
	
	printk("[E-PEN] boxfilter threshold set to %d\n", val);
	
	return count;
}

static ssize_t epen_boost_freq_show(struct device *dev,
									   struct device_attribute *attr,
									   char *buf)
{	
	return sprintf(buf, "%d\n", sttg_boost_freq);
}

static ssize_t epen_boost_freq_store(struct device *dev,
										struct device_attribute *attr, const char *buf,
										size_t count)
{
	unsigned int val;
	
	sscanf(buf, "%u", &val);
	
	if (val > 0) {
		if (val < 200000)
			sttg_boost_freq = 200000;
		
		if (val > 1600000)
			sttg_boost_freq = 1600000;
		
		sttg_boost_freq = val;
		
		// reinitialize epen_cpufreq_level so it will get the new freq.
		epen_cpufreq_level = -1;
		
		printk("[E-PEN] boost frequency set to %d\n", val);
	} else {
		printk("[E-PEN] boost disabled\n");
	}
	
	return count;
}

static ssize_t epen_min_pressure_show(struct device *dev,
									struct device_attribute *attr,
									char *buf)
{	
	return sprintf(buf, "%d\n", sttg_min_pressure);
}

static ssize_t epen_min_pressure_store(struct device *dev,
									 struct device_attribute *attr, const char *buf,
									 size_t count)
{
	unsigned int val;
	
	sscanf(buf, "%u", &val);
	
	if (val >= 0 && val < 1024) {
		if (val == 1024)
			val = 1023;
		sttg_min_pressure = val;
	}
	
	return count;
}

static ssize_t epen_side_button_mode_show(struct device *dev,
									  struct device_attribute *attr,
									  char *buf)
{	
	return sprintf(buf, "%d\n", sttg_side_button_mode);
}

static ssize_t epen_side_button_mode_store(struct device *dev,
									   struct device_attribute *attr, const char *buf,
									   size_t count)
{
	unsigned int val;
	
	sscanf(buf, "%u", &val);
	
	if (val == 0 || val == 1 || val == 2)
		sttg_side_button_mode = val;
	
	return count;
}

static ssize_t epen_fixed_pressure_show(struct device *dev,
											 struct device_attribute *attr,
											 char *buf)
{	
	return sprintf(buf, "%d\n", sttg_fixed_pressure);
}

static ssize_t epen_fixed_pressure_store(struct device *dev,
											  struct device_attribute *attr, const char *buf,
											  size_t count)
{
	unsigned int val;
	
	sscanf(buf, "%u", &val);
	
	if (val >=0 && val <= 1024) {
		if (val == 1024)
			val = 1023;
		sttg_fixed_pressure = val;
	}
	
	return count;
}

static ssize_t epen_avg_strength_show(struct device *dev,
										struct device_attribute *attr,
										char *buf)
{	
	return sprintf(buf, "%d\n", sttg_avg_strength);
}

static ssize_t epen_avg_strength_store(struct device *dev,
										 struct device_attribute *attr, const char *buf,
										 size_t count)
{
	unsigned int val;
	
	sscanf(buf, "%u", &val);
	
	if (val >= 0 && val < 5) {
		sttg_avg_strength = val;
	} else {
		sttg_avg_strength = 2;
	}

	
	return count;
}

static ssize_t epen_offset_override_x_show(struct device *dev,
								   struct device_attribute *attr,
								   char *buf)
{	
	return sprintf(buf, "%d\n", sttg_offset_override_x);
}

static ssize_t epen_offset_override_x_store(struct device *dev,
									struct device_attribute *attr, const char *buf,
									size_t count)
{
	int val;
	
	sscanf(buf, "%d", &val);
	
	if (val >= -1000 && val <= 1000)
		sttg_offset_override_x = val;
	
	return count;
}

static ssize_t epen_offset_override_y_show(struct device *dev,
										   struct device_attribute *attr,
										   char *buf)
{	
	return sprintf(buf, "%d\n", sttg_offset_override_y);
}

static ssize_t epen_offset_override_y_store(struct device *dev,
											struct device_attribute *attr, const char *buf,
											size_t count)
{
	int val;
	
	sscanf(buf, "%d", &val);
	
	if (val >= -1000 && val <= 1000)
		sttg_offset_override_y = val;
	
	return count;
}

static ssize_t epen_offset_override_enable_show(struct device *dev,
										   struct device_attribute *attr,
										   char *buf)
{	
	return sprintf(buf, "%u\n", sttg_offset_override_enabled);
}

static ssize_t epen_offset_override_enable_store(struct device *dev,
											struct device_attribute *attr, const char *buf,
											size_t count)
{
	int val;
	
	sscanf(buf, "%u", &val);
	
	if (val == 0 || val == 1)
		sttg_offset_override_enabled = val;
	
	return count;
}

static ssize_t epen_touchkey_block_length_show(struct device *dev,
										   struct device_attribute *attr,
										   char *buf)
{	
	return sprintf(buf, "%d\n", sttg_touchkey_block_length);
}

static ssize_t epen_touchkey_block_length_store(struct device *dev,
											struct device_attribute *attr, const char *buf,
											size_t count)
{
	int val;
	
	sscanf(buf, "%d", &val);
	
	if (val >= 0 && val <= 60000)
		sttg_touchkey_block_length = val;
	
	return count;
}

static ssize_t epen_worryfree_mode_show(struct device *dev,
										struct device_attribute *attr,
										char *buf)
{
	return sprintf(buf, "%u\n", sttg_epen_worryfree_mode);
}

static ssize_t epen_worryfree_mode_store(struct device *dev,
										 struct device_attribute *attr, const char *buf,
										 size_t count)
{
	int val;
	
	sscanf(buf, "%u", &val);
	
	if (val == 0) {
		// off.
		
		sttg_epen_worryfree_mode = val;
		flg_epen_worryfree_mode = val;
		
	} else if (val == 1) {
		// on.
		
		sttg_epen_worryfree_mode = val;
		
		if (!pen_insert_state) {
			// pen NOT inserted.
			flg_epen_worryfree_mode = val;
		}
	}
	
	return count;
}

static ssize_t epen_hover_side_hotspot_height_show(struct device *dev,
												   struct device_attribute *attr,
												   char *buf)
{
	return sprintf(buf, "%d\n", (WACOM_MAX_COORD_X - sttg_epen_hover_side_hotspot_height));
}

static ssize_t epen_hover_side_hotspot_height_store(struct device *dev,
													struct device_attribute *attr, const char *buf,
													size_t count)
{
	int val;
	
	sscanf(buf, "%d", &val);
	
	sttg_epen_hover_side_hotspot_height = (WACOM_MAX_COORD_X - val);
	
	return count;
}

static ssize_t epen_hover_side_left_keycode_show(struct device *dev,
												 struct device_attribute *attr,
												 char *buf)
{
	return sprintf(buf, "%d\n", sttg_epen_hover_side_left_keycode);
}

static ssize_t epen_hover_side_left_keycode_store(struct device *dev,
												  struct device_attribute *attr, const char *buf,
												  size_t count)
{
	int val;
	
	sscanf(buf, "%d", &val);
	
	sttg_epen_hover_side_left_keycode = val;
	
	return count;
}

static ssize_t epen_hover_side_left_keydelay_show(struct device *dev,
												  struct device_attribute *attr,
												  char *buf)
{
	return sprintf(buf, "%d\n", sttg_epen_hover_side_left_keydelay);
}

static ssize_t epen_hover_side_left_keydelay_store(struct device *dev,
												   struct device_attribute *attr, const char *buf,
												   size_t count)
{
	int val;
	
	sscanf(buf, "%d", &val);
	
	sttg_epen_hover_side_left_keydelay = val;
	
	return count;
}

static ssize_t epen_hover_side_center_keycode_show(struct device *dev,
												   struct device_attribute *attr,
												   char *buf)
{
	return sprintf(buf, "%d\n", sttg_epen_hover_side_center_keycode);
}

static ssize_t epen_hover_side_center_keycode_store(struct device *dev,
													struct device_attribute *attr, const char *buf,
													size_t count)
{
	int val;
	
	sscanf(buf, "%d", &val);
	
	sttg_epen_hover_side_center_keycode = val;
	
	return count;
}

static ssize_t epen_hover_side_center_keydelay_show(struct device *dev,
													struct device_attribute *attr,
													char *buf)
{
	return sprintf(buf, "%d\n", sttg_epen_hover_side_center_keydelay);
}

static ssize_t epen_hover_side_center_keydelay_store(struct device *dev,
													 struct device_attribute *attr, const char *buf,
													 size_t count)
{
	int val;
	
	sscanf(buf, "%d", &val);
	
	sttg_epen_hover_side_center_keydelay = val;
	
	return count;
}

static ssize_t epen_hover_side_right_keycode_show(struct device *dev,
												  struct device_attribute *attr,
												  char *buf)
{
	return sprintf(buf, "%d\n", sttg_epen_hover_side_right_keycode);
}

static ssize_t epen_hover_side_right_keycode_store(struct device *dev,
												   struct device_attribute *attr, const char *buf,
												   size_t count)
{
	int val;
	
	sscanf(buf, "%d", &val);
	
	sttg_epen_hover_side_right_keycode = val;
	
	return count;
}

static ssize_t epen_hover_side_right_keydelay_show(struct device *dev,
												   struct device_attribute *attr,
												   char *buf)
{
	return sprintf(buf, "%d\n", sttg_epen_hover_side_right_keydelay);
}

static ssize_t epen_hover_side_right_keydelay_store(struct device *dev,
													struct device_attribute *attr, const char *buf,
													size_t count)
{
	int val;
	
	sscanf(buf, "%d", &val);
	
	sttg_epen_hover_side_right_keydelay = val;
	
	return count;
}

// padding
//////////////////////

static ssize_t epen_padding_top_show(struct device *dev,
									 struct device_attribute *attr,
									 char *buf)
{
	return sprintf(buf, "%d\n", sttg_epen_padding_top);
}

static ssize_t epen_padding_top_store(struct device *dev,
									  struct device_attribute *attr, const char *buf,
									  size_t count)
{
	int val;
	
	sscanf(buf, "%d", &val);
	
	sttg_epen_padding_top = val;
	
	return count;
}

static ssize_t epen_padding_right_show(struct device *dev,
									   struct device_attribute *attr,
									   char *buf)
{
	return sprintf(buf, "%d\n", (WACOM_MAX_COORD_Y - sttg_epen_padding_right));
}

static ssize_t epen_padding_right_store(struct device *dev,
										struct device_attribute *attr, const char *buf,
										size_t count)
{
	int val;
	
	sscanf(buf, "%d", &val);
	
	sttg_epen_padding_right = (WACOM_MAX_COORD_Y - val);
	
	return count;
}

static ssize_t epen_padding_bottom_show(struct device *dev,
										struct device_attribute *attr,
										char *buf)
{
	return sprintf(buf, "%d\n", (WACOM_MAX_COORD_X - sttg_epen_padding_bottom));
}

static ssize_t epen_padding_bottom_store(struct device *dev,
										 struct device_attribute *attr, const char *buf,
										 size_t count)
{
	int val;
	
	sscanf(buf, "%d", &val);
	
	sttg_epen_padding_bottom = (WACOM_MAX_COORD_X - val);
	
	return count;
}

static ssize_t epen_padding_left_show(struct device *dev,
									  struct device_attribute *attr,
									  char *buf)
{
	return sprintf(buf, "%d\n", sttg_epen_padding_left);
}

static ssize_t epen_padding_left_store(struct device *dev,
									   struct device_attribute *attr, const char *buf,
									   size_t count)
{
	int val;
	
	sscanf(buf, "%d", &val);
	
	sttg_epen_padding_left = val;
	
	return count;
}

static ssize_t epen_reset_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct wacom_i2c *wac_i2c = dev_get_drvdata(dev);
	int ret;
	int val;

	sscanf(buf, "%d", &val);

	if (val == 1) {
		wacom_i2c_enable_irq(wac_i2c, false);

		/* Reset IC */
		wacom_i2c_reset_hw(wac_i2c->wac_pdata);

		/* I2C Test */
		ret = wacom_i2c_query(wac_i2c);

		wacom_i2c_enable_irq(wac_i2c, true);

		if (ret < 0)
			epen_reset_result = false;
		else
			epen_reset_result = true;

		printk(KERN_DEBUG "[E-PEN] %s, result %d\n", __func__,
		       epen_reset_result);
	}

	return count;
}

static ssize_t epen_reset_result_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	if (epen_reset_result) {
		printk(KERN_DEBUG "[E-PEN] %s, PASS\n", __func__);
		return sprintf(buf, "PASS\n");
	} else {
		printk(KERN_DEBUG "[E-PEN] %s, FAIL\n", __func__);
		return sprintf(buf, "FAIL\n");
	}
}

static ssize_t epen_pen_inserted_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	if (pen_insert_state) {
		// pen inserted.
		printk(KERN_DEBUG "[E-PEN] %s, pen is inserted.\n", __func__);
		return sprintf(buf, "1");
	} else {
		printk(KERN_DEBUG "[E-PEN] %s, pen is not inserted.\n", __func__);
		return sprintf(buf, "0");
	}
		
}

#ifdef WACOM_USE_AVE_TRANSITION
static ssize_t epen_ave_store(struct device *dev,
struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct wacom_i2c *wac_i2c = dev_get_drvdata(dev);
	bool check_version = false;
	int v1, v2, v3, v4, v5;
	int height;

	sscanf(buf, "%d%d%d%d%d%d", &height, &v1, &v2, &v3, &v4, &v5);

	if (height < 0 || height > 2) {
		printk(KERN_DEBUG"[E-PEN] Height err %d\n", height);
		return count;
	}

	g_aveLevel_C[height] = v1;
	g_aveLevel_X[height] = v2;
	g_aveLevel_Y[height] = v3;
	g_aveLevel_Trs[height] = v4;
	g_aveLevel_Cor[height] = v5;
	g_aveShift;

	printk(KERN_DEBUG "[E-PEN] %s, v1 %d v2 %d v3 %d v4 %d\n", __func__,
		v1, v2, v3, v4);

	return count;
}

static ssize_t epen_ave_result_show(struct device *dev,
struct device_attribute *attr,
	char *buf)
{
	struct wacom_i2c *wac_i2c = dev_get_drvdata(dev);

	printk(KERN_DEBUG "[E-PEN] %s\n%d %d %d %d\n"
		"%d %d %d %d\n%d %d %d %d\n",
		__func__,
		g_aveLevel_C[0], g_aveLevel_X[0],
		g_aveLevel_Y[0], g_aveLevel_Trs[0],
		g_aveLevel_C[1], g_aveLevel_X[1],
		g_aveLevel_Y[1], g_aveLevel_Trs[1],
		g_aveLevel_C[2], g_aveLevel_X[2],
		g_aveLevel_Y[2], g_aveLevel_Trs[2]);
	return sprintf(buf, "%d %d %d %d\n%d %d %d %d\n%d %d %d %d\n",
		g_aveLevel_C[0], g_aveLevel_X[0],
		g_aveLevel_Y[0], g_aveLevel_Trs[0],
		g_aveLevel_C[1], g_aveLevel_X[1],
		g_aveLevel_Y[1], g_aveLevel_Trs[1],
		g_aveLevel_C[2], g_aveLevel_X[2],
		g_aveLevel_Y[2], g_aveLevel_Trs[2]);
}
#endif

static ssize_t epen_checksum_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct wacom_i2c *wac_i2c = dev_get_drvdata(dev);
	bool check_version = false;
	int val;

	sscanf(buf, "%d", &val);

#if defined(CONFIG_MACH_P4NOTE)
	if (wac_i2c->checksum_result)
		return count;
	else
		check_version = true;
#elif defined(CONFIG_MACH_Q1_BD)
	if (wac_i2c->wac_feature->fw_version >= 0x31E)
		check_version = true;
#elif defined(CONFIG_MACH_T0)
	if (wac_i2c->wac_feature->fw_version == 0x179) {
		wac_i2c->checksum_result = 1;
		check_version = false;
	} else
		check_version = true;
#endif

	if (val == 1 && check_version) {
		wacom_i2c_enable_irq(wac_i2c, false);

		wacom_checksum(wac_i2c);

		wacom_i2c_enable_irq(wac_i2c, true);
	}
	printk(KERN_DEBUG "[E-PEN] %s, check %d, result %d\n",
		__func__, check_version, wac_i2c->checksum_result);

	return count;
}

static ssize_t epen_checksum_result_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct wacom_i2c *wac_i2c = dev_get_drvdata(dev);

	if (wac_i2c->checksum_result) {
		printk(KERN_DEBUG "[E-PEN] checksum, PASS\n");
		return sprintf(buf, "PASS\n");
	} else {
		printk(KERN_DEBUG "[E-PEN] checksum, FAIL\n");
		return sprintf(buf, "FAIL\n");
	}
}

#ifdef WACOM_CONNECTION_CHECK
static ssize_t epen_connection_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct wacom_i2c *wac_i2c = dev_get_drvdata(dev);

	printk(KERN_DEBUG
		"[E-PEN] connection_check : %d\n",
		wac_i2c->connection_check);
	return sprintf(buf, "%s\n",
		wac_i2c->connection_check ?
		"OK" : "NG");
}
#endif

#if defined(CONFIG_MACH_P4NOTE)
static ssize_t epen_type_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct wacom_i2c *wac_i2c = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", wac_i2c->pen_type);
}

static ssize_t epen_type_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct wacom_i2c *wac_i2c = dev_get_drvdata(dev);
	int val;

	sscanf(buf, "%d", &val);

	wac_i2c->pen_type = !!val;

	return count;
}
#endif

#ifdef BATTERY_SAVING_MODE
static ssize_t epen_saving_mode_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct wacom_i2c *wac_i2c = dev_get_drvdata(dev);
	int val;

	if (sscanf(buf, "%u", &val) == 1)
		wac_i2c->battery_saving_mode = !!val;

	if (wac_i2c->battery_saving_mode) {
		if (wac_i2c->pen_insert)
			wacom_i2c_disable(wac_i2c);
	} else
		wacom_i2c_enable(wac_i2c);
	return count;
}
#endif

/* firmware update */
static DEVICE_ATTR(epen_firm_update,
		   S_IWUSR | S_IWGRP, NULL, epen_firmware_update_store);
/* return firmware update status */
static DEVICE_ATTR(epen_firm_update_status,
		   S_IRUGO, epen_firm_update_status_show, NULL);
/* return firmware version */
static DEVICE_ATTR(epen_firm_version, S_IRUGO, epen_firm_version_show, NULL);
#if defined(WACOM_IMPORT_FW_ALGO)
/* return tuning data version */
static DEVICE_ATTR(epen_tuning_version, S_IRUGO,
			epen_tuning_version_show, NULL);
#endif

#if defined(CONFIG_MACH_P4NOTE)
static DEVICE_ATTR(epen_sampling_rate,
		   S_IWUSR | S_IWGRP, NULL, epen_sampling_rate_store);
static DEVICE_ATTR(epen_type,
		S_IRUGO | S_IWUSR | S_IWGRP, epen_type_show, epen_type_store);
#else
/* screen rotation */
static DEVICE_ATTR(epen_rotation, S_IWUSR | S_IWGRP, NULL, epen_rotation_store);
/* hand type */
static DEVICE_ATTR(epen_hand, S_IRUGO | S_IWUSR, epen_hand_show, epen_hand_store);
#endif
static DEVICE_ATTR(epen_boxfilter_threshold, S_IRUGO | S_IWUSR, epen_boxfilter_threshold_show, epen_boxfilter_threshold_store);
static DEVICE_ATTR(epen_boxfilter_threshold_hover, S_IRUGO | S_IWUSR, epen_boxfilter_threshold_hover_show, epen_boxfilter_threshold_hover_store);
static DEVICE_ATTR(epen_boost_freq, S_IRUGO | S_IWUSR, epen_boost_freq_show, epen_boost_freq_store);
static DEVICE_ATTR(epen_min_pressure, S_IRUGO | S_IWUSR, epen_min_pressure_show, epen_min_pressure_store);
static DEVICE_ATTR(epen_side_button_mode, S_IRUGO | S_IWUSR, epen_side_button_mode_show, epen_side_button_mode_store);
static DEVICE_ATTR(epen_fixed_pressure, S_IRUGO | S_IWUSR, epen_fixed_pressure_show, epen_fixed_pressure_store);
static DEVICE_ATTR(epen_avg_strength, S_IRUGO | S_IWUSR, epen_avg_strength_show, epen_avg_strength_store);
static DEVICE_ATTR(epen_offset_override_x, S_IRUGO | S_IWUSR, epen_offset_override_x_show, epen_offset_override_x_store);
static DEVICE_ATTR(epen_offset_override_y, S_IRUGO | S_IWUSR, epen_offset_override_y_show, epen_offset_override_y_store);
static DEVICE_ATTR(epen_offset_override_enable, S_IRUGO | S_IWUSR, epen_offset_override_enable_show, epen_offset_override_enable_store);
static DEVICE_ATTR(epen_touchkey_block_length, S_IRUGO | S_IWUSR, epen_touchkey_block_length_show, epen_touchkey_block_length_store);
static DEVICE_ATTR(epen_hover_side_hotspot_height, S_IRUGO | S_IWUSR, epen_hover_side_hotspot_height_show, epen_hover_side_hotspot_height_store);
static DEVICE_ATTR(epen_hover_side_left_keycode, S_IRUGO | S_IWUSR, epen_hover_side_left_keycode_show, epen_hover_side_left_keycode_store);
static DEVICE_ATTR(epen_hover_side_left_keydelay, S_IRUGO | S_IWUSR, epen_hover_side_left_keydelay_show, epen_hover_side_left_keydelay_store);
static DEVICE_ATTR(epen_hover_side_center_keycode, S_IRUGO | S_IWUSR, epen_hover_side_center_keycode_show, epen_hover_side_center_keycode_store);
static DEVICE_ATTR(epen_hover_side_center_keydelay, S_IRUGO | S_IWUSR, epen_hover_side_center_keydelay_show, epen_hover_side_center_keydelay_store);
static DEVICE_ATTR(epen_hover_side_right_keycode, S_IRUGO | S_IWUSR, epen_hover_side_right_keycode_show, epen_hover_side_right_keycode_store);
static DEVICE_ATTR(epen_hover_side_right_keydelay, S_IRUGO | S_IWUSR, epen_hover_side_right_keydelay_show, epen_hover_side_right_keydelay_store);
static DEVICE_ATTR(epen_padding_top, S_IRUGO | S_IWUSR, epen_padding_top_show, epen_padding_top_store);
static DEVICE_ATTR(epen_padding_right, S_IRUGO | S_IWUSR, epen_padding_right_show, epen_padding_right_store);
static DEVICE_ATTR(epen_padding_bottom, S_IRUGO | S_IWUSR, epen_padding_bottom_show, epen_padding_bottom_store);
static DEVICE_ATTR(epen_padding_left, S_IRUGO | S_IWUSR, epen_padding_left_show, epen_padding_left_store);
static DEVICE_ATTR(epen_worryfree_mode, S_IRUGO | S_IWUSR, epen_worryfree_mode_show, epen_worryfree_mode_store);
static DEVICE_ATTR(epen_pen_inserted,
				   S_IRUSR | S_IRGRP, epen_pen_inserted_show, NULL);
/* For SMD Test */
static DEVICE_ATTR(epen_reset, S_IWUSR | S_IWGRP, NULL, epen_reset_store);
static DEVICE_ATTR(epen_reset_result,
		   S_IRUSR | S_IRGRP, epen_reset_result_show, NULL);

/* For SMD Test. Check checksum */
static DEVICE_ATTR(epen_checksum, S_IWUSR | S_IWGRP, NULL, epen_checksum_store);
static DEVICE_ATTR(epen_checksum_result, S_IRUSR | S_IRGRP,
		   epen_checksum_result_show, NULL);

#ifdef WACOM_USE_AVE_TRANSITION
static DEVICE_ATTR(epen_ave, S_IWUSR | S_IWGRP, NULL, epen_ave_store);
static DEVICE_ATTR(epen_ave_result, S_IRUSR | S_IRGRP,
	epen_ave_result_show, NULL);
#endif

#ifdef WACOM_CONNECTION_CHECK
static DEVICE_ATTR(epen_connection,
		   S_IRUGO, epen_connection_show, NULL);
#endif

#ifdef BATTERY_SAVING_MODE
static DEVICE_ATTR(epen_saving_mode,
		   S_IWUSR | S_IWGRP, NULL, epen_saving_mode_store);
#endif

static struct attribute *epen_attributes[] = {
	&dev_attr_epen_firm_update.attr,
	&dev_attr_epen_firm_update_status.attr,
	&dev_attr_epen_pen_inserted.attr,
	&dev_attr_epen_firm_version.attr,
#if defined(WACOM_IMPORT_FW_ALGO)
	&dev_attr_epen_tuning_version.attr,
#endif
#if defined(CONFIG_MACH_P4NOTE)
	&dev_attr_epen_sampling_rate.attr,
	&dev_attr_epen_type.attr,
#else
	&dev_attr_epen_rotation.attr,
	&dev_attr_epen_hand.attr,
#endif
	&dev_attr_epen_reset.attr,
	&dev_attr_epen_reset_result.attr,
	&dev_attr_epen_checksum.attr,
	&dev_attr_epen_checksum_result.attr,
#ifdef WACOM_USE_AVE_TRANSITION
	&dev_attr_epen_ave.attr,
	&dev_attr_epen_ave_result.attr,
#endif
#ifdef WACOM_CONNECTION_CHECK
	&dev_attr_epen_connection.attr,
#endif
#ifdef BATTERY_SAVING_MODE
	&dev_attr_epen_saving_mode.attr,
#endif
	&dev_attr_epen_boxfilter_threshold.attr,
	&dev_attr_epen_boxfilter_threshold_hover.attr,
	&dev_attr_epen_boost_freq.attr,
	&dev_attr_epen_min_pressure.attr,
	&dev_attr_epen_side_button_mode.attr,
	&dev_attr_epen_fixed_pressure.attr,
	&dev_attr_epen_avg_strength.attr,
	&dev_attr_epen_offset_override_x.attr,
	&dev_attr_epen_offset_override_y.attr,
	&dev_attr_epen_offset_override_enable.attr,
	&dev_attr_epen_touchkey_block_length.attr,
	&dev_attr_epen_hover_side_hotspot_height.attr,
	&dev_attr_epen_hover_side_left_keycode.attr,
	&dev_attr_epen_hover_side_left_keydelay.attr,
	&dev_attr_epen_hover_side_center_keycode.attr,
	&dev_attr_epen_hover_side_center_keydelay.attr,
	&dev_attr_epen_hover_side_right_keycode.attr,
	&dev_attr_epen_hover_side_right_keydelay.attr,
	&dev_attr_epen_worryfree_mode.attr,
	&dev_attr_epen_padding_top.attr,
	&dev_attr_epen_padding_right.attr,
	&dev_attr_epen_padding_bottom.attr,
	&dev_attr_epen_padding_left.attr,
	NULL,
};

static struct attribute_group epen_attr_group = {
	.attrs = epen_attributes,
};

static int wacom_i2c_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct wacom_g5_platform_data *pdata = client->dev.platform_data;
	struct wacom_i2c *wac_i2c;
	struct input_dev *input;
	int ret = 0;
#if defined(CONFIG_MACH_T0)
	int digitizer_type = 0;
#endif

	firmware_updating_state = false;

	if (pdata == NULL) {
		printk(KERN_ERR "%s: no pdata\n", __func__);
		ret = -ENODEV;
		goto err_i2c_fail;
	}

	/*Check I2C functionality */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "[E-PEN] No I2C functionality found\n");
		ret = -ENODEV;
		goto err_i2c_fail;
	}

	/*Obtain kernel memory space for wacom i2c */
	wac_i2c = kzalloc(sizeof(struct wacom_i2c), GFP_KERNEL);
	if (NULL == wac_i2c) {
		printk(KERN_ERR "[E-PEN] failed to allocate wac_i2c.\n");
		ret = -ENOMEM;
		goto err_freemem;
	}

	wac_i2c->client_boot = i2c_new_dummy(client->adapter,
		WACOM_I2C_BOOT);
	if (!wac_i2c->client_boot) {
		dev_err(&client->dev, "Fail to register sub client[0x%x]\n",
			 WACOM_I2C_BOOT);
	}

	input = input_allocate_device();
	if (NULL == input) {
		printk(KERN_ERR "[E-PEN] failed to allocate input device.\n");
		ret = -ENOMEM;
		goto err_input_allocate_device;
	} else
		wacom_i2c_set_input_values(client, wac_i2c, input);

	wac_i2c->wac_feature = &wacom_feature_EMR;
	wac_i2c->wac_pdata = pdata;
	wac_i2c->input_dev = input;
	wac_i2c->client = client;
	wac_i2c->irq = client->irq;
#ifdef WACOM_PDCT_WORK_AROUND
	wac_i2c->irq_pdct = gpio_to_irq(pdata->gpio_pendct);
#endif

#ifdef WACOM_PEN_DETECT
	wac_i2c->gpio_pen_insert = pdata->gpio_pen_insert;
#endif

	/*Change below if irq is needed */
	wac_i2c->irq_flag = 1;

	/*Register callbacks */
	wac_i2c->callbacks.check_prox = wacom_check_emr_prox;
	if (wac_i2c->wac_pdata->register_cb)
		wac_i2c->wac_pdata->register_cb(&wac_i2c->callbacks);

	/* Firmware Feature */
	wacom_i2c_init_firm_data();
#ifdef WACOM_IMPORT_FW_ALGO
	wac_i2c->use_offset_table = true;
	wac_i2c->use_aveTransition = false;
#endif

#if defined(CONFIG_MACH_Q1_BD)
	/* Change Origin offset by rev */
	if (system_rev < 6) {
		origin_offset[0] = origin_offset_48[0];
		origin_offset[1] = origin_offset_48[1];
	}
	/* Reset IC */
	wacom_i2c_reset_hw(wac_i2c->wac_pdata);
#elif defined(CONFIG_MACH_T0)
	wac_i2c->wac_pdata->late_resume_platform_hw();
	msleep(200);

	/*Set data by digitizer type*/
	digitizer_type = wacom_i2c_get_digitizer_type();

	if (digitizer_type == EPEN_DTYPE_B746) {
		printk(KERN_DEBUG"[E-PEN] Use Box filter\n");
		wac_i2c->use_aveTransition = true;
	} else if (digitizer_type == EPEN_DTYPE_B713) {
		printk(KERN_DEBUG"[E-PEN] Reset tilt for B713\n");

		/*Change tuning version for B713*/
		tuning_version = tuning_version_B713;

		memcpy(tilt_offsetX, tilt_offsetX_B713, sizeof(tilt_offsetX));
		memcpy(tilt_offsetY, tilt_offsetY_B713, sizeof(tilt_offsetY));
	} else if (digitizer_type == EPEN_DTYPE_B660) {
		printk(KERN_DEBUG"[E-PEN] Reset tilt and origin for B660\n");

		origin_offset[0] = EPEN_B660_ORG_X;
		origin_offset[1] = EPEN_B660_ORG_Y;
		memset(tilt_offsetX, 0, sizeof(tilt_offsetX));
		memset(tilt_offsetY, 0, sizeof(tilt_offsetY));
		wac_i2c->use_offset_table = false;
	}

	/*Set switch type*/
	wac_i2c->invert_pen_insert = wacom_i2c_invert_by_switch_type();
#endif
#ifdef WACOM_PDCT_WORK_AROUND
	wac_i2c->pen_pdct = PDCT_NOSIGNAL;
#endif

#if defined(CONFIG_MACH_P4NOTE)
	wac_i2c->wac_pdata->resume_platform_hw();
	msleep(200);
#endif
	wac_i2c->power_enable = true;

	ret = wacom_i2c_query(wac_i2c);

	if (ret < 0)
		epen_reset_result = false;
	else
		epen_reset_result = true;

#if defined(CONFIG_MACH_P4NOTE)
	input_set_abs_params(input, ABS_X, WACOM_POSX_OFFSET,
			     wac_i2c->wac_feature->x_max, 4, 0);
	input_set_abs_params(input, ABS_Y, WACOM_POSY_OFFSET,
			     wac_i2c->wac_feature->y_max, 4, 0);
	input_set_abs_params(input, ABS_PRESSURE, 0,
			     WACOM_MAX_PRESSURE, 0, 0);
#else
	input_set_abs_params(wac_i2c->input_dev, ABS_X, pdata->min_x,
			     pdata->max_x, 4, 0);
	input_set_abs_params(wac_i2c->input_dev, ABS_Y, pdata->min_y,
			     pdata->max_y, 4, 0);
#ifdef CONFIG_MACH_T0
	input_set_abs_params(wac_i2c->input_dev, ABS_PRESSURE,
			0, wac_i2c->wac_feature->pressure_max, 0, 0);
#else
	input_set_abs_params(wac_i2c->input_dev, ABS_PRESSURE,
			     pdata->min_pressure, pdata->max_pressure, 0, 0);
#endif
#endif
	input_set_drvdata(input, wac_i2c);

	/*Before registering input device, data in each input_dev must be set */
	ret = input_register_device(input);
	if (ret) {
		pr_err("[E-PEN] failed to register input device.\n");
		goto err_register_device;
	}

	/*Change below if irq is needed */
	wac_i2c->irq_flag = 1;

	/*Set client data */
	i2c_set_clientdata(client, wac_i2c);
	i2c_set_clientdata(wac_i2c->client_boot, wac_i2c);

	/*Initializing for semaphor */
	mutex_init(&wac_i2c->lock);
	wake_lock_init(&wac_i2c->wakelock, WAKE_LOCK_SUSPEND, "wacom");
	INIT_DELAYED_WORK(&wac_i2c->resume_work, wacom_i2c_resume_work);
#if defined(WACOM_IRQ_WORK_AROUND)
	INIT_DELAYED_WORK(&wac_i2c->pendct_dwork, wacom_i2c_pendct_work);
#endif
#ifdef WACOM_STATE_CHECK
	INIT_DELAYED_WORK(&wac_i2c->wac_statecheck_work, wac_statecheck_work);
#endif


#ifdef CONFIG_HAS_EARLYSUSPEND
	wac_i2c->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	wac_i2c->early_suspend.suspend = wacom_i2c_early_suspend;
	wac_i2c->early_suspend.resume = wacom_i2c_late_resume;
	register_early_suspend(&wac_i2c->early_suspend);
#endif

	wac_i2c->dev = device_create(sec_class, NULL, 0, NULL, "sec_epen");
	if (IS_ERR(wac_i2c->dev)) {
		printk(KERN_ERR "Failed to create device(wac_i2c->dev)!\n");
		goto err_sysfs_create_group;
	} else {
		dev_set_drvdata(wac_i2c->dev, wac_i2c);
		ret = sysfs_create_group(&wac_i2c->dev->kobj, &epen_attr_group);
		if (ret) {
			printk(KERN_ERR
			       "[E-PEN]: failed to create sysfs group\n");
			goto err_sysfs_create_group;
		}
	}

	/* firmware info */
	printk(KERN_NOTICE "[E-PEN] wacom fw ver : 0x%x, new fw ver : 0x%x\n",
	       wac_i2c->wac_feature->fw_version, Firmware_version_of_file);

#ifdef CONFIG_SEC_TOUCHSCREEN_DVFS_LOCK
	INIT_DELAYED_WORK(&wac_i2c->dvfs_work, free_dvfs_lock);
	if (exynos_cpufreq_get_level(WACOM_DVFS_LOCK_FREQ,
			&wac_i2c->cpufreq_level))
		printk(KERN_ERR "[E-PEN] exynos_cpufreq_get_level Error\n");
#ifdef SEC_BUS_LOCK
	wac_i2c->dvfs_lock_status = false;
#if defined(CONFIG_MACH_P4NOTE)
	wac_i2c->bus_dev = dev_get("exynos-busfreq");
#endif	/* CONFIG_MACH_P4NOTE */
#endif	/* SEC_BUS_LOCK */
#endif	/* CONFIG_SEC_TOUCHSCREEN_DVFS_LOCK */
	
	INIT_DELAYED_WORK(&wac_i2c->activitylockout_work, epen_activity_lockout_free);

	/*Request IRQ */
	if (wac_i2c->irq_flag) {
		ret =
		    request_threaded_irq(wac_i2c->irq, NULL, wacom_interrupt,
					 IRQF_DISABLED | IRQF_TRIGGER_RISING |
					 IRQF_ONESHOT, wac_i2c->name, wac_i2c);
		if (ret < 0) {
			printk(KERN_ERR
			       "[E-PEN] failed to request irq(%d) - %d\n",
			       wac_i2c->irq, ret);
			goto err_request_irq;
		}

#if defined(WACOM_PDCT_WORK_AROUND)
		ret =
			request_threaded_irq(wac_i2c->irq_pdct, NULL,
					wacom_interrupt_pdct,
					IRQF_DISABLED | IRQF_TRIGGER_RISING |
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					wac_i2c->name, wac_i2c);
		if (ret < 0) {
			printk(KERN_ERR
				"[E-PEN] failed to request irq(%d) - %d\n",
				wac_i2c->irq_pdct, ret);
			goto err_request_irq;
		}
#endif
	}

#ifdef WACOM_DEBOUNCEINT_BY_ESD
	/*Invert gpio value for  first irq.
	    schedule_delayed_work in wacom_i2c_input_open*/
	pen_insert_state = gpio_get_value(wac_i2c->gpio_pen_insert);
	wac_i2c->pen_insert = pen_insert_state;
#if defined(CONFIG_MACH_T0)
	if (wac_i2c->invert_pen_insert) {
		wac_i2c->pen_insert = !wac_i2c->pen_insert;
		pen_insert_state = wac_i2c->pen_insert;
	}
#endif
#endif

	return 0;

 err_request_irq:
 err_sysfs_create_group:
 err_register_device:
	input_unregister_device(input);
 err_input_allocate_device:
	input_free_device(input);
 err_freemem:
	kfree(wac_i2c);
 err_i2c_fail:
	return ret;
}

static const struct i2c_device_id wacom_i2c_id[] = {
	{"wacom_g5sp_i2c", 0},
	{},
};

/*Create handler for wacom_i2c_driver*/
static struct i2c_driver wacom_i2c_driver = {
	.driver = {
		   .name = "wacom_g5sp_i2c",
		   },
	.probe = wacom_i2c_probe,
	.remove = wacom_i2c_remove,
	.suspend = wacom_i2c_suspend,
	.resume = wacom_i2c_resume,
	.id_table = wacom_i2c_id,
};

static int __init wacom_i2c_init(void)
{
	int ret = 0;
	
	epen_cpufreq_level = -1;

#if defined(WACOM_SLEEP_WITH_PEN_SLP)
	printk(KERN_ERR "[E-PEN] %s: Sleep type-PEN_SLP pin\n", __func__);
#elif defined(WACOM_SLEEP_WITH_PEN_LDO_EN)
	printk(KERN_ERR "[E-PEN] %s: Sleep type-PEN_LDO_EN pin\n", __func__);
#endif

	ret = i2c_add_driver(&wacom_i2c_driver);
	if (ret)
		printk(KERN_ERR "[E-PEN] fail to i2c_add_driver\n");
	return ret;
}

static void __exit wacom_i2c_exit(void)
{
	i2c_del_driver(&wacom_i2c_driver);
}

late_initcall(wacom_i2c_init);
module_exit(wacom_i2c_exit);

MODULE_AUTHOR("Samsung");
MODULE_DESCRIPTION("Driver for Wacom G5SP Digitizer Controller");

MODULE_LICENSE("GPL");
