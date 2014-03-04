/*
 * mms_ts.c - Touchscreen driver for Melfas MMS-series touch controllers
 *
 * Copyright (C) 2011 Google Inc.
 * Author: Dima Zavin <dima@android.com>
 *         Simon Wilson <simonwilson@google.com>
 *
 * ISP reflashing code based on original code from Melfas.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#define DEBUG
/* #define VERBOSE_DEBUG */
#define SEC_TSP_DEBUG

/* #define FORCE_FW_FLASH */
/* #define FORCE_FW_PASS */
/* #define ESD_DEBUG */

#define SEC_TSP_FACTORY_TEST
#define SEC_TSP_FW_UPDATE
#define TSP_BUF_SIZE 1024
#define FAIL -1
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <mach/gpio.h>
#include <linux/uaccess.h>
#include <linux/wacom_i2c.h>
#include <linux/cpufreq.h>

#include <mach/cpufreq.h>
#include <mach/dev.h>

#include <linux/cpuidle.h>

#ifdef CONFIG_TOUCHSCREEN_GESTURES
#include <linux/spinlock.h>
#include <linux/miscdevice.h>

#define MAX_GESTURES 30
#define MAX_GESTURE_FINGERS 5
#define MAX_GESTURE_STEPS 10

#define GESTURE_BOOSTER				1		// enable the gesture booster.
#define GESTURE_BOOSTER_FREQ		1600000		// frequency to boost to.
#define GESTURE_BOOSTER_DURATION	2500	// duration to hold boost (msecs).

// Definitions
struct gesture_point {
	int min_x;
	int min_y;
	int max_x;
	int max_y;
};

int ary_gestures_flg_touchkey[MAX_GESTURES];
int ary_gestures_flg_exclusive[MAX_GESTURES];

typedef struct gesture_point gesture_points_t[MAX_GESTURES][MAX_GESTURE_FINGERS][MAX_GESTURE_STEPS];
static gesture_points_t gesture_points = { { { { 0, 0, 0, 0 } } } };

static int max_configured_gesture = -1;
static int max_gesture_finger[MAX_GESTURES] = { -1 };
typedef int gestures_step_count_t[MAX_GESTURES][MAX_GESTURE_FINGERS];
static gestures_step_count_t gestures_step_count = { { 0 } };

// Track state
static bool gestures_detected[MAX_GESTURES] = { false };
static bool has_gestures = false;

bool ignore_gestures = false;

struct gesture_finger {
	int finger_order;
	int current_step;
};
static struct gesture_finger gesture_fingers[MAX_GESTURES][MAX_GESTURE_FINGERS] = { { { -1, -1 } } };

// Enabled state
static bool gestures_enabled = true;
DECLARE_WAIT_QUEUE_HEAD(gestures_wq);
static spinlock_t gestures_lock;

#if GESTURE_BOOSTER
static unsigned int gesturebooster_enabled = GESTURE_BOOSTER;
static unsigned int gesturebooster_freq = GESTURE_BOOSTER_FREQ;
static unsigned int gesturebooster_duration = GESTURE_BOOSTER_DURATION;
static int gesturebooster_actual_freq;
static int prev_gesturebooster_actual_freq;
static int gesturebooster_cpufreq_level;
static int gesturebooster_dvfs_locked;
static struct mutex gesturebooster_dvfs_lock;
static struct delayed_work work_gesturebooster_dvfs_off;
static int flg_gestures_detected = 0;
bool flg_gestures_touchkey_was_pressed = false;
#endif
#endif

static bool sttg_gestures_only = false;
static bool sttg_require_touchkey = false;
#include <linux/platform_data/mms152_ts.h>

#ifdef CONFIG_CPU_FREQ_GOV_ONDEMAND_FLEXRATE
#include <mach/midas-tsp.h>
#endif

#include <asm/unaligned.h>

#include "../keyboard/cypress/cypress-touchkey.h"

#ifdef CONFIG_CPU_FREQ_GOV_ONDEMAND_FLEXRATE
#include <mach/midas-tsp.h>
#endif

#ifdef CONFIG_INPUT_FBSUSPEND
#ifdef CONFIG_DRM
#include <drm/drm_backlight.h>
#endif
#include <linux/fb.h>
#endif

#ifdef CONFIG_TOUCH_WAKE
#include <linux/touch_wake.h>
#endif

#define MAX_FINGERS		10
#define MAX_WIDTH		30
#define MAX_PRESSURE		255
#define MAX_ANGLE		90
#define MIN_ANGLE		-90

/* Registers */
#define MMS_MODE_CONTROL	0x01
#define MMS_XYRES_HI		0x02
#define MMS_XRES_LO		0x03
#define MMS_YRES_LO		0x04

#define MMS_INPUT_EVENT_PKT_SZ	0x0F
#define MMS_INPUT_EVENT0	0x10
#define EVENT_SZ_PALM	8
#define EVENT_SZ_OLD	6

#define MMS_CORE_VERSION	0xE1
#define MMS_TSP_REVISION	0xF0
#define MMS_HW_REVISION		0xF1
#define MMS_COMPAT_GROUP	0xF2
#define MMS_FW_VERSION		0xF4

enum {
	ISP_MODE_FLASH_ERASE = 0x59F3,
	ISP_MODE_FLASH_WRITE = 0x62CD,
	ISP_MODE_FLASH_READ = 0x6AC9,
};

/* each address addresses 4-byte words */
#define ISP_MAX_FW_SIZE		(0x1F00 * 4)
#define ISP_IC_INFO_ADDR	0x1F00
#define ISP_CAL_DATA_SIZE	256

#ifdef SEC_TSP_FW_UPDATE

#define WORD_SIZE		4

#define ISC_PKT_SIZE			1029
#define ISC_PKT_DATA_SIZE		1024
#define ISC_PKT_HEADER_SIZE		3
#define ISC_PKT_NUM				31

#define ISC_ENTER_ISC_CMD		0x5F
#define ISC_ENTER_ISC_DATA		0x01
#define ISC_CMD					0xAE
#define ISC_ENTER_UPDATE_DATA		0x55
#define ISC_ENTER_UPDATE_DATA_LEN	9
#define ISC_DATA_WRITE_SUB_CMD		0xF1
#define ISC_EXIT_ISC_SUB_CMD		0x0F
#define ISC_EXIT_ISC_SUB_CMD2		0xF0
#define ISC_CHECK_STATUS_CMD		0xAF
#define ISC_CONFIRM_CRC			0x03
#define ISC_DEFAULT_CRC			0xFFFF

#endif

#ifdef SEC_TSP_FACTORY_TEST
#define TX_NUM_A	29
#define TX_NUM_M	30
#define RX_NUM		17
#define NODE_NUM	510	/* 30x17 */

/* self diagnostic */
#define ADDR_CH_NUM		0x0B
#define ADDR_UNIV_CMD	0xA0
#define CMD_ENTER_TEST	0x40
#define CMD_EXIT_TEST	0x4F
#define CMD_CM_DELTA	0x41
#define CMD_GET_DELTA	0x42
#define CMD_CM_ABS		0X43
#define CMD_GET_ABS		0X44
#define CMD_CM_JITTER	0X45
#define CMD_GET_JITTER	0X46
#define CMD_GET_INTEN	0x70
#define CMD_RESULT_SZ	0XAE
#define CMD_RESULT		0XAF

/* VSC(Vender Specific Command)  */
#define MMS_VSC_CMD			0xB0	/* vendor specific command */
#define MMS_VSC_MODE			0x1A	/* mode of vendor */

#define MMS_VSC_CMD_ENTER		0X01
#define MMS_VSC_CMD_CM_DELTA		0X02
#define MMS_VSC_CMD_CM_ABS		0X03
#define MMS_VSC_CMD_EXIT		0X05
#define MMS_VSC_CMD_INTENSITY		0X04
#define MMS_VSC_CMD_RAW			0X06
#define MMS_VSC_CMD_REFER		0X07

#define TSP_CMD_STR_LEN 32
#define TSP_CMD_RESULT_STR_LEN 512
#define TSP_CMD_PARAM_NUM 8
#endif /* SEC_TSP_FACTORY_TEST */

/* Touch booster */
#if defined(CONFIG_EXYNOS4_CPUFREQ) &&\
	defined(CONFIG_BUSFREQ_OPP)
#define TOUCH_BOOSTER			0
#define TOUCH_BOOSTER_OFF_TIME		100
#define TOUCH_BOOSTER_CHG_TIME		200
#else
#define TOUCH_BOOSTER			0
#endif

#ifdef CONFIG_CPU_FREQ_LCD_FREQ_DFS
extern int _lcdfreq_lock(int lock);
static unsigned int flg_enable_lcdfreq_touchboost = 0;
#endif

struct device *sec_touchscreen;
static struct device *bus_dev;

int touch_is_pressed;

bool flg_touch_was_pressed = false;

bool flg_touchwake_slide2wake = true;
bool flg_touchwake_arc_started = false;
bool flg_touchwake_swipe_started = false;
bool flg_touchwake_longpressoff = false;

struct timeval time_touchwake_longpressoff;
struct timeval time_touchwake_longpressoff_start;
static int touchwake_longpressoff_time_start_secs = -1;
static spinlock_t touchwake_longpressoff_lock;

static void touchwake_reset_longpressoff(struct work_struct * touchwake_reset_longpressoff_work);
static DECLARE_DELAYED_WORK(touchwake_reset_longpressoff_work, touchwake_reset_longpressoff);

static void touchwake_check_longpressoff(struct work_struct * touchwake_check_longpressoff_work);
static DECLARE_DELAYED_WORK(touchwake_check_longpressoff_work, touchwake_check_longpressoff);

static unsigned int arc_start_x_min;
static unsigned int arc_start_x_max;
static unsigned int arc_start_y_min;
static unsigned int arc_start_y_max;
static unsigned int arc_end_y_min;
static unsigned int arc_end_y_max;

static unsigned int rh_arc_end_x_min;
static unsigned int rh_arc_end_x_max;
static unsigned int lh_arc_end_x_min;
static unsigned int lh_arc_end_x_max;

static int wake_start = -1;
static unsigned int swipe_x_start = 9999;
static unsigned int swipe_y_start = 9999;
static unsigned int x_lo;
static unsigned int x_hi;

static bool sttg_longpressoff_alwayson = false;
bool flg_screen_on = true;

static bool flg_swipe_in_progress = false;

static unsigned int delta_between_press;
static unsigned int ctr_typingbooster;
static int64_t time_last_pressed;

unsigned int sttg_typingbooster_mincores = 0;
unsigned int sttg_typingbooster_upthreshold = 50;
unsigned int sttg_typingbooster_cycles = 30;
unsigned int sttg_typingbooster_mintaps = 3;
unsigned int sttg_typingbooster_maxmsbetweentaps = 250;
unsigned int sttg_typingbooster_taptimeoutms = 1000;

unsigned int flg_ctr_typingbooster_cycles = 0;

static bool sttg_touchbooster_enabled = true;
static unsigned int sttg_touchbooster_freq = 1000000;
static unsigned int sttg_touchbooster_duration = 100;
static unsigned int sttg_touchbooster_relax_delay = 300;
static unsigned int sttg_touchbooster_relax_freq = 600000;
static unsigned int sttg_touchbooster_mincores = 0;
static unsigned int sttg_touchbooster_relax_mincores = 1;

static struct delayed_work work_touchbooster_off;
static struct delayed_work work_touchbooster_relax;
static int touchbooster_actual_freq;
static int prev_touchbooster_actual_freq;
static int touchbooster_cpufreq_level;
static int prev_touchbooster_relaxed_freq;
static int touchbooster_relaxed_cpufreq_level;
static int touchbooster_dvfs_locked;
static struct mutex touchbooster_dvfs_lock;

#define ISC_DL_MODE	1

/* 5.55" OCTA LCD */
#define FW_VERSION_L 0x29
#define FW_VERSION_M 0x50
#define MAX_FW_PATH 255
#define TSP_FW_FILENAME "melfas_fw.bin"

#if ISC_DL_MODE	/* ISC_DL_MODE start */

/*
 *      Default configuration of ISC mode
 */
#define DEFAULT_SLAVE_ADDR	0x48

#define SECTION_NUM		3
#define SECTION_NAME_LEN	5

#define PAGE_HEADER		3
#define PAGE_DATA		1024
#define PAGE_TAIL		2
#define PACKET_SIZE		(PAGE_HEADER + PAGE_DATA + PAGE_TAIL)
#define TS_WRITE_REGS_LEN		1030

#define TIMEOUT_CNT		10
#define STRING_BUF_LEN		100


/* State Registers */
#define MIP_ADDR_INPUT_INFORMATION	0x01

#define ISC_ADDR_VERSION		0xE1
#define ISC_ADDR_SECTION_PAGE_INFO	0xE5

/* Config Update Commands */
#define ISC_CMD_ENTER_ISC		0x5F
#define ISC_CMD_ENTER_ISC_PARA1		0x01
#define ISC_CMD_UPDATE_MODE		0xAE
#define ISC_SUBCMD_ENTER_UPDATE		0x55
#define ISC_SUBCMD_DATA_WRITE		0XF1
#define ISC_SUBCMD_LEAVE_UPDATE_PARA1	0x0F
#define ISC_SUBCMD_LEAVE_UPDATE_PARA2	0xF0
#define ISC_CMD_CONFIRM_STATUS		0xAF

#define ISC_STATUS_UPDATE_MODE		0x01
#define ISC_STATUS_CRC_CHECK_SUCCESS	0x03

#define ISC_CHAR_2_BCD(num)	(((num/10)<<4) + (num%10))
#define ISC_MAX(x, y)		(((x) > (y)) ? (x) : (y))

static const char section_name[SECTION_NUM][SECTION_NAME_LEN] = {
	"BOOT", "CORE", "CONF"
};

static const unsigned char crc0_buf[31] = {
	0x1D, 0x2C, 0x05, 0x34, 0x95, 0xA4, 0x8D, 0xBC,
	0x59, 0x68, 0x41, 0x70, 0xD1, 0xE0, 0xC9, 0xF8,
	0x3F, 0x0E, 0x27, 0x16, 0xB7, 0x86, 0xAF, 0x9E,
	0x7B, 0x4A, 0x63, 0x52, 0xF3, 0xC2, 0xEB
};

static const unsigned char crc1_buf[31] = {
	0x1E, 0x9C, 0xDF, 0x5D, 0x76, 0xF4, 0xB7, 0x35,
	0x2A, 0xA8, 0xEB, 0x69, 0x42, 0xC0, 0x83, 0x01,
	0x04, 0x86, 0xC5, 0x47, 0x6C, 0xEE, 0xAD, 0x2F,
	0x30, 0xB2, 0xF1, 0x73, 0x58, 0xDA, 0x99
};

enum {
	ISC_NONE = -1,
	ISC_SUCCESS = 0,
	ISC_FILE_OPEN_ERROR,
	ISC_FILE_CLOSE_ERROR,
	ISC_FILE_FORMAT_ERROR,
	ISC_WRITE_BUFFER_ERROR,
	ISC_I2C_ERROR,
	ISC_UPDATE_MODE_ENTER_ERROR,
	ISC_CRC_ERROR,
	ISC_VALIDATION_ERROR,
	ISC_COMPATIVILITY_ERROR,
	ISC_UPDATE_SECTION_ERROR,
	ISC_SLAVE_ERASE_ERROR,
	ISC_SLAVE_DOWNLOAD_ERROR,
	ISC_DOWNLOAD_WHEN_SLAVE_IS_UPDATED_ERROR,
	ISC_INITIAL_PACKET_ERROR,
	ISC_NO_NEED_UPDATE_ERROR,
	ISC_LIMIT
};

enum {
	EC_NONE = -1,
	EC_DEPRECATED = 0,
	EC_BOOTLOADER_RUNNING = 1,
	EC_BOOT_ON_SUCCEEDED = 2,
	EC_ERASE_END_MARKER_ON_SLAVE_FINISHED = 3,
	EC_SLAVE_DOWNLOAD_STARTS = 4,
	EC_SLAVE_DOWNLOAD_FINISHED = 5,
	EC_2CHIP_HANDSHAKE_FAILED = 0x0E,
	EC_ESD_PATTERN_CHECKED = 0x0F,
	EC_LIMIT
};

enum {
	SEC_NONE = -1,
	SEC_BOOTLOADER = 0,
	SEC_CORE,
	SEC_CONFIG,
	SEC_LIMIT
};

struct tISCFWInfo_t {
	unsigned char version;
	unsigned char compatible_version;
	unsigned char start_addr;
	unsigned char end_addr;
};

static struct tISCFWInfo_t mbin_info[SECTION_NUM];
static struct tISCFWInfo_t ts_info[SECTION_NUM];
static bool section_update_flag[SECTION_NUM];

const struct firmware *fw_mbin[SECTION_NUM];

static unsigned char g_wr_buf[1024 + 3 + 2];
#endif

enum fw_flash_mode {
	ISP_FLASH,
	ISC_FLASH,
};

enum {
	BUILT_IN = 0,
	UMS,
	REQ_FW,
};

struct tsp_callbacks {
	void (*inform_charger)(struct tsp_callbacks *tsp_cb, bool mode);
};

#ifdef CONFIG_LCD_FREQ_SWITCH
struct tsp_lcd_callbacks {
	void (*inform_lcd)(struct tsp_lcd_callbacks *tsp_cb, bool en);
};
#endif

bool bus_dvfs_lock_status;
struct mutex bus_dvfs_lock;
struct delayed_work work_bus_dvfs_off;

struct mms_ts_info {
	struct i2c_client *client;
	struct input_dev *input_dev;
	char phys[32];
	int max_x;
	int max_y;
	bool invert_x;
	bool invert_y;
	u8 palm_flag;
	const u8 *config_fw_version;
	int irq;
	int (*power) (bool on);
	void (*input_event)(void *data);
	int tx_num;

	struct melfas_tsi_platform_data *pdata;
	struct early_suspend early_suspend;
    
	/* protects the enabled flag */
	struct mutex lock;
	bool enabled;

	enum fw_flash_mode fw_flash_mode;
	void (*register_cb)(void *);
	struct tsp_callbacks callbacks;
#ifdef CONFIG_LCD_FREQ_SWITCH
	void (*register_lcd_cb)(void *);
	struct tsp_lcd_callbacks lcd_callback;
	bool tsp_lcdfreq_flag;
#endif
	bool ta_status;
	bool noise_mode;
	bool sleep_wakeup_ta_check;

#if defined(SEC_TSP_DEBUG)
	unsigned char finger_state[MAX_FINGERS];
#endif

#if defined(SEC_TSP_FW_UPDATE)
	u8 fw_update_state;
#endif
	u8 fw_ic_ver;
	char panel;
	char ldi; /* LSI : L, Magna : M */
	u8 fw_core_ver;
#if defined(SEC_TSP_FACTORY_TEST)
	struct list_head cmd_list_head;
	u8 cmd_state;
	char cmd[TSP_CMD_STR_LEN];
	int cmd_param[TSP_CMD_PARAM_NUM];
	char cmd_result[TSP_CMD_RESULT_STR_LEN];
	struct mutex cmd_lock;
	bool cmd_is_running;

	unsigned int reference[NODE_NUM];
	unsigned int raw[NODE_NUM]; /* CM_ABS */
	unsigned int inspection[NODE_NUM];/* CM_DELTA */
	unsigned int intensity[NODE_NUM];
	bool ft_flag;
#endif				/* SEC_TSP_FACTORY_TEST */

	int (*lcd_type) (void);

#ifdef CONFIG_INPUT_FBSUSPEND
	struct	notifier_block	fb_notif;
	bool was_enabled_at_suspend;
#endif
};

struct mms_fw_image {
	__le32 hdr_len;
	__le32 data_len;
	__le32 fw_ver;
	__le32 hdr_ver;
	u8 data[0];
} __packed;

#ifdef CONFIG_HAS_EARLYSUSPEND
static void mms_ts_early_suspend(struct early_suspend *h);
static void mms_ts_late_resume(struct early_suspend *h);
#endif

#if defined(SEC_TSP_FACTORY_TEST)
#define TSP_CMD(name, func) .cmd_name = name, .cmd_func = func

struct tsp_cmd {
	struct list_head	list;
	const char	*cmd_name;
	void	(*cmd_func)(void *device_data);
};

static void fw_update(void *device_data);
static void get_fw_ver_bin(void *device_data);
static void get_fw_ver_ic(void *device_data);
static void get_config_ver(void *device_data);
static void get_threshold(void *device_data);
static void module_off_master(void *device_data);
static void module_on_master(void *device_data);
/*static void module_off_slave(void *device_data);
static void module_on_slave(void *device_data);*/
static void get_chip_vendor(void *device_data);
static void get_chip_name(void *device_data);
static void get_reference(void *device_data);
static void get_cm_abs(void *device_data);
static void get_cm_delta(void *device_data);
static void get_intensity(void *device_data);
static void get_x_num(void *device_data);
static void get_y_num(void *device_data);
static void run_reference_read(void *device_data);
static void run_cm_abs_read(void *device_data);
static void run_cm_delta_read(void *device_data);
static void run_intensity_read(void *device_data);
static void not_support_cmd(void *device_data);

struct tsp_cmd tsp_cmds[] = {
	{TSP_CMD("fw_update", fw_update),},
	{TSP_CMD("get_fw_ver_bin", get_fw_ver_bin),},
	{TSP_CMD("get_fw_ver_ic", get_fw_ver_ic),},
	{TSP_CMD("get_config_ver", get_config_ver),},
	{TSP_CMD("get_threshold", get_threshold),},
	{TSP_CMD("module_off_master", module_off_master),},
	{TSP_CMD("module_on_master", module_on_master),},
	{TSP_CMD("module_off_slave", not_support_cmd),},
	{TSP_CMD("module_on_slave", not_support_cmd),},
	{TSP_CMD("get_chip_vendor", get_chip_vendor),},
	{TSP_CMD("get_chip_name", get_chip_name),},
	{TSP_CMD("get_x_num", get_x_num),},
	{TSP_CMD("get_y_num", get_y_num),},
	{TSP_CMD("get_reference", get_reference),},
	{TSP_CMD("get_cm_abs", get_cm_abs),},
	{TSP_CMD("get_cm_delta", get_cm_delta),},
	{TSP_CMD("get_intensity", get_intensity),},
	{TSP_CMD("run_reference_read", run_reference_read),},
	{TSP_CMD("run_cm_abs_read", run_cm_abs_read),},
	{TSP_CMD("run_cm_delta_read", run_cm_delta_read),},
	{TSP_CMD("run_intensity_read", run_intensity_read),},
	{TSP_CMD("not_support_cmd", not_support_cmd),},
};
#endif

static inline int64_t get_time_ms(void) {
	int64_t ms;
	struct timespec cur_time = current_kernel_time();
	ms =  cur_time.tv_sec * MSEC_PER_SEC;
	ms += cur_time.tv_nsec / NSEC_PER_MSEC;
	return ms;
}

static void touchwake_reset_longpressoff(struct work_struct * touchwake_reset_longpressoff_work)
{
    flg_touchwake_longpressoff = false;
    pr_info("[TSP/touch] flg_touchwake_longpressoff unset\n");
	return;
}

#ifdef CONFIG_TOUCHSCREEN_GESTURES
static void press_power()
{
	input_event(powerkey_device, EV_KEY, KEY_POWER, 1);
    //input_event(powerkey_device, EV_SYN, 0, 0);
    input_sync(powerkey_device);
    msleep(10);
    input_event(powerkey_device, EV_KEY, KEY_POWER, 0);
    input_sync(powerkey_device);
    //input_event(powerkey_device, EV_SYN, 0, 0);
}

static void touchwake_check_longpressoff(struct work_struct * touchwake_check_longpressoff_work)
{
    // time is up from when the longpressoff started. has it been broken?
    if (flg_screen_on && flg_touchwake_longpressoff && swipe_x_start < 9999) {
        // looks good, press power.
        
        // turn off!
        flg_screen_on = false;
        pr_info("[TSP/touch]: longpressoff still down, pressing power...\n");
        touchwake_longpressoff_time_start_secs = -1;
        flg_touchwake_longpressoff = false;
        
        press_power();
        return;
    }
    
    pr_info("[TSP/touch] longpressoff not down anymore, skipping\n");
	return;
}

// probably should be called with the gestures_lock spinlock held
static void reset_gesture_progress(int gesture_no)
{
    int finger_no;
    
    gestures_detected[gesture_no] = false;
    for (finger_no = 0; finger_no <= max_gesture_finger[gesture_no]; finger_no++) {
        gesture_fingers[gesture_no][finger_no].finger_order = -1;
        gesture_fingers[gesture_no][finger_no].current_step = -1;
    }
}
#endif

////////////////// TOUCHBOOSTER

static void touchbooster_off (struct work_struct *work)
{
	mutex_lock(&touchbooster_dvfs_lock);
	
	zzmoove_touchbooster_mincores(0);
	exynos_cpufreq_lock_free(DVFS_LOCK_ID_TSP);
	touchbooster_dvfs_locked = 0;
	dev_unlock(bus_dev, sec_touchscreen);
	bus_dvfs_lock_status = false;
	//pr_info("[TSP/touchbooster] boost off!\n");
	
	mutex_unlock(&touchbooster_dvfs_lock);
}

static void touchbooster_relax (struct work_struct *work)
{
	mutex_lock(&touchbooster_dvfs_lock);
	
	if (sttg_touchbooster_relax_freq) {
		// reset the cpu lock for a lower level.
		
		
		if (prev_touchbooster_relaxed_freq != sttg_touchbooster_relax_freq) {
			touchbooster_relaxed_cpufreq_level = -1;
		}
		
		if (touchbooster_relaxed_cpufreq_level < 0) {
			
			prev_touchbooster_relaxed_freq = sttg_touchbooster_relax_freq;
			
			exynos_cpufreq_get_level(sttg_touchbooster_relax_freq, &touchbooster_relaxed_cpufreq_level);
			pr_info("[TSP/touchbooster] got level %d for freq: %d\n", touchbooster_relaxed_cpufreq_level, sttg_touchbooster_relax_freq);
		}
		
		exynos_cpufreq_lock_free(DVFS_LOCK_ID_TSP);
		exynos_cpufreq_lock(DVFS_LOCK_ID_TSP, touchbooster_relaxed_cpufreq_level);
		
		//pr_info("[TSP/touchbooster] relaxed to %d mhz [L%d]\n",
		//		sttg_touchbooster_relax_freq, touchbooster_relaxed_cpufreq_level);
		
		// reset the bus lock to a lower level.
		//dev_lock(bus_dev, sec_touchscreen, 176176);
	}
	
	if (sttg_touchbooster_relax_mincores) {
		if (sttg_touchbooster_relax_mincores == 1) {
			zzmoove_touchbooster_mincores(0);
		} else {
			zzmoove_touchbooster_mincores(sttg_touchbooster_relax_mincores);
		}
	}
	
	dev_unlock(bus_dev, sec_touchscreen);
	bus_dvfs_lock_status = false;
	
	//pr_info("[TSP/touchbooster] boost relaxed!\n");
	
	mutex_unlock(&touchbooster_dvfs_lock);
}

void touchbooster_dvfs_lock_on(int touchbooster_mode, int touchbooster_freq_override)
{
	unsigned int maxfreq = 0;
	unsigned int minfreq = 0;
	
	mutex_lock(&touchbooster_dvfs_lock);
	
    if (touchbooster_freq_override == 0) {
        // no override is set, proceed normally.
        
		// March 3rd 2014 - removed reference to exynos_cpufreq_get_maxfreq to prevent possible exception
		//   and disabled this IF statement, as it is now useless.
        /*if (sttg_touchbooster_freq == 0) {
            pr_info("[TSP/touchbooster] boost_freq max autodetect\n");
            // use the highest frequency we can.
            maxfreq = exynos_cpufreq_get_maxfreq();
            touchbooster_actual_freq = maxfreq;
        } else {*/
            // use whatever frequency has been set.
            touchbooster_actual_freq = sttg_touchbooster_freq;
        /*}*/
        
    } else {
        // use the override frequency instead.
        touchbooster_actual_freq = touchbooster_freq_override;
    }
    
    // if this has changed, we need to refresh the level.
    if (prev_touchbooster_actual_freq != touchbooster_actual_freq) {
        touchbooster_cpufreq_level = -1;
        pr_info("[TSP/touchbooster] boost_freq changed, level refresh needed, doing so now\n");
    }
	
	if (touchbooster_cpufreq_level < 0) {
		
		if (maxfreq == 0) {
			// check to see if we already have this value.
			// we don't, so get it.
			
			// March 3rd 2014 - removed old reference to exynos_cpufreq_get_maxfreq to prevent possible exception
			maxfreq = 1600000;
		}
		minfreq = exynos_cpufreq_get_minfreq();
		
		if (maxfreq < 1) {
			// something went wrong. just set a safe value.
			maxfreq = 1000000;
		}
		
		if (minfreq < 1) {
			// something went wrong. just set a safe value.
			minfreq = 200000;
		}
		
		if (touchbooster_actual_freq < minfreq) {
			touchbooster_actual_freq = minfreq;
			pr_info("[TSP/touchbooster] boost_freq too low. set to: %d\n", minfreq);
		} else if (touchbooster_actual_freq > maxfreq) {
			touchbooster_actual_freq = maxfreq;
			pr_info("[TSP/touchbooster] boost_freq too high. set to: %d\n", maxfreq);
		}
		
		prev_touchbooster_actual_freq = touchbooster_actual_freq;
		
		exynos_cpufreq_get_level(touchbooster_actual_freq, &touchbooster_cpufreq_level);
		pr_info("[TSP/touchbooster] got level %d for freq: %d\n", touchbooster_cpufreq_level, touchbooster_actual_freq);
	}
	
	if (touchbooster_mode == 0) {
		// this means the last finger has pulled away.
		// cancel checks, and schedule off_work.
		
		cancel_delayed_work_sync(&work_touchbooster_relax);
		schedule_delayed_work(&work_touchbooster_off, msecs_to_jiffies(sttg_touchbooster_duration));
		
	} else if (touchbooster_mode == 1) {
		// this means a finger is down.
		// set the cpu lock,
		// set the bus lock.
		
		if (!touchbooster_dvfs_locked) {
			
			// get the cpu lock.
			exynos_cpufreq_lock(DVFS_LOCK_ID_TSP, touchbooster_cpufreq_level);
			
			if (sttg_touchbooster_mincores) {
				zzmoove_touchbooster_mincores(sttg_touchbooster_mincores);
			}
			
			// schedule work.
			schedule_delayed_work(&work_touchbooster_relax, msecs_to_jiffies(sttg_touchbooster_relax_delay));
			
			touchbooster_dvfs_locked = 1;
			
			if (!bus_dvfs_lock_status) {
				dev_lock(bus_dev, sec_touchscreen, 440220);
				bus_dvfs_lock_status = true;
			}
			
			//pr_info("[TSP/touchbooster] boost on! - MODE 1 - boosted to %d mhz [L%d] for %d msecs\n",
			//		touchbooster_actual_freq, touchbooster_cpufreq_level, sttg_touchbooster_duration);
		}
		
	} else if (touchbooster_mode == 2) {
		cancel_delayed_work_sync(&work_touchbooster_relax);
		cancel_delayed_work_sync(&work_touchbooster_off);
		schedule_work(&work_touchbooster_off.work);
	}
	
	mutex_unlock(&touchbooster_dvfs_lock);
}

//////////////////////// TOUCHBOOSTER

#if GESTURE_BOOSTER
static void gesturebooster_dvfs_lock_off(struct work_struct *work)
{
	mutex_lock(&gesturebooster_dvfs_lock);

	exynos_cpufreq_lock_free(DVFS_LOCK_ID_GESTURE_BOOSTER);
	gesturebooster_dvfs_locked = 0;
	pr_info("[TSP/gesturebooster] boost off!\n");
	
	mutex_unlock(&gesturebooster_dvfs_lock);
}

void gesturebooster_dvfs_lock_on(int gesturebooster_freq_override)
{
	unsigned int maxfreq = 0;
	unsigned int minfreq = 0;
	
	mutex_lock(&gesturebooster_dvfs_lock);
	
    if (gesturebooster_freq_override == 0) {
        // no override is set, proceed normally.
        
		// March 3rd 2014 - removed reference to exynos_cpufreq_get_maxfreq to prevent possible exception
		//   and disabled this IF statement, as it is now useless.
        /*if (gesturebooster_freq == 0) {
            pr_info("[TSP/gesturebooster] boost_freq max autodetect\n");
            // use the highest frequency we can.
            maxfreq = exynos_cpufreq_get_maxfreq();
            gesturebooster_actual_freq = maxfreq;
        } else {
            // use whatever frequency has been set.*/
            gesturebooster_actual_freq = gesturebooster_freq;
        /*}*/
        
    } else {
        // use the override frequency instead.
        gesturebooster_actual_freq = gesturebooster_freq_override;
    }
    
    // if this has changed, we need to refresh the level.
    if (prev_gesturebooster_actual_freq != gesturebooster_actual_freq) {
        gesturebooster_cpufreq_level = -1;
        pr_info("[TSP/gesturebooster] boost_freq changed, level refresh needed, doing so now\n");
    }
	
	if (gesturebooster_cpufreq_level < 0) {
		
		if (maxfreq == 0) {
			// check to see if we already have this value.
			// we don't, so get it.
			
			// March 3rd 2014 - removed old reference to exynos_cpufreq_get_maxfreq to prevent possible exception
			maxfreq = 1600000;
		}
		minfreq = exynos_cpufreq_get_minfreq();
		
		if (maxfreq < 1) {
			// something went wrong. just set a safe value.
			maxfreq = 1000000;
		}
		
		if (minfreq < 1) {
			// something went wrong. just set a safe value.
			minfreq = 200000;
		}
		
		if (gesturebooster_actual_freq < minfreq) {
			gesturebooster_actual_freq = minfreq;
			pr_info("[TSP/gesturebooster] boost_freq too low. set to: %d\n", minfreq);
		} else if (gesturebooster_actual_freq > maxfreq) {
			gesturebooster_actual_freq = maxfreq;
			pr_info("[TSP/gesturebooster] boost_freq too high. set to: %d\n", maxfreq);
		}
		
		prev_gesturebooster_actual_freq = gesturebooster_actual_freq;
		
		exynos_cpufreq_get_level(gesturebooster_actual_freq, &gesturebooster_cpufreq_level);
		pr_info("[TSP/gesturebooster] got level %d for freq: %d\n", gesturebooster_cpufreq_level, gesturebooster_actual_freq);
	}
	
	if (!gesturebooster_dvfs_locked) {
		
		if (gesturebooster_duration < 1) {
			// give a floor to this setting so STweaks can use 0 as a min value.
			gesturebooster_duration = 1;
		}
		
		exynos_cpufreq_lock(DVFS_LOCK_ID_GESTURE_BOOSTER, gesturebooster_cpufreq_level);
		
		// set timer to disable the boost in x ms.
		schedule_delayed_work(&work_gesturebooster_dvfs_off, msecs_to_jiffies(gesturebooster_duration));
		
		gesturebooster_dvfs_locked = 1;
		pr_info("[TSP/gesturebooster] boost on! boosted to %d mhz [L%d] for %d msecs\n",
				gesturebooster_actual_freq, gesturebooster_cpufreq_level, gesturebooster_duration);
	}

	mutex_unlock(&gesturebooster_dvfs_lock);
}
EXPORT_SYMBOL(gesturebooster_dvfs_lock_on);
#endif

static void bus_dvfs_lock_off(struct work_struct *work)
{
	int ret;

	mutex_lock(&bus_dvfs_lock);

	ret = dev_unlock(bus_dev, sec_touchscreen);
	if (ret < 0)
			pr_err("%s: dev unlock failed(%d)\n", __func__, __LINE__);
    
    flg_ignore_cpuidle = false;

	bus_dvfs_lock_status = false;
	pr_info("[TSP/touch] dvfs buslock off\n");
    
	mutex_unlock(&bus_dvfs_lock);
}

void bus_dvfs_lock_on(unsigned int mode)
{
	int ret;
    int busfreqs = 440220;

	mutex_lock(&bus_dvfs_lock);
    
    cancel_delayed_work(&work_bus_dvfs_off);
    
    if (mode == 0) {
        // if mode is 0, set to the highest memory/bus freqs.
        busfreqs = 440220;
    } else {
        // otherwise set to whatever it wants to override with.
        // yes, i know there should be a sanity check here...
        busfreqs = mode;
    }
    
    if (!bus_dvfs_lock_status) {
		
		flg_ignore_cpuidle = true;
        
        ret = dev_lock(bus_dev, sec_touchscreen, busfreqs);
        if (ret < 0) {
            pr_err("%s: dev lock failed(%d)\n", __func__, __LINE__);
        }
        
        schedule_delayed_work(&work_bus_dvfs_off, msecs_to_jiffies(2000));
        
        bus_dvfs_lock_status = true;
        pr_info("[TSP/touch] dvfs buslock on at %d\n", busfreqs);
        
    }
	mutex_unlock(&bus_dvfs_lock);
}
EXPORT_SYMBOL(bus_dvfs_lock_on);

#ifdef CONFIG_INPUT_FBSUSPEND
#ifdef CONFIG_DRM
static void melfas_set_power(void *priv, int power)
{
	struct mms_ts_info *info = (struct mms_ts_info *)priv;
	int i;

	switch (power) {
	case FB_BLANK_UNBLANK:
		if (info->enabled == 0) {
			info->pdata->power(true);
			msleep(200);
			enable_irq(info->client->irq);
			info->enabled = 1;
		} else {
			pr_err("[TSP] touchscreen already on\n");
		}
		break;
	case FB_BLANK_POWERDOWN:
		for (i = 0; i < MAX_FINGERS; i++) {
			input_mt_slot(info->input_dev, i);
			input_mt_report_slot_state(info->input_dev,
						MT_TOOL_FINGER, false);
			input_sync(info->input_dev);
		}
		if (info->enabled == 1) {
			disable_irq(info->client->irq);
			info->pdata->power(false);
			info->enabled = 0;
		} else {
			pr_err("[TSP] touchscreen already off\n");
		}
		break;
	default:
		break;
	}
}

static struct drm_bl_notifier bl_notifier = {
	.set_power = melfas_set_power
};

static int tsp_register_fb(struct mms_ts_info *info)
{
	bl_notifier.dev = info->input_dev->dev;
	bl_notifier.priv = (void *)info;

	return drm_bl_register(&bl_notifier.dev, BL_TSP_CLASS);
}

static void tsp_unregister_fb(struct mms_ts_info *info)
{
	drm_bl_unregister(&bl_notifier.dev);
}
#else
static int
melfas_fb_notifier_callback(struct notifier_block *self,
			    unsigned long event, void *fb_evdata)
{
	struct mms_ts_info *info;
	struct fb_event *evdata = fb_evdata;
	int blank;
	int i;

	/* If we aren't interested in this event, skip it immediately ... */
	if (event != FB_EVENT_BLANK)
		return 0;

	info = container_of(self, struct mms_ts_info, fb_notif);
	blank = *(int *)evdata->data;

	switch (blank) {
	case FB_BLANK_UNBLANK:
		if (info->enabled == 0) {
			info->pdata->power(true);
			msleep(20);
            pr_info("[TSP/touch] delay was here\n");
			enable_irq(info->client->irq);
			info->enabled = 1;
		} else {
			pr_err("[TSP] touchscreen already on\n");
		}
		break;
	case FB_BLANK_POWERDOWN:
		for (i = 0; i < MAX_FINGERS; i++) {
			input_mt_slot(info->input_dev, i);
			input_mt_report_slot_state(info->input_dev,
						MT_TOOL_FINGER, false);
		input_sync(info->input_dev);
		}
		if (info->enabled == 1) {
			disable_irq(info->client->irq);
			info->pdata->power(false);
			info->enabled = 0;
		} else {
			pr_err("[TSP] touchscreen already off\n");
		}
		break;
	default:
		break;
	}
	return 0;
}

static int tsp_register_fb(struct mms_ts_info *info)
{
	memset(&info->fb_notif, 0, sizeof(info->fb_notif));
	info->fb_notif.notifier_call = melfas_fb_notifier_callback;
	return fb_register_client(&info->fb_notif);
}
static void tsp_unregister_fb(struct mms_ts_info *info)
{
	fb_unregister_client(&info->fb_notif);
}
#endif
#endif

static inline void mms_pwr_on_reset(struct mms_ts_info *info)
{
	struct i2c_adapter *adapter = to_i2c_adapter(info->client->dev.parent);

	if (!info->pdata->mux_fw_flash) {
		dev_info(&info->client->dev,
			 "missing platform data, can't do power-on-reset\n");
		return;
	}

	i2c_lock_adapter(adapter);
	info->pdata->mux_fw_flash(true);

	info->pdata->power(0);
	gpio_direction_output(info->pdata->gpio_sda, 0);
	gpio_direction_output(info->pdata->gpio_scl, 0);
	gpio_direction_output(info->pdata->gpio_int, 0);
	msleep(50);
	info->pdata->power(1);
	msleep(200);

	info->pdata->mux_fw_flash(false);
	i2c_unlock_adapter(adapter);
    
    pr_info("[TSP/touch] mms_pwr_on\n");

	/* TODO: Seems long enough for the firmware to boot.
	 * Find the right value */
	msleep(250);
}
static void release_all_fingers(struct mms_ts_info *info)
{
	struct i2c_client *client = info->client;
	int i;

	dev_dbg(&client->dev, "[TSP] %s\n", __func__);

	for (i = 0; i < MAX_FINGERS; i++) {
#ifdef SEC_TSP_DEBUG
		if (info->finger_state[i] == 1)
			dev_notice(&client->dev, "finger %d up(force)\n", i);
#endif
		info->finger_state[i] = 0;
		input_mt_slot(info->input_dev, i);
		input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER,
					   false);
	}
	
	input_sync(info->input_dev);
	touchbooster_dvfs_lock_on(2, 0);
#if TOUCH_BOOSTER
	set_dvfs_lock(info, 2);
	pr_info("[TSP] dvfs_lock free.\n ");
#endif
}

static void mms_set_noise_mode(struct mms_ts_info *info)
{
	struct i2c_client *client = info->client;

	if (!(info->noise_mode && info->enabled))
		return;
	dev_notice(&client->dev, "%s\n", __func__);

	if (info->ta_status) {
		dev_notice(&client->dev, "noise_mode & TA connect!!!\n");
		i2c_smbus_write_byte_data(info->client, 0x30, 0x1);
	} else {
		dev_notice(&client->dev, "noise_mode & TA disconnect!!!\n");
		i2c_smbus_write_byte_data(info->client, 0x30, 0x2);
		info->noise_mode = 0;
	}
}

static void reset_mms_ts(struct mms_ts_info *info)
{
	struct i2c_client *client = info->client;

	if (info->enabled == false)
		return;

	dev_notice(&client->dev, "%s++\n", __func__);
	disable_irq_nosync(info->irq);
	info->enabled = false;
	touch_is_pressed = 0;

	release_all_fingers(info);

	mms_pwr_on_reset(info);

	info->enabled = true;
	if (info->fw_ic_ver < 0x18) {
		if (info->ta_status) {
			dev_notice(&client->dev, "TA connect!!!\n");
			i2c_smbus_write_byte_data(info->client, 0x33, 0x1);
		} else {
			dev_notice(&client->dev, "TA disconnect!!!\n");
			i2c_smbus_write_byte_data(info->client, 0x33, 0x2);
		}
	}
	mms_set_noise_mode(info);

	enable_irq(info->irq);

	dev_notice(&client->dev, "%s--\n", __func__);
}

static void melfas_ta_cb(struct tsp_callbacks *cb, bool ta_status)
{
	struct mms_ts_info *info =
			container_of(cb, struct mms_ts_info, callbacks);
	struct i2c_client *client = info->client;

	dev_notice(&client->dev, "%s\n", __func__);

	info->ta_status = ta_status;

	if (info->enabled) {
		if (info->ta_status) {
			dev_notice(&client->dev, "TA connect!!!\n");
			i2c_smbus_write_byte_data(info->client, 0x33, 0x1);
		} else {
			dev_notice(&client->dev, "TA disconnect!!!\n");
			i2c_smbus_write_byte_data(info->client, 0x33, 0x2);
		}
		mms_set_noise_mode(info);
	}

/*	if (!ta_status)
		mms_set_noise_mode(info);
*/
}

#ifdef CONFIG_LCD_FREQ_SWITCH
static void melfas_lcd_cb(struct tsp_lcd_callbacks *cb, bool en)
{
	struct mms_ts_info *info =
			container_of(cb, struct mms_ts_info, lcd_callback);

	if (info->enabled == false) {
		dev_err(&info->client->dev,
			"[TSP] do not excute %s.(touch off)\n", __func__);
		return;
	}

	if (info->fw_ic_ver < 0x21) {
		dev_err(&info->client->dev,
			"[TSP] Do not support firmware LCD framerate changing(ver = 0x%x)\n",
			info->fw_ic_ver);
		return;
	}

	if (en) {
		if (info->tsp_lcdfreq_flag == 0) {
			info->tsp_lcdfreq_flag = 1;
			dev_info(&info->client->dev, "[TSP] LCD framerate to 40 Hz\n");
			i2c_smbus_write_byte_data(info->client, 0x34, 0x1);
		}
	} else {
		if (info->tsp_lcdfreq_flag == 1) {
			info->tsp_lcdfreq_flag = 0;
			dev_info(&info->client->dev, "[TSP] LCD framreate to 60 Hz\n");
			i2c_smbus_write_byte_data(info->client, 0x34, 0x1);
		}
	}
}
#endif

static irqreturn_t mms_ts_interrupt(int irq, void *dev_id)
{
	struct mms_ts_info *info = dev_id;
	struct i2c_client *client = info->client;
	int ret;
	int i;
	int sz;
	u8 buf[MAX_FINGERS * EVENT_SZ_PALM] = { 0 };
	int event_sz;
	u8 reg = MMS_INPUT_EVENT0;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = &reg,
			.len = 1,
		}, {
			.addr = client->addr,
			.flags = I2C_M_RD,
			.buf = buf,
		},
	};
#ifdef CONFIG_TOUCHSCREEN_GESTURES
	int gesture_no, finger_no;
	int finger_pos;
	struct gesture_point *point;
	int step;
	bool fingers_completed;
	unsigned long flags;
	bool track_gestures;
	int flg_call_gesturebooster = 0;
	bool flg_gestures_only = false;
    unsigned int gesturebooster_freq_override = 0;
    bool flg_call_touchwake_droplock = false;
	
	track_gestures = info->enabled;
	if (sttg_gestures_only) {
		flg_gestures_only = true;
	}
    
    if (flg_swipe_in_progress) {
        flg_gestures_only = true;
    }
    
    if (touchwake_enabled && flg_touchwake_active && sttg_touchwake_ignoregestures) {
        track_gestures = false;
    }
#endif
	if (get_epen_status()) {
		//pr_info("[TSP] screen touched but potential gesture ignored because of epen input\n");
		track_gestures = false;
	}

	if (info->panel == 'M')
		event_sz = EVENT_SZ_PALM;
	else
		event_sz = EVENT_SZ_OLD;

	sz = i2c_smbus_read_byte_data(client, MMS_INPUT_EVENT_PKT_SZ);
	if (sz < 0) {
		dev_err(&client->dev, "%s bytes=%d\n", __func__, sz);
		for (i = 0; i < 50; i++) {
			sz = i2c_smbus_read_byte_data(client,
						      MMS_INPUT_EVENT_PKT_SZ);
			if (sz > 0)
				break;
		}

		if (i == 50) {
			dev_dbg(&client->dev, "i2c failed... reset!!\n");
			reset_mms_ts(info);
			goto out;
		}
	}
    
    if (prox_near && !flg_screen_on) {
        // if the proximity sensor has been activated and the screen is off, skip input.
        pr_info("[TSP/touch] ignoring touch event because prox_near (%d) and screen is off (%d)\n", prox_near, flg_screen_on);
        goto out;
    }

	if (sz == 0)
		goto out;

	if (sz > MAX_FINGERS * event_sz) {
		dev_err(&client->dev, "[TSP] abnormal data inputed.\n");
		goto out;
	}

	msg[1].len = sz;
	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret != ARRAY_SIZE(msg)) {
		dev_err(&client->dev,
			"failed to read %d bytes of touch data (%d)\n",
			sz, ret);
		if (ret < 0) {
			dev_err(&client->dev,
				"%s bytes=%d\n", __func__, sz);
			for (i = 0; i < 5; i++) {
				msleep(20);
				ret = i2c_transfer(client->adapter,
					msg, ARRAY_SIZE(msg));
				if (ret > 0)
					break;
			}
			if (i == 5) {
				dev_dbg(&client->dev,
					"[TSP] i2c failed E2... reset!!\n");
				reset_mms_ts(info);
				goto out;
			}
		}
	}
#if defined(VERBOSE_DEBUG)
	print_hex_dump(KERN_DEBUG, "mms_ts raw: ",
		       DUMP_PREFIX_OFFSET, 32, 1, buf, sz, false);

#endif
	if (buf[0] == 0x0F) {	/* ESD */
		dev_dbg(&client->dev, "ESD DETECT.... reset!!\n");
		reset_mms_ts(info);
		goto out;
	}

	if (buf[0] == 0x0E) { /* NOISE MODE */
		//dev_dbg(&client->dev, "[TSP] noise mode enter!!\n");
		//info->noise_mode = 1;
		//mms_set_noise_mode(info);
		goto out;
	}

	touch_is_pressed = 0;
	for (i = 0; i < sz; i += event_sz) {
		u8 *tmp = &buf[i];
		int id = (tmp[0] & 0xf) - 1;
		int x = tmp[2] | ((tmp[1] & 0xf) << 8);
		int y = tmp[3] | ((tmp[1] >> 4) << 8);
		int angle = 0;
		int palm = 0;
		if (info->panel == 'M') {
			angle =
				(tmp[5] >= 127) ? (-(256 - tmp[5])) : tmp[5];
			palm = (tmp[0] & 0x10) >> 4;
		}
		if (info->invert_x) {
			x = info->max_x - x;
			if (x < 0)
				x = 0;
		}
		if (info->invert_y) {
			y = info->max_y - y;
			if (y < 0)
				y = 0;
		}
		if (palm) {
			if (info->palm_flag == 3) {
				info->palm_flag = 1;
			} else {
				info->palm_flag = 3;
				palm = 3;
			}
		} else {
			if (info->palm_flag == 2) {
				info->palm_flag = 0;
			} else {
				info->palm_flag = 2;
				palm = 2;
			}
		}
		if (id >= MAX_FINGERS) {
			dev_notice(&client->dev,
				"finger id error [%d]\n", id);
			reset_mms_ts(info);
			goto out;
		}

		if ((tmp[0] & 0x80) == 0) {
			input_mt_slot(info->input_dev, id);
			input_mt_report_slot_state(info->input_dev,
						   MT_TOOL_FINGER, false);
#ifdef CONFIG_SAMSUNG_PRODUCT_SHIP
			if (info->panel == 'M') {
				if (info->finger_state[id] != 0) {
#ifdef CONFIG_TOUCHSCREEN_GESTURES
					if (track_gestures && !ignore_gestures) {
						// When a finger is released and its movement was not completed yet, reset it
						spin_lock_irqsave(&gestures_lock, flags);
						for (gesture_no = 0; gesture_no <= max_configured_gesture; gesture_no++) {
							if (gestures_detected[gesture_no])
								// Ignore gestures already reported
								continue;
							
							for (finger_no = 0; finger_no <= max_gesture_finger[gesture_no]; finger_no++) {
								if (gesture_fingers[gesture_no][finger_no].finger_order == id) {
									// Found a match for ongoing movement
									// Reset the finger progress if path not completed
									if (gesture_fingers[gesture_no][finger_no].current_step <
										gestures_step_count[gesture_no][finger_no]) {
										gesture_fingers[gesture_no][finger_no].finger_order = -1;
										gesture_fingers[gesture_no][finger_no].current_step = -1;
									}
									break;
								}
							}
						}
						spin_unlock_irqrestore(&gestures_lock, flags);
					}
#endif
					info->finger_state[id] = 0;

					// report state to cypress-touchkey for backlight timeout
					touchscreen_state_report(0);
#ifdef CONFIG_LCD_FREQ_SWITCH
					//dev_notice(&client->dev,
					//	"R(%c)(%d) [%2d]", info->ldi,
					//(info->tsp_lcdfreq_flag ? 40 : 60),
					//	id);
#else
					//dev_notice(&client->dev,
					//	"R(%c) [%2d]", info->ldi, id);
#endif
				}
			} else {
				if (info->finger_state[id] != 0) {
					info->finger_state[id] = 0;
					
#ifdef CONFIG_TOUCHSCREEN_GESTURES
					if (track_gestures && !ignore_gestures) {
						// When a finger is released and its movement was not completed yet, reset it
						spin_lock_irqsave(&gestures_lock, flags);
						for (gesture_no = 0; gesture_no <= max_configured_gesture; gesture_no++) {
							if (gestures_detected[gesture_no])
								// Ignore gestures already reported
								continue;
							
							for (finger_no = 0; finger_no <= max_gesture_finger[gesture_no]; finger_no++) {
								if (gesture_fingers[gesture_no][finger_no].finger_order == id) {
									// Found a match for ongoing movement
									// Reset the finger progress if path not completed
									if (gesture_fingers[gesture_no][finger_no].current_step <
										gestures_step_count[gesture_no][finger_no]) {
										gesture_fingers[gesture_no][finger_no].finger_order = -1;
										gesture_fingers[gesture_no][finger_no].current_step = -1;
									}
									break;
								}
							}
						}
						spin_unlock_irqrestore(&gestures_lock, flags);
					}
#endif

					// report state to cypress-touchkey for backlight timeout
					touchscreen_state_report(0);
					
#ifdef CONFIG_LCD_FREQ_SWITCH
					dev_notice(&client->dev,
							   "R(%d) [%2d]",
							   (info->tsp_lcdfreq_flag ? 40 : 60),
							   id);
#else
					dev_notice(&client->dev,
							   "R [%2d]", id);
#endif
				}
			}
#else
			if (info->panel == 'M') {
				if (info->finger_state[id] != 0) {
					info->finger_state[id] = 0;
					
#ifdef CONFIG_TOUCHSCREEN_GESTURES
					if (track_gestures && !ignore_gestures) {
						// When a finger is released and its movement was not completed yet, reset it
						spin_lock_irqsave(&gestures_lock, flags);
						for (gesture_no = 0; gesture_no <= max_configured_gesture; gesture_no++) {
							if (gestures_detected[gesture_no])
								// Ignore gestures already reported
								continue;
							
							for (finger_no = 0; finger_no <= max_gesture_finger[gesture_no]; finger_no++) {
								if (gesture_fingers[gesture_no][finger_no].finger_order == id) {
									// Found a match for ongoing movement
									// Reset the finger progress if path not completed
									if (gesture_fingers[gesture_no][finger_no].current_step <
										gestures_step_count[gesture_no][finger_no]) {
										gesture_fingers[gesture_no][finger_no].finger_order = -1;
										gesture_fingers[gesture_no][finger_no].current_step = -1;
									}
									break;
								}
							}
						}
						spin_unlock_irqrestore(&gestures_lock, flags);
					}
#endif

					// report state to cypress-touchkey for backlight timeout
					touchscreen_state_report(0);

#ifdef CONFIG_LCD_FREQ_SWITCH
					//dev_notice(&client->dev,
					//	"R(%c)(%d) [%2d],([%4d],[%3d])",
					//	info->ldi,
					//(info->tsp_lcdfreq_flag ? 40 : 60),
					//	id, x, y);
#else
					//dev_notice(&client->dev,
					//	"R(%c) [%2d],([%4d],[%3d])",
					//	info->ldi, id, x, y);
#endif
				}
			} else {
				if (info->finger_state[id] != 0) {
					info->finger_state[id] = 0;
					
#ifdef CONFIG_TOUCHSCREEN_GESTURES
					if (track_gestures && !ignore_gestures) {
						// When a finger is released and its movement was not completed yet, reset it
						spin_lock_irqsave(&gestures_lock, flags);
						for (gesture_no = 0; gesture_no <= max_configured_gesture; gesture_no++) {
							if (gestures_detected[gesture_no])
								// Ignore gestures already reported
								continue;
							
							for (finger_no = 0; finger_no <= max_gesture_finger[gesture_no]; finger_no++) {
								if (gesture_fingers[gesture_no][finger_no].finger_order == id) {
									// Found a match for ongoing movement
									// Reset the finger progress if path not completed
									if (gesture_fingers[gesture_no][finger_no].current_step <
										gestures_step_count[gesture_no][finger_no]) {
										gesture_fingers[gesture_no][finger_no].finger_order = -1;
										gesture_fingers[gesture_no][finger_no].current_step = -1;
									}
									break;
								}
							}
						}
						spin_unlock_irqrestore(&gestures_lock, flags);
					}
#endif

					// report state to cypress-touchkey for backlight timeout
					touchscreen_state_report(0);

					dev_notice(&client->dev,
						"R [%2d],([%4d],[%3d]),S:%d W:%d",
						id, x, y, tmp[4], tmp[5]);
				}
			}
#endif
#ifdef CONFIG_TOUCH_WAKE
			// slide2wake trigger
			if (id == 0
                && touchwake_enabled
                && flg_touchwake_active
                && flg_touchwake_slide2wake
                && wake_start == id
                && flg_touchwake_swipe_started
                && x > x_hi
                //&& tmp[4] <= sttg_touchwake_swipe_max_pressure
                && tmp[6] <= sttg_touchwake_swipe_max_pressure
                //&& abs(angle) <= sttg_touchwake_swipe_finger_angle_tolerance
                && flg_touchwake_swipe_only
                && !flg_touchkey_pressed
                && !flg_touchkey_was_pressed) {
                
                gesturebooster_dvfs_lock_on(1600000);
                
                bus_dvfs_lock_on(0);
                pr_info("[TSP] dvfs_lock buslock started...\n ");
                
                pr_info("[TSP touch] slide2wake up at: %4d\n", x);
				flg_call_gesturebooster = 1;
                touch_press();
                gesturebooster_freq_override = 1200000;
                track_gestures = false;
			} else if (id == 0) {
                // debug code. this will run once on swipe failure, to let us know which parameter was violated.
                if (flg_touchwake_slide2wake) {
                    if (id != 0) {
                        pr_info("[TSP/touch] slide2wake cancelled because id was %d (not 0).\n", id);
                    }
                    //if (tmp[4] > sttg_touchwake_swipe_max_pressure) {
                    //    pr_info("[TSP/touch] slide2wake cancelled because pressure (width) was %d (not <= %d).\n", tmp[4], sttg_touchwake_swipe_max_pressure);
                    //}
                    if (tmp[6] > sttg_touchwake_swipe_max_pressure) {
                        pr_info("[TSP/touch] slide2wake cancelled because pressure (touch) was %d (not <= %d).\n", tmp[6], sttg_touchwake_swipe_max_pressure);
                    }
                    if (abs(angle) > sttg_touchwake_swipe_finger_angle_tolerance) {
                        pr_info("[TSP/touch] slide2wake cancelled because angle was %d (not <= %d).\n", angle, sttg_touchwake_swipe_finger_angle_tolerance);
                    }
                }
                flg_touchwake_slide2wake = false;
            }
            
            if (id == 0) {
                // also reset slide2wake detection flag.
                //printk("[TSP/touchwake] resetting all on finger 0\n");
                wake_start = -1;
                flg_touchwake_slide2wake = true;
                touchwake_longpressoff_time_start_secs = -1;
                flg_touchwake_swipe_started = false;
                flg_touchwake_arc_started = false;
                
                flg_swipe_in_progress = false;
                
                // finger is up or broken, do not press power.
                cancel_delayed_work_sync(&touchwake_check_longpressoff_work);
            }
#endif
			if (sttg_typingbooster_mincores
				&& (sttg_typingbooster_mincores > 1 || sttg_typingbooster_upthreshold)
				&& y > 680
				&& flg_screen_on) {
				
				delta_between_press = get_time_ms() - time_last_pressed;
				time_last_pressed = get_time_ms();
				
				if (delta_between_press > sttg_typingbooster_taptimeoutms) {
					//pr_info("[tsp/typingbooster] tapcounter: reset\n");
					ctr_typingbooster = 0;
					delta_between_press = 0;
				}
				
				if (delta_between_press < sttg_typingbooster_maxmsbetweentaps) {
					ctr_typingbooster++;
					//pr_info("[tsp/typingbooster] tapcounter: %d\n", ctr_typingbooster);
				}
				
				if (ctr_typingbooster >= sttg_typingbooster_mintaps) {
					flg_ctr_typingbooster_cycles = sttg_typingbooster_cycles;
					//pr_info("[tsp/typingbooster] triggered! cycle counter reset to %d\n", sttg_typingbooster_cycles);
					ctr_typingbooster--;
				}
			}
			continue;
		}
#ifdef CONFIG_TOUCH_WAKE
		// slide2wake gesture start
		if (id == 0
            && touchwake_enabled
            && flg_touchwake_active
            && flg_touchwake_slide2wake
            //&& (!flg_touchwake_swipe_started || (flg_touchwake_swipe_started && tmp[4] <= sttg_touchwake_swipe_max_pressure))
            && (!flg_touchwake_swipe_started || (flg_touchwake_swipe_started && tmp[6] <= sttg_touchwake_swipe_max_pressure))
            //&& (!flg_touchwake_swipe_started || (flg_touchwake_swipe_started && abs(angle) <= sttg_touchwake_swipe_finger_angle_tolerance))
            ) {
            
            if (wake_start == -1) {
                swipe_x_start = x;
                swipe_y_start = y;
                wake_start = id;
                flg_touchwake_arc_started = false;
                flg_touchwake_swipe_started = false;
                //pr_info("[TSP touch] slide2wake wake_start init at: %4d, %4d\n", x, y);
            }
            
            // check for arc end first.
            if (flg_touchwake_arc_started
                && (
                    (sttg_touchwake_swipe_arc_rh && x >= rh_arc_end_x_min && x <= rh_arc_end_x_max
                     && y >= arc_end_y_min && y <= arc_end_y_max)
                    
                    || (sttg_touchwake_swipe_arc_lh && x >= lh_arc_end_x_min && x <= lh_arc_end_x_max
                     && y >= arc_end_y_min && y <= arc_end_y_max)
                    )
                ) {
                
                gesturebooster_dvfs_lock_on(1600000);
                
                bus_dvfs_lock_on(0);
                pr_info("[TSP] dvfs_lock buslock started...\n ");
                
                pr_info("[TSP touch] slide2wake arc crossed at: %4d, %4d\n", x, y);
                touch_press();
                wake_start = -1;
                flg_swipe_in_progress = true;
                
                if (sttg_touchwake_longpressoff_enabled) {
                    // only set the longpressoff flag if the user has requested it.
                    
                    // set the flag.
                    flg_touchwake_longpressoff = true;
                    
                    if (sttg_touchwake_longpressoff_timeout > 0) {
                        // if the timeout is set to 0, assume infinite mode, otherwise schedule delayed work to turn it off
                        
                        schedule_delayed_work(&touchwake_reset_longpressoff_work, msecs_to_jiffies(sttg_touchwake_longpressoff_timeout));
                    }
                }
                
                // stop reporting this finger to prevent accidental input.
                return IRQ_HANDLED;
            }
            
            // if we're at this point, then no arc end was found. so now check for arc start.
            // if that fails, check for swipe start. if that fails, disable slide2wake until
            // finger-up.
            if (!flg_touchwake_swipe_started
                && !flg_touchwake_arc_started
                && swipe_x_start >= arc_start_x_min && swipe_x_start <= arc_start_x_max
                && swipe_y_start >= arc_start_y_min && swipe_y_start <= arc_start_y_max) {
                
                // we're within the boundaries of the arc starting area.
                // there is no drift protection, so just set a flag, as
                // this code doesn't have to be run anymore.
                
                flg_touchwake_arc_started = true;
                
                pr_info("[TSP touch] slide2wake rh arc down at: id: %d, x: %4d, y: %4d, width: %3d, touch: %3d, angle: %d. swipe_y_start: %d.\n", id, x, y, tmp[4], tmp[6], angle, swipe_y_start);
                
            } else if (!flg_touchwake_arc_started
                       && swipe_x_start < x_lo) {
                
                // we aren't within the start boundaries of the arc, but we are within
                // the starting boundaries of the swipe. so this must be a swipe.
                
                //pr_info("[TSP touch] slide2wake down at: id: %d, x: %4d, y: %4d, width: %3d, touch: %3d, angle: %d. swipe_y_start: %d. swipe_y_drift: %d\n", id, x, y, tmp[4], tmp[6], angle, swipe_y_start, (abs(swipe_y_start - y)));
                
                flg_touchwake_swipe_started = true;
                
                if (abs(swipe_y_start - y) > sttg_touchwake_swipe_y_drift_tolerance) {
                    // swipe drifted too far vertically, resetting detection.
                    pr_info("[TSP touch] slide2wake y_drift_tolerance exceeded at id: %d, x: %4d, y: %4d, swipe_y_start: %d (drifted %d), width: %3d, touch: %3d, angle: %d. detection reset.\n", id, x, y, swipe_y_start, abs(swipe_y_start - y), tmp[4], tmp[6], angle);
                    flg_touchwake_slide2wake = false;
                    wake_start = -1;
                }
                
                if (sttg_touchwake_swipe_fast_trigger && x > x_hi) {
                    gesturebooster_dvfs_lock_on(1600000);
                    bus_dvfs_lock_on(0);
                    pr_info("[TSP] dvfs_lock buslock started...\n ");
                    pr_info("[TSP touch] slide2wake crossed at: %4d\n", x);
                    touch_press();
                    wake_start = -1;
                    flg_swipe_in_progress = true;
                    // stop reporting this finger to prevent accidental input.
                    return IRQ_HANDLED;
                }
                
            } else if (!flg_touchwake_arc_started && !flg_touchwake_swipe_started) {
                
                // this finger event is not within the starting boundaries of the arc
                // or the swipe. cancel slide2wake until finger-up.
                    
                flg_touchwake_slide2wake = false;
                wake_start = -1;
            
            }
            
		} else if (touchwake_enabled && flg_touchwake_active && id == 0) {
            // debug code. this will run once on swipe failure, to let us know which parameter was violated.
            if (flg_touchwake_slide2wake) {
                if (id != 0) {
                    pr_info("[TSP/touch] slide2wake cancelled because id was %d (not 0).\n", id);
                }
                //if (tmp[4] > sttg_touchwake_swipe_max_pressure) {
                //    pr_info("[TSP/touch] slide2wake cancelled because pressure (width) was %d (not <= %d).\n", tmp[4], sttg_touchwake_swipe_max_pressure);
                //}
                if (tmp[6] > sttg_touchwake_swipe_max_pressure) {
                    pr_info("[TSP/touch] slide2wake cancelled because pressure (touch) was %d (not <= %d).\n", tmp[6], sttg_touchwake_swipe_max_pressure);
                }
                if (abs(angle) > sttg_touchwake_swipe_finger_angle_tolerance) {
                    pr_info("[TSP/touch] slide2wake cancelled because angle was %d (not <= %d).\n", angle, sttg_touchwake_swipe_finger_angle_tolerance);
                }
            }
            flg_touchwake_slide2wake = false;
        }
        
        // we are out of the touchwake IF block now, the code below runs on every update.
        if (((sttg_longpressoff_alwayson && flg_screen_on) || (touchwake_enabled && !flg_touchwake_active && flg_touchwake_longpressoff)) && touchwake_longpressoff_time_start_secs == -1 && id == 0 && y > 320 && tmp[6] >= sttg_touchwake_longpressoff_min_pressure) {
            // finger is down and longpressoff is starting.
            
            // we need a fairly unique identifier to check against later, otherwise
            // we won't be able to tell if the finger was maybe lifted, cancelled, and back
            // down again within the 5 seconds.
            
            // in n seconds we will schedule work to be done that will check to see if the finger
            // has moved or otherwise been cancelled or broken. if it hasn't, power will be pressed.
            touchwake_longpressoff_time_start_secs = 1;
            pr_info("[TSP/touch]: longpressoff started, blocking press and scheduling check in %i ms...\n", sttg_touchwake_longpressoff_duration);
            schedule_delayed_work(&touchwake_check_longpressoff_work, msecs_to_jiffies(sttg_touchwake_longpressoff_duration));
            
            // in case we got here from sttg_longpressoff_alwayson, the longpressoff flag might not be set,
            // so set it again just to be sure.
            flg_touchwake_longpressoff = true;
            
            swipe_x_start = x;
            swipe_y_start = y;
            
            // lift this finger to prevent accidental input.
            input_mt_slot(info->input_dev, 0);
            input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, false);
            input_sync(info->input_dev);
            
            // stop reporting this finger to prevent accidental input.
            return IRQ_HANDLED;
            
        } else if (swipe_x_start < 9999 && ((sttg_longpressoff_alwayson && flg_screen_on) || (touchwake_enabled && !flg_touchwake_active && flg_touchwake_longpressoff)) && id == 0 && y > 640 && abs(x - swipe_x_start) <= sttg_touchwake_longpressoff_xy_drift_tolerance && abs(y - swipe_y_start) <= sttg_touchwake_longpressoff_xy_drift_tolerance && tmp[6] >= sttg_touchwake_longpressoff_min_pressure) {
            
            // even though this is a new event, it is still the same finger-down since swipe_x_start hasn't been set to 9999 yet.
            // there's nothing to do but block this input, and reset it back to finger-up
            
            input_mt_slot(info->input_dev, 0);
            input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, false);
            input_sync(info->input_dev);
            
            //pr_info("[TSP/touch]: longpressoff INTACT. x-drift: %3d (%i), y-drift: %4d (%i), pressure (width): %i (n/a), pressure (touch): %i (%i)\n", abs(x - swipe_x_start), sttg_touchwake_longpressoff_xy_drift_tolerance, abs(y - swipe_y_start), sttg_touchwake_longpressoff_xy_drift_tolerance, tmp[4], tmp[6], sttg_touchwake_longpressoff_min_pressure);
            
            // stop reporting this finger to prevent accidental input.
            return IRQ_HANDLED;
            
        } else if (swipe_x_start < 9999 && ((sttg_longpressoff_alwayson && flg_screen_on) || (touchwake_enabled && !flg_touchwake_active && flg_touchwake_longpressoff)) && id == 0) {
            
            pr_info("[TSP/touch]: longpressoff BROKEN. x-drift: %3d, y-drift: %4d, pressure (width): %i, pressure (touch): %i\n", abs(x - swipe_x_start), abs(y - swipe_y_start), tmp[4], tmp[6]);
            
            // block this last input, as it was still part of the initial longpressoff finger-down.
            swipe_x_start = 9999;
            
            // finger is up or broken, do not press power.
            cancel_delayed_work_sync(&touchwake_check_longpressoff_work);
            
            input_mt_slot(info->input_dev, 0);
            input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, false);
            input_sync(info->input_dev);
            
            // stop reporting this finger to prevent accidental input.
            return IRQ_HANDLED;
        }
        
        // longtap off.
        
#endif

		if (info->panel == 'M') {
#ifdef CONFIG_TOUCHSCREEN_GESTURES
			if (track_gestures && !ignore_gestures) {
				// Finger being moved, check the gesture steps progress
				spin_lock_irqsave(&gestures_lock, flags);
				for (gesture_no = 0; gesture_no <= max_configured_gesture; gesture_no++) {
					if (gestures_detected[gesture_no])
						// Ignore further movement for gestures already reported
						continue;
					
					// check to see if this gesture requires a touchkey, or if they all do.
                    // and skip gesture 0 because it is the power gesture and needs to be easier to trigger.
					if ((ary_gestures_flg_touchkey[gesture_no] || sttg_require_touchkey) && gesture_no != 0) {
                        // if we're here, we need a touckey to be down.
                        if (sttg_gesture_delay == 0 || !flg_gestures_only) {
                            // this gesture requires a touchkey to be held down but one isn't,
                            // or the gesture_delay isn't set. either way, let's skip it.
                            //printk("[TSP] this gesture (%d) requires touchkey!\n", gesture_no + 1);
                            continue;
                        }
					}
					
					// Find which finger definition this touch maps to
					finger_pos = -1;
					for (finger_no = 0; finger_no <= max_gesture_finger[gesture_no]; finger_no++) {
						if (gesture_fingers[gesture_no][finger_no].finger_order == id) {
							// Found a match for ongoing movement
							finger_pos = finger_no;
							break;
						}
					}
					if (finger_pos < 0) {
						// This finger is not yet tracked, check the first zone it matches
						for (finger_no = 0; finger_no <= max_gesture_finger[gesture_no]; finger_no++) {
							if (gestures_step_count[gesture_no][finger_no] < 1) {
								// This finger definition has no steps, no more to check
								break;
							} else {
								point = &gesture_points[gesture_no][finger_no][0];
								if (gesture_fingers[gesture_no][finger_no].finger_order < 0 &&
									x >= point->min_x &&
									x <= point->max_x &&
									y >= point->min_y &&
									y <= point->max_y) {
									// This finger definition is still pending
									// and this touch matches the area
									finger_pos = finger_no;
									gesture_fingers[gesture_no][finger_pos].finger_order = id;
									gesture_fingers[gesture_no][finger_pos].current_step = 1;
									////printk("[TSP] Gesture %d, finger %d - Associated index %d\n",
									////	   gesture_no, finger_pos, id);
									break;
								}
							}
						}
					}
					if (finger_pos >= 0) {
						// Track next zones where the finger should move
						for (step = gesture_fingers[gesture_no][finger_pos].current_step;
							 step < gestures_step_count[gesture_no][finger_pos];
							 step++) {
							point = &gesture_points[gesture_no][finger_pos][step];
							if (x >= point->min_x &&
								x <= point->max_x &&
								y >= point->min_y &&
								y <= point->max_y) {
								// Next zone reached, keep testing
								////printk("[TSP] Gesture %d, finger %d - Associated index %d - Moved through step, next is %d\n",
								////	   gesture_no, finger_pos, id, step+1);
								gesture_fingers[gesture_no][finger_pos].current_step++;
							} else {
								break;
							}
						}
					}
				}
				spin_unlock_irqrestore(&gestures_lock, flags);
			}
#endif
			if (!flg_gestures_only) {
                
                //pr_info("[TSP/touch] x: %d, y: %d, fingerwidth: %d, touchpointwidth: %d, minor: %d, angle: %d, palm: %d\n", x, y, tmp[4], tmp[6], tmp[7], angle, palm);
                
                input_mt_slot(info->input_dev, id);
                input_mt_report_slot_state(info->input_dev,
                                           MT_TOOL_FINGER, true);
                input_report_abs(info->input_dev,
                                 ABS_MT_POSITION_X, x);
                input_report_abs(info->input_dev,
                                 ABS_MT_POSITION_Y, y);
                input_report_abs(info->input_dev,
                                 ABS_MT_WIDTH_MAJOR, tmp[4]);
                input_report_abs(info->input_dev,
                                 ABS_MT_TOUCH_MAJOR, tmp[6]);
                input_report_abs(info->input_dev,
                                 ABS_MT_TOUCH_MINOR, tmp[7]);
                input_report_abs(info->input_dev,
                                 ABS_MT_ANGLE, angle);
                input_report_abs(info->input_dev,
                                 ABS_MT_PALM, palm);
				
			} else {
				
				pr_info("[TSP/touch] IGNORED! x: %d, y: %d, fingerwidth: %d, touchpointwidth: %d, minor: %d, angle: %d, palm: %d\n", x, y, tmp[4], tmp[6], tmp[7], angle, palm);
				
			}
#ifdef CONFIG_SAMSUNG_PRODUCT_SHIP
			if (info->finger_state[id] == 0) {
				info->finger_state[id] = 1;
				// report state to cypress-touchkey for backlight timeout
				touchscreen_state_report(1);
#ifdef CONFIG_LCD_FREQ_SWITCH
				//dev_notice(&client->dev,
				//	"P(%c)(%d) [%2d]", info->ldi,
				//	(info->tsp_lcdfreq_flag ? 40 : 60), id);
#else
				//dev_notice(&client->dev,
				//	"P(%c) [%2d]", info->ldi, id);
#endif
			}
#else
			if (info->finger_state[id] == 0) {
				info->finger_state[id] = 1;
				// report state to cypress-touchkey for backlight timeout
				touchscreen_state_report(1);
#ifdef CONFIG_LCD_FREQ_SWITCH
				dev_notice(&client->dev,
					"P(%c)(%d) [%2d],([%4d],[%3d]) w=%d, major=%d, minor=%d, angle=%d, palm=%d",
					info->ldi,
					(info->tsp_lcdfreq_flag ? 40 : 60),
					id, x, y, tmp[4], tmp[6],
					tmp[7], angle, palm);
#else
				dev_notice(&client->dev,
					"P(%c) [%2d],([%4d],[%3d]) w=%d, major=%d, minor=%d, angle=%d, palm=%d",
					info->ldi, id, x, y, tmp[4], tmp[6],
					tmp[7], angle, palm);
#endif
			}
#endif
		} else {
			
#ifdef CONFIG_TOUCHSCREEN_GESTURES
			if (track_gestures && !ignore_gestures) {
				// Finger being moved, check the gesture steps progress
				spin_lock_irqsave(&gestures_lock, flags);
				for (gesture_no = 0; gesture_no <= max_configured_gesture; gesture_no++) {
					if (gestures_detected[gesture_no])
						// Ignore further movement for gestures already reported
						continue;
					
					// check to see if this gesture requires a touchkey, or if they all do.
                    // and skip gesture 0 because it is the power gesture and needs to be easier to trigger.
					if ((ary_gestures_flg_touchkey[gesture_no] || sttg_require_touchkey) && gesture_no != 0) {
                        // if we're here, we need a touckey to be down.
                        if (sttg_gesture_delay == 0 || !flg_gestures_only) {
                            // this gesture requires a touchkey to be held down but one isn't,
                            // or the gesture_delay isn't set. either way, let's skip it.
                            //printk("[TSP] this gesture (%d) requires touchkey!\n", gesture_no + 1);
                            continue;
                        }
					}
					
					// Find which finger definition this touch maps to
					finger_pos = -1;
					for (finger_no = 0; finger_no <= max_gesture_finger[gesture_no]; finger_no++) {
						if (gesture_fingers[gesture_no][finger_no].finger_order == id) {
							// Found a match for ongoing movement
							finger_pos = finger_no;
							break;
						}
					}
					if (finger_pos < 0) {
						// This finger is not yet tracked, check the first zone it matches
						for (finger_no = 0; finger_no <= max_gesture_finger[gesture_no]; finger_no++) {
							if (gestures_step_count[gesture_no][finger_no] < 1) {
								// This finger definition has no steps, no more to check
								break;
							} else {
								point = &gesture_points[gesture_no][finger_no][0];
								if (gesture_fingers[gesture_no][finger_no].finger_order < 0 &&
									x >= point->min_x &&
									x <= point->max_x &&
									y >= point->min_y &&
									y <= point->max_y) {
									// This finger definition is still pending
									// and this touch matches the area
									finger_pos = finger_no;
									gesture_fingers[gesture_no][finger_pos].finger_order = id;
									gesture_fingers[gesture_no][finger_pos].current_step = 1;
									////printk("[TSP] Gesture %d, finger %d - Associated index %d\n",
									////	   gesture_no, finger_pos, id);
									break;
								}
							}
						}
					}
					if (finger_pos >= 0) {
						// Track next zones where the finger should move
						for (step = gesture_fingers[gesture_no][finger_pos].current_step;
							 step < gestures_step_count[gesture_no][finger_pos];
							 step++) {
							point = &gesture_points[gesture_no][finger_pos][step];
							if (x >= point->min_x &&
								x <= point->max_x &&
								y >= point->min_y &&
								y <= point->max_y) {
								// Next zone reached, keep testing
								////printk("[TSP] Gesture %d, finger %d - Associated index %d - Moved through step, next is %d\n",
								////	   gesture_no, finger_pos, id, step+1);
								gesture_fingers[gesture_no][finger_pos].current_step++;
							} else {
								break;
							}
						}
					}
				}
				spin_unlock_irqrestore(&gestures_lock, flags);
			}
#endif
			
			if (!flg_gestures_only) {
                
                input_mt_slot(info->input_dev, id);
                input_mt_report_slot_state(info->input_dev,
                                           MT_TOOL_FINGER, true);
                input_report_abs(info->input_dev,
                                 ABS_MT_POSITION_X, x);
                input_report_abs(info->input_dev,
                                 ABS_MT_POSITION_Y, y);
                input_report_abs(info->input_dev,
                                 ABS_MT_TOUCH_MAJOR, tmp[4]);
                input_report_abs(info->input_dev,
                                 ABS_MT_PRESSURE, tmp[5]);
				
			} else {
				
				pr_info("[TSP/touch] IGNORED! x: %d, y: %d\n", x, y);
				
			}
#ifdef CONFIG_SAMSUNG_PRODUCT_SHIP
			if (info->finger_state[id] == 0) {
				info->finger_state[id] = 1;
				// report state to cypress-touchkey for backlight timeout
				touchscreen_state_report(1);
				dev_notice(&client->dev,
					"P [%2d]", id);
			}
#else
			if (info->finger_state[id] == 0) {
				info->finger_state[id] = 1;
				dev_notice(&client->dev,
					"P [%2d],([%4d],[%3d]),S:%d W:%d",
					id, x, y, tmp[4], tmp[5]);
			}
#endif
		}
		touch_is_pressed++;
		
#ifdef CONFIG_TOUCH_WAKE
        if (id == 0
            && touchwake_enabled
            && flg_touchwake_active
            && !flg_touchwake_swipe_only
            && !flg_touchkey_pressed
            && !flg_touchkey_was_pressed) {
            flg_call_gesturebooster = 1;
            touch_press();
            flg_touchwake_active = false;
            gesturebooster_freq_override = 1600000;
            track_gestures = false;
            pr_info("TSP/touchwake] single tap detected.\n");
            // stop reporting this finger to prevent accidental input.
            return IRQ_HANDLED;
        }
#endif
	}
	
	input_sync(info->input_dev);
	
#ifdef CONFIG_TOUCHSCREEN_GESTURES
	if (track_gestures && !ignore_gestures) {
		// Check completed gestures or reset all progress if all fingers released
		spin_lock_irqsave(&gestures_lock, flags);
		for (gesture_no = 0; gesture_no <= max_configured_gesture; gesture_no++) {
            
			if (gestures_detected[gesture_no]) {
				// Gesture already reported, skip
                //printk("[TSP/gestures] gesture %d already reported.\n", gesture_no);
                
                // but before we skip, check to see if gesture 0 (power) needs to be processed again.
                if (!touch_is_pressed) {
                    // below are gestures that need processing after finger-up.
                    
                    if (gestures_detected[0]) {
                        // post-processing for gesture #0 (power).
                        //printk("[TSP/gestures] post-processing gesture %d...\n", gesture_no);
                        
                        if (ary_gestures_flg_touchkey[gesture_no] && (flg_touchkey_pressed || flg_touchkey_was_pressed || flg_gestures_only)) {
                            // if this gesture requires a touchkey, AND either a.) a touchkey is pressed, or b.) a touchkey was pressed at some
                            // point while this finger was down, or c.) the gestures only flag is set (which would mean a touchkey was pressed)

                            //printk("[TSP/gestures] trap 2: touchkey is/was down\n");
                            
                            // GESTURE #1
                            // Power off
                            //
                            // special kernel-level processing for the power-off gesture.
                            
                            printk("[TSP/gestures] pressing power key\n");
                            press_power();
                            
                            // reset the flag.
                            flg_touchkey_was_pressed = false;
                        }
                        
                        // regardless of whether or not a touchkey was pressed, this gesture is done. reset it.
                        //printk("[TSP/gestures] trap 2: resetting gesture %d progress\n", gesture_no);
                        gestures_detected[gesture_no] = false;
                        
                        // and reset this gesture's progress on all fingers.
                        reset_gesture_progress(gesture_no);
                    }
                }
                
				continue;
            }
			
			if (gestures_step_count[gesture_no][0] < 1) {
                //printk("[TSP/gestures] gesture %d no configured.\n", gesture_no);
				continue; // Gesture not configured
            }
			
			fingers_completed = true;
			for (finger_no = 0; finger_no <= max_gesture_finger[gesture_no]; finger_no++) {
				if (gestures_step_count[gesture_no][finger_no] > 0 &&
				    gesture_fingers[gesture_no][finger_no].current_step <
					gestures_step_count[gesture_no][finger_no]) {
					
					fingers_completed = false;
					break;
				}
			}
			if (fingers_completed && !flg_gestures_detected) {
				// All finger steps completed for this gesture, wake any consumers
				if (ary_gestures_flg_exclusive[gesture_no]) {
					// this is an exclusive gesture. this means it must be the ONLY
					// matching gesture for it to trigger. meaning, most likely, it
					// is so simple that if we process it immediately it will trip
					// before other, more sophisticated gestures can be drawn.
					// so don't process it until we are sure the user is done, which
					// will be when they take their fingers off.
					if (touch_is_pressed || flg_gestures_detected) {
						// fingers aren't up, so don't wake consumers yet.
						//printk("[TSP] touch_is_pressed so skipping\n");
						continue;
					} else {
						// fingers are up, and no other gestures were triggered,
						// so wake the exclusive gesture's consumers.
						//printk("[TSP] touch_is_NOT_pressed so processing\n");
					}
				}
				printk("[TSP/gestures] gesture %d completed, waking consumers\n", gesture_no + 1);
                
				flg_gestures_detected = 1;
                
                // give gesturebooster its own flag so we don't call it a zillion times.
                flg_call_gesturebooster = 1;
                
                // ff - Oct 7 2013 - some gestures can be handled very quickly natively within the kernel
                //      instead of waiting for a userspace script to process them.
                //
                //      also, in order to make it easier to trigger the modifier key, there are two traps
                //      to detect it a touchkey being pressed. the first is directly below, if that fails
                //      to detect a touchkey pressed event, processing is defered until finger-up, where
                //      it is checked again, in case the user ends up pressing the tk after finger-down.
                //      for now, this method is limited to gestures that are processed within the kernel.
                
                if (gesture_no == 0) {
                    // GESTURE #1
                    // Power off
                    //
                    // special kernel-level processing for the power-off gesture.
                    // also, don't use the gesturebooster for this.
                    
                    // disable gesturebooster
                    flg_call_gesturebooster = 0;
                    
                    // check to see if touchwake is inactive. we don't want the power on gesture triggering unless the screen is on (aka touchwake isn't active).
                    if (!flg_touchwake_active) {
                        
                        // check to see if a touckey has been pressed yet. if not, we will check again on finger-up.
                        if (ary_gestures_flg_touchkey[gesture_no] && (flg_touchkey_pressed || flg_touchkey_was_pressed || flg_gestures_only)) {
                            //printk("[TSP/gestures] trap 1: touchkey is/was down\n");
                            
                            printk("[TSP/gestures] trap 1: pressing power key\n");
                            press_power();
                            
                            // reset the touchkey_was_pressed flag.
                            flg_touchkey_was_pressed = false;
                            
                            // reset this gesture's progress
                            //printk("[TSP/gestures] trap 1: resetting gesture %d progress\n", gesture_no);
                            reset_gesture_progress(gesture_no);
                            
                            // since we just processed this gesture internally, no need to wake the handler script.
                            // just continue to the next gesture.
                            continue;
                            
                        } else if (ary_gestures_flg_touchkey[gesture_no]) {
                            
                            //printk("[TSP/gestures] gesture %d found and it requires a touchkey. scheduling for trap 2...\n", gesture_no);
                            gestures_detected[gesture_no] = true;
                            
                            // since we just processed this gesture internally, no need to wake the handler script.
                            // just continue to the next gesture.
                            continue;
                        }
                        
                    } else {
                        
                        // reset this gesture's progress
                        //printk("[TSP/gestures] trap 1: resetting gesture %d progress\n", gesture_no);
                        reset_gesture_progress(gesture_no);
                        
                        // since we just processed this gesture internally, no need to wake the handler script.
                        // just continue to the next gesture.
                        continue;
                    }
                    
                } else if (gesture_no == 1) {
                    
                    // GESTURE #2
                    // Toggle touchwake persistent mode
                    printk("[TSP/gestures] toggling touchwake persistent mode. was: %d now: %d\n", sttg_touchwake_persistent, !sttg_touchwake_persistent);
                    
                    // disable gesturebooster
                    flg_call_gesturebooster = 0;
                    
                    /* TODO: change next three variables to sttg_touchwake_persistent_wakelock when deepsleep works */
                    sttg_touchwake_persistent = !sttg_touchwake_persistent;
                    if (!sttg_touchwake_persistent) {
                        if (flg_touchwake_active) {
                            flg_call_touchwake_droplock = true;
                        }
                    }
                    
                    // reset the touchkey_was_pressed flag.
                    flg_touchkey_was_pressed = false;
                    
                    // reset this gesture's progress
                    //printk("[TSP/gestures] trap 1: resetting gesture %d progress\n", gesture_no);
                    reset_gesture_progress(gesture_no);
                    
                    // since we just processed this gesture internally, no need to wake the handler script.
                    // just continue to the next gesture.
                    continue;
                }
                
                printk("[TSP/gestures] gesture %d completed\n", gesture_no + 1);
				gestures_detected[gesture_no] = true;
				has_gestures = true;
				wake_up_interruptible_all(&gestures_wq);
                
			} else if (!touch_is_pressed) {
				// All fingers released, reset progress for all paths
                
                if (gestures_detected[0] || gestures_detected[1]) {
                    //printk("[TSP/gestures] gesture %d not resetting touchkey_was_pressed because gesture 0 or 1 is awaiting post-processing\n", gesture_no);
                } else if (gesture_no == max_configured_gesture) {
                    //printk("[TSP/gestures] gesture %d resetting touchkey_was_pressed\n", gesture_no);
                    flg_touchkey_was_pressed = false;
                }
				for (finger_no = 0; finger_no <= max_gesture_finger[gesture_no]; finger_no++) {
					gesture_fingers[gesture_no][finger_no].finger_order = -1;
					gesture_fingers[gesture_no][finger_no].current_step = -1;
				}
			}
		}
		spin_unlock_irqrestore(&gestures_lock, flags);
        
        // move this outside the spinlock so we don't crash.
        if (flg_call_touchwake_droplock) {
            flg_call_touchwake_droplock = false;
            printk("[TSP/gestures] unlocking wakelock...\n");
            touchwake_droplock();
        }
	}
#endif
    
#if GESTURE_BOOSTER
    // ff - Oct 5 2013 - move this outside of above gesture track/ignore if statement
    //      so other things can use the booster regardless of gesture engine status.
    if (flg_call_gesturebooster) {
        if (gesturebooster_enabled) {
            //printk("[TSP] calling gesturebooster\n");
            gesturebooster_dvfs_lock_on(gesturebooster_freq_override);
        }
        
        // gesture is complete, but don't unblock touch events yet,
        // or else the OS will get a few points before the user
        // pulls their finger away completely.
        
        // reset flag.
        flg_call_gesturebooster = 0;
    }
#endif

#if TOUCH_BOOSTER
	set_dvfs_lock(info, !!touch_is_pressed);
#endif
	
	if (sttg_touchbooster_enabled && flg_screen_on) {
		touchbooster_dvfs_lock_on(!!touch_is_pressed, sttg_touchbooster_freq);
	}

#ifdef CONFIG_CPU_FREQ_LCD_FREQ_DFS
	if((!!touch_is_pressed) && flg_enable_lcdfreq_touchboost){
		_lcdfreq_lock(0);
		//pr_info("TSP: lcdfreq_lock(0)\n");
	}
#endif

#ifdef CONFIG_CPU_FREQ_GOV_ONDEMAND_FLEXRATE
	if(!!touch_is_pressed){
		midas_tsp_request_qos();
	}
#endif
	
	if (!touch_is_pressed) {
		//printk("[TSP] resetting flg_gestures_detected\n");
		flg_gestures_detected = 0;
	}
	
	if (!!touch_is_pressed) {
		flg_touch_was_pressed = true;
	}

out:
	return IRQ_HANDLED;
}

int get_tsp_status(void)
{
	return touch_is_pressed;
}
EXPORT_SYMBOL(get_tsp_status);

bool tsp_check_touched_flag(unsigned int mode)
{
	if (mode == 0) {
		// reset flg_touch_was_pressed.
		flg_touch_was_pressed = false;
		return 0;
	} else if (mode == 1) {
		// return flg_touch_was_pressed.
		return flg_touch_was_pressed;
	}
	return 0;
}
EXPORT_SYMBOL(tsp_check_touched_flag);

void tsp_gestures_only(bool mode)
{
	//pr_info("[TSP] setting gestures_only=%d\n", mode);
	sttg_gestures_only = mode;
}
EXPORT_SYMBOL(tsp_gestures_only);


#if ISC_DL_MODE
static int mms100_i2c_read(struct i2c_client *client,
		u16 addr, u16 length, u8 *value)
{
	struct i2c_adapter *adapter = client->adapter;
	struct i2c_msg msg;
	int ret = -1;

	msg.addr = client->addr;
	msg.flags = 0x00;
	msg.len = 1;
	msg.buf = (u8 *) &addr;

	ret = i2c_transfer(adapter, &msg, 1);

	if (ret >= 0) {
		msg.addr = client->addr;
		msg.flags = I2C_M_RD;
		msg.len = length;
		msg.buf = (u8 *) value;

		ret = i2c_transfer(adapter, &msg, 1);
	}

	if (ret < 0)
		pr_err("[TSP] : read error : [%d]", ret);

	return ret;
}


static int mms100_reset(struct mms_ts_info *info)
{
	info->pdata->power(false);
	msleep(30);
	info->pdata->power(true);
	msleep(300);

	return ISC_SUCCESS;
}
/*
static int mms100_check_operating_mode(struct i2c_client *_client,
		const int _error_code)
{
	int ret;
	unsigned char rd_buf = 0x00;
	unsigned char count = 0;

	if(_client == NULL)
		pr_err("[TSP ISC] _client is null");

	ret = mms100_i2c_read(_client, ISC_ADDR_VERSION, 1, &rd_buf);

	if (ret<0) {
		pr_info("[TSP ISC] %s,%d: i2c read fail[%d]\n",
			__func__, __LINE__, ret);
		return _error_code;
	}

	return ISC_SUCCESS;
}
*/
static int mms100_get_version_info(struct i2c_client *_client)
{
	int i, ret;
	unsigned char rd_buf[8];

	/* config version brust read (core, private, public) */
	ret = mms100_i2c_read(_client, ISC_ADDR_VERSION, 3, rd_buf);
	if (ret < 0) {
		pr_info("[TSP ISC] %s,%d: i2c read fail[%d]\n",
			__func__, __LINE__, ret);
		return ISC_I2C_ERROR;
	}

	for (i = 0; i < SECTION_NUM; i++)
		ts_info[i].version = rd_buf[i];

	ts_info[SEC_CORE].compatible_version =
		ts_info[SEC_BOOTLOADER].version;
	ts_info[SEC_CONFIG].compatible_version =
		ts_info[SEC_CORE].version;

	ret = mms100_i2c_read(_client, ISC_ADDR_SECTION_PAGE_INFO, 8, rd_buf);
	if (ret < 0) {
		pr_info("[TSP ISC] %s,%d: i2c read fail[%d]\n",
			__func__, __LINE__, ret);
		return ISC_I2C_ERROR;
	}

	for (i = 0; i < SECTION_NUM; i++) {
		ts_info[i].start_addr = rd_buf[i];
		ts_info[i].end_addr = rd_buf[i + SECTION_NUM + 1];
	}

	for (i = 0; i < SECTION_NUM; i++) {
		pr_info("TS : Section(%d) version: 0x%02X\n",
			i, ts_info[i].version);
		pr_info("TS : Section(%d) Start Address: 0x%02X\n",
			i, ts_info[i].start_addr);
		pr_info("TS : Section(%d) End Address: 0x%02X\n",
			i, ts_info[i].end_addr);
		pr_info("TS : Section(%d) Compatibility: 0x%02X\n",
			i, ts_info[i].compatible_version);
	}

	return ISC_SUCCESS;
}

static int mms100_seek_section_info(void)
{
	int i;
	char str_buf[STRING_BUF_LEN];
	char name_buf[SECTION_NAME_LEN];
	int version;
	int page_num;

	const unsigned char *buf;
	int next_ptr;

	for (i = 0; i < SECTION_NUM; i++) {
		if (fw_mbin[i] == NULL) {
			buf = NULL;
			pr_info("[TSP ISC] fw_mbin[%d]->data is NULL", i);
		} else {
			buf = fw_mbin[i]->data;
		}

		if (buf == NULL) {
			mbin_info[i].version = ts_info[i].version;
			mbin_info[i].compatible_version =
				ts_info[i].compatible_version;
			mbin_info[i].start_addr = ts_info[i].start_addr;
			mbin_info[i].end_addr = ts_info[i].end_addr;
		} else {
			next_ptr = 0;

			do {
				sscanf(buf + next_ptr, "%s", str_buf);
				next_ptr += strlen(str_buf) + 1;
			} while (!strstr(str_buf, "SECTION_NAME"));

			sscanf(buf + next_ptr, "%s%s", str_buf, name_buf);

			if (strncmp(section_name[i], name_buf,
				SECTION_NAME_LEN))
				return ISC_FILE_FORMAT_ERROR;

			do {
				sscanf(buf + next_ptr, "%s", str_buf);
				next_ptr += strlen(str_buf) + 1;
			} while (!strstr(str_buf, "SECTION_VERSION"));

			sscanf(buf + next_ptr, "%s%d", str_buf, &version);
			mbin_info[i].version = ISC_CHAR_2_BCD(version);

			do {
				sscanf(buf + next_ptr, "%s", str_buf);
				next_ptr += strlen(str_buf) + 1;
			} while (!strstr(str_buf, "START_PAGE_ADDR"));

			sscanf(buf + next_ptr, "%s%d", str_buf, &page_num);
			mbin_info[i].start_addr = page_num;

			do {
				sscanf(buf + next_ptr, "%s", str_buf);
				next_ptr += strlen(str_buf) + 1;
			} while (!strstr(str_buf, "END_PAGE_ADDR"));

			sscanf(buf + next_ptr, "%s%d", str_buf, &page_num);
			mbin_info[i].end_addr = page_num;

			do {
				sscanf(buf + next_ptr, "%s", str_buf);
				next_ptr += strlen(str_buf) + 1;
			} while (!strstr(str_buf, "COMPATIBLE_VERSION"));

			sscanf(buf + next_ptr, "%s%d", str_buf, &version);
			mbin_info[i].compatible_version =
				ISC_CHAR_2_BCD(version);

			do {
				sscanf(buf + next_ptr, "%s", str_buf);
				next_ptr += strlen(str_buf) + 1;
			} while (!strstr(str_buf, "[Binary]"));

			if (mbin_info[i].version == 0xFF)
				return ISC_FILE_FORMAT_ERROR;
		}
	}

	for (i = 0; i < SECTION_NUM; i++) {
		pr_info("[TSP ISC] MBin : Section(%d) Version: 0x%02X\n",
			i, mbin_info[i].version);
		pr_info("[TSP ISC] MBin : Section(%d) Start Address: 0x%02X\n",
			i, mbin_info[i].start_addr);
		pr_info("[TSP ISC] MBin : Section(%d) End Address: 0x%02X\n",
			i, mbin_info[i].end_addr);
		pr_info("[TSP ISC] MBin : Section(%d) Compatibility: 0x%02X\n",
			i, mbin_info[i].compatible_version);
	}

	return ISC_SUCCESS;
}

static int mms100_compare_version_info(struct i2c_client *_client)
{
	int i, ret;
	unsigned char expected_compatibility[SECTION_NUM];

	if (mms100_get_version_info(_client) != ISC_SUCCESS)
		return ISC_I2C_ERROR;

	ret = mms100_seek_section_info();

	/* Check update areas , 0 : bootloader 1: core 2: private 3: public */
	for (i = 0; i < SECTION_NUM; i++) {
		if ((mbin_info[i].version == 0) ||
			(mbin_info[i].version != ts_info[i].version)) {
			section_update_flag[i] = true;
			pr_info("[TSP ISC] [%d] section will be updated!", i);
		}
	}
	section_update_flag[0] = false;
	section_update_flag[1] = false;
	pr_info("[TSP ISC] [%d] [%d] [%d]", section_update_flag[0],
		section_update_flag[1], section_update_flag[2]);

	if (section_update_flag[SEC_BOOTLOADER]) {
		expected_compatibility[SEC_CORE] =
		mbin_info[SEC_BOOTLOADER].version;
	} else {
		expected_compatibility[SEC_CORE] =
		ts_info[SEC_BOOTLOADER].version;
	}

	if (section_update_flag[SEC_CORE]) {
		expected_compatibility[SEC_CONFIG] =
		mbin_info[SEC_CORE].version;
	} else {
		expected_compatibility[SEC_CONFIG] =
		ts_info[SEC_CORE].version;
	}

	for (i = SEC_CORE; i < SEC_CONFIG; i++) {
		if (section_update_flag[i]) {
			pr_info("[TSP ISC] section_update_flag(%d), 0x%02x, 0x%02x\n",
				i, expected_compatibility[i],
				mbin_info[i].compatible_version);

			if (expected_compatibility[i] !=
				mbin_info[i].compatible_version)
				return ISC_COMPATIVILITY_ERROR;
		} else {
			pr_info("[TSP ISC] !section_update_flag(%d), 0x%02x, 0x%02x\n",
				i, expected_compatibility[i],
				ts_info[i].compatible_version);
			if (expected_compatibility[i] !=
				ts_info[i].compatible_version)
				return ISC_COMPATIVILITY_ERROR;
		}
	}
	return ISC_SUCCESS;
}

static int mms100_enter_ISC_mode(struct i2c_client *_client)
{
	int ret;
	unsigned char wr_buf[2];

	pr_info("[TSP ISC] %s\n", __func__);

	wr_buf[0] = ISC_CMD_ENTER_ISC;
	wr_buf[1] = ISC_CMD_ENTER_ISC_PARA1;

	ret = i2c_master_send(_client, wr_buf, 2);

	if (ret < 0) {
		pr_info("[TSP ISC] %s,%d: i2c write fail[%d]\n",
			__func__, __LINE__, ret);
		return ISC_I2C_ERROR;
	}

	msleep(50);

	return ISC_SUCCESS;
}

static int mms100_enter_config_update(struct i2c_client *_client)
{
	int ret;
	unsigned char wr_buf[10] = {0,};
	unsigned char rd_buf;

	wr_buf[0] = ISC_CMD_UPDATE_MODE;
	wr_buf[1] = ISC_SUBCMD_ENTER_UPDATE;

	ret = i2c_master_send(_client, wr_buf, 10);

	if (ret < 0) {
		pr_info("[TSP ISC] %s,%d: i2c write fail[%d]\n",
			__func__, __LINE__, ret);
		return ISC_I2C_ERROR;
	}

	ret = mms100_i2c_read(_client, ISC_CMD_CONFIRM_STATUS, 1, &rd_buf);
	if (ret < 0) {
		pr_info("[TSP ISC] %s,%d: i2c read fail[%d]\n",
			__func__, __LINE__, ret);
		return ISC_I2C_ERROR;
	}

	if (rd_buf != ISC_STATUS_UPDATE_MODE)
		return ISC_UPDATE_MODE_ENTER_ERROR;

	pr_info("[TSP ISC]End mms100_enter_config_update()\n");

	return ISC_SUCCESS;
}

static int mms100_ISC_clear_page(struct i2c_client *_client,
		unsigned char _page_addr)
{
	int ret;
	unsigned char rd_buf;

	memset(&g_wr_buf[3], 0xFF, PAGE_DATA);

	g_wr_buf[0] = ISC_CMD_UPDATE_MODE;	/* command */
	g_wr_buf[1] = ISC_SUBCMD_DATA_WRITE;	/* sub_command */
	g_wr_buf[2] = _page_addr;

	g_wr_buf[PAGE_HEADER + PAGE_DATA] = crc0_buf[_page_addr];
	g_wr_buf[PAGE_HEADER + PAGE_DATA + 1] = crc1_buf[_page_addr];

	ret = i2c_master_send(_client, g_wr_buf, PACKET_SIZE);

	if (ret < 0) {
		pr_info("[TSP ISC] %s,%d: i2c write fail[%d]\n",
			__func__, __LINE__, ret);
		return ISC_I2C_ERROR;
	}

	ret = mms100_i2c_read(_client, ISC_CMD_CONFIRM_STATUS, 1, &rd_buf);

	if (ret < 0) {
		pr_info("[TSP ISC] %s,%d: i2c read fail[%d]\n",
			__func__, __LINE__, ret);
		return ISC_I2C_ERROR;
	}

	if (rd_buf != ISC_STATUS_CRC_CHECK_SUCCESS)
		return ISC_UPDATE_MODE_ENTER_ERROR;

	pr_info("[TSP ISC]End mms100_ISC_clear_page()\n");
	return ISC_SUCCESS;

}

static int mms100_ISC_clear_validate_markers(struct i2c_client *_client)
{
	int ret_msg;
	int i, j;
	bool is_matched_address;

	for (i = SEC_CORE; i <= SEC_CONFIG; i++) {
		if (section_update_flag[i]) {
			if (ts_info[i].end_addr <= 30 &&
				ts_info[i].end_addr > 0) {
				ret_msg = mms100_ISC_clear_page(_client,
					ts_info[i].end_addr);

				if (ret_msg != ISC_SUCCESS)
					return ret_msg;
			}
		}
	}

	for (i = SEC_CORE; i <= SEC_CONFIG; i++) {
		if (section_update_flag[i]) {
			is_matched_address = false;
			for (j = SEC_CORE; j <= SEC_CONFIG; j++) {
				if (mbin_info[i].end_addr ==
					ts_info[i].end_addr) {
					is_matched_address = true;
					break;
				}
			}

			if (!is_matched_address) {
				if (mbin_info[i].end_addr <= 30 &&
					mbin_info[i].end_addr > 0) {
					ret_msg = mms100_ISC_clear_page(_client,
						mbin_info[i].end_addr);

				if (ret_msg != ISC_SUCCESS)
					return ret_msg;
				}
			}
		}
	}

	return ISC_SUCCESS;
}

static void mms100_calc_crc(unsigned char *crc,
		int page_addr, unsigned char *ptr_fw)
{
	int	i, j;

	unsigned char  ucData;

	unsigned short SeedValue;
	unsigned short CRC_check_buf;
	unsigned short CRC_send_buf;
	unsigned short IN_data;
	unsigned short XOR_bit_1;
	unsigned short XOR_bit_2;
	unsigned short XOR_bit_3;

	CRC_check_buf = 0xFFFF;
	SeedValue = (unsigned short)page_addr;

	for (i = 7; i >= 0; i--) {
		IN_data = (SeedValue >> i) & 0x01;
		XOR_bit_1 = (CRC_check_buf & 0x0001) ^ IN_data;
		XOR_bit_2 = XOR_bit_1^(CRC_check_buf >> 11 & 0x01);
		XOR_bit_3 = XOR_bit_1^(CRC_check_buf >> 4 & 0x01);
		CRC_send_buf = (XOR_bit_1 << 4) | (CRC_check_buf >> 12 & 0x0F);
		CRC_send_buf =
			(CRC_send_buf << 7) | (XOR_bit_2 << 6) |
			(CRC_check_buf >> 5 & 0x3F);
		CRC_send_buf =
			(CRC_send_buf << 4) | (XOR_bit_3 << 3) |
			(CRC_check_buf >> 1 & 0x0007);
		CRC_check_buf = CRC_send_buf;
	}

	for (j = 0; j < 1024; j++) {
		ucData = ptr_fw[j];

		for (i = 7; i >= 0; i--) {
			IN_data = (ucData >> i) & 0x0001;
			XOR_bit_1 = (CRC_check_buf & 0x0001) ^ IN_data;
			XOR_bit_2 = XOR_bit_1^(CRC_check_buf >> 11 & 0x01);
			XOR_bit_3 = XOR_bit_1^(CRC_check_buf >> 4 & 0x01);
			CRC_send_buf =
				(XOR_bit_1 << 4) | (CRC_check_buf >> 12 & 0x0F);
			CRC_send_buf =
				(CRC_send_buf << 7) | (XOR_bit_2 << 6) |
				(CRC_check_buf >> 5 & 0x3F);
			CRC_send_buf =
				(CRC_send_buf << 4) | (XOR_bit_3 << 3) |
				(CRC_check_buf >> 1 & 0x0007);
			CRC_check_buf = CRC_send_buf;
		}
	}

	crc[0] = (unsigned char)((CRC_check_buf >> 8) & 0xFF);
	crc[1] = (unsigned char)((CRC_check_buf	>> 0) & 0xFF);
}

static int mms100_update_section_data(struct i2c_client *_client)
{
	int i, j, ret;
	unsigned char rd_buf;
	unsigned char crc[2];
	const unsigned char *ptr_fw;
	char str_buf[STRING_BUF_LEN];
	int page_addr;

	for (i = 0; i < SECTION_NUM; i++) {
		if (section_update_flag[i]) {
			pr_info("[TSP ISC] section data i2c flash : [%d]", i);

			ptr_fw = fw_mbin[i]->data;

			do {
				sscanf(ptr_fw, "%s", str_buf);
				ptr_fw += strlen(str_buf) + 1;
/*
	pr_info("[TSP ISC] Section[%d] %s", i, str_buf );
*/
			} while (!strstr(str_buf, "[Binary]"));
			ptr_fw += 1;

			for (page_addr = mbin_info[i].start_addr;
				page_addr <= mbin_info[i].end_addr;
				page_addr++) {
				if (page_addr - mbin_info[i].start_addr > 0)
					ptr_fw += 1024;

				g_wr_buf[0] = ISC_CMD_UPDATE_MODE;
				g_wr_buf[1] = ISC_SUBCMD_DATA_WRITE;
				g_wr_buf[2] = (unsigned char)page_addr;

				for (j = 0; j < 1024; j += 4) {
					g_wr_buf[3+j] = ptr_fw[j+3];
					g_wr_buf[3+j+1] = ptr_fw[j+2];
					g_wr_buf[3+j+2] = ptr_fw[j+1];
					g_wr_buf[3+j+3] = ptr_fw[j+0];
				}

				mms100_calc_crc(crc, page_addr, &g_wr_buf[3]);

				g_wr_buf[1027] = crc[0];
				g_wr_buf[1028] = crc[1];
/*
	pr_info("[TSP ISC] [%d] DATA %02X %02X %02X %02X   CRC %02X %02X ",
		page_addr, g_wr_buf[3], g_wr_buf[4],
		g_wr_buf[5], g_wr_buf[6] , crc[0] , crc[1] );
*/
				ret = i2c_master_send(_client,
					g_wr_buf, PACKET_SIZE);
				if (ret < 0) {
					pr_info("[TSP ISC] %s,%d: i2c write fail[%d]\n",
						__func__, __LINE__, ret);
					return ISC_I2C_ERROR;
				}

				ret = mms100_i2c_read(_client,
					ISC_CMD_CONFIRM_STATUS, 1, &rd_buf);
				if (ret < 0) {
					pr_info("[TSP ISC] %s,%d: i2c read fail[%d]\n",
						__func__, __LINE__, ret);
					return ISC_I2C_ERROR;
				}

				if (rd_buf != ISC_STATUS_CRC_CHECK_SUCCESS)
					return ISC_CRC_ERROR;

				section_update_flag[i] = false;
			}
		}
	}

	return ISC_SUCCESS;
}

static int mms100_open_mbinary(struct mms_ts_info *info)
{
	struct i2c_client *_client = info->client;
	int ret = 0;

	ret += request_firmware(&(fw_mbin[0]),\
			"tsp_melfas/note/BOOT.fw", &_client->dev);
	ret += request_firmware(&(fw_mbin[1]),\
			"tsp_melfas/note/CORE.fw", &_client->dev);
	if (info->ldi == 'L') {
		ret += request_firmware(&(fw_mbin[2]),\
			"tsp_melfas/note/CONFL.fw", &_client->dev);
	} else {
		ret += request_firmware(&(fw_mbin[2]),\
			"tsp_melfas/note/CONFM.fw", &_client->dev);
	}

	if (!ret)
		return ISC_SUCCESS;
	else {
		pr_info("[TSP ISC] request_firmware fail");
		return ret;
	}
}

static int mms100_close_mbinary(void)
{
	int i;

	for (i = 0; i < SECTION_NUM; i++) {
		if (fw_mbin[i] != NULL)
			release_firmware(fw_mbin[i]);
	}
	return ISC_SUCCESS;
}

int mms100_ISC_download_mbinary(struct mms_ts_info *info)
{
	struct i2c_client *_client = info->client;
	int ret_msg = ISC_NONE;

	pr_info("[TSP ISC] %s\n", __func__);

	mms100_reset(info);

/*	ret_msg = mms100_check_operating_mode(_client, EC_BOOT_ON_SUCCEEDED);
	if (ret_msg != ISC_SUCCESS)
		goto ISC_ERROR_HANDLE;
*/
	ret_msg = mms100_open_mbinary(info);
	if (ret_msg != ISC_SUCCESS)
		goto ISC_ERROR_HANDLE;

	/*Config version Check*/
	ret_msg = mms100_compare_version_info(_client);
	if (ret_msg != ISC_SUCCESS)
		goto ISC_ERROR_HANDLE;

	ret_msg = mms100_enter_ISC_mode(_client);
	if (ret_msg != ISC_SUCCESS)
		goto ISC_ERROR_HANDLE;

	ret_msg = mms100_enter_config_update(_client);
	if (ret_msg != ISC_SUCCESS)
		goto ISC_ERROR_HANDLE;

	ret_msg = mms100_ISC_clear_validate_markers(_client);
	if (ret_msg != ISC_SUCCESS)
		goto ISC_ERROR_HANDLE;

	pr_info("[TSP ISC]mms100_update_section_data start");

	ret_msg = mms100_update_section_data(_client);
	if (ret_msg != ISC_SUCCESS)
		goto ISC_ERROR_HANDLE;

	pr_info("[TSP ISC]mms100_update_section_data end");

/*	mms100_reset(info); */

	pr_info("[TSP ISC]FIRMWARE_UPDATE_FINISHED!!!\n");

	ret_msg = ISC_SUCCESS;

ISC_ERROR_HANDLE:
	if (ret_msg != ISC_SUCCESS)
		pr_info("[TSP ISC]ISC_ERROR_CODE: %d\n", ret_msg);

	mms100_reset(info);
	mms100_close_mbinary();

	return ret_msg;
}
#endif	/* ISC_DL_MODE start */

static void hw_reboot(struct mms_ts_info *info, bool bootloader)
{
	info->pdata->power(0);
	gpio_direction_output(info->pdata->gpio_sda, bootloader ? 0 : 1);
	gpio_direction_output(info->pdata->gpio_scl, bootloader ? 0 : 1);
	gpio_direction_output(info->pdata->gpio_int, 0);
	msleep(30);
	info->pdata->power(1);
	msleep(30);

	if (bootloader) {
		gpio_set_value(info->pdata->gpio_scl, 0);
		gpio_set_value(info->pdata->gpio_sda, 1);
	} else {
		gpio_set_value(info->pdata->gpio_int, 1);
		gpio_direction_input(info->pdata->gpio_int);
		gpio_direction_input(info->pdata->gpio_scl);
		gpio_direction_input(info->pdata->gpio_sda);
	}
	msleep(40);
}

static inline void hw_reboot_bootloader(struct mms_ts_info *info)
{
	hw_reboot(info, true);
}

static inline void hw_reboot_normal(struct mms_ts_info *info)
{
	hw_reboot(info, false);
}

static void isp_toggle_clk(struct mms_ts_info *info, int start_lvl, int end_lvl,
			   int hold_us)
{
	gpio_set_value(info->pdata->gpio_scl, start_lvl);
	udelay(hold_us);
	gpio_set_value(info->pdata->gpio_scl, end_lvl);
	udelay(hold_us);
}

/* 1 <= cnt <= 32 bits to write */
static void isp_send_bits(struct mms_ts_info *info, u32 data, int cnt)
{
	gpio_direction_output(info->pdata->gpio_int, 0);
	gpio_direction_output(info->pdata->gpio_scl, 0);
	gpio_direction_output(info->pdata->gpio_sda, 0);

	/* clock out the bits, msb first */
	while (cnt--) {
		gpio_set_value(info->pdata->gpio_sda, (data >> cnt) & 1);
		udelay(3);
		isp_toggle_clk(info, 1, 0, 3);
	}
}

/* 1 <= cnt <= 32 bits to read */
static u32 isp_recv_bits(struct mms_ts_info *info, int cnt)
{
	u32 data = 0;

	gpio_direction_output(info->pdata->gpio_int, 0);
	gpio_direction_output(info->pdata->gpio_scl, 0);
	gpio_set_value(info->pdata->gpio_sda, 0);
	gpio_direction_input(info->pdata->gpio_sda);

	/* clock in the bits, msb first */
	while (cnt--) {
		isp_toggle_clk(info, 0, 1, 1);
		data = (data << 1) | (!!gpio_get_value(info->pdata->gpio_sda));
	}

	gpio_direction_output(info->pdata->gpio_sda, 0);
	return data;
}

static void isp_enter_mode(struct mms_ts_info *info, u32 mode)
{
	int cnt;
	unsigned long flags;

	local_irq_save(flags);
	gpio_direction_output(info->pdata->gpio_int, 0);
	gpio_direction_output(info->pdata->gpio_scl, 0);
	gpio_direction_output(info->pdata->gpio_sda, 1);

	mode &= 0xffff;
	for (cnt = 15; cnt >= 0; cnt--) {
		gpio_set_value(info->pdata->gpio_int, (mode >> cnt) & 1);
		udelay(3);
		isp_toggle_clk(info, 1, 0, 3);
	}

	gpio_set_value(info->pdata->gpio_int, 0);
	local_irq_restore(flags);
}

static void isp_exit_mode(struct mms_ts_info *info)
{
	int i;
	unsigned long flags;

	local_irq_save(flags);
	gpio_direction_output(info->pdata->gpio_int, 0);
	udelay(3);

	for (i = 0; i < 10; i++)
		isp_toggle_clk(info, 1, 0, 3);
	local_irq_restore(flags);
}

static void flash_set_address(struct mms_ts_info *info, u16 addr)
{
	/* Only 13 bits of addr are valid.
	 * The addr is in bits 13:1 of cmd */
	isp_send_bits(info, (u32) (addr & 0x1fff) << 1, 18);
}

static void flash_erase(struct mms_ts_info *info)
{
	isp_enter_mode(info, ISP_MODE_FLASH_ERASE);

	gpio_direction_output(info->pdata->gpio_int, 0);
	gpio_direction_output(info->pdata->gpio_scl, 0);
	gpio_direction_output(info->pdata->gpio_sda, 1);

	/* 4 clock cycles with different timings for the erase to
	 * get processed, clk is already 0 from above */
	udelay(7);
	isp_toggle_clk(info, 1, 0, 3);
	udelay(7);
	isp_toggle_clk(info, 1, 0, 3);
	usleep_range(25000, 35000);
	isp_toggle_clk(info, 1, 0, 3);
	usleep_range(150, 200);
	isp_toggle_clk(info, 1, 0, 3);

	gpio_set_value(info->pdata->gpio_sda, 0);

	isp_exit_mode(info);
}

static u32 flash_readl(struct mms_ts_info *info, u16 addr)
{
	int i;
	u32 val;
	unsigned long flags;

	local_irq_save(flags);
	isp_enter_mode(info, ISP_MODE_FLASH_READ);
	flash_set_address(info, addr);

	gpio_direction_output(info->pdata->gpio_scl, 0);
	gpio_direction_output(info->pdata->gpio_sda, 0);
	udelay(40);

	/* data load cycle */
	for (i = 0; i < 6; i++)
		isp_toggle_clk(info, 1, 0, 10);

	val = isp_recv_bits(info, 32);
	isp_exit_mode(info);
	local_irq_restore(flags);

	return val;
}

static void flash_writel(struct mms_ts_info *info, u16 addr, u32 val)
{
	unsigned long flags;

	local_irq_save(flags);
	isp_enter_mode(info, ISP_MODE_FLASH_WRITE);
	flash_set_address(info, addr);
	isp_send_bits(info, val, 32);

	gpio_direction_output(info->pdata->gpio_sda, 1);
	/* 6 clock cycles with different timings for the data to get written
	 * into flash */
	isp_toggle_clk(info, 0, 1, 3);
	isp_toggle_clk(info, 0, 1, 3);
	isp_toggle_clk(info, 0, 1, 6);
	isp_toggle_clk(info, 0, 1, 12);
	isp_toggle_clk(info, 0, 1, 3);
	isp_toggle_clk(info, 0, 1, 3);

	isp_toggle_clk(info, 1, 0, 1);

	gpio_direction_output(info->pdata->gpio_sda, 0);
	isp_exit_mode(info);
	local_irq_restore(flags);
	usleep_range(300, 400);
}

static bool flash_is_erased(struct mms_ts_info *info)
{
	struct i2c_client *client = info->client;
	u32 val;
	u16 addr;

	for (addr = 0; addr < (ISP_MAX_FW_SIZE / 4); addr++) {
		udelay(40);
		val = flash_readl(info, addr);

		if (val != 0xffffffff) {
			dev_dbg(&client->dev,
				"addr 0x%x not erased: 0x%08x != 0xffffffff\n",
				addr, val);
			return false;
		}
	}
	return true;
}

static int fw_write_image(struct mms_ts_info *info, const u8 * data, size_t len)
{
	struct i2c_client *client = info->client;
	u16 addr = 0;

	for (addr = 0; addr < (len / 4); addr++, data += 4) {
		u32 val = get_unaligned_le32(data);
		u32 verify_val;
		int retries = 3;

		while (retries--) {
			flash_writel(info, addr, val);
			verify_val = flash_readl(info, addr);
			if (val == verify_val)
				break;
			dev_err(&client->dev,
				"mismatch @ addr 0x%x: 0x%x != 0x%x\n",
				addr, verify_val, val);
			continue;
		}
		if (retries < 0)
			return -ENXIO;
	}

	return 0;
}

static int fw_download(struct mms_ts_info *info, const u8 * data, size_t len)
{
	struct i2c_client *client = info->client;
	u32 val;
	int ret = 0;
	int i;
	u32 *buf = kzalloc(ISP_CAL_DATA_SIZE * 4, GFP_KERNEL);
	if (!buf) {
		dev_err(&info->client->dev, "%s: failed to allocate memory\n",
			__func__);
		return -ENOMEM;
	}

	if (len % 4) {
		dev_err(&client->dev,
			"fw image size (%d) must be a multiple of 4 bytes\n",
			len);
		kfree(buf);
		return -EINVAL;
	} else if (len > ISP_MAX_FW_SIZE) {
		dev_err(&client->dev,
			"fw image is too big, %d > %d\n", len, ISP_MAX_FW_SIZE);
		kfree(buf);
		return -EINVAL;
	}

	dev_info(&client->dev, "fw download start\n");

	info->pdata->power(0);
	gpio_direction_output(info->pdata->gpio_sda, 0);
	gpio_direction_output(info->pdata->gpio_scl, 0);
	gpio_direction_output(info->pdata->gpio_int, 0);

	hw_reboot_bootloader(info);

	dev_info(&client->dev, "calibration data backup\n");
	for (i = 0; i < ISP_CAL_DATA_SIZE; i++)
		buf[i] = flash_readl(info, ISP_IC_INFO_ADDR);

	val = flash_readl(info, ISP_IC_INFO_ADDR);
	dev_info(&client->dev, "IC info: 0x%02x (%x)\n", val & 0xff, val);

	dev_info(&client->dev, "fw erase...\n");
	flash_erase(info);
	if (!flash_is_erased(info)) {
		ret = -ENXIO;
		goto err;
	}

	dev_info(&client->dev, "fw write...\n");
	/* XXX: what does this do?! */
	flash_writel(info, ISP_IC_INFO_ADDR, 0xffffff00 | (val & 0xff));
	usleep_range(1000, 1500);
	ret = fw_write_image(info, data, len);
	if (ret)
		goto err;
	usleep_range(1000, 1500);

	dev_info(&client->dev, "restoring data\n");
	for (i = 0; i < ISP_CAL_DATA_SIZE; i++)
		flash_writel(info, ISP_IC_INFO_ADDR, buf[i]);
	kfree(buf);

	dev_info(&client->dev, "fw download done...\n");
	hw_reboot_normal(info);
	msleep(200);
	return 0;

err:
	dev_err(&client->dev, "fw download failed...\n");
	kfree(buf);
	hw_reboot_normal(info);
	return ret;
}

#if defined(SEC_TSP_ISC_FW_UPDATE)
static u16 gen_crc(u8 data, u16 pre_crc)
{
	u16 crc;
	u16 cur;
	u16 temp;
	u16 bit_1;
	u16 bit_2;
	int i;

	crc = pre_crc;
	for (i = 7; i >= 0; i--) {
		cur = ((data >> i) & 0x01) ^ (crc & 0x0001);
		bit_1 = cur ^ (crc >> 11 & 0x01);
		bit_2 = cur ^ (crc >> 4 & 0x01);
		temp = (cur << 4) | (crc >> 12 & 0x0F);
		temp = (temp << 7) | (bit_1 << 6) | (crc >> 5 & 0x3F);
		temp = (temp << 4) | (bit_2 << 3) | (crc >> 1 & 0x0007);
		crc = temp;
	}
	return crc;
}

static int isc_fw_download(struct mms_ts_info *info, const u8 * data,
			   size_t len)
{
	u8 *buff;
	u16 crc_buf;
	int src_idx;
	int dest_idx;
	int ret;
	int i, j;

	buff = kzalloc(ISC_PKT_SIZE, GFP_KERNEL);
	if (!buff) {
		dev_err(&info->client->dev, "%s: failed to allocate memory\n",
			__func__);
		ret = -1;
		goto err_mem_alloc;
	}

	/* enterring ISC mode */
	*buff = ISC_ENTER_ISC_DATA;
	ret = i2c_smbus_write_byte_data(info->client,
		ISC_ENTER_ISC_CMD, *buff);
	if (ret < 0) {
		dev_err(&info->client->dev,
			"fail to enter ISC mode(err=%d)\n", ret);
		goto fail_to_isc_enter;
	}
	usleep_range(10000, 20000);
	dev_info(&info->client->dev, "Enter ISC mode\n");

	/*enter ISC update mode */
	*buff = ISC_ENTER_UPDATE_DATA;
	ret = i2c_smbus_write_i2c_block_data(info->client,
					     ISC_CMD,
					     ISC_ENTER_UPDATE_DATA_LEN, buff);
	if (ret < 0) {
		dev_err(&info->client->dev,
			"fail to enter ISC update mode(err=%d)\n", ret);
		goto fail_to_isc_update;
	}
	dev_info(&info->client->dev, "Enter ISC update mode\n");

	/* firmware write */
	*buff = ISC_CMD;
	*(buff + 1) = ISC_DATA_WRITE_SUB_CMD;
	for (i = 0; i < ISC_PKT_NUM; i++) {
		*(buff + 2) = i;
		crc_buf = gen_crc(*(buff + 2), ISC_DEFAULT_CRC);

		for (j = 0; j < ISC_PKT_DATA_SIZE; j++) {
			dest_idx = ISC_PKT_HEADER_SIZE + j;
			src_idx = i * ISC_PKT_DATA_SIZE +
			    ((int)(j / WORD_SIZE)) * WORD_SIZE -
			    (j % WORD_SIZE) + 3;
			*(buff + dest_idx) = *(data + src_idx);
			crc_buf = gen_crc(*(buff + dest_idx), crc_buf);
		}

		*(buff + ISC_PKT_DATA_SIZE + ISC_PKT_HEADER_SIZE + 1) =
		    crc_buf & 0xFF;
		*(buff + ISC_PKT_DATA_SIZE + ISC_PKT_HEADER_SIZE) =
		    crc_buf >> 8 & 0xFF;

		ret = i2c_master_send(info->client, buff, ISC_PKT_SIZE);
		if (ret < 0) {
			dev_err(&info->client->dev,
				"fail to firmware writing on packet %d.(%d)\n",
				i, ret);
			goto fail_to_fw_write;
		}
		usleep_range(1, 5);

		/* confirm CRC */
		ret = i2c_smbus_read_byte_data(info->client,
					       ISC_CHECK_STATUS_CMD);
		if (ret == ISC_CONFIRM_CRC) {
			dev_info(&info->client->dev,
				 "updating %dth firmware data packet.\n", i);
		} else {
			dev_err(&info->client->dev,
				"fail to firmware update on %dth (%X).\n",
				i, ret);
			ret = -1;
			goto fail_to_confirm_crc;
		}
	}

	ret = 0;

fail_to_confirm_crc:
fail_to_fw_write:
	/* exit ISC mode */
	*buff = ISC_EXIT_ISC_SUB_CMD;
	*(buff + 1) = ISC_EXIT_ISC_SUB_CMD2;
	i2c_smbus_write_i2c_block_data(info->client, ISC_CMD, 2, buff);
	usleep_range(10000, 20000);
fail_to_isc_update:
	hw_reboot_normal(info);
fail_to_isc_enter:
	kfree(buff);
err_mem_alloc:
	return ret;
}
#endif /* SEC_TSP_ISC_FW_UPDATE */

static int get_fw_version(struct mms_ts_info *info, u8 area)
{
	struct i2c_client *client = info->client;
	struct i2c_adapter *adapter = client->adapter;
	struct i2c_msg msg;
	u8 reg = MMS_CORE_VERSION;
	int ret;
	unsigned char buf[4];

	msg.addr = client->addr;
	msg.flags = 0x00;
	msg.len = 1;
	msg.buf = &reg;

	ret = i2c_transfer(adapter, &msg, 1);

	if (ret >= 0) {
		msg.addr = client->addr;
		msg.flags = I2C_M_RD;
		msg.len = 4;
		msg.buf = buf;

		ret = i2c_transfer(adapter, &msg, 1);
	}
	if (ret < 0) {
		pr_err("[TSP] : read error : [%d]", ret);
		return ret;
	}

	if (area == SEC_BOOTLOADER)
		return buf[0];
	else if (area == SEC_CORE)
		return buf[1];
	else if (area == SEC_CONFIG)
		return buf[2];
	else
		return 0;
}

static int get_panel_version(struct mms_ts_info *info)
{
	int ret;
	int retries = 3;

	/* this seems to fail sometimes after a reset.. retry a few times */
	do {
		ret = i2c_smbus_read_byte_data(info->client, MMS_COMPAT_GROUP);
	} while (ret < 0 && retries-- > 0);

	return ret;
}

/*
static int mms_ts_enable(struct mms_ts_info *info, int wakeupcmd)
{
	mutex_lock(&info->lock);
	if (info->enabled)
		goto out;

	if (wakeupcmd == 1) {
		i2c_smbus_write_byte_data(info->client, 0, 0);
		usleep_range(3000, 5000);
	}
	info->enabled = true;
	enable_irq(info->irq);
out:
	mutex_unlock(&info->lock);
	return 0;
}

static int mms_ts_disable(struct mms_ts_info *info, int sleepcmd)
{
	mutex_lock(&info->lock);
	if (!info->enabled)
		goto out;
	disable_irq_nosync(info->irq);
	if (sleepcmd == 1) {
		i2c_smbus_write_byte_data(info->client, MMS_MODE_CONTROL, 0);
		usleep_range(10000, 12000);
	}
	info->enabled = false;
	touch_is_pressed = 0;
out:
	mutex_unlock(&info->lock);
	return 0;
}
*/

static int mms_ts_fw_info(struct mms_ts_info *info)
{
	struct i2c_client *client = info->client;
	int ret = 0;
	int ver;

	ver = get_fw_version(info, SEC_CONFIG);
	info->fw_ic_ver = ver;
		dev_info(&client->dev,
			 "[TSP]fw version 0x%02x !!!!\n", ver);

	if (ver < 0) {
		ret = 1;
		dev_err(&client->dev,
			"i2c fail...tsp driver unload.\n");
		return ret;
	}

	if (!info->pdata || !info->pdata->mux_fw_flash) {
		ret = 1;
		dev_err(&client->dev,
			"fw cannot be updated, missing platform data\n");
		return ret;
	}

	return ret;
}

static int mms_ts_fw_load(struct mms_ts_info *info, bool force, char ldi)
{

	struct i2c_client *client = info->client;
	int ret = 0;
	int ver;
	int bin_ver;
	int retries = 3;

	ver = get_fw_version(info, SEC_CONFIG);
	info->fw_ic_ver = ver;
	dev_info(&client->dev,
		 "[TSP]fw version 0x%02x !!!!\n", ver);

	if (!info->pdata || !info->pdata->mux_fw_flash) {
		ret = 1;
		dev_err(&client->dev,
			"fw cannot be updated, missing platform data\n");
		goto out;
	}

	if (ldi == 'N') {
		if (info->ldi == 'M')
			bin_ver = FW_VERSION_M;
		else
			bin_ver = FW_VERSION_L;
	} else {
		if (ldi == 'M')
			bin_ver = FW_VERSION_M;
		else
			bin_ver = FW_VERSION_L;
	}

	if (!force) {
		if ((ver >= bin_ver) && (ver != 0xff)) {
			dev_info(&client->dev,
				"fw version update does not need\n");
			goto done;
		}
	}

	while (retries--) {
		ret = mms100_ISC_download_mbinary(info);

		ver = get_fw_version(info, SEC_CONFIG);
		info->fw_ic_ver = ver;

		if (ret == 0) {
			pr_err("[TSP] mms100_ISC_download_mbinary success");
			goto done;
		} else {
			pr_err("[TSP] mms100_ISC_download_mbinary fail [%d]",
						ret);
			ret = 1;
		}

		dev_err(&client->dev, "retrying flashing\n");
	}

out:
done:
	return ret;
}

#ifdef SEC_TSP_FACTORY_TEST
static void set_cmd_result(struct mms_ts_info *info, char *buff, int len)
{
	strncat(info->cmd_result, buff, len);
}

static int get_data(struct mms_ts_info *info, u8 addr, u8 size, u8 *array)
{
	struct i2c_client *client = info->client;
	struct i2c_adapter *adapter = client->adapter;
	struct i2c_msg msg;
	u8 reg = addr;
	unsigned char buf[size];
	int ret;

	msg.addr = client->addr;
	msg.flags = 0x00;
	msg.len = 1;
	msg.buf = &reg;

	ret = i2c_transfer(adapter, &msg, 1);

	if (ret >= 0) {
		msg.addr = client->addr;
		msg.flags = I2C_M_RD;
		msg.len = size;
		msg.buf = buf;

		ret = i2c_transfer(adapter, &msg, 1);
	}
	if (ret < 0) {
		pr_err("[TSP] : read error : [%d]", ret);
		return ret;
	}

	memcpy(array, &buf, size);
	return size;
}

static void get_intensity_data(struct mms_ts_info *info)
{
	u8 w_buf[4];
	u8 r_buf;
	u8 read_buffer[60] = {0};
	int i, j;
	int ret;
	u16 max_value = 0, min_value = 0;
	u16 raw_data;
	char buff[TSP_CMD_STR_LEN] = {0};

	disable_irq(info->irq);

	w_buf[0] = ADDR_UNIV_CMD;
	w_buf[1] = CMD_GET_INTEN;
	w_buf[2] = 0xFF;
	for (i = 0; i < RX_NUM; i++) {
		w_buf[3] = i;

		ret = i2c_smbus_write_i2c_block_data(info->client,
			w_buf[0], 3, &w_buf[1]);
		if (ret < 0)
			goto err_i2c;
		usleep_range(1, 5);

		ret = i2c_smbus_read_i2c_block_data(info->client,
			CMD_RESULT_SZ, 1, &r_buf);
		if (ret < 0)
			goto err_i2c;

		ret = get_data(info, CMD_RESULT, r_buf, read_buffer);
		if (ret < 0)
			goto err_i2c;

		for (j = 0; j < r_buf/2; j++) {
			raw_data = read_buffer[2*j] | (read_buffer[2*j+1] << 8);
			if (raw_data > 32767)
				raw_data = 0;
			if (i == 0 && j == 0) {
				max_value = min_value = raw_data;
			} else {
				max_value = max(max_value, raw_data);
				min_value = min(min_value, raw_data);
			}
			info->intensity[i * info->tx_num + j] = raw_data;
			dev_dbg(&info->client->dev,
				"[TSP] intensity[%d][%d] = %d\n", j, i,
				info->intensity[i * info->tx_num + j]);
		}
	}

	snprintf(buff, sizeof(buff), "%d,%d", min_value, max_value);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));

	enable_irq(info->irq);

	return;

err_i2c:
	dev_err(&info->client->dev, "%s: fail to i2c (cmd=%d)\n",
		__func__, MMS_VSC_CMD_INTENSITY);
}

static void get_raw_data(struct mms_ts_info *info, u8 cmd)
{
	u8 w_buf[4];
	u8 r_buf = 0;
	u8 read_buffer[60] = {0};
	int ret;
	int i, j;
	int max_value = 0, min_value = 0;
	int raw_data;
	int retry;
	char buff[TSP_CMD_STR_LEN] = {0};
	int gpio = info->pdata->gpio_int;

	disable_irq(info->irq);

	ret = i2c_smbus_write_byte_data(info->client,
		ADDR_UNIV_CMD, CMD_ENTER_TEST);
	if (ret < 0)
		goto err_i2c;

	/* event type check */
	retry = 1;
	while (retry) {
		while (gpio_get_value(gpio))
			udelay(100);

		ret = i2c_smbus_read_i2c_block_data(info->client,
			0x0F, 1, &r_buf);
		if (ret < 0)
			goto err_i2c;

		ret = i2c_smbus_read_i2c_block_data(info->client,
			0x10, 1, &r_buf);
		if (ret < 0)
			goto err_i2c;

		dev_info(&info->client->dev, "event type = 0x%x\n", r_buf);
		if (r_buf == 0x0C)
			retry = 0;
	}

	w_buf[0] = ADDR_UNIV_CMD;
	if (cmd == MMS_VSC_CMD_CM_DELTA)
		w_buf[1] = CMD_CM_DELTA;
	else
		w_buf[1] = CMD_CM_ABS;
	ret = i2c_smbus_write_i2c_block_data(info->client,
		 w_buf[0], 1, &w_buf[1]);
	if (ret < 0)
		goto err_i2c;
	while (gpio_get_value(gpio))
		udelay(100);

	ret = i2c_smbus_read_i2c_block_data(info->client,
		CMD_RESULT_SZ, 1, &r_buf);
	if (ret < 0)
		goto err_i2c;
	ret = i2c_smbus_read_i2c_block_data(info->client,
		CMD_RESULT, 1, &r_buf);
	if (ret < 0)
		goto err_i2c;

	if (r_buf == 1)
		dev_info(&info->client->dev, "PASS\n");
	else
		dev_info(&info->client->dev, "FAIL\n");

	if (cmd == MMS_VSC_CMD_CM_DELTA)
		w_buf[1] = CMD_GET_DELTA;
	else
		w_buf[1] = CMD_GET_ABS;
	w_buf[2] = 0xFF;

	for (i = 0; i < RX_NUM; i++) {
		w_buf[3] = i;

		ret = i2c_smbus_write_i2c_block_data(info->client,
			w_buf[0], 3, &w_buf[1]);
		if (ret < 0)
			goto err_i2c;

		while (gpio_get_value(gpio))
			udelay(100);

		ret = i2c_smbus_read_i2c_block_data(info->client,
			CMD_RESULT_SZ, 1, &r_buf);
		if (ret < 0)
			goto err_i2c;

		ret = get_data(info, CMD_RESULT, r_buf, read_buffer);
		if (ret < 0)
			goto err_i2c;

		for (j = 0; j < info->tx_num; j++) {
			raw_data = read_buffer[2*j] | (read_buffer[2*j+1] << 8);
			if (i == 0 && j == 0) {
				max_value = min_value = raw_data;
			} else {
				max_value = max(max_value, raw_data);
				min_value = min(min_value, raw_data);
			}

			if (cmd == MMS_VSC_CMD_CM_DELTA) {
				info->inspection[i * info->tx_num + j] =
					raw_data;
				dev_dbg(&info->client->dev,
					"[TSP] delta[%d][%d] = %d\n", j, i,
					info->inspection[i * info->tx_num + j]);
			} else if (cmd == MMS_VSC_CMD_CM_ABS) {
				info->raw[i * info->tx_num + j] =
					raw_data;
				dev_dbg(&info->client->dev,
					"[TSP] raw[%d][%d] = %d\n", j, i,
					info->raw[i * info->tx_num + j]);
			} else if (cmd == MMS_VSC_CMD_REFER) {
				info->reference[i * info->tx_num + j] =
					raw_data;
				dev_dbg(&info->client->dev,
					"[TSP] reference[%d][%d] = %d\n", j, i,
					info->reference[i * info->tx_num + j]);
			}
		}
	}

	ret = i2c_smbus_write_byte_data(info->client,
		ADDR_UNIV_CMD, CMD_EXIT_TEST);

	snprintf(buff, sizeof(buff), "%d,%d", min_value, max_value);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));

	touch_is_pressed = 0;
	release_all_fingers(info);

	mms_pwr_on_reset(info);
	info->enabled = true;

	if (info->fw_ic_ver < 0x18) {
		if (info->ta_status) {
			dev_notice(&info->client->dev, "TA connect!!!\n");
			i2c_smbus_write_byte_data(info->client, 0x33, 0x1);
		} else {
			dev_notice(&info->client->dev, "TA disconnect!!!\n");
			i2c_smbus_write_byte_data(info->client, 0x33, 0x2);
		}
	}
	mms_set_noise_mode(info);

	enable_irq(info->irq);

	return;

err_i2c:
	dev_err(&info->client->dev, "%s: fail to i2c (cmd=%d)\n",
		__func__, cmd);
}

static void get_raw_data_all(struct mms_ts_info *info, u8 cmd)
{
	u8 w_buf[6];
	u8 read_buffer[2];	/* 52 */
	int gpio;
	int ret;
	int i, j;
	u32 max_value = 0, min_value = 0;
	u32 raw_data;
	char buff[TSP_CMD_STR_LEN] = {0};
	gpio = info->pdata->gpio_int;

/*      gpio = msm_irq_to_gpio(info->irq); */
	disable_irq(info->irq);

	w_buf[0] = MMS_VSC_CMD;	/* vendor specific command id */
	w_buf[1] = MMS_VSC_MODE;	/* mode of vendor */
	w_buf[2] = 0;		/* tx line */
	w_buf[3] = 0;		/* rx line */
	w_buf[4] = 0;		/* reserved */
	w_buf[5] = 0;		/* sub command */

	if (cmd == MMS_VSC_CMD_EXIT) {
		w_buf[5] = MMS_VSC_CMD_EXIT;	/* exit test mode */

		ret = i2c_smbus_write_i2c_block_data(info->client,
						     w_buf[0], 5, &w_buf[1]);
		if (ret < 0)
			goto err_i2c;
		enable_irq(info->irq);
		msleep(200);
		return;
	}

	/* MMS_VSC_CMD_CM_DELTA or MMS_VSC_CMD_CM_ABS
	 * this two mode need to enter the test mode
	 * exit command must be followed by testing.
	 */
	if (cmd == MMS_VSC_CMD_CM_DELTA || cmd == MMS_VSC_CMD_CM_ABS) {
		/* enter the debug mode */
		w_buf[2] = 0x0;	/* tx */
		w_buf[3] = 0x0;	/* rx */
		w_buf[5] = MMS_VSC_CMD_ENTER;

		ret = i2c_smbus_write_i2c_block_data(info->client,
						     w_buf[0], 5, &w_buf[1]);
		if (ret < 0)
			goto err_i2c;

		/* wating for the interrupt */
		while (gpio_get_value(gpio))
			udelay(100);
	}

	for (i = 0; i < RX_NUM; i++) {
		for (j = 0; j < info->tx_num; j++) {

			w_buf[2] = j;	/* tx */
			w_buf[3] = i;	/* rx */
			w_buf[5] = cmd;

			ret = i2c_smbus_write_i2c_block_data(info->client,
					w_buf[0], 5, &w_buf[1]);
			if (ret < 0)
				goto err_i2c;

			usleep_range(1, 5);

			ret = i2c_smbus_read_i2c_block_data(info->client, 0xBF,
							    2, read_buffer);
			if (ret < 0)
				goto err_i2c;

			raw_data = ((u16) read_buffer[1] << 8) | read_buffer[0];
			if (i == 0 && j == 0) {
				max_value = min_value = raw_data;
			} else {
				max_value = max(max_value, raw_data);
				min_value = min(min_value, raw_data);
			}

			if (cmd == MMS_VSC_CMD_INTENSITY) {
				info->intensity[i * info->tx_num + j] =
					raw_data;
				dev_dbg(&info->client->dev,
					"[TSP] intensity[%d][%d] = %d\n", j, i,
					info->intensity[i * info->tx_num + j]);
			} else if (cmd == MMS_VSC_CMD_CM_DELTA) {
				info->inspection[i * info->tx_num + j] =
					raw_data;
				dev_dbg(&info->client->dev,
					"[TSP] delta[%d][%d] = %d\n", j, i,
					info->inspection[i * info->tx_num + j]);
			} else if (cmd == MMS_VSC_CMD_CM_ABS) {
				info->raw[i * info->tx_num + j] =
					raw_data;
				dev_dbg(&info->client->dev,
					"[TSP] raw[%d][%d] = %d\n", j, i,
					info->raw[i * info->tx_num + j]);
			} else if (cmd == MMS_VSC_CMD_REFER) {
				info->reference[i * info->tx_num + j] =
					raw_data;
				dev_dbg(&info->client->dev,
					"[TSP] reference[%d][%d] = %d\n", j, i,
					info->reference[i * info->tx_num + j]);
			}
		}

	}

	snprintf(buff, sizeof(buff), "%d,%d", min_value, max_value);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));

	enable_irq(info->irq);

	return;

err_i2c:
	dev_err(&info->client->dev, "%s: fail to i2c (cmd=%d)\n",
		__func__, cmd);
}

static ssize_t show_close_tsp_test(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct mms_ts_info *info = dev_get_drvdata(dev);

	get_raw_data_all(info, MMS_VSC_CMD_EXIT);
	info->ft_flag = 0;

	return snprintf(buf, TSP_BUF_SIZE, "%u\n", 0);
}

static void set_default_result(struct mms_ts_info *info)
{
	char delim = ':';

	memset(info->cmd_result, 0x00, ARRAY_SIZE(info->cmd_result));
	memcpy(info->cmd_result, info->cmd, strlen(info->cmd));
	strncat(info->cmd_result, &delim, 1);
}

static int check_rx_tx_num(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;

	char buff[TSP_CMD_STR_LEN] = {0};
	int node;

	if (info->cmd_param[0] < 0 ||
			info->cmd_param[0] >= info->tx_num  ||
			info->cmd_param[1] < 0 ||
			info->cmd_param[1] >= RX_NUM) {
		snprintf(buff, sizeof(buff) , "%s", "NG");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = 3;

		dev_info(&info->client->dev, "%s: parameter error: %u,%u\n",
				__func__, info->cmd_param[0],
				info->cmd_param[1]);
		node = -1;
		return node;
}
	node = info->cmd_param[1] * info->tx_num + info->cmd_param[0];
	dev_info(&info->client->dev, "%s: node = %d\n", __func__,
			node);
	return node;

}

static void not_support_cmd(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;
	char buff[16] = {0};

	set_default_result(info);
	sprintf(buff, "%s", "NA");
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 4;
	dev_info(&info->client->dev, "%s: \"%s(%d)\"\n", __func__,
				buff, strnlen(buff, sizeof(buff)));
	return;
}

static int mms_ts_core_fw_load(struct mms_ts_info *info)
{
	struct i2c_client *client = info->client;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	int ret = 0;
	int ver = 0, fw_bin_ver = 0;
	int retries = 5;
	const u8 *buff = 0;
	long fsize = 0;
	const struct firmware *tsp_fw = NULL;

	ver = get_fw_version(info, SEC_CONFIG);
	info->fw_ic_ver = ver;

	dev_info(&client->dev, "Entered REQ_FW\n");
	if (info->ldi == 'L') {
		fw_bin_ver = FW_VERSION_L;
		ret = request_firmware(&tsp_fw,
			"tsp_melfas/note/melfasl.fw", &(client->dev));
	} else {
		fw_bin_ver = FW_VERSION_M;
		ret = request_firmware(&tsp_fw,
			"tsp_melfas/note/melfasm.fw", &(client->dev));
	}
	dev_info(&client->dev,
		"fw_ic_ver = 0x%02x, fw_bin_ver = 0x%02x\n",
		info->fw_ic_ver, fw_bin_ver);
	if (ret) {
		dev_err(&client->dev, "request firmware error!!\n");
		return 1;
	}

	fsize = tsp_fw->size;
	buff = kzalloc((size_t)fsize, GFP_KERNEL);
	if (!buff) {
		dev_err(&client->dev, "fail to alloc buffer for fw\n");
		info->cmd_state = 3;
		release_firmware(tsp_fw);
		return 1;
	}

	memcpy((void *)buff, tsp_fw->data, fsize);
	release_firmware(tsp_fw);

	disable_irq(info->irq);
	while (retries--) {
		i2c_lock_adapter(adapter);
		info->pdata->mux_fw_flash(true);

		ret = fw_download(info, (const u8 *)buff,
				(const size_t)fsize);

		info->pdata->mux_fw_flash(false);
		i2c_unlock_adapter(adapter);

		if (ret < 0) {
			dev_err(&client->dev, "retrying flashing\n");
			continue;
		}

		ver = get_fw_version(info, SEC_CONFIG);
		info->fw_ic_ver = ver;

		if (ver == fw_bin_ver) {
			dev_info(&client->dev,
			  "fw update done. ver = 0x%02x\n", ver);
			enable_irq(info->irq);
			kfree(buff);
			return 0;
		} else {
			dev_err(&client->dev,
				"ERROR : fw version is still wrong (0x%x != 0x%x)\n",
				ver, fw_bin_ver);
		}
		dev_err(&client->dev, "retrying flashing\n");
	}
	kfree(buff);
	return 1;
}

static void fw_update(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	int ret = 0;
	int ver = 0, fw_bin_ver = 0;
	int retries = 5;
	const u8 *buff = 0;
	mm_segment_t old_fs = {0};
	struct file *fp = NULL;
	long fsize = 0, nread = 0;
	const struct firmware *tsp_fw = NULL;
	char fw_path[MAX_FW_PATH+1];
	char result[16] = {0};

	if (info->panel == 'A') {
		dev_dbg(&client->dev, "support only Melfas panel\n");
		dev_dbg(&client->dev, "fw update do not excute\n");
		goto not_support;
	}
	set_default_result(info);
	if (info->ldi == 'L')
		fw_bin_ver = FW_VERSION_L;
	else
		fw_bin_ver = FW_VERSION_M;

	dev_info(&client->dev,
		"fw_ic_ver = 0x%02x, fw_bin_ver = 0x%02x\n",
		info->fw_ic_ver, fw_bin_ver);

	if (info->cmd_param[0] == 0) {
		if (info->fw_core_ver > 0x53) {
			dev_info(&client->dev,
				"fw version update does not need\n");
			info->cmd_state = 2;
			goto do_not_need_update;
		} else if (info->fw_core_ver == 0x53) {
			if (info->fw_ic_ver >= fw_bin_ver) {
				dev_info(&client->dev,
					"fw version update does not need\n");
				info->cmd_state = 2;
				goto do_not_need_update;
			}
		} else { /* core < 0x53 */
			dev_info(&client->dev,
				"fw version update need(core:0x%x)\n",
				info->fw_core_ver);
		}
	}

	switch (info->cmd_param[0]) {
	case BUILT_IN:
		dev_info(&client->dev, "built in fw is loaded!!\n");
		disable_irq(info->irq);
		while (retries--) {
#if 0
			ret = mms100_ISC_download_mbinary(info);
#else
			ret = mms_ts_core_fw_load(info);
#endif
			ver = get_fw_version(info, SEC_CONFIG);
			info->fw_ic_ver = ver;
			if (ret == 0) {
				pr_err("[TSP] mms100_ISC_download_mbinary success");
				info->cmd_state = 2;
				enable_irq(info->irq);
				return;
			} else {
				pr_err("[TSP] mms100_ISC_download_mbinary fail[%d]",
							ret);
				info->cmd_state = 3;
			}
		}
		enable_irq(info->irq);
		return;
		break;

	case UMS:
		old_fs = get_fs();
		set_fs(get_ds());

		snprintf(fw_path, MAX_FW_PATH, "/sdcard/%s", TSP_FW_FILENAME);
		fp = filp_open(fw_path, O_RDONLY, 0);
		if (IS_ERR(fp)) {
			dev_err(&client->dev,
				"file %s open error:%d\n", fw_path, (s32)fp);
			info->cmd_state = 3;
			goto err_open;
		}

		fsize = fp->f_path.dentry->d_inode->i_size;

		buff = kzalloc((size_t)fsize, GFP_KERNEL);
		if (!buff) {
			dev_err(&client->dev, "fail to alloc buffer for fw\n");
			info->cmd_state = 3;
			goto err_alloc;
		}

		nread = vfs_read(fp, (char __user *)buff, fsize, &fp->f_pos);
		if (nread != fsize) {
			/*dev_err("fail to read file %s (nread = %d)\n",
					fw_path, nread);*/
			info->cmd_state = 3;
			goto err_fw_size;
		}

		filp_close(fp, current->files);
		set_fs(old_fs);
		dev_info(&client->dev, "ums fw is loaded!!\n");
		break;

	case REQ_FW:
		dev_info(&client->dev, "Entered REQ_FW case\n");
		ret = request_firmware(&tsp_fw, TSP_FW_FILENAME,
					&(client->dev));
		if (ret) {
			dev_err(&client->dev, "request firmware error!!\n");
			goto not_support;
		}

		fsize = tsp_fw->size;
		buff = kzalloc((size_t)fsize, GFP_KERNEL);
		if (!buff) {
			dev_err(&client->dev, "fail to alloc buffer for fw\n");
			info->cmd_state = 3;
			release_firmware(tsp_fw);
			goto not_support;
		}

		memcpy((void *)buff, tsp_fw->data, fsize);
		release_firmware(tsp_fw);
		break;

	default:
		dev_err(&client->dev, "invalid fw file type!!\n");
		goto not_support;
	}

	disable_irq(info->irq);
	while (retries--) {
		i2c_lock_adapter(adapter);
		info->pdata->mux_fw_flash(true);

		ret = fw_download(info, (const u8 *)buff,
				(const size_t)fsize);

		info->pdata->mux_fw_flash(false);
		i2c_unlock_adapter(adapter);

		if (ret < 0) {
			dev_err(&client->dev, "retrying flashing\n");
			continue;
		}

		ver = get_fw_version(info, SEC_CONFIG);
		info->fw_ic_ver = ver;

		if (info->cmd_param[0] == 1 || info->cmd_param[0] == 2) {
			dev_info(&client->dev,
			  "fw update done. ver = 0x%02x\n", ver);
			info->cmd_state = 2;
			snprintf(result, sizeof(result) , "%s", "OK");
			set_cmd_result(info, result,
					strnlen(result, sizeof(result)));
			enable_irq(info->irq);
			kfree(buff);
			return;
		} else if (ver == fw_bin_ver) {
			dev_info(&client->dev,
			  "fw update done. ver = 0x%02x\n", ver);
			info->cmd_state = 2;
			snprintf(result, sizeof(result) , "%s", "OK");
			set_cmd_result(info, result,
					strnlen(result, sizeof(result)));
			enable_irq(info->irq);
			return;
		} else {
			dev_err(&client->dev,
					"ERROR : fw version is still wrong (0x%x != 0x%x)\n",
					ver, fw_bin_ver);
		}
		dev_err(&client->dev, "retrying flashing\n");
	}

if (fp != NULL) {
err_fw_size:
	kfree(buff);
err_alloc:
	filp_close(fp, NULL);
err_open:
	set_fs(old_fs);
}
not_support:
do_not_need_update:
	snprintf(result, sizeof(result) , "%s", "NG");
	set_cmd_result(info, result, strnlen(result, sizeof(result)));
	return;
}

static void get_fw_ver_bin(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;

	char buff[16] = {0};

	set_default_result(info);
	if (info->ldi == 'L')
		snprintf(buff, sizeof(buff), "ME0053%02x", FW_VERSION_L);
	else
		snprintf(buff, sizeof(buff), "ME0053%02x", FW_VERSION_M);

	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}

static void get_fw_ver_ic(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;

	char buff[16] = {0};

	set_default_result(info);

	if (info->enabled) {
		info->fw_core_ver = get_fw_version(info, SEC_CORE);
		info->fw_ic_ver = get_fw_version(info, SEC_CONFIG);
	}
	snprintf(buff, sizeof(buff), "ME00%02x%02x",
		info->fw_core_ver, info->fw_ic_ver);

	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}

static void get_config_ver(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;

	char buff[20] = {0};

	set_default_result(info);

	if (info->ldi == 'L')
		snprintf(buff, sizeof(buff), "N7100_Me_0921_L");
	else
		snprintf(buff, sizeof(buff), "N7100_Me_0911_M");
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}

static void get_threshold(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;

	char buff[16] = {0};
	int threshold;

	set_default_result(info);

	threshold = i2c_smbus_read_byte_data(info->client, 0x05);
	if (threshold < 0) {
		snprintf(buff, sizeof(buff), "%s", "NG");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = 3;
		return;
}
	snprintf(buff, sizeof(buff), "%d", threshold);

	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}

static void module_off_master(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;

	char buff[3] = {0};

	mutex_lock(&info->lock);
	if (info->enabled) {
		disable_irq(info->irq);
		info->enabled = false;
		touch_is_pressed = 0;
	}
	mutex_unlock(&info->lock);

	info->pdata->power(0);

	if (info->pdata->is_vdd_on() == 0)
		snprintf(buff, sizeof(buff), "%s", "OK");
	else
		snprintf(buff, sizeof(buff), "%s", "NG");

	set_default_result(info);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));

	if (strncmp(buff, "OK", 2) == 0)
		info->cmd_state = 2;
	else
		info->cmd_state = 3;

	dev_info(&info->client->dev, "%s: %s\n", __func__, buff);
}

static void module_on_master(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;

	char buff[3] = {0};

	mms_pwr_on_reset(info);

	mutex_lock(&info->lock);
	if (!info->enabled) {
		enable_irq(info->irq);
		info->enabled = true;
	}
	mutex_unlock(&info->lock);

	if (info->pdata->is_vdd_on() == 1)
		snprintf(buff, sizeof(buff), "%s", "OK");
	else
		snprintf(buff, sizeof(buff), "%s", "NG");

	set_default_result(info);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));

	if (strncmp(buff, "OK", 2) == 0)
		info->cmd_state = 2;
	else
		info->cmd_state = 3;

	dev_info(&info->client->dev, "%s: %s\n", __func__, buff);

}
/*
static void module_off_slave(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;

	not_support_cmd(info);
}

static void module_on_slave(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;

	not_support_cmd(info);
}
*/
static void get_chip_vendor(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;

	char buff[16] = {0};

	set_default_result(info);

	snprintf(buff, sizeof(buff), "%s", "MELFAS");
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}

static void get_chip_name(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;

	char buff[16] = {0};

	set_default_result(info);

	snprintf(buff, sizeof(buff), "%s", "MMS152");
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}

static void get_reference(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;

	char buff[16] = {0};
	unsigned int val;
	int node;

	set_default_result(info);
	node = check_rx_tx_num(info);

	if (node < 0)
		return;

	val = info->reference[node];
	snprintf(buff, sizeof(buff), "%u", val);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));

	info->cmd_state = 2;

	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));

}

static void get_cm_abs(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;

	char buff[16] = {0};
	unsigned int val;
	int node;

	set_default_result(info);
	node = check_rx_tx_num(info);

	if (node < 0)
		return;

	val = info->raw[node];
	snprintf(buff, sizeof(buff), "%u", val);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;

	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__, buff,
			strnlen(buff, sizeof(buff)));
}

static void get_cm_delta(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;

	char buff[16] = {0};
	unsigned int val;
	int node;

	set_default_result(info);
	node = check_rx_tx_num(info);

	if (node < 0)
		return;

	val = info->inspection[node];
	snprintf(buff, sizeof(buff), "%u", val);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;

	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__, buff,
			strnlen(buff, sizeof(buff)));
}

static void get_intensity(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;

	char buff[16] = {0};
	unsigned int val;
	int node;

	set_default_result(info);
	node = check_rx_tx_num(info);

	if (node < 0)
		return;

	val = info->intensity[node];

	snprintf(buff, sizeof(buff), "%u", val);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;

	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__, buff,
			strnlen(buff, sizeof(buff)));
}

static void get_x_num(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;

	char buff[16] = {0};
	int val;
	u8 r_buf[2];
	int ret;

	set_default_result(info);

	if (info->fw_core_ver == 0x45) {
		val = i2c_smbus_read_byte_data(info->client, 0xEF);
		if (val < 0) {
			snprintf(buff, sizeof(buff), "%s", "NG");
			set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
			info->cmd_state = 3;

			dev_info(&info->client->dev,
				"%s: fail to read num of x (%d).\n",
				__func__, val);
			return;
		}
	} else if (info->fw_ic_ver < 0x29) {
		ret = i2c_smbus_read_i2c_block_data(info->client,
			ADDR_CH_NUM, 2, r_buf);
		val = r_buf[0];
		if (ret < 0) {
			snprintf(buff, sizeof(buff), "%s", "NG");
			set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
			info->cmd_state = 3;

			dev_info(&info->client->dev,
				"%s: fail to read num of x (%d).\n",
				__func__, val);
			return;
		}
	} else {
		val = 30;
	}
	snprintf(buff, sizeof(buff), "%u", val);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;

	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__, buff,
			strnlen(buff, sizeof(buff)));
}

static void get_y_num(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;

	char buff[16] = {0};
	int val;
	u8 r_buf[2];
	int ret;

	set_default_result(info);

	if (info->fw_core_ver == 0x45) {
		val = i2c_smbus_read_byte_data(info->client, 0xEE);
		if (val < 0) {
			snprintf(buff, sizeof(buff), "%s", "NG");
			set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
			info->cmd_state = 3;

			dev_info(&info->client->dev,
				"%s: fail to read num of y (%d).\n",
				__func__, val);
			return;
		}
	} else if (info->fw_ic_ver < 0x29) {
		ret = i2c_smbus_read_i2c_block_data(info->client,
			ADDR_CH_NUM, 2, r_buf);
		val = r_buf[1];
		if (ret < 0) {
			snprintf(buff, sizeof(buff), "%s", "NG");
			set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
			info->cmd_state = 3;

			dev_info(&info->client->dev,
				"%s: fail to read num of x (%d).\n",
				__func__, val);
			return;
		}
	} else {
		val = 17;
	}
	snprintf(buff, sizeof(buff), "%u", val);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;

	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__, buff,
			strnlen(buff, sizeof(buff)));
}

static void run_reference_read(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;

	set_default_result(info);
	if (info->fw_ic_ver == 0x45)
		get_raw_data_all(info, MMS_VSC_CMD_REFER);
	else
		get_raw_data(info, MMS_VSC_CMD_REFER);
	info->cmd_state = 2;

/*	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__); */
}

static void run_cm_abs_read(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;

	set_default_result(info);
	if (info->fw_ic_ver == 0x45) {
		get_raw_data_all(info, MMS_VSC_CMD_CM_ABS);
		get_raw_data_all(info, MMS_VSC_CMD_EXIT);
	} else {
		get_raw_data(info, MMS_VSC_CMD_CM_ABS);
	}
	info->cmd_state = 2;

/*	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__); */
}

static void run_cm_delta_read(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;

	set_default_result(info);
	if (info->fw_ic_ver == 0x45) {
		get_raw_data_all(info, MMS_VSC_CMD_CM_DELTA);
		get_raw_data_all(info, MMS_VSC_CMD_EXIT);
	} else {
		get_raw_data(info, MMS_VSC_CMD_CM_DELTA);
	}
	info->cmd_state = 2;

/*	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__); */
}

static void run_intensity_read(void *device_data)
{
	struct mms_ts_info *info = (struct mms_ts_info *)device_data;

	set_default_result(info);
	if (info->fw_ic_ver == 0x45)
		get_raw_data_all(info, MMS_VSC_CMD_INTENSITY);
	else
		get_intensity_data(info);
	info->cmd_state = 2;

/*	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__); */
}

static ssize_t store_cmd(struct device *dev, struct device_attribute
				  *devattr, const char *buf, size_t count)
{
	struct mms_ts_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->client;

	char *cur, *start, *end;
	char buff[TSP_CMD_STR_LEN] = {0};
	int len, i;
	struct tsp_cmd *tsp_cmd_ptr = NULL;
	char delim = ',';
	bool cmd_found = false;
	int param_cnt = 0;
	int ret;

	if (info->cmd_is_running == true) {
		dev_err(&info->client->dev, "tsp_cmd: other cmd is running.\n");
		goto err_out;
	}


	/* check lock  */
	mutex_lock(&info->cmd_lock);
	info->cmd_is_running = true;
	mutex_unlock(&info->cmd_lock);

	info->cmd_state = 1;

	for (i = 0; i < ARRAY_SIZE(info->cmd_param); i++)
		info->cmd_param[i] = 0;

	len = (int)count;
	if (*(buf + len - 1) == '\n')
		len--;
	memset(info->cmd, 0x00, ARRAY_SIZE(info->cmd));
	memcpy(info->cmd, buf, len);

	cur = strchr(buf, (int)delim);
	if (cur)
		memcpy(buff, buf, cur - buf);
	else
		memcpy(buff, buf, len);

	/* find command */
	list_for_each_entry(tsp_cmd_ptr, &info->cmd_list_head, list) {
		if (!strcmp(buff, tsp_cmd_ptr->cmd_name)) {
			cmd_found = true;
			break;
		}
	}

	/* set not_support_cmd */
	if (!cmd_found) {
		list_for_each_entry(tsp_cmd_ptr, &info->cmd_list_head, list) {
			if (!strcmp("not_support_cmd", tsp_cmd_ptr->cmd_name))
				break;
		}
	}

	/* parsing parameters */
	if (cur && cmd_found) {
		cur++;
		start = cur;
		memset(buff, 0x00, ARRAY_SIZE(buff));
		do {
			if (*cur == delim || cur - buf == len) {
				end = cur;
				memcpy(buff, start, end - start);
				*(buff + strlen(buff)) = '\0';
				ret = kstrtoint(buff, 10,\
						info->cmd_param + param_cnt);
				start = cur + 1;
				memset(buff, 0x00, ARRAY_SIZE(buff));
				param_cnt++;
			}
			cur++;
		} while (cur - buf <= len);
	}

	dev_info(&client->dev, "cmd = %s\n", tsp_cmd_ptr->cmd_name);
	for (i = 0; i < param_cnt; i++)
		dev_info(&client->dev, "cmd param %d= %d\n", i,
							info->cmd_param[i]);

	tsp_cmd_ptr->cmd_func(info);


err_out:
	return count;
}

static ssize_t show_cmd_status(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	struct mms_ts_info *info = dev_get_drvdata(dev);
	char buff[16] = {0};

	dev_info(&info->client->dev, "tsp cmd: status:%d\n",
			info->cmd_state);

	if (info->cmd_state == 0)
		snprintf(buff, sizeof(buff), "WAITING");

	else if (info->cmd_state == 1)
		snprintf(buff, sizeof(buff), "RUNNING");

	else if (info->cmd_state == 2)
		snprintf(buff, sizeof(buff), "OK");

	else if (info->cmd_state == 3)
		snprintf(buff, sizeof(buff), "FAIL");

	else if (info->cmd_state == 4)
		snprintf(buff, sizeof(buff), "NOT_APPLICABLE");

	return snprintf(buf, TSP_BUF_SIZE, "%s\n", buff);
}

static ssize_t show_cmd_result(struct device *dev, struct device_attribute
				    *devattr, char *buf)
{
	struct mms_ts_info *info = dev_get_drvdata(dev);

	dev_info(&info->client->dev, "tsp cmd: result: %s\n", info->cmd_result);

	mutex_lock(&info->cmd_lock);
	info->cmd_is_running = false;
	mutex_unlock(&info->cmd_lock);

	info->cmd_state = 0;

	return snprintf(buf, TSP_BUF_SIZE, "%s\n", info->cmd_result);
}

#ifdef ESD_DEBUG

static bool intensity_log_flag;

static u32 get_raw_data_one(struct mms_ts_info *info, u16 rx_idx, u16 tx_idx,
			    u8 cmd)
{
	u8 w_buf[6];
	u8 read_buffer[2];
	int ret;
	u32 raw_data;

	w_buf[0] = MMS_VSC_CMD;	/* vendor specific command id */
	w_buf[1] = MMS_VSC_MODE;	/* mode of vendor */
	w_buf[2] = 0;		/* tx line */
	w_buf[3] = 0;		/* rx line */
	w_buf[4] = 0;		/* reserved */
	w_buf[5] = 0;		/* sub command */

	if (cmd != MMS_VSC_CMD_INTENSITY && cmd != MMS_VSC_CMD_RAW &&
	    cmd != MMS_VSC_CMD_REFER) {
		dev_err(&info->client->dev, "%s: not profer command(cmd=%d)\n",
			__func__, cmd);
		return FAIL;
	}

	w_buf[2] = tx_idx;	/* tx */
	w_buf[3] = rx_idx;	/* rx */
	w_buf[5] = cmd;		/* sub command */

	ret = i2c_smbus_write_i2c_block_data(info->client, w_buf[0], 5,
					     &w_buf[1]);
	if (ret < 0)
		goto err_i2c;

	ret = i2c_smbus_read_i2c_block_data(info->client, 0xBF, 2, read_buffer);
	if (ret < 0)
		goto err_i2c;

	raw_data = ((u16) read_buffer[1] << 8) | read_buffer[0];
	if (cmd == MMS_VSC_CMD_REFER)
		raw_data = raw_data >> 4;

	return raw_data;

err_i2c:
	dev_err(&info->client->dev, "%s: fail to i2c (cmd=%d)\n",
		__func__, cmd);
	return FAIL;
}

static ssize_t show_intensity_logging_on(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	struct mms_ts_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->client;
	struct file *fp;
	char log_data[160] = { 0, };
	char buff[16] = { 0, };
	mm_segment_t old_fs;
	long nwrite;
	u32 val;
	int i, y, c;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

#define MELFAS_DEBUG_LOG_PATH "/sdcard/melfas_log"

	dev_info(&client->dev, "%s: start.\n", __func__);
	fp = filp_open(MELFAS_DEBUG_LOG_PATH, O_RDWR | O_CREAT,
		       S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR(fp)) {
		dev_err(&client->dev, "%s: fail to open log file\n", __func__);
		goto open_err;
	}

	intensity_log_flag = 1;
	do {
		for (y = 0; y < 3; y++) {
			/* for tx chanel 0~2 */
			memset(log_data, 0x00, 160);

			snprintf(buff, 16, "%1u: ", y);
			strncat(log_data, buff, strnlen(buff, 16));

			for (i = 0; i < RX_NUM; i++) {
				val = get_raw_data_one(info, i, y,
						       MMS_VSC_CMD_INTENSITY);
				snprintf(buff, 16, "%5u, ", val);
				strncat(log_data, buff, strnlen(buff, 16));
			}
			memset(buff, '\n', 2);
			c = (y == 2) ? 2 : 1;
			strncat(log_data, buff, c);
			nwrite = vfs_write(fp, (const char __user *)log_data,
					   strnlen(log_data, 160), &fp->f_pos);
		}
		usleep_range(3000, 5000);
	} while (intensity_log_flag);

	filp_close(fp, current->files);
	set_fs(old_fs);

	return 0;

open_err:
	set_fs(old_fs);
	return FAIL;
}

static ssize_t show_intensity_logging_off(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	struct mms_ts_info *info = dev_get_drvdata(dev);
	intensity_log_flag = 0;
	usleep_range(10000, 12000);
	get_raw_data_all(info, MMS_VSC_CMD_EXIT);
	return 0;
}

#endif

#ifdef CONFIG_CPU_FREQ_LCD_FREQ_DFS
static ssize_t enable_lcdfreq_touchboost_show(struct device *dev,
									   struct device_attribute *attr,
									   char *buf)
{
	sprintf(buf, "%d\n", flg_enable_lcdfreq_touchboost ? 1 : 0);
	return strlen(buf);
}

static ssize_t enable_lcdfreq_touchboost_store(struct device *dev,
                                               struct device_attribute *attr,
                                               char *buf, size_t size)
{
	if (!strncmp(buf, "1", 1)) {
		flg_enable_lcdfreq_touchboost = 1;
	} else if (!strncmp(buf, "0", 1)) {
		flg_enable_lcdfreq_touchboost = 0;
	}
	return size;
}
#endif

static ssize_t longpressoff_alwayson_show(struct device *dev, struct device_attribute *attr, char *buf) {
	
	return sprintf(buf, "%u\n", sttg_longpressoff_alwayson);
}

static ssize_t longpressoff_alwayson_store(struct device *dev, struct device_attribute *attr, char *buf, size_t size) {
	
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if(ret && data == 1) {
		sttg_longpressoff_alwayson = true;
		pr_info("TOUCHWAKE longpressoff_alwayson has been set\n");
	} else {
		sttg_longpressoff_alwayson = false;
		pr_info("TOUCHWAKE longpressoff_alwayson has been unset\n");
	}
	
	return size;
}

static ssize_t typingbooster_mincores_show(struct device *dev, struct device_attribute *attr, char *buf) {
	
	return sprintf(buf, "%u\n", sttg_typingbooster_mincores);
}

static ssize_t typingbooster_mincores_store(struct device *dev, struct device_attribute *attr, char *buf, size_t size) {
	
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if (ret != 1) {
		return -EINVAL;
	}
	
	if (data == 0) {
		sttg_typingbooster_mincores = 0;
		pr_info("[tsp/typingbooster] typingbooster_mincores has been disabled\n");
	} else if (data > 0 && data <= 4) {
		sttg_typingbooster_mincores = data;
		pr_info("[tsp/typingbooster] typingbooster_mincores is enabled and has been set to: %d\n", data);
	} else {
		return -EINVAL;
	}
	
	return size;
}

static ssize_t typingbooster_upthreshold_show(struct device *dev, struct device_attribute *attr, char *buf) {
	
	return sprintf(buf, "%u\n", sttg_typingbooster_upthreshold);
}

static ssize_t typingbooster_upthreshold_store(struct device *dev, struct device_attribute *attr, char *buf, size_t size) {
	
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if (ret != 1) {
		return -EINVAL;
	}
	
	if (data == 0) {
		sttg_typingbooster_upthreshold = 0;
		pr_info("[tsp/typingbooster] typingbooster_upthreshold has been disabled\n");
	} else if (data < 11) {
		sttg_typingbooster_upthreshold = 11;
		pr_info("[tsp/typingbooster] typingbooster_upthreshold is enabled and has been set to: %d\n", data);
	} else if (data <= 100) {
		sttg_typingbooster_upthreshold = data;
		pr_info("[tsp/typingbooster] typingbooster_upthreshold is enabled and has been set to: %d\n", data);
	} else {
		return -EINVAL;
	}
	
	return size;
}

static ssize_t typingbooster_cycles_show(struct device *dev, struct device_attribute *attr, char *buf) {
	
	return sprintf(buf, "%u\n", sttg_typingbooster_cycles);
}

static ssize_t typingbooster_cycles_store(struct device *dev, struct device_attribute *attr, char *buf, size_t size) {
	
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if (ret != 1) {
		return -EINVAL;
	}
	
	if (data > 0 && data <= 100) {
		sttg_typingbooster_cycles = data;
		pr_info("[tsp/typingbooster] typingbooster_cycles has been set to: %d\n", data);
	} else {
		return -EINVAL;
	}
	
	return size;
}

static ssize_t typingbooster_mintaps_show(struct device *dev, struct device_attribute *attr, char *buf) {
	
	return sprintf(buf, "%u\n", sttg_typingbooster_mintaps);
}

static ssize_t typingbooster_mintaps_store(struct device *dev, struct device_attribute *attr, char *buf, size_t size) {
	
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if (ret != 1) {
		return -EINVAL;
	}
	
	if (data > 2 && data <= 20) {
		sttg_typingbooster_mintaps = data;
		pr_info("[tsp/typingbooster] typingbooster_mintaps has been set to: %d\n", data);
	} else {
		return -EINVAL;
	}
	
	return size;
}

static ssize_t typingbooster_maxmsbetweentaps_show(struct device *dev, struct device_attribute *attr, char *buf) {
	
	return sprintf(buf, "%u\n", sttg_typingbooster_maxmsbetweentaps);
}

static ssize_t typingbooster_maxmsbetweentaps_store(struct device *dev, struct device_attribute *attr, char *buf, size_t size) {
	
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if (ret != 1) {
		return -EINVAL;
	}
	
	if (data >= 50 && data <= 1000) {
		sttg_typingbooster_maxmsbetweentaps = data;
		pr_info("[tsp/typingbooster] typingbooster_maxmsbetweentaps has been set to: %d\n", data);
	} else {
		return -EINVAL;
	}
	
	return size;
}

static ssize_t touchbooster_enabled_show(struct device *dev, struct device_attribute *attr, char *buf) {
	
	return sprintf(buf, "%u\n", sttg_touchbooster_enabled);
}

static ssize_t touchbooster_enabled_store(struct device *dev, struct device_attribute *attr, char *buf, size_t size) {
	
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if(ret && data == 1) {
		sttg_touchbooster_enabled = true;
		pr_info("[tsp] sttg_touchbooster_enabled has been set\n");
	} else {
		sttg_touchbooster_enabled = false;
		pr_info("[tsp] sttg_touchbooster_enabled has been unset\n");
	}
	
	return size;
}

static ssize_t touchbooster_freq_show(struct device *dev, struct device_attribute *attr, char *buf) {
	
	return sprintf(buf, "%u\n", sttg_touchbooster_freq);
}

static ssize_t touchbooster_freq_store(struct device *dev, struct device_attribute *attr, char *buf, size_t size) {
	
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if(ret && data >= 0) {
		sttg_touchbooster_freq = data;
		pr_info("[tsp] sttg_touchbooster_freq has been set to: %d\n", data);
	}
	
	return size;
}

static ssize_t touchbooster_duration_show(struct device *dev, struct device_attribute *attr, char *buf) {
	
	return sprintf(buf, "%u\n", sttg_touchbooster_duration);
}

static ssize_t touchbooster_duration_store(struct device *dev, struct device_attribute *attr, char *buf, size_t size) {
	
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if(ret) {
		if (data == 0) {
			data = 1;
		}
		sttg_touchbooster_duration = data;
		pr_info("[tsp] sttg_touchbooster_duration has been set to: %d\n", data);
	}
	
	return size;
}

static ssize_t touchbooster_relax_delay_show(struct device *dev, struct device_attribute *attr, char *buf) {
	
	return sprintf(buf, "%u\n", sttg_touchbooster_relax_delay);
}

static ssize_t touchbooster_relax_delay_store(struct device *dev, struct device_attribute *attr, char *buf, size_t size) {
	
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if(ret) {
		if (data == 0) {
			data = 1;
		}
		sttg_touchbooster_relax_delay = data;
		pr_info("[tsp] sttg_touchbooster_relax_delay has been set to: %d\n", data);
	}
	
	return size;
}

static ssize_t touchbooster_relax_freq_show(struct device *dev, struct device_attribute *attr, char *buf) {
	
	return sprintf(buf, "%u\n", sttg_touchbooster_relax_freq);
}

static ssize_t touchbooster_relax_freq_store(struct device *dev, struct device_attribute *attr, char *buf, size_t size) {
	
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if(ret && data >= 0) {
		sttg_touchbooster_relax_freq = data;
		pr_info("[tsp] sttg_touchbooster_relax_freq has been set to: %d\n", data);
	}
	
	return size;
}

static ssize_t touchbooster_mincores_show(struct device *dev, struct device_attribute *attr, char *buf) {
	
	return sprintf(buf, "%u\n", sttg_touchbooster_mincores);
}

static ssize_t touchbooster_mincores_store(struct device *dev, struct device_attribute *attr, char *buf, size_t size) {
	
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if(ret) {
		if (data < 0)
			data = 0;
		
		if (data > 4)
			data = 4;
		
		sttg_touchbooster_mincores = data;
		pr_info("[tsp] sttg_touchbooster_mincores has been set to: %d\n", data);
	}
	
	return size;
}

static ssize_t touchbooster_relax_mincores_show(struct device *dev, struct device_attribute *attr, char *buf) {
	
	return sprintf(buf, "%u\n", sttg_touchbooster_relax_mincores);
}

static ssize_t touchbooster_relax_mincores_store(struct device *dev, struct device_attribute *attr, char *buf, size_t size) {
	
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if(ret) {
		if (data < 0)
			data = 0;
		
		if (data > 4)
			data = 4;
		
		sttg_touchbooster_relax_mincores = data;
		pr_info("[tsp] sttg_touchbooster_relax_mincores has been set to: %d\n", data);
	}
	
	return size;
}

#ifdef CONFIG_TOUCHSCREEN_GESTURES
// Must be called with the gestures_lock spinlock held
static void reset_gestures_detection_locked(bool including_detected)
{
	int gesture_no, finger_no;
	
	has_gestures = false;
	for (gesture_no = 0; gesture_no <= max_configured_gesture; gesture_no++) {
		if (gestures_detected[gesture_no] && !including_detected) {
			has_gestures = true;
			// Gesture already reported, skip
			continue;
		}
		
		gestures_detected[gesture_no] = false;
		
		if (gestures_step_count[gesture_no][0] < 1)
			continue; // Gesture not configured
		
		// Reset progress for all paths of this gesture
		for (finger_no = 0; finger_no <= max_gesture_finger[gesture_no]; finger_no++) {
			gesture_fingers[gesture_no][finger_no].finger_order = -1;
			gesture_fingers[gesture_no][finger_no].current_step = -1;
		}
	}
}

static void reset_gestures_detection(bool including_detected)
{
	unsigned long flags;
	
	spin_lock_irqsave(&gestures_lock, flags);
	if (gestures_enabled)
		reset_gestures_detection_locked(including_detected);
	spin_unlock_irqrestore(&gestures_lock, flags);
}

static ssize_t gesture_patterns_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
	char *s;
	int gesture_no, finger_no, step;
	struct gesture_point *point;
	
	s = buf;
	s += sprintf(s, "# Touch gestures\n#\n");
	s += sprintf(s, "# Syntax\n");
	s += sprintf(s, "#   <gesture_no>:<finger_no>:(x_min|x_max,y_min|y_max)\n");
	s += sprintf(s, "#   ...\n");
	s += sprintf(s, "#   gesture_no: 1 to %d\n", MAX_GESTURES);
	s += sprintf(s, "#   finger_no : 1 to %d\n", MAX_GESTURE_FINGERS);
	s += sprintf(s, "#   max steps per gesture and finger: %d\n\n", MAX_GESTURE_STEPS);
	
	// No special need for thread safety, at worst there might be incoherent definitions output
	for (gesture_no = 0; gesture_no <= max_configured_gesture; gesture_no++) {
		if (gestures_step_count[gesture_no][0] < 1)
			continue; // Gesture not configured
		s += sprintf(s, "# Gesture %d:\n", gesture_no+1);
		for (finger_no = 0; finger_no <= max_gesture_finger[gesture_no]; finger_no++) {
			for (step = 0; step < gestures_step_count[gesture_no][finger_no]; step++) {
				point = &gesture_points[gesture_no][finger_no][step];
				s += sprintf(s, "%d:%d:(%d|%d,%d|%d)\n", gesture_no+1, finger_no+1,
				             point->min_x, point->max_x,
				             point->min_y, point->max_y);
			}
		}
	}
	return strlen(buf);
}

static ssize_t gesture_patterns_store(struct device *dev,
									  struct device_attribute *attr,
									  const char *buf, size_t size)
{
	gesture_points_t *tmp_gesture_points;
	gestures_step_count_t *tmp_gestures_step_count;
	int highest_configured_gesture;
	int highest_gesture_finger[MAX_GESTURES];
	unsigned long flags;
	int res;
	int gesture_no, finger_no, min_x, max_x, min_y, max_y;
	struct gesture_point *point;
	int step;
	
	memset(ary_gestures_flg_touchkey, 0, sizeof(ary_gestures_flg_touchkey));
	memset(ary_gestures_flg_exclusive, 0, sizeof(ary_gestures_flg_exclusive));
	
	tmp_gesture_points = kmalloc(sizeof(*tmp_gesture_points), GFP_KERNEL);
	if (!tmp_gesture_points)
		return -ENOMEM;
	tmp_gestures_step_count = kmalloc(sizeof(*tmp_gestures_step_count), GFP_KERNEL);
	if (!tmp_gestures_step_count) {
		kfree(tmp_gesture_points);
		return -ENOMEM;
	}
	highest_configured_gesture = -1;
	for (gesture_no = 0; gesture_no < MAX_GESTURES; gesture_no++) {
		for (finger_no = 0; finger_no < MAX_GESTURE_FINGERS; finger_no++) {
			for (step = 0; step < MAX_GESTURE_STEPS; step++) {
				point = &(*tmp_gesture_points)[gesture_no][finger_no][step];
				point->min_x = 0;
				point->max_x = 0;
				point->min_y = 0;
				point->max_y = 0;
			}
			(*tmp_gestures_step_count)[gesture_no][finger_no] = 0;
		}
		highest_gesture_finger[gesture_no] = -1;
	}
	
	for (;;) {
		// Skip all whitespace and comment lines
		for (;;) {
			while (*buf == ' ' || *buf == '\t' || *buf == '\r' || *buf == '\n')
				buf++;
			
			if (*buf == '\0') {
				// EOF
				goto finalize;
			} else if (*buf == '#') {
				// Comment line
				buf = strstr(buf, "\n");
				if (!buf)
					goto finalize; // No more data
			} else {
				break;
			}
		}
		
		
		// parse gesture action data.
		// try the original-style (no flags) first.
		res = sscanf(buf, "%d:%d:(%d|%d,%d|%d)", &gesture_no, &finger_no, &min_x, &max_x, &min_y, &max_y);
		if (res != 6) {
			// the original-style string was not matched. see if the touchkey flag is set.
			res = sscanf(buf, "%d:%d:TKEYS:(%d|%d,%d|%d)", &gesture_no, &finger_no, &min_x, &max_x, &min_y, &max_y);
			if (res != 6) {
				// nope, no touchkey flags. try the exclusive flag.
				res = sscanf(buf, "%d:%d:EXCLUSIVE:(%d|%d,%d|%d)", &gesture_no, &finger_no, &min_x, &max_x, &min_y, &max_y);
				if (res != 6) {
					// nope, no exclusive flag either. try both.
					res = sscanf(buf, "%d:%d:EXCLUSIVE:TKEYS:(%d|%d,%d|%d)", &gesture_no, &finger_no, &min_x, &max_x, &min_y, &max_y);
					if (res != 6) {
						res = sscanf(buf, "%d:%d:TKEYS:EXCLUSIVE:(%d|%d,%d|%d)", &gesture_no, &finger_no, &min_x, &max_x, &min_y, &max_y);
						if (res != 6) {
							// well then, still no matches. let's give up.
							printk("[TSP/gestures/gesture_patterns_show] no valid gestures found in '%s'\n", buf);
							kfree(tmp_gestures_step_count);
							kfree(tmp_gesture_points);
							return -EINVAL; // Invalid line format
						} else {
							// success! this gesture has flags for both exclusive and touchkeys (order reversed).
							printk("[TSP/gestures/gesture_patterns_show] gesture #%d: found exclusive flag and touchkey flag\n", gesture_no);
							ary_gestures_flg_exclusive[gesture_no - 1] = 1;
							ary_gestures_flg_touchkey[gesture_no - 1] = 1;
						}
					} else {
						// success! this gesture has flags for both exclusive and touchkeys.
						printk("[TSP/gestures/gesture_patterns_show] gesture #%d: found exclusive flag and touchkey flag\n", gesture_no);
						ary_gestures_flg_exclusive[gesture_no - 1] = 1;
						ary_gestures_flg_touchkey[gesture_no - 1] = 1;
					}
				} else {
					// success! this gesture has flags for exclusive.
					printk("[TSP/gestures/gesture_patterns_show] gesture #%d: found exclusive flag\n", gesture_no);
					ary_gestures_flg_exclusive[gesture_no - 1] = 1;
					ary_gestures_flg_touchkey[gesture_no - 1] = 0;
				}
			} else {
				// success! this gesture has flags for touchkeys.
				printk("[TSP/gestures/gesture_patterns_show] gesture #%d: found touchkey flag\n", gesture_no);
				ary_gestures_flg_exclusive[gesture_no - 1] = 0;
				ary_gestures_flg_touchkey[gesture_no - 1] = 1;
			}
		} else {
			// success! no flags set.
			printk("[TSP/gestures/gesture_patterns_show] gesture #%d: found no flags\n", gesture_no);
			ary_gestures_flg_exclusive[gesture_no - 1] = 0;
			ary_gestures_flg_touchkey[gesture_no - 1] = 0;
		}
		
		// Validate args boundary
		if (gesture_no <= 0 || gesture_no > MAX_GESTURES) {
			kfree(tmp_gestures_step_count);
			kfree(tmp_gesture_points);
			return -ENOMEM; // Too many gestures
		}
		gesture_no--;
		if (gesture_no > highest_configured_gesture)
			highest_configured_gesture = gesture_no;
		if (finger_no <= 0 || finger_no > MAX_GESTURE_FINGERS) {
			kfree(tmp_gestures_step_count);
			kfree(tmp_gesture_points);
			return -ENOMEM; // Too many fingers
		}
		finger_no--;
		if (finger_no > highest_gesture_finger[gesture_no])
			highest_gesture_finger[gesture_no] = finger_no;
		if ((*tmp_gestures_step_count)[gesture_no][finger_no] >= MAX_GESTURE_STEPS) {
			kfree(tmp_gestures_step_count);
			kfree(tmp_gesture_points);
			return -ENOMEM; // Too many steps
		}
		
		step = (*tmp_gestures_step_count)[gesture_no][finger_no]++;
		point = &(*tmp_gesture_points)[gesture_no][finger_no][step];
		point->min_x = min_x;
		point->max_x = max_x;
		point->min_y = min_y;
		point->max_y = max_y;
		buf = strstr(buf, ")");
		if (!buf)
			goto finalize; // No more data
		else
			buf++;
		
		// Continue to next input contents
	}
	
finalize:
	spin_lock_irqsave(&gestures_lock, flags);
	reset_gestures_detection_locked(true);
	memcpy(&gesture_points, tmp_gesture_points, sizeof(*tmp_gesture_points));
	memcpy(&gestures_step_count, tmp_gestures_step_count, sizeof(*tmp_gestures_step_count));
	max_configured_gesture = highest_configured_gesture;
	memcpy(&max_gesture_finger, &(highest_gesture_finger[0]), sizeof(highest_gesture_finger));
	spin_unlock_irqrestore(&gestures_lock, flags);
	
	kfree(tmp_gestures_step_count);
	kfree(tmp_gesture_points);
	return size;
}

static ssize_t wait_for_gesture_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
	int ret;
	int gesture_no, finger_no;
	char *s;
	int detected_gesture;
	bool has_more_gestures;
	unsigned long flags;
	
	// Does not stop if there's a gesture already reported
	ret = wait_event_interruptible(gestures_wq,
								   has_gestures);
	if (ret)
		return ret; // Interrupted
	
	s = buf;
	spin_lock_irqsave(&gestures_lock, flags);
	for (gesture_no = 0; gesture_no <= max_configured_gesture; gesture_no++) {
		if (gestures_detected[gesture_no]) {
			detected_gesture = gesture_no;
			
			has_more_gestures = false;
			while (++gesture_no <= max_configured_gesture) {
				if (gestures_detected[gesture_no]) {
					has_more_gestures = true;
					break;
				}
			}
			// In most cases, additional waiting clients will not have run yet and will
			// keep waiting if this goes back to false
			has_gestures = has_more_gestures;
			
			// Reset detection of this gesture
			for (finger_no = 0; finger_no <= max_gesture_finger[detected_gesture]; finger_no++) {
				gesture_fingers[detected_gesture][finger_no].finger_order = -1;
				gesture_fingers[detected_gesture][finger_no].current_step = -1;
			}
			gestures_detected[detected_gesture] = false;
			
			spin_unlock_irqrestore(&gestures_lock, flags);
			
			s += sprintf(buf, "%d", detected_gesture + 1);
			return s - buf;
		}
	}
	// False-positive, avoid further wakeups until new gestures are detected
	has_gestures = false;
	spin_unlock_irqrestore(&gestures_lock, flags);
	
	// Despite waking up, no gestures were detected
	s += sprintf(buf, "0");
	return s - buf;
}

static ssize_t wait_for_gesture_store(struct device *dev,
                                      struct device_attribute *attr,
                                      const char *buf, size_t size)
{
	if (!strncmp(buf, "reset", 5)) {
		// Clear any pending gestures and reset detection
		reset_gestures_detection(true);
		return size;
		
	} else {
		return -EINVAL;
	}
}

static ssize_t gestures_enabled_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
	sprintf(buf, "%d\n", gestures_enabled ? 1 : 0);
	return strlen(buf);
}

static ssize_t gestures_enabled_store(struct device *dev,
                                      struct device_attribute *attr,
                                      const char *buf,
                                      size_t size)
{
	unsigned int data = -1;
	
	if (!strncmp(buf, "1", 1)) {
		data = 1;
		ignore_gestures = false;
	} else if (!strncmp(buf, "0", 1)) {
		data = 0;
		ignore_gestures = true;
	} else if (sscanf(buf, "%u\n", &data) != 1) {
		return -EINVAL;
	}
	
	if (data < 0 || data > 1)
		return -EINVAL;
	
	if (data == 0)
		reset_gestures_detection(true);
	gestures_enabled = (data == 1);
	
	return size;
}

static ssize_t gestures_require_touchkey_show(struct device *dev,
											struct device_attribute *attr, char *buf)
{
	sprintf(buf, "%d\n", sttg_require_touchkey);
	return strlen(buf);
}

static ssize_t gestures_require_touchkey_store(struct device *dev, struct device_attribute *attr,
											 const char *buf, size_t size)
{
	unsigned int input;
	int ret;
	
	ret = sscanf(buf, "%u", &input);
	
	if (ret != 1 && (input < 0 || input > 1)) {
		return -EINVAL;
	}
	
	if (input == 0) {
		// normal.
		sttg_require_touchkey = false;
		pr_info("[TSP/gestures] sttg_require_touchkey=off\n");
	} else {
		// require the touckey to be pressed.
		sttg_require_touchkey = true;
		pr_info("[TSP/gestures] sttg_require_touchkey=on\n");
	}
	
	return size;
}

#if GESTURE_BOOSTER
static ssize_t gesturebooster_enabled_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
	sprintf(buf, "%d\n", gesturebooster_enabled ? 1 : 0);
	return strlen(buf);
}

static ssize_t gesturebooster_enabled_store(struct device *dev, struct device_attribute *attr,
                                      const char *buf, size_t size)
{
	unsigned int input;
	int ret;
	
	ret = sscanf(buf, "%u", &input);
	
	if (ret != 1 && (input > 1 || input < 0)) {
		return -EINVAL;
	}
	
	if (input == 0) {
		gesturebooster_enabled = 0;
		pr_info("[TSP/gesturebooster] gesturebooster disabled\n");
	} else if (input == 1) {
		gesturebooster_enabled = 1;
		pr_info("[TSP/gesturebooster] gesturebooster enabled\n");
	}
	
	return size;
}

static ssize_t gesturebooster_freq_show(struct device *dev,
										   struct device_attribute *attr, char *buf)
{
	sprintf(buf, "%d\n", gesturebooster_freq);
	return strlen(buf);
}

static ssize_t gesturebooster_freq_store(struct device *dev, struct device_attribute *attr,
											const char *buf, size_t size)
{
	unsigned int input;
	int ret;
	unsigned int maxfreq = 0;
	unsigned int minfreq = 0;
	
	ret = sscanf(buf, "%u", &input);
	
	if (ret != 1) {
		return -EINVAL;
	}
	
	// March 3rd 2014 - removed old reference to exynos_cpufreq_get_maxfreq to prevent possible exception
	maxfreq = 1600000;
	minfreq = exynos_cpufreq_get_minfreq();
	
	if (maxfreq < 1) {
		// something went wrong. just set a safe value.
		maxfreq = 1000000;
	}
	
	if (minfreq < 1) {
		// something went wrong. just set a safe value.
		minfreq = 200000;
	}
	
	if (input == 0) {
		// set to default.
		gesturebooster_freq = GESTURE_BOOSTER_FREQ;
	} else if (input < minfreq) {
		gesturebooster_freq = minfreq;
		pr_info("[TSP/gesturebooster] boost_freq too low. set to: %d\n", minfreq);
	} else if (input > maxfreq) {
		gesturebooster_freq = maxfreq;
		pr_info("[TSP/gesturebooster] boost_freq too high. set to: %d\n", maxfreq);
	} else {
		gesturebooster_freq = input;
		pr_info("[TSP/gesturebooster] boost_freq set to: %d\n", input);
	}
	
	// reset the level so it can be set later with the new freq.
	gesturebooster_cpufreq_level = -1;
	
	return size;
}

static ssize_t gesturebooster_duration_show(struct device *dev,
										   struct device_attribute *attr, char *buf)
{
	sprintf(buf, "%d\n", gesturebooster_duration);
	return strlen(buf);
}

static ssize_t gesturebooster_duration_store(struct device *dev, struct device_attribute *attr,
											const char *buf, size_t size)
{
	unsigned int input;
	int ret;
	
	ret = sscanf(buf, "%u", &input);
	
	if (ret != 1 && (input < 0 || input > 20000)) {
		return -EINVAL;
	}
	
	if (input == 0) {
		// set to default.
		gesturebooster_duration = GESTURE_BOOSTER_DURATION;
		pr_info("[TSP/gesturebooster] gesturebooster duration set to default (%d)\n", GESTURE_BOOSTER_DURATION);
	} else {
		gesturebooster_duration = input;
		pr_info("[TSP/gesturebooster] gesturebooster duration set to: %d\n", input);
	}
	
	return size;
}
#endif

/*
 TODO:
 static ssize_t lcdfreq_tsp_mode_show(struct device *dev,
										   struct device_attribute *attr, char *buf)
{
	sprintf(buf, "%d\n", 0);
	return strlen(buf);
}

static ssize_t lcdfreq_tsp_mode_store(struct device *dev, struct device_attribute *attr,
											const char *buf, size_t size)
{
	struct mms_ts_info *info = dev_get_drvdata(dev);
	unsigned int input;
	int ret;
	
	ret = sscanf(buf, "%u", &input);
	
	if (ret != 1 || input > 1 || input < 0) {
		return -EINVAL;
	}
	
	if (input == 0) {
		melfas_lcd_cb();
		pr_info("[TSP/lcdfreq] tsp mode: off\n");
	} else if (input == 1) {
		melfas_lcd_cb();
		pr_info("[TSP/lcdfreq] tsp mode: on\n");
	}
	
	return size;
}*/


static DEVICE_ATTR(gesture_patterns, S_IRUGO | S_IWUSR,
                   gesture_patterns_show, gesture_patterns_store);
static DEVICE_ATTR(wait_for_gesture, S_IRUGO | S_IWUSR,
                   wait_for_gesture_show, wait_for_gesture_store);
static DEVICE_ATTR(gestures_enabled, S_IRUGO | S_IWUSR,
                   gestures_enabled_show, gestures_enabled_store);
static DEVICE_ATTR(gestures_require_touchkey, S_IRUGO | S_IWUSR,
                   gestures_require_touchkey_show, gestures_require_touchkey_store);
#if GESTURE_BOOSTER
static DEVICE_ATTR(gesturebooster_enabled, S_IRUGO | S_IWUSR,
                   gesturebooster_enabled_show, gesturebooster_enabled_store);
static DEVICE_ATTR(gesturebooster_freq, S_IRUGO | S_IWUSR,
                   gesturebooster_freq_show, gesturebooster_freq_store);
static DEVICE_ATTR(gesturebooster_duration, S_IRUGO | S_IWUSR,
                   gesturebooster_duration_show, gesturebooster_duration_store);
#endif

static struct attribute *gestures_attrs[] = {
	&dev_attr_gesture_patterns.attr,
	&dev_attr_wait_for_gesture.attr,
	&dev_attr_gestures_enabled.attr,
	&dev_attr_gestures_require_touchkey.attr,
#if GESTURE_BOOSTER
	&dev_attr_gesturebooster_enabled.attr,
	&dev_attr_gesturebooster_freq.attr,
	&dev_attr_gesturebooster_duration.attr,
#endif
	NULL
};

static const struct attribute_group gestures_attr_group = {
	.attrs = gestures_attrs,
};

static struct miscdevice gestures_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = "touch_gestures",
};
static bool gestures_device_registered = false;
#endif

static DEVICE_ATTR(close_tsp_test, S_IRUGO, show_close_tsp_test, NULL);
static DEVICE_ATTR(cmd, S_IWUSR | S_IWGRP, NULL, store_cmd);
static DEVICE_ATTR(cmd_status, S_IRUGO, show_cmd_status, NULL);
static DEVICE_ATTR(cmd_result, S_IRUGO, show_cmd_result, NULL);
static DEVICE_ATTR(longpressoff_alwayson, S_IRUGO | S_IWUSR, longpressoff_alwayson_show, longpressoff_alwayson_store);
static DEVICE_ATTR(typingbooster_mincores, S_IRUGO | S_IWUSR, typingbooster_mincores_show, typingbooster_mincores_store);
static DEVICE_ATTR(typingbooster_upthreshold, S_IRUGO | S_IWUSR, typingbooster_upthreshold_show, typingbooster_upthreshold_store);
static DEVICE_ATTR(typingbooster_cycles, S_IRUGO | S_IWUSR, typingbooster_cycles_show, typingbooster_cycles_store);
static DEVICE_ATTR(typingbooster_maxmsbetweentaps, S_IRUGO | S_IWUSR, typingbooster_maxmsbetweentaps_show, typingbooster_maxmsbetweentaps_store);
static DEVICE_ATTR(typingbooster_mintaps, S_IRUGO | S_IWUSR, typingbooster_mintaps_show, typingbooster_mintaps_store);
static DEVICE_ATTR(touchbooster_enabled, S_IRUGO | S_IWUSR, touchbooster_enabled_show, touchbooster_enabled_store);
static DEVICE_ATTR(touchbooster_freq, S_IRUGO | S_IWUSR, touchbooster_freq_show, touchbooster_freq_store);
static DEVICE_ATTR(touchbooster_duration, S_IRUGO | S_IWUSR, touchbooster_duration_show, touchbooster_duration_store);
static DEVICE_ATTR(touchbooster_relax_delay, S_IRUGO | S_IWUSR, touchbooster_relax_delay_show, touchbooster_relax_delay_store);
static DEVICE_ATTR(touchbooster_relax_freq, S_IRUGO | S_IWUSR, touchbooster_relax_freq_show, touchbooster_relax_freq_store);
static DEVICE_ATTR(touchbooster_mincores, S_IRUGO | S_IWUSR, touchbooster_mincores_show, touchbooster_mincores_store);
static DEVICE_ATTR(touchbooster_relax_mincores, S_IRUGO | S_IWUSR, touchbooster_relax_mincores_show, touchbooster_relax_mincores_store);
#ifdef CONFIG_CPU_FREQ_LCD_FREQ_DFS
static DEVICE_ATTR(enable_lcdfreq_touchboost, S_IRUGO | S_IWUSR,
				   enable_lcdfreq_touchboost_show, enable_lcdfreq_touchboost_store);
#endif
//static DEVICE_ATTR(lcdfreq_tsp_mode, S_IRUGO | S_IWUSR, lcdfreq_tsp_mode_show, lcdfreq_tsp_mode_store); // todo
#ifdef ESD_DEBUG
static DEVICE_ATTR(intensity_logging_on, S_IRUGO, show_intensity_logging_on,
		   NULL);
static DEVICE_ATTR(intensity_logging_off, S_IRUGO, show_intensity_logging_off,
		   NULL);
#endif

static struct attribute *sec_touch_factory_attributes[] = {
	&dev_attr_close_tsp_test.attr,
	&dev_attr_cmd.attr,
	&dev_attr_cmd_status.attr,
	&dev_attr_cmd_result.attr,
    &dev_attr_longpressoff_alwayson.attr,
	&dev_attr_typingbooster_mincores.attr,
	&dev_attr_typingbooster_upthreshold.attr,
	&dev_attr_typingbooster_cycles.attr,
	&dev_attr_typingbooster_maxmsbetweentaps.attr,
	&dev_attr_typingbooster_mintaps.attr,
	&dev_attr_touchbooster_enabled.attr,
	&dev_attr_touchbooster_freq.attr,
	&dev_attr_touchbooster_duration.attr,
	&dev_attr_touchbooster_relax_delay.attr,
	&dev_attr_touchbooster_relax_freq.attr,
	&dev_attr_touchbooster_mincores.attr,
	&dev_attr_touchbooster_relax_mincores.attr,
#ifdef CONFIG_CPU_FREQ_LCD_FREQ_DFS
	&dev_attr_enable_lcdfreq_touchboost.attr,
#endif
//	&dev_attr_lcdfreq_tsp_mode.attr, // todo
#ifdef ESD_DEBUG
	&dev_attr_intensity_logging_on.attr,
	&dev_attr_intensity_logging_off.attr,
#endif
	NULL,
};

static struct attribute_group sec_touch_factory_attr_group = {
	.attrs = sec_touch_factory_attributes,
};
#endif /* SEC_TSP_FACTORY_TEST */

#if defined(CONFIG_PM) || defined(CONFIG_HAS_EARLYSUSPEND)
static int mms_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mms_ts_info *info = i2c_get_clientdata(client);
	
#ifdef CONFIG_TOUCHSCREEN_GESTURES
	reset_gestures_detection(false);
#endif

	if (!info->enabled) {
#ifdef CONFIG_INPUT_FBSUSPEND
		info->was_enabled_at_suspend = false;
#endif
		return 0;
	}

#ifdef CONFIG_INPUT_FBSUSPEND
	info->was_enabled_at_suspend = true;
#endif
	dev_notice(&info->client->dev, "%s: users=%d\n", __func__,
		   info->input_dev->users);

	disable_irq(info->irq);
	info->enabled = false;
	touch_is_pressed = 0;
#ifdef CONFIG_LCD_FREQ_SWITCH
	info->tsp_lcdfreq_flag = 0;
#endif
	release_all_fingers(info);
	info->pdata->power(0);
	info->sleep_wakeup_ta_check = info->ta_status;
	/* This delay needs to prevent unstable POR by
	rapid frequently pressing of PWR key. */
	msleep(50);
	return 0;
}

static int mms_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mms_ts_info *info = i2c_get_clientdata(client);

	if (info->enabled)
		return 0;

#ifdef CONFIG_INPUT_FBSUSPEND
	if (!info->was_enabled_at_suspend)
		return 0;
#endif
	dev_notice(&info->client->dev, "%s: users=%d\n", __func__,
		   info->input_dev->users);
	info->pdata->power(1);
	msleep(120);

	if (info->fw_ic_ver < 0x18) {
		if (info->ta_status) {
			dev_notice(&client->dev, "TA connect!!!\n");
			i2c_smbus_write_byte_data(info->client, 0x33, 0x1);
		} else {
			dev_notice(&client->dev, "TA disconnect!!!\n");
			i2c_smbus_write_byte_data(info->client, 0x33, 0x2);
		}
	}
	info->enabled = true;
	mms_set_noise_mode(info);

	if (info->fw_ic_ver >= 0x21) {
		if ((info->ta_status == 1) &&
			(info->sleep_wakeup_ta_check == 0)) {
			dev_notice(&client->dev,
				"TA connect!!! %s\n", __func__);
			i2c_smbus_write_byte_data(info->client, 0x32, 0x1);
		}
	}

	/* Because irq_type by EXT_INTxCON register is changed to low_level
	 *  after wakeup, irq_type set to falling edge interrupt again.
	 */
	enable_irq(info->irq);

	return 0;
}
#endif

#ifdef CONFIG_TOUCH_WAKE
static struct mms_ts_info * touchwake_data;
void touchscreen_disable(void)
{
	if (likely(touchwake_data != NULL))
    mms_ts_suspend(&touchwake_data->client->dev);
	
    return;
}
EXPORT_SYMBOL(touchscreen_disable);

void touchscreen_enable(void)
{
	if (likely(touchwake_data != NULL))
    mms_ts_resume(&touchwake_data->client->dev);
    
    flg_touchwake_active = false;
	
    return;
}
EXPORT_SYMBOL(touchscreen_enable);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void mms_ts_early_suspend(struct early_suspend *h)
{
	
	struct mms_ts_info *info;
	info = container_of(h, struct mms_ts_info, early_suspend);
	
    flg_screen_on = false;
    sttg_gestures_only = false;
    flg_touchwake_slide2wake = true;
    wake_start = -1;
    flg_touchwake_longpressoff = false;
    touchwake_longpressoff_time_start_secs = -1;
    flg_touchkey_was_pressed = false;
    flg_swipe_in_progress = false;
	ctr_typingbooster = 0;
	flg_ctr_typingbooster_cycles = 0;
	swipe_x_start = 9999;
	swipe_y_start = 9999;
	release_all_fingers(info);
	pr_info("[TSP/touch] -SUSPEND- cleared variables and fingers\n");
    
    cancel_delayed_work_sync(&touchwake_reset_longpressoff_work);
    cancel_delayed_work_sync(&touchwake_check_longpressoff_work);
    
    // unlock busfreqs just in case.
    dev_unlock(bus_dev, sec_touchscreen);
    bus_dvfs_lock_status = false;
    
    // unlock the cpu just in case.
    exynos_cpufreq_lock_free(DVFS_LOCK_ID_GESTURE_BOOSTER);
	gesturebooster_dvfs_locked = 0;
	
	cancel_delayed_work_sync(&work_touchbooster_relax);
	exynos_cpufreq_lock_free(DVFS_LOCK_ID_TSP);
	touchbooster_dvfs_locked = 0;
	
	zzmoove_touchbooster_mincores(0);
    
    flg_ignore_cpuidle = false;
    
#ifdef CONFIG_TOUCHSCREEN_GESTURES
	reset_gestures_detection(false);
#endif
    
#ifndef CONFIG_TOUCH_WAKE
	mms_ts_suspend(&info->client->dev);
#endif
}

static void mms_ts_late_resume(struct early_suspend *h)
{
    flg_screen_on = true;
    sttg_gestures_only = false;
    flg_touchwake_slide2wake = true;
    wake_start = -1;
    touchwake_longpressoff_time_start_secs = -1;
    flg_touchkey_was_pressed = false;
	swipe_x_start = 9999;
	swipe_y_start = 9999;
    
    pr_info("[TSP/touch] -WAKE- initialized variables\n");
    
    if (sttg_longpressoff_alwayson) {
        // only set the longpressoff flag if the user has requested it always be on.
        
        pr_info("[TSP/Touch] -WAKE- longpressoff always on\n");
        
        // set the flag.
        flg_touchwake_longpressoff = true;
    }
	
	touchscreen_enable();
	pr_info("[TSP/touch] -WAKE- forced touchscreen enabled\n");
	
#ifndef CONFIG_TOUCH_WAKE
	struct mms_ts_info *info;
	info = container_of(h, struct mms_ts_info, early_suspend);
	mms_ts_resume(&info->client->dev);
#endif
}
#endif

static int __devinit mms_ts_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct mms_ts_info *info;
	struct input_dev *input_dev;
	int ret = 0;
	char buf[4] = { 0, };
    struct gesture_point *point;

#ifdef SEC_TSP_FACTORY_TEST
	int i;
	struct device *fac_dev_ts;
#endif
	touch_is_pressed = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -EIO;

	info = kzalloc(sizeof(struct mms_ts_info), GFP_KERNEL);
	if (!info) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		ret = -ENOMEM;
		goto err_alloc;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&client->dev, "Failed to allocate memory for input device\n");
		ret = -ENOMEM;
		goto err_input_alloc;
	}

	info->client = client;
	info->input_dev = input_dev;
	info->pdata = client->dev.platform_data;
	if (NULL == info->pdata) {
		pr_err("failed to get platform data\n");
		goto err_config;
	}
	info->irq = -1;
	mutex_init(&info->lock);
    
    spin_lock_init(&touchwake_longpressoff_lock);
	
#ifdef CONFIG_TOUCHSCREEN_GESTURES
	spin_lock_init(&gestures_lock);
#endif

	if (info->pdata) {
		info->max_x = info->pdata->max_x;
		info->max_y = info->pdata->max_y;
		info->invert_x = info->pdata->invert_x;
		info->invert_y = info->pdata->invert_y;
		info->config_fw_version = info->pdata->config_fw_version;
		info->lcd_type = info->pdata->lcd_type;
		info->input_event = info->pdata->input_event;
		info->register_cb = info->pdata->register_cb;
#ifdef CONFIG_LCD_FREQ_SWITCH
		info->register_lcd_cb = info->pdata->register_lcd_cb;
#endif
	} else {
		info->max_x = 720;
		info->max_y = 1280;
	}

	i2c_set_clientdata(client, info);

	info->pdata->power(true);
	msleep(250);

	if (gpio_get_value(GPIO_OLED_ID)) {
		info->ldi = 'L';
		dev_info(&client->dev, "LSI LDI\n");
	} else {
		info->ldi = 'M';
		dev_info(&client->dev, "Magna LDI\n");
	}

	ret = i2c_master_recv(client, buf, 1);
	if (ret < 0) {		/* tsp connect check */
		pr_err("%s: i2c fail...tsp driver unload [%d], Add[%d]\n",
			   __func__, ret, info->client->addr);
		goto err_config;
	}

	info->fw_core_ver = get_fw_version(info, SEC_CORE);
	dev_info(&client->dev, "core version : 0x%02x\n",
		info->fw_core_ver);

	if (info->fw_core_ver == 0x50) {
		dev_err(&client->dev, "Do not use 0x50 core version\n");
		dev_err(&client->dev, "excute core firmware update\n");
		ret = mms_ts_core_fw_load(info);
		if (ret) {
			dev_err(&client->dev,
				"failed to initialize (%d)\n",
				ret);
			goto err_config;
		}
		info->fw_core_ver = get_fw_version(info, SEC_CORE);
	}

	if (info->ldi == 'L') {
		if ((info->fw_core_ver < 0x53) ||
			(info->fw_core_ver == 0xff)) {
			dev_err(&client->dev, "core version must be 0x53\n");
			dev_err(&client->dev, "excute core firmware update\n");
			ret = mms_ts_core_fw_load(info);
			if (ret) {
				dev_err(&client->dev,
					"failed to initialize (%d)\n", ret);
				goto err_config;
			}
			info->fw_core_ver = get_fw_version(info, SEC_CORE);
		}
		info->panel = get_panel_version(info);
		if (info->panel != 'M') {
			if (info->fw_core_ver == 0x53) {
				dev_err(&client->dev, "cannot read panel info\n");
				dev_err(&client->dev, "excute core firmware update\n");
				ret = mms_ts_fw_load(info, true, 'L');
			} else {
				dev_err(&client->dev, "excute core firmware update\n");
				ret = mms_ts_core_fw_load(info);
			}
			if (ret) {
				dev_err(&client->dev,
					"failed to initialize (%d)\n",
					ret);
			}
		}
		info->fw_ic_ver = get_fw_version(info, SEC_CONFIG);
		if (((info->fw_ic_ver < FW_VERSION_L) ||
			(info->fw_ic_ver == 0xff)) &&
			(info->fw_core_ver == 0x53)) {
			dev_err(&client->dev, "firmware update\n");
			dev_err(&client->dev, "ic:0x%x, bin:0x%x\n",
				info->fw_ic_ver, FW_VERSION_L);
			if ((info->fw_ic_ver >= 0x21) ||
				(info->fw_ic_ver == 0) ||
				(info->fw_ic_ver == 0xff))
				ret = mms_ts_fw_load(info, false, 'N');
			else
				ret = mms_ts_core_fw_load(info);
			if (ret) {
				dev_err(&client->dev,
					"failed to initialize (%d)\n", ret);
				goto err_config;
			}
		}
		if ((info->fw_ic_ver >= 0x50) &&
			(info->fw_ic_ver <= 0x69)) {
			dev_err(&client->dev, "LSI panel, Magna firmware written\n");
			dev_err(&client->dev, "ic:0x%x, bin:0x%x\n",
				info->fw_ic_ver, FW_VERSION_L);
			ret = mms_ts_fw_load(info, true, 'L');
			if (ret) {
				dev_err(&client->dev,
					"failed to initialize (%d)\n", ret);
				goto err_config;
			}
		}
	} else {
		info->panel = get_panel_version(info);
		dev_info(&client->dev, "%c panel\n", info->panel);
		if (info->panel == 'M') {
			if ((info->fw_core_ver < 0x53) ||
				(info->fw_core_ver == 0xff)) {
				dev_err(&client->dev, "core version must be 0x53\n");
				dev_err(&client->dev, "excute core firmware update\n");
				ret = mms_ts_core_fw_load(info);
				if (ret) {
					dev_err(&client->dev,
						"failed to initialize (%d)\n",
						ret);
					goto err_config;
				}
				info->fw_core_ver =
					get_fw_version(info, SEC_CORE);
			}
			info->fw_ic_ver = get_fw_version(info, SEC_CONFIG);
			if ((info->fw_ic_ver < FW_VERSION_M) &&
				(info->fw_core_ver == 0x53)) {
				dev_err(&client->dev, "firmware update\n");
				dev_err(&client->dev, "ic:0x%x, bin:0x%x\n",
					info->fw_ic_ver, FW_VERSION_M);
				if ((info->fw_ic_ver >= 0x24) ||
					(info->fw_ic_ver == 0))
					ret = mms_ts_fw_load(info, false, 'N');
				else
					ret = mms_ts_core_fw_load(info);
				if (ret) {
					dev_err(&client->dev,
						"failed to initialize (%d)\n",
						ret);
					goto err_config;
				}
			}
		} else if (info->panel == 'A') {
			dev_info(&client->dev, "A panel. Do not firm update\n");
		} else {
			dev_err(&client->dev, "cannot read panel info\n");
			info->fw_ic_ver = get_fw_version(info, SEC_CONFIG);
			if (info->fw_core_ver == 0x53) {
				dev_err(&client->dev, "firmware update\n");
				dev_err(&client->dev, "ic:0x%x, bin:0x%x\n",
					info->fw_ic_ver, FW_VERSION_M);
				ret = mms_ts_fw_load(info, true, 'N');
			} else {
				dev_err(&client->dev, "excute core firmware update\n");
				ret = mms_ts_core_fw_load(info);
			}
			if (ret) {
				dev_err(&client->dev,
					"failed to initialize (%d)\n",
					ret);
				goto err_config;
			}
		}
		if ((info->fw_ic_ver >= 0x30) &&
			(info->fw_ic_ver <= 0x49)) {
			dev_err(&client->dev, "Magna panel, LSI firmware written\n");
			dev_err(&client->dev, "ic:0x%x, bin:0x%x\n",
				info->fw_ic_ver, FW_VERSION_M);
			ret = mms_ts_fw_load(info, true, 'M');
			if (ret) {
				dev_err(&client->dev,
					"failed to initialize (%d)\n", ret);
				goto err_config;
			}
		}
	}
	info->panel = get_panel_version(info);
	dev_info(&client->dev, "%c panel\n", info->panel);

	ret = mms_ts_fw_info(info);
	if (ret) {
		dev_err(&client->dev,
			"failed to initialize (%d)\n", ret);
		goto err_config;
	}

	if (info->panel == 'M')
		info->tx_num = TX_NUM_M;
	else
		info->tx_num = TX_NUM_A;

	info->callbacks.inform_charger = melfas_ta_cb;
	if (info->register_cb)
		info->register_cb(&info->callbacks);
#ifdef CONFIG_LCD_FREQ_SWITCH
	info->lcd_callback.inform_lcd = melfas_lcd_cb;
	if (info->register_lcd_cb)
		info->register_lcd_cb(&info->lcd_callback);
	info->tsp_lcdfreq_flag = 0;
#endif

	snprintf(info->phys, sizeof(info->phys),
		 "%s/input0", dev_name(&client->dev));
	input_dev->name = "sec_touchscreen";
	input_dev->phys = info->phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	input_mt_init_slots(input_dev, MAX_FINGERS);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
				0, MAX_PRESSURE, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MINOR,
				0, MAX_PRESSURE, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
				0, (info->max_x)-1, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
				0, (info->max_y)-1, 0, 0);
	if (info->panel == 'M') {
		input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR,
					0, MAX_WIDTH, 0, 0);
		input_set_abs_params(input_dev, ABS_MT_ANGLE,
					MIN_ANGLE, MAX_ANGLE, 0, 0);
		input_set_abs_params(input_dev, ABS_MT_PALM,
					0, 1, 0, 0);
	} else {
		input_set_abs_params(input_dev, ABS_MT_PRESSURE,
					0, MAX_WIDTH, 0, 0);
	}
	input_set_drvdata(input_dev, info);

	ret = input_register_device(input_dev);
	if (ret) {
		dev_err(&client->dev, "failed to register input dev (%d)\n",
			ret);
		goto err_reg_input_dev;
	}

//#if TOUCH_BOOSTER
	mutex_init(&bus_dvfs_lock);
	INIT_DELAYED_WORK(&work_bus_dvfs_off, bus_dvfs_lock_off);
	bus_dev = dev_get("exynos-busfreq");
	bus_dvfs_lock_status = false;
//#endif
	
	mutex_init(&touchbooster_dvfs_lock);
	touchbooster_cpufreq_level = -1;
	touchbooster_dvfs_locked = 0;
	INIT_DELAYED_WORK(&work_touchbooster_off, touchbooster_off);
	INIT_DELAYED_WORK(&work_touchbooster_relax, touchbooster_relax);
	
#if GESTURE_BOOSTER
	mutex_init(&gesturebooster_dvfs_lock);
	INIT_DELAYED_WORK(&work_gesturebooster_dvfs_off, gesturebooster_dvfs_lock_off);
	gesturebooster_cpufreq_level = -1;
	gesturebooster_dvfs_locked = 0;
#endif
    
    
    max_configured_gesture = 1;
    
    max_gesture_finger[0] = 0;
    max_gesture_finger[1] = 0;
    
    gestures_step_count[0][0] = 1;
    gestures_step_count[1][0] = 2;

    ary_gestures_flg_touchkey[0] = 1;
    ary_gestures_flg_exclusive[0] = 1;
    
    ary_gestures_flg_touchkey[1] = 1;
    ary_gestures_flg_exclusive[1] = 1;
    
    point = &gesture_points[0][0][0];
    point->min_x = 0;
    point->max_x = 720;
    point->min_y = 0;
    point->max_y = 427;
    
    //point = &gesture_points[0][0][1];
    //point->min_x = 0;
    //point->max_x = 720;
    //point->min_y = 427;
    //point->max_y = 1280;
    
    point = &gesture_points[1][0][0];
    point->min_x = 0;
    point->max_x = 360;
    point->min_y = 480;
    point->max_y = 800;
    
    point = &gesture_points[1][0][1];
    point->min_x = 360;
    point->max_x = 720;
    point->min_y = 480;
    point->max_y = 800;
    
    reset_gestures_detection(true);

	info->enabled = true;

	ret = request_threaded_irq(client->irq, NULL, mms_ts_interrupt,
				   IRQF_TRIGGER_LOW  | IRQF_ONESHOT,
				   MELFAS_TS_NAME, info);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		goto err_req_irq;
	}
	info->irq = client->irq;

#ifdef CONFIG_HAS_EARLYSUSPEND
	info->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING;
	info->early_suspend.suspend = mms_ts_early_suspend;
	info->early_suspend.resume = mms_ts_late_resume;
	register_early_suspend(&info->early_suspend);
#endif
	
#ifdef CONFIG_TOUCHSCREEN_GESTURES
	ret = misc_register(&gestures_device);
	if (ret) {
		printk(KERN_ERR "[TSP] gestures misc_register failed.\n");
	} else {
		if (sysfs_create_group(&gestures_device.this_device->kobj, &gestures_attr_group)) {
			printk(KERN_ERR "[TSP] sysfs_create_group() has failed\n");
			misc_deregister(&gestures_device);
		} else {
			gestures_device_registered = true;
		}
	}
#endif

#ifdef CONFIG_TOUCH_WAKE
  touchwake_data = info;
  	if (touchwake_data == NULL)
		pr_err("[TOUCHWAKE] Failed to set touchwake_data\n");
    
    x_lo = info->max_x / 10 * 1;  /* 10% display width */
    x_hi = info->max_x / 10 * 7;  /* 70% display width */
    
    arc_start_x_min = 275;
    arc_start_x_max = 445;
    arc_start_y_min = 1210;
    arc_start_y_max = info->max_y;
    arc_end_y_min = 600;
    arc_end_y_max = 1000;
    
    rh_arc_end_x_min = 620;
    rh_arc_end_x_max = info->max_x;
    
    lh_arc_end_x_min = 0;
    lh_arc_end_x_max = 100;
    
#endif  

#ifdef CONFIG_INPUT_FBSUSPEND
	ret = tsp_register_fb(info);
	if (ret)
		pr_err("[TSP] Failed to register fb\n");
#endif

	sec_touchscreen = device_create(sec_class,
					NULL, 0, info, "sec_touchscreen");
	if (IS_ERR(sec_touchscreen)) {
		dev_err(&client->dev,
			"Failed to create device for the sysfs1\n");
		ret = -ENODEV;
	}

#ifdef SEC_TSP_FACTORY_TEST
		INIT_LIST_HEAD(&info->cmd_list_head);
		for (i = 0; i < ARRAY_SIZE(tsp_cmds); i++)
			list_add_tail(&tsp_cmds[i].list, &info->cmd_list_head);

		mutex_init(&info->cmd_lock);
		info->cmd_is_running = false;

	fac_dev_ts = device_create(sec_class,
			NULL, 0, info, "tsp");
	if (IS_ERR(fac_dev_ts))
		dev_err(&client->dev, "Failed to create device for the sysfs\n");

	ret = sysfs_create_group(&fac_dev_ts->kobj,
				 &sec_touch_factory_attr_group);
	if (ret)
		dev_err(&client->dev, "Failed to create sysfs group\n");
#endif
	return 0;

err_req_irq:
	input_unregister_device(input_dev);
err_reg_input_dev:
err_config:
	input_free_device(input_dev);
	/*input_dev = NULL;*/
err_input_alloc:
	kfree(info);
err_alloc:
	return ret;

}

static int __devexit mms_ts_remove(struct i2c_client *client)
{
	struct mms_ts_info *info = i2c_get_clientdata(client);

	unregister_early_suspend(&info->early_suspend);
#ifdef CONFIG_INPUT_FBSUSPEND
	tsp_unregister_fb(info);
#endif
	if (info->irq >= 0)
		free_irq(info->irq, info);
	input_unregister_device(info->input_dev);
	kfree(info);

	return 0;
}

#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
static const struct dev_pm_ops mms_ts_pm_ops = {
	.suspend = mms_ts_suspend,
	.resume = mms_ts_resume,
#ifdef CONFIG_HIBERNATION
	.freeze = mms_ts_suspend,
	.thaw = mms_ts_resume,
	.restore = mms_ts_resume,
#endif
};
#endif

static const struct i2c_device_id mms_ts_id[] = {
	{MELFAS_TS_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, mms_ts_id);

static struct i2c_driver mms_ts_driver = {
	.probe = mms_ts_probe,
	.remove = __devexit_p(mms_ts_remove),
	.driver = {
		   .name = MELFAS_TS_NAME,
#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
		   .pm = &mms_ts_pm_ops,
#endif
		   },
	.id_table = mms_ts_id,
};

static int __init mms_ts_init(void)
{

	return i2c_add_driver(&mms_ts_driver);

}

static void __exit mms_ts_exit(void)
{
#ifdef CONFIG_TOUCHSCREEN_GESTURES
	if (gestures_device_registered) {
		sysfs_remove_group(&gestures_device.this_device->kobj, &gestures_attr_group);
		misc_deregister(&gestures_device);
		gestures_device_registered = false;
	}
#endif
	i2c_del_driver(&mms_ts_driver);
}

module_init(mms_ts_init);
module_exit(mms_ts_exit);

/* Module information */
MODULE_DESCRIPTION("Touchscreen driver for Melfas MMS-series controllers");
MODULE_LICENSE("GPL");
