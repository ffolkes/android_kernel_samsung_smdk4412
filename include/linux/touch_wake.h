/* include/linux/touch_wake.h */

#ifndef _LINUX_TOUCH_WAKE_H
#define _LINUX_TOUCH_WAKE_H

#include <linux/input.h>

struct touchwake_implementation
{
    void (* enable)(void);
    void (* disable)(void);
};

void register_touchwake_implementation(struct touchwake_implementation * imp);

void powerkey_pressed(void);
void powerkey_released(void);
void proximity_detected(void);
void proximity_off(void);
void touch_press(bool forceunlock);
int get_touchoff_delay(void);
bool device_is_suspended(void);
void set_powerkeydev(struct input_dev * input_device);
void touchwake_droplock(void);
void bus_dvfs_lock_on(unsigned int mode);
void gesturebooster_dvfs_lock_on(int gesturebooster_freq_override);

extern bool flg_power_suspend;

void enableSensor(unsigned int dev_id);
void setProxDelay(int64_t dNewDelay);
void disableSensor(unsigned int uChangedSensor, unsigned int uNewEnable);
void forceDisableSensor(unsigned int sensorId);
void forceEnableSensor(unsigned int sensorId, bool force);
bool sensorStatus(unsigned int sensorId);
void forceChangeDelay(unsigned int sensorId, int64_t newDelay);
void an30259a_ledsoff(void);
extern bool flg_ww_prox_on;
extern bool flg_tw_prox_on;
extern int ctr_ww_prox_hits;
extern struct timeval time_suspended;
extern struct timeval time_ledwenton;
extern struct timeval time_ww_last_screenon;
extern struct timeval time_ww_last_screenoff;
extern unsigned int sttg_ww_mode;
extern unsigned int sttg_ww_linger;
extern unsigned int sttg_ww_dismissled;
extern bool sttg_ww_lightflow;
extern bool flg_ww_userspace_noti;
extern bool sttg_ww_waveoff;
extern void ww_set_disable_prox(unsigned int delay);
extern void ww_disable_prox(void);
extern void leds_reset_last(void);
extern unsigned int sttg_ww_waveoff_linger;
extern bool sttg_ww_noredundancies;
extern bool sttg_ww_trigger_noti_only;

extern bool flg_bb_mag_on;

extern unsigned int sttg_pu_mode;
//extern bool sttg_pu_allow_s2w;
extern bool flg_pu_recording;
extern bool flg_pu_tamperevident;
extern bool flg_pu_locktsp;
extern bool flg_pu_locktsp_saved_beforesuspend;
extern bool flg_pu_blackout;
extern unsigned int pu_recording_end(void);
extern bool pu_valid_pattern(void);
extern bool pu_valid(void);
extern void pu_clearAll(void);
extern void pu_setFrontLED(unsigned int mode);
extern void alternateFrontLED(unsigned int duty1, unsigned int r1, unsigned int g1, unsigned int b1,
							  unsigned int duty2, unsigned int r2, unsigned int g2, unsigned int b2);
extern void flashFrontLED(unsigned int duty1, unsigned int r1, unsigned int g1, unsigned int b1);

extern bool sttg_pu_tamperevident;
extern bool sttg_pu_warnled;
extern bool sttg_pu_allow_tw;

extern struct timeval time_homekey_lastpressed;

extern unsigned int sttg_ka_mode;
extern unsigned int sttg_ka_knock2_window;
extern unsigned int sttg_ka_knock3_window;
extern unsigned int sttg_ka_knock2_keycode;
extern bool sttg_ka_knock2_keydelay;
extern unsigned int sttg_ka_knock3_keycode;
extern bool sttg_ka_knock3_keydelay;
extern unsigned int sttg_ka_knock2_min_speed;
extern unsigned int sttg_ka_knock3_min_speed;

extern bool flg_kw_gyro_on;
extern unsigned int ctr_knocks;
extern unsigned int sttg_kw_mode;
extern int64_t sttg_kw_resolution;
extern unsigned int sttg_kw_gyro_y_trigger_min_threshold;
extern unsigned int sttg_kw_gyro_y_trigger_max_threshold;
extern unsigned int sttg_kw_gyro_y_history;
extern unsigned int sttg_kw_knock2_window;
extern unsigned int sttg_kw_knock3_window;
extern unsigned int sttg_kw_knock3_keycode;
extern bool sttg_kw_knock3_keydelay;
extern unsigned int sttg_kw_knock4_window;
extern unsigned int sttg_kw_knock4_keycode;
extern bool sttg_kw_knock4_keydelay;
extern unsigned int sttg_kw_gyro_y_min_threshold;
extern unsigned int sttg_kw_gyro_y_max_threshold;
extern int sttg_kw_gyro_x_min_threshold;
extern int sttg_kw_gyro_x_max_threshold;
extern int sttg_kw_gyro_z_min_threshold;
extern int sttg_kw_gyro_z_max_threshold;
extern unsigned int sttg_kw_z_instab_suspensions;
extern unsigned int sttg_kw_gyro_y_min_average;
extern unsigned int sttg_kw_gyro_y_max_average;
extern unsigned int sttg_kw_init_detection_suspensions;
extern unsigned int sttg_kw_post_suspension_validations;
extern bool sttg_kw_strict_validation;
extern unsigned int sttg_kw_instab_tolerance;
extern unsigned int sttg_kw_extreme_instab_tolerance;
extern unsigned int sttg_kw_extreme_instab_suspensions;
extern unsigned int sttg_kw_knock_min_speed;
extern unsigned int kw_status_code;
extern bool sttg_kw_debug;
extern unsigned int sttg_kw_tsp_event_threshold;
extern unsigned int sttg_kw_tsp_event_suspensions;
extern void ignoreKnocks(unsigned int suspensions);

extern bool prox_near;
extern bool ignore_once;

extern unsigned int wpmk_block_touch;
extern bool flg_kw_pressedpower;

extern bool flg_touchkey_pressed;
extern bool flg_touchkey_was_pressed;

extern bool sttg_epen_worryfree_mode;
extern bool flg_epen_worryfree_mode;

extern unsigned int sttg_gesture_delay;

extern struct input_dev * powerkey_device;
extern struct ssp_data * prox_device;

extern bool flg_touchwake_active;
extern bool flg_screen_on;
extern bool sttg_keys_ignorehomekeywake;
extern bool touchwake_enabled;
extern unsigned int touchoff_delay;
extern bool sttg_touchwake_swipe_only;
extern bool flg_touchwake_pressed;
extern bool flg_touchwake_swipe_only;
extern bool sttg_touchwake_persistent;
extern bool sttg_touchwake_persistent_wakelock;
extern bool sttg_touchwake_ignoretkeys;
extern bool sttg_touchwake_ignoregestures;
extern bool sttg_touchwake_ignorepowerkey;
extern unsigned int sttg_touchwake_swipe_y_drift_tolerance;
extern unsigned int sttg_touchwake_swipe_max_pressure;
extern unsigned int sttg_touchwake_swipe_finger_angle_tolerance;
extern bool sttg_touchwake_swipe_fast_trigger;
extern bool sttg_touchwake_swipe_arc_rh;
extern bool sttg_touchwake_swipe_arc_lh;
extern bool sttg_touchwake_longpressoff_enabled;
extern unsigned int sttg_touchwake_longpressoff_duration;
extern unsigned int sttg_touchwake_longpressoff_timeout;
extern unsigned int sttg_touchwake_longpressoff_min_pressure;
extern unsigned int sttg_touchwake_longpressoff_xy_drift_tolerance;

extern unsigned int wpmk_min_pressure_fuzzy;
extern unsigned int wpmk_min_pressure_entry_tolerance;

#endif
