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
void touch_press(void);
int get_touchoff_delay(void);
bool device_is_suspended(void);
void set_powerkeydev(struct input_dev * input_device);
void touchwake_droplock(void);
void bus_dvfs_lock_on(unsigned int mode);
void gesturebooster_dvfs_lock_on(int gesturebooster_freq_override);

extern bool prox_near;

extern bool flg_touchkey_pressed;
extern bool flg_touchkey_was_pressed;

extern unsigned int sttg_gesture_delay;

extern struct input_dev * powerkey_device;

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

#endif
