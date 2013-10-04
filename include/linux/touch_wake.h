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
bool device_is_suspended(void);
void set_powerkeydev(struct input_dev * input_device);

extern bool flg_touchwake_active;
extern bool touchwake_enabled;
extern unsigned int touchoff_delay;

#endif
