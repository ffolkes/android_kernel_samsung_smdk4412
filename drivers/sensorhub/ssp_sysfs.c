/*
 *  Copyright (C) 2012, Samsung Electronics Co. Ltd. All Rights Reserved.
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
 */
#include "ssp.h"
#include <linux/touch_wake.h>

/*************************************************************************/
/* SSP data delay function                                              */
/*************************************************************************/

unsigned int get_msdelay(int64_t dDelayRate)
{
	if (dDelayRate <= SENSOR_NS_DELAY_FASTEST)
		return SENSOR_MS_DELAY_FASTEST;
	else if (dDelayRate <= SENSOR_NS_DELAY_GAME)
		return SENSOR_MS_DELAY_GAME;
	else if (dDelayRate <= SENSOR_NS_DELAY_UI)
		return SENSOR_MS_DELAY_UI;
	else
		return SENSOR_MS_DELAY_NORMAL;
}

unsigned int get_delay_cmd(u8 uDelayRate)
{
	if (uDelayRate <= SENSOR_MS_DELAY_FASTEST)
		return SENSOR_CMD_DELAY_FASTEST;
	else if (uDelayRate <= SENSOR_MS_DELAY_GAME)
		return SENSOR_CMD_DELAY_GAME;
	else if (uDelayRate <= SENSOR_MS_DELAY_UI)
		return SENSOR_CMD_DELAY_UI;
	else
		return SENSOR_CMD_DELAY_NORMAL;
}

static void change_sensor_delay(struct ssp_data *data,
	int iSensorType, int64_t dNewDelay)
{
	u8 uBuf[2];
	unsigned int uNewEnable = 0;
	int64_t dTempDelay = data->adDelayBuf[iSensorType];

	if (!(atomic_read(&data->aSensorEnable) & (1 << iSensorType))) {
		//if (iSensorType == 5) {
		//	pr_info("[SSPff]: sensor: %d, PROX isn't enabled in %d, trying to manually reactivatate...\n", iSensorType, atomic_read(&data->aSensorEnable));
		//	forceEnableSensor(iSensorType, true);
		//} else {
			pr_info("[SSPff]: sensor: %d, this sensor isn't enabled in %d\n", iSensorType, atomic_read(&data->aSensorEnable));
			data->aiCheckStatus[iSensorType] = NO_SENSOR_STATE;
			return;
		//}
	}
	
	// filter changes.
	if (iSensorType == GYROSCOPE_SENSOR && flg_kw_gyro_on) {
		// gyro is having its rate changed. set it to our default.
		
		pr_info("[SSP]: GYRO DELAY READJUSTING! was going to be: %lld\n", dNewDelay);
		
		// set it back to what we want.
		dNewDelay = sttg_kw_resolution;
		
		pr_info("[SSP]: GYRO DELAY READJUSTED! is now: %lld\n", dNewDelay);
		
		dNewDelay = (int64_t)sttg_kw_resolution;
		
		pr_info("[SSP]: GYRO DELAY READJUSTED2! is now: %lld\n", dNewDelay);
		
	} else if (iSensorType == GYROSCOPE_SENSOR) {
		pr_info("[SSP]: GYRO DELAY CHANGED! now: %lld, was: %lld\n", dNewDelay, dTempDelay);
	}

	data->adDelayBuf[iSensorType] = dNewDelay;

	if (iSensorType == ORIENTATION_SENSOR)
		iSensorType = ACCELEROMETER_SENSOR;

	switch (data->aiCheckStatus[iSensorType]) {
	case ADD_SENSOR_STATE:
		ssp_dbg("[SSP]: %s - add %u, New = %lldns\n",
			 __func__, 1 << iSensorType, dNewDelay);

		uBuf[1] = (u8)get_msdelay(dNewDelay);
		uBuf[0] = (u8)get_delay_cmd(uBuf[1]);

		if (send_instruction(data, ADD_SENSOR, iSensorType, uBuf, 2)
			!= SUCCESS) {
			uNewEnable =
				(unsigned int)atomic_read(&data->aSensorEnable)
				& (~(unsigned int)(1 << iSensorType));
			atomic_set(&data->aSensorEnable, uNewEnable);

			data->aiCheckStatus[iSensorType] = NO_SENSOR_STATE;
			data->uMissSensorCnt++;
			break;
		}

		data->aiCheckStatus[iSensorType] = RUNNING_SENSOR_STATE;

		if (iSensorType == PROXIMITY_SENSOR) {
			proximity_open_lcd_ldi(data);
			proximity_open_calibration(data);

			input_report_abs(data->prox_input_dev, ABS_DISTANCE, 1);
			input_sync(data->prox_input_dev);
		}
		break;
	case RUNNING_SENSOR_STATE:
		if (get_msdelay(dTempDelay)
			== get_msdelay(data->adDelayBuf[iSensorType]))
			break;

		ssp_dbg("[SSP]: %s - Change %u, New = %lldns\n",
			__func__, 1 << iSensorType, dNewDelay);

		uBuf[1] = (u8)get_msdelay(dNewDelay);
		uBuf[0] = (u8)get_delay_cmd(uBuf[1]);
		send_instruction(data, CHANGE_DELAY, iSensorType, uBuf, 2);

		break;
	default:
		break;
	}
}

/*************************************************************************/
/* SSP data enable function                                              */
/*************************************************************************/

static int ssp_add_sensor(struct ssp_data *data, unsigned int uChangedSensor)
{
	if ((data->aiCheckStatus[uChangedSensor] != INITIALIZATION_STATE)
		&& (!atomic_read(&data->aSensorEnable))) {
		if (data->bCheckSuspend == false)
			data->bDebugEnabled = true;
	}

	return 0;
}

static int ssp_remove_sensor(struct ssp_data *data,
	unsigned int uChangedSensor, unsigned int uNewEnable)
{
	u8 uBuf[2];
	int64_t dSensorDelay = data->adDelayBuf[uChangedSensor];

	ssp_dbg("[SSP]: %s - remove sensor = %d, current state = %d\n",
		__func__, (1 << uChangedSensor), uNewEnable);

	data->adDelayBuf[uChangedSensor] = DEFUALT_POLLING_DELAY;

        if (uChangedSensor == ORIENTATION_SENSOR) {
		if (!(atomic_read(&data->aSensorEnable)
			& (1 << ACCELEROMETER_SENSOR))) {
			uChangedSensor = ACCELEROMETER_SENSOR;
		} else {
			change_sensor_delay(data, ACCELEROMETER_SENSOR,
				data->adDelayBuf[ACCELEROMETER_SENSOR]);
			return 0;
		}
	} else if (uChangedSensor == ACCELEROMETER_SENSOR) {
		if (atomic_read(&data->aSensorEnable)
			& (1 << ORIENTATION_SENSOR)) {
			change_sensor_delay(data, ORIENTATION_SENSOR,
				data->adDelayBuf[ORIENTATION_SENSOR]);
			return 0;
		}
	}

	if (!uNewEnable) {
		if (data->bCheckSuspend == false)
			data->bDebugEnabled = false;
	}

	if (atomic_read(&data->aSensorEnable) & (1 << uChangedSensor)) {
		uBuf[1] = (u8)get_msdelay(dSensorDelay);
		uBuf[0] = (u8)get_delay_cmd(uBuf[1]);

		send_instruction(data, REMOVE_SENSOR, uChangedSensor, uBuf, 2);
	}
	data->aiCheckStatus[uChangedSensor] = NO_SENSOR_STATE;
	return 0;
}

/*************************************************************************/
/* ssp Sysfs                                                             */
/*************************************************************************/


void forceDisableSensor(unsigned int sensorId)
{
	
	//return;
	unsigned int uSensorsEnabled = 0;
	
	// 1. get sensorenable number.
	// 2. see if this sensor's bit is disabled.
	// 3. if not, disable the sensor and the bit.
	
	// check the sensor's bit. 5 = prox.
	if ((atomic_read(&prox_device->aSensorEnable) & (1 << sensorId))) {
		// IF sensor is enabled, disable it.
		
		pr_info("[SSP/disablesensor]: sensor was enabled, now disabling...\n");
		
		if (sensorId == GYROSCOPE_SENSOR && !flg_kw_gyro_on) {
			// don't turn the prox sensor off unless we think we turned it on.
			pr_info("[SSP/disablesensor]: calling for gyro off but flg_kw_gyro_on was not set, probably shouldn't disable\n");
			//return;
		}
		
		if (sensorId == PROXIMITY_SENSOR && !flg_ww_prox_on) {
			// don't turn the prox sensor off unless we think we turned it on.
			pr_info("[SSP/disablesensor]: calling for prox off but flg_ww_prox_on/flg_tw_prox_on was not set, probably shouldn't disable\n");
			//return;
		}
		
		/*if (sensorId == PROXIMITY_SENSOR && !flg_tw_prox_on) {
			// don't turn the prox sensor off unless we think we turned it on.
			pr_info("[SSP/disablesensor]: calling for prox off but flg_ww_prox_on/flg_tw_prox_on was not set, probably shouldn't disable\n");
			//return;
		}*/
		
		// get enabled sensors.
		uSensorsEnabled = atomic_read(&prox_device->aSensorEnable);
		
		pr_info("[SSP/disablesensor]: aSensorEnable was %d\n", atomic_read(&prox_device->aSensorEnable));
		
		// set this sensor's bit to 0.
		uSensorsEnabled &= ~(1 << sensorId);
		
		// remove sensor.
		ssp_remove_sensor(prox_device, sensorId, uSensorsEnabled);
		
		// save it.
		atomic_set(&prox_device->aSensorEnable, uSensorsEnabled);
		
		if (sensorId == GYROSCOPE_SENSOR) {
			flg_kw_gyro_on = false;
			pr_info("[SSP/disablesensor]: KW_GYRO OFF!\n");
		}
		
		if (sensorId == PROXIMITY_SENSOR) {
			flg_ww_prox_on = false;
			pr_info("[SSP/disablesensor]: WW_PROX OFF!\n");
		}
		
		pr_info("[SSP/disablesensor]: disabled %d, aSensorEnable now: %d\n", sensorId, uSensorsEnabled);
		
	} else {
		pr_info("[SSP/disablesensor]: sensor was disabled already. ignoring.\n");
	}
	
	/*u8 uBuf[2];
	int64_t dSensorDelay = prox_device->adDelayBuf[sensorId];
	
	pr_info("[SSP]: force disable\n");
	
	uBuf[1] = (u8)get_msdelay(dSensorDelay);
	uBuf[0] = (u8)get_delay_cmd(uBuf[1]);
	
	send_instruction(prox_device, REMOVE_SENSOR, sensorId, uBuf, 2);*/
	
}
EXPORT_SYMBOL(forceDisableSensor);

void forceEnableSensor(unsigned int sensorId, bool force)
{
	
	//return;
	unsigned int uSensorsEnabled = 0;
	u8 uBuf[2];
	int64_t dSensorDelay = prox_device->adDelayBuf[sensorId];
	
	// 1. get sensorenable number.
	// 2. see if this sensor's bit is enabled.
	// 3. if not, enable the sensor and the bit.
	
	// check the sensor's bit. 5 = prox.
	if (!(atomic_read(&prox_device->aSensorEnable) & (1 << sensorId))) {
		// IF sensor is disabled, enable it.
		
		pr_info("[SSP/enablesensor]: sensor (%d) was disabled, enabling...\n", sensorId);
		
		pr_info("[SSP/enablesensor]: checking aicheckstatus. set to: %d, looking for %d. add is: %d\n",
				prox_device->aiCheckStatus[sensorId], INITIALIZATION_STATE, ADD_SENSOR_STATE);
		
		if (prox_device->aiCheckStatus[sensorId] == INITIALIZATION_STATE) {
			
			if (sensorId == ACCELEROMETER_SENSOR) {
				pr_info("[SSP/enablesensor]: sensor (%d) ACCEL\n", sensorId);
				accel_open_calibration(prox_device);
			} else if (sensorId == GYROSCOPE_SENSOR) {
				pr_info("[SSP/enablesensor]: sensor (%d) GYRO\n", sensorId);
				gyro_open_calibration(prox_device);
			} else if (sensorId == PRESSURE_SENSOR) {
				pr_info("[SSP/enablesensor]: sensor (%d) PRES\n", sensorId);
				pressure_open_calibration(prox_device);
			} else if (sensorId == PROXIMITY_SENSOR) {
				pr_info("[SSP/enablesensor]: sensor (%d) PROX\n", sensorId);
				proximity_open_lcd_ldi(prox_device);
				proximity_open_calibration(prox_device);
			}
		}
		
		prox_device->aiCheckStatus[sensorId] = ADD_SENSOR_STATE;
		
		// get enabled sensors.
		uSensorsEnabled = atomic_read(&prox_device->aSensorEnable);
		
		pr_info("[SSP/enablesensor]: enabled was %d\n", atomic_read(&prox_device->aSensorEnable));
		
		// set this sensor's bit to 1.
		uSensorsEnabled |= 1 << sensorId;
		
		// save it.
		atomic_set(&prox_device->aSensorEnable, uSensorsEnabled);
		
		pr_info("[SSP/enablesensor]: enabled is %d and should be %d\n", atomic_read(&prox_device->aSensorEnable), uSensorsEnabled);
		
		if (sensorId == 1) {
			
			if (!force) {
				pr_info("[SSP/enablesensor]: flg_kw_gyro_on=true changing delay\n");
				flg_kw_gyro_on = true;
			} else {
				pr_info("[SSP/enablesensor]: flg_kw_gyro_on=false changing delay\n");
			}
			
			change_sensor_delay(prox_device, GYROSCOPE_SENSOR, sttg_kw_resolution);
			pr_info("[SSP/enablesensor]: GYRO ON!\n");
			
		} else if (sensorId == 5) {
			
			if (!force) {
				pr_info("[SSP/enablesensor]: flg_ww_prox_on= true changing delay\n");
				flg_ww_prox_on = true;
			} else {
				pr_info("[SSP/enablesensor]: flg_ww_prox_on=false changing delay\n");
			}
			
			change_sensor_delay(prox_device, PROXIMITY_SENSOR, 66667000);
			pr_info("[SSP/enablesensor]: PROX ON!\n");
		}
		
	} else {
		pr_info("[SSP/enablesensor]: sensor was enabled already, ignoring.\n");
	}
	
	/*u8 uBuf[2];
	int64_t dSensorDelay = prox_device->adDelayBuf[sensorId];
	
	pr_info("[SSP]: force enable\n");
	
	uBuf[1] = (u8)get_msdelay(dSensorDelay);
	uBuf[0] = (u8)get_delay_cmd(uBuf[1]);
	
	send_instruction(prox_device, ADD_SENSOR, sensorId, uBuf, 2);
	proximity_open_lcd_ldi(prox_device);
	proximity_open_calibration(prox_device);*/
	
}
EXPORT_SYMBOL(forceEnableSensor);

bool sensorStatus(unsigned int sensorId)
{
	
	unsigned int uSensorsEnabled = 0;
	
	// check the sensor's bit. 5 = prox.
	if ((atomic_read(&prox_device->aSensorEnable) & (1 << sensorId))) {
		// IF sensor is disabled, enable it.
		
		pr_info("[SSP/sensorStatus]: sensor (%d) is enabled\n", sensorId);
		return true;
		
	} else {
		
		pr_info("[SSP/sensorStatus]: sensor (%d) is disabled\n");
		return false;
	}
	
}
EXPORT_SYMBOL(sensorStatus);

void forceChangeDelay(unsigned int sensorId, int64_t newDelay)
{
	change_sensor_delay(prox_device, sensorId, newDelay);
}
EXPORT_SYMBOL(forceChangeDelay);

static ssize_t show_sensors_enable(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	ssp_dbg("[SSP]: %s - cur_enable = %d\n", __func__,
		 atomic_read(&data->aSensorEnable));

	return sprintf(buf, "%9u\n", atomic_read(&data->aSensorEnable));
}

static ssize_t set_sensors_enable(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t dTemp;
	unsigned int uNewEnable = 0, uChangedSensor = 0;
	struct ssp_data *data = dev_get_drvdata(dev);
	int iRet;
	
	prox_device = dev_get_drvdata(dev);

	if (strict_strtoll(buf, 10, &dTemp) < 0)
		return -1;

	uNewEnable = (unsigned int)dTemp;
	ssp_dbg("[SSP]: %s - new_enable = %u, old_enable = %u\n", __func__,
		 uNewEnable, atomic_read(&data->aSensorEnable));
	
	pr_info("[SSP]: flg_power_suspend: %d\n", flg_power_suspend);
	
	// filter changes.
	if (sttg_kw_mode
		&& !(uNewEnable & (1 << GYROSCOPE_SENSOR))
		&& (!flg_screen_on || flg_power_suspend || sttg_ka_mode)) {
		// 1 is being disabled, don't do it.
		
		// set this sensor's bit to 1.
		uNewEnable |= 1 << GYROSCOPE_SENSOR;
		
		pr_info("[SSP]: REENABLED GYRO! uNewEnable: %u (%d)\n", uNewEnable, (uNewEnable & (1 << GYROSCOPE_SENSOR)));
		
	} else {
		pr_info("[SSP]: DIDN'T REENABLE GYRO! (%d)\n", (uNewEnable & (1 << GYROSCOPE_SENSOR)));
	}
	
	if (!(uNewEnable & (1 << PROXIMITY_SENSOR)) && flg_ww_prox_on) {
		// 5 (prox) is being disabled, don't do it.
		
		pr_info("[SSP]: REENABLED PROX!\n");
		
		// set this sensor's bit to 1.
		uNewEnable |= 1 << PROXIMITY_SENSOR;
		
	} else {
		pr_info("[SSP]: DIDN'T REENABLE PROX!\n");
	}

	if (uNewEnable == atomic_read(&data->aSensorEnable))
		return size;

	for (uChangedSensor = 0; uChangedSensor < SENSOR_MAX; uChangedSensor++) {
		
		//pr_info("[SSP]: checking %d going from %d to %d\n", uChangedSensor, atomic_read(&data->aSensorEnable), uNewEnable);
		
		if ((atomic_read(&data->aSensorEnable) & (1 << uChangedSensor)) != (uNewEnable & (1 << uChangedSensor))) {
			
			pr_info("[SSP]: FOUND A CHANGE! %d\n", uChangedSensor);

			if (!(uNewEnable & (1 << uChangedSensor))) {
				
				//if ((uChangedSensor == 5 && flg_ww_prox_on) || (uChangedSensor == 1 && (!flg_screen_on || flg_power_suspend || sttg_ka_mode))) {
				//	pr_info("[SSP]: ___________________ skipping 5 or 2...\n");
				//} else {
					ssp_dbg("[SSP]: %s - removing. changed=%u, new=%u\n", __func__, uChangedSensor, uNewEnable);
					ssp_remove_sensor(data, uChangedSensor, uNewEnable); /* disable */
					//break;
				//}
				
			} else { /* Change to ADD_SENSOR_STATE from KitKat */
				
				pr_info("[SSP]: %d isn't being removed\n", uChangedSensor);
				
				if (data->aiCheckStatus[uChangedSensor] == INITIALIZATION_STATE) {
					
					if (uChangedSensor == ACCELEROMETER_SENSOR)
						accel_open_calibration(data);
					else if (uChangedSensor == GYROSCOPE_SENSOR)
						gyro_open_calibration(data);
					else if (uChangedSensor == PRESSURE_SENSOR)
						pressure_open_calibration(data);
					else if (uChangedSensor == PROXIMITY_SENSOR) {
						proximity_open_lcd_ldi(data);
						proximity_open_calibration(data);
					}
				}
				
				data->aiCheckStatus[uChangedSensor] = ADD_SENSOR_STATE;
			}
			
			break;
		}
	}

	atomic_set(&data->aSensorEnable, uNewEnable);

	return size;
}

static ssize_t show_acc_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%lld\n", data->adDelayBuf[ACCELEROMETER_SENSOR]);
}

static ssize_t set_acc_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t dNewDelay;
	struct ssp_data *data = dev_get_drvdata(dev);

	if (strict_strtoll(buf, 10, &dNewDelay) < 0)
		return -1;

	if ((atomic_read(&data->aSensorEnable) & (1 << ORIENTATION_SENSOR)) &&
		(data->adDelayBuf[ORIENTATION_SENSOR] < dNewDelay))
		data->adDelayBuf[ACCELEROMETER_SENSOR] = dNewDelay;
	else
		change_sensor_delay(data, ACCELEROMETER_SENSOR, dNewDelay);

	return size;
}

static ssize_t show_ori_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%lld\n", data->adDelayBuf[ORIENTATION_SENSOR]);
}

static ssize_t set_ori_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t dNewDelay;
	struct ssp_data *data = dev_get_drvdata(dev);

	if (strict_strtoll(buf, 10, &dNewDelay) < 0)
		return -1;

	if (data->aiCheckStatus[ACCELEROMETER_SENSOR] == NO_SENSOR_STATE) {
		data->aiCheckStatus[ACCELEROMETER_SENSOR] = ADD_SENSOR_STATE;
		change_sensor_delay(data, ORIENTATION_SENSOR, dNewDelay);
	} else if (data->aiCheckStatus[ACCELEROMETER_SENSOR] ==
			RUNNING_SENSOR_STATE) {
		if (dNewDelay < data->adDelayBuf[ACCELEROMETER_SENSOR])
			change_sensor_delay(data,
				ORIENTATION_SENSOR, dNewDelay);
		else
			data->adDelayBuf[ORIENTATION_SENSOR] = dNewDelay;
	}
	return size;
}

static ssize_t show_gyro_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%lld\n", data->adDelayBuf[GYROSCOPE_SENSOR]);
}

static ssize_t set_gyro_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t dNewDelay;
	struct ssp_data *data = dev_get_drvdata(dev);

	if (strict_strtoll(buf, 10, &dNewDelay) < 0)
		return -1;

	change_sensor_delay(data, GYROSCOPE_SENSOR, dNewDelay);
	return size;
}

static ssize_t show_mag_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%lld\n", data->adDelayBuf[GEOMAGNETIC_SENSOR]);
}

static ssize_t set_mag_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t dNewDelay;
	struct ssp_data *data = dev_get_drvdata(dev);

	if (strict_strtoll(buf, 10, &dNewDelay) < 0)
		return -1;

	change_sensor_delay(data, GEOMAGNETIC_SENSOR, dNewDelay);

	return size;
}

static ssize_t show_pressure_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data  = dev_get_drvdata(dev);

	return sprintf(buf, "%lld\n", data->adDelayBuf[PRESSURE_SENSOR]);
}

static ssize_t set_pressure_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t dNewDelay;
	struct ssp_data *data  = dev_get_drvdata(dev);

	if (strict_strtoll(buf, 10, &dNewDelay) < 0)
		return -1;

	change_sensor_delay(data, PRESSURE_SENSOR, dNewDelay);
	return size;
}

static ssize_t show_light_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data  = dev_get_drvdata(dev);

	return sprintf(buf, "%lld\n", data->adDelayBuf[LIGHT_SENSOR]);
}

static ssize_t set_light_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t dNewDelay;
	struct ssp_data *data  = dev_get_drvdata(dev);

	if (strict_strtoll(buf, 10, &dNewDelay) < 0)
		return -1;

	change_sensor_delay(data, LIGHT_SENSOR, dNewDelay);
	return size;
}

static ssize_t show_prox_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data  = dev_get_drvdata(dev);

	return sprintf(buf, "%lld\n", data->adDelayBuf[PROXIMITY_SENSOR]);
}

static ssize_t set_prox_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t dNewDelay;
	struct ssp_data *data  = dev_get_drvdata(dev);

	if (strict_strtoll(buf, 10, &dNewDelay) < 0)
		return -1;

	change_sensor_delay(data, PROXIMITY_SENSOR, dNewDelay);
	return size;
}

static DEVICE_ATTR(mcu_rev, S_IRUGO, mcu_revision_show, NULL);
static DEVICE_ATTR(mcu_name, S_IRUGO, mcu_model_name_show, NULL);
static DEVICE_ATTR(mcu_update, S_IRUGO, mcu_update_show, NULL);
static DEVICE_ATTR(mcu_update2, S_IRUGO, mcu_update2_show, NULL);
static DEVICE_ATTR(mcu_reset, S_IRUGO, mcu_reset_show, NULL);

static DEVICE_ATTR(mcu_test, S_IRUGO | S_IWUSR | S_IWGRP,
	mcu_factorytest_show, mcu_factorytest_store);
static DEVICE_ATTR(mcu_sleep_test, S_IRUGO | S_IWUSR | S_IWGRP,
	mcu_sleep_factorytest_show, mcu_sleep_factorytest_store);
static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
	show_sensors_enable, set_sensors_enable);
static DEVICE_ATTR(acc_poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_acc_delay, set_acc_delay);
static DEVICE_ATTR(gyro_poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_gyro_delay, set_gyro_delay);
static DEVICE_ATTR(mag_poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_mag_delay, set_mag_delay);
static DEVICE_ATTR(ori_poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_ori_delay, set_ori_delay);
static DEVICE_ATTR(pressure_poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_pressure_delay, set_pressure_delay);
static DEVICE_ATTR(light_poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_light_delay, set_light_delay);
static DEVICE_ATTR(prox_poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_prox_delay, set_prox_delay);

static struct device_attribute *mcu_attrs[] = {
	&dev_attr_enable,
	&dev_attr_mcu_rev,
	&dev_attr_mcu_name,
	&dev_attr_mcu_test,
	&dev_attr_mcu_reset,
	&dev_attr_mcu_update,
	&dev_attr_mcu_update2,
	&dev_attr_mcu_sleep_test,
	&dev_attr_mag_poll_delay,
	&dev_attr_ori_poll_delay,
	NULL,
};

static void initialize_mcu_factorytest(struct ssp_data *data)
{
	sensors_register(data->mcu_device, data, mcu_attrs, "ssp_sensor");
}

static void remove_mcu_factorytest(struct ssp_data *data)
{
	sensors_unregister(data->mcu_device, mcu_attrs);
}

int initialize_sysfs(struct ssp_data *data)
{
	if (device_create_file(&data->acc_input_dev->dev,
		&dev_attr_acc_poll_delay))
		goto err_acc_input_dev;

	if (device_create_file(&data->gyro_input_dev->dev,
		&dev_attr_gyro_poll_delay))
		goto err_gyro_input_dev;

	if (device_create_file(&data->pressure_input_dev->dev,
		&dev_attr_pressure_poll_delay))
		goto err_pressure_input_dev;

	if (device_create_file(&data->light_input_dev->dev,
		&dev_attr_light_poll_delay))
		goto err_light_input_dev;

	if (device_create_file(&data->prox_input_dev->dev,
		&dev_attr_prox_poll_delay))
		goto err_prox_input_dev;

	initialize_accel_factorytest(data);
	initialize_gyro_factorytest(data);
	initialize_prox_factorytest(data);
	initialize_light_factorytest(data);
	initialize_pressure_factorytest(data);
	initialize_magnetic_factorytest(data);
	initialize_mcu_factorytest(data);

	return SUCCESS;

err_prox_input_dev:
	device_remove_file(&data->light_input_dev->dev,
		&dev_attr_light_poll_delay);
err_light_input_dev:
	device_remove_file(&data->pressure_input_dev->dev,
		&dev_attr_pressure_poll_delay);
err_pressure_input_dev:
	device_remove_file(&data->gyro_input_dev->dev,
		&dev_attr_gyro_poll_delay);
err_gyro_input_dev:
	device_remove_file(&data->acc_input_dev->dev,
		&dev_attr_acc_poll_delay);
err_acc_input_dev:
	return ERROR;
}

void remove_sysfs(struct ssp_data *data)
{
	device_remove_file(&data->acc_input_dev->dev,
		&dev_attr_acc_poll_delay);
	device_remove_file(&data->gyro_input_dev->dev,
		&dev_attr_gyro_poll_delay);
	device_remove_file(&data->pressure_input_dev->dev,
		&dev_attr_pressure_poll_delay);
	device_remove_file(&data->light_input_dev->dev,
		&dev_attr_light_poll_delay);
	device_remove_file(&data->prox_input_dev->dev,
		&dev_attr_prox_poll_delay);

	remove_accel_factorytest(data);
	remove_gyro_factorytest(data);
	remove_prox_factorytest(data);
	remove_light_factorytest(data);
	remove_pressure_factorytest(data);
	remove_magnetic_factorytest(data);
	remove_mcu_factorytest(data);
	destroy_sensor_class();
}
