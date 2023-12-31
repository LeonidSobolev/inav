/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "config/parameter_group.h"

#ifndef VBAT_SCALE_DEFAULT
#define VBAT_SCALE_DEFAULT 1100
#endif
#define VBAT_SCALE_MIN 0
#define VBAT_SCALE_MAX 65535

#ifndef CURRENT_METER_SCALE
#define CURRENT_METER_SCALE 400 // for Allegro ACS758LCB-100U (40mV/A)
#endif

#ifndef CURRENT_METER_OFFSET
#define CURRENT_METER_OFFSET 0
#endif

typedef enum {
    CURRENT_SENSOR_NONE = 0,
    CURRENT_SENSOR_ADC,
    CURRENT_SENSOR_VIRTUAL,
    CURRENT_SENSOR_MAX = CURRENT_SENSOR_VIRTUAL
} currentSensor_e;

typedef enum {
    BAT_CAPACITY_UNIT_MAH,
    BAT_CAPACITY_UNIT_MWH,
} batCapacityUnit_e;

typedef struct batteryConfig_s {

    struct {
        uint16_t scale;         // adjust this to match battery voltage to reported value
        uint16_t cellMax;       // maximum voltage per cell, used for auto-detecting battery voltage in 0.01V units, default is 421 (4.21V)
        uint16_t cellMin;       // minimum voltage per cell, this triggers battery critical alarm, in 0.01V units, default is 330 (3.3V)
        uint16_t cellWarning;   // warning voltage per cell, this triggers battery warning alarm, in 0.01V units, default is 350 (3.5V)
    } voltage;

    struct {
        int16_t scale;          // scale the current sensor output voltage to milliamps. Value in 1/10th mV/A
        int16_t offset;         // offset of the current sensor in millivolt steps
        currentSensor_e type;   // type of current meter used, either ADC or virtual
    } current;

    struct {
        uint32_t value;         // mAh or mWh (see capacity.unit)
        uint32_t warning;       // mAh or mWh (see capacity.unit)
        uint32_t critical;      // mAh or mWh (see capacity.unit)
        batCapacityUnit_e unit; // Describes unit of capacity.value, capacity.warning and capacity.critical
    } capacity;

} batteryConfig_t;

PG_DECLARE(batteryConfig_t, batteryConfig);

typedef enum {
    BATTERY_OK = 0,
    BATTERY_WARNING,
    BATTERY_CRITICAL,
    BATTERY_NOT_PRESENT

} batteryState_e;

float get_voltage_launch_time(void);
void ics_clearVoltageFilters(void);

uint16_t batteryAdcToVoltage(uint16_t src);
batteryState_e getBatteryState(void);
bool batteryWasFullWhenPluggedIn(void);
bool batteryUsesCapacityThresholds(void);
void batteryUpdate(uint32_t vbatTimeDelta);
void batteryInit(void);

bool isBatteryVoltageConfigured(void);
uint16_t getBatteryVoltage(void);
uint16_t getBatteryVoltageLatestADC(void);
uint16_t getBatteryWarningVoltage(void);
uint8_t getBatteryCellCount(void);
uint16_t getBatteryAverageCellVoltage(void);
uint32_t getBatteryRemainingCapacity(void);

bool isAmperageConfigured(void);
int32_t getAmperage(void);
int32_t getAmperageLatestADC(void);
int32_t getPower(void);
int32_t getMAhDrawn(void);
int32_t getMWhDrawn(void);

void currentMeterUpdate(int32_t lastUpdateAt);

void powerMeterUpdate(int32_t lastUpdateAt);

uint8_t calculateBatteryPercentage(void);
