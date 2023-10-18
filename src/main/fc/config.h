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

#include <stdint.h>
#include <stdbool.h>
#include "common/time.h"
#include "config/parameter_group.h"
#include "drivers/adc.h"
#include "drivers/rx_pwm.h"
#include "fc/stats.h"

#define MAX_PROFILE_COUNT 3
#define ONESHOT_FEATURE_CHANGED_DELAY_ON_BOOT_MS 1500
#define MAX_NAME_LENGTH 16

#define ACC_TASK_FREQUENCY_DEFAULT 500
#define ACC_TASK_FREQUENCY_MIN 100
#define ACC_TASK_FREQUENCY_MAX 1000
#define ATTITUDE_TASK_FREQUENCY_DEFAULT 250
#define ATTITUDE_TASK_FREQUENCY_MIN 100
#define ATTITUDE_TASK_FREQUENCY_MAX 1000

typedef enum {
    ASYNC_MODE_NONE,
    ASYNC_MODE_GYRO,
    ASYNC_MODE_ALL
} asyncMode_e;

typedef enum {
    FEATURE_UNUSED_1 = 1 << 0,          // RX_PPM
    FEATURE_VBAT = 1 << 1,
    FEATURE_TX_PROF_SEL = 1 << 2,       // Profile selection by TX stick command
    FEATURE_UNUSED_2 = 1 << 3,          // RX_SERIAL
    FEATURE_MOTOR_STOP = 1 << 4,
    FEATURE_SERVO_TILT = 1 << 5,
    FEATURE_SOFTSERIAL = 1 << 6,
    FEATURE_GPS = 1 << 7,
    FEATURE_UNUSED_3 = 1 << 8,          // was FEATURE_FAILSAFE
    FEATURE_UNUSED_4 = 1 << 9,          // was FEATURE_SONAR
    FEATURE_TELEMETRY = 1 << 10,
    FEATURE_CURRENT_METER = 1 << 11,
    FEATURE_3D = 1 << 12,
    FEATURE_UNUSED_5 = 1 << 13,         // RX_PARALLEL_PWM
    FEATURE_UNUSED_6 = 1 << 14,         // RX_MSP
    FEATURE_RSSI_ADC = 1 << 15,
    FEATURE_LED_STRIP = 1 << 16,
    FEATURE_DASHBOARD = 1 << 17,
    FEATURE_UNUSED_7 = 1 << 18,         // Unused in INAV
    FEATURE_BLACKBOX = 1 << 19,
    FEATURE_CHANNEL_FORWARDING = 1 << 20,
    FEATURE_TRANSPONDER = 1 << 21,
    FEATURE_FAILOFF = 1 << 22,
    FEATURE_SUPEREXPO_RATES = 1 << 23,
    FEATURE_VTX = 1 << 24,
    FEATURE_UNUSED_8 = 1 << 25,         // RX_SPI
    FEATURE_UNUSED_9 = 1 << 26,        //SOFTSPI
    FEATURE_PWM_SERVO_DRIVER = 1 << 27,
    FEATURE_PWM_OUTPUT_ENABLE = 1 << 28,
    FEATURE_OSD = 1 << 29,
    FEATURE_FW_LAUNCH = 1 << 30,
    FEATURE_DEBUG_TRACE = 1 << 31,
} features_e;

typedef struct systemConfig_s {
    uint16_t accTaskFrequency;
    uint16_t attitudeTaskFrequency;
    uint8_t current_profile_index;
    uint8_t asyncMode;
    uint8_t debug_mode;
    uint8_t i2c_speed;
    uint8_t cpuUnderclock;
    uint8_t throttle_tilt_compensation_strength;      // the correction that will be applied at throttle_correction_angle.
    inputFilteringMode_e pwmRxInputFilteringMode;
    int16_t cx_planemode_pitch_angle_decidegi;
    uint8_t cx_emulator_mode;
    uint32_t cx_debug;
    uint32_t  cx_linefollow_active_distance;
    uint8_t cx_payload_A;
    uint16_t cx_payload_B;    
    uint8_t cx_payload_always_on;
    uint16_t cx_payload_off_distance_cm;   
    uint8_t  cx_land_brake_enable;    
    uint16_t cx_land_brake_distance2_m;
    uint16_t cx_land_brake_throttle_cruise_reduction;
    uint16_t cx_landing_poshold_enable;
    uint16_t cx_launch_height;
    uint16_t cx_launch_height_mid;
    uint16_t cx_launch_destangle_dd;
    uint16_t cx_launch_start_throttle;
    uint8_t cx_use_yaw_controller;
    uint8_t cx_landing_version;
    float cx_alt_spd_pid_p;
    float cx_alt_spd_pid_i;
    float cx_alt_spd_pid_d;

    uint16_t cx_cruise_as;
    float cx_as_pid_p;    
    float cx_as_pid_i;    
    float cx_as_pid_d;    
    uint8_t cx_use_plane_mode;
    uint8_t cx_use_new_nav;    
    float cx_alt_pid_p;
    float cx_alt_pid_i;
    float cx_alt_pid_d;
    float cx_altmc_pid_p;
    float cx_altmc_pid_i;
    float cx_altmc_pid_d;
    float cx_nav_pid_p;
    float cx_nav_pid_i;
    float cx_nav_pid_d;    
    float cx_ph_pid_p;
    float cx_ph_pid_i;
    float cx_ph_pid_d;
    float cx_pr_pid_p;
    float cx_pr_pid_i;
    float cx_pr_pid_d;
    uint8_t cx_use_gps_az;
    uint16_t cx_cruise_throttle;
    uint16_t cx_max_route_throttle;
    uint8_t cx_plane_or_copter;
    int16_t cx_fly_maxpitch_at_0ms;
    int16_t cx_fly_maxpitch_at_20ms;
    float cx_linefollow_p;
    float cx_linefollow_i;
    float cx_linefollow_d;

    float cx_alt_t_p;
    float cx_alt_t_i;
    float cx_alt_t_d;
    uint16_t cx_full_voltage;
    float cx_circ_power;
    uint8_t cx_ae_infl;
    uint16_t cx_ae_infl_threshold_roll;

    float cx_land_detector_sens;

    float cx_desc_pid_p;
    float cx_desc_pid_i;
    float cx_desc_pid_d;

    uint16_t cx_landing_stage_time;
    int16_t cx_landing_stage_alt;
    uint8_t cx_landing_stage_count;
    uint16_t cx_landing_radius;

    uint16_t cx_land_detector_filter_t;
    uint16_t cx_land_detector_filter_h;
	uint16_t cx_land_detector_filter_als;

    uint16_t cx_landing_max_roll;
    uint16_t cx_landing_max_climb;
    uint16_t cx_landing_max_dive;


    float cx_batt_low_value;
    float cx_batt_empty_value;
    float cx_batt_filter_time;
    float cx_batt_launch_min_value;

	uint8_t cx_land_type;

	uint8_t cx_yaw_poshold;
	uint16_t cx_max_poshold_tail_throttle;

	uint16_t cx_land_detector_max_gps_speed;

	uint32_t cx_circle_radius;
	uint8_t cx_autonomous;
	uint32_t cx_autonomous_height;
	uint32_t cx_autonomous_prelanding_height;

	uint32_t cx_polygon_prepare_mode;

	uint16_t cx_telemetry_period_ms;
	uint16_t cx_max_mission_len_km;
	uint8_t cx_gps_lost_test_mode;

	uint8_t cx_reduce_mc_in_plane;
	uint8_t cx_allow_plane_in_manual;
	int16_t cx_plane_switch_speed;

	uint16_t cx_small_angle_limit;

	uint16_t cx_version;

	uint8_t cx_use_only_gps_speed;
    
    uint8_t cx_land_by_speed;
    uint16_t cx_land_speed;
    int16_t cx_land_speed_outside_radius;
    uint16_t cx_land_careful_speed;
    int16_t cx_land_careful_speed_outside_radius;

    uint8_t cx_land_careful_height;       

    uint16_t cx_land_careful_max_roll;
    
    uint8_t cx_rangefinder_height;
    float cx_rangefinder_time;

    float cx_keyboard_la_scale;
    float cx_keyboard_lo_scale;
    float cx_keyboard_h_scale;

    int16_t cx_servo_parking_value;

    int16_t cx_camera_roll_offset;
    int16_t cx_camera_pitch_offset;
    uint8_t cx_land_detector_enable_h;


    uint8_t cx_xlogger_rcData_channels;
    uint32_t cx_xlogger_min_period_us;
    uint32_t cx_xlogger_heartbeat_period_us;

    int16_t cx_cam_ant_offset_left;
    int16_t cx_cam_ant_offset_top;
    int16_t cx_cam_ant_offset_ground;

    uint8_t cx_launch_check_height;
    uint16_t cx_launch_check_time;
    uint8_t cx_launch_kind;
    uint16_t cx_launch_hover_throttle;
    uint8_t cx_unit_number;
    uint16_t cx_launch_speed;
    float cx_launch_acceleration_gain;

    float cx_py_pid_p;
	uint8_t cx_py_pid_i;
	uint8_t cx_py_pid_d;

	uint8_t cx_yaw_poshold_roll_compensation;

	uint32_t cx_max_gyro_lag_time_us;

	uint16_t cx_max_telemetry_lag_for_rth_s;

	int16_t cx_rth_safe_h;
	uint8_t cx_telemetry_lost_rth_or_land;
	uint16_t cx_check_motor_pwm;

	uint8_t cx_prelanding_altspeed_enable;
	uint16_t cx_altspeed_gain;
    int16_t cx_yaw_mix_procent;
    uint8_t cx_yaw_mix_procent_roll_limit;
    int16_t cx_pitch_mix_procent;
    uint8_t cx_pitch_mix_procent_roll_limit;
    int16_t cx_throttle_mix_procent;
    uint8_t cx_throttle_mix_procent_roll_limit;

    uint8_t cx_use_baro_or_filter_altspd;

    uint8_t  cx_alt_reach_m;
    uint8_t cx_max_landing_pause;

    uint16_t cx_altspeed_drop;


    /*----new althold params----*/
    float 		cx_new_althold_alt_pid_p;
    uint16_t 	cx_new_althold_altspeed_max_cms;
    uint8_t 	cx_new_althold_use_origin;
    uint16_t 	cx_new_althold_rc_throttle_deadband;
    float 		cx_new_althold_sqcomp_gain;
    float 		cx_new_althold_speed_hor_pid_maxI;

    /*----new poshold params----*/
    uint8_t cx_use_new_poshold;

    float cx_new_poshold_speed_x_lim_plus;
    float cx_new_poshold_speed_x_lim_minus;
    float cx_new_poshold_speed_x_pid_p;
    float cx_new_poshold_speed_x_pid_i;
    float cx_new_poshold_speed_x_pid_d;

    float cx_new_poshold_speed_y_lim;
    float cx_new_poshold_speed_y_pid_p;
    float cx_new_poshold_speed_y_pid_i;
    float cx_new_poshold_speed_y_pid_d;

    float cx_new_poshold_pos_x_pid_p;
    float cx_new_poshold_pos_y_pid_p;

    uint16_t	cx_new_poshold_pitchroll_deadband;
    uint16_t 	cx_new_poshold_manual_dpos_cm;
    float 		cx_new_poshold_yaw_radius;
    uint8_t 	cx_new_poshold_use_heading_hold;
    float 		cx_new_poshold_heading_hold_max_as_sm;
    float 		cx_new_poshold_heading_hold_rate_I_corr;
    float 		cx_new_poshold_speed_hor_pid_maxI;


    uint8_t cx_use_Iterm_reset_mode;



    int16_t cx_launch_type_four_max_alt_err;
    int16_t cx_launch_route_acceleration_gain_s;
    int16_t cx_launch_speed_border_ms;
    uint8_t cx_launch_heading_hold_enable;

    uint16_t cx_min_as_for_comp_az;
    uint16_t cx_min_gs_for_comp_az;

    

    uint8_t cx_rangefinder_check_in_flight;
    uint8_t cx_rangefinder_noise_filter;
    float cx_rangefinder_noise_max_time;

    float cx_yaw_rate_Iterm_limit;

    uint8_t cx_min_telemetry_lag_for_rth_s;
    float cx_min_tkoff_land_now_alt;

    float cx_ptsV_0;
    float cx_ptsV_1;
    float cx_ptsV_2;
    float cx_ptsV_3;
    float cx_ptsV_4;
    float cx_ptsV_5;

    float cx_ptsC_0;
    float cx_ptsC_1;
    float cx_ptsC_2;
    float cx_ptsC_3;
    float cx_ptsC_4;
    float cx_ptsC_5;

    uint8_t cx_preland_use_current_height;

    uint8_t as_for_zero_dive;

    int16_t cx_moca_m;

    float cx_yaw_prelaunch_test_time;
    float cx_yaw_prelaunch_test_value;


	char name[MAX_NAME_LENGTH + 1];
} systemConfig_t;

extern uint8_t ram_reduce_mc_in_plane;
extern uint8_t ram_allow_plane_in_manual;
extern int16_t ram_plane_switch_speed;

extern float ram_linefollow_p;
extern float ram_linefollow_i;
extern float ram_linefollow_d;

extern float ram_nav_pid_p;
extern float ram_nav_pid_i;
extern float ram_nav_pid_d;

int16_t ram_yaw_help_a;
int16_t ram_yaw_help_v;

uint8_t ram_use_only_gps_speed;

uint16_t ram_max_telemetry_lag_for_rth_s;
int16_t ram_rth_safe_h;

uint32_t ram_circle_radius;

extern uint16_t ram_small_angle_limit;

PG_DECLARE(systemConfig_t, systemConfig);

typedef struct beeperConfig_s {
    uint32_t beeper_off_flags;
    uint32_t preferred_beeper_off_flags;
} beeperConfig_t;

PG_DECLARE(beeperConfig_t, beeperConfig);

typedef struct adcChannelConfig_s {
    uint8_t adcFunctionChannel[ADC_FUNCTION_COUNT];
} adcChannelConfig_t;

PG_DECLARE(adcChannelConfig_t, adcChannelConfig);


#ifdef USE_STATS
PG_DECLARE(statsConfig_t, statsConfig);
#endif

void beeperOffSet(uint32_t mask);
void beeperOffSetAll(uint8_t beeperCount);
void beeperOffClear(uint32_t mask);
void beeperOffClearAll(void);
uint32_t getBeeperOffMask(void);
void setBeeperOffMask(uint32_t mask);
uint32_t getPreferredBeeperOffMask(void);
void setPreferredBeeperOffMask(uint32_t mask);

void copyCurrentProfileToProfileSlot(uint8_t profileSlotIndex);

void initEEPROM(void);
void resetEEPROM(void);
void readEEPROM(void);
void writeEEPROM(void);
void ensureEEPROMContainsValidData(void);

void saveConfigAndNotify(void);
void validateAndFixConfig(void);

uint8_t getConfigProfile(void);
bool setConfigProfile(uint8_t profileIndex);
void setConfigProfileAndWriteEEPROM(uint8_t profileIndex);

bool canSoftwareSerialBeUsed(void);
void applyAndSaveBoardAlignmentDelta(int16_t roll, int16_t pitch);

void createDefaultConfig(void);
void resetConfigs(void);
void targetConfiguration(void);

uint32_t getPidUpdateRate(void);
timeDelta_t getGyroUpdateRate(void);
uint16_t getAccUpdateRate(void);
#ifdef USE_ASYNC_GYRO_PROCESSING
uint16_t getAttitudeUpdateRate(void);
uint8_t getAsyncMode(void);
#endif
