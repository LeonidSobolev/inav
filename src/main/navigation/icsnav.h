#pragma once

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>

#include "platform.h"


#define NAVSTATE_OFF (0)
#define NAVSTATE_LAUNCH (1)
#define NAVSTATE_ROUTE (2)
#define NAVSTATE_LAND (3)
#define NAVSTATE_RTH (4)
#define NAVSTATE_WAIT_CIRCLE (6)
#define NAVSTATE_STABILIZE (5)
#define NAVSTATE_POSHOLD (7)
#define NAVSTATE_PRELAUNCH (8)
#define NAVSTATE_LANDED (9)
#define NAVSTATE_DESCEND (10)
#define NAVSTATE_RTH_LAND (11)
#define NAVSTATE_PRELAND_1 (12)
#define NAVSTATE_PRELAND_2 (13)
#define NAVSTATE_PRELAND_3 (14)
#define NAVSTATE_LAND_WO_POSHOLD (15)
#define NAVSTATE_PRELINE (16)
#define NAVSTATE_GPSLOST (17)
#define NAVSTATE_GOTO (18)
#define NAVSTATE_SET_ALT (19)
#define NAVSTATE_ALTHOLD (20)
//skipped some modes from 308-350 FW
#define NAVSTATE_CAREFULL_LAND (30)
#define NAVSTATE_TELEMETRY_RTH (31)


#define NAVSTATE_CHECK_SERVO (100)
#define NAVSTATE_AUTONOMOUS_PREPARING (150)
//Do not forget to modify:
uint8_t PCMODE( uint16_t curmode );

#define WNG_RANGEFINDER_BROKE (1)
#define WNG_LAUNCH_BROKE (2)
#define WNG_LOWBATT_LAUNCH (4)
#define WNG_LOWBATT_ROUTE (8)
#define WNG_LOWBATT_CRITICAL (16)
#define WNG_GPS_LOST (32)
#define WNG_GYRO_LAG (64)
#define WNG_TELEMETRY_LOST (128)
#define WNG_MANUAL_ON (256)
#define WNG_PAUSE_ON (512)
#define WNG_YAW_MIX_ON (1024)



void ics_nav_controller( timeUs_t currentTimeUs );



uint8_t ics_get_state();
uint8_t ics_start();
uint8_t ics_stop();
uint8_t ics_pause();
uint8_t ics_start_rth();
uint8_t ics_start_descend();
uint8_t ics_start_preline();
uint8_t ics_start_route();
uint8_t ics_start_goto( uint16_t new_point_number );

uint8_t ics_goto_origin( int la, int lo, int h );

uint8_t ics_start_telemetry_lost();

uint8_t ics_continue_route();
uint8_t ics_continue_from_pause();
uint8_t ics_start_land(uint8_t is_emergency_landing);

uint8_t ics_on_vector_mode_message( uint8_t enabled, int32_t d_az, int32_t d_as, int32_t d_h );
 
void ics_check_servos( double dt );
int ics_init();

void payloadExtraShot();

float ics_pid( float dt, float err, float *olderr, float *ierr, float p, float i, float d, float maxi );
float ics_pid_known_der( float dt, float err, float *olderr, float *ierr, float p, float i, float d, float maxi, float der );

void XM_setHome(void);
void CP_INAV_TO_LOCAL(void);

uint8_t ics_navstate_use_new_poshold(void);
uint8_t ics_get_yaw_Iterm_reset(void);

void ics_set_warning(uint32_t new_warning, uint8_t off_on);
