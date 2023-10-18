#ifndef _APGEO_H_
#define _APGEO_H_

#include <stdint.h>
#include "types.h"

uint8_t ics_route_controller( double dt );
uint8_t ics_circle_controller( double dt );

void xgeo_pid_nu_ierr_reset(void);

float xgeo_get_alt();
//uint32_t x_dist(int32_t la, int32_t lo);
float x_get_vbat_compansation_factor();
int32_t x_az2p(int32_t la, int32_t lo);
int32_t x_az2p_cm(float x_tgt, float y_tgt);

uint32_t x_dist_m(int la, int lo);
uint32_t x_dist_2point_m(float x1, float y1, float x2, float y2);
float x_dist_2point_cm(float x1, float y1, float x2, float y2);
void x_preline_off();

void X_PL_Add();
int X_PL_GetIx();

void x_check_preline();
void x_clear_preline();
uint8_t x_is_preline_reached(double dt);


extern int8_t x_big_rotate;



typedef struct tag_GPOS{
    int la;
    int lo;
    int h;

    // our local coord system
    float local_x_pos;
    float local_y_pos;
    // inav coords system
    float inav_x_pos;
    float inav_y_pos;

    // home position
    int home_h;

    float az;
    float mag_az;
    uint16_t gndspd;
    uint16_t airspd;
    uint16_t cruise_spd;
    uint16_t used_spd;
    uint8_t is_use_airspeed;
    int hspd;

    int16_t need_az;
    uint32_t need_h;
    int16_t need_roll;
    int16_t need_pitch;
    int16_t need_nu;


    uint8_t manual_modes;

    int16_t alt_thro_pid_res;
    int16_t thro_pid_res;

    uint8_t gps_ok;
    uint8_t is_plane_mode;

    uint32_t warnings;
    int8_t launch_check;

    int override_target_h;
    uint8_t land_stage;
    uint16_t throttle_origin;

    float drift_area;
} TicsPos;

typedef struct tag_Origin{
    int la;
    int lo;
    float h;

    // координаты в нашей СК
    float x;
    float y;

    // heading
    float heading;
} TicsOrigin;

typedef struct tag_PLInfo{
    int la;
    int lo;
    int h;
    int16_t r,p,y;
    uint8_t is_nsu_received;
    rtcTime_t time;
    int gpsh_cm;
} TxPLInfo;

#define POLYGON_NONE (0)
#define POLYGON_PREPARE (1)
#define POLYGON_ONLINE (2)
#define POLYGON_FARAWAY (3)

//extern uint8_t X_Polygon_state;

void set_X_Polygon_state (uint8_t  newState);
uint8_t get_X_Polygon_state (void);
float x_get_drift_area( TicsOrigin origin, int prev_la, int prev_lo);


//extern x_PID_state cur_PID;
extern TicsPos cur_pos;
extern uint16_t X_PLCount;
#define MAX_PL_COUNT (1024)
TxPLInfo X_PL[MAX_PL_COUNT];

#endif
