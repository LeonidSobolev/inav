#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "drivers/exti.h"
#include "drivers/sensor.h"

#include "drivers/time.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"
#include "common/vector.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"

#include "fc/config.h"
#include "fc/fc_core.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/rc_curves.h"
#include "fc/runtime_config.h"
#include "fc/controlrate_profile.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/mixer.h"

#include "telemetry/mavlink.h"
#include "telemetry/xlogger.h"

#include "navigation/apgeo.h"
#include "navigation/navigation.h"
#include "navigation/icsnav.h"
#include "navigation/icsmission.h"

#include "io/gps.h"
#include "io/gps_private.h"

extern uint8_t navstate;

uint16_t X_PLCount = 0;
uint16_t X_PLSended = 0;
TxPLInfo X_PL[MAX_PL_COUNT];
//x_PID_state cur_PID;

XFloatPoint cur_circle_origin;
int8_t cur_circle_direction = 0;

int16_t xgeo_get_nu_on_circle( double dt );
int16_t xgeo_get_nu_from_dist(double dt, float dist, float need_course);
int32_t x_az_circle( XMissionPoint circle, double radius_m );
double xgeo_d2c( XMissionPoint circle, float radius_m );
int32_t x_next_az_line();
double angle_subtract( double a1, double a2 );
int16_t xgeo_get_nu_on_point();
int16_t xgeo_get_nu_to_start_line();
float ics_az_to_point( int32_t la, int32_t lo );
float ics_az_controler( double dt, int16_t cur_az_err );
int32_t x_az_line();
int16_t xgeo_get_nu_on_line( double dt );

void X_PL_Add(){
	if(X_PLCount < MAX_PL_COUNT ){
		X_PL[X_PLCount].la = cur_pos.la;
		X_PL[X_PLCount].lo = cur_pos.lo;
		X_PL[X_PLCount].h = cur_pos.h;
		X_PL[X_PLCount].r = attitude.values.roll+ systemConfig()->cx_camera_roll_offset;;
		X_PL[X_PLCount].p = attitude.values.pitch + systemConfig()->cx_camera_pitch_offset;
		X_PL[X_PLCount].y = attitude.values.yaw;
		X_PL[X_PLCount].is_nsu_received = 0;
		X_PL[X_PLCount].gpsh_cm = gpsSol.llh.alt;

		rtcGet(&(X_PL[X_PLCount].time));

		X_PLCount++;
		xlogger_set_payload_answer_time(X_PLCount);
	}
}

int X_PL_GetIx(){
    if( X_PLSended < X_PLCount ){
    	return X_PLSended++;
    }
    return -1;
}


float x_get_vbat_compansation_factor(){
	float res;
	uint16_t cur_vbat = getBatteryVoltage();
	if(cur_vbat < 30){
		return 1.0;
	}
	res = systemConfigMutable()->cx_full_voltage / 10.0 / cur_vbat;
	res = res * res;
	if( res > 1.5 ) return 1.5;
    if( res < 1.0 ) return 1.0;
    return res;
}




XMissionPoint mp;
XMissionPoint np;
fpVector3_t tmpcur;
int16_t nu = 0;
int8_t x_big_rotate = 0;
uint32_t mydist = 0;
int16_t plane_az;

uint8_t is_i_line = 0;

//Return distance WITH SIGN!
float xgeo_d2l_f( float x1, float y1, float x2, float y2){
	XFloatPoint plane_pos = XM_ToLocal( cur_pos.la, cur_pos.lo );

	float t1 = (y2 - y1) * plane_pos.x - (x2 - x1) * plane_pos.y + x2*y1 - y2*x1;
	float t2 = sqrtf( (y2-y1) * (y2-y1) + (x2 - x1) * (x2 - x1) );
	if( t2 < 1 ){
		return 0;
	}
    return t1 / t2;
}

//SIGN!
float xgeo_d2l(){
	return xgeo_d2l_f(XM_GetLineStart().x, XM_GetLineStart().y, XM_GetCurrent().x, XM_GetCurrent().y);
}

double xgeo_d2c( XMissionPoint circle, float radius_m ){
	XFloatPoint plane_pos = XM_ToLocal( cur_pos.la, cur_pos.lo );

	float D = 1.0 * x_dist_m( circle.la, circle.lo );
	float alpha = 0;
	if( D > 1.0f ){
	    alpha = radius_m / D;
	}
	if( alpha > 0.999 ){
		alpha = 0.999;
	}
	alpha = asin( alpha );

	return xgeo_d2l_f(plane_pos.x, plane_pos.y, plane_pos.x + (circle.x - plane_pos.x) * cos(alpha), plane_pos.y + (circle.y - plane_pos.y) * sin(alpha));
}

int32_t x_az_circle( XMissionPoint circle, double radius_m )
{
	XFloatPoint plane_pos = XM_ToLocal( cur_pos.la, cur_pos.lo );

	double D = 1.0 * x_dist_m( circle.la, circle.lo );
	double alpha = 0;
	if( D > 1.0 ){
	    alpha = radius_m / D;
	}
	if( alpha > 0.999 ){
		alpha = 0.999;
	}
	alpha = asin( alpha );



    return wrap_36000(RADIANS_TO_CENTIDEGREES(atan2_approx((circle.y - plane_pos.y) * sin(alpha), (circle.x - plane_pos.x) * cos(alpha))));
}

float override_target_h;

float xgeo_get_alt(){
	if( XM_Current == 0 ) return cur_pos.home_h;

	if( cur_pos.override_target_h > 0 ){
		return cur_pos.override_target_h - XM_GetHome().h;
	}

	//return XM_GetCurrent().h - XM_GetHome().h;
	if(( XM_Current == 1 ) || ( systemConfig()->cx_autonomous == 1 ) ){
		return XM_GetCurrent().h - XM_GetHome().h;
	}

	int32_t cur_h = XM_GetCurrent().h;
	int32_t prev_h = XM_GetLineStart().h;

//	//After line end we need choose max height between previous and next points
//	if( XM_GetLineStart().kind == XMISSION_KIND_LINE_END ){
//		if( cur_h > prev_h ) return cur_h - XM_GetHome().h;
//		return prev_h - XM_GetHome().h;
//	}

	//Small alt diff is not irrelevant
	if( fabs(1.0 * cur_h - prev_h) < 50.0 ){
		return XM_GetCurrent().h - XM_GetHome().h;
	}

	XFloatPoint plane_pos = XM_ToLocal( cur_pos.la, cur_pos.lo );
	XMissionPoint sl = XM_GetLineStart();
	XMissionPoint el = XM_GetCurrent();

	//Calculate triangle edges
    //float d2s = (plane_pos.x - sl.x) * (plane_pos.x - sl.x) + (plane_pos.y - sl.y) * (plane_pos.y - sl.y);
    float d2e = (plane_pos.x - el.x) * (plane_pos.x - el.x) + (plane_pos.y - el.y) * (plane_pos.y - el.y);
    float ds2e = (el.x - sl.x) * (el.x - sl.x) + (el.y - sl.y) * (el.y - sl.y);

    float k = sqrtf( d2e / ds2e );
    if( k > 1 ) k = 1;

    return (XM_GetCurrent().h *(1-k) + XM_GetLineStart().h * k) - XM_GetHome().h;
}

float d2s;
float d2e;
float ds2e;
float ds2e_sqrt;

XMissionPoint sl, el;
XFloatPoint plane_pos;
// X_polygon_state
uint8_t X_Polygon_state = POLYGON_NONE;

x_Flags_state cur_Flags;

void set_X_Polygon_state (uint8_t  newState) {
	// setter that allows write to xlogger
	X_Polygon_state = newState;

#if defined(USE_TELEMETRY_XLOGGER)
	if(cur_Flags.CX_flags.cx_polygon_state != newState) {
		cur_Flags.CX_flags.cx_polygon_state = newState;
    	cur_Flags.CX_flags.curTime = millis();
    	//cur_Flags.CX_flags.newData = 1;
    	// на развороте X_Polygon_state обновляется очень часто, причем несмотря на ограничение
    	// отключен триггер передачи
	}
#endif

}

uint8_t get_X_Polygon_state (void) {
	// getter for X_Polygon_state
	return X_Polygon_state;
}

//functions

int16_t xgeo_get_nu_on_circle( double dt ){
	//mp = XM_GetCurrent();
	int16_t res = xgeo_get_nu_on_point();

	//if we so far, go to first point

	plane_pos = XM_ToLocal( cur_pos.la, cur_pos.lo );

	//Set orto_line with two points K and L
	double Kx, Ky, Lx, Ly;
	double R = systemConfig()->cx_circle_radius / 100.0;

	double D = 1.0 * x_dist_2point_m( plane_pos.x, plane_pos.y, cur_circle_origin.x, cur_circle_origin.y );  //Distance to center

	if( D < 1.0 ) return 0;  // We in center, don't care

	//K=C+(P-C)R/D
	Kx = cur_circle_origin.x + (plane_pos.x - cur_circle_origin.x) * R / D;
	Ky = cur_circle_origin.y + (plane_pos.y - cur_circle_origin.y) * R / D;


	if( cur_circle_direction == 0 ){
		cur_circle_direction = 1;
	}

	Lx = Kx + (cur_circle_origin.y - Ky) * cur_circle_direction;
	Ly = Ky - (cur_circle_origin.x - Kx) * cur_circle_direction;


	//double dist_to_circle_cm = 100.0 * x_dist_m(mp.la, mp.lo);
	double dist_to_circle_cm = xgeo_d2l_f( Kx, Ky, Lx, Ly );

	//If so far, just fly
	if( dist_to_circle_cm > 200000.0 ){
		return res;
	}

//    double cur_center_course = x_az_line() / 10.0;
//    double cur_circle_course = x_az_circle( XM_GetCurrent() ) / 10.0;
//    int16_t cir_sign = (int16_t)(cur_center_course - cur_pos.az);
//    double cur_circle_dist = 1;
//    if( cir_sign < 0 ){
//    	cur_circle_dist = -1;
//    }
//    cur_circle_dist *= cur_circle_direction * (dist_to_circle_cm - cur_circle_radius);
    double az_circle = wrap_36000(RADIANS_TO_CENTIDEGREES(atan2_approx(Ly-Ky, Lx-Kx))) / 10.0;

    return xgeo_get_nu_from_dist(dt, dist_to_circle_cm, az_circle);
}

int16_t xgeo_get_nu_on_line( double dt ){
	//mp = XM_GetCurrent();
	int16_t res = xgeo_get_nu_on_point();

	//if we so far, go to first point
	float dist_to_line = xgeo_d2l();
	if( abs(dist_to_line) > (systemConfig()->cx_linefollow_active_distance)){
		is_i_line = 3;
		return xgeo_get_nu_to_start_line();
	}

	plane_pos = XM_ToLocal( cur_pos.la, cur_pos.lo );
	sl = XM_GetLineStart();

	el = XM_GetCurrent();

	//Calculate triangle edges
	d2s = 1.0 * (plane_pos.x - sl.x) * (plane_pos.x - sl.x) + 1.0 * (plane_pos.y - sl.y) * (plane_pos.y - sl.y);
	d2e = 1.0 * (plane_pos.x - el.x) * (plane_pos.x - el.x) + 1.0 * (plane_pos.y - el.y) * (plane_pos.y - el.y);
    ds2e = 1.0 * (el.x - sl.x) * (el.x - sl.x) + 1.0 * (el.y - sl.y) * (el.y - sl.y);

    //drop small lines
    if( ds2e < 90000.0){
    	is_i_line = 5;
    	// Think, never COPTER in this place
    	if( !XM_IsCopter() ){
    		XM_Reached();
    	}
    	return res;
    }

    ds2e_sqrt = sqrtf(ds2e);


    //For line-by-line and small angle we reached line rather
    double azl = x_az_line()/10.0;
    double nazl = x_next_az_line()/10.0;
    double azd = angle_subtract( azl, nazl );
    double cur_line_diff_abs = fabs( azd );

    if( (cur_line_diff_abs < ram_small_angle_limit) || (XM_GetCurrent().pdist == 0) ){
    	if( (X_Dist2_m() < (navConfig()->general.waypoint_radius)) ){
    		XM_Reached();
    		is_i_line = 3;
    		return xgeo_get_nu_on_point();
    	}
    }


    //We done this line
    float tmp = d2s - d2e - ds2e;
    if( tmp > 0 ){
        XM_Reached();
    	//Don't know what I must return:) Recalc new nu.
        is_i_line = 3;
    	return xgeo_get_nu_on_point();
    }

    //Prepare to line
    tmp = d2e - d2s - ds2e;
    if( tmp > 0 ){
    	//X_Polygon_state = POLYGON_PREPARE;
    	set_X_Polygon_state(POLYGON_PREPARE);
    	is_i_line = 2;
    	//If distance to point is very large, goto point directly. Else, we can goto line for best curves.
    	//if( abs(dist_to_line) > 10000.0f ){
    	  //return res;
    	//}
    } else {
    	//X_Polygon_state = POLYGON_ONLINE;
    	set_X_Polygon_state(POLYGON_ONLINE);
    }

    return xgeo_get_nu_from_dist(dt, dist_to_line, x_az_line() / 10.0);
}

float pid_prev_nu_err = 0;
float pid_nu_ierr = 0;

void xgeo_pid_nu_ierr_reset(void) {
	pid_nu_ierr = 0;
}


int16_t xgeo_get_nu_from_dist(double dt, float dist, float need_course){

#define MAX_NU_PID_I (100)
    //We are in workplace; Do PID
	float nu_on_line = constrainf( ics_pid(
            dt,
            1.0f * dist, &pid_prev_nu_err, &pid_nu_ierr,
            ram_linefollow_p,
            ram_linefollow_i,
            ram_linefollow_d,
            MAX_NU_PID_I
        ), -900, 900);


    float az_on_line = need_course;
    float need_az = az_on_line + nu_on_line;

#if defined(USE_TELEMETRY_XLOGGER)
    cur_PID.flight_azimuth.new = 1;
    cur_PID.flight_azimuth.curTime = millis();
    cur_PID.flight_azimuth.dist = dist;
    cur_PID.flight_azimuth.cx_linefollow_p_cont = cur_PID.buff_p_cont;
    cur_PID.flight_azimuth.cx_linefollow_i_cont = cur_PID.buff_i_cont;
    cur_PID.flight_azimuth.cx_linefollow_d_cont = cur_PID.buff_d_cont;
    cur_PID.flight_azimuth.nu_on_line = nu_on_line;
    cur_PID.flight_azimuth.need_course = need_course;
    cur_PID.flight_azimuth.need_az = need_az;
#endif


    int16_t res = (int16_t)(need_az - cur_pos.az);
	if( res > 1800 )  res -= 3600;
	if( res < -1800 )  res += 3600;
	is_i_line = 1;

	//X_Polygon_state = POLYGON_ONLINE;
	//set_X_Polygon_state(POLYGON_ONLINE);

	return res;
}

int16_t xgeo_get_nu_on_point(){
	mp = XM_GetCurrent();
	//tmpcur.x = mp.la;
	//tmpcur.y = mp.lo;

    int32_t az_to_point = x_az2p(mp.la, mp.lo) / 10.0;

	int16_t res = (int16_t)(az_to_point - cur_pos.az);
	if( res > 1800 )  res -= 3600;
	if( res < -1800 )  res += 3600;
	return res;
}

int16_t xgeo_get_nu_to_start_line(){
	mp = XM_GetLineStart();
	//tmpcur.x = mp.la;
	//tmpcur.y = mp.lo;

    int32_t az_to_point = x_az2p(mp.la, mp.lo) / 10.0;

	int16_t res = (int16_t)(az_to_point - cur_pos.az);
	if( res > 1800 )  res -= 3600;
	if( res < -1800 )  res += 3600;
	return res;
}

uint8_t is_preline_end(){
	return 0;
}

uint8_t ics_route_controller( double dt ){
    //Do not reach home point
//	if( XM_Current == 0 ){
//    	XM_Current = 1;
//    }

	if( (navstate != NAVSTATE_GOTO) && XM_IsLine() ){
		nu = xgeo_get_nu_on_line(dt);
	} else if( (navstate != NAVSTATE_GOTO) && XM_IsCircle() ){
		cur_circle_direction = 1;
		mp = XM_GetCurrent();
		cur_circle_origin.x = mp.x;
		cur_circle_origin.y = mp.y;

		nu = xgeo_get_nu_on_circle(dt);
	} else if( (navstate != NAVSTATE_GOTO) && XM_IsCopter() ){
		nu = xgeo_get_nu_on_point();
		is_i_line = 0;
		mydist = X_Dist2_m();
		if( mydist < navConfig()->general.waypoint_radius ){
			navstate = NAVSTATE_SET_ALT;
			return 1;
		}
	} else {
	    mydist = X_Dist2_m();

	    if( XM_GetCurrent().kind == XMISSION_KIND_LAND ){
		    if( mydist < (navConfig()->general.waypoint_radius / 2.0f) ){
		        XM_Reached();
		    }
	    } else {
		    if( mydist < navConfig()->general.waypoint_radius ){
		        XM_Reached();
		    }
	    }


		nu = xgeo_get_nu_on_point();
        is_i_line = 0;
	}
    if( nu > 1500 ){
        if( x_big_rotate == 0 ){
            x_big_rotate = 1;
        } else {
            nu = 900 * x_big_rotate;
        }
    } else if( nu < -1500 ){
        if( x_big_rotate == 0 ){
            x_big_rotate = -1;
        } else {
            nu = 900 * x_big_rotate;
        }
    } else {
        nu = constrain( nu, -900, 900 );
        x_big_rotate = 0;
    }
    
    cur_pos.need_nu = nu;
    
    ics_az_controler(dt, nu);
    return 1;
}

uint32_t r2 = 10000;

uint8_t ics_circle_controller( double dt ){
    //Do not reach home point
//	if( XM_Current == 0 ){
//    	XM_Current = 1;
//    }

	if( cur_circle_direction == 0 ){
		cur_circle_direction = 1;
		mp = XM_GetCurrent();
		cur_circle_origin.x = mp.x;
		cur_circle_origin.y = mp.y;
	}


	nu = xgeo_get_nu_on_circle(dt);

    if( nu > 1500 ){
        if( x_big_rotate == 0 ){
            x_big_rotate = 1;
        } else {
            nu = 900 * x_big_rotate;
        }
    } else if( nu < -1500 ){
        if( x_big_rotate == 0 ){
            x_big_rotate = -1;
        } else {
            nu = 900 * x_big_rotate;
        }
    } else {
        nu = constrain( nu, -900, 900 );
        x_big_rotate = 0;
    }

    cur_pos.need_nu = nu;

    ics_az_controler(dt, nu);

    return 1;
//	mydist = X_Dist2_m();
//
//	is_i_line = 6;
//
//	//soo far from circle, > 500m (500*500=250000)
//	if( mydist > r2 ){
//		nu = xgeo_get_nu_on_point();
//	} else if( mydist < 25 ){
//	//so close
//		return 0;
//	} else {
//	//in interest place
//		mp = XM_GetCurrent();
//		int32_t az_to_point = x_az2p(mp.la, mp.lo) / 10.0;
//		float r_gain = 1.0 * mydist / r2;
//		//r_gain in [1;0,001]
//		r_gain = (float)pow(r_gain, systemConfig()->cx_circ_power);
//		int32_t need_az = az_to_point - 900 * (1.0-r_gain);
//
//		int16_t res = (int16_t)(need_az - cur_pos.az);
//		if( res > 1800 )  res -= 3600;
//		if( res < -1800 )  res += 3600;
//		nu = res;
//	}
//
//    if( nu > 1500 ){
//        if( x_big_rotate == 0 ){
//            x_big_rotate = 1;
//        } else {
//            nu = 900 * x_big_rotate;
//        }
//    } else if( nu < -1500 ){
//        if( x_big_rotate == 0 ){
//            x_big_rotate = -1;
//        } else {
//            nu = 900 * x_big_rotate;
//        }
//    } else {
//        nu = constrain( nu, -900, 900 );
//        x_big_rotate = 0;
//    }
//
//    ics_az_controler(dt, nu);
//    return 1;
}


void x_preline_off(){
	cur_circle_direction = 0;
}

uint8_t preline_dir = 1;

void x_check_preline(){
	if( cur_circle_direction == 0){
		//default direction is right;
		preline_dir = 1;


		cur_circle_direction = 1;

					XFloatPoint prev_point;

					if( XM_Current >= 2 ){
						prev_point.x = XM_Get(XM_Current-2).x;
						prev_point.y = XM_Get(XM_Current-2).y;
						//In linear direction reversed
						if( XM_Get(XM_Current-1).kind == XMISSION_KIND_LINE_END){
							preline_dir *= -1;
						}
					} else {
						prev_point = XM_ToLocal( cur_pos.la, cur_pos.lo );
					}



		XMissionPoint L = XM_GetLineStart();
		XMissionPoint E = XM_GetCurrent();

		//Get distance between PP and Line:
		double D = 1.0 * x_dist_m( L.la, L.lo );  //Distance to center
		double R = systemConfig()->cx_circle_radius/100.0;




			double Cx, Cy, Clx, Cly;

			double scale = R / 1.0 / x_dist_2point_m(E.x, E.y, L.x, L.y);
			double dist_by_lane = 15.0;

			if( D < R ){
				dist_by_lane += 2*(R-D);
			}

			double scale_along = dist_by_lane / 1.0 / x_dist_2point_m(E.x, E.y, L.x, L.y);

			Cx = L.x - (E.x - L.x) * scale_along  - (E.y - L.y) * scale * cur_circle_direction;
			Cy = L.y - (E.y - L.y) * scale_along  + (E.x - L.x) * scale * cur_circle_direction;

			Clx = L.x - (E.x - L.x) * scale_along  - (E.y - L.y) * scale * cur_circle_direction * (-1);
			Cly = L.y - (E.y - L.y) * scale_along  + (E.x - L.x) * scale * cur_circle_direction * (-1);

			double d1 = x_dist_2point_m(Cx, Cy, prev_point.x, prev_point.y );
			double d2 = x_dist_2point_m(Clx, Cly, prev_point.x, prev_point.y );

			//Choose direction in polygon mode
			if( d1 < d2 ){
				preline_dir *= -1;
			}


			if( preline_dir == 1 ){
				cur_circle_direction = -1;
				cur_circle_origin.x = Clx;
				cur_circle_origin.y = Cly;
			} else {
				cur_circle_origin.x = Cx;
				cur_circle_origin.y = Cy;
			}


	}
}

double angle_subtract( double a1, double a2 ){
	double res = a1 - a2;
	if( res > 1800 )  res -= 3600;
	if( res < -1800 )  res += 3600;
	return res;
}

double cur_circle_integral_angle = 0;
double cur_circle_last_plane_az = 0;

void x_clear_preline(){
	cur_circle_direction = 0;
	cur_circle_integral_angle = 0;
	cur_circle_last_plane_az = cur_pos.az;
}

uint8_t x_is_preline_reached(double dt){

	double dd = angle_subtract(cur_pos.az,cur_circle_last_plane_az);

	if( fabs(dd) > 100 ){
		cur_circle_integral_angle += dd;
		cur_circle_last_plane_az = cur_pos.az;

		if( fabs(cur_circle_integral_angle) > 3600){
			x_clear_preline();
			return 1;
		}
	}

	//return 0;

	XMissionPoint L = XM_GetLineStart();
	XMissionPoint E = XM_GetCurrent();

	double dist_to_center = 1.0 * x_dist_m( L.la, L.lo );  //Distance to center




	if( dist_to_center < systemConfig()->cx_circle_radius * 2 / 100.0 ){
		double line_azimuth = x_az_line() / 10.0;

		double dd = angle_subtract( line_azimuth, cur_pos.az );

		float dist_to_line = xgeo_d2l();
		float dscale = dist_to_line / systemConfig()->cx_circle_radius;
		if( dscale > 1 ) dscale = 1;
		if( dscale < -1 ) dscale = -1;
		//Our destination angle to line
		double target_dd = dscale * 900.0;

		if( systemConfig()->cx_polygon_prepare_mode > 0 ){

		} else {
			target_dd = 0;
		}

		if( (dd < ram_small_angle_limit + target_dd) && (dd > -ram_small_angle_limit + target_dd) ){
			x_clear_preline();
			return 1;
		}


		//int16_t nu_to_line = xgeo_get_nu_from_dist(dt, dist_to_line, x_az_line() / 10.0);
	}

	return 0;
}


/**********************************************************
*****          Low level calculations
***********************************************************/

float deltaX = 0;
float deltaY = 0;

int32_t x_az2p(int32_t la, int32_t lo)
{
    deltaX = la - cur_pos.la;
    deltaY = (lo - cur_pos.lo) * cos_approx(cur_pos.la * 3.14 / 180.0 / 10000000);

    return wrap_36000(RADIANS_TO_CENTIDEGREES(atan2_approx(deltaY, deltaX)));
}

int32_t x_az2p_cm(float x_tgt, float y_tgt)
{
    deltaX = x_tgt - cur_pos.local_x_pos;
    deltaY = y_tgt - cur_pos.local_y_pos;

    return wrap_3600(RADIANS_TO_DECIDEGREES(atan2_approx(deltaY, deltaX)));
}


int32_t x_az_line()
{
	XMissionPoint sl = XM_GetLineStart();
	XMissionPoint el = XM_GetCurrent();

    return wrap_36000(RADIANS_TO_CENTIDEGREES(atan2_approx(el.y-sl.y, el.x-sl.x)));
}

int32_t x_next_az_line()
{
	XMissionPoint sl = XM_GetCurrent();
	XMissionPoint el = XM_GetNext();

    return wrap_36000(RADIANS_TO_CENTIDEGREES(atan2_approx(el.y-sl.y, el.x-sl.x)));
}

uint32_t x_dist_m(int la, int lo){
    #define ICS_ONE_DEGREE_LONG (1.113195f)
    float dx = (la - cur_pos.la) * ICS_ONE_DEGREE_LONG;
    float dy = (lo - cur_pos.lo) * ICS_ONE_DEGREE_LONG * cos_approx(cur_pos.la * 3.14 / 180.0 / 10000000);

    float res = sqrtf(dx*dx + dy*dy) / 100;

    return (uint32_t)(res);
}

uint32_t x_dist_2point_m(float x1, float y1, float x2, float y2){

    float res = sqrtf((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2)) / 100;

    return (uint32_t)(res);
}

float x_dist_2point_cm(float x1, float y1, float x2, float y2){
	// output in the same dimension as input. Inav has cm everywhere, so assume this as cm
    return (sqrtf((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2)));
}


/*****************************************
GEO functions 
*****************************************/

//float cur_az_err;
float prev_az_err;
float az_ierr;
float az_res;
float ics_az_controler( double dt, int16_t cur_az_err ){
    #define MAX_AZ_I_PART (500.0)
	az_res = ics_pid(
        dt, 
        1.0f * cur_az_err, &prev_az_err, &az_ierr,
        ram_nav_pid_p,
        ram_nav_pid_i,
        ram_nav_pid_d,
        MAX_AZ_I_PART
    ); 
//logger pid output
#if defined(USE_TELEMETRY_XLOGGER)
    cur_PID.flight_azimuth.az_res = az_res;
    cur_PID.flight_azimuth.cur_az_err = cur_az_err;
    cur_PID.flight_azimuth.cx_nav_pid_p_cont = cur_PID.buff_p_cont;
    cur_PID.flight_azimuth.cx_nav_pid_i_cont = cur_PID.buff_i_cont;
    cur_PID.flight_azimuth.cx_nav_pid_d_cont = cur_PID.buff_d_cont;
#endif
    //az_res = cur_az_err * systemConfig()->cx_nav_pid_p;
    cur_pos.need_roll = az_res;

    rcCommand[YAW] = 0;

	rcCommand[ROLL] = pidAngleToRcCommand( az_res, pidProfile()->max_angle_inclination[FD_ROLL] );

	if(
			(attitude.values.roll > systemConfig()->cx_yaw_mix_procent_roll_limit * 10)  ||
			(attitude.values.roll < -systemConfig()->cx_yaw_mix_procent_roll_limit * 10)
	){
		rcCommand[YAW] = - rcCommand[ROLL] * (systemConfig()->cx_yaw_mix_procent / 100.0);
	}

//	if( cur_az_err > ram_yaw_help_a){
//	  rcCommand[YAW] = -ram_yaw_help_v;
//	}
//	if( cur_az_err < -ram_yaw_help_a){
//	  rcCommand[YAW] = ram_yaw_help_v;
//	}
	return rcCommand[ROLL];
}

float x_get_drift_area( TicsOrigin origin, int prev_la, int prev_lo){
    #define ICS_ONE_DEGREE_LONG (1.113195f)
    float dx1 = (origin.la - cur_pos.la) * ICS_ONE_DEGREE_LONG;
    float dy1 = (origin.lo - cur_pos.lo) * ICS_ONE_DEGREE_LONG * cos_approx(cur_pos.la * 3.14 / 180.0 / 10000000);
    float dx2 = (origin.la - prev_la) * ICS_ONE_DEGREE_LONG;
    float dy2 = (origin.lo - prev_lo) * ICS_ONE_DEGREE_LONG * cos_approx(cur_pos.la * 3.14 / 180.0 / 10000000);

    float res = dx1*dy2 + dx2*dy1;

    return res;
}



