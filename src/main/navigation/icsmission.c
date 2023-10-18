
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "drivers/time.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/boardalignment.h"

#include "fc/config.h"
#include "fc/fc_core.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/rc_curves.h"
#include "fc/runtime_config.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/mixer.h"

#include "telemetry/mavlink.h"
#include "telemetry/xlogger.h"

#include "navigation/navigation.h"
#include "navigation/icsmission.h"
#include "navigation/icsnav.h"
#include "navigation/apgeo.h"

uint8_t cur_point;
x_Flags_state cur_Flags;

uint8_t is_mission_valid = 0;
uint8_t is_home_valid = 0;
float mission_full_dist = 0;
float mission_wp_dist = 0;

uint16_t XM_Count = 0;
uint16_t XM_Current = 0;
uint16_t XM_RCount = 0;
uint16_t XM_RCurrent = 0;

int32_t new_current_point = -1;

XMissionPoint xmission_received[GT_MISSION_CAPACITY];
XMissionPoint xmission[GT_MISSION_CAPACITY];

float origin_scale_factor =1.0f;


void set_XM_Current (uint16_t  newNumber) {
	// setter that allows write to xlogger
	XM_Current = newNumber;
#if defined(USE_TELEMETRY_XLOGGER)
	if(cur_Flags.CX_flags.cx_point_hdg != XM_Current) {
		cur_Flags.CX_flags.cx_point_hdg = XM_Current;
    	cur_Flags.CX_flags.curTime = millis();
    	cur_Flags.CX_flags.newData = 1;
	}
#endif
}


void XM_Next(){
	cur_pos.override_target_h = 0;
	if( (XM_Current + 1) < XM_Count ){
		set_XM_Current( XM_Current + 1 );
	}
}

void XM_Prev(){
	if( (XM_Current - 1) > 0 ){
		set_XM_Current( XM_Current - 1 );
	}

}

void XM_reset_local_systems(void) {
	// home point
		    XFloatPoint dL = XM_ToLocal(cur_pos.la, cur_pos.lo);
		    xmission[0].inavX =  dL.x - cur_pos.inav_x_pos;
		    xmission[0].inavY =  dL.y - cur_pos.inav_y_pos;
}

uint8_t XM_AddReceived( uint16_t cur, uint16_t size, int32_t la, int32_t lo, int32_t h, uint8_t kind, uint8_t p1, uint16_t pdist, uint8_t object_id, uint8_t object_type ){
	//first point -- it is home

	if( cur == 0 ){
		reset_baro_alt();

		xmission[0].la = la;
	    xmission[0].lo = lo;
	    //Absolute altitude
	    xmission[0].h = h;
	    xmission[0].kind = kind;
	    xmission[0].p1 = p1;
	    xmission[0].pdist = pdist;
	    xmission[0].object_id = object_id;
	    xmission[0].object_type = object_type;

	    origin_scale_factor = constrainf(cos_approx((ABS(la) / 10000000.0f) * 0.0174532925f), 0.01f, 1.0f);




	    //We need to recalculate all mission -- local coordinates
	    for( int i = 1; i < XM_Count; i++){
	        xmission[i].x = (xmission[i].la - xmission[0].la) * 1.113195f;
	        xmission[i].y = (xmission[i].lo - xmission[0].lo) * 1.113195f * origin_scale_factor;
	    }
	    //After change home position we must clear loading mission
	    XM_RCurrent = 0;


	    is_home_valid = 1;

	    return 1;
	}

	if(is_home_valid != 1){
		return 0;
	}

	if( cur == 1 ){
	    mission_full_dist = 0;
	    is_mission_valid = 0;

		if( size <= GT_MISSION_CAPACITY ){
			XM_RCount = size;
			XM_RCurrent = 1;
			is_mission_valid = 0;
	  }

    }
  
	//Prev point, already loaded
	if( cur == (XM_RCurrent-1)  ) return 1;

	if( (cur != XM_RCurrent)  ) return 0;
    if( cur >= GT_MISSION_CAPACITY ) return 0;
    
    xmission_received[XM_RCurrent].la = la;
    xmission_received[XM_RCurrent].lo = lo;
    xmission_received[XM_RCurrent].h = h;
    xmission_received[XM_RCurrent].alt_h = h - xmission_received[0].h;
    xmission_received[XM_RCurrent].kind = kind;
    xmission_received[XM_RCurrent].p1 = p1;
    xmission_received[XM_RCurrent].pdist = pdist;
    xmission_received[XM_RCurrent].object_id = object_id;
    xmission_received[XM_RCurrent].object_type = object_type;
//    if( XM_RCurrent == 1 ){
//        xmission_received[XM_RCurrent].x = 0;
//        xmission_received[XM_RCurrent].y = 0;
//        origin_scale_factor = constrainf(cos_approx((ABS(la) / 10000000.0f) * 0.0174532925f), 0.01f, 1.0f);
//    } else {
        xmission_received[XM_RCurrent].x = (xmission_received[XM_RCurrent].la - xmission[0].la) * 1.113195f;
        xmission_received[XM_RCurrent].y = (xmission_received[XM_RCurrent].lo - xmission[0].lo) * 1.113195f * origin_scale_factor;
    //}
    
    XM_RCurrent++;
    
    if( XM_RCurrent == XM_RCount ){
        if(XM_Count > XM_RCount){
            XM_Count = XM_RCount;
            if( XM_Current >= XM_Count ){
            	set_XM_Current(0);//XM_Current = 0; //! Error! Need RTH
            }
        }

        mission_full_dist = 0;

        for( uint16_t i = 1; i <  XM_RCount; i++ ){
        	xmission[i] = xmission_received[i];

        	float tmp = x_dist_between_points_m(i, i-1);
        	if( tmp > 100000.0 ){
        		is_mission_valid = 0;
        	}
        	mission_full_dist += tmp;

        }
        XM_Count = XM_RCount;
        if( XM_Current >= XM_Count ){
        	set_XM_Current(0);//XM_Current = 0; //! Error! Need RTH
        }

        if( (mission_full_dist  < (systemConfig()->cx_max_mission_len_km * 1000)) && (XM_Count > 2) ) {
          is_mission_valid = 1;
        } else {
		  is_mission_valid = 0;
        }
    } 
    return 1;
}    

uint8_t XM_SetCurrent( uint16_t new_point_number){
//    if( new_point_number == 0 ){
//    	new_point_number = 1;
//    }
    if( new_point_number >= XM_Count){
    	return 0;
    }

    //new_current_point = new_point_number;
    if( ics_continue_route()){
    	//XM_Current = new_point_number;
    	set_XM_Current(new_point_number);
    }
    return 1;
}

XMissionPoint XM_GetCurrent(){
	return xmission[XM_Current];
}

XMissionPoint XM_GetNext(){
	if( XM_Current + 1 < XM_Count)
		return XM_Get(XM_Current+1);
	return XM_Get(XM_Current);
}

XMissionPoint XM_Get(int ix){
	if( ix < 0 ){
		ix = XM_Count + ix;
	}
	if( ix >= XM_Count ){
		return xmission[0];
	}
	return xmission[ix];
}

XMissionPoint XM_GetHome(){
	return xmission[0];
}


XMissionPoint XM_GetPrelanding(){
	return XM_Get(-2);
}

XMissionPoint XM_GetLineStart(){
	if( XM_Current == 0 ){
		//this is unacceptable state
		return xmission[XM_Current];
	}
	return xmission[XM_Current-1];
}

XFloatPoint XM_ToLocal( int32_t la, int32_t lo){
	XFloatPoint res = {0,0};
	if( (XM_Count > 1) || (is_home_valid == 1)  ) {
		res.x = (la - xmission[0].la) * 1.113195f;
	    res.y = (lo - xmission[0].lo) * 1.113195f * origin_scale_factor;
    }
    return res;
}

uint8_t XM_IsLine(){
    if( XM_Current == 0 ) return 0;
	if( xmission[XM_Current].kind == XMISSION_KIND_LINE_END ) return 1;
	//if( xmission[XM_Current].kind == XMISSION_KIND_LAND ) return 1;
    return 0;
}


uint8_t XM_IsCopter(){
    if( XM_Current == 0 ) return 0;
	if( xmission[XM_Current].kind == XMISSION_KIND_COPTER) return 1;
	//if( xmission[XM_Current].kind == XMISSION_KIND_LAND ) return 1;
    return 0;
}

uint8_t XM_IsCircle(){
    if( XM_Current == 0 ) return 0;
	if( xmission[XM_Current].kind == XMISSION_KIND_CIRCLE_INDEF ) return 1;
	//if( xmission[XM_Current].kind == XMISSION_KIND_LAND ) return 1;
    return 0;
}

void XM_Reached(){
    // DO ACTION 

	XM_Next();

	if( XM_Current >= XM_Count - 1 ){
		ics_start_land(0);
		return;
	}

	if( ram_my_type == PLANE ){
		if( XM_Current < XM_Count - 2 ){
			if( (XM_Current == XM_Count - 2)  ){
				xmission[XM_Current].kind = XMISSION_KIND_POINT;
			} else {
				if(XM_GetCurrent().kind == XMISSION_KIND_LINE_START){
					//Do not skip first point
					if(XM_Current > 1) XM_Next();
					if(XM_GetCurrent().pdist != 0){
						ics_start_preline();
						return;
					}

				}
				if(XM_GetCurrent().kind == XMISSION_KIND_LINE_END){
					if(XM_GetCurrent().pdist != 0){
						ics_start_preline();
						return;
					}

				}
			}
		}
	}

	ics_start_route();

}

uint8_t XM_IsPenult(){
    if( XM_Current + 2 == XM_Count ) return 1;
    return 0;
}

uint32_t X_Dist2_m(){
//uint32_t x_dist(int32_t la, int32_t lo)
    #define ICS_ONE_DEGREE_LONG (1.113195f)    
    float dx = (xmission[XM_Current].la - cur_pos.la) * ICS_ONE_DEGREE_LONG;
    float dy = (xmission[XM_Current].lo - cur_pos.lo) * ICS_ONE_DEGREE_LONG * cos_approx(cur_pos.la * 3.14 / 180.0 / 10000000);

    return (int32_t)((dx*dx + dy*dy) / 10000);
}


uint32_t x_dist_between_points_m(uint16_t cur_point1, uint16_t cur_point2 ){
    float dx = (xmission[cur_point1].la - xmission[cur_point2].la) * ICS_ONE_DEGREE_LONG;
    float dy = (xmission[cur_point1].lo - xmission[cur_point2].lo) * ICS_ONE_DEGREE_LONG * cos_approx(xmission[cur_point2].la * 3.14 / 180.0 / 10000000);

    return (int32_t)(sqrtf((dx*dx + dy*dy)) / 100);
}

