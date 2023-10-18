#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "drivers/time.h"
#include "drivers/nvic.h"
#include "drivers/io.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/boardalignment.h"
#include "sensors/gyro.h"

#include "rx/rx.h"

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
#include "flight/servos.h"
#include "flight/imu.h"

#include "telemetry/mavlink.h"
#include "telemetry/xlogger.h"

#include "navigation/icsmission.h"
#include "navigation/apgeo.h"
#include "navigation/icsnav.h"
#include "navigation/navigation.h"
#include "navigation/navigation_private.h"

#include "flight/payload.h"
#include "io/beeper.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/rangefinder.h"

#include "ver.h"

/*****
 * 		RAM PARAM
 */

float ram_altmc_pid_p;
float ram_altmc_pid_i;
float ram_altmc_pid_d;

uint8_t ram_alt_reach_m;

uint8_t ram_my_type;

uint8_t ram_reduce_mc_in_plane;
uint8_t ram_allow_plane_in_manual;
int16_t ram_plane_switch_speed;

float ram_linefollow_p;
float ram_linefollow_i;
float ram_linefollow_d;

float ram_nav_pid_p;
float ram_nav_pid_i;
float ram_nav_pid_d;

uint16_t ram_small_angle_limit;


float cur_alts_err;
float prev_alts_err;
float alts_ierr;
float alts_res;

float speed_X_err_curr,	speed_X_err_prev, speed_X_ierr, speed_X_res;
float speed_Y_err_curr,	speed_Y_err_prev, speed_Y_ierr, speed_Y_res;


void ics_idle( double dt );
void ics_landed( double dt );
void ics_prelaunch( double dt );
void ics_launch( double dt );
void ics_route( double dt );
void ics_preline( double dt );
void ics_land( double dt, uint8_t disable_poshold );
void ics_set_alt( double dt );
void ics_care_land( double dt );
void check_rangefinder( double dt );
void ics_preland( double dt, uint8_t stage );
double ics_is_landed( double dt );
void ics_circle( double dt );
void ics_descend( double dt );
void ics_stab( void );
void ics_poshold_2d(  double dt, uint8_t allow_sticks, uint8_t is_careful);
void ics_althold_1d(  double dt, uint8_t allow_sticks  );
uint16_t get_cruise_as( void );
uint16_t get_cur_as(void);

float ics_as_controler( double dt, uint16_t need_as_in_cm, uint32_t need_alt );
float ics_alt_controler( double dt, int32_t need_alt );
float ics_alt_mc_controler( double dt, int32_t need_alt );
float ics_alt_speed_controler( double dt, int16_t need_alt_speed );
void ics_process_rth_land( double dt );
float ics_descend_controler( double dt, uint32_t need_as );
void ics_update_batt_monitor(void);
uint8_t ics_goto_land();
uint8_t ics_do_monitoring( double dt );
uint8_t ics_start_gpslost();

void ics_nav_controller( timeUs_t currentTimeUs );
uint8_t ics_route_controller( double dt );

int32_t get_need_alt(void);

void clear_all_filters(void);

void check_rangefinder_in_flight( double dt );


uint8_t ics_apply_manual_althold_control( double dt);
uint8_t ics_new_althold_controller( double dt, int32_t need_alt, int32_t alt_speed_min, int32_t alt_speed_max );


void ics_new_poshold_heading_hold_controller(double dt);
void ics_new_poshold_hor_spd_controller(double dt, float speed_X_tgt, float speed_Y_tgt);
float ics_new_poshold_pos_controller(double dt, uint8_t allow_sticks);
uint8_t ics_apply_manual_poshold_control( double dt) ;

void check_drift();


// cx_navstate
uint8_t navstate = NAVSTATE_OFF;
uint8_t prevstate = NAVSTATE_OFF;
uint8_t is_pause_landing = 0;
double landing_pause_time = 0;


uint8_t PCMODE( uint16_t curmode ){
	if( curmode ==  NAVSTATE_ROUTE ) return PLANE;
	if( curmode ==  NAVSTATE_RTH) return PLANE;
	if( curmode ==  NAVSTATE_WAIT_CIRCLE) return PLANE;
	if( curmode ==  NAVSTATE_DESCEND) return PLANE;
	if( curmode ==  NAVSTATE_RTH_LAND) return PLANE;
	if( curmode ==  NAVSTATE_PRELINE) return PLANE;
	if( curmode ==  NAVSTATE_GPSLOST) return PLANE;
	if( curmode ==  NAVSTATE_GOTO) return PLANE;
	if( curmode ==  NAVSTATE_TELEMETRY_RTH) return PLANE;
	return COPTER;
}

void ics_set_state (uint8_t  newState) {
	if( newState == navstate ) return;
	//Clear integral part when plane-copter mode switch
	if( PCMODE(navstate) != PCMODE(newState) ){
		clear_all_filters();
	}

	// setter that allows write to xlogger
	navstate = newState;


#if defined(USE_TELEMETRY_XLOGGER)
	if(cur_Flags.CX_flags.cx_navstate != navstate) {
		// на случай, если постоянно отправляется одно и то же
		// TODO: сделать то же самое для остальных сеттеров
		cur_Flags.CX_flags.cx_navstate = navstate;
		cur_Flags.CX_flags.curTime = millis();
		cur_Flags.CX_flags.newData = 1;
	}
#endif
}

uint8_t ics_get_state(){
    return navstate;
}

void ics_set_warning(uint32_t new_warning, uint8_t off_on) {
	// this function turns on or off warnings by bits.
	// off_on = 1 turns the flag on
	// off_on = 0 turns the flag off
	if(off_on){
        cur_pos.warnings |= new_warning;
    }
	else{
        cur_pos.warnings &= ~new_warning;
    }
#if defined(USE_TELEMETRY_XLOGGER)
	if(cur_Flags.CX_flags.warnings != cur_pos.warnings) {
		// на случай, если постоянно отправляется одно и то же
		cur_Flags.CX_flags.warnings = cur_pos.warnings;
		cur_Flags.CX_flags.curTime = millis();
		cur_Flags.CX_flags.newData = 1;
	}
#endif

}


void ics_set_manual_modes(uint32_t new_manual_modes) {

	if		(new_manual_modes == 0)	cur_pos.manual_modes = 0;
	else if	(new_manual_modes == 1)	cur_pos.manual_modes = 1;
	else	cur_pos.manual_modes |= new_manual_modes;

#if defined(USE_TELEMETRY_XLOGGER)
	if(cur_Flags.CX_flags.manual_modes != cur_pos.manual_modes) {
		// на случай, если постоянно отправляется одно и то же
		cur_Flags.CX_flags.manual_modes = cur_pos.manual_modes;
		cur_Flags.CX_flags.curTime = millis();
		cur_Flags.CX_flags.newData = 1;
	}
#endif

}


uint8_t is_as_error = 0;

//x_PID_state cur_PID;
TicsPos cur_pos;
uint8_t cur_point = 0;

timeUs_t prevTimeUs = 0, ics_currentTimeUs = 0;

int payload_init();
void payloadExtiHandler(extiCallbackRec_t *cb);

extiCallbackRec_t pl_exti;
IO_t pl_exti_pin;
IO_t pl_shot_pin;

float cur_alt_change = 0;

int last_shot_pos_lo = 0;
int last_shot_pos_la = 0;

int last_tg_pos_lo = 0;
int last_tg_pos_la = 0;


double stay_time = 0;
double preland_time = 0;
float acc_raw_abs = 0;


int8_t rangefinder_mode = 0;

/*PID VARIABLES*/
float desc_err, desc_ierr = 0, prev_desc_err;

uint8_t payload_is_on = 0;

void payloadExtiHandler(extiCallbackRec_t *cb){
	if( payload_is_on > 0 ){
		X_PL_Add();
		last_shot_pos_la = cur_pos.la;
		last_shot_pos_lo = cur_pos.lo;
	}
}

uint8_t is_need_shot = 0;
uint8_t is_need_extra_shot = 1;
uint8_t is_need_extra_tg = 0;
int payload_state = -1;

double time_impulse = 0;
double time_impulse_relax = 0;

void payloadExtraShot(){
	payload_is_on = 1;
	is_need_extra_shot = 1;
	// is_need_extra_tg = 1;
}


uint8_t tg_state = 0;
double tg_pulse = 0;
uint16_t cur_pldist = 0;
uint8_t is_shot_cond = 0;

#define ENABLE_TG (1)

void payloadCheckShot(double dt){
	
#if ENABLE_TG == 0
    if( tg_state > 0){
		if( tg_state == 1){
			tg_pulse += dt;
			payload[0] = -1;
			if( tg_pulse > (systemConfig()->cx_payload_B / 1000.0f) ){
				tg_pulse = 0;
				tg_state = 2;
			}
		}
		if( tg_state == 2){
			tg_pulse += dt;
			payload[0] = 1;
			if( tg_pulse > (systemConfig()->cx_payload_B / 1000.0) * 2 ){
				tg_pulse = 0;
				tg_state = 0;
				//payload[0] = 0;
			}
		}
	}
#endif

	cur_pldist = XM_GetCurrent().pdist;

	//is_shot_cond = ( cur_pldist > 0) && (X_Polygon_state == POLYGON_ONLINE) && (navstate == NAVSTATE_ROUTE);
	is_shot_cond = ( cur_pldist > 0) && (get_X_Polygon_state() == POLYGON_ONLINE) && (ics_get_state() == NAVSTATE_ROUTE);
	// Беляев. проверить, что я не сломал срабатывание фотика.

	if( is_shot_cond || (is_need_extra_shot == 1) || (is_need_extra_tg == 1)){

    	//tg update
		if( tg_state == 0){
			if( (x_dist_m(last_tg_pos_la, last_tg_pos_lo) > XM_GetCurrent().pdist)  || (is_need_extra_tg == 1) ){
				last_tg_pos_la = cur_pos.la;
				last_tg_pos_lo = cur_pos.lo;
				is_need_extra_tg = 0;

	//			if( payload[0] > 0){
	//				payload[0] = -1;
	//			} else {
	//				payload[0] = 1;
	//			}
				tg_pulse = 0;
				tg_state = 1;
			}
		}



    	switch( payload_state ){
		//Initial
    		case -1:
				IOWrite(pl_shot_pin, 1);
				time_impulse_relax += dt;
				if( time_impulse_relax > 0.5f ){
					payload_state = 0;
				}
			break;
    	//ready

    		case 0:
				IOWrite(pl_shot_pin, 1);
				if( x_dist_m(last_shot_pos_la, last_shot_pos_lo) > XM_GetCurrent().pdist ){
					is_need_shot = 1;
				}
				if( (is_need_shot == 1) || (is_need_extra_shot == 1)){
					payload_state = 1;
					time_impulse = 0;
					is_need_shot = 0;
					is_need_extra_shot = 0;
					is_need_extra_tg = 0;
					xlogger_set_payload_shot_time();	// Belyaev - this add shot time into log
				}
			break;
		//shot
			case 1:
				IOWrite(pl_shot_pin, 0);	// Belyaev - this makes photo shot
				time_impulse += dt;

				if( time_impulse > 0.15f ){
					payload_state = 2;
					time_impulse_relax = 0;
				}
			break;
		//relax
			case 2:
				IOWrite(pl_shot_pin, 1);
				time_impulse_relax += dt;
				if( time_impulse_relax > 0.5f ){
					payload_state = 0;
				}
			break;
			default:
				payload_state = -1;
		}
    }
}


int payload_init(){
#if defined(STM32F7) ||  defined(STM32H7)
#warning "NOT READY TO PAYLOAD FOR F7 MK"
	pl_exti_pin = IOGetByTag(IO_TAG(PAYLOAD_EXTI));
	pl_shot_pin = IOGetByTag(IO_TAG(PAYLOAD_SHOT));

    IOInit(pl_exti_pin, OWNER_XNAV, RESOURCE_EXTI, 0);
    IOConfigGPIO(pl_exti_pin, IOCFG_IN_FLOATING);

    IOInit(pl_shot_pin, OWNER_XNAV, RESOURCE_OUTPUT, 0);
    //IOConfigGPIO(pl_shot_pin, IOCFG_OUT_OD_UP);
    IOConfigGPIO(pl_exti_pin, IOCFG_IN_FLOATING);
    IOWrite(pl_shot_pin, 1);
    //IOWrite(pl_shot_pin, 0);

    EXTIHandlerInit(&pl_exti, payloadExtiHandler);
    EXTIConfig(pl_exti_pin, &pl_exti, NVIC_PRIO_GYRO_INT_EXTI, IO_CONFIG(GPIO_MODE_INPUT,0,GPIO_NOPULL));
    EXTIEnable(pl_exti_pin, true);
    return 1;



//    IOInit(gyro->busDev->irqPin, OWNER_MPU, RESOURCE_EXTI, 0);
//
//    EXTIHandlerInit(&gyro->exti, gyroIntExtiHandler);
//    EXTIConfig(gyro->busDev->irqPin, &gyro->exti, NVIC_PRIO_GYRO_INT_EXTI, IO_CONFIG(GPIO_MODE_INPUT,0,GPIO_NOPULL));   // TODO - maybe pullup / pulldown ?

#else
	pl_exti_pin = IOGetByTag(IO_TAG(PAYLOAD_EXTI));
	pl_shot_pin = IOGetByTag(IO_TAG(PAYLOAD_SHOT));

    IOInit(pl_exti_pin, OWNER_XNAV, RESOURCE_EXTI, 0);
    IOConfigGPIO(pl_exti_pin, IOCFG_IN_FLOATING);

    IOInit(pl_shot_pin, OWNER_XNAV, RESOURCE_OUTPUT, 0);
    IOConfigGPIO(pl_shot_pin, IOCFG_OUT_OD_UP);
    IOWrite(pl_shot_pin, 1);
    //IOWrite(pl_shot_pin, 0);

    EXTIHandlerInit(&pl_exti, payloadExtiHandler);
    EXTIConfig(pl_exti_pin, &pl_exti, NVIC_PRIO_GYRO_INT_EXTI, EXTI_Trigger_Rising);
    EXTIEnable(pl_exti_pin, true);
    return 1;
#endif
}

int ics_init(){
	int result = 0;
	payload_is_on = 0;

	payload_init();

	//LOAD RAM
	ram_my_type = systemConfig()->cx_plane_or_copter;
	if( (ram_my_type < 1) && (ram_my_type > 2) ){
		ram_my_type = 1;
		result |= 1;
	}

	if( systemConfig()->as_for_zero_dive < 25 ){
		systemConfigMutable()->as_for_zero_dive = 25;
		result |= 1;
	}

	ram_alt_reach_m = systemConfig()->cx_alt_reach_m;
	if( ram_alt_reach_m == 0 ) ram_alt_reach_m = 3;

	ram_altmc_pid_p = systemConfig()->cx_altmc_pid_p;
	ram_altmc_pid_i = systemConfig()->cx_altmc_pid_i;
	ram_altmc_pid_d = systemConfig()->cx_altmc_pid_d;

	ram_reduce_mc_in_plane = systemConfig()->cx_reduce_mc_in_plane;
	ram_allow_plane_in_manual = systemConfig()->cx_allow_plane_in_manual;
	ram_plane_switch_speed = systemConfig()->cx_plane_switch_speed;

	ram_linefollow_p= systemConfig()->cx_linefollow_p;
	ram_linefollow_i = systemConfig()->cx_linefollow_i;
	ram_linefollow_d = systemConfig()->cx_linefollow_d;

	ram_nav_pid_p = systemConfig()->cx_nav_pid_p;
	ram_nav_pid_i = systemConfig()->cx_nav_pid_i;
	ram_nav_pid_d = systemConfig()->cx_nav_pid_d;

	ram_small_angle_limit = systemConfig()->cx_small_angle_limit;

	ram_max_telemetry_lag_for_rth_s = systemConfig()->cx_max_telemetry_lag_for_rth_s;

	ram_rth_safe_h = systemConfig()->cx_rth_safe_h;
	if( systemConfig()->cx_rth_safe_h < systemConfig()->cx_moca_m ){
		ram_rth_safe_h = 120;
		result |= 4;
	}

	if( systemConfig()->cx_telemetry_lost_rth_or_land < 0 ){
		systemConfigMutable()->cx_telemetry_lost_rth_or_land = 1;
	}
	if( systemConfig()->cx_telemetry_lost_rth_or_land > 2 ){
		systemConfigMutable()->cx_telemetry_lost_rth_or_land = 1;
	}


	ram_circle_radius = systemConfig()->cx_circle_radius;

	if( ram_plane_switch_speed < 300 ){
		ram_plane_switch_speed = 1000;
		result |= 4;
	}

	if( systemConfig()->cx_check_motor_pwm < 900 ){
		systemConfigMutable()->cx_check_motor_pwm = 1030;
	}

	if( (ram_small_angle_limit < 100) || (ram_small_angle_limit > 1800) ){
		ram_small_angle_limit = 300;
	}

	//check VBat full voltage
	if( (systemConfig()->cx_full_voltage < 85) || ( systemConfig()->cx_full_voltage > 420 ) ){
		systemConfigMutable()->cx_full_voltage = 210; //5s default
		result |= 1;
	}
	//check ae_infl
	if( systemConfig()->cx_ae_infl > 100 ){
		systemConfigMutable()->cx_ae_infl = 10;
		result |= 2;
	}
	if( systemConfig()->cx_max_route_throttle > 2200 ){
		systemConfigMutable()->cx_max_route_throttle = 2200;
	}
	if( systemConfig()->cx_max_route_throttle < 1700 ){
		systemConfigMutable()->cx_max_route_throttle = 1800;
	}

	if( systemConfig()->cx_desc_pid_p < 0.01 ){
		systemConfigMutable()->cx_desc_pid_p = 2;
		systemConfigMutable()->cx_desc_pid_i = 0;
		systemConfigMutable()->cx_desc_pid_d = 0;
	}

	if( (systemConfig()->cx_telemetry_period_ms < 100) || (systemConfig()->cx_telemetry_period_ms > 5000) ){
		systemConfigMutable()->cx_telemetry_period_ms = 200;
	}

	if( (systemConfig()->cx_gps_lost_test_mode < 0 ) || (systemConfig()->cx_gps_lost_test_mode > 1) ){
		systemConfigMutable()->cx_gps_lost_test_mode = 0;
	}


	ram_use_only_gps_speed = systemConfig()->cx_use_only_gps_speed;
	if( ram_use_only_gps_speed < 0) ram_use_only_gps_speed = 0;
	if( ram_use_only_gps_speed > 1) ram_use_only_gps_speed = 0;

	systemConfigMutable()->cx_version = MY_SUBVERSION;



	cur_pos.airspd = 0;
	cur_pos.home_h = systemConfig()->cx_launch_height;

	cur_pos.inav_x_pos = 0;
	cur_pos.inav_y_pos = 0;
	XM_setHome();

	ics_set_state(NAVSTATE_OFF);

	if( systemConfig()->cx_autonomous == 1 ){
		ics_set_state(NAVSTATE_AUTONOMOUS_PREPARING);
	}

	return result;

}

uint8_t is_inited = 0;
TicsOrigin poshold_origin;



void ics_nav_controller(timeUs_t currentTimeUs){
    float dt = (currentTimeUs-prevTimeUs) * 0.000001f;
    if( dt > 0.1 ) dt = 0.1;
    prevTimeUs = currentTimeUs;
    ics_currentTimeUs = currentTimeUs;
    

    if( cur_alt_change > 0.5 ){
    	cur_alt_change -= dt*10;
    }
    if( cur_alt_change < -0.5 ){
    	cur_alt_change += dt*10;
    }

//    if( is_inited == 0 ){
//    	is_inited = 1;
//    	ics_init();
//    }

    payloadCheckShot(dt);

    cur_pos.cruise_spd = get_cruise_as();
    cur_pos.used_spd = get_cur_as();

    
    if( IS_RC_MODE_ACTIVE(BOXMANUAL)) {
    	ics_set_warning(WNG_MANUAL_ON, 1);

    	if (cur_pos.manual_modes == 0) {
    		clear_all_filters();
    	}



    	disable_gps_az = 0;
     	//cur_pos.manual_modes = 1;

    		// set it only we're not in maunal mode
    	ics_set_manual_modes(1);

    	if( IS_RC_MODE_ACTIVE(BOXNAVWP) ){
    		ics_set_manual_modes(8);
    		}
    	else if( IS_RC_MODE_ACTIVE(BOXNAVPOSHOLD) ){
    		if( ics_get_state() != NAVSTATE_POSHOLD){
    			ics_set_state(NAVSTATE_POSHOLD);
    			ics_set_manual_modes(4);
    			clear_all_filters();	// clear our filters
    			pidResetErrorAccumulatorAxis(FD_YAW);	// reset YAW error accumulator
    			poshold_origin.la = cur_pos.la;
    			poshold_origin.lo = cur_pos.lo;
    			poshold_origin.h = cur_pos.h;
    			poshold_origin.heading = cur_pos.mag_az;
    			}

    		}
    	else if( IS_RC_MODE_ACTIVE(BOXNAVALTHOLD) ){
    		//cur_pos.manual_modes |= 2;
    		if (ics_get_state() != NAVSTATE_ALTHOLD) {
    			ics_set_state(NAVSTATE_ALTHOLD);
    			clear_all_filters();
        		poshold_origin.h = cur_pos.h;

        		// set initial state for the althold tgt
    			}

    		//return;
    		}
    	else {
    		// if no modes enabled - reset navstate
    		// this allows to set new poshold/althold points
    		if (ics_get_state() != NAVSTATE_OFF) {
    		    ics_set_state(NAVSTATE_OFF);
    		    clear_all_filters();
    			}
    		return;
    		}
    }
    else {
	    ics_set_manual_modes(0);
	    ics_set_warning(WNG_MANUAL_ON, 0);
	    if (ics_get_state() == NAVSTATE_ALTHOLD) {
	    	ics_set_state(NAVSTATE_OFF);
	    	}
    	}

    //Check all systems and do something
    ics_do_monitoring(dt);

    cur_pos.cruise_spd = get_cruise_as();
    cur_pos.used_spd = get_cur_as();

    if( 	(ics_get_state() == NAVSTATE_OFF)||
    		(ics_get_state() == NAVSTATE_LAND)||
			(ics_get_state() == NAVSTATE_CAREFULL_LAND)
    	){
        servos_block = 1;
    } else {
        servos_block = 0;
    }

    //check if landing pause > max:
	if( landing_pause_time > systemConfig()->cx_max_landing_pause ){
		ics_continue_from_pause();
	}

    // TODO: write there getters everywhere
    if( navstate == NAVSTATE_OFF ) ics_idle(dt);
    if( navstate == NAVSTATE_LANDED ) ics_landed(dt);
	if( navstate == NAVSTATE_PRELAUNCH ) ics_prelaunch(dt);
	if( navstate == NAVSTATE_LAUNCH ) ics_launch(dt);
	if( navstate == NAVSTATE_ROUTE ) ics_route(dt);
	if( navstate == NAVSTATE_GOTO ) ics_route(dt);
	if( navstate == NAVSTATE_LAND ) ics_land(dt, 0);
	if( navstate == NAVSTATE_CAREFULL_LAND ) ics_care_land(dt);
	if( navstate == NAVSTATE_LAND_WO_POSHOLD ) ics_land(dt, 1);
	if( navstate == NAVSTATE_RTH ) ics_circle(dt);
	if( navstate == NAVSTATE_TELEMETRY_RTH ) ics_circle(dt);
	if( navstate == NAVSTATE_RTH_LAND ) ics_route(dt);
	if( navstate == NAVSTATE_WAIT_CIRCLE ) ics_circle(dt);
	if( navstate == NAVSTATE_STABILIZE ) ics_stab();
	if( navstate == NAVSTATE_POSHOLD ) ics_apply_manual_poshold_control( dt);	// manual poshold handler
	if( navstate == NAVSTATE_ALTHOLD ) ics_apply_manual_althold_control( dt);	// manual althold handler
	if( navstate == NAVSTATE_DESCEND ) ics_descend(dt);
	if( navstate == NAVSTATE_PRELAND_1 ) ics_preland(dt, 1);
	if( navstate == NAVSTATE_PRELAND_2 ) ics_preland(dt, 2);
	if( navstate == NAVSTATE_PRELAND_3 ) ics_preland(dt, 3);
	if( navstate == NAVSTATE_GPSLOST ) ics_circle(dt);
	if( navstate == NAVSTATE_SET_ALT ) ics_set_alt(dt);

	if( navstate == NAVSTATE_PRELINE ) {
		ics_preline(dt);
	} else {
		if( navstate != NAVSTATE_WAIT_CIRCLE ){
			x_preline_off();
		}
	}

	if( navstate == NAVSTATE_CHECK_SERVO ) ics_check_servos(dt);
	if( navstate == NAVSTATE_AUTONOMOUS_PREPARING ) ics_autonomous_prepaire(dt);



	debug[0] = rangefinder.rawAltitude;
	debug[1] = rangefinder.rawAltitude;
}

uint8_t ics_do_monitoring( double dt ){
    //Check GPS
	if( cur_pos.gps_ok == 0 ){
		ics_start_gpslost();
		ics_set_warning(WNG_GPS_LOST, 1);
	}


	//Check volates
	ics_update_batt_monitor();
	return 1;
}





uint8_t voltage_error_flag = 0;
#define VERR_LAUNCH (1)
#define VERR_LOW (2)
#define VERR_EMPTY (4)

void ics_update_batt_monitor(void){
	//This function updates NAV controller behaviour according to the battery state

	//voltage trigger in LAUNCH mode - override
	if( navstate == NAVSTATE_LAUNCH ){
		if( (systemConfig()->cx_batt_launch_min_value > 1.0f) &&
			((voltage_error_flag & VERR_LAUNCH) == 0) &&
			(get_voltage_launch_time() > systemConfig()->cx_batt_filter_time)
			){
		    ics_start_land(3);
		    voltage_error_flag |= VERR_LAUNCH;
		    ics_set_warning(WNG_LOWBATT_LAUNCH, 1);
		    return;

		}
	}

	// Warning action -> home and land
	// Capacity trigger ONLY (see battery.c)
	if( ((voltage_error_flag & VERR_LOW) == 0) &&
		(getBatteryState() == BATTERY_WARNING) ){

		if(
			(navstate != NAVSTATE_PRELAND_1)&&
			(navstate != NAVSTATE_PRELAND_2)&&
			(navstate != NAVSTATE_PRELAND_3)&&
			(navstate != NAVSTATE_LAND)&&
			(navstate != NAVSTATE_LAND_WO_POSHOLD)&&
			(navstate != NAVSTATE_CAREFULL_LAND)&&
			(navstate != NAVSTATE_LANDED)&&
			(navstate != NAVSTATE_OFF)&&
			(navstate != NAVSTATE_CHECK_SERVO)
			){
				ics_goto_land();
				ics_set_warning(WNG_LOWBATT_ROUTE, 1);
				voltage_error_flag |= VERR_LOW;
				return;
		}

	}

	// Critical action -> land now
	// Capacity trigger or voltage trigger (see battery.c)
	if(
		((voltage_error_flag & VERR_EMPTY) == 0)  &&
		(getBatteryState() == BATTERY_CRITICAL)
		){

		if(
			(navstate != NAVSTATE_PRELAND_1)&&
			(navstate != NAVSTATE_PRELAND_2)&&
			(navstate != NAVSTATE_PRELAND_3)&&
			(navstate != NAVSTATE_LAND)&&
			(navstate != NAVSTATE_LAND_WO_POSHOLD)&&
			(navstate != NAVSTATE_CAREFULL_LAND)&&
			(navstate != NAVSTATE_LANDED)&&
			(navstate != NAVSTATE_OFF)&&
			(navstate != NAVSTATE_CHECK_SERVO)
			){
				ics_start_land(3);
				ics_set_warning(WNG_LOWBATT_CRITICAL, 1);
				voltage_error_flag |= VERR_EMPTY + VERR_LOW;
				return;
		}
	}



}


double prelaunch_time = 0;
double launch_time = 0;

void ics_idle( double dt ){
	//unused(dt);
	//rcCommand[ROLL] = 100;
	//rcCommand[PITCH] = 100;

	disable_gps_az = 0;
	prelaunch_time = 0;
	cx_beeper_on = 0;
	//mwDisarm(DISARM_SWITCH);
	if(systemConfig()->cx_land_detector_sens > 0.5f){
		acc_raw_abs = cx_acc[X]*cx_acc[X] + cx_acc[Y]*cx_acc[Y] + cx_acc[X]*cx_acc[X];

		if( acc_raw_abs > systemConfig()->cx_land_detector_sens ){
			beeperConfirmationBeeps(1);
		}
	}

}

double auto_time = 0;
double auto_block_contlol_time = 1;
uint8_t auto_state = 0;
uint8_t auto_system_ready = 0;

void ics_autonomous_prepaire( double dt ){
	//unused(dt);
	disable_gps_az = 0;
	prelaunch_time = 0;

	if( auto_system_ready == 0 ){
		if (STATE(GPS_FIX) && gpsSol.numSat >= 5) {
			beeperConfirmationBeeps(3);
			auto_system_ready = 1;
		}
		return;
	}

	if( auto_state == 1 ){
		cx_beeper_on = 1;
		auto_time += dt;
		if( auto_time > 10.0 ){
			if (STATE(GPS_FIX) && gpsSol.numSat >= 5){
				int32_t la = cur_pos.la;
				int32_t lo = cur_pos.lo;

				XM_AddReceived(0, 4, la, lo, 0, XMISSION_KIND_POINT, 0, 0, 0, 0  );
				XM_AddReceived(1, 4, la, lo, systemConfig()->cx_autonomous_height, XMISSION_KIND_CIRCLE_INDEF, 0, 0, 0, 0  );
				XM_AddReceived(2, 4, la, lo, systemConfig()->cx_autonomous_prelanding_height, XMISSION_KIND_POINT, 0, 0, 0, 0  );
				XM_AddReceived(3, 4, la, lo, 0, XMISSION_KIND_LAND, 0, 0, 0, 0  );

				ics_set_state(NAVSTATE_PRELAUNCH);
			} else {
				auto_state = 0;
			}
		}
	} else {
		cx_beeper_on = 0;
		auto_time = 0;
	}

	if( auto_block_contlol_time > 0 ){
		auto_block_contlol_time -= dt;
	} else {
		//mwDisarm(DISARM_SWITCH);
		if(systemConfig()->cx_land_detector_sens > 0.5f){
			acc_raw_abs = cx_acc[X]*cx_acc[X] + cx_acc[Y]*cx_acc[Y] + cx_acc[X]*cx_acc[X];

			if( acc_raw_abs > systemConfig()->cx_land_detector_sens ){
				beeperConfirmationBeeps(1);
				auto_state = !auto_state;
				auto_block_contlol_time = 0.5;
			}
		}
	}

}

// home point set infusion
void XM_setHome(void) {
	if ((XM_Count == 0)) {
		// we set new home point only if there is no mission (or if only 1 point available, so it's home point)
		XM_AddReceived(0, 1, cur_pos.la, cur_pos.lo, cur_pos.h, XMISSION_KIND_POINT, 0, 0, 0, 0  );
		//XM_Count = 1;		// if we write only home point, set count to 1. Will be replaced if we load mission

	}
	XM_reset_local_systems();	// forcely reset local systems to use INAV pos estimator
}



void CP_INAV_TO_LOCAL(void) {
	cur_pos.local_x_pos = cur_pos.inav_x_pos + XM_GetHome().inavX;
	cur_pos.local_y_pos = cur_pos.inav_y_pos + XM_GetHome().inavY;
//#if defined (USE_TELEMETRY_XLOGGER)
//	xlogger_send_debug(cur_pos.local_x_pos, 0);
//	xlogger_send_debug(cur_pos.local_y_pos, 1);
//	xlogger_send_debug(cur_pos.inav_x_pos, 2);
//	xlogger_send_debug(cur_pos.inav_y_pos, 3);
//#endif
}


void ics_landed( double dt ){
	//unused(dt);
	disable_gps_az = 0;
	prelaunch_time = 0;
	cx_beeper_on = 1;
	//mwDisarm(DISARM_SWITCH);
}

int launch_dest_alt = 1500;
int dest_mid_alt = 500;

uint8_t is_yaw_ok = 0;
double yaw_test_time = 0;

stdev_t yaw_filter;

uint8_t check_yaw( double dt ){
	if( systemConfig()->cx_yaw_prelaunch_test_time > 0.05){
		if( !gyroIsCalibrationComplete() ){
			return 0;
		}
		float stddev = devStandardDeviation(&yaw_filter);
		float stmean = devMean(&yaw_filter);

		xlogger_send_debug(stddev, 0);
		xlogger_send_debug(stmean, 1);

		if( ( ((float)yaw_test_time) > systemConfig()->cx_yaw_prelaunch_test_time ) ){

			// does this check pass????
			// it calculates the deviation, but not the mean value
			if( ( (ABS(stddev) + ABS(stmean)) > systemConfig()->cx_yaw_prelaunch_test_value) ){
				gyroSetCalibrationCycles(CALIBRATING_GYRO_CYCLES);
			} else {
				return 1;
			}
			devClear(&yaw_filter);
			yaw_test_time = 0;
		}
		devPush(&yaw_filter, gyro.gyroADCf[YAW]);
		yaw_test_time += dt;
	} else {
		return 1;
	}
	return 0;
}

void ics_prelaunch( double dt ){
	disable_gps_az = 0;

	if(is_yaw_ok == 1){
		cx_check_engines = 1;

		prelaunch_time += dt;
		if( prelaunch_time > 3 ){
			cx_check_engines = 0;
			mwArm();
			set_XM_Current(0);

	//		int first_point_alt = XM_Get(1).h - XM_GetHome().h;
	//		if( first_point_alt < launch_dest_alt ){
	//		  launch_dest_alt = first_point_alt;
	//		  if( first_point_alt < 1500 ) launch_dest_alt = 1500;
	//		}
			if (ARMING_FLAG(ARMED)) {
				launch_dest_alt = XM_Get(1).h - XM_GetHome().h;
				if( launch_dest_alt < systemConfig()->cx_launch_height ){
					launch_dest_alt = systemConfig()->cx_launch_height;
				}

				dest_mid_alt = systemConfig()->cx_launch_height_mid;

				if( dest_mid_alt > (launch_dest_alt-500) ){
					dest_mid_alt = launch_dest_alt / 2;
					if( dest_mid_alt < 1000 ) dest_mid_alt = 1000;
				}

				poshold_origin.h = launch_dest_alt;
				poshold_origin.la = cur_pos.la;
				poshold_origin.lo = cur_pos.lo;
				poshold_origin.heading = cur_pos.mag_az;

				//during preparing to launch, we write flags state to xlogger and reset launch state
				xlogger_send_flags();
				xlogger_launch_reset ();

				launch_time = 0;
				cur_pos.launch_check = 0;
				gyro_lag_flag = 0;
				ics_set_state(NAVSTATE_LAUNCH);
			}

		}
	} else {
		is_yaw_ok = check_yaw(dt);
	}
	cx_beeper_on = 2;
    rcCommand[THROTTLE] = constrain(0 , motorConfig()->minthrottle, motorConfig()->maxthrottle);
}

uint8_t cx_launch_stage = 0;
double cx_launch_accel_time = 0;

uint16_t cur_launch_speed = 0;

void ics_launch( double dt ){
    int dest_angle_dd = systemConfig()->cx_launch_destangle_dd; // cm  
    xlogger_launch(1);// отправляем запись об успешном запуске
	disable_gps_az = 1;

	cx_beeper_on = 0;

	rcCommand[YAW] = 0;

	if( gyro_lag_flag != 0 ){
    	mwDisarm(DISARM_SWITCH);
    	ics_set_state(NAVSTATE_LANDED);
    	ics_set_warning(WNG_GYRO_LAG, 1);
	}

    int current_alt = cur_pos.h;//baroGetLatestAltitude();
    launch_time += dt;
    if( systemConfig()->cx_launch_check_time > 0 ){
    	if(cur_pos.launch_check == 0 ){
    		if( launch_time > systemConfig()->cx_launch_check_time ){
				if( current_alt >= systemConfig()->cx_launch_check_height ){
					cur_pos.launch_check = 1;
				} else {
					cur_pos.launch_check = -1;
					ics_set_warning( WNG_LAUNCH_BROKE, 0 );
					ics_start_land(3);
				}
    		}
    	}
    }

    if( systemConfig()->cx_launch_acceleration_gain > 0.5 ){
    	cur_launch_speed = (launch_time * launch_time) * systemConfig()->cx_launch_speed / systemConfig()->cx_launch_acceleration_gain;
    	if( cur_launch_speed > systemConfig()->cx_launch_speed ){
    		cur_launch_speed = systemConfig()->cx_launch_speed;
    	}
    } else {
    	cur_launch_speed = systemConfig()->cx_launch_speed;
    }


    if( systemConfig()->cx_launch_kind == 0){
		if(launch_dest_alt <= current_alt){
			XM_Reached();
			//ics_start_preline();
			return;
		}

		if( (dest_mid_alt > 0) && (dest_mid_alt < launch_dest_alt) && (current_alt > dest_mid_alt) ){
			int angle_dd = 1.0 * (current_alt-dest_mid_alt) / (launch_dest_alt-dest_mid_alt) * dest_angle_dd;
			rcCommand[PITCH] = pidAngleToRcCommand( angle_dd, pidProfile()->max_angle_inclination[FD_PITCH]);
			rcCommand[ROLL] = 0;
		} else {
				rcCommand[PITCH] = 0;
				rcCommand[ROLL] = 0;
		}
		rcCommand[THROTTLE] = constrain((int16_t)(systemConfig()->cx_launch_start_throttle*x_get_vbat_compansation_factor()) , motorConfig()->minthrottle, motorConfig()->maxthrottle);
    } else if( systemConfig()->cx_launch_kind == 1){
		if(launch_dest_alt <= current_alt){
			XM_Reached();
			//ics_start_preline();
			return;
		}

		if( (dest_mid_alt > 0) && (dest_mid_alt < launch_dest_alt) && (current_alt > dest_mid_alt) ){
			int angle_dd = 1.0 * (current_alt-dest_mid_alt) / (launch_dest_alt-dest_mid_alt) * dest_angle_dd;
			rcCommand[PITCH] = pidAngleToRcCommand( angle_dd, pidProfile()->max_angle_inclination[FD_PITCH]);
			rcCommand[ROLL] = 0;
		} else {
			rcCommand[PITCH] = 0;
			rcCommand[ROLL] = 0;
		}
		ics_alt_speed_controler( dt, cur_launch_speed );
    } else if( systemConfig()->cx_launch_kind == 2){
		if(launch_dest_alt <= current_alt){
			XM_Reached();
			//ics_start_preline();
			return;
		}

		if( (dest_mid_alt > 0) && (dest_mid_alt < launch_dest_alt) && (current_alt > dest_mid_alt) ){
			if( cx_trust_gps_azumuth ){
				XM_Reached();
				return;
			}

			int angle_dd = 1.0 * (current_alt-dest_mid_alt) / (launch_dest_alt-dest_mid_alt) * dest_angle_dd;
			rcCommand[PITCH] = pidAngleToRcCommand( angle_dd, pidProfile()->max_angle_inclination[FD_PITCH]);
			rcCommand[ROLL] = 0;
		} else {
			rcCommand[PITCH] = 0;
			rcCommand[ROLL] = 0;
		}
		ics_alt_speed_controler( dt, cur_launch_speed );
    } else if( systemConfig()->cx_launch_kind == 3){
    	if(cx_launch_stage == 0){
			rcCommand[PITCH] = 0;
			rcCommand[ROLL] = 0;
			ics_new_althold_controller( dt, launch_dest_alt, systemConfig()->cx_altspeed_gain, cur_launch_speed);
			if(launch_dest_alt <= current_alt){
				cx_launch_stage = 1;
			}
    	} else {
    		rcCommand[PITCH] = pidAngleToRcCommand( dest_angle_dd, pidProfile()->max_angle_inclination[FD_PITCH]);
			rcCommand[ROLL] = 0;

			ics_new_althold_controller( dt, launch_dest_alt, systemConfig()->cx_launch_speed, systemConfig()->cx_launch_speed);

			if( cx_trust_gps_azumuth ){
				XM_Reached();
				return;
			}
    	}
	} else if( systemConfig()->cx_launch_kind == 4){
//		if(abs(launch_dest_alt - current_alt) < systemConfig()->cx_launch_type4_max_alt_err ){		// paraams
//			XM_Reached();
//			//ics_start_preline();
//			return;
//		}

		if( (abs(launch_dest_alt - current_alt) < systemConfig()->cx_launch_type_four_max_alt_err) ||  (launch_dest_alt <= current_alt)){
			if( cx_trust_gps_azumuth ){
				XM_Reached();
				return;
			}
			disable_gps_az = 0;
			int angle_dd = 1.0 * (current_alt-dest_mid_alt) / (launch_dest_alt-dest_mid_alt) * dest_angle_dd;
			rcCommand[PITCH] = pidAngleToRcCommand( angle_dd, pidProfile()->max_angle_inclination[FD_PITCH]);
			rcCommand[ROLL] = 0;
		} else {
			disable_gps_az = 1;
			rcCommand[PITCH] = 0;
			rcCommand[ROLL] = 0;
		}
		//ics_alt_speed_controler( dt, cur_launch_speed );
		ics_new_althold_controller( dt, launch_dest_alt, systemConfig()->cx_altspeed_gain, cur_launch_speed);
	}  else if( systemConfig()->cx_launch_kind == 5){
		if(cx_launch_stage == 0){
			disable_gps_az = 1;
			rcCommand[PITCH] = 0;
			if( current_alt > launch_dest_alt - systemConfig()->cx_launch_type_four_max_alt_err ){
				cx_launch_accel_time = 0;
				cx_launch_stage = 1;
			}
		} else if(cx_launch_stage == 1){

			int angle_dd = dest_angle_dd;
			cx_launch_accel_time += dt;
			if( systemConfig()->cx_launch_route_acceleration_gain_s > 0 ){
				float acc_gain = 1.0 * (cx_launch_accel_time) / (systemConfig()->cx_launch_route_acceleration_gain_s);
				acc_gain = constrainf(acc_gain, 0, 1);
				angle_dd = acc_gain * dest_angle_dd;
			}
			// When mode changed to COPTER switch on azimuth corrector
			if( cx_launch_accel_time > systemConfig()->cx_launch_route_acceleration_gain_s ){
				disable_gps_az = 0;
			} else {
				disable_gps_az = 1;
			}

			rcCommand[PITCH] = pidAngleToRcCommand( angle_dd, pidProfile()->max_angle_inclination[FD_PITCH]);
			if( cur_pos.airspd > (systemConfig()->cx_launch_speed_border_ms * 100) ){
				cx_launch_stage = 2;
			}
		} else {
			disable_gps_az = 0;
			rcCommand[PITCH] = pidAngleToRcCommand( dest_angle_dd, pidProfile()->max_angle_inclination[FD_PITCH]);
			if( cx_trust_gps_azumuth ){
				XM_Reached();
				return;
			}
		}
		//ics_alt_speed_controler( dt, cur_launch_speed );
		rcCommand[ROLL] = 0;
		ics_new_althold_controller( dt, launch_dest_alt, systemConfig()->cx_altspeed_gain, cur_launch_speed);
		//Flight in
		if( systemConfig()->cx_launch_heading_hold_enable > 0 ){
			ics_new_poshold_heading_hold_controller(dt);
		}
	}
}

void ics_land( double dt, uint8_t disable_poshold ){
	disable_gps_az = 1;
	check_drift();
    if( systemConfig()->cx_land_type >= 1){
    	disable_mag = 1;
	}

	cx_beeper_on = 1;



    if( disable_poshold == 1 ){
      rcCommand[ROLL] = 0;
      rcCommand[PITCH] = 0;
      rcCommand[YAW] = 0;
      if( is_pause_landing == 1 ){
    	  ics_alt_speed_controler( dt, 0 );
    	  landing_pause_time += dt;
      } else {
    	  ics_alt_speed_controler( dt, -systemConfig()->cx_land_speed );
      }
    } else {
    	poshold_origin.h -= systemConfig()->cx_land_speed * dt;

        //check if it is time to go careful landing
        if( systemConfig()->cx_land_careful_height > 0 ){
            int32_t prelanding_h = XM_GetPrelanding().h;
            int32_t home_h = XM_GetHome().h;
            int32_t caref_h = systemConfig()->cx_land_careful_height * 100;
            //if( cur_pos.h < (  prelanding_h - home_h - caref_h )){

            if( cur_pos.h < caref_h ){
                ics_set_state(NAVSTATE_CAREFULL_LAND);
            }
        }
        
        float d2p_poshold;
        if (systemConfig()->cx_use_new_poshold > 0) {
        	d2p_poshold = ics_new_poshold_pos_controller(dt, 0);
        }
        else {
        	ics_poshold_2d(dt, 0, 0);
        	d2p_poshold = X_Dist2_m() ;
        }
         
        if( systemConfig()->cx_land_by_speed == 1 ){
            if( is_pause_landing == 1  ){
          	  ics_alt_speed_controler( dt, 0 );
          	  landing_pause_time += dt;
              //ics_new_althold_controller( dt, launch_dest_alt, systemConfig()->cx_altspeed_gain, cur_launch_speed);
            } else {
            	if( d2p_poshold < systemConfig()->cx_landing_radius){
    				// inside cx_landing_radius radius -- normal speed
    				ics_alt_speed_controler( dt, -systemConfig()->cx_land_speed );
            	} else {
    				// ouside cx_landing_radius radius -- normal speed
    				ics_alt_speed_controler( dt, -systemConfig()->cx_land_speed_outside_radius );
            	}
            }


        	//ics_alt_speed_controler( dt, -systemConfig()->cx_land_speed );
        }else {
            ics_althold_1d(dt, 0);
        }

        check_rangefinder( dt );
    }
}

uint8_t ics_set_alt_stage = 0;
double drop_pause_time = 0;

void ics_set_alt( double dt ){
	disable_gps_az = 1;
	cx_beeper_on = 0;
	uint32_t drop_h = XM_GetCurrent().pdist * 100 - XM_GetHome().h;

	poshold_origin.la = XM_GetCurrent().la;
	poshold_origin.lo = XM_GetCurrent().lo;

	if( ics_set_alt_stage == 0 ){
		// Descend to pdist
		//we in COPTER mode. see PL_DIST -- alt in h for alt hold
		poshold_origin.h = drop_h;



		if( cur_pos.h < poshold_origin.h + ram_alt_reach_m ) {
			//Wait command
			ics_set_alt_stage = 1;
			is_pause_landing = 1;
			drop_pause_time = 0;
		}
	} else if( ics_set_alt_stage == 1 ){
		// Waiting command
		cx_beeper_on = 1;
		poshold_origin.h = drop_h;

		drop_pause_time += dt;

		if( drop_pause_time > systemConfig()->cx_max_landing_pause ){
		  ics_set_alt_stage = 2;
		}

		if( is_pause_landing == 0 ){
		  ics_set_alt_stage = 2;
		}

	} else if( ics_set_alt_stage == 2 ){
		// Descend to pdist
		//we in COPTER mode. see PL_DIST -- alt in h for alt hold
		poshold_origin.h = XM_GetCurrent().h - XM_GetHome().h;

		if( cur_pos.h > poshold_origin.h - ram_alt_reach_m ) {
			ics_set_alt_stage = 3;
		}
	}else if( ics_set_alt_stage == 3 ){
		// Need to AS
		poshold_origin.h = XM_GetCurrent().h - XM_GetHome().h;

		//hold this H
//		double alts_gain = constrainf( (poshold_origin.h - cur_pos.h * 1.0) / 10.0, -1.0, 1.0);
//		int16_t cur_alt_speed_target = alts_gain * systemConfig()->cx_altspeed_drop;
//		ics_alt_speed_controler( dt, cur_alt_speed_target );

		ics_new_althold_controller( dt, poshold_origin.h, systemConfig()->cx_altspeed_drop, systemConfig()->cx_altspeed_drop);



		//Increase AirSpeed
		rcCommand[PITCH] = pidAngleToRcCommand( systemConfig()->cx_launch_destangle_dd, pidProfile()->max_angle_inclination[FD_PITCH]);
		rcCommand[ROLL] = 0;

		//check if AirSpeed is OK
		if( cx_trust_gps_azumuth ){
			XM_Reached();
		}
		return;
	}


//	double alts_gain = constrainf( (poshold_origin.h - cur_pos.h * 1.0) / 10.0, -1.0, 1.0);
//	int16_t cur_alt_speed_target = alts_gain * systemConfig()->cx_altspeed_drop;
//	ics_alt_speed_controler( dt, cur_alt_speed_target );
	ics_new_althold_controller( dt, poshold_origin.h, systemConfig()->cx_altspeed_drop, systemConfig()->cx_altspeed_drop);

	if (systemConfig()->cx_use_new_poshold > 0) {
		float d2p_poshold  = ics_new_poshold_pos_controller(dt, 0);
		UNUSED(d2p_poshold);
	}
	else {
		ics_poshold_2d(dt, 0, 0);
	}
}

double rangefinder_noise_time = 0;
double rangefinder_noise_frame_time = 0;

void check_rangefinder_in_flight( double dt ){
	//Check noise time in 60 sec
	if( systemConfig()->cx_rangefinder_check_in_flight){
		rangefinder_noise_frame_time += dt;
		if( rangefinder.rawAltitude > 0 ){
			rangefinder_noise_time += dt;
		}
		if( rangefinder_noise_frame_time > 60 ){
			rangefinder.cur_noise = rangefinder_noise_time * 100.0 / rangefinder_noise_frame_time;
			if( rangefinder_noise_time > systemConfig()->cx_rangefinder_noise_max_time ){
				rangefinder_mode = 0;
				ics_set_warning(WNG_RANGEFINDER_BROKE, 1);
			}
			rangefinder_noise_frame_time = 0;
			rangefinder_noise_time = 0;
		}
	}
}

void check_rangefinder( double dt ){
	if( rangefinder_mode == 1 ){
		if( systemConfig()->cx_rangefinder_height > 0 ){
			if( rangefinder.rawAltitude > 0 ){
				rangefinder_mode = 2;
				rangefinder.x_land_detector_time = 0;
			}
		}
	} else if( rangefinder_mode == 2 ){
		if( rangefinder.rawAltitude < 0 ){
			if( (systemConfig()->cx_rangefinder_noise_filter == 1) && (rangefinder.x_land_detector_time > dt) ){
				rangefinder.x_land_detector_time -= dt;
			} else {
				rangefinder_mode = 1;
				rangefinder.x_land_detector_time = 0;
			}
		} else {
			if( rangefinder.rawAltitude <= systemConfig()->cx_rangefinder_height ){
				rangefinder.x_land_detector_time += dt;

				if( rangefinder.x_land_detector_time > (systemConfig()->cx_rangefinder_time) ){
                    ics_set_state(NAVSTATE_LANDED);
					mwDisarm(DISARM_SWITCH);
					xlogger_landed();
					set_XM_Current(0);
				}
			} else {
				rangefinder.x_land_detector_time = 0;
			}
		}
	} else {
		rangefinder.x_land_detector_time = 0;
	}
}

void ics_care_land( double dt ){
	disable_gps_az = 1;
	cx_beeper_on = 1;
	check_drift();

    if( ics_is_landed(dt)){
    	ics_set_state(NAVSTATE_LANDED);
    	mwDisarm(DISARM_SWITCH);
    	xlogger_landed();
    }

    float d2p_poshold;
    if (systemConfig()->cx_use_new_poshold > 0) {
        d2p_poshold = ics_new_poshold_pos_controller(dt, 0);
    }
    else {
    	ics_poshold_2d(dt, 0, 1);
    	d2p_poshold = X_Dist2_m();
    }

    if( is_pause_landing == 1  ){
    	ics_alt_speed_controler( dt, 0 );
    	landing_pause_time += dt;
    } else {
		if( d2p_poshold < systemConfig()->cx_landing_radius ){
			// inside cx_landing_radius radius -- normal speed
			ics_alt_speed_controler( dt, -systemConfig()->cx_land_careful_speed );
		} else {
			// ouside cx_landing_radius radius -- normal speed
			ics_alt_speed_controler( dt, -systemConfig()->cx_land_careful_speed_outside_radius );
		}
    }

    check_rangefinder(dt);
}

void ics_preland( double dt, uint8_t stage ){
	/* run pre-calculations */
	cx_beeper_on = 1;
	disable_gps_az = 1;
	check_drift();
    if( systemConfig()->cx_landing_poshold_enable < 0.5 ){
    	ics_set_state(NAVSTATE_LAND_WO_POSHOLD);
    	return;
    }
    if( systemConfig()->cx_land_type >= 2){
		if( (navstate == NAVSTATE_PRELAND_2) || (navstate == NAVSTATE_PRELAND_3) ){
			disable_mag = 1;
		} else {
			disable_mag = 0;
		}
    }


	/* run controllers */
    // poshold
    float d2p_poshold = 0;
    if (systemConfig()->cx_use_new_poshold > 0) {
    	d2p_poshold = ics_new_poshold_pos_controller(dt, 0);
    }
    else {
    	ics_poshold_2d(dt, 0, 0);
    	d2p_poshold = X_Dist2_m();
    }
    // althold
	if( systemConfig()->cx_prelanding_altspeed_enable ){
		ics_new_althold_controller( dt, poshold_origin.h, systemConfig()->cx_altspeed_gain, systemConfig()->cx_altspeed_gain);
	} else {
    	ics_althold_1d(dt, 0);
	}

	/* check if we close to the point */
	preland_time += dt;
    float st = systemConfig()->cx_landing_stage_time / 1000.0;
    //Wait until reached
    if( stage == 1 ){
    	if( d2p_poshold < (systemConfig()->cx_landing_radius) ){
    		ics_set_state(NAVSTATE_PRELAND_2);
    		poshold_origin.h += systemConfig()->cx_landing_stage_alt;
    		preland_time = 0;
    		cur_pos.land_stage = 0;
    		//stages is OFF
    		if( systemConfig()->cx_landing_stage_count == 0 ){
    			ics_set_state( NAVSTATE_LAND );
    		}
    	}
    } else if( preland_time > st ){
		poshold_origin.h += systemConfig()->cx_landing_stage_alt;
		preland_time = 0;
		cur_pos.land_stage += 1;
		if( cur_pos.land_stage >= systemConfig()->cx_landing_stage_count )		{
    		ics_set_state(NAVSTATE_LAND);
    	}
    }

}

int l_min_h = -0xFFF;
int l_max_h = 0xFFF;

double ics_is_landed( double dt ){
	if( cur_pos.h > l_max_h ) l_max_h = cur_pos.h;
	if( cur_pos.h < l_min_h ) l_min_h = cur_pos.h;

	stay_time += dt;

	if( systemConfig()->cx_land_detector_max_gps_speed > 0 ){
		if( cur_pos.gndspd > systemConfig()->cx_land_detector_max_gps_speed){
			stay_time = 0;
			l_max_h = cur_pos.h;
			l_min_h = cur_pos.h;
		}
	}


	if( (l_max_h - l_min_h) > systemConfig()->cx_land_detector_filter_h ){
		//clear timing
		stay_time = 0;
		l_max_h = cur_pos.h;
		l_min_h = cur_pos.h;
	}


	if( (cur_pos.hspd < -systemConfig()->cx_land_detector_filter_als) || (cur_pos.hspd > systemConfig()->cx_land_detector_filter_als) ){
		stay_time = 0;
		l_max_h = cur_pos.h;
		l_min_h = cur_pos.h;
	}

    int32_t home_h = XM_GetHome().h;
    int32_t ld_h = systemConfig()->cx_land_detector_enable_h * 100;
    int32_t tmp_need_t = (systemConfig()->cx_land_detector_filter_t / 1000.0);
    if( cur_pos.h < (home_h + ld_h)){
    	tmp_need_t *= 5;
    }

	if( systemConfig()->cx_land_detector_enable_h > 0 ){
		//if( stay_time > (systemConfig()->cx_land_detector_filter_t / 1000.0 * get_landing_timing_scale())){
		if( stay_time > (systemConfig()->cx_land_detector_filter_t / 1000.0)){
					//clear timing
			stay_time = 0;
			l_max_h = cur_pos.h;
			l_min_h = cur_pos.h;
			xlogger_landed();
			return 1;
		}
	}



//	if( (systemConfig()->cx_land_detector_sens > 0.5f) && (cur_pos.h < 1000) ){
//		acc_raw_abs = cx_acc[X]*cx_acc[X] + cx_acc[Y]*cx_acc[Y] + cx_acc[X]*cx_acc[X];
//
//		if( acc_raw_abs > systemConfig()->cx_land_detector_sens ){
//		    beeperConfirmationBeeps(1);
//			return 1;
//		}
//	}
	return 0;
}

uint32_t dist_to_origin = 0;

int16_t poshold_direction = 0;

float poshold_althold_tail_factor = 0;
float res_rads;
float vx_angle;
float vy_angle;







//#define MAX_HORS_I_PART (350)	//Maximum integral part in PWM units



void ics_new_poshold_hor_spd_controller(double dt, float speed_X_tgt, float speed_Y_tgt) {
	// this is speed controller function for new poshold subproject
	// speed_X_tgt is target speed in cm/s. X is connected with the PITCH axis. 'PITCH +' = 'X +'
	// speed_Y_tgt is target speed in cm/s. Y is connected with the ROLL axis.  'ROLL +'  = 'Y +'

	// calculate speeds in the projection in the axes of the connected coordinates system

	disable_gps_az = 1;
	float speed_ABS = (float) cur_pos.gndspd;
	float speed_AZ_rad = DECIDEGREES_TO_RADIANS(cur_pos.az - cur_pos.mag_az);
	float speed_X = speed_ABS * cos_approx(speed_AZ_rad);
	float speed_Y = speed_ABS * sin_approx(speed_AZ_rad);

	// calculate speed errors in connected coordinates system
	speed_X_err_curr = speed_X_tgt - speed_X;
	speed_Y_err_curr = speed_Y_tgt - speed_Y;

    // X axis control = PITCH
	speed_X_res = ics_pid(
        dt,
		speed_X_err_curr, &speed_X_err_prev, &speed_X_ierr,
        systemConfig()->cx_new_poshold_speed_x_pid_p,
        systemConfig()->cx_new_poshold_speed_x_pid_i,
        systemConfig()->cx_new_poshold_speed_x_pid_d,
		systemConfig()->cx_new_poshold_speed_hor_pid_maxI
    );
#if defined(USE_TELEMETRY_XLOGGER)
	cur_PID.new_poshold.speed_x_tgt = speed_X_tgt;
	cur_PID.new_poshold.speed_x_val = speed_X;
	cur_PID.new_poshold.speed_x_p_cont = cur_PID.buff_p_cont;
	cur_PID.new_poshold.speed_x_i_cont = cur_PID.buff_i_cont;
	cur_PID.new_poshold.speed_x_d_cont = cur_PID.buff_d_cont;
#endif

    // Y axis control = ROLL
	speed_Y_res = ics_pid(
        dt,
		speed_Y_err_curr, &speed_Y_err_prev, &speed_Y_ierr,
        systemConfig()->cx_new_poshold_speed_y_pid_p,
        systemConfig()->cx_new_poshold_speed_y_pid_i,
        systemConfig()->cx_new_poshold_speed_y_pid_d,
		systemConfig()->cx_new_poshold_speed_hor_pid_maxI
    );
#if defined(USE_TELEMETRY_XLOGGER)
	cur_PID.new_poshold.speed_y_tgt = speed_Y_tgt;
	cur_PID.new_poshold.speed_y_val = speed_Y;
	cur_PID.new_poshold.speed_y_p_cont = cur_PID.buff_p_cont;
	cur_PID.new_poshold.speed_y_i_cont = cur_PID.buff_i_cont;
	cur_PID.new_poshold.speed_y_d_cont = cur_PID.buff_d_cont;
#endif



	// angle tgt limits
	speed_X_res = constrainf( speed_X_res, -systemConfig()->cx_landing_max_climb, systemConfig()->cx_landing_max_dive );
	speed_Y_res = constrainf( speed_Y_res, -systemConfig()->cx_landing_max_roll, systemConfig()->cx_landing_max_roll );

	// apply controls
	rcCommand[PITCH] = pidAngleToRcCommand( speed_X_res, pidProfile()->max_angle_inclination[FD_PITCH]);
	rcCommand[ROLL] = pidAngleToRcCommand( speed_Y_res, pidProfile()->max_angle_inclination[FD_ROLL]);

#if defined(USE_TELEMETRY_XLOGGER)
	cur_PID.new_poshold.curTime = millis();
	cur_PID.new_poshold.new = 1;
	cur_PID.new_poshold.speed_x_pid_out = speed_X_res;
	cur_PID.new_poshold.speed_y_pid_out = speed_Y_res;
#endif

}


float calcHorizonRateMagnitude_from20(float tgt_in, float val_in)
{
		// input
		float l, r, res;
		float tgt = tgt_in;
		float val = val_in;

		//wrap if not done
		tgt = wrap_3600(tgt);
		val = wrap_3600(val);

		// calculate left and right values

		if (tgt >= val) {
		    l = (tgt - val);
		    r = -fmod(3600-l, 3600);
		}
		else {
		    r = -(val - tgt);
		    l = fmod(3600+r,  3600);
		}
		if (fabs(l) < fabs(r)) {
		    res = l;
		}
		else {
		    res = r;
		}
		return res;

}

uint8_t yaw_Iterm_reset = 0;		// set to 1 to disable yaw Iterm

uint8_t ics_navstate_use_new_poshold(void) {
	uint8_t cx_navstate_now = ics_get_state();
	return 	(					(systemConfig()->cx_use_new_poshold) &&
								(
								(cx_navstate_now == NAVSTATE_POSHOLD) ||
								(cx_navstate_now == NAVSTATE_ALTHOLD) ||
								(cx_navstate_now == NAVSTATE_PRELAND_1) ||
								(cx_navstate_now == NAVSTATE_PRELAND_2) ||
								(cx_navstate_now == NAVSTATE_PRELAND_3) ||
								(cx_navstate_now == NAVSTATE_LAND) ||
								(cx_navstate_now == NAVSTATE_CAREFULL_LAND) ||
								(cx_navstate_now == NAVSTATE_SET_ALT)
								)
			);
}

uint8_t ics_get_yaw_Iterm_reset(void) {
	return yaw_Iterm_reset;
}

void ics_new_poshold_heading_hold_controller(double dt) {

	// dt is seconds?!
    uint8_t use_heading_hold = 	(
    							(systemConfig()->cx_new_poshold_use_heading_hold)&&
								(cur_pos.airspd < systemConfig()->cx_new_poshold_heading_hold_max_as_sm)
								);


    float rateIErr = pidGetErrorIntegral(FD_YAW);		// collect yaw rate integral - see if error is too high

    if (ram_my_type != COPTER) {
    // rate I compensation
    	// idea: slowly correct heading hold target using rate I error so it shuold stand against the wind
    	// do it only for FIXAR mode!
    	poshold_origin.heading -=  (rateIErr * systemConfig()->cx_new_poshold_heading_hold_rate_I_corr* ((float)dt));
    }

    // circle the circle
    poshold_origin.heading = wrap_3600(poshold_origin.heading);
    float heading_err = calcHorizonRateMagnitude_from20(poshold_origin.heading, cur_pos.mag_az);		// correct rotation target (check!)




	if (use_heading_hold) {
		// heading p controller
		rcCommand[YAW] = constrainf(( -heading_err * systemConfig()->cx_py_pid_p ) / 4, -500, 500);
		yaw_Iterm_reset = 0;	// don't reset yaw Iterm cause we need it here
	}
	else {
		//if we don't use heading hold - hold zero rate and remember this heading for future
		poshold_origin.heading = cur_pos.mag_az;
		rcCommand[YAW] = 0;
		yaw_Iterm_reset = 1;  // reset yaw Iterm here - either this leads to altitude grow in the wind
	}

#ifdef USE_TELEMETRY_XLOGGER
	cur_pos.need_az = (int16_t) poshold_origin.heading;
#endif

}

float ics_new_poshold_pos_controller(double dt, uint8_t allow_sticks) {
	// position hold function
	// inner contour - horizontal velocity
	// outer contour - horizontal position
	// return distance to point in cm

	// needed coords stay in poshold_origin
    if( allow_sticks == 0 ){
	  rcCommand[YAW] = 0;
    }


	XFloatPoint ph_tgt_pos = XM_ToLocal(poshold_origin.la, poshold_origin.lo);	// tgt position into local coords
	XFloatPoint ph_cur_pos = {cur_pos.local_x_pos,cur_pos.local_y_pos};
	float coord_d2orig = x_dist_2point_cm(ph_tgt_pos.x, ph_tgt_pos.y, ph_cur_pos.x, ph_cur_pos.y);	// calc dist to origin in cm
	//int32_t coord_az2pt = x_az2p(poshold_origin.la, poshold_origin.lo) / 10.0;	// calc azimuth to point in decidegrees
	int32_t coord_az2pt = x_az2p_cm(ph_tgt_pos.x, ph_tgt_pos.y);	// calc azimuth to point in decidegrees
	int16_t coord_azErr = (int16_t)(coord_az2pt - cur_pos.mag_az);	// compare az2p with current azimuth. Not error, but projection to connected system
	if( coord_azErr > 1800 )  coord_azErr -= 3600;					// correction
	if( coord_azErr < -1800 )  coord_azErr += 3600;

	float coord_azErrRad = DECIDEGREES_TO_RADIANS(coord_azErr);		// basically, azimuth to point in connected coordinates system

	// errors
	float coord_X_err = coord_d2orig * cos_approx(coord_azErrRad);
	float coord_Y_err = coord_d2orig * sin_approx(coord_azErrRad);

	// apply P gain
	float speed_X_tgt = coord_X_err*systemConfig()->cx_new_poshold_pos_x_pid_p;
	float speed_Y_tgt = coord_Y_err*systemConfig()->cx_new_poshold_pos_y_pid_p;

	// apply limits
	speed_X_tgt = constrainf(speed_X_tgt, -systemConfig()->cx_new_poshold_speed_x_lim_minus,	systemConfig()->cx_new_poshold_speed_x_lim_plus);
	speed_Y_tgt = constrainf(speed_Y_tgt, -systemConfig()->cx_new_poshold_speed_y_lim,			systemConfig()->cx_new_poshold_speed_y_lim);

	// call horizontal speed controller
	ics_new_poshold_hor_spd_controller(dt, speed_X_tgt, speed_Y_tgt);

	// apply yaw rotation
	ics_new_poshold_heading_hold_controller(dt);

    if( ram_my_type != COPTER ){
//    	rcCommand[YAW] = ( -coord_azErr * systemConfig()->cx_py_pid_p ) / 4;
//    	//rcCommand[ROLL] = 0;
//    } else {		// dist_to_origin - cm!

    	// this case calls only if point behind fixar and outside radius -> wind in the back
    	  if( (systemConfig()->cx_yaw_poshold > 0) && (coord_d2orig > systemConfig()->cx_new_poshold_yaw_radius) && ((coord_azErr > 900)||(coord_azErr < -900) ) ){
    		  // disable heading hold, override yaw command
    		  	yaw_Iterm_reset = 1;		// because this was the conditions of this push before
    	    	int16_t yaw_res = pidAngleToRcCommand( -coord_azErr, currentControlRateProfile->stabilized.rates[FD_YAW] );
    	    	poshold_origin.heading = cur_pos.mag_az;

    	    	if( (systemConfig()->cx_yaw_poshold_roll_compensation > 0) ){
    	    		rcCommand[YAW] = yaw_res * cos_approx(DECIDEGREES_TO_RADIANS(vy_angle)) * cos_approx(DECIDEGREES_TO_RADIANS(vx_angle));

    	    	} else {
    	    		rcCommand[YAW] = yaw_res;
    	    	}
    	    }
    }

#if defined(USE_TELEMETRY_XLOGGER)
	cur_PID.new_poshold.curTime = millis();
	cur_PID.new_poshold.new = 1;
	cur_PID.new_poshold.coord_az2pt = coord_az2pt;
	cur_PID.new_poshold.coord_d2orig = coord_d2orig;
	cur_PID.new_poshold.coord_azErr = coord_azErr;
	cur_PID.new_poshold.coord_x_tgt = ph_tgt_pos.x;
	cur_PID.new_poshold.coord_y_tgt = ph_tgt_pos.y;
	cur_PID.new_poshold.coord_x_now = cur_pos.local_x_pos;
	cur_PID.new_poshold.coord_y_now = cur_pos.local_y_pos;
#endif

	return coord_d2orig;
}

uint8_t new_pos, new_heading;

uint8_t ics_apply_manual_poshold_control( double dt) {

#define ICS_EARTH_RADIUS (6378137)

		// apply tgt setting from rc:
		float rll_val = (float)rcData[ROLL] - 1500.0;  // 0 is center// hyi tam! 1500 is center!
		float pit_val = (float)rcData[PITCH]- 1500.0;  // 0 is center// hyi tam! 1500 is center!
		float yaw_val = (float)rcData[YAW]- 1500.0;

		float cur_X_speed_target, cur_Y_speed_target;
		// NAVSTATE_POSHOLD is ONLY MANUAL MODE!!!

		// these variables made to move poshold tgt
		float ddist_m = ((float)systemConfig()->cx_new_poshold_manual_dpos_cm)/100;
		float dn, de, dLat, dLon;




		// apply pitch control === x axis
		if ((sq(rll_val) + sq(pit_val)) > sq(systemConfig()->cx_new_poshold_pitchroll_deadband)){
			// pitch channel not in center
			new_pos = 1;
			// calc new speed tgts
			cur_X_speed_target = scaleRangef_zero(pit_val, -500, 0, +500, -systemConfig()->cx_new_poshold_speed_x_lim_minus,	0,	systemConfig()->cx_new_poshold_speed_x_lim_plus );
			cur_Y_speed_target = scaleRangef_zero(rll_val, -500, 0, +500, -systemConfig()->cx_new_poshold_speed_y_lim,			0,	systemConfig()->cx_new_poshold_speed_y_lim);
			// run speed control
			ics_new_poshold_hor_spd_controller(dt, cur_X_speed_target, cur_Y_speed_target);
		}
		else {
			if (new_pos) {
	 			// if position changed
				dn = ddist_m*cos_approx(DECIDEGREES_TO_RADIANS(cur_pos.mag_az));
				de = ddist_m*sin_approx(DECIDEGREES_TO_RADIANS(cur_pos.mag_az));

				dLat = (dn / ICS_EARTH_RADIUS)*1e7;
				dLon = (de / (ICS_EARTH_RADIUS * cos_approx(DEGREES_TO_RADIANS(cur_pos.la/1e7))))*1e7;

				poshold_origin.la = cur_pos.la + (int)RADIANS_TO_DEGREES(dLat);
				poshold_origin.lo = cur_pos.lo + (int)RADIANS_TO_DEGREES(dLon);

				new_pos = 0;
			}
			// run position control
			float d2p_poshold = ics_new_poshold_pos_controller(dt, 1);
			UNUSED(d2p_poshold);
		}


		if (abs(yaw_val) > systemConfig()->cx_new_poshold_pitchroll_deadband) {
			new_heading = 1;
			rcCommand[YAW] = -getAxisRcCommand(rcData[YAW], FLIGHT_MODE(MANUAL_MODE) ? currentControlRateProfile->manual.rcYawExpo8 : currentControlRateProfile->stabilized.rcYawExpo8, rcControlsConfig()->yaw_deadband);
		}
		else {
			if(new_heading) {
				poshold_origin.heading = cur_pos.mag_az;
				new_heading = 0;
    			//pidResetErrorAccumulatorAxis(FD_YAW);	// reset YAW error accumulator
			}
			// run heading control
			ics_new_poshold_heading_hold_controller(dt);

		}

		ics_apply_manual_althold_control(dt);
		return 1;

}

void ics_poshold_2d(  double dt, uint8_t allow_sticks, uint8_t is_careful){
    if( allow_sticks == 0 ){
	  rcCommand[YAW] = 0;
    }
    if( ram_my_type == PLANE ){
    	disable_gps_az = 1;
    } else{
    	disable_gps_az = 0;
    }

    dist_to_origin = x_dist_m(poshold_origin.la, poshold_origin.lo);

    int32_t az_to_point = x_az2p(poshold_origin.la, poshold_origin.lo) / 10.0;	// таргет направления
	int16_t res = (int16_t)(az_to_point - cur_pos.mag_az);	// ошибка направления
	if( res > 1800 )  res -= 3600;
	if( res < -1800 )  res += 3600;

	poshold_direction = res;

	poshold_althold_tail_factor = fabs(poshold_direction*1.0) / 1800.0;

	//res shows direction to point in plane coordinate system
	//well, we can calculate velocity vextor:
	res_rads = DECIDEGREES_TO_RADIANS(res);
	int16_t vx = dist_to_origin * sin_approx(res_rads);
	int16_t vy = dist_to_origin * cos_approx(res_rads);


					// этот кусок кода корректно считает скорости в связанной СК

					//D term
					int16_t gps_spd = gpsSol.groundSpeed;
					float gps_az_rad = DECIDEGREES_TO_RADIANS(cur_pos.az - cur_pos.mag_az);		// направление вектора скорости в связанной СК

					int16_t gpsspd_x = gps_spd * sin_approx(gps_az_rad);						// проекции вектора скорости на оси связанной СК
					int16_t gpsspd_y = gps_spd * cos_approx(gps_az_rad);						// проекции вектора скорости на оси связанной СК

					// Вот до сюда
					// x и y перепутаны!
	vx_angle = (vx * 1.0) * systemConfig()->cx_pr_pid_p - 1.0 * gpsspd_x * systemConfig()->cx_pr_pid_d;
	vy_angle = (vy * 1.0) * systemConfig()->cx_ph_pid_p - 1.0 * gpsspd_y * systemConfig()->cx_ph_pid_d;

#if defined(USE_TELEMETRY_XLOGGER)
	cur_PID.poshold.curTime = millis();
	cur_PID.poshold.az_to_point = az_to_point;
	cur_PID.poshold.poshold_direction = res;
	cur_PID.poshold.dist_to_origin = dist_to_origin;
	cur_PID.poshold.cx_pr_p_cont = (vx * 1.0) * systemConfig()->cx_pr_pid_p;
	cur_PID.poshold.cx_pr_i_cont = 0;
	cur_PID.poshold.cx_pr_d_cont = 1.0 * gpsspd_x * systemConfig()->cx_pr_pid_d;
	cur_PID.poshold.cx_ph_p_cont = (vy * 1.0) * systemConfig()->cx_ph_pid_p;
	cur_PID.poshold.cx_ph_i_cont = 0;
	cur_PID.poshold.cx_ph_d_cont = 1.0 * gpsspd_y * systemConfig()->cx_ph_pid_d;
	cur_PID.poshold.vx_angle = vx_angle;
	cur_PID.poshold.vy_angle = vy_angle;
	cur_PID.poshold.new = 1;
#endif

	vy_angle = constrainf( vy_angle, -systemConfig()->cx_landing_max_climb, systemConfig()->cx_landing_max_dive );
	if( is_careful == 1 ){
        vx_angle = constrainf( vx_angle, -systemConfig()->cx_land_careful_max_roll, systemConfig()->cx_land_careful_max_roll );
    } else {
        vx_angle = constrainf( vx_angle, -systemConfig()->cx_landing_max_roll, systemConfig()->cx_landing_max_roll );
    }

	//No think, just apply this
    rcCommand[PITCH] = pidAngleToRcCommand( vy_angle, pidProfile()->max_angle_inclination[FD_PITCH]);
    rcCommand[ROLL] = pidAngleToRcCommand( vx_angle, pidProfile()->max_angle_inclination[FD_ROLL]);



    if( ram_my_type == COPTER ){
    	rcCommand[YAW] = ( -res * systemConfig()->cx_py_pid_p ) / 4;
    	//rcCommand[ROLL] = 0;
    } else {		// dist_to_origin в метрах!
    	  if( (systemConfig()->cx_yaw_poshold > 0) && (dist_to_origin > 10.0) && ((res > 900)||(res < -900) ) ){
    	    	int16_t yaw_res = pidAngleToRcCommand( -res, currentControlRateProfile->stabilized.rates[FD_YAW] );

    	    	if( (systemConfig()->cx_yaw_poshold_roll_compensation > 0) ){
    	    		rcCommand[YAW] = yaw_res * cos_approx(DECIDEGREES_TO_RADIANS(vy_angle)) * cos_approx(DECIDEGREES_TO_RADIANS(vx_angle));

    	    	} else {
    	    		rcCommand[YAW] = yaw_res;
    	    	}
    	    } else {
    	    	rcCommand[YAW] = 0;
    	    }
    }
    //ics_althold_1d(dt, allow_sticks);
}

void ics_althold_1d(  double dt, uint8_t allow_sticks){
    if( allow_sticks > 0 ){
    	poshold_origin.h += ((rcData[THROTTLE] - 1500) / 5.0 ) * dt ;
    }


//	if( systemConfig()->cx_prelanding_altspeed_enable ){
//		double alts_gain = constrainf( (poshold_origin.h -  cur_pos.hspd * 1.0) / 10.0, -0.5, 1.0);
//
//		int16_t cur_alt_speed_target = alts_gain * systemConfig()->cx_altspeed_gain;
//		ics_alt_speed_controler( dt, cur_alt_speed_target );
//	} else {
//		ics_alt_mc_controler( dt, (int)(poshold_origin.h) );
//	}
    ics_alt_mc_controler( dt, (int)(poshold_origin.h) );

}

void ics_preline( double dt ){
    rcCommand[ROLL] = 0;
    rcCommand[YAW] = 0;
	disable_gps_az = 0;

	if( systemConfig()->cx_circle_radius < 10 ){
		ics_set_state(NAVSTATE_ROUTE);
		return;
	}

	//First line is home--first point.
	if( XM_Current <= 1 ){
		set_XM_Current(2);
	}


	XMissionPoint L = XM_GetLineStart();
	XMissionPoint E = XM_GetCurrent();

	//Small line
	if( x_dist_2point_m(E.x, E.y, L.x, L.y) < 1.0 ){
		ics_set_state(NAVSTATE_ROUTE);
		return;
	}

	x_check_preline();

	if( x_is_preline_reached(dt) ){
		ics_set_state(NAVSTATE_ROUTE);
		return;
	}

    uint16_t cas = get_cruise_as();

    ics_as_controler( dt, cas, xgeo_get_alt() );

    ics_circle_controller( dt );
    ics_alt_controler( dt, xgeo_get_alt());
}


void ics_route( double dt ){
    rcCommand[ROLL] = 0;
    rcCommand[YAW] = 0;
    
	disable_gps_az = 0;

	check_rangefinder_in_flight(dt);

    uint16_t cas = get_cruise_as();
//    if( systemConfig()->cx_land_brake_enable > 0 ){
//        if( XM_IsPenult() > 0 ){
//            if( X_Dist2_m() < systemConfig()->cx_land_brake_distance2_m ){
//                cas -= systemConfig()->cx_land_brake_throttle_cruise_reduction;
//            }
//        }
//    }

    if( ram_my_type == PLANE ){
		ics_as_controler( dt, cas, xgeo_get_alt() );
		ics_route_controller( dt );
		//calc after route: add ae_infl
		ics_alt_controler( dt, xgeo_get_alt() + cur_alt_change);
    } else {
    	if( X_Dist2_m() < (navConfig()->general.waypoint_radius) ){
    		XM_Reached();
    	}

    	poshold_origin.h = xgeo_get_alt() + cur_alt_change;
		poshold_origin.la = XM_GetCurrent().la;
		poshold_origin.lo = XM_GetCurrent().lo;

		ics_poshold_2d(dt, 0, 0);
		if (systemConfig()->cx_use_new_poshold > 0) {
			float d2p_poshold = ics_new_poshold_pos_controller(dt, 0);
			UNUSED(d2p_poshold);
		}
		else {
			ics_althold_1d(dt, 0);
		}
    }
}

uint8_t ics_start_rth(){
	cur_pos.override_target_h = 0;
	if(
    		(navstate != NAVSTATE_OFF) &&
			(navstate != NAVSTATE_LAUNCH) &&
			(navstate != NAVSTATE_LAND) &&
			(navstate != NAVSTATE_CAREFULL_LAND) &&
			(navstate != NAVSTATE_PRELAUNCH)&&
			(navstate != NAVSTATE_PRELAND_1)&&
			(navstate != NAVSTATE_PRELAND_2)&&
			(navstate != NAVSTATE_PRELAND_3)&&
			(navstate != NAVSTATE_LAND_WO_POSHOLD)&&
			(navstate != NAVSTATE_CHECK_SERVO)&&
			(navstate != NAVSTATE_LANDED)

    ){
//    	if( cur_pos.h < systemConfig()->cx_launch_height){
//			cur_pos.home_h = systemConfig()->cx_launch_height;
//		} else {
//			cur_pos.home_h = cur_pos.h ;
//		}

		//fix https://youtrack.fixar/issue/RND-12
		cur_pos.home_h = ram_rth_safe_h * 100;



    	XM_SetCurrent(0);
    	x_preline_off();
    	ics_set_state(NAVSTATE_RTH);
    	// Override AFTER change point
    	cur_pos.override_target_h = cur_pos.home_h + XM_GetHome().h;
		return 1;
    }
	if( (navstate == NAVSTATE_PRELAND_1 ) || (navstate == NAVSTATE_PRELAND_2 ) || (navstate == NAVSTATE_PRELAND_3 ) ){
		if( is_pause_landing == 1 ){
	    	if( cur_pos.h < systemConfig()->cx_launch_height){
				cur_pos.home_h = systemConfig()->cx_launch_height;
			} else {
				cur_pos.home_h = cur_pos.h ;
			}
	    	XM_SetCurrent(0);
	    	x_preline_off();
			ics_set_state( NAVSTATE_RTH );
			return 1;
		}
	}
    return 0;
}





XFloatPoint tmp_plane_pos;

extern XFloatPoint cur_circle_origin;
extern int8_t cur_circle_direction;

uint8_t ics_pause(){
	if( navstate == NAVSTATE_OFF ) return 0;
	if( navstate == NAVSTATE_LAUNCH ) return 0;
	if( navstate == NAVSTATE_LANDED ) return 0;
	if( navstate == NAVSTATE_GPSLOST ) return 0;
	if( navstate == NAVSTATE_CHECK_SERVO ) return 0;
	//anti-repeat pause
	if( navstate ==  NAVSTATE_WAIT_CIRCLE ) return 0;
	if(
    		(navstate == NAVSTATE_PRELAUNCH) ||
			(navstate == NAVSTATE_AUTONOMOUS_PREPARING)
	) {
		ics_set_state( NAVSTATE_OFF );
		prevstate = NAVSTATE_OFF;
    	return 1;
    }

	if(
				(navstate == NAVSTATE_LAND) ||
				(navstate == NAVSTATE_PRELAND_1) ||
				(navstate == NAVSTATE_PRELAND_2) ||
				(navstate == NAVSTATE_PRELAND_3) ||
				(navstate == NAVSTATE_LAND_WO_POSHOLD) ||
				(navstate == NAVSTATE_CAREFULL_LAND) ||
				(navstate == NAVSTATE_POSHOLD)
	) {
//do not change mode, just stop landing
		is_pause_landing = 1;
		landing_pause_time = 0;
		ics_set_warning(WNG_PAUSE_ON, 1);
    	return 1;
    }

//#define NAVSTATE_ROUTE (2)
//#define NAVSTATE_RTH (4)
//#define NAVSTATE_WAIT_CIRCLE (6)
//#define NAVSTATE_STABILIZE (5)
//#define NAVSTATE_DESCEND (10)
//#define NAVSTATE_RTH_LAND (11)
//#define NAVSTATE_PRELINE (16)
//#define NAVSTATE_GOTO (18)
//#define NAVSTATE_TELEMETRY_RTH (31)

	tmp_plane_pos = XM_ToLocal( cur_pos.la, cur_pos.lo );

	cur_circle_direction = -1;
	cur_circle_origin.x = tmp_plane_pos.x;
	cur_circle_origin.y = tmp_plane_pos.y;

	prevstate = navstate;
	ics_set_state( NAVSTATE_WAIT_CIRCLE );
    ics_set_warning(WNG_PAUSE_ON, 1);
    //fix pause alt
    int moca_m = systemConfig()->cx_moca_m;

	if( cur_pos.h < moca_m * 100){
		cur_pos.override_target_h = moca_m * 100  + XM_GetHome().h;
	} else {
		cur_pos.override_target_h = cur_pos.h  + XM_GetHome().h;
	}


	return 1;
}

uint8_t ics_goto_origin( int la, int lo, int h ){
	if( navstate == NAVSTATE_OFF ) return 0;
	if( navstate == NAVSTATE_LAUNCH ) return 0;
	if( navstate == NAVSTATE_LANDED ) return 0;
	if( navstate == NAVSTATE_GPSLOST ) return 0;
	if( navstate == NAVSTATE_CHECK_SERVO ) return 0;
	if(
    		(navstate == NAVSTATE_PRELAUNCH) ||
			(navstate == NAVSTATE_AUTONOMOUS_PREPARING)
	) return 0;

	if(
					(navstate == NAVSTATE_LAND) ||
					(navstate == NAVSTATE_LAND_WO_POSHOLD) ||
					(navstate == NAVSTATE_CAREFULL_LAND) ||
					(navstate == NAVSTATE_POSHOLD)
		) {

	    	return 0;
	}

	//#define NAVSTATE_ROUTE (2)
	//#define NAVSTATE_RTH (4)
	//#define NAVSTATE_WAIT_CIRCLE (6)
	//#define NAVSTATE_STABILIZE (5)
	//#define NAVSTATE_DESCEND (10)
	//#define NAVSTATE_RTH_LAND (11)
	//#define NAVSTATE_PRELINE (16)
	//#define NAVSTATE_GOTO (18)
	//#define NAVSTATE_TELEMETRY_RTH (31)
	//  (navstate == NAVSTATE_PRELAND_1) ||
	//					(navstate == NAVSTATE_PRELAND_2) ||
	//					(navstate == NAVSTATE_PRELAND_3) ||

	tmp_plane_pos = XM_ToLocal( la, lo );

	cur_circle_direction = -1;
	cur_circle_origin.x = tmp_plane_pos.x;
	cur_circle_origin.y = tmp_plane_pos.y;

	cur_pos.override_target_h = h;

	if( navstate != NAVSTATE_WAIT_CIRCLE){
		prevstate = navstate;
		ics_set_state( NAVSTATE_WAIT_CIRCLE );
	}

	return 1;
}


uint8_t ics_start_gpslost(){
	ics_set_warning( WNG_GPS_LOST, 1);
	if(
    		(navstate != NAVSTATE_OFF) &&
			(navstate != NAVSTATE_AUTONOMOUS_PREPARING) &&
			(navstate != NAVSTATE_LAUNCH) &&
			(navstate != NAVSTATE_LAND) &&
			(navstate != NAVSTATE_CAREFULL_LAND) &&
			(navstate != NAVSTATE_PRELAUNCH)&&
			(navstate != NAVSTATE_PRELAND_1)&&
			(navstate != NAVSTATE_PRELAND_2)&&
			(navstate != NAVSTATE_PRELAND_3)&&
			(navstate != NAVSTATE_LAND_WO_POSHOLD)&&
			(navstate != NAVSTATE_CHECK_SERVO)&&
			(navstate != NAVSTATE_LANDED)&&
			(navstate != NAVSTATE_ALTHOLD)

    ){
    	XM_SetCurrent(0);
    	ics_set_state(NAVSTATE_GPSLOST);
    	return 0;
    }

	if( navstate == NAVSTATE_PRELAUNCH )
	{
		ics_set_state( NAVSTATE_LANDED );
		return 0;
	}

	if( (navstate == NAVSTATE_LAND) || (navstate == NAVSTATE_CAREFULL_LAND)|| (navstate == NAVSTATE_LAUNCH) )
	{
		ics_set_state( NAVSTATE_LAND_WO_POSHOLD );
		return 0;
	}
	return 0;
}

uint8_t ics_start_preline(){
	x_preline_off();
	xgeo_pid_nu_ierr_reset();
	ics_set_state(NAVSTATE_PRELINE);
	return 1;
}


uint8_t ics_goto_land(){
    if(
    		(navstate != NAVSTATE_OFF) &&
			(navstate != NAVSTATE_LAUNCH) &&
			(navstate != NAVSTATE_LAND) &&
			(navstate != NAVSTATE_CAREFULL_LAND) &&
			(navstate != NAVSTATE_PRELAUNCH)&&
			(navstate != NAVSTATE_PRELAND_1)&&
			(navstate != NAVSTATE_PRELAND_2)&&
			(navstate != NAVSTATE_PRELAND_3)&&
			(navstate != NAVSTATE_LAND_WO_POSHOLD)&&
			(navstate != NAVSTATE_CHECK_SERVO)&&
			(navstate != NAVSTATE_LANDED)

    ){

//		if( cur_pos.h < ram_rth_safe_h * 100){
//			cur_pos.home_h = ram_rth_safe_h * 100;
//		} else {
//			cur_pos.home_h = cur_pos.h ;
//		}
    	cur_pos.home_h = ram_rth_safe_h * 100;

		if( (XM_Count - 3) > 1 ){
			XM_SetCurrent(XM_Count - 3);
		} else {
			XM_SetCurrent(XM_Count - 2);
		}
		ics_set_state(NAVSTATE_RTH_LAND);
		// Override AFTER change point
		cur_pos.override_target_h = cur_pos.home_h + XM_GetHome().h;
		return 1;
    }
    return 0;
}



uint8_t ics_start_descend(){
	ics_set_state(NAVSTATE_DESCEND);
    desc_ierr = 0;
    return 0;
}

uint8_t ics_start_route(){
	xgeo_pid_nu_ierr_reset();
	ics_set_state(NAVSTATE_ROUTE);
	return 1;
}

uint8_t ics_start_goto( uint16_t new_point_number ){
//goto do not works with prelanding and landing
	cur_pos.override_target_h = 0;

	//Clear warnings RND-30
 	ics_set_warning( WNG_TELEMETRY_LOST, 0);
//	ics_set_warning( WNG_GPS_LOST, 0);


	if( new_point_number+2 >= XM_Count){
    	return 0;
    }

	if( (navstate == NAVSTATE_RTH) || (navstate == NAVSTATE_ROUTE) || (navstate == NAVSTATE_GOTO) || (navstate == NAVSTATE_WAIT_CIRCLE) || (navstate == NAVSTATE_DESCEND)|| (navstate == NAVSTATE_GPSLOST)|| (navstate == NAVSTATE_PRELINE) || (navstate == NAVSTATE_TELEMETRY_RTH) ){
		ics_set_state(NAVSTATE_GOTO);
		set_XM_Current(new_point_number);
		return 1;
    }
	if( (navstate == NAVSTATE_PRELAND_1 ) || (navstate == NAVSTATE_PRELAND_2 ) || (navstate == NAVSTATE_PRELAND_3 ) ){
		if( is_pause_landing == 1 ){
			ics_set_state( NAVSTATE_GOTO );
			set_XM_Current(new_point_number);
			return 1;
		}
	}

    return 0;
}

uint8_t ics_continue_route(){
	cur_pos.override_target_h = 0;
    if( (navstate == NAVSTATE_RTH) || (navstate == NAVSTATE_ROUTE) || (navstate == NAVSTATE_GOTO) || (navstate == NAVSTATE_WAIT_CIRCLE) || (navstate == NAVSTATE_DESCEND)|| (navstate == NAVSTATE_GPSLOST)|| (navstate == NAVSTATE_PRELINE) ){
    	ics_set_state(NAVSTATE_ROUTE);
		return 1;
    }
    return 0;

}

uint8_t ics_continue_from_pause(){
    is_pause_landing = 0;
    cur_circle_direction = 0;
    ics_set_warning( WNG_PAUSE_ON, 0);
    cur_pos.override_target_h = 0;

    if( navstate != NAVSTATE_WAIT_CIRCLE) return 0;
	if( prevstate == NAVSTATE_OFF) return 0;

// This modes never set. Just check for future bugs
	if( prevstate == NAVSTATE_LAUNCH ) return 0;
	if( prevstate == NAVSTATE_LANDED ) return 0;
	if( prevstate == NAVSTATE_GPSLOST ) return 0;
	if( prevstate == NAVSTATE_CHECK_SERVO ) return 0;
	if( prevstate == NAVSTATE_PRELAUNCH ) return 0;
	if( prevstate == NAVSTATE_AUTONOMOUS_PREPARING ) return 0;

    if(
    				(navstate == NAVSTATE_LAND) ||
    				(navstate == NAVSTATE_PRELAND_1) ||
    				(navstate == NAVSTATE_PRELAND_2) ||
    				(navstate == NAVSTATE_PRELAND_3) ||
    				(navstate == NAVSTATE_LAND_WO_POSHOLD) ||
    				(navstate == NAVSTATE_CAREFULL_LAND) ||
    				(navstate == NAVSTATE_POSHOLD)
    	) {
		   //is_pause_landing = 0;
        	return 1;
        }

    //#define NAVSTATE_ROUTE (2)
    //#define NAVSTATE_RTH (4)
    //#define NAVSTATE_WAIT_CIRCLE (6)
    //#define NAVSTATE_STABILIZE (5)
    //#define NAVSTATE_DESCEND (10)
    //#define NAVSTATE_RTH_LAND (11)
    //#define NAVSTATE_PRELINE (16)
    //#define NAVSTATE_GOTO (18)
    //#define NAVSTATE_TELEMETRY_RTH (31)

	if( XM_IsLine() && (prevstate == NAVSTATE_ROUTE) ){
		//XM_Prev();
		ics_set_state( NAVSTATE_PRELINE );
	} else {
		ics_set_state( prevstate );
	}
	prevstate = NAVSTATE_OFF;

	return 1;

}

void ics_circle( double dt ){
    rcCommand[ROLL] = 0;
    rcCommand[YAW] = 0;
	disable_gps_az = 0;

    if( ram_my_type == PLANE ){
        ics_alt_controler( dt, xgeo_get_alt());
        ics_as_controler( dt, get_cruise_as(), xgeo_get_alt());
        ics_circle_controller( dt );
    } else {
    	if( X_Dist2_m() < (navConfig()->general.waypoint_radius) ){
    		XM_Reached();
    	}

    	poshold_origin.h = xgeo_get_alt() + cur_alt_change;
		poshold_origin.la = XM_GetCurrent().la;
		poshold_origin.lo = XM_GetCurrent().lo;

		if (systemConfig()->cx_use_new_poshold > 0) {
			float d2p_poshold = ics_new_poshold_pos_controller(dt, 0);
			UNUSED(d2p_poshold);
		}
		else {
			ics_poshold_2d(dt, 0, 0);
    	}
		ics_althold_1d(dt, 0);
    }

}

void ics_process_rth_land( double dt ){
    rcCommand[ROLL] = 0;
    rcCommand[YAW] = 0;

    ics_alt_controler( dt, xgeo_get_alt());
    ics_as_controler( dt, get_cruise_as(), xgeo_get_alt());
    ics_circle_controller( dt );
}

void ics_descend( double dt ){
    rcCommand[ROLL] = 0;
    rcCommand[YAW] = 0;
	disable_gps_az = 0;

    float desc_alt_need = XM_GetCurrent().h - XM_GetHome().h;

    ics_alt_controler( dt, desc_alt_need);
    ics_as_controler( dt, get_cruise_as(), desc_alt_need);
//    ics_descend_controler( dt, get_cruise_as() );
    ics_circle_controller( dt );

    if( cur_pos.h < ( desc_alt_need + 2000) ){
		//To avoid circular switching to descend
    	set_XM_Current(XM_Current+1);
		ics_continue_route();
	}
}

double stage_time = 0;

void ics_start_check_servos(){
	stage_time = 0;
	servos_check_pitch = 0;
	ics_set_state(NAVSTATE_CHECK_SERVO);
}

void ics_stop_check_servos(){
	stage_time = 0;
	servos_check_pitch = 0;
	ics_set_state(NAVSTATE_OFF);
}

void ics_check_servos( double dt ){
    rcCommand[ROLL] = 0;
    rcCommand[YAW] = 0;
    rcCommand[THROTTLE] = 0;


    if( stage_time > 1.0f){
    	servos_check_pitch += 1;
    	if( servos_check_pitch > 3){
    		servos_check_pitch = 0;
    		ics_stop();
    	}
    }
    stage_time += dt;
}

void ics_stab(void){
    //rcCommand[ROLL] = pidAngleToRcCommand(posControl.rcAdjustment[ROLL], pidProfile()->max_angle_inclination[FD_ROLL]);
    rcCommand[ROLL] = 0;
    //rcCommand[PITCH] = 0;
    rcCommand[PITCH] = pidAngleToRcCommand( -systemConfig()->cx_planemode_pitch_angle_decidegi, pidProfile()->max_angle_inclination[FD_PITCH]);
    rcCommand[YAW] = 0;
    rcCommand[THROTTLE] = constrain(0, motorConfig()->minthrottle, motorConfig()->maxthrottle);
}


uint8_t ics_start(){
	if( (navstate == NAVSTATE_OFF) && IS_RC_MODE_ACTIVE(BOXANGLE)){
    	if( is_mission_valid && is_home_valid){

            //XM_Current = 0;
        	set_XM_Current(0);
			//TurnOnEngines
			// cx_check_engines = 1;
    		//Turn on after GYRO check, in prelaunch
    		//Clear yaw test filter
    		is_yaw_ok = 0;
    		yaw_test_time = 0;
    		devClear(&yaw_filter);

			ics_set_warning( WNG_GPS_LOST, 0);
			cur_pos.override_target_h = 0;

			ics_set_state(NAVSTATE_PRELAUNCH);


			//init hold position
			fpVector3_t targetHoldPos;
			calculateInitialHoldPosition(&targetHoldPos);
			setDesiredPosition(&targetHoldPos, posControl.actualState.yaw, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_HEADING);

			//RANGEFINDER_LANDING DETECTOR Check
			rangefinder_mode = 0;
			if( rangefinder.rawAltitude == -1 ){
				rangefinder_mode = -1;
				ics_set_warning(WNG_RANGEFINDER_BROKE, 1);
			} else {
				rangefinder_mode = 1;
				ics_set_warning(WNG_RANGEFINDER_BROKE, 0);
			}
			return 1;
    	}
    }
    return 0;
}

uint8_t ics_stop(){
  ics_set_state(NAVSTATE_OFF);
  mwDisarm(DISARM_SWITCH);
  cx_check_engines = 0;
  return 1;
}

uint8_t ics_start_land(uint8_t is_emergency_landing){
	// If POSHOLD Landing
  if( is_emergency_landing <= 1 ){
	  if(     (navstate == NAVSTATE_PRELAND_1) ||
			  (navstate == NAVSTATE_PRELAND_2) ||
			  (navstate == NAVSTATE_PRELAND_3) ||
			  (navstate == NAVSTATE_CAREFULL_LAND) ||
			  (navstate == NAVSTATE_LAND))
	  {
		  return 0;
	  }
  }

  if(     (navstate == NAVSTATE_OFF) ||
		  (navstate == NAVSTATE_LANDED) ||
		  (navstate == NAVSTATE_LAND_WO_POSHOLD) ||
		  (navstate == NAVSTATE_CHECK_SERVO))
  {
	  return 0;
  }
  if( navstate == NAVSTATE_LAUNCH ){
	  if (cur_pos.h >= systemConfig()->cx_min_tkoff_land_now_alt) {
		  poshold_origin.h = cur_pos.h;
		  ics_set_state(NAVSTATE_LAND_WO_POSHOLD);
	  }
	  return 0;
  }

  if(     (navstate == NAVSTATE_PRELAUNCH) ||
		  (navstate == NAVSTATE_AUTONOMOUS_PREPARING))
  {
	  ics_set_state( NAVSTATE_OFF );
	  return 0;
  }



  preland_time = 0;
  ics_set_state(NAVSTATE_PRELAND_1);

  cur_pos.throttle_origin = rcCommand[THROTTLE];

  if( is_emergency_landing > 0 ){
	  poshold_origin.h = cur_pos.h;
	  poshold_origin.la = cur_pos.la;
  	  poshold_origin.lo = cur_pos.lo;
  	  poshold_origin.heading = cur_pos.mag_az;
  	  //Quick landing as possibly
  	  if( is_emergency_landing > 1 ){
  		  ics_set_state(NAVSTATE_LAND_WO_POSHOLD);
  	  }
  } else {
	  //FIX RND-45
	  if( systemConfig()->cx_preland_use_current_height == 1){
		  poshold_origin.h = cur_pos.h;
	  } else {
		  poshold_origin.h = (XM_GetCurrent().h - XM_GetHome().h);
	  }
	  poshold_origin.la = XM_GetCurrent().la;
	  poshold_origin.lo = XM_GetCurrent().lo;
	  poshold_origin.heading = cur_pos.mag_az;
  }


  return 1;
}

uint16_t rth_h;

uint8_t ics_start_telemetry_lost(){
	if(   (navstate == NAVSTATE_OFF) ||
		  (navstate == NAVSTATE_LAND) ||
		  (navstate == NAVSTATE_CAREFULL_LAND) ||
		  (navstate == NAVSTATE_LANDED) ||
		  (navstate == NAVSTATE_LAND_WO_POSHOLD) ||
		  (navstate == NAVSTATE_CHECK_SERVO) ||
		  (navstate == NAVSTATE_PRELAND_1) ||
		  (navstate == NAVSTATE_PRELAND_2) ||
		  (navstate == NAVSTATE_PRELAND_3) ||
		  (navstate == NAVSTATE_RTH_LAND) ||
		  (navstate == NAVSTATE_RTH) ||
		  (navstate == NAVSTATE_TELEMETRY_RTH))
	{
	  return 0;
	}


	ics_set_warning( WNG_TELEMETRY_LOST, 1);

	//If we go to land, do not change mode
	if( XM_Current >= XM_Count - 2 ){
		return 0;
	}

	if( navstate == NAVSTATE_LAUNCH ){
		if (cur_pos.h >= systemConfig()->cx_min_tkoff_land_now_alt) {
		  poshold_origin.h = cur_pos.h;
		  ics_set_state(NAVSTATE_LAND_WO_POSHOLD);
		}
	  return 0;
	}

	if(     (navstate == NAVSTATE_PRELAUNCH) ||
		  (navstate == NAVSTATE_AUTONOMOUS_PREPARING))
	{
	  ics_set_state(NAVSTATE_OFF);
	  return 0;
	}

//	rth_h = systemConfig()->cx_launch_height;
//	if( ram_rth_safe_h * 100 > rth_h ){
//		rth_h = ram_rth_safe_h * 100;
//	}

//	if( cur_pos.h < rth_h){
//		cur_pos.home_h = rth_h;
//	} else {
//		cur_pos.home_h = cur_pos.h ;
//	}

	//cur_pos.home_h = ram_rth_safe_h;
	cur_pos.home_h = ram_rth_safe_h * 100;

	if( systemConfig()->cx_telemetry_lost_rth_or_land == 1 ){
		XM_SetCurrent(0);
		x_preline_off();
		ics_set_state(NAVSTATE_TELEMETRY_RTH);
	} else {
		//Home and LAND
		//Circles OFF
		x_preline_off();
		ics_goto_land();
	}
	return 1;
}

float cur_speed_err;
float prev_speed_err;
float speed_ierr;

float alt_err_tho;
float prev_alt_err_tho;
float alt_ierr_tho;

float throttle_res = 0;
float throttle_alt_res = 0;
float throttle_spd_res = 0;
float ics_as_controler( double dt, uint16_t need_as_in_cm, uint32_t need_alt ){
    cur_speed_err = 1.0f * need_as_in_cm - get_cur_as();						// ошибка прямолинейной скорости
    alt_err_tho =  1.0f * need_alt - 1.0f * getEstimatedActualPosition(Z);		// ошибка высоты
    #define MAX_SPEED_I_PART (300)

//    throttle_alt_res = ics_pid(
//	        dt,
//			alt_err_tho, &prev_alt_err_tho, &alt_ierr_tho,
//			systemConfig()->cx_alt_t_p,
//			systemConfig()->cx_alt_t_i,
//			systemConfig()->cx_alt_t_d,
//			MAX_SPEED_I_PART
//	);
    throttle_alt_res = alt_err_tho * systemConfig()->cx_alt_t_p - cur_pos.hspd * systemConfig()->cx_alt_t_d;	// результат расчета ПИДа высоты (ПД)

    cur_pos.alt_thro_pid_res = throttle_alt_res;

    throttle_spd_res = ics_pid(

    //spd_res = -systemConfig()->cx_alt_t_p * cur_alt_err + ics_pid(
    //spd_res = ics_pid(
    		        dt,
        cur_speed_err, &prev_speed_err, &speed_ierr,
        systemConfig()->cx_as_pid_p,
        systemConfig()->cx_as_pid_i,
        systemConfig()->cx_as_pid_d,
        MAX_SPEED_I_PART

    ); // результат расчета ПИДа прямолинейной скорости

	if(
			(attitude.values.roll > systemConfig()->cx_throttle_mix_procent_roll_limit * 10)  ||
			(attitude.values.roll < -systemConfig()->cx_throttle_mix_procent_roll_limit * 10)
	){
		throttle_res -= rcCommand[ROLL] * (systemConfig()->cx_throttle_mix_procent / 100.0);
	}

    throttle_res = throttle_alt_res + throttle_spd_res;	// сумма ПИДов
    cur_pos.thro_pid_res = throttle_res;

#if defined(USE_TELEMETRY_XLOGGER)
    cur_PID.flight_thr.curTime = millis();
    cur_PID.flight_thr.need_alt = need_alt;
    cur_PID.flight_thr.alt_err_tho = alt_err_tho;
    cur_PID.flight_thr.cx_alt_t_p_cont = alt_err_tho * systemConfig()->cx_alt_t_p;
    cur_PID.flight_thr.cx_alt_t_i_cont = 0;
    cur_PID.flight_thr.cx_alt_t_d_cont = - cur_pos.hspd * systemConfig()->cx_alt_t_d;
    cur_PID.flight_thr.throttle_alt_res = throttle_alt_res;
    cur_PID.flight_thr.need_as_in_cm = need_as_in_cm;
    cur_PID.flight_thr.cur_speed_err = cur_speed_err;
    cur_PID.flight_thr.cx_as_pid_p_cont = cur_PID.buff_p_cont;
    cur_PID.flight_thr.cx_as_pid_i_cont = cur_PID.buff_i_cont;
    cur_PID.flight_thr.cx_as_pid_d_cont = cur_PID.buff_d_cont;
    cur_PID.flight_thr.throttle_spd_res = throttle_spd_res;
    cur_PID.flight_thr.throttle_res = throttle_res;
    cur_PID.flight_thr.vbat_compensation_factor = systemConfig()->cx_cruise_throttle*x_get_vbat_compansation_factor();
    cur_PID.flight_thr.new = 1;
#endif


    rcCommand[THROTTLE] = (int)(constrainf(systemConfig()->cx_cruise_throttle*x_get_vbat_compansation_factor() + throttle_res, motorConfig()->minthrottle, systemConfig()->cx_max_route_throttle));
    return 1;
}

float altmc_res;
float cur_altmc_err, prev_altmc_err, altmc_ierr;
float ics_alt_mc_controler( double dt, int32_t need_alt ){
    cur_altmc_err = - 1.0f * getEstimatedActualPosition(Z) + need_alt;
    #define MAX_ALTMC_I_PART (300)
    altmc_res = ics_pid_known_der(
    		        dt,
					cur_altmc_err, &prev_altmc_err, &altmc_ierr,
        ram_altmc_pid_p,
        ram_altmc_pid_i,
        ram_altmc_pid_d,
		MAX_ALTMC_I_PART,
		cur_pos.hspd
    );

    //Moving to back
    if( (poshold_althold_tail_factor > 0.6) && (systemConfig()->cx_yaw_poshold > 0) ){
        rcCommand[THROTTLE] = (int)(constrainf(navConfig()->mc.hover_throttle * x_get_vbat_compansation_factor() + altmc_res, motorConfig()->minthrottle, systemConfig()->cx_max_poshold_tail_throttle));
    } else {
        rcCommand[THROTTLE] = (int)(constrainf(navConfig()->mc.hover_throttle * x_get_vbat_compansation_factor() + altmc_res, motorConfig()->minthrottle, systemConfig()->cx_max_route_throttle));
    }
    return 1;
}

uint8_t new_alt;
uint8_t ics_apply_manual_althold_control( double dt) {
		// apply tgt setting from rc:
		int16_t thr_diff = rcData[THROTTLE] - 1500; // 1500 is center


		// NAVSTATE_ALTHOLD is ONLY MANUAL MODE!!!
		// check if throttle not in center -> increase or decrease altitude

		if (abs(thr_diff) > systemConfig()->cx_new_althold_rc_throttle_deadband){
			// throttle channel not in center
			new_alt = 1;
			int16_t cur_alt_speed_target = scaleRangef(thr_diff, -500, +500, -systemConfig()->cx_new_althold_altspeed_max_cms, systemConfig()->cx_new_althold_altspeed_max_cms );
			ics_alt_speed_controler( dt, cur_alt_speed_target );

		}
		else {
			if (new_alt == 1) {
				poshold_origin.h = cur_pos.h;
				new_alt = 0;
			}
			ics_new_althold_controller( dt, poshold_origin.h, systemConfig()->cx_new_althold_altspeed_max_cms, systemConfig()->cx_new_althold_altspeed_max_cms);
		}
		return 1;

}
uint8_t ics_new_althold_controller( double dt, int32_t need_alt, int32_t alt_speed_min, int32_t alt_speed_max){
	cur_pos.need_h = need_alt;
	double pid_p = systemConfig()->cx_new_althold_alt_pid_p; // P-component from the parameter
	// пределы берутся по модулю, знак "-" добавляется уже в функции
	double error =  (need_alt - getEstimatedActualPosition(Z) * 1.0);
	double alt_pid_res = pid_p * error;
	int16_t cur_alt_speed_target = constrainf( alt_pid_res, -alt_speed_min, alt_speed_max );

	ics_alt_speed_controler( dt, cur_alt_speed_target );
	return 1;
}

float cur_alt_err;
float prev_alt_err;
float alt_ierr;
float alt_res;

float ics_alt_controler( double dt, int32_t need_alt ){
    cur_pos.need_h = need_alt;

    cur_alt_err = 1.0f * getEstimatedActualPosition(Z) - need_alt;
    //Maximum integral part in PWM units
    #define MAX_ALT_I_PART (200)
    alt_res = ics_pid(
        dt, 
        cur_alt_err, &prev_alt_err, &alt_ierr,
        systemConfig()->cx_alt_pid_p,
        systemConfig()->cx_alt_pid_i,
        systemConfig()->cx_alt_pid_d,
        MAX_ALT_I_PART
    ); 

    // limitations
    int16_t max_climb_angle = systemConfig()->cx_fly_maxpitch_at_20ms;
    if( get_cur_as() < 2000 ){
        float gain = (2000.0 - get_cur_as()) / 2000;
        max_climb_angle -=  gain * (systemConfig()->cx_fly_maxpitch_at_20ms - systemConfig()->cx_fly_maxpitch_at_0ms);
    }

    //At 20m/s -- max dive, at 40m/s -- dive = 0
    float max_dive_angle = 1.0 * DEGREES_TO_DECIDEGREES(navConfig()->fw.max_dive_angle);
    if( get_cur_as() > 2000 ){
    	float zero_dive_as = systemConfig()->as_for_zero_dive * 100.0;
        float gain = (zero_dive_as - get_cur_as()) / (zero_dive_as - 2000.0);
        if( gain < 0 ) gain = 0;
        if( gain > 1) gain = 1;
        max_dive_angle *=  gain;
    }

    if( (attitude.values.roll > systemConfig()->cx_ae_infl_threshold_roll) || (-attitude.values.roll < -systemConfig()->cx_ae_infl_threshold_roll) ){
        alt_res -=  ABS( attitude.values.roll / 100.0 * systemConfig()->cx_ae_infl );
    } else {
    }

	if(
			(attitude.values.roll > systemConfig()->cx_pitch_mix_procent_roll_limit * 10)  ||
			(attitude.values.roll < -systemConfig()->cx_pitch_mix_procent_roll_limit * 10)
	){
		alt_res -= rcCommand[ROLL] * (systemConfig()->cx_pitch_mix_procent / 100.0);
	}


    alt_res = constrainf(alt_res, -max_climb_angle, max_dive_angle );

    //alt_res +=  ABS( attitude.values.roll / 100.0 * systemConfig()->cx_ae_infl );
    //alt_res = constrainf(alt_res, -DEGREES_TO_DECIDEGREES(navConfig()->fw.max_dive_angle), max_climb_angle);


#if defined(USE_TELEMETRY_XLOGGER)
    cur_PID.flight_alt.alt_res = alt_res;
    cur_PID.flight_alt.curTime = millis();
    cur_PID.flight_alt.cur_alt_err = cur_alt_err;
    cur_PID.flight_alt.need_alt = need_alt;
    cur_PID.flight_alt.cx_alt_pid_p_cont = cur_PID.buff_p_cont;
    cur_PID.flight_alt.cx_alt_pid_i_cont = cur_PID.buff_i_cont;
    cur_PID.flight_alt.cx_alt_pid_d_cont = cur_PID.buff_d_cont;
    cur_PID.flight_alt.max_climb_angle = max_climb_angle;
    cur_PID.flight_alt.max_dive_angle = max_dive_angle;
    cur_PID.flight_alt.new = 1;
#endif

    rcCommand[PITCH] = pidAngleToRcCommand( alt_res - systemConfig()->cx_planemode_pitch_angle_decidegi, pidProfile()->max_angle_inclination[FD_PITCH]);

    return 1;
}



float desc_res = 0;
float ics_descend_controler( double dt, uint32_t need_as ){
	desc_err = 1.0f * need_as - get_cur_as();//posControl.actualState.velXY;


    //Maximum integral part in PWM units
    #define MAX_DESC_I_PART (200)
	desc_res = ics_pid_known_der(
        dt,
		desc_err, &prev_desc_err, &desc_ierr,
        systemConfig()->cx_desc_pid_p,
        systemConfig()->cx_desc_pid_i,
        systemConfig()->cx_desc_pid_d,
		MAX_DESC_I_PART,
		cur_pos.hspd
    );

    int16_t max_climb_angle = systemConfig()->cx_fly_maxpitch_at_20ms;
    if( get_cur_as() < 2000 ){
        float gain = (2000.0 - get_cur_as()) / 2000;
        max_climb_angle -=  gain * (systemConfig()->cx_fly_maxpitch_at_20ms - systemConfig()->cx_fly_maxpitch_at_0ms);
    }

    desc_res -=  ABS( attitude.values.roll / 100.0 * systemConfig()->cx_ae_infl );
    desc_res = constrainf(alt_res, -max_climb_angle, DEGREES_TO_DECIDEGREES(navConfig()->fw.max_dive_angle) );

    rcCommand[PITCH] = pidAngleToRcCommand( desc_res - systemConfig()->cx_planemode_pitch_angle_decidegi, pidProfile()->max_angle_inclination[FD_PITCH]);
    rcCommand[THROTTLE] = motorConfig()->minthrottle;
    return 1;
}


float ics_alt_speed_controler( double dt, int16_t need_alt_speed ){
    cur_alts_err = 1.0f * need_alt_speed - cur_pos.hspd;
    //Maximum integral part in PWM units
    #define MAX_ALTS_I_PART (300)
    //alts_res = cur_alts_err * systemConfig()->cx_alt_spd_pid_p;
    alts_res = ics_pid(
        dt, 
        cur_alts_err, &prev_alts_err, &alts_ierr,
        systemConfig()->cx_alt_spd_pid_p,
        systemConfig()->cx_alt_spd_pid_i,
        systemConfig()->cx_alt_spd_pid_d,
        systemConfig()->cx_new_althold_speed_hor_pid_maxI
    ); 


#if defined(USE_TELEMETRY_XLOGGER)
    cur_PID.landing.curTime = millis();
    cur_PID.landing.need_alt_speed = need_alt_speed;
    cur_PID.landing.cur_alts_err = cur_alts_err;
    cur_PID.landing.cx_alt_spd_pid_p_cont = cur_PID.buff_p_cont;
    cur_PID.landing.cx_alt_spd_pid_i_cont = cur_PID.buff_i_cont;
    cur_PID.landing.cx_alt_spd_pid_d_cont = cur_PID.buff_d_cont;
    cur_PID.landing.alts_res = alts_res;
    cur_PID.landing.new = 1;
#endif
    static double thr_corr;
    // TODO: убрать костыль
    if( (cur_pos.throttle_origin > 900) && (systemConfig()->cx_new_althold_use_origin > 0) ){
    	rcCommand[THROTTLE] = (int)(constrainf( cur_pos.throttle_origin + alts_res, motorConfig()->minthrottle, motorConfig()->maxthrottle));
    }
//    else if (systemConfig()->cx_new_althold_sqcomp_gain > 0) {
//    	thr_corr = pow((double)cur_pos.airspd * systemConfig()->cx_new_althold_sqcomp_gain, 2);
//    	rcCommand[THROTTLE] = (int)(constrainf( navConfig()->mc.hover_throttle + alts_res + (int)thr_corr, motorConfig()->minthrottle, motorConfig()->maxthrottle));
//    }
    else {

    	rcCommand[THROTTLE] = (int)(constrainf( navConfig()->mc.hover_throttle + alts_res, motorConfig()->minthrottle, motorConfig()->maxthrottle));
    	}

    return 1;
}

float ics_pid( float dt, float err, float *olderr, float *ierr, float p, float i, float d, float maxi ){
    float de = err - (*olderr);
    *olderr = err * 0.1 + (*olderr) * 0.9;

    (*ierr) = (*ierr) + err * i * dt;
    if( (*ierr) > maxi ){
    	(*ierr) = maxi;
    } else if( (*ierr) < -maxi ){
    	(*ierr) = -maxi;
    }

    float deriv = 0;
    if( d > 0.01 ){
      deriv = de * d / dt;
      if( deriv > maxi ){
    	  deriv = maxi;
      }
      if( deriv < -maxi ){
    	  deriv = -maxi;
      }
    }
//components are send to buffer. need to catch them after ics_pid call if we want to log it
#if defined(USE_TELEMETRY_XLOGGER)
    cur_PID.buff_p_cont = err*p;
    cur_PID.buff_i_cont = *ierr;
    cur_PID.buff_d_cont = deriv;
#endif

    return (float)(err * p + deriv + (*ierr));
}

uint8_t ics_on_vector_mode_message( uint8_t enabled, int32_t d_az, int32_t d_as, int32_t d_h ){
#define ICS_ONE_DEGREE_LONG (1.113195f)
	float dla = (d_az*10) / 1.113195f;
	float dlo = (d_h) / ICS_ONE_DEGREE_LONG / cos_approx(cur_pos.la * 3.14 / 180.0 / 10000000);

	poshold_origin.la += dla * systemConfig()->cx_keyboard_la_scale;
    poshold_origin.lo += dlo * systemConfig()->cx_keyboard_lo_scale;

    cur_alt_change += d_h * systemConfig()->cx_keyboard_h_scale;
    return 1;
}


float ics_pid_known_der( float dt, float err, float *olderr, float *ierr, float p, float i, float d, float maxi, float der ){
    *olderr = err * 0.1 + (*olderr) * 0.9;

    (*ierr) = (*ierr) + err * i * dt;
    if( (*ierr) > maxi ){
    	(*ierr) = maxi;
    } else if( (*ierr) < -maxi ){
    	(*ierr) = -maxi;
    }

    float deriv = der;
    return (float)(err * p + deriv * d + (*ierr));
}

uint16_t get_cruise_as( void ){
	if( cur_pos.gndspd < 600 ){
		return systemConfig()->cx_cruise_as + 800 + (600 - cur_pos.gndspd)*4;
	}
	if( cur_pos.gndspd < 1000 ){
		return systemConfig()->cx_cruise_as + (1000 - cur_pos.gndspd)*2;
	}
	return systemConfig()->cx_cruise_as;
}

uint16_t get_cur_as(void){
	if(ram_use_only_gps_speed == 1){
		return cur_pos.gndspd;
	}
	if( cur_pos.is_use_airspeed ){
		if( cur_pos.airspd < 6000 ){
			is_as_error = 0;
			return cur_pos.airspd;
		}
	}
	is_as_error = 1;
	return cur_pos.gndspd;
}

void clear_all_filters(void){
	ics_clearVoltageFilters();

	alts_ierr = 0;
	desc_ierr = 0;
	speed_ierr = 0;
	alt_ierr_tho = 0;
	altmc_ierr = 0;
	alt_ierr = 0;
	alts_ierr = 0;

	speed_X_ierr = 0;
	speed_Y_ierr = 0;


}

uint8_t drift_mode_on = 0;
int drift_prev_la = 0;
int drift_prev_lo = 0;

void check_drift(){
	drift_prev_la = cur_pos.la;
	drift_prev_lo = cur_pos.lo;

	if( drift_mode_on == 2 ){
		cur_pos.drift_area += x_get_drift_area( poshold_origin, drift_prev_la, drift_prev_lo);
	}
	if( drift_mode_on == 1 ){
		//fill prev value
		drift_mode_on = 2;
	}
}

