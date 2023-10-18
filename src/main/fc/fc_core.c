/*
 * This file is part of GTNAV.
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

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>

#include "platform.h"

#include "blackbox/blackbox.h"

#include "build/debug.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"
#include "common/utils.h"
#include "common/filter.h"

#include "drivers/light_led.h"
#include "drivers/serial.h"
#include "drivers/time.h"

#include "sensors/sensors.h"
#include "sensors/diagnostics.h"
#include "sensors/boardalignment.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/pitotmeter.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"
#include "sensors/rangefinder.h"
#include "sensors/opflow.h"

#include "fc/fc_core.h"
#include "fc/cli.h"
#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_controls.h"
#include "fc/rc_curves.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "io/beeper.h"
#include "io/dashboard.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/serial.h"
#include "io/statusindicator.h"
#include "io/asyncfatfs/asyncfatfs.h"

#include "msp/msp_serial.h"

#include "navigation/navigation.h"
#include "navigation/icsnav.h"
#include "navigation/apgeo.h"

#include "rx/rx.h"
#include "rx/msp.h"
#include "rx/sbus.h"


#include "scheduler/scheduler.h"

#include "telemetry/telemetry.h"
#include "telemetry/xlogger.h"

#include "flight/mixer.h"
#include "flight/servos.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/payload.h"

#include "flight/failsafe.h"

#include "config/feature.h"


timeUs_t next_operation;
timeUs_t autostart_timeout;
uint8_t operation_step;
uint16_t msp_chan[8];
uint16_t calculated_altitude;
uint16_t current_alt, prev_alt;


extern TicsPos cur_pos;
extern uint8_t navstate;

enum {
    ALIGN_GYRO = 0,
    ALIGN_ACCEL = 1,
    ALIGN_MAG = 2
};

#define GYRO_WATCHDOG_DELAY 100  // Watchdog for boards without interrupt for gyro

timeDelta_t cycleTime = 0;         // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop

float dT;

int16_t headFreeModeHold;

uint8_t motorControlEnable = false;

static uint32_t disarmAt;     // Time of automatic disarm when "Don't spin the motors when armed" is enabled and auto_disarm_delay is nonzero

static bool isRXDataNew;
static disarmReason_t lastDisarmReason = DISARM_NONE;



//emulate rc command to set some function
void SetAuxFunc(uint16_t in,uint16_t low,uint16_t high){
	msp_chan[in] = low + ((high - low)/2);
	rxMspWriteRawRC(msp_chan);
}
bool GetAuxFunc(uint16_t in,uint16_t low,uint16_t high){
	return (msp_chan[in]<=high&& msp_chan[in]>=low);
}

bool isCalibrating(void)
{
#ifdef USE_BARO
    if (sensors(SENSOR_BARO) && !baroIsCalibrationComplete()) {
        return true;
    }
#endif

#ifdef USE_PITOT
    if (sensors(SENSOR_PITOT) && !pitotIsCalibrationComplete()) {
        return true;
    }
#endif

#ifdef USE_NAV
    if (!navIsCalibrationComplete()) {
        return true;
    }
#endif

    if (!accIsCalibrationComplete() && sensors(SENSOR_ACC)) {
        return true;
    }

    if (!gyroIsCalibrationComplete()) {
        return true;
    }

    return false;
}

int16_t getAxisRcCommand(int16_t rawData, int16_t rate, int16_t deadband)
{
    int16_t stickDeflection;

    stickDeflection = constrain(rawData - rxConfig()->midrc, -500, 500);
    stickDeflection = applyDeadband(stickDeflection, deadband);

    return rcLookup(stickDeflection, rate);
}

static void updateArmingStatus(void)
{
    if (ARMING_FLAG(ARMED)) {
        LED0_ON;
    } else {
        /* CHECK: Run-time calibration */
        static bool calibratingFinishedBeep = false;
        if (isCalibrating()) {
            ENABLE_ARMING_FLAG(ARMING_DISABLED_SENSORS_CALIBRATING);
            calibratingFinishedBeep = false;
        }
        else {
            DISABLE_ARMING_FLAG(ARMING_DISABLED_SENSORS_CALIBRATING);

            if (!calibratingFinishedBeep) {
                calibratingFinishedBeep = true;
                beeper(BEEPER_RUNTIME_CALIBRATION_DONE);
            }
        }

        /* CHECK: RX signal */
        DISABLE_ARMING_FLAG(ARMING_DISABLED_RC_LINK);
        // if (!failsafeIsReceivingRxData()) {
            // ENABLE_ARMING_FLAG(ARMING_DISABLED_RC_LINK);
        // }
        // else {
            // DISABLE_ARMING_FLAG(ARMING_DISABLED_RC_LINK);
        // }

        /* CHECK: Throttle */
        if (!armingConfig()->fixed_wing_auto_arm) {
            // Don't want this check if fixed_wing_auto_arm is in use - machine arms on throttle > LOW
            if (calculateThrottleStatus() != THROTTLE_LOW) {
                ENABLE_ARMING_FLAG(ARMING_DISABLED_THROTTLE);
            } else {
                DISABLE_ARMING_FLAG(ARMING_DISABLED_THROTTLE);
            }
        }

	/* CHECK: pitch / roll sticks centered when NAV_LAUNCH_MODE enabled */
	if (isNavLaunchEnabled()) {
	  if (areSticksDeflectedMoreThanPosHoldDeadband()) {
	    ENABLE_ARMING_FLAG(ARMING_DISABLED_ROLLPITCH_NOT_CENTERED);
	  } else {
	    DISABLE_ARMING_FLAG(ARMING_DISABLED_ROLLPITCH_NOT_CENTERED);
	  }
	}

        /* CHECK: Angle */
        if (!STATE(SMALL_ANGLE)) {
            ENABLE_ARMING_FLAG(ARMING_DISABLED_NOT_LEVEL);
        }
        else {
            DISABLE_ARMING_FLAG(ARMING_DISABLED_NOT_LEVEL);
        }

        /* CHECK: CPU load */
        if (isSystemOverloaded()) {
            ENABLE_ARMING_FLAG(ARMING_DISABLED_SYSTEM_OVERLOADED);
        }
        else {
            DISABLE_ARMING_FLAG(ARMING_DISABLED_SYSTEM_OVERLOADED);
        }
        
#if defined(USE_NAV)
        /* CHECK: Navigation safety */
        if (navigationBlockArming()) {
            ENABLE_ARMING_FLAG(ARMING_DISABLED_NAVIGATION_UNSAFE);
        }
        else {
            DISABLE_ARMING_FLAG(ARMING_DISABLED_NAVIGATION_UNSAFE);
        }
#endif

#if defined(USE_MAG)
        /* CHECK: */
        if (sensors(SENSOR_MAG) && !STATE(COMPASS_CALIBRATED)) {
            ENABLE_ARMING_FLAG(ARMING_DISABLED_COMPASS_NOT_CALIBRATED);
        }
        else {
            DISABLE_ARMING_FLAG(ARMING_DISABLED_COMPASS_NOT_CALIBRATED);
        }
#endif

        /* CHECK: */
        if (sensors(SENSOR_ACC) && !STATE(ACCELEROMETER_CALIBRATED)) {
            ENABLE_ARMING_FLAG(ARMING_DISABLED_ACCELEROMETER_NOT_CALIBRATED);
        }
        else {
            DISABLE_ARMING_FLAG(ARMING_DISABLED_ACCELEROMETER_NOT_CALIBRATED);
        }

        /* CHECK: */
        if (!isHardwareHealthy()) {
            ENABLE_ARMING_FLAG(ARMING_DISABLED_HARDWARE_FAILURE);
        }        
        else {
            DISABLE_ARMING_FLAG(ARMING_DISABLED_HARDWARE_FAILURE);
        }

        /* CHECK: Arming switch */
        //if RC is present
        if (!failsafeIsReceivingRxData()) {
            if (!isUsingSticksForArming()) {
                // If arming is disabled and the ARM switch is on
                if (isArmingDisabled() && IS_RC_MODE_ACTIVE(BOXARM)) {
                    ENABLE_ARMING_FLAG(ARMING_DISABLED_ARM_SWITCH);
                } else if (!IS_RC_MODE_ACTIVE(BOXARM)) {
                    DISABLE_ARMING_FLAG(ARMING_DISABLED_ARM_SWITCH);
                }
            }
        } else {
            DISABLE_ARMING_FLAG(ARMING_DISABLED_ARM_SWITCH);
        }

        /* CHECK: BOXFAILSAFE */
        if (IS_RC_MODE_ACTIVE(BOXFAILSAFE)) {
            ENABLE_ARMING_FLAG(ARMING_DISABLED_BOXFAILSAFE);
        }
        else {
            DISABLE_ARMING_FLAG(ARMING_DISABLED_BOXFAILSAFE);
        }

        /* CHECK: BOXFAILSAFE */
        if (IS_RC_MODE_ACTIVE(BOXKILLSWITCH)) {
            ENABLE_ARMING_FLAG(ARMING_DISABLED_BOXKILLSWITCH);
        }
        else {
            DISABLE_ARMING_FLAG(ARMING_DISABLED_BOXKILLSWITCH);
        }
        /* CHECK: Do not allow arming if Servo AutoTrim is enabled */
        if (IS_RC_MODE_ACTIVE(BOXAUTOTRIM)) {
	    ENABLE_ARMING_FLAG(ARMING_DISABLED_SERVO_AUTOTRIM);
	    } 
        else {
	    DISABLE_ARMING_FLAG(ARMING_DISABLED_SERVO_AUTOTRIM);
	    }

        if (isArmingDisabled()) {
            warningLedFlash();
        } else {
            warningLedDisable();
        }

        warningLedUpdate();
    }
}

void annexCode(void)
{
    int32_t throttleValue;

    if (failsafeShouldApplyControlInput()) {
        // Failsafe will apply rcCommand for us
        failsafeApplyControlInput();
    }
    else {
        // Compute ROLL PITCH and YAW command
        rcCommand[ROLL] = getAxisRcCommand(rcData[ROLL], FLIGHT_MODE(MANUAL_MODE) ? currentControlRateProfile->manual.rcExpo8 : currentControlRateProfile->stabilized.rcExpo8, rcControlsConfig()->deadband);
        rcCommand[PITCH] = getAxisRcCommand(rcData[PITCH], FLIGHT_MODE(MANUAL_MODE) ? currentControlRateProfile->manual.rcExpo8 : currentControlRateProfile->stabilized.rcExpo8, rcControlsConfig()->deadband);
        rcCommand[YAW] = -getAxisRcCommand(rcData[YAW], FLIGHT_MODE(MANUAL_MODE) ? currentControlRateProfile->manual.rcYawExpo8 : currentControlRateProfile->stabilized.rcYawExpo8, rcControlsConfig()->yaw_deadband);

        // Apply manual control rates
        if (FLIGHT_MODE(MANUAL_MODE)) {
            rcCommand[ROLL] = rcCommand[ROLL] * currentControlRateProfile->manual.rates[FD_ROLL] / 100L;
            rcCommand[PITCH] = rcCommand[PITCH] * currentControlRateProfile->manual.rates[FD_PITCH] / 100L;
            rcCommand[YAW] = rcCommand[YAW] * currentControlRateProfile->manual.rates[FD_YAW] / 100L;
        }

        //Compute THROTTLE command
        throttleValue = constrain(rcData[THROTTLE], rxConfig()->mincheck, PWM_RANGE_MAX);
        throttleValue = (uint32_t)(throttleValue - rxConfig()->mincheck) * PWM_RANGE_MIN / (PWM_RANGE_MAX - rxConfig()->mincheck);       // [MINCHECK;2000] -> [0;1000]
        rcCommand[THROTTLE] = rcLookupThrottle(throttleValue);

        // Signal updated rcCommand values to Failsafe system
        failsafeUpdateRcCommandValues();

        if (FLIGHT_MODE(HEADFREE_MODE)) {
            const float radDiff = degreesToRadians(DECIDEGREES_TO_DEGREES(attitude.values.yaw) - headFreeModeHold);
            const float cosDiff = cos_approx(radDiff);
            const float sinDiff = sin_approx(radDiff);
            const int16_t rcCommand_PITCH = rcCommand[PITCH] * cosDiff + rcCommand[ROLL] * sinDiff;
            rcCommand[ROLL] = rcCommand[ROLL] * cosDiff - rcCommand[PITCH] * sinDiff;
            rcCommand[PITCH] = rcCommand_PITCH;
        }
    }

    updateArmingStatus();
}

void mwDisarm(disarmReason_t disarmReason)
{
    if (ARMING_FLAG(ARMED)) {
        lastDisarmReason = disarmReason;
        DISABLE_ARMING_FLAG(ARMED);


#ifdef USE_BLACKBOX
        if (feature(FEATURE_BLACKBOX)) {
            blackboxFinish();
        }
#endif

        statsOnDisarm();
        xlogger_send_arm_disarm_result(0);	// write to xlogger that we have disarmed vehicle
        beeper(BEEPER_DISARMING);      // emit disarm tone
    }

}

disarmReason_t getDisarmReason(void)
{
    return lastDisarmReason;
}

#define TELEMETRY_FUNCTION_MASK (FUNCTION_TELEMETRY_FRSKY | FUNCTION_TELEMETRY_HOTT | FUNCTION_TELEMETRY_SMARTPORT | FUNCTION_TELEMETRY_LTM | FUNCTION_TELEMETRY_GT | FUNCTION_TELEMETRY_IBUS)

void releaseSharedTelemetryPorts(void) {
    serialPort_t *sharedPort = findSharedSerialPort(TELEMETRY_FUNCTION_MASK, FUNCTION_MSP);
    while (sharedPort) {
        mspSerialReleasePortIfAllocated(sharedPort);
        sharedPort = findNextSharedSerialPort(TELEMETRY_FUNCTION_MASK, FUNCTION_MSP);
    }
}

void mwArm(void)
{
    updateArmingStatus();




    if (!isArmingDisabled()) {
        if (ARMING_FLAG(ARMED)) {
            return;
        }

        ENABLE_ARMING_FLAG(ARMED);
        ENABLE_ARMING_FLAG(WAS_EVER_ARMED);

        headFreeModeHold = DECIDEGREES_TO_DEGREES(attitude.values.yaw);

        resetHeadingHoldTarget(DECIDEGREES_TO_DEGREES(attitude.values.yaw));
        xlogger_send_arm_disarm_result(1);		// write to xlogger that we are trying to arm

        XM_setHome();

#ifdef USE_BLACKBOX
        if (feature(FEATURE_BLACKBOX)) {
            serialPort_t *sharedBlackboxAndMspPort = findSharedSerialPort(FUNCTION_BLACKBOX, FUNCTION_MSP);
            if (sharedBlackboxAndMspPort) {
                mspSerialReleasePortIfAllocated(sharedBlackboxAndMspPort);
            }
            blackboxStart();
        }
#endif
        disarmAt = millis() + armingConfig()->auto_disarm_delay * 1000;   // start disarm timeout, will be extended when throttle is nonzero

        //beep to indicate arming
#ifdef USE_NAV
        if (navigationPositionEstimateIsHealthy())
            beeper(BEEPER_ARMING_GPS_FIX);
        else
            beeper(BEEPER_ARMING);
#else
        beeper(BEEPER_ARMING);
#endif
        statsOnArm();

#ifdef USE_RANGEFINDER
        /*
         * Since each arm can happen over different surface type, we have to reset
         * previously computed max. dynamic range threshold
         */ 
        rangefinderResetDynamicThreshold();
#endif

        return;
    }



    if (!ARMING_FLAG(ARMED)) {
        beeperConfirmationBeeps(1);
        xlogger_send_arm_disarm_result(0);		// write to xlogger that we are trying to arm
    }

}

void processRx(timeUs_t currentTimeUs)
{
    static bool armedBeeperOn = false;

    calculateRxChannelsAndUpdateFailsafe(currentTimeUs);

    // in 3D mode, we need to be able to disarm by switch at any time
    if (feature(FEATURE_3D)) {
        if (!IS_RC_MODE_ACTIVE(BOXARM))
            mwDisarm(DISARM_SWITCH_3D);
    }

    updateRSSI(currentTimeUs);

    // Update failsafe monitoring system
    if (currentTimeUs > FAILSAFE_POWER_ON_DELAY_US && !failsafeIsMonitoring()) {
        failsafeStartMonitoring();
    }

    //check failsafeOff RC mode active and change state
	//failsafeSetOff(isFailOffActive());
    
    failsafeUpdateState();

    // if (isFailsafeOffActive()){
		// //if failsafeOff and ARM start autopilot sequence 
		// if (rxMspReadRawRC(0,0)==1500 && rxMspReadRawRC(0,1)==1500 && rxMspReadRawRC(0,2)==1000 && rxMspReadRawRC(0,3)==1500 && ARMING_FLAG(ARMED)){ 
			// next_operation = NEXT_CYCLE_TIMEOUT + currentTimeUs;
			// msp_chan[0] = 1500;
			// msp_chan[1] = 1500;
			// msp_chan[2] = 1010;
			// msp_chan[3] = 1500;
			// msp_chan[4] = rxMspReadRawRC(0,4);
			// msp_chan[5] = rxMspReadRawRC(0,5);
			// msp_chan[6] = rxMspReadRawRC(0,6);
			// msp_chan[7] = rxMspReadRawRC(0,7);
			// rxMspWriteRawRC(msp_chan);
			// operation_step = 1;
			// //get altitude first WP
			// navWaypoint_t * wpData;
			// getWaypoint(1, wpData);
			// calculated_altitude = wpData->alt/100; // m
			// // initial baro alt
			// prev_alt=baroGetLatestAltitude()/100;
		// }
		// if (currentTimeUs>next_operation){
			// switch (operation_step){
				// case 1:
                    // if( systemConfig()->cx_autostart_poshold_enable == 1 ){
					    // SetAuxFunc(PosHoldAuxChannel, PosHoldAuxStart, PosHoldAuxStop);
                    // }
					// SetAuxFunc(AngleAuxChannel, AngleAuxStart, AngleAuxStop);
					// operation_step++;
					// next_operation = NEXT_CYCLE_TIMEOUT + currentTimeUs;
					// break;
				// case 2:
					// msp_chan[2] = navConfig()->general.autostart_throttle;
					// rxMspWriteRawRC(msp_chan);
					// operation_step++;
					// next_operation = NEXT_CYCLE_TIMEOUT + currentTimeUs;
					// //setup launch timeout
					// autostart_timeout = navConfig()->general.autostart_timeout*1000000 + currentTimeUs;// us
					// break;
				// case 3:
					// //if low speed -> increase throttle
					// next_operation = navConfig()->general.autostart_update_time*1000 + currentTimeUs;
					// if(navConfig()->general.autostart_baro_only){
						// current_alt = baroGetLatestAltitude()/100;
						// if (((current_alt - prev_alt)/navConfig()->general.autostart_update_time/1000)<navConfig()->general.autostart_vertical_speed) // m/s
							// if (msp_chan[2]<2000-navConfig()->general.autostart_throttle_adds)
								// msp_chan[2] += navConfig()->general.autostart_throttle_adds;
						// prev_alt=baroGetLatestAltitude()/100;
					// } else {
						// current_alt =  getEstimatedActualPosition(Z)/100;
						// if (getEstimatedActualVelocity(Z)/100<navConfig()->general.autostart_vertical_speed) // m/s
							// if (msp_chan[2]<2000-navConfig()->general.autostart_throttle_adds)
								// msp_chan[2] += navConfig()->general.autostart_throttle_adds;
					// }
					// rxMspWriteRawRC(msp_chan);
					// //if the alt of the first WP or timeout is reached -> move on
					// if ((autostart_timeout<= currentTimeUs)||(calculated_altitude<=current_alt)) //m
						// operation_step++;
					// break;
				// case 4:
					// SetAuxFunc(AltHoldAuxChannel, AltHoldAuxStart, AltHoldAuxStop);
					// operation_step++;
					// next_operation = NEXT_CYCLE_TIMEOUT + currentTimeUs;
					// break;
				// case 5:
					// SetAuxFunc(NavWPAuxChannel, NavWPAuxStart, NavWPAuxStop);
					// operation_step++;
					// next_operation = NEXT_CYCLE_TIMEOUT + currentTimeUs;
					// break;
			// }
		// }
	// }






















    
    
    
    
    
    
    
    

    
    
    const throttleStatus_e throttleStatus = calculateThrottleStatus();

    // When armed and motors aren't spinning, do beeps and then disarm
    // board after delay so users without buzzer won't lose fingers.
    // mixTable constrains motor commands, so checking  throttleStatus is enough
    if (ARMING_FLAG(ARMED)
        && feature(FEATURE_MOTOR_STOP)
        && !STATE(FIXED_WING)
    ) {
        if (isUsingSticksForArming()) {
            if (throttleStatus == THROTTLE_LOW) {
                if (armingConfig()->auto_disarm_delay != 0
                    && (int32_t)(disarmAt - millis()) < 0
                ) {
                    // auto-disarm configured and delay is over
                    mwDisarm(DISARM_TIMEOUT);
                    armedBeeperOn = false;
                } else {
                    // still armed; do warning beeps while armed
                    beeper(BEEPER_ARMED);
                    armedBeeperOn = true;
                }
            } else {
                // throttle is not low
                if (armingConfig()->auto_disarm_delay != 0) {
                    // extend disarm time
                    disarmAt = millis() + armingConfig()->auto_disarm_delay * 1000;
                }

                if (armedBeeperOn) {
                    beeperSilence();
                    armedBeeperOn = false;
                }
            }
        } else {
            // arming is via AUX switch; beep while throttle low
            if (throttleStatus == THROTTLE_LOW) {
                beeper(BEEPER_ARMED);
                armedBeeperOn = true;
            } else if (armedBeeperOn) {
                beeperSilence();
                armedBeeperOn = false;
            }
        }
    }

    processRcStickPositions(throttleStatus, armingConfig()->disarm_kill_switch, armingConfig()->fixed_wing_auto_arm);

    //if( rxIsReceivingSignal() ){
    if( 1==1 ){
        updateActivatedModes();
  

        if (!cliMode) {
            updateAdjustmentStates();
            processRcAdjustments(CONST_CAST(controlRateConfig_t*, currentControlRateProfile));
        }

        bool canUseHorizonMode = true;


        if ((IS_RC_MODE_ACTIVE(BOXANGLE) || failsafeRequiresAngleMode() || navigationRequiresAngleMode()) && sensors(SENSOR_ACC)) {
            // bumpless transfer to Level mode
            canUseHorizonMode = false;

            if (!FLIGHT_MODE(ANGLE_MODE)) {
                ENABLE_FLIGHT_MODE(ANGLE_MODE);
            }
        } else {
            DISABLE_FLIGHT_MODE(ANGLE_MODE); // failsafe support
        }

        if (IS_RC_MODE_ACTIVE(BOXHORIZON) && canUseHorizonMode) {

            DISABLE_FLIGHT_MODE(ANGLE_MODE);

            if (!FLIGHT_MODE(HORIZON_MODE)) {
                ENABLE_FLIGHT_MODE(HORIZON_MODE);
            }
        } else {
            DISABLE_FLIGHT_MODE(HORIZON_MODE);
        }

        if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) {
            LED1_ON;
        } else {
            LED1_OFF;
        }

    #ifdef USE_SERVOS
        /* Flaperon mode */
        if (IS_RC_MODE_ACTIVE(BOXFLAPERON) && STATE(FLAPERON_AVAILABLE)) {
            if (!FLIGHT_MODE(FLAPERON)) {
                ENABLE_FLIGHT_MODE(FLAPERON);
            }
        } else {
            DISABLE_FLIGHT_MODE(FLAPERON);
        }
    #endif

    #ifdef USE_FLM_TURN_ASSIST
        /* Turn assistant mode */
        if (IS_RC_MODE_ACTIVE(BOXTURNASSIST)) {
            if (!FLIGHT_MODE(TURN_ASSISTANT)) {
                ENABLE_FLIGHT_MODE(TURN_ASSISTANT);
            }
        } else {
            DISABLE_FLIGHT_MODE(TURN_ASSISTANT);
        }
    #endif

        if (sensors(SENSOR_ACC)) {
            if (IS_RC_MODE_ACTIVE(BOXHEADINGHOLD)) {
                if (!FLIGHT_MODE(HEADING_MODE)) {
                    resetHeadingHoldTarget(DECIDEGREES_TO_DEGREES(attitude.values.yaw));
                    ENABLE_FLIGHT_MODE(HEADING_MODE);
                }
            } else {
                DISABLE_FLIGHT_MODE(HEADING_MODE);
            }
        }

    #if defined(USE_MAG)
        if (sensors(SENSOR_ACC) || sensors(SENSOR_MAG)) {
            if (IS_RC_MODE_ACTIVE(BOXHEADFREE)) {
                if (!FLIGHT_MODE(HEADFREE_MODE)) {
                    ENABLE_FLIGHT_MODE(HEADFREE_MODE);
                }
            } else {
                DISABLE_FLIGHT_MODE(HEADFREE_MODE);
            }
            if (IS_RC_MODE_ACTIVE(BOXHEADADJ)) {
                headFreeModeHold = DECIDEGREES_TO_DEGREES(attitude.values.yaw); // acquire new heading
            }
        }
    #endif

        // Handle passthrough mode
        if (STATE(FIXED_WING)) {
            if ((IS_RC_MODE_ACTIVE(BOXMANUAL) && !navigationRequiresAngleMode() && !failsafeRequiresAngleMode()) ||    // Normal activation of passthrough
                (!ARMING_FLAG(ARMED) && isCalibrating())){                                                              // Backup - if we are not armed - enforce passthrough while calibrating
                ENABLE_FLIGHT_MODE(MANUAL_MODE);
            } else {
                DISABLE_FLIGHT_MODE(MANUAL_MODE);
            }
        }

        /* In airmode Iterm should be prevented to grow when Low thottle and Roll + Pitch Centered.
           This is needed to prevent Iterm winding on the ground, but keep full stabilisation on 0 throttle while in air
           Low Throttle + roll and Pitch centered is assuming the copter is on the ground. Done to prevent complex air/ground detections */
        if (FLIGHT_MODE(MANUAL_MODE) || !ARMING_FLAG(ARMED)) {
            /* In MANUAL mode we reset integrators prevent I-term wind-up (PID output is not used in MANUAL) */
            pidResetErrorAccumulators();
        }
        else {
        	uint8_t cx_navstate_now = ics_get_state();
        	uint8_t cx_use_Iterm_reset = (throttleStatus == THROTTLE_LOW);
        	uint8_t throttleStatusRcCommand = calculateThrottleStatusRcCommand();

        	switch (systemConfig()->cx_use_Iterm_reset_mode) {
        		default:
        			break;
        		case 0:
        			// default - reset everywhere
        			break;
        		case 1:
        			// mode 1 - don't reset in new_poshold
        			if	(ics_navstate_use_new_poshold()) {
        				cx_use_Iterm_reset = 0;
        			}
        			break;
        		case 2:
        			// mode 2 - don't reset in new_poshold except new_poshold wants it to be resetted
        			if	(ics_navstate_use_new_poshold()) {
        				cx_use_Iterm_reset = 0;
        			}
    				if( (ics_navstate_use_new_poshold()) &&  (ics_get_yaw_Iterm_reset()) ) {
    					// if new_poshold says that we want to reset yaw Iterm
    					pidResetErrorAccumulatorAxis(FD_YAW);
    				}
        			break;
        		case 3:
        			// mode 3 - don't reset in new_poshold except new_poshold wants it to be resetted and if throttle is too low
        			if	( (ics_navstate_use_new_poshold()) &&  (throttleStatusRcCommand != THROTTLE_LOW) ) {
        				cx_use_Iterm_reset = 0;
        			}
    				if( (ics_navstate_use_new_poshold()) &&  (ics_get_yaw_Iterm_reset()) ) {
    					// if new_poshold says that we want to reset yaw Iterm
    					pidResetErrorAccumulatorAxis(FD_YAW);
    				}
        			break;
        		case 4:
        			// mode 4 - get Iterm back for all the flight; yaw Iterm is still in reset mode!
        			if	( (cx_navstate_now != NAVSTATE_OFF) &&  (throttleStatusRcCommand != THROTTLE_LOW) ) {
        				cx_use_Iterm_reset = 0;
        				if (!ics_navstate_use_new_poshold()) { //yaw Iterm is still in reset mode!
        					pidResetErrorAccumulatorAxis(FD_YAW);
        				}
        			}
        			if( (ics_navstate_use_new_poshold()) &&  (ics_get_yaw_Iterm_reset()) ) {
    					// if new_poshold says that we want to reset yaw Iterm
    					pidResetErrorAccumulatorAxis(FD_YAW);
    				}

        			break;
        		case 5:
        			// mode 5 - get Iterm back for all the flight;
        			if	( (cx_navstate_now != NAVSTATE_OFF) &&  (throttleStatusRcCommand != THROTTLE_LOW) ) {
        				cx_use_Iterm_reset = 0;
        			}
        			if( (ics_navstate_use_new_poshold()) &&  (ics_get_yaw_Iterm_reset()) ) {
    					// if new_poshold says that we want to reset yaw Iterm
    					pidResetErrorAccumulatorAxis(FD_YAW);
    				}
        			break;

        	}
            //if (throttleStatus == THROTTLE_LOW) {
                // if (isAirmodeActive() && !failsafeIsActive() && ARMING_FLAG(ARMED)) {
                    // rollPitchStatus_e rollPitchStatus = calculateRollPitchCenterStatus();

                    // // ANTI_WINDUP at centred stick with MOTOR_STOP is needed on MRs and not needed on FWs
                    // if ((rollPitchStatus == CENTERED) || (feature(FEATURE_MOTOR_STOP) && !STATE(FIXED_WING))) {
                        // ENABLE_STATE(ANTI_WINDUP);
                    // }
                    // else {
                        // DISABLE_STATE(ANTI_WINDUP);
                    // }
                // }
                // else {

            	//cur_pos.need_az = (int16_t)(millis()/1000);
            	// so this code is called in infinite loop while in auto mode


            if (cx_use_Iterm_reset) {
                    DISABLE_STATE(ANTI_WINDUP);
                    pidResetErrorAccumulators();		// seems like all PID I components are reset from here in auto flight....
    //            }
            }
            //}
            else {
                DISABLE_STATE(ANTI_WINDUP);
            }
        }

        if (mixerConfig()->mixerMode == MIXER_FLYING_WING || mixerConfig()->mixerMode == MIXER_AIRPLANE || mixerConfig()->mixerMode == MIXER_CUSTOM_AIRPLANE) {
            DISABLE_FLIGHT_MODE(HEADFREE_MODE);
        }
    
    }

#if defined(AUTOTUNE_FIXED_WING) || defined(AUTOTUNE_MULTIROTOR)
    autotuneUpdateState();
#endif

#ifdef USE_TELEMETRY
    if (feature(FEATURE_TELEMETRY)) {
        if ((!telemetryConfig()->telemetry_switch && ARMING_FLAG(ARMED)) ||
                (telemetryConfig()->telemetry_switch && IS_RC_MODE_ACTIVE(BOXTELEMETRY))) {

            releaseSharedTelemetryPorts();
        } else {
            // the telemetry state must be checked immediately so that shared serial ports are released.
            telemetryCheckState();
            mspSerialAllocatePorts();
        }
    }
#endif

}

void filterRc(bool isRXDataNew)
{
    static int16_t lastCommand[4] = { 0, 0, 0, 0 };
    static int16_t deltaRC[4] = { 0, 0, 0, 0 };
    static int16_t factor, rcInterpolationFactor;
    static biquadFilter_t filteredCycleTimeState;
    static bool filterInitialised;

    // Calculate average cycle time (1Hz LPF on cycle time)
    if (!filterInitialised) {
        biquadFilterInitLPF(&filteredCycleTimeState, 1, getPidUpdateRate());
        filterInitialised = true;
    }

    const timeDelta_t filteredCycleTime = biquadFilterApply(&filteredCycleTimeState, (float) cycleTime);
    rcInterpolationFactor = rxGetRefreshRate() / filteredCycleTime + 1;

    if (isRXDataNew) {
        for (int channel=0; channel < 4; channel++) {
            deltaRC[channel] = rcCommand[channel] -  (lastCommand[channel] - deltaRC[channel] * factor / rcInterpolationFactor);
            lastCommand[channel] = rcCommand[channel];
        }

        factor = rcInterpolationFactor - 1;
    } else {
        factor--;
    }

    // Interpolate steps of rcCommand
    if (factor > 0) {
        for (int channel=0; channel < 4; channel++) {
            rcCommand[channel] = lastCommand[channel] - deltaRC[channel] * factor/rcInterpolationFactor;
         }
    } else {
        factor = 0;
    }
}

// Function for loop trigger
void taskGyro(timeUs_t currentTimeUs) {
    // getTaskDeltaTime() returns delta time frozen at the moment of entering the scheduler. currentTime is frozen at the very same point.
    // To make busy-waiting timeout work we need to account for time spent within busy-waiting loop
    const timeDelta_t currentDeltaTime = getTaskDeltaTime(TASK_SELF);
    timeUs_t gyroUpdateUs = currentTimeUs;

    if (gyroConfig()->gyroSync) {
        while (true) {
            gyroUpdateUs = micros();
            if (gyroSyncCheckUpdate() || ((currentDeltaTime + cmpTimeUs(gyroUpdateUs, currentTimeUs)) >= (getGyroUpdateRate() + GYRO_WATCHDOG_DELAY))) {
                break;
            }
        }
    }

    /* Update actual hardware readings */
    gyroUpdate(currentDeltaTime + (timeDelta_t)(gyroUpdateUs - currentTimeUs));

#ifdef USE_OPTICAL_FLOW
    if (sensors(SENSOR_OPFLOW)) {
        opflowGyroUpdateCallback((timeUs_t)currentDeltaTime + (gyroUpdateUs - currentTimeUs));
    }
#endif
}

static float calculateThrottleTiltCompensationFactor(uint8_t throttleTiltCompensationStrength)
{
    if (throttleTiltCompensationStrength) {
        float tiltCompFactor = 1.0f / constrainf(calculateCosTiltAngle(), 0.6f, 1.0f);  // max tilt about 50 deg
        return 1.0f + (tiltCompFactor - 1.0f) * (throttleTiltCompensationStrength / 100.f);
    } else {
        return 1.0f;
    }
}

int16_t mid_motor;
int16_t m[4];

void cx_smooth_motors(){
	mid_motor = (motor[0] >> 2) + (motor[1] >> 2) + (motor[2] >> 2) + (motor[3] >> 2);
	for(int i = 0; i < 4; i++ ){
		m[i] = mid_motor;
	}
}

void taskMainPidLoop(timeUs_t currentTimeUs)
{
    cycleTime = getTaskDeltaTime(TASK_SELF);
    dT = (float)cycleTime * 0.000001f;

#ifdef USE_ASYNC_GYRO_PROCESSING
    if (getAsyncMode() == ASYNC_MODE_NONE) {
        taskGyro(currentTimeUs);
    }

    if (getAsyncMode() != ASYNC_MODE_ALL && sensors(SENSOR_ACC)) {
        imuUpdateAccelerometer();
        imuUpdateAttitude(currentTimeUs);
    }
#else
    /* Update gyroscope */
    taskGyro(currentTimeUs);
    imuUpdateAccelerometer();
    imuUpdateAttitude(currentTimeUs);
#endif


    annexCode();

    if (rxConfig()->rcSmoothing) {
        filterRc(isRXDataNew);
    }

    if( systemConfig()->cx_use_new_nav > 0 ){
        updatePositionEstimator();
        CP_INAV_TO_LOCAL();// recalc to our local here
        ics_nav_controller(currentTimeUs);
        isRXDataNew = false;
    } else {
    
#if defined(USE_NAV)
        if (isRXDataNew) {
            updateWaypointsAndNavigationMode();
        }
#endif
        

        isRXDataNew = false;

#if defined(USE_NAV)
        updatePositionEstimator();
        applyWaypointNavigationAndAltitudeHold();
#endif
    }

    // If we're armed, at minimum throttle, and we do arming via the
    // sticks, do not process yaw input from the rx.  We do this so the
    // motors do not spin up while we are trying to arm or disarm.
    // Allow yaw control for tricopters if the user wants the servo to move even when unarmed.
    if (isUsingSticksForArming() && rcData[THROTTLE] <= rxConfig()->mincheck
#ifndef USE_QUAD_MIXER_ONLY
#ifdef USE_SERVOS
            && !((mixerConfig()->mixerMode == MIXER_TRI || mixerConfig()->mixerMode == MIXER_CUSTOM_TRI) && servoConfig()->tri_unarmed_servo)
#endif
            && mixerConfig()->mixerMode != MIXER_AIRPLANE
            && mixerConfig()->mixerMode != MIXER_FLYING_WING
            && mixerConfig()->mixerMode != MIXER_CUSTOM_AIRPLANE
#endif
    ) {
        rcCommand[YAW] = 0;
    }

    // // Apply throttle tilt compensation
    // if (!STATE(FIXED_WING)) {
        // int16_t thrTiltCompStrength = 0;

        // if (navigationRequiresThrottleTiltCompensation()) {
            // thrTiltCompStrength = 100;
        // }
        // else if (systemConfig()->throttle_tilt_compensation_strength && (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE))) {
            // thrTiltCompStrength = systemConfig()->throttle_tilt_compensation_strength;
        // }

        // if (thrTiltCompStrength) {
            // rcCommand[THROTTLE] = constrain(motorConfig()->minthrottle
                                            // + (rcCommand[THROTTLE] - motorConfig()->minthrottle) * calculateThrottleTiltCompensationFactor(thrTiltCompStrength),
                                            // motorConfig()->minthrottle,
                                            // motorConfig()->maxthrottle);
        // }
    // }
    // else {
        // // FIXME: throttle pitch comp for FW
    // }

    // Update PID coefficients
    updatePIDCoefficients();

    // Calculate stabilisation
    pidController();

#ifdef HIL
    if (hilActive) {
        hilUpdateControlState();
        motorControlEnable = false;
    }
#endif

    mixTable();

#ifdef USE_SERVOS
    payload_update_state(currentTimeUs);

    if (isMixerUsingServos()) {
        servoMixer(dT);
        processServoAutotrim();
    }

    // Servo tilt is not part of servo mixer, but uses servos
    if (feature(FEATURE_SERVO_TILT)) {
        processServoTilt();
    }

    //Servos should be filtered or written only when mixer is using servos or special feaures are enabled
    if (isServoOutputEnabled()) {
        writeServos();
    }
#endif

    uint8_t pm = 0;


    for( int i = 0; i < 4;i++){
    	m[i] = motor[i];
    }

    if (motorControlEnable) {
        //FIXAR
    	if( ram_reduce_mc_in_plane > 0 ){
        	if( IS_RC_MODE_ACTIVE(BOXSWITCHTOPLANE) ){
        		pm = 1;
				cx_smooth_motors();
			} else if( (navstate == NAVSTATE_ROUTE) || (navstate == NAVSTATE_PRELINE) ){
				if( !IS_RC_MODE_ACTIVE(BOXMANUAL) || (ram_allow_plane_in_manual > 0)) {
					if( cur_pos.is_use_airspeed ){
						if( cur_pos.airspd > ram_plane_switch_speed ){
							pm = 1;
							cx_smooth_motors();
						}
					}
				}
			}
    	}

    	writeMotors_x4(m[0], m[1], m[2], m[3]);
    }
    cur_pos.is_plane_mode = pm;

#ifdef USE_SDCARD
    afatfs_poll();
#endif

#ifdef USE_BLACKBOX
    if (!cliMode && feature(FEATURE_BLACKBOX)) {
        blackboxUpdate(micros());
    }
#endif

#ifdef USE_TELEMETRY_XLOGGER
    handleXLoggerTelemetry(micros());
#endif
}

bool taskUpdateRxCheck(timeUs_t currentTimeUs, timeDelta_t currentDeltaTime)
{
    UNUSED(currentDeltaTime);

    return rxUpdateCheck(currentTimeUs, currentDeltaTime);
}

void taskUpdateRxMain(timeUs_t currentTimeUs)
{
    processRx(currentTimeUs);
    isRXDataNew = true;
}
