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

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h>

#include "platform.h"

#if defined(USE_NAV)

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "drivers/time.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/boardalignment.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/mixer.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "navigation/navigation.h"
#include "navigation/navigation_private.h"

// If we are going slower than NAV_FW_MIN_VEL_SPEED_BOOST - boost throttle to fight against the wind
#define NAV_FW_THROTTLE_SPEED_BOOST_GAIN        1.5f
#define NAV_FW_MIN_VEL_SPEED_BOOST              700.0f      // 7 m/s

// If this is enabled navigation won't be applied if velocity is below 3 m/s
//#define NAV_FW_LIMIT_MIN_FLY_VELOCITY

static bool isPitchAdjustmentValid = false;
static bool isRollAdjustmentValid = false;
static float throttleSpeedAdjustment = 0;

uint8_t cx_nav_mode = 0;
uint32_t cx_nav_d2e = 0, cx_nav_d2s = 0, cx_nav_ds2e = 0;

uint8_t is_on_polygon = 0;

uint8_t cxIsOnPolygon(){
    return is_on_polygon;
}

/*-----------------------------------------------------------
 * Altitude controller
 *-----------------------------------------------------------*/
void setupFixedWingAltitudeController(void)
{
    // TODO
}

void resetFixedWingAltitudeController(void)
{
    navPidReset(&posControl.pids.fw_alt);
    posControl.rcAdjustment[PITCH] = 0;
    isPitchAdjustmentValid = false;
    throttleSpeedAdjustment = 0;
}

bool adjustFixedWingAltitudeFromRCInput(void)
{
    int16_t rcAdjustment = applyDeadband(rcCommand[PITCH], rcControlsConfig()->alt_hold_deadband);

    if (rcAdjustment) {
        // set velocity proportional to stick movement
        float rcClimbRate = -rcAdjustment * navConfig()->general.max_manual_climb_rate / (500.0f - rcControlsConfig()->alt_hold_deadband);
        updateClimbRateToAltitudeController(rcClimbRate, ROC_TO_ALT_NORMAL);
        return true;
    }
    else {
        // Adjusting finished - reset desired position to stay exactly where pilot released the stick
        if (posControl.flags.isAdjustingAltitude) {
            updateClimbRateToAltitudeController(0, ROC_TO_ALT_RESET);
        }
        return false;
    }
}

// Position to velocity controller for Z axis
static void updateAltitudeVelocityAndPitchController_FW(timeDelta_t deltaMicros)
{
    static pt1Filter_t velzFilterState;

    // On a fixed wing we might not have a reliable climb rate source (if no BARO available), so we can't apply PID controller to
    // velocity error. We use PID controller on altitude error and calculate desired pitch angle

    // Update energies
    const float demSPE = (posControl.desiredState.pos.z / 100.0f) * GRAVITY_MSS;
    const float demSKE = 0.0f;

    const float estSPE = (posControl.actualState.pos.z / 100.0f) * GRAVITY_MSS;
    const float estSKE = 0.0f;

    // speedWeight controls balance between potential and kinetic energy used for pitch controller
    //  speedWeight = 1.0 : pitch will only control airspeed and won't control altitude
    //  speedWeight = 0.5 : pitch will be used to control both airspeed and altitude
    //  speedWeight = 0.0 : pitch will only control altitude
    const float speedWeight = 0.0f; // no speed sensing for now

    const float demSEB = demSPE * (1.0f - speedWeight) - demSKE * speedWeight;
    const float estSEB = estSPE * (1.0f - speedWeight) - estSKE * speedWeight;

    // SEB to pitch angle gain to account for airspeed (with respect to specified reference (tuning) speed
    const float pitchGainInv = 1.0f / 1.0f;

    // Here we use negative values for dive for better clarity
    const float maxClimbDeciDeg = DEGREES_TO_DECIDEGREES(navConfig()->fw.max_climb_angle);
    const float minDiveDeciDeg = -DEGREES_TO_DECIDEGREES(navConfig()->fw.max_dive_angle);

    // PID controller to translate energy balance error [J] into pitch angle [decideg]
    float targetPitchAngle = navPidApply3(&posControl.pids.fw_alt, demSEB, estSEB, US2S(deltaMicros), minDiveDeciDeg, maxClimbDeciDeg, 0, pitchGainInv);
    targetPitchAngle = pt1FilterApply4(&velzFilterState, targetPitchAngle, NAV_FW_PITCH_CUTOFF_FREQENCY_HZ, US2S(deltaMicros));

    // Reconstrain pitch angle ( >0 - climb, <0 - dive)
    targetPitchAngle = constrainf(targetPitchAngle, minDiveDeciDeg, maxClimbDeciDeg);
    posControl.rcAdjustment[PITCH] = targetPitchAngle - systemConfig()->cx_planemode_pitch_angle_decidegi;
}

void applyFixedWingAltitudeAndThrottleController(timeUs_t currentTimeUs)
{
    static timeUs_t previousTimePositionUpdate;         // Occurs @ altitude sensor update rate (max MAX_ALTITUDE_UPDATE_RATE_HZ)
    static timeUs_t previousTimeUpdate;                 // Occurs @ looptime rate

    const timeDelta_t deltaMicros = currentTimeUs - previousTimeUpdate;
    previousTimeUpdate = currentTimeUs;

    // If last time Z-controller was called is too far in the past - ignore it (likely restarting altitude controller)
    if (deltaMicros > HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
        previousTimeUpdate = currentTimeUs;
        previousTimePositionUpdate = currentTimeUs;
        resetFixedWingAltitudeController();
        return;
    }

    if ((posControl.flags.estAltStatus >= EST_USABLE)) {
        if (posControl.flags.verticalPositionDataNew) {
            const timeDelta_t deltaMicrosPositionUpdate = currentTimeUs - previousTimePositionUpdate;
            previousTimePositionUpdate = currentTimeUs;

            // Check if last correction was too log ago - ignore this update
            if (deltaMicrosPositionUpdate < HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
                updateAltitudeVelocityAndPitchController_FW(deltaMicrosPositionUpdate);
            }
            else {
                // due to some glitch position update has not occurred in time, reset altitude controller
                resetFixedWingAltitudeController();
            }

            // Indicate that information is no longer usable
            posControl.flags.verticalPositionDataConsumed = 1;
        }

        isPitchAdjustmentValid = true;
    }
    else {
        // No valid altitude sensor data, don't adjust pitch automatically, rcCommand[PITCH] is passed through to PID controller
        isPitchAdjustmentValid = false;
    }
}

/*-----------------------------------------------------------
 * Adjusts desired heading from pilot's input
 *-----------------------------------------------------------*/
bool adjustFixedWingHeadingFromRCInput(void)
{
    return false;
}

/*-----------------------------------------------------------
 * XY-position controller for multicopter aircraft
 *-----------------------------------------------------------*/
static fpVector3_t virtualDesiredPosition;
static pt1Filter_t fwPosControllerCorrectionFilterState;

void resetFixedWingPositionController(void)
{
    virtualDesiredPosition.x = 0;
    virtualDesiredPosition.y = 0;
    virtualDesiredPosition.z = 0;

    navPidReset(&posControl.pids.fw_nav);
    posControl.rcAdjustment[ROLL] = 0;
    isRollAdjustmentValid = false;

    pt1FilterReset(&fwPosControllerCorrectionFilterState, 0.0f);
}


float cx_dist_to_line( float x0, float y0, float x1, float y1, float x2, float y2 ){
    float t1 = abs( (y2 - y1) * x0 - (x2 - x1) * y0 + x2*y1 - y2*x1 );
    float t2 = sqrtf( (y2-y1) * (y2-y1) + (x2 - x1) * (x2 - x1) ); 
    return t1 / t2;
}

fpVector3_t cx_get_line_target_pos( fpVector3_t line_start, fpVector3_t line_end, fpVector3_t my_pos ){
    float slx = line_start.x;    
    float sly = line_start.y;    
    float elx = line_end.x;    
    float ely = line_end.y;  

    float myx = my_pos.x;
    float myy = my_pos.y;

    float dist_to_line = cx_dist_to_line( myx, myy, slx, sly, elx, ely ) / 100.0;
    
    //calculate square of distances of triangle [me, line_start, line_end]
    float d2s = (myx - slx) * (myx - slx) + (myy - sly) * (myy - sly);
    float d2e = (myx - elx) * (myx - elx) + (myy - ely) * (myy - ely);
    float ds2e = (elx - slx) * (elx - slx) + (ely - sly) * (ely - sly);
    
    cx_nav_d2e = d2e / 100.0 / 100.0;
    cx_nav_d2s = d2s / 100.0 / 100.0;
    cx_nav_ds2e = ds2e / 100.0 / 100.0;
    //cx_nav_ds2e = (int)(dist_to_line);    
    
    
    //check if we in route for PL
    if( dist_to_line * 100 > (systemConfig()->cx_payload_off_distance_cm)){
        is_on_polygon = 0;
    } else {
        is_on_polygon = 1;
    }
    
    
    //if we so far, go to line_start
    //in this description (line = [point, point-1]) this condition does not occur in normal flight
    if( dist_to_line > 20 * (systemConfig()->cx_linefollow_active_distance)){
        cx_nav_mode = 1;
        
        return line_start;
    }

    
    if( d2s >= d2e + ds2e ){
        //wp reached
        cx_nav_mode = 2;      
        return line_end;     
    }
    if( d2e >= d2s + ds2e ){
        //prepare to line
        cx_nav_mode = 3;      
        return line_start;  
    }
    cx_nav_mode = 4;      
    fpVector3_t virtual_line_target;
    
    float k = (systemConfig()->cx_linefollow_active_distance * 100 + sqrtf(d2s));
    k = k / (k + sqrtf(d2e) - systemConfig()->cx_linefollow_active_distance * 100);
    virtual_line_target.x = ( line_start.x * (1.0-k) + line_end.x * k );
    virtual_line_target.y = ( line_start.y * (1.0-k) + line_end.y * k );
    virtual_line_target.z = ( line_start.z * (1.0-k) + line_end.z * k );
    
    return virtual_line_target;
}

int cx_calc_line_virtual_position( float tracking_period ){
    int cur_wp_index = posControl.activeWaypointIndex;
    
    fpVector3_t target = posControl.desiredState.pos;
    
    if(cur_wp_index > 0){
    //if(cur_wp_index < (posControl.waypointCount - 1)){
        fpVector3_t start_line_loc;
        fpVector3_t end_line_loc;
        mapWaypointToLocalPosition(&start_line_loc, &posControl.waypointList[posControl.activeWaypointIndex-1]);
        mapWaypointToLocalPosition(&end_line_loc, &posControl.waypointList[posControl.activeWaypointIndex]);
        target = cx_get_line_target_pos(
            start_line_loc,
            end_line_loc,
            posControl.actualState.pos
        );
    }  

    float posErrorX = target.x - posControl.actualState.pos.x;
    float posErrorY = target.y - posControl.actualState.pos.y;
    float distanceToActualTarget = sqrtf(sq(posErrorX) + sq(posErrorY));
    float trackingDistance = tracking_period * MAX(posControl.actualState.velXY, 100.0f);

    virtualDesiredPosition.x = posControl.actualState.pos.x + posErrorX * (trackingDistance / distanceToActualTarget);
    virtualDesiredPosition.y = posControl.actualState.pos.y + posErrorY * (trackingDistance / distanceToActualTarget);
    
    //payload_update_state();
    
    return 1;
}

static void calculateVirtualPositionTarget_FW(float trackingPeriod)
{
    float posErrorX = posControl.desiredState.pos.x - posControl.actualState.pos.x;
    float posErrorY = posControl.desiredState.pos.y - posControl.actualState.pos.y;

    float distanceToActualTarget = sqrtf(sq(posErrorX) + sq(posErrorY));

    // Limit minimum forward velocity to 1 m/s
    float trackingDistance = trackingPeriod * MAX(posControl.actualState.velXY, 100.0f);

    // // If angular visibility of a waypoint is less than 30deg, don't calculate circular loiter, go straight to the target
    // #define TAN_15DEG    0.26795f
    // bool needToCalculateCircularLoiter = isApproachingLastWaypoint()
                                            // && (distanceToActualTarget <= (navConfig()->fw.loiter_radius / TAN_15DEG))
                                            // && (distanceToActualTarget > 50.0f);

    // // Calculate virtual position for straight movement
    // if (needToCalculateCircularLoiter) {
        // // We are closing in on a waypoint, calculate circular loiter
        // float loiterAngle = atan2_approx(-posErrorY, -posErrorX) + DEGREES_TO_RADIANS(45.0f);

        // float loiterTargetX = posControl.desiredState.pos.x + navConfig()->fw.loiter_radius * cos_approx(loiterAngle);
        // float loiterTargetY = posControl.desiredState.pos.y + navConfig()->fw.loiter_radius * sin_approx(loiterAngle);

        // // We have temporary loiter target. Recalculate distance and position error
        // posErrorX = loiterTargetX - posControl.actualState.pos.x;
        // posErrorY = loiterTargetY - posControl.actualState.pos.y;
        // distanceToActualTarget = sqrtf(sq(posErrorX) + sq(posErrorY));
    // }

    // Calculate virtual waypoint 
    // if( systemConfig()->cx_linefollow_enabled > 0){
        // cx_calc_line_virtual_position(trackingPeriod);
    // } else {  
    virtualDesiredPosition.x = posControl.actualState.pos.x + posErrorX * (trackingDistance / distanceToActualTarget);
    virtualDesiredPosition.y = posControl.actualState.pos.y + posErrorY * (trackingDistance / distanceToActualTarget);
    // }
    // Shift position according to pilot's ROLL input (up to max_manual_speed velocity)
    // if (posControl.flags.isAdjustingPosition) {
        // int16_t rcRollAdjustment = applyDeadband(rcCommand[ROLL], rcControlsConfig()->pos_hold_deadband);

        // if (rcRollAdjustment) {
            // float rcShiftY = rcRollAdjustment * navConfig()->general.max_manual_speed / 500.0f * trackingPeriod;

            // // Rotate this target shift from body frame to to earth frame and apply to position target
            // virtualDesiredPosition.x += -rcShiftY * posControl.actualState.sinYaw;
            // virtualDesiredPosition.y +=  rcShiftY * posControl.actualState.cosYaw;

            // posControl.flags.isAdjustingPosition = true;
        // }
    // }
}

bool adjustFixedWingPositionFromRCInput(void)
{
    int16_t rcRollAdjustment = applyDeadband(rcCommand[ROLL], rcControlsConfig()->pos_hold_deadband);
    return (rcRollAdjustment);
}

static void updatePositionHeadingController_FW(timeUs_t currentTimeUs, timeDelta_t deltaMicros)
{
    static timeUs_t previousTimeMonitoringUpdate;
    static float previousHeadingError;
    static bool errorIsDecreasing;
    static bool forceTurnDirection = false;

    // We have virtual position target, calculate heading error
    int32_t virtualTargetBearing = calculateBearingToDestination(&virtualDesiredPosition);

    // Calculate NAV heading error
    int32_t headingError = wrap_18000(virtualTargetBearing - posControl.actualState.yaw);

    // Forced turn direction
    // If heading error is close to 180 deg we initiate forced turn and only disable it when heading error goes below 90 deg
    if (ABS(headingError) > 17000) {
        forceTurnDirection = true;
    }
    else if (ABS(headingError) < 9000 && forceTurnDirection) {
        forceTurnDirection = false;
    }

    // If forced turn direction flag is enabled we fix the sign of the direction
    if (forceTurnDirection) {
        headingError = ABS(headingError);
    }

    // Slow error monitoring (2Hz rate)
    if ((currentTimeUs - previousTimeMonitoringUpdate) >= HZ2US(NAV_FW_CONTROL_MONITORING_RATE)) {
        // Check if error is decreasing over time
        errorIsDecreasing = (ABS(previousHeadingError) > ABS(headingError));

        // Save values for next iteration
        previousHeadingError = headingError;
        previousTimeMonitoringUpdate = currentTimeUs;
    }

    // Only allow PID integrator to shrink if error is decreasing over time
    const pidControllerFlags_e pidFlags = PID_DTERM_FROM_ERROR | (errorIsDecreasing ? PID_SHRINK_INTEGRATOR : 0);

    // Input error in (deg*100), output pitch angle (deg*100)
    float rollAdjustment = navPidApply2(&posControl.pids.fw_nav, posControl.actualState.yaw + headingError, posControl.actualState.yaw, US2S(deltaMicros),
                                       -DEGREES_TO_CENTIDEGREES(navConfig()->fw.max_bank_angle),
                                        DEGREES_TO_CENTIDEGREES(navConfig()->fw.max_bank_angle),
                                        pidFlags);

    // Apply low-pass filter to prevent rapid correction
    rollAdjustment = pt1FilterApply4(&fwPosControllerCorrectionFilterState, rollAdjustment, NAV_FW_ROLL_CUTOFF_FREQUENCY_HZ, US2S(deltaMicros));

    // Convert rollAdjustment to decidegrees (rcAdjustment holds decidegrees)
    posControl.rcAdjustment[ROLL] = CENTIDEGREES_TO_DECIDEGREES(rollAdjustment);
}

void applyFixedWingPositionController(timeUs_t currentTimeUs)
{
    static timeUs_t previousTimePositionUpdate;         // Occurs @ GPS update rate
    static timeUs_t previousTimeUpdate;                 // Occurs @ looptime rate

    const timeDelta_t deltaMicros = currentTimeUs - previousTimeUpdate;
    previousTimeUpdate = currentTimeUs;

    // If last position update was too long in the past - ignore it (likely restarting altitude controller)
    if (deltaMicros > HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
        previousTimeUpdate = currentTimeUs;
        previousTimePositionUpdate = currentTimeUs;
        resetFixedWingPositionController();
        return;
    }

    // Apply controller only if position source is valid. In absence of valid pos sensor (GPS loss), we'd stick in forced ANGLE mode
    if ((posControl.flags.estPosStatue >= EST_USABLE)) {
        // If we have new position - update velocity and acceleration controllers
        if (posControl.flags.horizontalPositionDataNew) {
            const timeDelta_t deltaMicrosPositionUpdate = currentTimeUs - previousTimePositionUpdate;
            previousTimePositionUpdate = currentTimeUs;

            if (deltaMicrosPositionUpdate < HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
                // Calculate virtual position target at a distance of forwardVelocity * HZ2S(POSITION_TARGET_UPDATE_RATE_HZ)
                // Account for pilot's roll input (move position target left/right at max of max_manual_speed)
                // POSITION_TARGET_UPDATE_RATE_HZ should be chosen keeping in mind that position target shouldn't be reached until next pos update occurs
                // FIXME: verify the above
                calculateVirtualPositionTarget_FW(HZ2S(MIN_POSITION_UPDATE_RATE_HZ) * 2);

                updatePositionHeadingController_FW(currentTimeUs, deltaMicrosPositionUpdate);
            }
            else {
                resetFixedWingPositionController();
            }

            // Indicate that information is no longer usable
            posControl.flags.horizontalPositionDataConsumed = 1;
        }

        isRollAdjustmentValid = true;
    }
    else {
        // No valid pos sensor data, don't adjust pitch automatically, rcCommand[ROLL] is passed through to PID controller
        isRollAdjustmentValid = false;
    }
}

int16_t applyFixedWingMinSpeedController(timeUs_t currentTimeUs)
{
    static timeUs_t previousTimePositionUpdate;         // Occurs @ GPS update rate
    static timeUs_t previousTimeUpdate;                 // Occurs @ looptime rate

    const timeDelta_t deltaMicros = currentTimeUs - previousTimeUpdate;
    previousTimeUpdate = currentTimeUs;

    // If last position update was too long in the past - ignore it (likely restarting altitude controller)
    if (deltaMicros > HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
        previousTimeUpdate = currentTimeUs;
        previousTimePositionUpdate = currentTimeUs;
        throttleSpeedAdjustment = 0;
        return 0;
    }

    // Apply controller only if position source is valid
    if ((posControl.flags.estPosStatue >= EST_USABLE)) {
        // If we have new position - update velocity and acceleration controllers
        if (posControl.flags.horizontalPositionDataNew) {
            const timeDelta_t deltaMicrosPositionUpdate = currentTimeUs - previousTimePositionUpdate;
            previousTimePositionUpdate = currentTimeUs;

            if (deltaMicrosPositionUpdate < HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
                float velThrottleBoost = (NAV_FW_MIN_VEL_SPEED_BOOST - posControl.actualState.velXY) * NAV_FW_THROTTLE_SPEED_BOOST_GAIN * US2S(deltaMicrosPositionUpdate);

                // If we are in the deadband of 50cm/s - don't update speed boost
                if (ABS(posControl.actualState.velXY - NAV_FW_MIN_VEL_SPEED_BOOST) > 50) {
                    throttleSpeedAdjustment += velThrottleBoost;
                }

                throttleSpeedAdjustment = constrainf(throttleSpeedAdjustment, 0.0f, 500.0f);
            }
            else {
                throttleSpeedAdjustment = 0;
            }

            // Indicate that information is no longer usable
            posControl.flags.horizontalPositionDataConsumed = 1;
        }
    }
    else {
        // No valid pos sensor data, we can't calculate speed
        throttleSpeedAdjustment = 0;
    }

    return throttleSpeedAdjustment;
}

void applyFixedWingPitchRollThrottleController(navigationFSMStateFlags_t navStateFlags, timeUs_t currentTimeUs)
{
    int16_t pitchCorrection = 0;        // >0 climb, <0 dive
    int16_t rollCorrection = 0;         // >0 right, <0 left
    int16_t throttleCorrection = 0;     // raw throttle
    
    uint16_t cruise_throttle_with_reduction = navConfig()->fw.cruise_throttle - speed_brake_gain * systemConfig()->cx_land_brake_throttle_cruise_reduction;

    int16_t minThrottleCorrection = navConfig()->fw.min_throttle - cruise_throttle_with_reduction;
    int16_t maxThrottleCorrection = navConfig()->fw.max_throttle - cruise_throttle_with_reduction;

    // Mix Pitch/Roll/Throttle
    if (isRollAdjustmentValid && (navStateFlags & NAV_CTL_POS)) {
        rollCorrection += posControl.rcAdjustment[ROLL];
    }

    if (isPitchAdjustmentValid && (navStateFlags & NAV_CTL_ALT)) {
        pitchCorrection += posControl.rcAdjustment[PITCH];
        throttleCorrection += DECIDEGREES_TO_DEGREES(pitchCorrection) * navConfig()->fw.pitch_to_throttle;

#ifdef NAV_FIXED_WING_LANDING
        if (navStateFlags & NAV_CTL_LAND) {
            /*
             * During LAND we do not allow to raise THROTTLE when nose is up
             * to reduce speed
             */
            throttleCorrection = constrain(throttleCorrection, minThrottleCorrection, 0);
        } else {
#endif
            throttleCorrection = constrain(throttleCorrection, minThrottleCorrection, maxThrottleCorrection);
#ifdef NAV_FIXED_WING_LANDING
        }
#endif
    }

    // // Speed controller - only apply in POS mode when NOT NAV_CTL_LAND
    if ((navStateFlags & NAV_CTL_POS) && !(navStateFlags & NAV_CTL_LAND)) {
        throttleCorrection += applyFixedWingMinSpeedController(currentTimeUs);
        throttleCorrection = constrain(throttleCorrection, minThrottleCorrection, maxThrottleCorrection);
    }

    // Limit and apply
    if (isPitchAdjustmentValid && (navStateFlags & NAV_CTL_ALT)) {
        // PITCH correction is measured according to altitude: <0 - dive/lose altitude, >0 - climb/gain altitude
        // PITCH angle is measured in opposite direction ( >0 - dive, <0 - climb)
        pitchCorrection = constrain(pitchCorrection, -DEGREES_TO_DECIDEGREES(navConfig()->fw.max_dive_angle), DEGREES_TO_DECIDEGREES(navConfig()->fw.max_climb_angle));
        rcCommand[PITCH] = -pidAngleToRcCommand(pitchCorrection, pidProfile()->max_angle_inclination[FD_PITCH]);
    }

    if (isRollAdjustmentValid && (navStateFlags & NAV_CTL_POS)) {
        rollCorrection = constrain(rollCorrection, -DEGREES_TO_DECIDEGREES(navConfig()->fw.max_bank_angle), DEGREES_TO_DECIDEGREES(navConfig()->fw.max_bank_angle));
        rcCommand[ROLL] = pidAngleToRcCommand(rollCorrection, pidProfile()->max_angle_inclination[FD_ROLL]);
    }


    
    if ((navStateFlags & NAV_CTL_ALT) || (navStateFlags & NAV_CTL_POS)) {
        uint16_t correctedThrottleValue = constrain( cruise_throttle_with_reduction + throttleCorrection, navConfig()->fw.min_throttle, navConfig()->fw.max_throttle);
        rcCommand[THROTTLE] = constrain(correctedThrottleValue, motorConfig()->minthrottle, motorConfig()->maxthrottle);
    }

#ifdef NAV_FIXED_WING_LANDING
    /*
     * Then altitude is below landing slowdown min. altitude, enable final approach procedure
     * TODO refactor conditions in this metod if logic is proven to be correct
     */
    if (navStateFlags & NAV_CTL_LAND) {
        if ( ((posControl.flags.estAltStatus >= EST_USABLE) && (posControl.actualState.pos.z <= navConfig()->general.land_slowdown_minalt)) ||
             ((posControl.flags.estSurfaceStatus == EST_TRUSTED) && (posControl.actualState.surface <= navConfig()->general.land_slowdown_minalt)) ) {
            /*
             * Set motor to min. throttle and stop it when MOTOR_STOP feature is enabled
             */
            rcCommand[THROTTLE] = motorConfig()->minthrottle;
            ENABLE_STATE(NAV_MOTOR_STOP_OR_IDLE);

            /*
             * Stabilize ROLL axis on 0 degrees banking regardless of loiter radius and position
             */
            rcCommand[ROLL] = 0;

            /*
             * Stabilize PITCH angle into shallow dive as specified by the
             * nav_fw_land_dive_angle setting (default value is 2 - defined
             * in navigation.c).
             * PITCH angle is measured: >0 - dive, <0 - climb)
             */
            rcCommand[PITCH] = pidAngleToRcCommand(DEGREES_TO_DECIDEGREES(navConfig()->fw.land_dive_angle), pidProfile()->max_angle_inclination[FD_PITCH]);
        }
    }
#endif
}

/*-----------------------------------------------------------
 * FixedWing land detector
 *-----------------------------------------------------------*/
static timeUs_t landingTimerUs;

void resetFixedWingLandingDetector(void)
{
    landingTimerUs = micros();
}

bool isFixedWingLandingDetected(void)
{
    timeUs_t currentTimeUs = micros();

    landingTimerUs = currentTimeUs;
    return false;
}

/*-----------------------------------------------------------
 * FixedWing emergency landing
 *-----------------------------------------------------------*/
void applyFixedWingEmergencyLandingController(void)
{
    // FIXME: Use altitude controller if available (similar to MC code)
    rcCommand[ROLL] = pidAngleToRcCommand(failsafeConfig()->failsafe_fw_roll_angle, pidProfile()->max_angle_inclination[FD_ROLL]);
    rcCommand[PITCH] = pidAngleToRcCommand(failsafeConfig()->failsafe_fw_pitch_angle, pidProfile()->max_angle_inclination[FD_PITCH]);
    rcCommand[YAW] = -pidRateToRcCommand(failsafeConfig()->failsafe_fw_yaw_rate, currentControlRateProfile->stabilized.rates[FD_YAW]);
    rcCommand[THROTTLE] = failsafeConfig()->failsafe_throttle;
}

/*-----------------------------------------------------------
 * Calculate loiter target based on current position and velocity
 *-----------------------------------------------------------*/
void calculateFixedWingInitialHoldPosition(fpVector3_t * pos)
{
    // TODO: stub, this should account for velocity and target loiter radius
    *pos = posControl.actualState.pos;
}

void resetFixedWingHeadingController(void)
{
    updateHeadingHoldTarget(CENTIDEGREES_TO_DEGREES(posControl.actualState.yaw));
}

void applyFixedWingNavigationController(navigationFSMStateFlags_t navStateFlags, timeUs_t currentTimeUs)
{
    // if (navStateFlags & NAV_CTL_LAUNCH) {
        // applyFixedWingLaunchController(currentTimeUs);
    // }
    // else if (navStateFlags & NAV_CTL_EMERG) {
        // applyFixedWingEmergencyLandingController();
    if (navStateFlags & NAV_CTL_LAUNCH) {
        applyMulticopterLaunchController(currentTimeUs);
    } else if (navStateFlags & NAV_CTL_LAND) {
        applyMulticopterLandController(currentTimeUs);
    } else if (navStateFlags & NAV_CTL_EMERG) {
        applyMulticopterEmergencyLandingController(currentTimeUs);
    } else {
#ifdef NAV_FW_LIMIT_MIN_FLY_VELOCITY
        // Don't apply anything if ground speed is too low (<3m/s)
        if (posControl.actualState.velXY > 300) {
            if (navStateFlags & NAV_CTL_ALT)
                applyFixedWingAltitudeAndThrottleController(currentTimeUs);

            if (navStateFlags & NAV_CTL_POS)
                applyFixedWingPositionController(currentTimeUs);
        }
        else {
            posControl.rcAdjustment[PITCH] = 0;
            posControl.rcAdjustment[ROLL] = 0;
        }
#else
        if (navStateFlags & NAV_CTL_ALT)
            applyFixedWingAltitudeAndThrottleController(currentTimeUs);

        if (navStateFlags & NAV_CTL_POS)
            applyFixedWingPositionController(currentTimeUs);
#endif

        //if (navStateFlags & NAV_CTL_YAW)
        if ((navStateFlags & NAV_CTL_ALT) || (navStateFlags & NAV_CTL_POS))
            applyFixedWingPitchRollThrottleController(navStateFlags, currentTimeUs);
    }
}

#endif  // NAV
