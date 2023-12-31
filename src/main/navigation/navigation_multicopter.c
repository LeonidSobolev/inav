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

#include "platform.h"

#if defined(USE_NAV)

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
#include "fc/rc_controls.h"
#include "fc/rc_curves.h"
#include "fc/runtime_config.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/mixer.h"

#include "navigation/navigation.h"
#include "navigation/navigation_private.h"


/*-----------------------------------------------------------
 * Altitude controller for multicopter aircraft
 *-----------------------------------------------------------*/
static int16_t rcCommandAdjustedThrottle;
static int16_t altHoldThrottleRCZero = 1500;
static pt1Filter_t altholdThrottleFilterState;
static bool prepareForTakeoffOnReset = false;

// Position to velocity controller for Z axis
static void updateAltitudeVelocityController_MC(timeDelta_t deltaMicros)
{
    const float altitudeError = posControl.desiredState.pos.z - posControl.actualState.pos.z;
    float targetVel = altitudeError * posControl.pids.pos[Z].param.kP;

    // hard limit desired target velocity to max_climb_rate
    if (posControl.flags.isAdjustingAltitude) {
        targetVel = constrainf(targetVel, -navConfig()->general.max_manual_climb_rate, navConfig()->general.max_manual_climb_rate);
    }
    else {
        targetVel = constrainf(targetVel, -navConfig()->general.max_auto_climb_rate, navConfig()->general.max_auto_climb_rate);
    }

    // limit max vertical acceleration to 1/5G (~200 cm/s/s) if we are increasing velocity.
    // if we are decelerating - don't limit (allow better recovery from falling)
    if (ABS(targetVel) > ABS(posControl.desiredState.vel.z)) {
        const float maxVelDifference = US2S(deltaMicros) * (GRAVITY_CMSS / 5.0f);
        posControl.desiredState.vel.z = constrainf(targetVel, posControl.desiredState.vel.z - maxVelDifference, posControl.desiredState.vel.z + maxVelDifference);
    }
    else {
        posControl.desiredState.vel.z = targetVel;
    }

#if defined(NAV_BLACKBOX)
    navDesiredVelocity[Z] = constrain(lrintf(posControl.desiredState.vel.z), -32678, 32767);
#endif
}

static void updateAltitudeThrottleController_MC(timeDelta_t deltaMicros)
{
    // Calculate min and max throttle boundaries (to compensate for integral windup)
    const int16_t thrAdjustmentMin = (int16_t)motorConfig()->minthrottle - (int16_t)navConfig()->mc.hover_throttle;
    const int16_t thrAdjustmentMax = (int16_t)motorConfig()->maxthrottle - (int16_t)navConfig()->mc.hover_throttle;

    posControl.rcAdjustment[THROTTLE] = navPidApply2(&posControl.pids.vel[Z], posControl.desiredState.vel.z, posControl.actualState.vel.z, US2S(deltaMicros), thrAdjustmentMin, thrAdjustmentMax, 0);

    posControl.rcAdjustment[THROTTLE] = pt1FilterApply4(&altholdThrottleFilterState, posControl.rcAdjustment[THROTTLE], NAV_THROTTLE_CUTOFF_FREQENCY_HZ, US2S(deltaMicros));
    posControl.rcAdjustment[THROTTLE] = constrain(posControl.rcAdjustment[THROTTLE], thrAdjustmentMin, thrAdjustmentMax);
}

bool adjustMulticopterAltitudeFromRCInput(void)
{
    const int16_t rcThrottleAdjustment = applyDeadband(rcCommand[THROTTLE] - altHoldThrottleRCZero, rcControlsConfig()->alt_hold_deadband);
    if (rcThrottleAdjustment) {
        // set velocity proportional to stick movement
        float rcClimbRate;

        // Make sure we can satisfy max_manual_climb_rate in both up and down directions
        if (rcThrottleAdjustment > 0) {
            // Scaling from altHoldThrottleRCZero to maxthrottle
            rcClimbRate = rcThrottleAdjustment * navConfig()->general.max_manual_climb_rate / (motorConfig()->maxthrottle - altHoldThrottleRCZero - rcControlsConfig()->alt_hold_deadband);
        }
        else {
            // Scaling from minthrottle to altHoldThrottleRCZero
            rcClimbRate = rcThrottleAdjustment * navConfig()->general.max_manual_climb_rate / (altHoldThrottleRCZero - motorConfig()->minthrottle - rcControlsConfig()->alt_hold_deadband);
        }

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

void setupMulticopterAltitudeController(void)
{
    const throttleStatus_e throttleStatus = calculateThrottleStatus();

    if (navConfig()->general.flags.use_thr_mid_for_althold) {
        altHoldThrottleRCZero = rcLookupThrottleMid();
    }
    else {
        // If throttle status is THROTTLE_LOW - use Thr Mid anyway
        if (throttleStatus == THROTTLE_LOW) {
            altHoldThrottleRCZero = rcLookupThrottleMid();
        }
        else {
            altHoldThrottleRCZero = rcCommand[THROTTLE];
        }
    }

    // Make sure we are able to satisfy the deadband
    altHoldThrottleRCZero = constrain(altHoldThrottleRCZero,
                                      motorConfig()->minthrottle + rcControlsConfig()->alt_hold_deadband + 10,
                                      motorConfig()->maxthrottle - rcControlsConfig()->alt_hold_deadband - 10);

    /* Force AH controller to initialize althold integral for pending takeoff on reset */
    if (throttleStatus == THROTTLE_LOW) {
        prepareForTakeoffOnReset = true;
    }
}

void resetMulticopterAltitudeController(void)
{
    navPidReset(&posControl.pids.vel[Z]);
    navPidReset(&posControl.pids.surface);
    posControl.rcAdjustment[THROTTLE] = 0;

    if (prepareForTakeoffOnReset) {
        /* If we are preparing for takeoff - start with lowset possible climb rate, adjust alt target and make sure throttle doesn't jump */
        posControl.desiredState.vel.z = -navConfig()->general.max_manual_climb_rate;
        posControl.desiredState.pos.z = posControl.actualState.pos.z - (navConfig()->general.max_manual_climb_rate / posControl.pids.pos[Z].param.kP);
        posControl.pids.vel[Z].integrator = -500.0f;
        pt1FilterReset(&altholdThrottleFilterState, -500.0f);
        prepareForTakeoffOnReset = false;
    }
    else {
        posControl.desiredState.vel.z = posControl.actualState.vel.z;   // Gradually transition from current climb
        pt1FilterReset(&altholdThrottleFilterState, 0.0f);
    }
}

static void applyMulticopterAltitudeController(timeUs_t currentTimeUs)
{
    static timeUs_t previousTimePositionUpdate;         // Occurs @ altitude sensor update rate (max MAX_ALTITUDE_UPDATE_RATE_HZ)
    static timeUs_t previousTimeUpdate;                 // Occurs @ looptime rate

    const timeDelta_t deltaMicros = currentTimeUs - previousTimeUpdate;
    previousTimeUpdate = currentTimeUs;

    // If last position update was too long in the past - ignore it (likely restarting altitude controller)
    if (deltaMicros > HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
        previousTimeUpdate = currentTimeUs;
        previousTimePositionUpdate = currentTimeUs;
        resetMulticopterAltitudeController();
        return;
    }

    // If we have an update on vertical position data - update velocity and accel targets
    if (posControl.flags.verticalPositionDataNew) {
        const timeDelta_t deltaMicrosPositionUpdate = currentTimeUs - previousTimePositionUpdate;
        previousTimePositionUpdate = currentTimeUs;

        // Check if last correction was too log ago - ignore this update
        if (deltaMicrosPositionUpdate < HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
            updateAltitudeVelocityController_MC(deltaMicrosPositionUpdate);
            updateAltitudeThrottleController_MC(deltaMicrosPositionUpdate);
        }
        else {
            // due to some glitch position update has not occurred in time, reset altitude controller
            resetMulticopterAltitudeController();
        }

        // Indicate that information is no longer usable
        posControl.flags.verticalPositionDataConsumed = 1;
    }

    // Update throttle controller
    rcCommand[THROTTLE] = constrain((int16_t)navConfig()->mc.hover_throttle + posControl.rcAdjustment[THROTTLE], motorConfig()->minthrottle, motorConfig()->maxthrottle);

    // Save processed throttle for future use
    rcCommandAdjustedThrottle = rcCommand[THROTTLE];
}

/*-----------------------------------------------------------
 * Adjusts desired heading from pilot's input
 *-----------------------------------------------------------*/
bool adjustMulticopterHeadingFromRCInput(void)
{
    if (ABS(rcCommand[YAW]) > rcControlsConfig()->pos_hold_deadband) {
        // Can only allow pilot to set the new heading if doing PH, during RTH copter will target itself to home
        posControl.desiredState.yaw = posControl.actualState.yaw;

        return true;
    }
    else {
        return false;
    }
}

/*-----------------------------------------------------------
 * XY-position controller for multicopter aircraft
 *-----------------------------------------------------------*/
static pt1Filter_t mcPosControllerAccFilterStateX, mcPosControllerAccFilterStateY;
static float lastAccelTargetX = 0.0f, lastAccelTargetY = 0.0f;

void resetMulticopterPositionController(void)
{
    for (int axis = 0; axis < 2; axis++) {
        navPidReset(&posControl.pids.vel[axis]);
        posControl.rcAdjustment[axis] = 0;
        pt1FilterReset(&mcPosControllerAccFilterStateX, 0.0f);
        pt1FilterReset(&mcPosControllerAccFilterStateY, 0.0f);
        lastAccelTargetX = 0.0f;
        lastAccelTargetY = 0.0f;
    }
}

bool adjustMulticopterPositionFromRCInput(void)
{
    const int16_t rcPitchAdjustment = applyDeadband(rcCommand[PITCH], rcControlsConfig()->pos_hold_deadband);
    const int16_t rcRollAdjustment = applyDeadband(rcCommand[ROLL], rcControlsConfig()->pos_hold_deadband);

    if (rcPitchAdjustment || rcRollAdjustment) {
        // If mode is GPS_CRUISE, move target position, otherwise POS controller will passthru the RC input to ANGLE PID
        if (navConfig()->general.flags.user_control_mode == NAV_GPS_CRUISE) {
            const float rcVelX = rcPitchAdjustment * navConfig()->general.max_manual_speed / (500 - rcControlsConfig()->pos_hold_deadband);
            const float rcVelY = rcRollAdjustment * navConfig()->general.max_manual_speed / (500 - rcControlsConfig()->pos_hold_deadband);

            // Rotate these velocities from body frame to to earth frame
            const float neuVelX = rcVelX * posControl.actualState.cosYaw - rcVelY * posControl.actualState.sinYaw;
            const float neuVelY = rcVelX * posControl.actualState.sinYaw + rcVelY * posControl.actualState.cosYaw;

            // Calculate new position target, so Pos-to-Vel P-controller would yield desired velocity
            posControl.desiredState.pos.x = posControl.actualState.pos.x + (neuVelX / posControl.pids.pos[X].param.kP);
            posControl.desiredState.pos.y = posControl.actualState.pos.y + (neuVelY / posControl.pids.pos[Y].param.kP);
        }

        return true;
    }
    else {
        // Adjusting finished - reset desired position to stay exactly where pilot released the stick
        if (posControl.flags.isAdjustingPosition) {
            fpVector3_t stopPosition;
            calculateMulticopterInitialHoldPosition(&stopPosition);
            setDesiredPosition(&stopPosition, 0, NAV_POS_UPDATE_XY);
        }

        return false;
    }
 }

static float getVelocityHeadingAttenuationFactor(void)
{
    if( systemConfig()->cx_use_plane_mode == 1){
        if( systemConfig()->cx_use_yaw_controller == 0){
            return 1.0f;
        }
        if(posControl.navState !=  NAV_STATE_WAYPOINT_RTH_LAND){
            return 1.0f;
        }
    }
        
    // In WP mode scale velocity if heading is different from bearing
    if (navGetCurrentStateFlags() & NAV_AUTO_WP) {
        const int32_t headingError = constrain(wrap_18000(posControl.desiredState.yaw - posControl.actualState.yaw), -9000, 9000);
        const float velScaling = cos_approx(CENTIDEGREES_TO_RADIANS(headingError));

        return constrainf(velScaling * velScaling, 0.05f, 1.0f);
    } else {
        return 1.0f;
    }
}

static float getVelocityExpoAttenuationFactor(float velTotal, float velMax)
{
    // Calculate factor of how velocity with applied expo is different from unchanged velocity
    const float velScale = constrainf(velTotal / velMax, 0.01f, 1.0f);

    // navConfig()->max_speed * ((velScale * velScale * velScale) * posControl.posResponseExpo + velScale * (1 - posControl.posResponseExpo)) / velTotal;
    // ((velScale * velScale * velScale) * posControl.posResponseExpo + velScale * (1 - posControl.posResponseExpo)) / velScale
    // ((velScale * velScale) * posControl.posResponseExpo + (1 - posControl.posResponseExpo));
    return 1.0f - posControl.posResponseExpo * (1.0f - (velScale * velScale));  // x^3 expo factor
}

static void updatePositionVelocityController_MC(void)
{
    const float posErrorX = posControl.desiredState.pos.x - posControl.actualState.pos.x;
    const float posErrorY = posControl.desiredState.pos.y - posControl.actualState.pos.y;

    // Calculate target velocity
    float newVelX = posErrorX * posControl.pids.pos[X].param.kP;
    float newVelY = posErrorY * posControl.pids.pos[Y].param.kP;

    // Get max speed from generic NAV (waypoint specific), don't allow to move slower than 0.5 m/s
    const float maxSpeed = getActiveWaypointSpeed();

    // Scale velocity to respect max_speed
    float newVelTotal = sqrtf(sq(newVelX) + sq(newVelY));
    if (newVelTotal > maxSpeed) {
        newVelX = maxSpeed * (newVelX / newVelTotal);
        newVelY = maxSpeed * (newVelY / newVelTotal);
        newVelTotal = maxSpeed;
    }

    // Apply expo & attenuation if heading in wrong direction - turn first, accelerate later (effective only in WP mode)
    const float velHeadFactor = getVelocityHeadingAttenuationFactor();
    const float velExpoFactor = getVelocityExpoAttenuationFactor(newVelTotal, maxSpeed);
    posControl.desiredState.vel.x = newVelX * velHeadFactor * velExpoFactor;
    posControl.desiredState.vel.y = newVelY * velHeadFactor * velExpoFactor;

#if defined(NAV_BLACKBOX)
    navDesiredVelocity[X] = constrain(lrintf(posControl.desiredState.vel.x), -32678, 32767);
    navDesiredVelocity[Y] = constrain(lrintf(posControl.desiredState.vel.y), -32678, 32767);
#endif
}

static void updatePositionAccelController_MC(timeDelta_t deltaMicros, float maxAccelLimit)
{

    // Calculate velocity error
    const float velErrorX = posControl.desiredState.vel.x - posControl.actualState.vel.x;
    const float velErrorY = posControl.desiredState.vel.y - posControl.actualState.vel.y;

    // Calculate XY-acceleration limit according to velocity error limit
    float accelLimitX, accelLimitY;
    const float velErrorMagnitude = sqrtf(sq(velErrorX) + sq(velErrorY));
    if (velErrorMagnitude > 0.1f) {
        accelLimitX = maxAccelLimit / velErrorMagnitude * fabsf(velErrorX);
        accelLimitY = maxAccelLimit / velErrorMagnitude * fabsf(velErrorY);
    }
    else {
        accelLimitX = maxAccelLimit / 1.414213f;
        accelLimitY = accelLimitX;
    }

    // Apply additional jerk limiting of 1700 cm/s^3 (~100 deg/s), almost any copter should be able to achieve this rate
    // This will assure that we wont't saturate out LEVEL and RATE PID controller
    const float maxAccelChange = US2S(deltaMicros) * 1700.0f;
    const float accelLimitXMin = constrainf(lastAccelTargetX - maxAccelChange, -accelLimitX, +accelLimitX);
    const float accelLimitXMax = constrainf(lastAccelTargetX + maxAccelChange, -accelLimitX, +accelLimitX);
    const float accelLimitYMin = constrainf(lastAccelTargetY - maxAccelChange, -accelLimitY, +accelLimitY);
    const float accelLimitYMax = constrainf(lastAccelTargetY + maxAccelChange, -accelLimitY, +accelLimitY);

    // TODO: Verify if we need jerk limiting after all

    // Apply PID with output limiting and I-term anti-windup
    // Pre-calculated accelLimit and the logic of navPidApply2 function guarantee that our newAccel won't exceed maxAccelLimit
    // Thus we don't need to do anything else with calculated acceleration
    const float newAccelX = navPidApply2(&posControl.pids.vel[X], posControl.desiredState.vel.x, posControl.actualState.vel.x, US2S(deltaMicros), accelLimitXMin, accelLimitXMax, 0);
    const float newAccelY = navPidApply2(&posControl.pids.vel[Y], posControl.desiredState.vel.y, posControl.actualState.vel.y, US2S(deltaMicros), accelLimitYMin, accelLimitYMax, 0);

    // Save last acceleration target
    lastAccelTargetX = newAccelX;
    lastAccelTargetY = newAccelY;

    // Apply LPF to jerk limited acceleration target
    const float accelN = pt1FilterApply4(&mcPosControllerAccFilterStateX, newAccelX, NAV_ACCEL_CUTOFF_FREQUENCY_HZ, US2S(deltaMicros));
    const float accelE = pt1FilterApply4(&mcPosControllerAccFilterStateY, newAccelY, NAV_ACCEL_CUTOFF_FREQUENCY_HZ, US2S(deltaMicros));

    // Rotate acceleration target into forward-right frame (aircraft)
    const float accelForward = accelN * posControl.actualState.cosYaw + accelE * posControl.actualState.sinYaw;
    const float accelRight = -accelN * posControl.actualState.sinYaw + accelE * posControl.actualState.cosYaw;

    // Calculate banking angles
    const float desiredPitch = atan2_approx(accelForward, GRAVITY_CMSS);
    const float desiredRoll = atan2_approx(accelRight * cos_approx(desiredPitch), GRAVITY_CMSS);

    const int16_t maxBankAngle = DEGREES_TO_DECIDEGREES(navConfig()->mc.max_bank_angle);
    posControl.rcAdjustment[ROLL] = constrain(RADIANS_TO_DECIDEGREES(desiredRoll), -maxBankAngle, maxBankAngle);
    posControl.rcAdjustment[PITCH] = constrain(RADIANS_TO_DECIDEGREES(desiredPitch), -maxBankAngle, maxBankAngle);
}

void applyMulticopterPositionController(timeUs_t currentTimeUs)
{
    static timeUs_t previousTimePositionUpdate;         // Occurs @ GPS update rate
    static timeUs_t previousTimeUpdate;                 // Occurs @ looptime rate

    const timeDelta_t deltaMicros = currentTimeUs - previousTimeUpdate;
    previousTimeUpdate = currentTimeUs;
    bool bypassPositionController;

    // We should passthrough rcCommand is adjusting position in GPS_ATTI mode
    bypassPositionController = (navConfig()->general.flags.user_control_mode == NAV_GPS_ATTI) && posControl.flags.isAdjustingPosition;

    // If last call to controller was too long in the past - ignore it (likely restarting position controller)
    if (deltaMicros > HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
        previousTimeUpdate = currentTimeUs;
        previousTimePositionUpdate = currentTimeUs;
        resetMulticopterPositionController();
        return;
    }

    // Apply controller only if position source is valid. In absence of valid pos sensor (GPS loss), we'd stick in forced ANGLE mode
    // and pilots input would be passed thru to PID controller
    if ((posControl.flags.estPosStatue >= EST_USABLE)) {
        // If we have new position - update velocity and acceleration controllers
        if (posControl.flags.horizontalPositionDataNew) {
            const timeDelta_t deltaMicrosPositionUpdate = currentTimeUs - previousTimePositionUpdate;
            previousTimePositionUpdate = currentTimeUs;

            if (!bypassPositionController) {
                // Update position controller
                if (deltaMicrosPositionUpdate < HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
                    updatePositionVelocityController_MC();
                    updatePositionAccelController_MC(deltaMicrosPositionUpdate, NAV_ACCELERATION_XY_MAX);
                }
                else {
                    resetMulticopterPositionController();
                }
            }

            // Indicate that information is no longer usable
            posControl.flags.horizontalPositionDataConsumed = 1;
        }
    }
    else {
        /* No position data, disable automatic adjustment, rcCommand passthrough */
        posControl.rcAdjustment[PITCH] = 0;
        posControl.rcAdjustment[ROLL] = 0;
        bypassPositionController = true;
    }

    if (!bypassPositionController) {
        rcCommand[PITCH] = pidAngleToRcCommand(posControl.rcAdjustment[PITCH], pidProfile()->max_angle_inclination[FD_PITCH]);
        rcCommand[ROLL] = pidAngleToRcCommand(posControl.rcAdjustment[ROLL], pidProfile()->max_angle_inclination[FD_ROLL]);
    }
}

/*-----------------------------------------------------------
 * Multicopter land detector
 *-----------------------------------------------------------*/
static timeUs_t landingTimer;
static timeUs_t landingDetectorStartedAt;
static int32_t landingThrSum;
static int32_t landingThrSamples;

void resetMulticopterLandingDetector(void)
{
    // FIXME: This function is called some time before isMulticopterLandingDetected is first called
    landingTimer = micros();
    landingDetectorStartedAt = 0; // ugly hack for now

    landingThrSum = 0;
    landingThrSamples = 0;
}

bool isMulticopterLandingDetected(void)
{
    const timeUs_t currentTimeUs = micros();

    // FIXME: Remove delay between resetMulticopterLandingDetector and first run of this function so this code isn't needed.
    if (landingDetectorStartedAt == 0) {
        landingDetectorStartedAt = currentTimeUs;
    }

    // Average climb rate should be low enough
    bool verticalMovement = fabsf(posControl.actualState.vel.z) > 25.0f;

    // check if we are moving horizontally
    bool horizontalMovement = posControl.actualState.velXY > 100.0f;

    // We have likely landed if throttle is 40 units below average descend throttle
    // We use rcCommandAdjustedThrottle to keep track of NAV corrected throttle (isLandingDetected is executed
    // from processRx() and rcCommand at that moment holds rc input, not adjusted values from NAV core)
    // Wait for 1 second so throttle has stabilized.
    bool isAtMinimalThrust = false;
    if (currentTimeUs - landingDetectorStartedAt > 1000 * 1000) {
        landingThrSamples += 1;
        landingThrSum += rcCommandAdjustedThrottle;
        isAtMinimalThrust = rcCommandAdjustedThrottle < (landingThrSum / landingThrSamples - 40);
    }

    bool possibleLandingDetected = isAtMinimalThrust && !verticalMovement && !horizontalMovement;

    DEBUG_SET(DEBUG_NAV_LANDING_DETECTOR, 0, isAtMinimalThrust * 100 + !verticalMovement * 10 + !horizontalMovement * 1);
    DEBUG_SET(DEBUG_NAV_LANDING_DETECTOR, 1, (landingThrSamples == 0) ? 0 : (rcCommandAdjustedThrottle - (landingThrSum / landingThrSamples)));
    DEBUG_SET(DEBUG_NAV_LANDING_DETECTOR, 2, (currentTimeUs - landingTimer) / 1000);

    // If we have surface sensor (for example sonar) - use it to detect touchdown
    if ((posControl.flags.estSurfaceStatus == EST_TRUSTED) && (posControl.actualState.surfaceMin >= 0)) {
        // TODO: Come up with a clever way to let sonar increase detection performance, not just add extra safety.
        // TODO: Out of range sonar may give reading that looks like we landed, find a way to check if sonar is healthy.
        // surfaceMin is our ground reference. If we are less than 5cm above the ground - we are likely landed
        possibleLandingDetected = possibleLandingDetected && (posControl.actualState.surface <= (posControl.actualState.surfaceMin + 5.0f));
    }

    if (!possibleLandingDetected) {
        landingTimer = currentTimeUs;
        return false;
    }
    else {
        return ((currentTimeUs - landingTimer) > (navConfig()->mc.auto_disarm_delay * 1000)) ? true : false;
    }
}

/*-----------------------------------------------------------
 * Multicopter launch
 *-----------------------------------------------------------*/
void applyMulticopterLaunchController(timeUs_t currentTimeUs)
{
    /* Attempt to stabilise */
    rcCommand[ROLL] = 0;
    rcCommand[PITCH] = 0;
    rcCommand[YAW] = 0;
    
    // if( systemConfig()->cx_autostart_poshold_enable == 1 ){
        // applyMulticopterPositionController(currentTimeUs);
    // } else{
        rcCommand[ROLL] = 0;
        //rcCommand[PITCH] = baroGetLatestAltitude() * systemConfig()->cx_autostart_pitch_scale / 100.0;
        rcCommand[PITCH] = 0;
        rcCommand[YAW] = 0;     
    //}
    
    
    rcCommand[THROTTLE] = constrain((int16_t)systemConfig()->cx_launch_start_throttle , 800, 2200); 
}

/*-----------------------------------------------------------
 * Multicopter land
 *-----------------------------------------------------------*/
uint16_t land_prev_alt = 0;
timeUs_t land_prev_time_speed = 0;
timeUs_t land_prev_time = 0;

float has = 0;
float land_p;
float land_i;
float land_d;
float land_pid;
float land_dt;
 
timeDelta_t  previousTimePositionUpdate;
 
void applyMulticopterLandController(timeUs_t currentTimeUs)
{
    land_dt = (currentTimeUs - land_prev_time) / 1000000.0;
    //if first time or lag, no use integral part in PID
    if( land_dt > 1.0 ) land_dt = 0;
    /* Attempt to stabilise */
    rcCommand[ROLL] = 0;
    rcCommand[PITCH] = 0;
    rcCommand[YAW] = 0;
    
    if( systemConfig()->cx_landing_poshold_enable == 1 ){
        //applyMulticopterPositionController(currentTimeUs);
        const timeDelta_t deltaMicrosPositionUpdate = currentTimeUs - previousTimePositionUpdate;
        previousTimePositionUpdate = currentTimeUs;
        updatePositionVelocityController_MC();
        updatePositionAccelController_MC(deltaMicrosPositionUpdate, NAV_ACCELERATION_XY_MAX);
  
        rcCommand[PITCH] = pidAngleToRcCommand(posControl.rcAdjustment[PITCH], pidProfile()->max_angle_inclination[FD_PITCH]);
        rcCommand[ROLL] = pidAngleToRcCommand(posControl.rcAdjustment[ROLL], pidProfile()->max_angle_inclination[FD_ROLL]);       
    } else{

    }
    
    if( systemConfig()->cx_landing_version == 0 ){
        updateClimbRateToAltitudeController(-systemConfig()->cx_land_speed, ROC_TO_ALT_NORMAL);
    } else if( systemConfig()->cx_landing_version == 1 ){
        updateClimbRateToAltitudeController(0, ROC_TO_ALT_NORMAL);
    } else if( systemConfig()->cx_landing_version == 2 ){
        //rcCommand[THROTTLE] = constrain((int16_t)systemConfig()->cx_landing_throttle , 800, 2200);    
        rcCommand[THROTTLE] = constrain(0 , 800, 2200);    
    // } else if( systemConfig()->cx_landing_version == 3 ){
        // if( currentTimeUs > land_prev_time_speed + 500000 ){
            // has = (getEstimatedActualPosition(Z) - land_prev_alt ) / ( (currentTimeUs - land_prev_time_speed) / 1000000.0);
            // land_prev_alt = getEstimatedActualPosition(Z);
            // land_prev_time_speed = currentTimeUs;
        // }
        // float error = systemConfig()->cx_land_speed - has;
        // land_p = error * systemConfig()->cx_land_p / 10;
        // land_i += error * systemConfig()->cx_land_i * (land_dt) / 10;
        // if( land_i > 1000 ) land_i = 1000;
        // if( land_i < -1000 ) land_i = -1000;
        // land_d = 0;//(-error) * systemConfig()->cx_land_p / 10;
        // land_pid = land_p + land_i + land_d;
        
        // rcCommand[THROTTLE] = constrain((int16_t)systemConfig()->cx_landing_throttle + (int16_t)land_pid , 800, 2200);   
    
    // } else if( systemConfig()->cx_landing_version == 4 ){
        // float error = systemConfig()->cx_land_speed - baroGetLatestAltSpeed();
        // land_p = error * systemConfig()->cx_land_p / 10;
        // land_i += error * systemConfig()->cx_land_i * (land_dt) / 10;
        // if( land_i > 1000 ) land_i = 1000;
        // if( land_i < -1000 ) land_i = -1000;
        // land_d = 0;//(-error) * systemConfig()->cx_land_p / 10;
        // land_pid = land_p + land_i + land_d;        
    
        // rcCommand[THROTTLE] = constrain((int16_t)systemConfig()->cx_landing_throttle , 800, 2200);           

    }
    land_prev_time = currentTimeUs;
    //rcCommand[THROTTLE] = constrain((int16_t)systemConfig()->cx_launch_start_throttle , 800, 2200); 
}

/*-----------------------------------------------------------
 * Multicopter emergency landing
 *-----------------------------------------------------------*/
void applyMulticopterEmergencyLandingController(timeUs_t currentTimeUs)
{
    static timeUs_t previousTimeUpdate;
    static timeUs_t previousTimePositionUpdate;
    const timeDelta_t deltaMicros = currentTimeUs - previousTimeUpdate;
    previousTimeUpdate = currentTimeUs;

    /* Attempt to stabilise */
    rcCommand[ROLL] = 0;
    rcCommand[PITCH] = 0;
    rcCommand[YAW] = 0;

    if ((posControl.flags.estAltStatus >= EST_USABLE)) {
        // If last position update was too long in the past - ignore it (likely restarting altitude controller)
        if (deltaMicros > HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
            previousTimeUpdate = currentTimeUs;
            previousTimePositionUpdate = currentTimeUs;
            resetMulticopterAltitudeController();
            return;
        }

        if (posControl.flags.verticalPositionDataNew) {
            const timeDelta_t deltaMicrosPositionUpdate = currentTimeUs - previousTimePositionUpdate;
            previousTimePositionUpdate = currentTimeUs;

            // Check if last correction was too log ago - ignore this update
            if (deltaMicrosPositionUpdate < HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
                updateClimbRateToAltitudeController(-1.0f * navConfig()->general.emerg_descent_rate, ROC_TO_ALT_NORMAL);
                updateAltitudeVelocityController_MC(deltaMicrosPositionUpdate);
                updateAltitudeThrottleController_MC(deltaMicrosPositionUpdate);
            }
            else {
                // due to some glitch position update has not occurred in time, reset altitude controller
                resetMulticopterAltitudeController();
            }

            // Indicate that information is no longer usable
            posControl.flags.verticalPositionDataConsumed = 1;
        }

        // Update throttle controller
        rcCommand[THROTTLE] = constrain((int16_t)navConfig()->mc.hover_throttle + posControl.rcAdjustment[THROTTLE], motorConfig()->minthrottle, motorConfig()->maxthrottle);
    }
    else {
        /* Sensors has gone haywire, attempt to land regardless */
        if (failsafeConfig()) {
            rcCommand[THROTTLE] = failsafeConfig()->failsafe_throttle;
        }
        else {
            rcCommand[THROTTLE] = motorConfig()->minthrottle;
        }
    }
}

/*-----------------------------------------------------------
 * Calculate loiter target based on current position and velocity
 *-----------------------------------------------------------*/
void calculateMulticopterInitialHoldPosition(fpVector3_t * pos)
{
    const float stoppingDistanceX = posControl.actualState.vel.x * posControl.posDecelerationTime;
    const float stoppingDistanceY = posControl.actualState.vel.y * posControl.posDecelerationTime;

    pos->x = posControl.actualState.pos.x + stoppingDistanceX;
    pos->y = posControl.actualState.pos.y + stoppingDistanceY;
}

void resetMulticopterHeadingController(void)
{
    updateHeadingHoldTarget(CENTIDEGREES_TO_DEGREES(posControl.actualState.yaw));
}

static void applyMulticopterHeadingController(void)
{
    updateHeadingHoldTarget(CENTIDEGREES_TO_DEGREES(posControl.desiredState.yaw));
}

void applyMulticopterNavigationController(navigationFSMStateFlags_t navStateFlags, timeUs_t currentTimeUs)
{
    if (navStateFlags & NAV_CTL_LAUNCH) {
        applyMulticopterLaunchController(currentTimeUs);
    } else if (navStateFlags & NAV_CTL_LAND) {
        applyMulticopterLandController(currentTimeUs);
    } else if (navStateFlags & NAV_CTL_EMERG) {
        applyMulticopterEmergencyLandingController(currentTimeUs);
    } else {
        if (navStateFlags & NAV_CTL_ALT)
            applyMulticopterAltitudeController(currentTimeUs);

        if (navStateFlags & NAV_CTL_POS)
            applyMulticopterPositionController(currentTimeUs);

        if (navStateFlags & NAV_CTL_YAW)
            applyMulticopterHeadingController();
    }
}
#endif  // NAV
