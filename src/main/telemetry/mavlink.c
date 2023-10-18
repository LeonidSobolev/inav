/*
 * telemetry_mavlink.c
 *
 * Author: Konstantin Sharlaimov
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

extern uint8_t is_i_line;

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/feature.h"

#include "drivers/serial.h"
#include "drivers/time.h"

#include "fc/config.h"
#include "fc/fc_core.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/servos.h"
#include "flight/payload.h"

#include "io/gimbal.h"
#include "io/gps.h"
#include "io/gps_private.h"
#include "io/ledstrip.h"
#include "io/serial.h"

#include "navigation/navigation.h"
#include "navigation/navigation_private.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/gyro.h"
#include "sensors/pitotmeter.h"
#include "sensors/sensors.h"

#include "telemetry/mavlink.h"
#include "telemetry/telemetry.h"
#include "telemetry/xlogger.h"

#include "navigation/icsnav.h"
#include "navigation/icsmission.h"
#include "navigation/apgeo.h"


extern float ram_altmc_pid_p;
extern float ram_altmc_pid_i;
extern float ram_altmc_pid_d;

uint16_t  debug_test  = 0;

uint8_t gtbuf[22+16+16];

uint32_t silence_timeout = 30;

// mavlink library uses unnames unions that's causes GCC to complain if -Wpedantic is used
// until this is resolved in mavlink library - ignore -Wpedantic for mavlink code
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "common/mavlink.h"
#pragma GCC diagnostic pop

#define TELEMETRY_MAVLINK_PORT_MODE     MODE_RXTX
#define TELEMETRY_MAVLINK_MAXRATE       50
#define TELEMETRY_MAVLINK_DELAY         ((1000 * 1000) / TELEMETRY_MAVLINK_MAXRATE)


void gtSendMessage(void);
void gt_send_message_long( uint8_t block_count );
void gt_send_message( void );
void gt_parse_message(void);
void gtSend_PointOK(uint16_t curwp, uint8_t is_ok);
void gtSend_Point_Ex(uint16_t curwp, uint8_t action, uint32_t lat, uint32_t lon, uint32_t h, uint8_t p1, uint8_t p2, uint16_t mlen, uint16_t pl_dist, uint8_t oid, uint8_t otype);
void gtSend_GoAck( uint8_t result, uint32_t error_code, uint32_t warning_code);
void gtSend_CheckReady(uint8_t is_ready, uint32_t err_code, uint32_t warning_code);
void gtSend_Rotation();
void gtSend_PowerInfo();
void gtSend_Setting(uint8_t sname[10], uint8_t answ, uint32_t val_mul_100);
void gtSend_DebugInfo(uint8_t u1, uint8_t u2, uint8_t u3, uint32_t x1, uint32_t x2, uint32_t x3);
uint8_t gtParseSetting( uint8_t sname[10], uint8_t cmd, int32_t *val );
void gtSend_PayloadInfo( uint16_t payload_number );
void gtSend_PayloadInfoEx( uint16_t payload_number );
void gtSend_DebugLFInfo();
void gtSend_PayloadCount();
void gtSend_APInfo();

extern uint8_t is_gps_off;
//extern float cur_alt_change;

uint8_t gt_clear(void);
void gtcrc(uint8_t c);
uint8_t gt_parse_char( char c );

uint32_t gt_input_char_count = 0;


static serialPort_t *mavlinkPort = NULL;
static serialPortConfig_t *portConfig;

static bool mavlinkTelemetryEnabled =  false;
static portSharing_e mavlinkPortSharing;

/* MAVLink datastream rates in Hz */
static const uint8_t mavRates[] = {
    [MAV_DATA_STREAM_EXTENDED_STATUS] = 2, //2Hz
    [MAV_DATA_STREAM_RC_CHANNELS] = 5, //5Hz
    [MAV_DATA_STREAM_POSITION] = 2, //2Hz
    [MAV_DATA_STREAM_EXTRA1] = 10, //10Hz
    [MAV_DATA_STREAM_EXTRA2] = 10 //2Hz
};

#define MAXSTREAMS (sizeof(mavRates) / sizeof(mavRates[0]))

static uint8_t mavTicks[MAXSTREAMS];
static mavlink_message_t mavSendMsg;
// static mavlink_message_t mavRecvMsg;

static uint8_t mavSystemId = 1;
static uint8_t mavComponentId = MAV_COMP_ID_SYSTEM_CONTROL;

// MANUAL, ACRO, ANGLE, HRZN, ALTHOLD, POSHOLD, RTH, WP, LAUNCH, FAILSAFE
static uint8_t inavToArduCopterMap[FLM_COUNT] = { 1,  1,  0,  0,  2, 16,  6,  3, 18,  0 };
static uint8_t inavToArduPlaneMap[FLM_COUNT]  = { 0,  4,  2,  2,  5,  1, 11, 10, 15,  2 };

static int mavlinkStreamTrigger(enum MAV_DATA_STREAM streamNum)
{
    uint8_t rate = (uint8_t) mavRates[streamNum];
    if (rate == 0) {
        return 0;
    }

    if (mavTicks[streamNum] == 0) {
        // we're triggering now, setup the next trigger point
        if (rate > TELEMETRY_MAVLINK_MAXRATE) {
            rate = TELEMETRY_MAVLINK_MAXRATE;
        }

        mavTicks[streamNum] = (TELEMETRY_MAVLINK_MAXRATE / rate);
        return 1;
    }

    // count down at TASK_RATE_HZ
    mavTicks[streamNum]--;
    return 0;
}

void freeMAVLinkTelemetryPort(void)
{
    closeSerialPort(mavlinkPort);
    mavlinkPort = NULL;
    mavlinkTelemetryEnabled = false;
}

void initMAVLinkTelemetry(void)
{
    portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_MAVLINK);
    mavlinkPortSharing = determinePortSharing(portConfig, FUNCTION_TELEMETRY_MAVLINK);
}

void configureMAVLinkTelemetryPort(void)
{
    if (!portConfig) {
        return;
    }

    baudRate_e baudRateIndex = portConfig->telemetry_baudrateIndex;
    if (baudRateIndex == BAUD_AUTO) {
        // default rate for minimOSD
        baudRateIndex = BAUD_57600;
    }

    mavlinkPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_MAVLINK, NULL, NULL, baudRates[baudRateIndex], TELEMETRY_MAVLINK_PORT_MODE, SERIAL_NOT_INVERTED);

    if (!mavlinkPort) {
        return;
    }

    mavlinkTelemetryEnabled = true;
}

void checkMAVLinkTelemetryState(void)
{
    bool newTelemetryEnabledValue = telemetryDetermineEnabledState(mavlinkPortSharing);

    if (newTelemetryEnabledValue == mavlinkTelemetryEnabled) {
        return;
    }

    if (newTelemetryEnabledValue)
        configureMAVLinkTelemetryPort();
    else
        freeMAVLinkTelemetryPort();
}

static void mavlinkSendMessage(void)
{
    uint8_t mavBuffer[MAVLINK_MAX_PACKET_LEN];
    int msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavSendMsg);

    for (int i = 0; i < msgLength; i++) {
        serialWrite(mavlinkPort, mavBuffer[i]);
    }
}

void mavlinkSendSystemStatus(void)
{
    uint32_t onboardControlAndSensors = 35843;

    /*
    onboard_control_sensors_present Bitmask
    fedcba9876543210
    1000110000000011    For all   = 35843
    0001000000000100    With Mag  = 4100
    0010000000001000    With Baro = 8200
    0100000000100000    With GPS  = 16416
    0000001111111111
    */

    if (sensors(SENSOR_MAG))  onboardControlAndSensors |=  4100;
    if (sensors(SENSOR_BARO)) onboardControlAndSensors |=  8200;
    if (sensors(SENSOR_GPS))  onboardControlAndSensors |= 16416;

    mavlink_msg_sys_status_pack(mavSystemId, mavComponentId, &mavSendMsg,
        // onboard_control_sensors_present Bitmask showing which onboard controllers and sensors are present.
        //Value of 0: not present. Value of 1: present. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure,
        // 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position,
        // 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization,
        // 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control
        onboardControlAndSensors,
        // onboard_control_sensors_enabled Bitmask showing which onboard controllers and sensors are enabled
        onboardControlAndSensors,
        // onboard_control_sensors_health Bitmask showing which onboard controllers and sensors are operational or have an error.
        onboardControlAndSensors & 1023,
        // load Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
        0,
        // voltage_battery Battery voltage, in millivolts (1 = 1 millivolt)
        feature(FEATURE_VBAT) ? getBatteryVoltage() * 10 : 0,
        // current_battery Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
        feature(FEATURE_CURRENT_METER) ? getAmperage() : -1,
        // battery_remaining Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
        feature(FEATURE_VBAT) ? calculateBatteryPercentage() : 100,
        // drop_rate_comm Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
        0,
        // errors_comm Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
        0,
        // errors_count1 Autopilot-specific errors
        0,
        // errors_count2 Autopilot-specific errors
        0,
        // errors_count3 Autopilot-specific errors
        0,
        // errors_count4 Autopilot-specific errors
        0);

    mavlinkSendMessage();
}

void mavlinkSendRCChannelsAndRSSI(void)
{
    mavlink_msg_rc_channels_raw_pack(mavSystemId, mavComponentId, &mavSendMsg,
        // time_boot_ms Timestamp (milliseconds since system boot)
        millis(),
        // port Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode more than 8 servos.
        0,
        // chan1_raw RC channel 1 value, in microseconds
        (rxRuntimeConfig.channelCount >= 1) ? rcData[0] : 0,
        // chan2_raw RC channel 2 value, in microseconds
        (rxRuntimeConfig.channelCount >= 2) ? rcData[1] : 0,
        // chan3_raw RC channel 3 value, in microseconds
        (rxRuntimeConfig.channelCount >= 3) ? rcData[2] : 0,
        // chan4_raw RC channel 4 value, in microseconds
        (rxRuntimeConfig.channelCount >= 4) ? rcData[3] : 0,
        // chan5_raw RC channel 5 value, in microseconds
        (rxRuntimeConfig.channelCount >= 5) ? rcData[4] : 0,
        // chan6_raw RC channel 6 value, in microseconds
        (rxRuntimeConfig.channelCount >= 6) ? rcData[5] : 0,
        // chan7_raw RC channel 7 value, in microseconds
        (rxRuntimeConfig.channelCount >= 7) ? rcData[6] : 0,
        // chan8_raw RC channel 8 value, in microseconds
        (rxRuntimeConfig.channelCount >= 8) ? rcData[7] : 0,
        // rssi Receive signal strength indicator, 0: 0%, 255: 100%
        scaleRange(getRSSI(), 0, 1023, 0, 255));

    mavlinkSendMessage();
}

#if defined(USE_GPS)
void mavlinkSendPosition(timeUs_t currentTimeUs)
{
    uint8_t gpsFixType = 0;

    if (!sensors(SENSOR_GPS))
        return;

    if (gpsSol.fixType == GPS_NO_FIX)
        gpsFixType = 1;
    else if (gpsSol.fixType == GPS_FIX_2D)
            gpsFixType = 2;
    else if (gpsSol.fixType == GPS_FIX_3D)
            gpsFixType = 3;

    mavlink_msg_gps_raw_int_pack(mavSystemId, mavComponentId, &mavSendMsg,
        // time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        currentTimeUs,
        // fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
        gpsFixType,
        // lat Latitude in 1E7 degrees
        gpsSol.llh.lat,
        // lon Longitude in 1E7 degrees
        gpsSol.llh.lon,
        // alt Altitude in 1E3 meters (millimeters) above MSL
        gpsSol.llh.alt * 10,
        // eph GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
        gpsSol.eph,
        // epv GPS VDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
        gpsSol.epv,
        // vel GPS ground speed (m/s * 100). If unknown, set to: 65535
        gpsSol.groundSpeed,
        // cog Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
        gpsSol.groundCourse * 10,
        // satellites_visible Number of satellites visible. If unknown, set to 255
        gpsSol.numSat);

    mavlinkSendMessage();

    // Global position
    mavlink_msg_global_position_int_pack(mavSystemId, mavComponentId, &mavSendMsg,
        // time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        currentTimeUs,
        // lat Latitude in 1E7 degrees
        gpsSol.llh.lat,
        // lon Longitude in 1E7 degrees
        gpsSol.llh.lon,
        // alt Altitude in 1E3 meters (millimeters) above MSL
        gpsSol.llh.alt * 10,
        // relative_alt Altitude above ground in meters, expressed as * 1000 (millimeters)
#if defined(USE_NAV)
        getEstimatedActualPosition(Z) * 10,
#else
        gpsSol.llh.alt * 10,
#endif
        // Ground X Speed (Latitude), expressed as m/s * 100
        0,
        // Ground Y Speed (Longitude), expressed as m/s * 100
        0,
        // Ground Z Speed (Altitude), expressed as m/s * 100
        0,
        // heading Current heading in degrees, in compass units (0..360, 0=north)
        DECIDEGREES_TO_DEGREES(attitude.values.yaw)
    );

    mavlinkSendMessage();

    mavlink_msg_gps_global_origin_pack(mavSystemId, mavComponentId, &mavSendMsg,
        // latitude Latitude (WGS84), expressed as * 1E7
        GPS_home.lat,
        // longitude Longitude (WGS84), expressed as * 1E7
        GPS_home.lon,
        // altitude Altitude(WGS84), expressed as * 1000
        GPS_home.alt * 10); // FIXME

    mavlinkSendMessage();
}
#endif

void mavlinkSendAttitude(void)
{
    mavlink_msg_attitude_pack(mavSystemId, mavComponentId, &mavSendMsg,
        // time_boot_ms Timestamp (milliseconds since system boot)
        millis(),
        // roll Roll angle (rad)
        DECIDEGREES_TO_RADIANS(attitude.values.roll),
        // pitch Pitch angle (rad)
        DECIDEGREES_TO_RADIANS(-attitude.values.pitch),
        // yaw Yaw angle (rad)
        DECIDEGREES_TO_RADIANS(attitude.values.yaw),
        // rollspeed Roll angular speed (rad/s)
        0,
        // pitchspeed Pitch angular speed (rad/s)
        0,
        // yawspeed Yaw angular speed (rad/s)
        0);

    mavlinkSendMessage();
}

void mavlinkSendHUDAndHeartbeat(void)
{
    float mavAltitude = 0;
    float mavGroundSpeed = 0;
    float mavAirSpeed = 0;
    float mavClimbRate = 0;

#if defined(USE_GPS)
    // use ground speed if source available
    if (sensors(SENSOR_GPS)) {
        mavGroundSpeed = gpsSol.groundSpeed / 100.0f;
    }
#endif

#if defined(USE_PITOT)
    if (sensors(SENSOR_PITOT)) {
        mavAirSpeed = pitot.airSpeed / 100.0f;
    }
#endif

    // select best source for altitude
#if defined(USE_NAV)
    mavAltitude = getEstimatedActualPosition(Z) / 100.0f;
    mavClimbRate = getEstimatedActualVelocity(Z) / 100.0f;
#elif defined(USE_GPS)
    if (sensors(SENSOR_GPS)) {
        // No surface or baro, just display altitude above MLS
        mavAltitude = gpsSol.llh.alt;
    }
#endif

    mavlink_msg_vfr_hud_pack(mavSystemId, mavComponentId, &mavSendMsg,
        // airspeed Current airspeed in m/s
        mavAirSpeed,
        // groundspeed Current ground speed in m/s
        mavGroundSpeed,
        // heading Current heading in degrees, in compass units (0..360, 0=north)
        DECIDEGREES_TO_DEGREES(attitude.values.yaw),
        // throttle Current throttle setting in integer percent, 0 to 100
        scaleRange(constrain(rcData[THROTTLE], PWM_RANGE_MIN, PWM_RANGE_MAX), PWM_RANGE_MIN, PWM_RANGE_MAX, 0, 100),
        // alt Current altitude (MSL), in meters, if we have surface or baro use them, otherwise use GPS (less accurate)
        mavAltitude,
        // climb Current climb rate in meters/second
        mavClimbRate);

    mavlinkSendMessage();


    uint8_t mavModes = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    if (ARMING_FLAG(ARMED))
        mavModes |= MAV_MODE_FLAG_SAFETY_ARMED;

    uint8_t mavSystemType;
    switch (mixerConfig()->mixerMode)
    {
        case MIXER_TRI:
            mavSystemType = MAV_TYPE_TRICOPTER;
            break;
        case MIXER_QUADP:
        case MIXER_QUADX:
        case MIXER_Y4:
        case MIXER_VTAIL4:
            mavSystemType = MAV_TYPE_QUADROTOR;
            break;
        case MIXER_Y6:
        case MIXER_HEX6:
        case MIXER_HEX6X:
            mavSystemType = MAV_TYPE_HEXAROTOR;
            break;
        case MIXER_OCTOX8:
        case MIXER_OCTOFLATP:
        case MIXER_OCTOFLATX:
            mavSystemType = MAV_TYPE_OCTOROTOR;
            break;
        case MIXER_FLYING_WING:
        case MIXER_AIRPLANE:
        case MIXER_CUSTOM_AIRPLANE:
            mavSystemType = MAV_TYPE_FIXED_WING;
            break;
        case MIXER_HELI_120_CCPM:
        case MIXER_HELI_90_DEG:
            mavSystemType = MAV_TYPE_HELICOPTER;
            break;
        default:
            mavSystemType = MAV_TYPE_GENERIC;
            break;
    }

    flightModeForTelemetry_e flm = getFlightModeForTelemetry();
    uint8_t mavCustomMode;

    if (STATE(FIXED_WING)) {
        mavCustomMode = inavToArduPlaneMap[flm];
    }
    else {
        mavCustomMode = inavToArduCopterMap[flm];
    }

    if (flm != FLM_MANUAL) {
        mavModes |= MAV_MODE_FLAG_STABILIZE_ENABLED;
    }
    else if (flm == FLM_POSITION_HOLD || flm == FLM_RTH || flm == FLM_MISSION) {
        mavModes |= MAV_MODE_FLAG_GUIDED_ENABLED;
    }

    uint8_t mavSystemState = 0;
    if (ARMING_FLAG(ARMED)) {
        if (failsafeIsActive()) {
            mavSystemState = MAV_STATE_CRITICAL;
        }
        else {
            mavSystemState = MAV_STATE_ACTIVE;
        }
    }
    else if (isCalibrating()) {
        mavSystemState = MAV_STATE_CALIBRATING;
    }
    else {
        mavSystemState = MAV_STATE_STANDBY;
    }

    mavlink_msg_heartbeat_pack(mavSystemId, mavComponentId, &mavSendMsg,
        // type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
        mavSystemType,
        // autopilot Autopilot type / class. defined in MAV_AUTOPILOT ENUM
        MAV_AUTOPILOT_GENERIC,
        // base_mode System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h
        mavModes,
        // custom_mode A bitfield for use for autopilot-specific flags.
        mavCustomMode,
        // system_status System status flag, see MAV_STATE ENUM
        mavSystemState);

    mavlinkSendMessage();
}

void processMAVLinkTelemetry(timeUs_t currentTimeUs)
{
    // is executed @ TELEMETRY_MAVLINK_MAXRATE rate
    if (mavlinkStreamTrigger(MAV_DATA_STREAM_EXTENDED_STATUS)) {
        mavlinkSendSystemStatus();
    }

    if (mavlinkStreamTrigger(MAV_DATA_STREAM_RC_CHANNELS)) {
        mavlinkSendRCChannelsAndRSSI();
    }

#ifdef USE_GPS
    if (mavlinkStreamTrigger(MAV_DATA_STREAM_POSITION)) {
        mavlinkSendPosition(currentTimeUs);
    }
#endif

    if (mavlinkStreamTrigger(MAV_DATA_STREAM_EXTRA1)) {
        mavlinkSendAttitude();
    }

    if (mavlinkStreamTrigger(MAV_DATA_STREAM_EXTRA2)) {
        mavlinkSendHUDAndHeartbeat();
    }
}

// GTMissionPoint gtmission[GT_MISSION_CAPACITY];
// uint8_t gtmission_count;
//uint8_t gtmission_count_received;
    



uint8_t gtmsgbuf[32];
uint8_t gtstate = 0;
uint32_t gtmsglen = 0;
uint32_t gtmsgcurlen = 0;
uint8_t gtc1, gtc2;

uint8_t gt_clear(void){
    gtstate = 0;
    gtc1 = 0;
    gtc2 = 0;
    return 0;
}

void gtcrc(uint8_t c){
    gtc1 += c;
    gtc2 += gtc1;
}

uint8_t gt_parse_char( char c ){
   gt_input_char_count++;
   if( gtstate == 0 ){  //Header
       if( c != 0x67 ) return gt_clear();       
       gtstate++;
   }else if( gtstate == 1){  //Crypter
       if( c != 0x41 ) return gt_clear();       
       gtstate++;
   }else if( gtstate == 2){  // Source/Dest
       if( c != 0 ) return gt_clear();       
       gtstate++;
   }else if( gtstate == 3){  //Len
       if( c > 2 ) return gt_clear();       
       gtstate++;
       gtmsglen = c * 16;
       gtmsgcurlen = 0;
   }else if( gtstate == 4){
       gtmsgbuf[gtmsgcurlen] = c;
       gtmsgcurlen++;
       if( gtmsgcurlen >= gtmsglen ){
           gtstate++;
       }
   }else if( gtstate == 5){  
       if( c != gtc1 ){
           //gt_crc_err();
           return gt_clear();
       }
       gtstate++;
       return 0; // Not need to calc crc!
   }else if( gtstate == 6){  
       if( c != gtc2 ){
           //gt_crc_err();
           return gt_clear();
       }
       gt_parse_message();
       gt_clear();
       return 1; // Not need to calc crc!
   }
   
   gtcrc(c);
   return 0;
}

timeUs_t last_telemetry_time = 0;

static bool processMAVLinkIncomingTelemetry(timeUs_t currentTimeUs)
{
    while (serialRxBytesWaiting(mavlinkPort)) {
        // Limit handling to one message per cycle
        char c = serialRead(mavlinkPort);
        if( gt_parse_char(c) ){
		   last_telemetry_time = currentTimeUs;
        }
    }

    if( ram_max_telemetry_lag_for_rth_s > 0 ){
    	timeUs_t tmp = (currentTimeUs - last_telemetry_time);
    	tmp /= 1000000;
    	if( tmp > ram_max_telemetry_lag_for_rth_s ){
    		ics_start_telemetry_lost();
    	}
    	else {
    		//ics_set_warning( WNG_TELEMETRY_LOST, 0);
    	}
    }

    return false;
}

timeUs_t lastGTMessage;
uint8_t msg_step = 0;


extern float cur_alt_err;
//rcCommand[PITCH]
extern float alt_res;

extern float desired_pitch_rate;
extern int servoOutputEnabled;
extern uint8_t servoRuleCount;

extern uint32_t dist_to_origin;
extern int16_t poshold_direction;

uint8_t msg_step_low_priority = 0;

void handleMAVLinkTelemetry(timeUs_t currentTimeUs)
{
    if (!mavlinkTelemetryEnabled) {
        return;
    }

    if (!mavlinkPort) {
        return;
    }

    processMAVLinkIncomingTelemetry(currentTimeUs);
    
    
    
    if( lastGTMessage + systemConfig()->cx_telemetry_period_ms*1000 < currentTimeUs ){
        lastGTMessage = currentTimeUs;
        if( silence_timeout > 0){
        	silence_timeout--;
        } else {
			if( msg_step == 0 ){
			  gtSendMessage();
			  msg_step++;
			} else  if( msg_step == 1 ){
			  if( msg_step_low_priority == 0 ){
				  gtSend_APInfo();
				  msg_step_low_priority ++;
			  } else {
				  gtSend_DebugInfo(is_i_line,0, 0 , dist_to_origin, poshold_direction, 0 );
				  msg_step_low_priority = 0;
			  }
			  msg_step++;
			} else  if( msg_step == 2 ){
			  gtSend_PowerInfo();
			  gtSend_Rotation();
			  msg_step++;
			} else {
			  gtSend_CheckReady( isArmingDisabled(), armingFlags, cur_pos.warnings);
			  int new_pl_ix = X_PL_GetIx();
			  if( new_pl_ix >= 0 ){
				  //gtSend_PayloadInfo(new_pl_ix);
				  gtSend_PayloadInfoEx(new_pl_ix);
			  } else {
				  gtSend_DebugLFInfo();
			  }
			  msg_step = 0;
			}
        }
    }
    // if( en_mode == 0 ){
        // mwArm();
        // ENABLE_FLIGHT_MODE(NAV_LAUNCH_MODE);
        // //en_mode = 1;
    // }
}



void gtSendMessage(void){
    uint8_t spd = (int)(posControl.actualState.velXY / 100.0f);
    int16_t h = (int)(getEstimatedActualPosition(Z) / 100.0f);
    int lat = gpsSol.llh.lat;
    int lon = gpsSol.llh.lon;
    //uint8_t az_int = (int)( attitude.values.yaw * 40.74 );// 180/3.14 * 256/360
    uint8_t az_int = (int)( cur_pos.az * 0.711 / 10);// 180/3.14 * 256/360

    uint8_t fmode = 0;
    
    if (FLIGHT_MODE(MANUAL_MODE)) fmode = 1;

    if (FLIGHT_MODE(FAILSAFE_MODE)) fmode |= 2;

    if (FLIGHT_MODE(NAV_RTH_MODE)) fmode |= 4;

    if (FLIGHT_MODE(NAV_POSHOLD_MODE)) fmode |= 8;

    if (FLIGHT_MODE(NAV_WP_MODE)) fmode |= 16;

    if (FLIGHT_MODE(NAV_ALTHOLD_MODE)) fmode |= 32;

    if (FLIGHT_MODE(ANGLE_MODE)) fmode |= 64;

    if (FLIGHT_MODE(NAV_LAUNCH_MODE)) fmode |= 128;
    

    //if( ram_use_only_gps_speed > 0 ){
	//	gtbuf[16] = cur_pos.cruise_spd/100;
    //} else {
    	gtbuf[16] = spd;
    //}



    gtbuf[4] = 1;
    gtbuf[5] = fmode;
    gtbuf[6] = lat & 0xFF; lat = lat >> 8;
    gtbuf[7] = lat & 0xFF; lat = lat >> 8;
    gtbuf[8] = lat & 0xFF; lat = lat >> 8;
    gtbuf[9] = lat & 0xFF; lat = lat >> 8;
    gtbuf[10] = lon & 0xFF; lon = lon >> 8;
    gtbuf[11] = lon & 0xFF; lon = lon >> 8;
    gtbuf[12] = lon & 0xFF; lon = lon >> 8;
    gtbuf[13] = lon & 0xFF; lon = lon >> 8;
    gtbuf[14] = h & 0xFF; h = h >> 8;
    gtbuf[15] = h & 0xFF; h = h >> 8;
    //gtbuf[16] = spd;
    gtbuf[17] = XM_Current;
    gtbuf[18] = az_int;

    if( systemConfig()->cx_emulator_mode > 0 ){
    	gtbuf[19] = (int)(2300/10.0) & 0xFF;
    } else {
    	gtbuf[19] = (int)(getBatteryVoltage()/10.0) & 0xFF;
    }

    gt_send_message();

//  gtbuf[20] = 0;
//  gtbuf[21] = 0;
//
//  for( int i = 0; i < 20; i++ ){
//      gtbuf[20] = gtbuf[20] + gtbuf[i];
//      gtbuf[21] = gtbuf[21] + gtbuf[20];
//  }
//
//	for (int i = 0; i < 22; i++) {
//		serialWrite(mavlinkPort, gtbuf[i]);
//	}
    

}

void gt_send_message( void ){
  gtbuf[0] =0x67;
  gtbuf[1] = 0x41;  
  gtbuf[2] = 1;
  gtbuf[3] = 1; 

  gtbuf[20] = 0;
  gtbuf[21] = 0;
  for( int i = 0; i < 20; i++ ){
      gtbuf[20] = gtbuf[20] + gtbuf[i];
      gtbuf[21] = gtbuf[21] + gtbuf[20];
  }  

  for (int i = 0; i < 22; i++) {
      serialWrite(mavlinkPort, gtbuf[i]);
  }
}


void gt_send_message_long( uint8_t block_count ){
  gtbuf[0] =0x67;
  gtbuf[1] = 0x41;
  gtbuf[2] = 1;
  gtbuf[3] = block_count;

  gtbuf[4+16*block_count] = 0;
  gtbuf[5+16*block_count] = 0;
  for( int i = 0; i < 4+16*block_count; i++ ){
      gtbuf[4+16*block_count] = gtbuf[4+16*block_count] + gtbuf[i];
      gtbuf[5+16*block_count] = gtbuf[5+16*block_count] + gtbuf[4+16*block_count];
  }

  for (int i = 0; i < 6+16*block_count; i++) {
      serialWrite(mavlinkPort, gtbuf[i]);
  }
}

void gtSend_PointOK(uint16_t curwp, uint8_t is_ok){
    gtbuf[4] = 0x11;
    gtbuf[5] = is_ok;
    for( int i = 6; i < 20; i++ ){
        gtbuf[i] = 0;
    }

    gt_send_message();
}

void gtSend_APInfo(){
    gtbuf[4] = 0x04;

    gtbuf[5] = systemConfig()->cx_version & 0xFF;
    gtbuf[6] = (systemConfig()->cx_version >> 8) & 0xFF;

    for(uint8_t i = 0; i < 12; i++)
    	gtbuf[7+i] = systemConfig()->name[i];

    gt_send_message();
}


void gtSend_Point_Ex(uint16_t curwp, uint8_t action, uint32_t lat, uint32_t lon, uint32_t h, uint8_t p1, uint8_t p2, uint16_t mlen, uint16_t pl_dist, uint8_t oid, uint8_t otype){
    gtbuf[4] = 0x10;
    gtbuf[5] = action;
    for( int i = 6; i < 26; i++ ){
        gtbuf[i] = 0;
    }
    
    gtbuf[6] = lat & 0xFF; lat = lat >> 8;
    gtbuf[7] = lat & 0xFF; lat = lat >> 8;
    gtbuf[8] = lat & 0xFF; lat = lat >> 8;
    gtbuf[9] = lat & 0xFF; lat = lat >> 8;
    gtbuf[10] = lon & 0xFF; lon = lon >> 8;
    gtbuf[11] = lon & 0xFF; lon = lon >> 8;
    gtbuf[12] = lon & 0xFF; lon = lon >> 8;
    gtbuf[13] = lon & 0xFF; lon = lon >> 8;
    gtbuf[14] = h & 0xFF; h = h >> 8;
    gtbuf[15] = h & 0xFF; h = h >> 8;
    gtbuf[16] = h & 0xFF; h = h >> 8;
    gtbuf[17] = h & 0xFF; h = h >> 8;



    gtbuf[18] = curwp & 0xFF; curwp = curwp >> 8;
    gtbuf[19] = curwp & 0xFF; curwp = curwp >> 8;

    gtbuf[20] = p1;
    gtbuf[21] = p2;
    
    gtbuf[22] = mlen & 0xFF; mlen = mlen >> 8;
    gtbuf[23] = mlen & 0xFF; mlen = mlen >> 8;

    gtbuf[24] = pl_dist & 0xFF; pl_dist = pl_dist >> 8;
    gtbuf[25] = pl_dist & 0xFF; pl_dist = pl_dist >> 8;

    gtbuf[26] = oid;
    gtbuf[27] = otype;

    gt_send_message_long(2);
}

void gtSend_Setting(uint8_t sname[10], uint8_t answ, uint32_t val_mul_100){
    gtbuf[4] = 0x50;
    for( int i = 0; i < 10; i++ ){
        gtbuf[i+5] = sname[i];
    }
    
    gtbuf[15] = answ;
    
    gtbuf[16] = val_mul_100 & 0xFF; val_mul_100 = val_mul_100 >> 8;
    gtbuf[17] = val_mul_100 & 0xFF; val_mul_100 = val_mul_100 >> 8;
    gtbuf[18] = val_mul_100 & 0xFF; val_mul_100 = val_mul_100 >> 8;
    gtbuf[19] = val_mul_100 & 0xFF;
    
    gt_send_message();
}


extern float alts_res;
extern float cur_alts_err;
extern float cur_speed_err;
extern uint16_t XM_Count;
extern int az_to_point;
extern XMissionPoint mp;
extern float deltaX;
extern float deltaY;
extern int16_t nu;
extern uint32_t mydist;
extern TicsPos cur_pos;
extern float az_res;

//extern uint8_t is_i_line;

extern float d2s;
extern float d2e;

void gtSend_CheckReady(uint8_t is_ready, uint32_t err_code, uint32_t warning_code){
    gtbuf[4] = 0x21;
    gtbuf[5] = is_ready;
    gtbuf[6] = gpsSol.numSat;
    gtbuf[7] = cur_pos.is_plane_mode;//cxIsIPlane();
    
    //debug_test = baroGetLatestAltSpeed();
    debug_test = mydist;
    gtbuf[8] = debug_test  & 0xFF;
    gtbuf[9] = (debug_test >>8) & 0xFF;
    //AP state
    gtbuf[10] = ics_get_state();
    
    gtbuf[11] = posControl.navState;
    
    err_code = (err_code & 0xFFFFFFF) | (is_home_valid << 29) | (is_mission_valid << 30);
    
    gtbuf[12] = err_code & 0xFF; err_code = err_code >> 8;
    gtbuf[13] = err_code & 0xFF; err_code = err_code >> 8;
    gtbuf[14] = err_code & 0xFF; err_code = err_code >> 8;
    gtbuf[15] = err_code & 0xFF; err_code = err_code >> 8;
    
    //warning_code =
    gtbuf[16] = warning_code & 0xFF; warning_code = warning_code >> 8;
    gtbuf[17] = warning_code & 0xFF; warning_code = warning_code >> 8;
    gtbuf[18] = warning_code & 0xFF; warning_code = warning_code >> 8;
    gtbuf[19] = warning_code & 0xFF; warning_code = warning_code >> 8;

    
    gt_send_message();
}

void gtSend_DebugInfo(uint8_t u1, uint8_t u2, uint8_t u3, uint32_t x1, uint32_t x2, uint32_t x3){
    gtbuf[4] = 0x0F;
    gtbuf[5] = u1;
    gtbuf[6] = u2;
    gtbuf[7] = u3;
    

    
    gtbuf[8] = x1 & 0xFF; x1 = x1 >> 8;
    gtbuf[9] = x1 & 0xFF; x1 = x1 >> 8;
    gtbuf[10] = x1 & 0xFF; x1 = x1 >> 8;
    gtbuf[11] = x1 & 0xFF; x1 = x1 >> 8;
    gtbuf[12] = x2 & 0xFF; x2 = x2 >> 8;
    gtbuf[13] = x2 & 0xFF; x2 = x2 >> 8;
    gtbuf[14] = x2 & 0xFF; x2 = x2 >> 8;
    gtbuf[15] = x2 & 0xFF; x2 = x2 >> 8;
    gtbuf[16] = x3 & 0xFF; x3 = x3 >> 8;
    gtbuf[17] = x3 & 0xFF; x3 = x3 >> 8;
    gtbuf[18] = x3 & 0xFF; x3 = x3 >> 8;
    gtbuf[19] = x3 & 0xFF; x3 = x3 >> 8;    

    gt_send_message();
}

int vbat_div_100;
int32_t amp;
int32_t mah;
int16_t vert_speed;

void gtSend_PowerInfo(){
    gtbuf[4] = 0x03;

    if( systemConfig()->cx_emulator_mode > 0 ){
    	vbat_div_100 = 2412;
    } else {
    	vbat_div_100 = (int)(getBatteryVoltage());
    }
    amp = getAmperage();
    if( amp < 0 ) amp = 0;
    mah = getMAhDrawn();
    
    
    gtbuf[5] = vbat_div_100 & 0xFF; vbat_div_100 = vbat_div_100 >> 8;
    gtbuf[6] = vbat_div_100 & 0xFF; 

    gtbuf[7] = amp & 0xFF; amp = amp >> 8;
    gtbuf[8] = amp & 0xFF; 
    
    gtbuf[9] = mah & 0xFF; mah = mah >> 8;
    gtbuf[10] = mah & 0xFF; mah = mah >> 8;
    gtbuf[11] = mah & 0xFF; mah = mah >> 8;
    gtbuf[12] = mah & 0xFF;
    
    //In 0..250
    if( rcCommand[THROTTLE] <= 1000 ){
    	gtbuf[14] = 0;
    } else {
    	gtbuf[14] = (uint8_t)(( rcCommand[THROTTLE] - 1000.0 ) * 0.25);
    }

    gtbuf[15] = calculateBatteryPercentage();

    uint16_t mahrem_div = getBatteryRemainingCapacity() / 10;

    gtbuf[16] = mahrem_div & 0xFF; mahrem_div = mahrem_div >> 8;
    gtbuf[17] = mahrem_div & 0xFF;


    vert_speed = cur_pos.hspd;

    gtbuf[18] = vert_speed & 0xFF; vert_speed = vert_speed >> 8;
    gtbuf[19] = vert_speed & 0xFF;

    gt_send_message();
}

void gtSend_PayloadInfo( uint16_t payload_number ){
	gtbuf[4] = 0x46;
	gtbuf[5] = 0;

	if( payload_number >= X_PLCount){
		payload_number = 0;
	}

	gtbuf[6] = X_PL[payload_number].la & 0xFF;
	gtbuf[7] = (X_PL[payload_number].la >> 8) & 0xFF;
	gtbuf[8] = (X_PL[payload_number].la >> 16) & 0xFF;
	gtbuf[9] = (X_PL[payload_number].la >> 24) & 0xFF;
	gtbuf[10] = X_PL[payload_number].lo & 0xFF;
	gtbuf[11] = (X_PL[payload_number].lo >> 8) & 0xFF;
	gtbuf[12] = (X_PL[payload_number].lo >> 16) & 0xFF;
	gtbuf[13] = (X_PL[payload_number].lo >> 24) & 0xFF;

	gtbuf[14] = X_PL[payload_number].h & 0xFF;
	gtbuf[15] = (X_PL[payload_number].h >> 8) & 0xFF;
	gtbuf[16] = (X_PL[payload_number].h >> 16) & 0xFF;
	gtbuf[17] = (X_PL[payload_number].h >> 24) & 0xFF;

	gtbuf[18] = payload_number & 0xFF;
	gtbuf[19] = (payload_number>>8) & 0xFF;
    gtbuf[20] = X_PLCount & 0xFF;
    gtbuf[21] = (X_PLCount>>8) & 0xFF;

	gtbuf[22] = (X_PL[payload_number].r) & 0xFF;
	gtbuf[23] = (X_PL[payload_number].r >> 8) & 0xFF;
	gtbuf[24] = (X_PL[payload_number].p) & 0xFF;
	gtbuf[25] = (X_PL[payload_number].p >> 8) & 0xFF;
	gtbuf[26] = (X_PL[payload_number].y) & 0xFF;
	gtbuf[27] = (X_PL[payload_number].y >> 8) & 0xFF;

	uint32_t time_ns = X_PL[payload_number].time % 1000;
	uint32_t unix_time = X_PL[payload_number].time / 1000;
	time_ns *= 1000000;

	gtbuf[28] = (time_ns) & 0xFF;
	gtbuf[29] = (time_ns >> 8) & 0xFF;
	gtbuf[30] = (time_ns >> 16) & 0xFF;
	gtbuf[31] = (time_ns >> 24) & 0xFF;

	gtbuf[32] = (unix_time) & 0xFF;
	gtbuf[33] = (unix_time >> 8) & 0xFF;
	gtbuf[34] = (unix_time >> 16) & 0xFF;
	gtbuf[35] = (unix_time >> 24) & 0xFF;

	gt_send_message_long(2);
	xlogger_gtmsg_pass_send(gtbuf, 5, 35, 0x35);
}

void gtSend_PayloadInfoEx( uint16_t payload_number ){
	gtbuf[4] = 0x49;
	gtbuf[5] = 0;

	if( payload_number >= X_PLCount){
		payload_number = 0;
	}

	gtbuf[6] = X_PL[payload_number].la & 0xFF;
	gtbuf[7] = (X_PL[payload_number].la >> 8) & 0xFF;
	gtbuf[8] = (X_PL[payload_number].la >> 16) & 0xFF;
	gtbuf[9] = (X_PL[payload_number].la >> 24) & 0xFF;
	gtbuf[10] = X_PL[payload_number].lo & 0xFF;
	gtbuf[11] = (X_PL[payload_number].lo >> 8) & 0xFF;
	gtbuf[12] = (X_PL[payload_number].lo >> 16) & 0xFF;
	gtbuf[13] = (X_PL[payload_number].lo >> 24) & 0xFF;

	gtbuf[14] = X_PL[payload_number].h & 0xFF;
	gtbuf[15] = (X_PL[payload_number].h >> 8) & 0xFF;
	gtbuf[16] = (X_PL[payload_number].h >> 16) & 0xFF;
	gtbuf[17] = (X_PL[payload_number].h >> 24) & 0xFF;

	gtbuf[18] = payload_number & 0xFF;
	gtbuf[19] = (payload_number>>8) & 0xFF;
    gtbuf[20] = X_PLCount & 0xFF;
    gtbuf[21] = (X_PLCount>>8) & 0xFF;

	gtbuf[22] = (X_PL[payload_number].r) & 0xFF;
	gtbuf[23] = (X_PL[payload_number].r >> 8) & 0xFF;
	gtbuf[24] = (X_PL[payload_number].p) & 0xFF;
	gtbuf[25] = (X_PL[payload_number].p >> 8) & 0xFF;
	gtbuf[26] = (X_PL[payload_number].y) & 0xFF;
	gtbuf[27] = (X_PL[payload_number].y >> 8) & 0xFF;

	uint32_t time_ns = X_PL[payload_number].time % 1000;
	uint32_t unix_time = X_PL[payload_number].time / 1000;
	time_ns *= 1000000;

	gtbuf[28] = (time_ns) & 0xFF;
	gtbuf[29] = (time_ns >> 8) & 0xFF;
	gtbuf[30] = (time_ns >> 16) & 0xFF;
	gtbuf[31] = (time_ns >> 24) & 0xFF;

	gtbuf[32] = (unix_time) & 0xFF;
	gtbuf[33] = (unix_time >> 8) & 0xFF;
	gtbuf[34] = (unix_time >> 16) & 0xFF;
	gtbuf[35] = (unix_time >> 24) & 0xFF;

	gtbuf[36] = X_PL[payload_number].gpsh_cm & 0xFF;
	gtbuf[37] = (X_PL[payload_number].gpsh_cm >> 8) & 0xFF;
	gtbuf[38] = (X_PL[payload_number].gpsh_cm >> 16) & 0xFF;
	gtbuf[39] = (X_PL[payload_number].gpsh_cm >> 24) & 0xFF;

	gt_send_message_long(3);
	xlogger_gtmsg_pass_send(gtbuf, 5, 39, 0x36);
}

void gtSend_PayloadCount(){
	gtbuf[4] = 0x48;

	gtbuf[5] = (X_PLCount) & 0xFF;
	gtbuf[6] = (X_PLCount >> 8) & 0xFF;
	gtbuf[7] = (X_PLCount >> 16) & 0xFF;
	gtbuf[8] = (X_PLCount >> 24) & 0xFF;

	gt_send_message();
}

void gtSend_DebugLFInfo(){
	gtbuf[4] = 0x0E;
	gtbuf[5] = XM_Current;

	//0 in point nav
	//is_i_line == 1 in line
	//is_i_line == 2 in prepare
	//is_i_line == 3 if reached;
    gtbuf[6] = is_i_line;
    gtbuf[7] = x_big_rotate;
    gtbuf[8] = (cur_pos.need_nu) & 0xFF;
    gtbuf[9] = (cur_pos.need_nu >> 8) & 0xFF;

    gtbuf[10] = sl.la & 0xFF;
    gtbuf[11] = (sl.la >> 8) & 0xFF;
    gtbuf[12] = (sl.la >> 16) & 0xFF;
    gtbuf[13] = (sl.la >> 24) & 0xFF;
    gtbuf[14] = sl.lo & 0xFF;
    gtbuf[15] = (sl.lo >> 8) & 0xFF;
    gtbuf[16] = (sl.lo >> 16) & 0xFF;
    gtbuf[17] = (sl.lo >> 24) & 0xFF;

    gtbuf[18] = el.la & 0xFF;
    gtbuf[19] = (el.la >> 8) & 0xFF;
    gtbuf[20] = (el.la >> 16) & 0xFF;
    gtbuf[21] = (el.la >> 24) & 0xFF;
    gtbuf[22] = el.lo & 0xFF;
    gtbuf[23] = (el.lo >> 8) & 0xFF;
    gtbuf[24] = (el.lo >> 16) & 0xFF;
    gtbuf[25] = (el.lo >> 24) & 0xFF;


	for( int i = 26; i < 36; i++ ){
		gtbuf[i] = 0;
	}


	gt_send_message_long(2);
}

void gtSend_Rotation(){
    gtbuf[4] = 0x02;
    gtbuf[5] = attitude.values.roll & 0xFF;
    gtbuf[6] = (attitude.values.roll >> 8) & 0xFF;
    int16_t pitch_plane_mode_i16 = - attitude.values.pitch - systemConfig()->cx_planemode_pitch_angle_decidegi;

    gtbuf[7] = pitch_plane_mode_i16 & 0xFF;
    gtbuf[8] = (pitch_plane_mode_i16 >> 8) & 0xFF;
    gtbuf[9] = attitude.values.yaw & 0xFF;
    gtbuf[10] = (attitude.values.yaw >> 8) & 0xFF;

    int16_t az_i16 = (int16_t)(cur_pos.mag_az);
    gtbuf[11] = az_i16 & 0xFF;
    gtbuf[12] = (az_i16 >> 8) & 0xFF;

//    if( ram_use_only_gps_speed > 0 ){
//		gtbuf[13] = (cur_pos.used_spd / 10) & 0xFF;
//		gtbuf[14] = ((cur_pos.used_spd / 10) >> 8) & 0xFF;
//		gtbuf[15] = (cur_pos.cruise_spd/10) & 0xFF;
//		gtbuf[16] = ((cur_pos.cruise_spd/10) >> 8) & 0xFF;
//    } else {
    //gtbuf[13] = (int)(cur_alt_change) & 0xFF;
    //gtbuf[14] = ((int)(cur_alt_change) >> 8) & 0xFF;
		gtbuf[13] = (cur_pos.airspd / 10) & 0xFF;
		gtbuf[14] = ((cur_pos.airspd / 10) >> 8) & 0xFF;
		gtbuf[15] = (cur_pos.gndspd/10) & 0xFF;
		gtbuf[16] = ((cur_pos.gndspd/10) >> 8) & 0xFF;
//    }

    uint8_t gps_info = 0;


    if (gpsSol.fixType == GPS_NO_FIX)
    	gps_info = 1 << 6;
    else if (gpsSol.fixType == GPS_FIX_2D)
    	gps_info = 2 << 6;
    else if (gpsSol.fixType == GPS_FIX_3D)
    	gps_info = 3 << 6;

    if( gpsSol.numSat >= 64){
    	gps_info |= 63;
    } else {
    	gps_info |= gpsSol.numSat;
    }



    gtbuf[17] = gps_info;
	gtbuf[18] = ics_get_state();
	gtbuf[19] = cur_pos.manual_modes;


    gt_send_message();
}

void gtSend_GoAck( uint8_t result, uint32_t error_code, uint32_t warning_code){
    gtbuf[4] = 0x23;
    gtbuf[5] = result;
    for( int i = 6; i < 20; i++ ){
        gtbuf[i] = 0;
    }
    
    gtbuf[12] = error_code & 0xFF; error_code = error_code >> 8;
    gtbuf[13] = error_code & 0xFF; error_code = error_code >> 8;
    gtbuf[14] = error_code & 0xFF; error_code = error_code >> 8;
    gtbuf[15] = error_code & 0xFF; error_code = error_code >> 8;
    gtbuf[16] = warning_code & 0xFF; warning_code = warning_code >> 8;
    gtbuf[17] = warning_code & 0xFF; warning_code = warning_code >> 8;
    gtbuf[18] = warning_code & 0xFF; warning_code = warning_code >> 8;
    gtbuf[19] = warning_code & 0xFF; warning_code = warning_code >> 8;
    
    gt_send_message();
}

void gtSend_CheckAck( uint8_t hw, uint32_t state, uint32_t warning_code){
    gtbuf[4] = 0x26;
    gtbuf[5] = hw;
    for( int i = 6; i < 20; i++ ){
        gtbuf[i] = 0;
    }

    gtbuf[11] = state;

    gtbuf[16] = warning_code & 0xFF; warning_code = warning_code >> 8;
    gtbuf[17] = warning_code & 0xFF; warning_code = warning_code >> 8;
    gtbuf[18] = warning_code & 0xFF; warning_code = warning_code >> 8;
    gtbuf[19] = warning_code & 0xFF; warning_code = warning_code >> 8;

    gt_send_message();
}

#define CUR_SILENCE_TIMEOUT (1)

void gt_parse_message(void){
    //Extended point -- Real point with data
	if( gtmsgbuf[0] == 0x10 ){
		xlogger_gtmsg_pass_send(gtmsgbuf, 1, 31, 0xB5);
		silence_timeout = CUR_SILENCE_TIMEOUT;
		uint16_t cur_wp = (gtmsgbuf[15] << 8) + gtmsgbuf[14];
        uint8_t action = gtmsgbuf[1];
        if(  (action == 100) || (action == 101) ){ 
        	uint16_t new_mission_count = (gtmsgbuf[19] << 8) + gtmsgbuf[18];
        	uint16_t new_pdist = (gtmsgbuf[21] << 8) + gtmsgbuf[20];
            uint32_t la = (gtmsgbuf[5] << 24) + (gtmsgbuf[4] << 16) + (gtmsgbuf[3] << 8) + gtmsgbuf[2];
            uint32_t lo = (gtmsgbuf[9] << 24) + (gtmsgbuf[8] << 16) + (gtmsgbuf[7] << 8) + gtmsgbuf[6];
            int32_t h = (gtmsgbuf[13] << 24) + (gtmsgbuf[12] << 16) + (gtmsgbuf[11] << 8) + gtmsgbuf[10];
			//uint16_t h = (gtmsgbuf[11] << 8) + gtmsgbuf[10];

            uint8_t kind = gtmsgbuf[16];
            
            if( 1 == XM_AddReceived( cur_wp, new_mission_count, la, lo, h, kind, gtmsgbuf[17] , new_pdist, gtmsgbuf[22], gtmsgbuf[23]) ){
               //gtSend_PowerInfo();
               //NSU BUG
            	//if( cur_wp > 0 ){
            		gtSend_PointOK(cur_wp, 201);
            	//}
            } else {
               gtSend_PointOK(cur_wp, 203);
            }

//        } else if(  action == 102 ){
//            if( cur_wp < posControl.waypointCount ){
//                uint8_t p1 = posControl.waypointCount & 0xFF;
//                uint8_t p2 = (posControl.waypointCount>>8) & 0xFF;
//
//                gtSend_Point_Ex( cur_wp, 101, posControl.waypointList[cur_wp].lat, posControl.waypointList[cur_wp].lon, posControl.waypointList[cur_wp].alt, p1, p2, posControl.waypointCount, 0 );
//            } else {
//                if( posControl.waypointCount == 0 ){
//                    //if no mission -- ask zero point with 0 point size mission
//                    gtSend_Point_Ex( 0, 101, 0, 0, 0, 0, 0, 0, 0);
//                } else {
//                  gtSend_PointOK(0, 204);
//                }
//            }
        } else if( action == 6  ){
        	uint32_t la = (gtmsgbuf[5] << 24) + (gtmsgbuf[4] << 16) + (gtmsgbuf[3] << 8) + gtmsgbuf[2];
			uint32_t lo = (gtmsgbuf[9] << 24) + (gtmsgbuf[8] << 16) + (gtmsgbuf[7] << 8) + gtmsgbuf[6];
			int32_t h = (gtmsgbuf[13] << 24) + (gtmsgbuf[12] << 16) + (gtmsgbuf[11] << 8) + gtmsgbuf[10];

			ics_goto_origin(la, lo, h);
			gtSend_PointOK(0, 204);
        }
    }
	//Point Actions
    if( gtmsgbuf[0] == 0x11 ){
    	xlogger_gtmsg_pass_send(gtmsgbuf, 1, 31, 0x34);
        uint16_t cur_wp = (gtmsgbuf[13] << 8) + gtmsgbuf[12];
        uint8_t action = gtmsgbuf[1];
        //GOTO HOME
        if( action == 1){
        	//=1 if we in air, else =0
        	if( ics_start_rth() > 0 ){
        	    gtSend_PointOK(0, 204);
        	} else {
        		gtSend_PointOK(cur_wp, 203);
        	}
        } else if( action == 2){
        	//GOTO POINT

        	if( ics_start_goto(cur_wp) > 0 ){
        	    gtSend_PointOK(0, 204);
        	} else {
        		gtSend_PointOK(cur_wp, 203);
        	}
        } else if( action == 3){
        	if( systemConfig()->cx_gps_lost_test_mode == 1){
        		is_gps_off = !is_gps_off;
        	} else {
				if( ics_start_land(3) ){
					gtSend_PointOK(0, 204);
				} else {
					gtSend_PointOK(cur_wp, 203);
				}
        	}
        } else if( action == 4){
        	// PAUSE MISSION
			if( ics_pause() ){
				gtSend_PointOK(0, 204);
			} else {
				gtSend_PointOK(cur_wp, 203);
			}
        } else if( action == 5){
        	// RESUME MISSION
			if( ics_continue_from_pause() ){
				gtSend_PointOK(0, 204);
			} else {
				gtSend_PointOK(cur_wp, 203);
			}

        } else if(  action == 102 ){
        	silence_timeout = CUR_SILENCE_TIMEOUT;
        	if( cur_wp < XM_Count ){
                XMissionPoint p = XM_Get(cur_wp);
                gtSend_Point_Ex( cur_wp, 101, p.la, p.lo, p.h, p.kind, p.p1, XM_Count, p.pdist, p.object_id, p.object_type );
            } else {
                if( XM_Count == 0 ){
                    //if no mission -- ask zero point with 0 point size mission
                    gtSend_Point_Ex( 0, 101, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
                } else {
                  gtSend_PointOK(0, 204);
                }
            }
        }
    } else if( gtmsgbuf[0] == 0x22 ){
    	xlogger_gtmsg_pass_send(gtmsgbuf, 1, 31, 0x37);
        if( (gtmsgbuf[1] == 0x1) && (gtmsgbuf[2] == 0x0) && (gtmsgbuf[7] == 0x1) ){
            if( XM_Count > 0 ){



                ics_start();
    
                //ENABLE_FLIGHT_MODE(NAV_LAUNCH_MODE);
                
                //posControl.navState = NAV_STATE_LAUNCH_IN_PROGRESS;
                gtSend_GoAck(1, 0, 0); 
            } else {
                gtSend_GoAck(0, ARMING_DISABLED_NAVIGATION_UNSAFE, 0);   
            }
        } if( (gtmsgbuf[1] == 0x0) && (gtmsgbuf[2] == 0x1) && (gtmsgbuf[7] == 0xFF) ){
            mwDisarm(DISARM_SWITCH);
            posControl.navState = NAV_STATE_IDLE;
            ics_stop();
            gtSend_GoAck(255, 0, 0); 
        }else {
            gtSend_GoAck(0, 0, 1);   
        }
    // CHECK COMMAND
    } else if( gtmsgbuf[0] == 0x25 ){
        uint8_t is_on = gtmsgbuf[11];
    	if( (gtmsgbuf[1] == 0x1)){
        	gtSend_CheckAck(1, 1, 0);
        	if( is_on == 1){
        		payloadExtraShot();

        		ics_start_check_servos();
        	} else {
        	    ics_stop_check_servos();
        	}
        	//gtmsgbuf[14] -- LR

        } else if( (gtmsgbuf[1] == 0x2) ){
        	if(is_on == 1){
        		if( posControl.navState == NAV_STATE_IDLE ){
        			cx_check_engines  = 1;
        			gtSend_CheckAck(2, 1, 0);
        		}
        	}else{
        		if( posControl.navState == NAV_STATE_IDLE ){
        			cx_check_engines  = 0;
        			gtSend_CheckAck(2, 0, 0);
        		}
        	}
        } else if( (gtmsgbuf[1] == 0x3)){
			gtSend_CheckAck(1, 1, 0);
			if( is_on == 1){
				payload[0] = -1;
			} else {
				payload[0] = 1;
			}
			//gtmsgbuf[14] -- LR
        }else {
        	gtSend_CheckAck(0, 0, 1);
        }
    } else if( gtmsgbuf[0] == 0x45 ){ //Request PayloadInfo
    	uint32_t plinfo_number = (gtmsgbuf[4] << 24) + (gtmsgbuf[3] << 16) + (gtmsgbuf[2] << 8) + gtmsgbuf[1];
    	gtSend_PayloadInfoEx(plinfo_number);
    } else if( gtmsgbuf[0] == 0x27 ){ //ManualControl
		uint8_t manual_control_enabled = gtmsgbuf[1];
		int32_t d_az = (gtmsgbuf[5] << 24) + (gtmsgbuf[4] << 16) + (gtmsgbuf[3] << 8) + gtmsgbuf[2];
		int32_t d_as = (gtmsgbuf[9] << 24) + (gtmsgbuf[8] << 16) + (gtmsgbuf[7] << 8) + gtmsgbuf[6];
		int32_t d_h = (gtmsgbuf[13] << 24) + (gtmsgbuf[12] << 16) + (gtmsgbuf[11] << 8) + gtmsgbuf[10];


		//ics_on_vector_mode_message( manual_control_enabled, d_az, d_as, d_h);

    } else if( gtmsgbuf[0] == 0x47 ){ //Request PayloadCount
    	gtSend_PayloadCount();
    } else if( gtmsgbuf[0] == 0x50 ){
        //Set/Get setting
    	xlogger_gtmsg_pass_send(gtmsgbuf, 1, 31, 0xB6);
        uint8_t sname[10];
        for( int i = 0; i < 10; i++ ){
            sname[i] = gtmsgbuf[i+1];
        }
        uint8_t cmd = gtmsgbuf[11];
        int32_t val = (gtmsgbuf[15] << 24) + (gtmsgbuf[14] << 16) + (gtmsgbuf[13] << 8) + gtmsgbuf[12];   
        cmd = gtParseSetting( sname, cmd, &val );
        gtSend_Setting( sname, cmd, val );
    }else{
        gtSend_PointOK(0, 204);        
    }
}

uint8_t checks( uint8_t s1[10], char s2[10] ){
    for( uint8_t i = 0; i < 10; i++ ){
        if( s1[i] != s2[i] ) return 0;
    }
    return 1;
}

#define PS_SET (2)
#define PS_SETOK (4)
#define PS_SETRANGEERR (5)
#define PS_GET (1)
#define PS_GETOK (3)
#define PS_NOTFOUND (10)

uint8_t gtParseSetting( uint8_t sname[10], uint8_t cmd, int32_t * value ){

	if( checks( sname, "telem_rth " ) ){
		if( cmd == PS_SET ){
			ram_max_telemetry_lag_for_rth_s = 1.0 * (*value) / 100.0;		// seconds

			if (	(ram_max_telemetry_lag_for_rth_s > 0) &&
					(ram_max_telemetry_lag_for_rth_s < systemConfig()->cx_min_telemetry_lag_for_rth_s)	) {
				// (0, min value)  -> set min
				ram_max_telemetry_lag_for_rth_s = systemConfig()->cx_min_telemetry_lag_for_rth_s;
				return PS_SETRANGEERR;
			}

 			return PS_SETOK;
		}
		if( cmd == PS_GET ){
			(*value) = (ram_max_telemetry_lag_for_rth_s * 100.0);
			return PS_GETOK;
		}
	}

	if( checks( sname, "circ_rad  " ) ){
		if( cmd == PS_SET ){
			ram_circle_radius = 1.0 * (*value) / 100.0;
			if (ram_circle_radius < systemConfig()->cx_circle_radius) {
				ram_circle_radius = systemConfig()->cx_circle_radius;
				return PS_SETRANGEERR;
			}
			return PS_SETOK;
		}
		if( cmd == PS_GET ){
			(*value) = (ram_circle_radius * 100.0);
			return PS_GETOK;
		}
	}

	if( checks( sname, "rth_safe_h" ) ){
		if( cmd == PS_SET ){
			ram_rth_safe_h = 1.0 * (*value) / 10000.0;

			if (ram_rth_safe_h < systemConfig()->cx_rth_safe_h) {
				ram_rth_safe_h = systemConfig()->cx_rth_safe_h;
				return PS_SETRANGEERR;
			}

			return PS_SETOK;
		}
		if( cmd == PS_GET ){
			(*value) = (ram_rth_safe_h * 10000.0);
			return PS_GETOK;
		}
	}



	if( checks( sname, "use_gpsspd" ) ){
		if( cmd == PS_SET ){
			ram_use_only_gps_speed = 1.0 * (*value) / 100.0;
			return PS_SETOK;
		}
		if( cmd == PS_GET ){
			(*value) = (ram_use_only_gps_speed * 100.0);
			return PS_GETOK;
		}
	}
	if( checks( sname, "yaw_help_v" ) ){
		if( cmd == PS_SET ){
			ram_yaw_help_v = 1.0 * (*value) / 100.0;
			return PS_SETOK;
		}
		if( cmd == PS_GET ){
			(*value) = (ram_yaw_help_v * 100.0);
			return PS_GETOK;
		}
	}
	if( checks( sname, "yaw_help_a" ) ){
		if( cmd == PS_SET ){
			ram_yaw_help_a = 1.0 * (*value) / 100.0;
			return PS_SETOK;
		}
		if( cmd == PS_GET ){
			(*value) = (ram_yaw_help_a * 100.0);
			return PS_GETOK;
		}
	}

	if( checks( sname, "sml_a_lim " ) ){
		if( cmd == PS_SET ){
			ram_small_angle_limit = 1.0 * (*value) / 100.0;
			return PS_SETOK;
		}
		if( cmd == PS_GET ){
			(*value) = (ram_small_angle_limit * 100.0);
			return PS_GETOK;
		}
	}

	if( checks( sname, "nav_pid_p " ) ){
		if( cmd == PS_SET ){
			ram_nav_pid_p = 1.0 * (*value) / 100.0;
			return PS_SETOK;
		}
		if( cmd == PS_GET ){
			(*value) = (ram_nav_pid_p * 100.0);
			return PS_GETOK;
		}
	}

	if( checks( sname, "nav_pid_i " ) ){
		if( cmd == PS_SET ){
			ram_nav_pid_i = 1.0 * (*value) / 100.0;
			return PS_SETOK;
		}
		if( cmd == PS_GET ){
			(*value) = (ram_nav_pid_i * 100.0);
			return PS_GETOK;
		}
	}

	if( checks( sname, "nav_pid_d " ) ){
		if( cmd == PS_SET ){
			ram_nav_pid_d = 1.0 * (*value) / 100.0;
			return PS_SETOK;
		}
		if( cmd == PS_GET ){
			(*value) = (ram_nav_pid_d * 100.0);
			return PS_GETOK;
		}
	}

	if( checks( sname, "line_pid_p" ) ){
		if( cmd == PS_SET ){
			ram_linefollow_p = 1.0 * (*value) / 100.0;
			return PS_SETOK;
		}
		if( cmd == PS_GET ){
			(*value) = (ram_linefollow_p * 100.0);
			return PS_GETOK;
		}
	}
	if( checks( sname, "line_pid_i" ) ){
		if( cmd == PS_SET ){
			ram_linefollow_i = 1.0 * (*value) / 100.0;
			return PS_SETOK;
		}
		if( cmd == PS_GET ){
			(*value) = (ram_linefollow_i * 100.0);
			return PS_GETOK;
		}
	}
	if( checks( sname, "line_pid_d" ) ){
		if( cmd == PS_SET ){
			ram_linefollow_d = 1.0 * (*value) / 100.0;
			return PS_SETOK;
		}
		if( cmd == PS_GET ){
			(*value) = (ram_linefollow_d * 100.0);
			return PS_GETOK;
		}
	}
	if( checks( sname, "reduce_mc " ) ){
		if( cmd == PS_SET ){
			ram_reduce_mc_in_plane = 1.0 * (*value) / 100.0;
			return PS_SETOK;
		}
		if( cmd == PS_GET ){
			(*value) = (ram_reduce_mc_in_plane * 100.0);
			return PS_GETOK;
		}
	}
	if( checks( sname, "altmc_p   " ) ){
		if( cmd == PS_SET ){
			ram_altmc_pid_p = 1.0 * (*value) / 100.0;
			return PS_SETOK;
		}
		if( cmd == PS_GET ){
			(*value) = (ram_altmc_pid_p * 100.0);
			return PS_GETOK;
		}
	}
	if( checks( sname, "altmc_i   " ) ){
		if( cmd == PS_SET ){
			ram_altmc_pid_i = 1.0 * (*value) / 100.0;
			return PS_SETOK;
		}
		if( cmd == PS_GET ){
			(*value) = (ram_altmc_pid_i * 100.0);
			return PS_GETOK;
		}
	}
	if( checks( sname, "altmc_d   " ) ){
		if( cmd == PS_SET ){
			ram_altmc_pid_d = 1.0 * (*value) / 100.0;
			return PS_SETOK;
		}
		if( cmd == PS_GET ){
			(*value) = (ram_altmc_pid_d * 100.0);
			return PS_GETOK;
		}
	}

//    if( checks( sname, "pid_as_p  " ) ){
//        if( cmd == PS_SET ){
//            systemConfigMutable()->cx_as_pid_p = 1.0 * (*value) / 100.0;
//            return PS_SETOK;
//        }
//        if( cmd == PS_GET ){
//            (*value) = (systemConfig()->cx_as_pid_p * 100.0);
//            return PS_GETOK;
//        }
//    }
//    if( checks( sname, "pid_as_i  " ) ){
//        if( cmd == PS_SET ){
//            systemConfigMutable()->cx_as_pid_p = 1.0 * (*value) / 100.0;
//            return PS_SETOK;
//        }
//        if( cmd == PS_GET ){
//            (*value) = (systemConfig()->cx_as_pid_i * 100.0);
//            return PS_GETOK;
//        }
//    }
//    if( checks( sname, "pid_as_d  " ) ){
//        if( cmd == PS_SET ){
//            systemConfigMutable()->cx_as_pid_p = 1.0 * (*value) / 100.0;
//            return PS_SETOK;
//        }
//        if( cmd == PS_GET ){
//            (*value) = (systemConfig()->cx_as_pid_d * 100.0);
//            return PS_GETOK;
//        }
//    }
//
//    if( checks( sname, "pid_alt_p " ) ){
//        if( cmd == PS_SET ){
//            systemConfigMutable()->cx_alt_pid_p = 1.0 * (*value) / 100.0;
//            return PS_SETOK;
//        }
//        if( cmd == PS_GET ){
//            (*value) = (systemConfig()->cx_alt_pid_p * 100.0);
//            return PS_GETOK;
//        }
//    }
//    if( checks( sname, "pid_alt_i " ) ){
//        if( cmd == PS_SET ){
//            systemConfigMutable()->cx_alt_pid_i = 1.0 * (*value) / 100.0;
//            return PS_SETOK;
//        }
//        if( cmd == PS_GET ){
//            (*value) = (systemConfig()->cx_alt_pid_i * 100.0);
//            return PS_GETOK;
//        }
//    }
//    if( checks( sname, "pid_alt_d " ) ){
//        if( cmd == PS_SET ){
//            systemConfigMutable()->cx_alt_pid_d = 1.0 * (*value) / 100.0;
//            return PS_SETOK;
//        }
//        if( cmd == PS_GET ){
//            (*value) = (systemConfig()->cx_alt_pid_d * 100.0);
//            return PS_GETOK;
//        }
//    }
//
//    if( checks( sname, "pid_nav_p " ) ){
//        if( cmd == PS_SET ){
//            systemConfigMutable()->cx_nav_pid_p = 1.0 * (*value) / 100.0;
//            return PS_SETOK;
//        }
//        if( cmd == PS_GET ){
//            (*value) = (systemConfig()->cx_nav_pid_p * 100.0);
//            return PS_GETOK;
//        }
//    }
//    if( checks( sname, "pid_nav_i " ) ){
//        if( cmd == PS_SET ){
//            systemConfigMutable()->cx_nav_pid_i = 1.0 * (*value) / 100.0;
//            return PS_SETOK;
//        }
//        if( cmd == PS_GET ){
//            (*value) = (systemConfig()->cx_nav_pid_i * 100.0);
//            return PS_GETOK;
//        }
//    }
//    if( checks( sname, "pid_nav_d " ) ){
//        if( cmd == PS_SET ){
//            systemConfigMutable()->cx_nav_pid_d = 1.0 * (*value) / 100.0;
//            return PS_SETOK;
//        }
//        if( cmd == PS_GET ){
//            (*value) = (systemConfig()->cx_nav_pid_d * 100.0);
//            return PS_GETOK;
//        }
//    }
//
//    if( checks( sname, "pid_alts_p" ) ){
//        if( cmd == PS_SET ){
//            systemConfigMutable()->cx_alt_spd_pid_p = 1.0 * (*value) / 100.0;
//            return PS_SETOK;
//        }
//        if( cmd == PS_GET ){
//            (*value) = (systemConfig()->cx_alt_spd_pid_p * 100.0);
//            return PS_GETOK;
//        }
//    }
//    if( checks( sname, "pid_alts_i" ) ){
//        if( cmd == PS_SET ){
//            systemConfigMutable()->cx_alt_spd_pid_i = 1.0 * (*value) / 100.0;
//            return PS_SETOK;
//        }
//        if( cmd == PS_GET ){
//            (*value) = (systemConfig()->cx_alt_spd_pid_i * 100.0);
//            return PS_GETOK;
//        }
//    }
//    if( checks( sname, "pid_alts_d" ) ){
//        if( cmd == PS_SET ){
//            systemConfigMutable()->cx_alt_spd_pid_d = 1.0 * (*value) / 100.0;
//            return PS_SETOK;
//        }
//        if( cmd == PS_GET ){
//            (*value) = (systemConfig()->cx_alt_spd_pid_d * 100.0);
//            return PS_GETOK;
//        }
//    }
//    if( checks( sname, "pid_lf_p  " ) ){
//        if( cmd == PS_SET ){
//            systemConfigMutable()->cx_linefollow_p = 1.0 * (*value) / 100.0;
//            return PS_SETOK;
//        }
//        if( cmd == PS_GET ){
//            (*value) = (systemConfig()->cx_linefollow_p * 100.0);
//            return PS_GETOK;
//        }
//    }
//    if( checks( sname, "laun_h_mid" ) ){
//        if( cmd == PS_SET ){
//            systemConfigMutable()->cx_launch_height_mid =  1.0 * (*value) / 100.0;
//            return PS_SETOK;
//        }
//        if( cmd == PS_GET ){
//            (*value) = (systemConfig()->cx_launch_height_mid);
//            return PS_GETOK;
//        }
//    }
//    if( checks( sname, "laun_h    " ) ){
//        if( cmd == PS_SET ){
//            systemConfigMutable()->cx_launch_height =  1.0 * (*value) / 100.0;
//            return PS_SETOK;
//        }
//        if( cmd == PS_GET ){
//            (*value) = (systemConfig()->cx_launch_height);
//            return PS_GETOK;
//        }
//    }
//    if( checks( sname, "laun_throt" ) ){
//        if( cmd == PS_SET ){
//            systemConfigMutable()->cx_launch_start_throttle =  1.0 * (*value) / 100.0;
//            return PS_SETOK;
//        }
//        if( cmd == PS_GET ){
//            (*value) = (systemConfig()->cx_launch_start_throttle);
//            return PS_GETOK;
//        }
//    }
//    if( checks( sname, "laun_end_a" ) ){
//        if( cmd == PS_SET ){
//            systemConfigMutable()->cx_launch_destangle_dd =  1.0 * (*value) / 100.0;
//            return PS_SETOK;
//        }
//        if( cmd == PS_GET ){
//            (*value) = (systemConfig()->cx_launch_destangle_dd);
//            return PS_GETOK;
//        }
//    }
//    if( checks( sname, "cruise_as " ) ){
//        if( cmd == PS_SET ){
//            systemConfigMutable()->cx_cruise_as  =  1.0 * (*value) / 100.0;
//            return PS_SETOK;
//        }
//        if( cmd == PS_GET ){
//            (*value) = (systemConfig()->cx_cruise_as );
//            return PS_GETOK;
//        }
//    }
//    if( checks( sname, "land_alts " ) ){
//        if( cmd == PS_SET ){
//            systemConfigMutable()->cx_land_speed  =  1.0 * (*value) / 100.0;
//            return PS_SETOK;
//        }
//        if( cmd == PS_GET ){
//            (*value) = (systemConfig()->cx_land_speed );
//            return PS_GETOK;
//        }
//    }

    return PS_NOTFOUND;
}

