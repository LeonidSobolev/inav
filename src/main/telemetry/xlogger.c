#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"
#include "build/version.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/maths.h"
#include "common/utils.h"
#include "common/typeconversion.h"

#include "config/feature.h"

#include "drivers/serial.h"
#include "drivers/time.h"

#include "fc/config.h"
#include "fc/fc_core.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "fc/settings.h"
#include "fc/rc_modes.h"
#include "fc/rc_adjustments.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/servos.h"

#include "io/gimbal.h"
#include "io/gps.h"
#include "io/gps_private.h"
#include "io/ledstrip.h"
#include "io/serial.h"
#include "io/beeper.h"

#include "navigation/navigation.h"
#include "navigation/navigation_private.h"
#include "navigation/apgeo.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/gyro.h"
#include "sensors/pitotmeter.h"
#include "sensors/sensors.h"
#include "sensors/diagnostics.h"
#include "sensors/rangefinder.h"

#include "telemetry/telemetry.h"
#include "telemetry/xlogger.h"

#include "navigation/icsnav.h"
#include "navigation/icsmission.h"
#include "navigation/apgeo.h"

#include "scheduler/scheduler.h"

#define TELEMETRY_XLOGGER_PORT_MODE     MODE_RXTX
#define TELEMETRY_XLOGGER_MAXRATE       50
#define TELEMETRY_XLOGGER_DELAY         ((1000 * 1000) / TELEMETRY_XLogger_MAXRATE)
#define XLOGGERBUF_TX_LEN				255
#define XLOGGERBUF_RX_LEN				32

void xlogger_send_dump_reset();
// variables


// sync this with mixerMode_e
static const char * const mixerNames[] = {
    "TRI", "QUADP", "QUADX", "BI",
    "GIMBAL", "Y6", "HEX6",
    "FLYING_WING", "Y4", "HEX6X", "OCTOX8", "OCTOFLATP", "OCTOFLATX",
    "AIRPLANE", "HELI_120_CCPM", "HELI_90_DEG", "VTAIL4",
    "HEX6H", "PPM_TO_SERVO", "DUALCOPTER", "SINGLECOPTER",
    "ATAIL4", "CUSTOM", "CUSTOMAIRPLANE", "CUSTOMTRI", NULL
};


// sync this with features_e
static const char * const featureNames[] = {
    "RX_PPM", "VBAT", "TX_PROF_SEL", "", "MOTOR_STOP",
    "SERVO_TILT", "SOFTSERIAL", "GPS", "",
    "", "TELEMETRY", "CURRENT_METER", "3D", "RX_PARALLEL_PWM",
    "RX_MSP", "RSSI_ADC", "LED_STRIP", "DASHBOARD", "",
    "BLACKBOX", "CHANNEL_FORWARDING", "TRANSPONDER", "FAILOFF",
    //"BLACKBOX", "CHANNEL_FORWARDING", "TRANSPONDER", "AIRMODE",
    "SUPEREXPO", "VTX", "RX_SPI", "", "PWM_SERVO_DRIVER", "PWM_OUTPUT_ENABLE",
    "OSD", "FW_LAUNCH", "TRACE" , "?"
};

static serialPort_t *xloggerPort = NULL;
static serialPortConfig_t *portConfig;
int32_t xloggerPort_freespace;
// freeSpace = serialTxBytesFree(blackboxPort);

static bool xloggerTelemetryEnabled =  false;
static portSharing_e xloggerPortSharing;

xlogger_state cur_xlogger_status;
x_sens_state cur_xlog_sens;
uint8_t xloggerbuf[XLOGGERBUF_TX_LEN];
int f_value;
uint8_t is_logger_init = 0;
uint8_t need_send_launch = 0;
uint8_t send_launch_result = 0;
timeUs_t last_logger_message_time;
uint8_t msg_type;
uint8_t msg_type_buff = 0;
uint32_t blocks_sent_last_time = 0;
x_PID_state cur_PID;
x_Flags_state cur_Flags;
x_Payload_state cur_Payload;
bool gtmsg_pass_enable = 0;
uint8_t gtmsg_pass_blocks = 0;

uint32_t dump_current_setting = 0;

x_dump_counters dump_send_state;

uint32_t xlogger_input_char_count = 0;
uint8_t xlogger_input_msgbuf[XLOGGERBUF_RX_LEN];
uint8_t xlogger_input_state = 0;
uint32_t xlogger_input_msglen = 0;
uint32_t xlogger_input_msgcurlen = 0;
uint8_t xlogger_input_c1, xlogger_input_c2;

// xlogger online getter
bool isXloggerOnline(timeUs_t currentTimeUs) {
	cur_xlogger_status.wasXloggerAlive = cur_xlogger_status.isXLoggerAlive;
	// check timeout
	cur_xlogger_status.periodAfterLastXLoggerHeartbeatUs = currentTimeUs - cur_xlogger_status.lastXLoggerHeartbeatTimeUs;
	uint32_t temp = (systemConfig()->cx_xlogger_heartbeat_period_us);
	if(cur_xlogger_status.periodAfterLastXLoggerHeartbeatUs > temp) {
		// there we reset status and log initial info.
		cur_xlogger_status.isXLoggerAlive = 0;
		if(cur_xlogger_status.wasXloggerAlive == 0) {
			// reser send flag
			cur_xlogger_status.needSendInit = 1;
			xlogger_send_dump_reset();
		}
	}
	// return result
	return cur_xlogger_status.isXLoggerAlive;
}


// fill functions
void int32_fill( int offset, int value ){
	for( uint8_t i = 0; i < 4; i++){
		xloggerbuf[i+offset] = (value >> i*8) & 0xFF;
	}
}

void uint32_fill( int offset, uint32_t value ){
	for( uint8_t i = 0; i < 4; i++){
		xloggerbuf[i+offset] = (value >> i*8) & 0xFF;
	}
}

void int16_fill( int offset, int16_t value ){
	for( uint8_t i = 0; i < 2; i++){
		xloggerbuf[i+offset] = (value >> i*8) & 0xFF;
	}
}

void uint16_fill( uint8_t offset, uint16_t value ){
	for( uint8_t i = 0; i < 2; i++){
		xloggerbuf[i+offset] = (value >> i*8) & 0xFF;
	}
}

void float_fill( int offset, float value ){
	f_value = (int)(value * 100);
	for( uint8_t i = 0; i < 4; i++){
		xloggerbuf[i+offset] = (f_value >> i*8) & 0xFF;
	}
}

void bfill( uint8_t offset, void * value, uint8_t size){
	// пока не совсем пон€л, что делает эта функци€
	for( uint8_t i = 0; i < size; i++){
		xloggerbuf[i+offset] = ((uint8_t *)value)[i];
	}
}

//XLogger port fucntions
void freeXLoggerTelemetryPort(void)
{
    closeSerialPort(xloggerPort);
    xloggerPort = NULL;
    xloggerTelemetryEnabled = false;
}

void configureXLoggerTelemetryPort(void)
{
    if (!portConfig) {
        return;
    }

    baudRate_e baudRateIndex = portConfig->telemetry_baudrateIndex;
    if (baudRateIndex == BAUD_AUTO) {
        // default rate for minimOSD
        baudRateIndex = BAUD_57600;
    }

    xloggerPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_XLOGGER, NULL, NULL, baudRates[baudRateIndex], TELEMETRY_XLOGGER_PORT_MODE, SERIAL_NOT_INVERTED);

    if (!xloggerPort) {
        return;
    }

    xloggerTelemetryEnabled = true;
}

void checkXLoggerTelemetryState(void)
{
    bool newTelemetryEnabledValue = telemetryDetermineEnabledState(xloggerPortSharing);

    if (newTelemetryEnabledValue == xloggerTelemetryEnabled) {
        return;
    }

    if (newTelemetryEnabledValue)
        configureXLoggerTelemetryPort();
    else
        freeXLoggerTelemetryPort();
}

// buffer service functions
void xloggerbuf_clear (uint8_t block_count) {
	// TODO: очистка мемсетом до HEADER_LEN+BLOCK_LEN*BLOCK_COUNT+в конце что-там
    for( int i = 5; i < 10 + block_count*16; i++ ){
        xloggerbuf[i] = 0;
    }
}

void xlogger_send_message( uint8_t block_count ){
	// block = 16 bytes

	//if(xloggerPort_freespace > 6+16*block_count) {

		xloggerbuf[0] = 0x67;
		xloggerbuf[1] = 0x41;
		xloggerbuf[2] = 1;
		xloggerbuf[3] = block_count;

		xloggerbuf[4+16*block_count] = 0;
		xloggerbuf[5+16*block_count] = 0;
		for( int i = 0; i < 4+16*block_count; i++ ){
			xloggerbuf[4+16*block_count] = xloggerbuf[4+16*block_count] + xloggerbuf[i];
			xloggerbuf[5+16*block_count] = xloggerbuf[5+16*block_count] + xloggerbuf[4+16*block_count];
		}

		for (int i = 0; i < 6+16*block_count; i++) {
			serialWrite(xloggerPort, xloggerbuf[i]);	// TODO: попробовать serialWriteBuf(serialPort_t *instance, const uint8_t *data, int count)
		}
		blocks_sent_last_time = block_count;
	//}
}

//xlogger message functions

void xlogger_send_init(){
	xloggerbuf_clear(1);
    xloggerbuf[4] = 0xB0;
    int32_fill(5, last_logger_message_time/1000);
    int32_fill(9, 0xFFFF);
    int32_fill(13, 0);
    int32_fill(17, 1);

    cur_xlogger_status.needSendInitialInfoPocket = 0;
    xlogger_send_message(1);
}



void xlogger_send_0xD0(){
	// cur_pos
    xloggerbuf[4] = 0xD0;
    xloggerbuf_clear(5);

    uint32_fill(5, last_logger_message_time/1000);
    // TODO: fix time type - int32 or uint32

    int32_fill(9, cur_pos.la);
    int32_fill(13, cur_pos.lo);
    int32_fill(17, cur_pos.h);
    float_fill(21, cur_pos.az);
    float_fill(25, cur_pos.mag_az);

    uint16_fill(29, cur_pos.gndspd);
    uint16_fill(31, cur_pos.airspd);
    uint16_fill(33, cur_pos.cruise_spd);
    uint16_fill(35, cur_pos.used_spd);

    xloggerbuf[37] = cur_pos.is_use_airspeed;

    uint16_fill(38, cur_pos.need_az);


    int32_fill(40, cur_pos.need_h);
    uint16_fill(44, cur_pos.need_roll);
    uint16_fill(46, cur_pos.need_pitch);
    uint16_fill(48, cur_pos.need_nu);
    xloggerbuf[50] = cur_pos.manual_modes;
    	uint8_t fmode = 0;

        if (FLIGHT_MODE(MANUAL_MODE)) fmode = 1;

        if (FLIGHT_MODE(FAILSAFE_MODE)) fmode |= 2;

        if (FLIGHT_MODE(NAV_RTH_MODE)) fmode |= 4;

        if (FLIGHT_MODE(NAV_POSHOLD_MODE)) fmode |= 8;

        if (FLIGHT_MODE(NAV_WP_MODE)) fmode |= 16;

        if (FLIGHT_MODE(NAV_ALTHOLD_MODE)) fmode |= 32;

        if (FLIGHT_MODE(ANGLE_MODE)) fmode |= 64;

        if (FLIGHT_MODE(NAV_LAUNCH_MODE)) fmode |= 128;
        // add fmode later
    xloggerbuf[51] = fmode;
    float_fill(52, cur_pos.drift_area);
    uint16_fill(56, XM_Current);


    xlogger_send_message(5);
}

void xlogger_send_0xD1() {
	//Motors
	uint8_t blocks = (2*getMotorCount()+4);
	if(blocks % 16 > 0) blocks=blocks/16 + 1;
	else blocks = blocks/16;

	xloggerbuf[4] = 0xD1;
	xloggerbuf_clear(blocks);
	uint32_fill(5, last_logger_message_time/1000);
	for (uint8_t i = 0; i < getMotorCount(); i++) {
		int16_fill(9+i*2, motor[i]);
	}


	xlogger_send_message(blocks);
}

void xlogger_send_0xD2() {
	//servos. active only if we have servos
	if(isServoOutputEnabled() && isMixerUsingServos()) {
		xloggerbuf[4] = 0xD2;
		xloggerbuf_clear(2);
		uint32_fill(5,last_logger_message_time/1000);

		for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
			//target = getCurrentServoMixer(i).targetChannel;	// где-то тут ошибка
			int16_fill(9+i*2, servo[i]);
			//if(target > targetMax) targetMax = target;
		}
		xlogger_send_message(2);
	}
}



void xlogger_send_0x62() {
	// RCcontrols
	//TODO: проверить работу blocks

	uint8_t blocks = (2*systemConfig()->cx_xlogger_rcData_channels + 4 + 4*2);
	if(blocks % 16 > 0) blocks=blocks/16 + 1;
	else blocks = blocks/16;

	xloggerbuf[4] = 0x62;
	xloggerbuf_clear(blocks);
	uint32_fill(5, last_logger_message_time/1000);

	for (uint8_t i = 0; i < 4; i++) {
		int16_fill(9+i*2, rcCommand[i]);
	}
	for (uint8_t i = 0; i < systemConfig()->cx_xlogger_rcData_channels; i++) {
		int16_fill(17+i*2, rcData[i]);
	}
	xlogger_send_message(blocks);
}

void xlogger_send_0x63() {
	// GPS

		xloggerbuf[4] = 0x63;
		xloggerbuf_clear(3);
		uint32_fill(5, last_logger_message_time/1000);

		uint16_fill(9, gpsSol.fixType);
		xloggerbuf[11] = gpsSol.numSat;
		uint16_fill(12, gpsSol.hdop);
		uint16_fill(14, gpsSol.eph);
		uint16_fill(16, gpsSol.epv);
		int16_fill(18, gpsSol.groundCourse);
		int32_fill(20, gpsSol.llh.lat);
		int32_fill(24, gpsSol.llh.lon);
		int32_fill(28, gpsSol.llh.alt);
		int16_fill(32, gpsSol.groundSpeed);

		uint16_fill(34,   gpsSol.time.year);
		xloggerbuf[36] = gpsSol.time.month;
		xloggerbuf[37] = gpsSol.time.day;
		xloggerbuf[38] = gpsSol.time.hours;
		xloggerbuf[39] = gpsSol.time.minutes;
		xloggerbuf[40] = gpsSol.time.seconds;
		uint16_fill(41,  gpsSol.time.millis);

		int32_fill(45, getTotalTravelDistance());

		xlogger_send_message(3);
		gpsSol.xlogger_flags.isnew = 0;


}
void xlogger_send_0x64(){
	// параметры с датчиков
	xloggerbuf_clear(4);
	xloggerbuf[4] = 0x64;
	    uint32_fill(5, last_logger_message_time/1000);
	    int32_fill(9, baroGetLatestAltitude());
	    if (sensors(SENSOR_PITOT))  int32_fill(13, pitot.airSpeed);
	    if (feature(FEATURE_CURRENT_METER)) {
	    	uint16_fill(17, getAmperageLatestADC());
	    	int32_fill (19, getAmperage());
	    	int32_fill (31, getMAhDrawn());
	    }
	    if (feature(FEATURE_VBAT)) {
	    	uint16_fill(23, getBatteryVoltageLatestADC());
	    	uint16_fill(25, getBatteryVoltage());
	    }
	    if (feature(FEATURE_VBAT) && feature(FEATURE_CURRENT_METER)) {
	    	int32_fill(27,	getPower());
	    	int32_fill(35,	getMWhDrawn());
	    }
	    uint16_fill(39,	averageSystemLoadPercent);

		for( uint8_t i = 0; i < XYZ_AXIS_COUNT; i++) {
				float_fill(41+i*4, (float)cur_xlog_sens.gyroADCRaw[i]);
				float_fill(53+i*4, acc.accADCf[i]);
		}
		//mti_angles
		int16_fill(65, attitude.values.roll);
		int16_fill(67, attitude.values.pitch);
		int16_fill(69, attitude.values.yaw);
		//mpu angles
		int16_fill(83, attitude.values.roll);
		int16_fill(87, attitude.values.pitch);
		int16_fill(91, attitude.values.yaw);
		//rangefinder
		int16_fill(95, rangefinder.rawAltitude);
		int16_fill(97, (uint16_t)(rangefinder.x_land_detector_time * 1000));
		xloggerbuf[99] = calculateBatteryPercentage();

		uint32_fill(112, getBatteryRemainingCapacity());

	    xlogger_send_message(7);	// вызвать от количества блоков
}


void xlogger_send_0x65() {
	// система стабилизации

		xloggerbuf_clear(8);
		xloggerbuf[4] = 0x65;
		uint32_fill(5, last_logger_message_time/1000);

		for( uint8_t axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
			float_fill(9 +axis*4, cur_PID.inav_rate.Rate[axis]);
			float_fill(21+axis*4, cur_PID.inav_rate.Target[axis]);
			float_fill(33+axis*4, cur_PID.inav_rate.Error[axis]);
			float_fill(45+axis*4, cur_PID.inav_rate.Pterm[axis]);
			float_fill(57+axis*4, cur_PID.inav_rate.Iterm[axis]);
			float_fill(69+axis*4, cur_PID.inav_rate.Dterm[axis]);
			float_fill(81+axis*4, cur_PID.inav_rate.PIDres[axis]);
			//if(FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) float_fill(69+i*2, cur_PID.inav_SS.angleTarget[i]);	//write angle tgt only if we calculate it
			float_fill(93+axis*4, cur_PID.inav_angle.Target[axis]);
			float_fill(105+axis*4, cur_PID.inav_angle.Error[axis]);
		}

		xlogger_send_message(8);
		if(cur_PID.inav_rate.newData) cur_PID.inav_rate.newData = 0;
		if(cur_PID.inav_angle.newData) cur_PID.inav_angle.newData = 0;

}


void xlogger_send_0x66() {
	// навигационные пиды 1 - уровень газа
	//if (cur_PID.flight_thr.new) {
		xloggerbuf_clear(4);
		xloggerbuf[4] = 0x66;
		uint32_fill (5,  cur_PID.flight_thr.curTime);
		uint32_fill (9,  cur_PID.flight_thr.need_alt);
		float_fill  (13, cur_PID.flight_thr.alt_err_tho);
		float_fill  (17, cur_PID.flight_thr.cx_alt_t_p_cont);
		float_fill  (21, cur_PID.flight_thr.cx_alt_t_i_cont);
		float_fill  (25, cur_PID.flight_thr.cx_alt_t_d_cont);
		float_fill  (29, cur_PID.flight_thr.throttle_alt_res);
		int16_fill  (33, cur_PID.flight_thr.need_as_in_cm);
		float_fill  (35, cur_PID.flight_thr.cur_speed_err);
		float_fill  (39, cur_PID.flight_thr.cx_as_pid_p_cont);
		float_fill  (43, cur_PID.flight_thr.cx_as_pid_i_cont);
		float_fill  (47, cur_PID.flight_thr.cx_as_pid_d_cont);
		float_fill  (51, cur_PID.flight_thr.throttle_spd_res);
		float_fill  (55, cur_PID.flight_thr.throttle_res);
		float_fill  (59, cur_PID.flight_thr.vbat_compensation_factor);

		xlogger_send_message(4);

		cur_PID.flight_thr.new = 0;
	//}


}

void xlogger_send_0x67() {
	// навигационные пиды 2 - посхолд

		xloggerbuf_clear(3);
		xloggerbuf[4] = 0x67;
		uint32_fill (5,  cur_PID.poshold.curTime);
		int32_fill  (9,  cur_PID.poshold.az_to_point);
		int16_fill  (13, cur_PID.poshold.poshold_direction);
		uint32_fill (15, cur_PID.poshold.dist_to_origin);
		float_fill  (19, cur_PID.poshold.cx_pr_p_cont);
		float_fill  (23, cur_PID.poshold.cx_pr_i_cont);
		float_fill  (27, cur_PID.poshold.cx_pr_d_cont);
		float_fill  (31, cur_PID.poshold.cx_ph_p_cont);
		float_fill  (35, cur_PID.poshold.cx_ph_i_cont);
		float_fill  (39, cur_PID.poshold.cx_ph_d_cont);
		float_fill  (43, cur_PID.poshold.vx_angle);
		float_fill  (47, cur_PID.poshold.vy_angle);

		xlogger_send_message(3);

		cur_PID.poshold.new = 0;


}

void xlogger_send_0x68() {
	// навигационные пиды 3 - высотный контроллер при посадке
	//if (cur_PID.landing.new) {
		xloggerbuf_clear(2);
		xloggerbuf[4] = 0x68;
		int32_fill (5,  cur_PID.landing.curTime);


		int16_fill (9,  cur_PID.landing.need_alt_speed);
		float_fill (13, cur_PID.landing.cur_alts_err);
		float_fill (17, cur_PID.landing.cx_alt_spd_pid_p_cont);
		float_fill (21, cur_PID.landing.cx_alt_spd_pid_i_cont);
		float_fill (25, cur_PID.landing.cx_alt_spd_pid_d_cont);
		float_fill (29, cur_PID.landing.alts_res);

		xlogger_send_message(2);

		cur_PID.landing.new = 0;
	//}

}

void xlogger_send_0x69() {
	// навигационные пиды 4 - высотный контроллер
	//if (cur_PID.flight_alt.new) {
		xloggerbuf_clear(3);
		xloggerbuf[4] = 0x69;
		int32_fill (5,  cur_PID.flight_alt.curTime);

		float_fill (9,	cur_PID.flight_alt.need_alt);
		float_fill (13, cur_PID.flight_alt.cur_alt_err);
		float_fill (17, cur_PID.flight_alt.cx_alt_pid_p_cont);
		float_fill (21, cur_PID.flight_alt.cx_alt_pid_i_cont);
		float_fill (25, cur_PID.flight_alt.cx_alt_pid_d_cont);
		float_fill (29, cur_PID.flight_alt.alt_res);
		float_fill (33, cur_PID.flight_alt.max_climb_angle);
		float_fill (37, cur_PID.flight_alt.max_dive_angle);

		xlogger_send_message(3);
		cur_PID.flight_alt.new = 0;
	//}
}

void xlogger_send_0x6A() {
	// навигационные пиды 5 - курс
	//if (cur_PID.flight_azimuth.new) {
		xloggerbuf_clear(4);
		xloggerbuf[4] = 0x6A;
		int32_fill (5,  cur_PID.flight_azimuth.curTime);

		float_fill (9,	cur_PID.flight_azimuth.dist);
		float_fill (13, cur_PID.flight_azimuth.cx_linefollow_p_cont);
		float_fill (17, cur_PID.flight_azimuth.cx_linefollow_i_cont);
		float_fill (21, cur_PID.flight_azimuth.cx_linefollow_d_cont);
		float_fill (25, cur_PID.flight_azimuth.nu_on_line);
		float_fill (29, cur_PID.flight_azimuth.need_course);
		float_fill (33, cur_PID.flight_azimuth.need_az);
		float_fill (37, cur_PID.flight_azimuth.cx_nav_pid_p_cont);
		float_fill (41, cur_PID.flight_azimuth.cx_nav_pid_i_cont);
		float_fill (45, cur_PID.flight_azimuth.cx_nav_pid_d_cont);
		int16_fill (49, cur_PID.flight_azimuth.cur_az_err);
		float_fill (51, cur_PID.flight_azimuth.az_res);

		xlogger_send_message(4);
		cur_PID.flight_azimuth.new = 0;
	//}

}

void xlogger_send_0x6B() {
	// навигационные пиды 5 - курс
	//if (cur_PID.flight_azimuth.new) {
	uint8_t msg_len = (MAX_XLOGGER_DEBUG*4 + 4)/16 + 1;
	xloggerbuf_clear(msg_len);
	xloggerbuf[4] = 0x6B;
	int32_fill (5,  cur_Flags.Debug.curTime);

	for (uint8_t i=0; i< MAX_XLOGGER_DEBUG; i++) {
		float_fill(9+i*4, cur_Flags.Debug.arr[i]);
	}

	xlogger_send_message(msg_len);
	cur_Flags.Debug.newData = 0;
	//}

}

void xlogger_send_0x6C() {
	// new_poshold

		xloggerbuf_clear(6);
		xloggerbuf[4] = 0x6C;
		uint32_fill (5,  cur_PID.new_poshold.curTime);
		int32_fill 	(9,  cur_PID.new_poshold.coord_az2pt);
		int16_fill	(13, cur_PID.new_poshold.coord_azErr);
		float_fill	(15, cur_PID.new_poshold.coord_d2orig);

		float_fill	(19, cur_PID.new_poshold.speed_x_tgt);
		float_fill	(23, cur_PID.new_poshold.speed_x_val);
		float_fill	(27, cur_PID.new_poshold.speed_x_p_cont);
		float_fill	(31, cur_PID.new_poshold.speed_x_i_cont);
		float_fill	(35, cur_PID.new_poshold.speed_x_d_cont);
		float_fill	(39, cur_PID.new_poshold.speed_x_pid_out);

		float_fill	(43, cur_PID.new_poshold.speed_y_tgt);
		float_fill	(47, cur_PID.new_poshold.speed_y_val);
		float_fill	(51, cur_PID.new_poshold.speed_y_p_cont);
		float_fill	(55, cur_PID.new_poshold.speed_y_i_cont);
		float_fill	(59, cur_PID.new_poshold.speed_y_d_cont);
		float_fill	(63, cur_PID.new_poshold.speed_y_pid_out);

		float_fill	(67, cur_PID.new_poshold.coord_x_tgt);
		float_fill	(71, cur_PID.new_poshold.coord_x_now);
		float_fill	(75, cur_PID.new_poshold.coord_y_tgt);
		float_fill	(79, cur_PID.new_poshold.coord_y_now);

		xlogger_send_message(6);

		cur_PID.new_poshold.new = 0;


}

void xlogger_send_0x6D() {
	// INAV POS ESTIMATOR
		xloggerbuf_clear(6);
		xloggerbuf[4] = 0x6D;
		uint32_fill (5,  millis());

		float_fill (9,	posEstimator.est.pos.x);
		float_fill (13,	posEstimator.est.pos.y);
		float_fill (17,	posEstimator.est.pos.z);
		float_fill (21,	posEstimator.est.vel.x);
		float_fill (25,	posEstimator.est.vel.y);
		float_fill (29,	posEstimator.est.vel.z);
		float_fill (33,	posEstimator.est.eph);
		float_fill (37,	posEstimator.est.epv);

		float_fill (41, posEstimator.imu.accelNEU.x);
		float_fill (45, posEstimator.imu.accelNEU.y);
		float_fill (49, posEstimator.imu.accelNEU.z);

		float_fill (53, posEstimator.baro.alt);

		float_fill (57, posEstimator.gps.pos.x);
		float_fill (61, posEstimator.gps.pos.y);
		float_fill (65, posEstimator.gps.pos.z);
		float_fill (69, posEstimator.gps.vel.x);
		float_fill (73, posEstimator.gps.vel.y);
		float_fill (77, posEstimator.gps.vel.z);
		float_fill (81, posEstimator.gps.eph);
		float_fill (85, posEstimator.gps.epv);


		xlogger_send_message(6);
}

void xlogger_send_0x30() {
	// асинхронные флаги INAV
	xloggerbuf_clear(2);
	xloggerbuf[4] = 0x30;
	int32_fill (5,  cur_Flags.INAV_flags.curTime);

	uint32_fill(9,   cur_Flags.INAV_flags.flightModeFlags);
	uint32_fill(13,  cur_Flags.INAV_flags.stateFlags);
	uint32_fill(17,  cur_Flags.INAV_flags.hwHealthStatus);
	xloggerbuf[21] = cur_Flags.INAV_flags.failsafePhase;
	xloggerbuf[22] = cur_Flags.INAV_flags.receiver_status;


	xlogger_send_message(2);
	cur_Flags.INAV_flags.newData = 0;

}

void xlogger_send_0x31() {
	// асинхронные флаги ics
	xloggerbuf_clear(1);
	xloggerbuf[4] = 0x31;
	int32_fill (5,  cur_Flags.CX_flags.curTime);

	xloggerbuf[9] = cur_Flags.CX_flags.cx_navstate;
	xloggerbuf[10] = cur_Flags.CX_flags.cx_polygon_state;
	uint16_fill(11,	 cur_Flags.CX_flags.cx_point_hdg);
	uint32_fill(13,  cur_Flags.CX_flags.warnings);
	xloggerbuf[17] = cur_Flags.CX_flags.manual_modes;

	xlogger_send_message(1);
	cur_Flags.CX_flags.newData = 0;

}

void xlogger_send_0x32() {
	xloggerbuf_clear(1);
	xloggerbuf[4] = 0x32;
	uint32_fill(5, last_logger_message_time/1000);
	uint32_fill(9, cur_Payload.photo.cx_photo_call);
	uint32_fill(13, cur_Payload.photo.cx_photo_answer);
	uint16_fill(17, cur_Payload.photo.cx_photo_passport_num);

	xlogger_send_message(1);
	cur_Payload.photo.newData = 0;
}


void xlogger_send_0xB3() {
	xloggerbuf_clear(1);
	xloggerbuf[4] = 0xB3;
	uint32_fill(5, cur_Flags.ARM.curTime);
	xloggerbuf[9] = cur_Flags.ARM.arming_result;
	uint32_fill(10, cur_Flags.ARM.armingFlags);

	xlogger_send_message(1);
	cur_Flags.ARM.newData = 0;
}

void xlogger_send_launch_start(){
	xloggerbuf_clear(3);
    xloggerbuf[4] = 0xB1;

    uint32_fill(5, cur_Flags.Launch.launch_time);
    uint32_fill(9, 0);
    uint32_fill(13, 0);
    uint32_fill(17, 0);
    uint16_fill(21, systemConfig()->cx_version);
    uint16_fill(23, 1);
    uint16_fill(25, 0);
    xloggerbuf[27] = 0;
    xloggerbuf[28] = cur_Flags.Launch.result;

	uint16_fill(29,   cur_Flags.Launch.launch_time_UTC.year);
	xloggerbuf[31] = cur_Flags.Launch.launch_time_UTC.month;
	xloggerbuf[32] = cur_Flags.Launch.launch_time_UTC.day;
	xloggerbuf[33] = cur_Flags.Launch.launch_time_UTC.hours;
	xloggerbuf[34] = cur_Flags.Launch.launch_time_UTC.minutes;
	xloggerbuf[35] = cur_Flags.Launch.launch_time_UTC.seconds;
	uint16_fill(36,  cur_Flags.Launch.launch_time_UTC.millis);

    xlogger_send_message(3);
    cur_Flags.Launch.newData = 0;
}

void xlogger_send_landed(){
	xloggerbuf_clear(2);
	xloggerbuf[4] = 0xB2;

    uint32_fill(5,  last_logger_message_time/1000);
    uint16_fill(9,  0);
    uint32_fill(11, cur_Flags.Landed.flown_ms);

	uint16_fill(15,   cur_Flags.Landed.landed_time_UTC.year);
	xloggerbuf[17] = cur_Flags.Landed.landed_time_UTC.month;
	xloggerbuf[18] = cur_Flags.Landed.landed_time_UTC.day;
	xloggerbuf[19] = cur_Flags.Landed.landed_time_UTC.hours;
	xloggerbuf[20] = cur_Flags.Landed.landed_time_UTC.minutes;
	xloggerbuf[21] = cur_Flags.Landed.landed_time_UTC.seconds;
	uint16_fill(22,  cur_Flags.Landed.landed_time_UTC.millis);

    xlogger_send_message(2);
    cur_Flags.Landed.newData = 0;
}

// send log start UTC time


void xlogger_send_UTC_time(){

	xloggerbuf_clear(1);
	xloggerbuf[4] = 0xB8;

	uint32_fill(5,  last_logger_message_time/1000);

	uint16_fill(9,   cur_Flags.UTCTime.dt.year);
	xloggerbuf[11] = cur_Flags.UTCTime.dt.month;
	xloggerbuf[12] = cur_Flags.UTCTime.dt.day;
	xloggerbuf[13] = cur_Flags.UTCTime.dt.hours;
	xloggerbuf[14] = cur_Flags.UTCTime.dt.minutes;
	xloggerbuf[15] = cur_Flags.UTCTime.dt.seconds;
	uint16_fill(16,  cur_Flags.UTCTime.dt.millis);

	xlogger_send_message(1);
	cur_Flags.UTCTime.newData = 0;
}

/*---------------------- xlogger send dump functions -----------*/
void xlogger_send_string(char *strr, uint8_t length, uint8_t header) {
	// max string length is 255 bytes! more than enough!
	//uint8_t length = sizeof(strr)/sizeof(char);
	if(length<255) {
		uint8_t blocks = length + 6;
		if(blocks % 16 > 0) blocks=blocks/16 + 1;
		else blocks = blocks/16;
		xloggerbuf_clear(blocks);
		xloggerbuf[4] = header;
		uint32_fill(5, last_logger_message_time/1000);
		xloggerbuf[9] = 255;
		xloggerbuf[10] = length;
		for(uint8_t bt = 0; bt < length; bt ++) {
			xloggerbuf[bt + 11] = strr[bt];
		}

		xlogger_send_message(blocks);
	}
}

void xlogger_send_dump_value(const setting_t *setting) {

	char *format_int = "set %s = %i";
	char *format_str = "set %s = %s";
	uint8_t str_len = SETTING_MAX_NAME_LENGTH + 40;
	char str_buf[SETTING_MAX_NAME_LENGTH + 40] = {0};
	char val_buf[FTOA_BUFFER_SIZE] = {0};
	char nam_buf[SETTING_MAX_NAME_LENGTH] = {0};


	const void *valuePointer = setting_get_value_pointer(setting);
	uint8_t setting_type = SETTING_TYPE(setting);

	setting_get_name(setting, nam_buf);
	// data output
	uint8_t val_pos = SETTING_MAX_NAME_LENGTH+11;	// 7
	uint8_t data_len = 0;
	int32_t value = 0;
	float f_value = 0.0f;

	switch (setting_type) {
	case VAR_UINT8:
		data_len = 1;
		value = *(uint8_t *)valuePointer;
		break;
	case VAR_INT8:
		data_len = 1;
		value = *(int8_t *)valuePointer;
		break;
	case VAR_UINT16:
		data_len = 2;
		value = *(uint16_t *)valuePointer;
		break;
	case VAR_INT16:
		data_len = 2;
		value =  *(int16_t *)valuePointer;
		break;
	case VAR_UINT32:
		data_len = 4;
		value = *(uint32_t *)valuePointer;
		break;
	case VAR_FLOAT:
		data_len = 4;
		f_value =  *(float *)valuePointer;
		break;
	}
	switch (SETTING_MODE(setting)) {
	    case MODE_DIRECT:
	    	if(setting_type == VAR_FLOAT) {
	    		sprintf(val_buf, "%s", ftoa(f_value, val_buf));
	    	}
	    	else {
	    		sprintf(val_buf, "%i", value);
	    	}

	        break;
	    case MODE_LOOKUP:
	    	// length of string. max len is 200 - more than enough

	    	for(uint8_t ii=0; ii < 100; ii++) {
	    		if(settingLookupTables[setting->config.lookup.tableIndex].values[value][ii] == 0) {
	    			// по факту, мы провер€ем указатель на массив
	    			// любопытный факт: в пам€ти между элементами массива лежит 0, таким образом можно определить:
	    			// начальный элемент массива (указатель)
	    			// конечный элемент массива (0)
	    			data_len = ii;
	    			break;
	    		}
	    		val_buf[ii] = settingLookupTables[setting->config.lookup.tableIndex].values[value][ii];
	    	}
	        break;
	    }

	sprintf(str_buf, format_str, nam_buf, val_buf);
	xlogger_send_string(str_buf,str_len,0x07);
}

void xlogger_send_dump_reset_settings_counter () {
	if(dump_send_state.settings_counter == SETTINGS_TABLE_COUNT) {
			// hard reset counter
		dump_send_state.settings_counter = 0;
	}
}
void xloger_send_dump_all_values(uint16_t valueSection, uint32_t counter) {
	const setting_t *setting = &settingsTable[counter];
	// master and profile values
	if(SETTINGS_TABLE_COUNT >= counter) {
		if (SETTING_SECTION(setting) == valueSection){
			xlogger_send_dump_value(setting);
		}
		else blocks_sent_last_time = 0;	// по идее, это должно делать переход к следующему шагу с минимальной задержкой. Ќужна проверка!
	}
	dump_send_state.settings_counter++;	// counter
}

void xlogger_send_dump_profile() {
	xlogger_send_dump_reset_settings_counter();
	if(dump_send_state.profiles.newDataHeader){
		xlogger_send_string("# profile", 15, 0x07);
		dump_send_state.profiles.newDataHeader = 0;
	}
	else if(dump_send_state.profiles.newDataProfileNumber){
		char PN[10] = {0,0,0,0,0,0,0,0,0,0};
		sprintf(PN,"profile %d", dump_send_state.profiles.profile_number + 1);
		xlogger_send_string(PN, 15, 0x07);
		dump_send_state.profiles.newDataProfileNumber = 0;
	}
	else if(dump_send_state.profiles.newDataProfiles){
		xloger_send_dump_all_values(PROFILE_VALUE, dump_send_state.settings_counter);
		if(dump_send_state.settings_counter == SETTINGS_TABLE_COUNT) dump_send_state.profiles.newDataProfiles = 0;
	}
	else if(dump_send_state.profiles.newDataRate){
		xloger_send_dump_all_values(CONTROL_RATE_VALUE, dump_send_state.settings_counter);
		if(dump_send_state.settings_counter == SETTINGS_TABLE_COUNT) dump_send_state.profiles.newDataRate = 0;
	}

	else dump_send_state.profiles.newData = 0;
}


void xlogger_send_dump_master() {
	xlogger_send_dump_reset_settings_counter();
	if(dump_send_state.master.newDataHeader){
		xlogger_send_string("# master", 8, 0x07);
		dump_send_state.master.newDataHeader = 0;
	}
	else if(dump_send_state.master.newDataMaster){
		xloger_send_dump_all_values(MASTER_VALUE, dump_send_state.settings_counter);
		if(dump_send_state.settings_counter == SETTINGS_TABLE_COUNT) dump_send_state.master.newDataMaster = 0;
	}
	else dump_send_state.master.newData = 0;
}

bool xlogger_send_dump_mixer(uint8_t mot, char *s ) {
	// returns 1 if we have finished
	const motorMixer_t *motorMixer = customMotorMixer(0);
	const char *format_mixer = "mmix %d %s %s %s %s";
			//str_len = FTOA_BUFFER_SIZE;
	char buf0[FTOA_BUFFER_SIZE];
	char buf1[FTOA_BUFFER_SIZE];
	char buf2[FTOA_BUFFER_SIZE];
	char buf3[FTOA_BUFFER_SIZE];

	const float thr = motorMixer[mot].throttle;
	const float roll = motorMixer[mot].roll;
	const float pitch = motorMixer[mot].pitch;
	const float yaw = motorMixer[mot].yaw;
	sprintf(s, format_mixer,
			mot,
			ftoa(thr, buf0),
			ftoa(roll, buf1),
			ftoa(pitch, buf2),
			ftoa(yaw, buf3));
	dump_send_state.additional.motorMixerCounter ++;
	if((dump_send_state.additional.motorMixerCounter > MAX_SUPPORTED_MOTORS) || (motorMixer[mot].throttle == 0.0f) ) return 1;
	else return 0;
}

bool xlogger_send_dump_smixer(uint8_t smix, char *s ) {
	// returns 1 if we have finished
	const char *format = "smix %d %d %d %d %d";
	const servoMixer_t *servoMixers = customServoMixers(0);
	const servoMixer_t customServoMixer = servoMixers[smix];
	sprintf(s, format,
		smix,
		customServoMixer.targetChannel,
		customServoMixer.inputSource,
		customServoMixer.rate,
		customServoMixer.speed);

	dump_send_state.additional.servoMixerCounter ++;
	if((dump_send_state.additional.servoMixerCounter > MAX_SERVO_RULES) || (customServoMixer.rate == 0)) return 1;
	else return 0;
}

bool xlogger_send_dump_servo(uint8_t servo, char *s ) {
	// returns 1 if we have finished
	servoParam_t *servoParam = servoParams(0);
	char *format = "servo %u %d %d %d %d %d ";
	servoParam_t *servoConf = &servoParam[servo];

	sprintf(s, format,
			servo,
	        servoConf->min,
	        servoConf->max,
	        servoConf->middle,
	        servoConf->rate,
	        servoConf->forwardFromChannel);

	dump_send_state.additional.servoCounter ++;
	if((dump_send_state.additional.servoCounter > MAX_SUPPORTED_SERVOS) ) return 1;
	else return 0;
}

bool xlogger_send_feature(uint32_t feat_num, char *s) {
	// inav resets all to copy
	// wo do NOT
	const featureConfig_t *featureConfig_v = featureConfig();
	uint32_t mask = featureConfig_v->enabledFeatures;

	if( featureNames[feat_num][0] == '\0') {
		dump_send_state.additional.featureCounter ++;
		return 0;
	}
	//const char format[40] = {0};
	if (mask & (1 << feat_num))	sprintf(s, "feature %s", featureNames[feat_num]);
	else 						sprintf(s, "feature -%s", featureNames[feat_num]);



	dump_send_state.additional.featureCounter ++;
	if( featureNames[feat_num] == "?" ) return 1;
	else return 0;
}

bool xlogger_send_beeper (uint16_t beeper, char *s){
	const uint8_t beeperCount = beeperTableEntryCount();
	if (dump_send_state.additional.beeperCounter == (beeperCount - 2))	return 1;
	else {

		const beeperConfig_t *beeperConfigDefault = beeperConfig();
		const uint32_t defaultMask = beeperConfigDefault->beeper_off_flags;
		const char *formatOff = "beeper -%s";
		const char *formatOn = "beeper %s";

		if (defaultMask & (1 << beeper))	sprintf(s, formatOff,  beeperNameForTableIndex(beeper));
		else 								sprintf(s, formatOn, beeperNameForTableIndex(beeper));

		dump_send_state.additional.beeperCounter ++;
		return 0;
	}
}

bool xlogger_send_map (char *s){

	const rxConfig_t *rxConfigg = rxConfig();
	char buf[16] = {0};
	uint32_t i;

	for (i = 0; i < MAX_MAPPABLE_RX_INPUTS; i++) {
		 buf[rxConfigg->rcmap[i]] = rcChannelLetters[i];
	}
	sprintf(s, "map %s", buf);
	return 1;
}

bool xlogger_send_name (char *s){
	const systemConfig_t * sConfig = systemConfig();
	sprintf(s, "name %s", sConfig->name);
	return 1;
}

bool xlogger_send_serial (uint8_t i, char *s){
	const char *format = "serial %d %d %ld %ld %ld %ld";
	const serialConfig_t *serialConfigDefault = serialConfig();

	if (!serialIsPortAvailable(serialConfigDefault->portConfigs[i].identifier)) {
	//continue;
		if(dump_send_state.additional.serialCounter >= SERIAL_PORT_COUNT)return 1;
		else {
			dump_send_state.additional.serialCounter ++;
			return 0;
		}
	}
	if(i >= SERIAL_PORT_COUNT)return 1;
	else {
	sprintf(s, format,
	        serialConfigDefault->portConfigs[i].identifier,
	        serialConfigDefault->portConfigs[i].functionMask,
	        baudRates[serialConfigDefault->portConfigs[i].msp_baudrateIndex],
	        baudRates[serialConfigDefault->portConfigs[i].gps_baudrateIndex],
	        baudRates[serialConfigDefault->portConfigs[i].telemetry_baudrateIndex],
	        baudRates[serialConfigDefault->portConfigs[i].peripheral_baudrateIndex]
	        );

	dump_send_state.additional.serialCounter++;
	return 0;
	}
}

bool xlogger_send_led (char *s){


	if (dump_send_state.additional.ledCounter == LED_MAX_STRIP_LENGTH)	return 1;
	else {
		uint16_t led = dump_send_state.additional.ledCounter;
		const char *format = "led %u %s";
		char ledConfigBuffer[20];
		const ledConfig_t *ledConfigs = ledStripConfig()->ledConfigs;
		ledConfig_t ledConfig = ledConfigs[led];

		generateLedConfig(&ledConfig, ledConfigBuffer, sizeof(ledConfigBuffer));
		sprintf(s, format, led, ledConfigBuffer);
		dump_send_state.additional.ledCounter ++;
		return 0;
	}
}

bool xlogger_send_color (char *s){

	if (dump_send_state.additional.colorCounter == LED_CONFIGURABLE_COLOR_COUNT)	return 1;
	else {
		uint16_t num = dump_send_state.additional.colorCounter;
			hsvColor_t *colors = ledStripConfig()->colors;
			const hsvColor_t *color = &colors[num];
			const char *format = "color %u %d,%u,%u";
			sprintf(s, format, num, color->h, color->s, color->v);

			dump_send_state.additional.colorCounter ++;
		return 0;
	}
}

bool xlogger_send_mode_color (){
	//const ledStripConfig_t *ledStripConfig;
	// TODO: 3 for cycles
	//const char *format = "mode_color %u %u %u";
	return 1;
}

bool xlogger_send_aux (char *s){

	if (dump_send_state.additional.auxCounter == MAX_MODE_ACTIVATION_CONDITION_COUNT)	return 1;
	else {
		uint8_t num = dump_send_state.additional.auxCounter;
		const char *format = "aux %u %u %u %u %u";
		const modeActivationCondition_t *modeActivationConditionsa = modeActivationConditions(0);
		const modeActivationCondition_t *mac = &modeActivationConditionsa[num];

		sprintf(s, format,
		            num,
		            mac->modeId,
		            mac->auxChannelIndex,
		            MODE_STEP_TO_CHANNEL_VALUE(mac->range.startStep),
		            MODE_STEP_TO_CHANNEL_VALUE(mac->range.endStep)
		        );
		dump_send_state.additional.auxCounter++;
		return 0;
	}
}

bool xlogger_send_adjrange (char *s){



	if (dump_send_state.additional.adjRangeCounter == MAX_ADJUSTMENT_RANGE_COUNT)	return 1;
	else {
		uint8_t i = dump_send_state.additional.adjRangeCounter;
		const adjustmentRange_t *adjustmentRangess = adjustmentRanges(0);
		const char *format = "adjrange %u %u %u %u %u %u %u";
		const adjustmentRange_t *ar = &adjustmentRangess[i];

		sprintf(s, format,
			            i,
			            ar->adjustmentIndex,
			            ar->auxChannelIndex,
			            MODE_STEP_TO_CHANNEL_VALUE(ar->range.startStep),
			            MODE_STEP_TO_CHANNEL_VALUE(ar->range.endStep),
			            ar->adjustmentFunction,
			            ar->auxSwitchChannelIndex);
		dump_send_state.additional.adjRangeCounter++;
		return 0;
	}
}

bool xlogger_send_rxrange (char *s){

	if (dump_send_state.additional.rxRangeCounter == NON_AUX_CHANNEL_COUNT)	return 1;
	else {
		uint8_t i = dump_send_state.additional.rxRangeCounter;
		const char *format = "rxrange %u %u %u";
		const rxChannelRangeConfig_t *channelRangeConfigss=rxChannelRangeConfigs(0);

		sprintf(s, format,
		            i,
		            channelRangeConfigss[i].min,
		            channelRangeConfigss[i].max
		        );

		dump_send_state.additional.rxRangeCounter++;
		return 0;
	}
}



void xlogger_send_additional() {

	char str[55] = {0};
	//uint8_t count = dump_send_state.additional.counter;
	uint8_t str_len = 0;
	bool sendStr = 1;
	switch (dump_send_state.additional.counter) {

	// dump header with versionґ
	case INIT_HEADER:
		sprintf(str, "# version");
		str_len = 15;
		dump_send_state.additional.counter = HEADER_STR;
		break;
	case HEADER_STR:
		sprintf(str,"# %s/%s %s %s / %s (%s)",
				FC_FIRMWARE_NAME,
				targetName,
				FC_VERSION_STRING,
				buildDate,
				buildTime,
				shortGitRevision
		);
		str_len = 56; dump_send_state.additional.counter = MIXER_STR; break;

	// motor mixer
	case MIXER_STR:
		sprintf(str,"# mixer");
		str_len = 8; dump_send_state.additional.counter = MIXER_NAME; break;
	case MIXER_NAME:
		sprintf(str,"mixer %s", mixerNames[mixerConfig()->mixerMode - 1]);// TODO: make correct importґ
		str_len = 25; dump_send_state.additional.counter = MIXER_RESET; break;
	case MIXER_RESET:
		sprintf(str,"mmix reset");
		str_len = 15;
		dump_send_state.additional.counter++; break;
	case MIXER_BODY:
		if(xlogger_send_dump_mixer(dump_send_state.additional.motorMixerCounter, str)) dump_send_state.additional.counter = SMIXER_STR;
		else str_len = 35;
		break;

#ifdef USE_SERVOS
	// servo mixer
	case SMIXER_STR:
		sprintf(str,"# servo mix");
		str_len = 12;	dump_send_state.additional.counter = SMIXER_RESET;	break;
	case SMIXER_RESET:
		sprintf(str,"smix reset");
		str_len = 12;	dump_send_state.additional.counter = SMIXER_BODY;	break;
	case SMIXER_BODY:
		if(xlogger_send_dump_smixer(dump_send_state.additional.servoMixerCounter, str)) dump_send_state.additional.counter = SERVO_STR;
		else str_len = 22;
		break;
	// servos
	case SERVO_STR:
		sprintf(str, "# servo");
		str_len = 7;	dump_send_state.additional.counter = SERVO_BODY;	break;
	case SERVO_BODY:
		if(xlogger_send_dump_servo(dump_send_state.additional.servoCounter, str)) dump_send_state.additional.counter = FEATURE_STR;
		else str_len = 45;
		break;
#else
	case SMIXER_STR:
		dump_send_state.additional.counter = FEATURE_STR;	// TODO: add servos
		break;
#endif
	// festures
	case FEATURE_STR:
		sprintf(str,"# feature");
		str_len = 15; dump_send_state.additional.counter = FEATURE_BODY; break;
	case FEATURE_BODY:
		if (xlogger_send_feature(dump_send_state.additional.featureCounter, str)) dump_send_state.additional.counter = BEEPER_STR;
		else str_len = 45;
		break;
	// beeper
#ifdef BEEPER
	case BEEPER_STR:
		sprintf(str,"# beeper");
		str_len = 10; dump_send_state.additional.counter = BEEPER_BODY; break;
	case BEEPER_BODY:
		if (xlogger_send_beeper(dump_send_state.additional.beeperCounter, str)) dump_send_state.additional.counter = MAP_STR;	// TODO: func
		else str_len = 35;
		break;
#else
	case BEEPER_STR:
		dump_send_state.additional.counter = MAP_STR;
		break;
#endif
	//map
	case MAP_STR:
		sprintf(str,"# map");
		str_len = 5; dump_send_state.additional.counter = MAP_BODY; break;
	case MAP_BODY:
		if (xlogger_send_map(str)) {
			str_len = 25;
			dump_send_state.additional.counter = NAME_STR;	// TODO: func
		}
		else str_len = 25;
		break;
	// name
	case NAME_STR:
		sprintf(str,"# name");
		str_len = 6; dump_send_state.additional.counter = NAME_BODY; break;
	case NAME_BODY:
		if (xlogger_send_name(str)) {
			str_len = 50;
			dump_send_state.additional.counter = SERIAL_STR;	// TODO: func
		}
		else str_len = 25;
		break;
	//serial
	case SERIAL_STR:
		sprintf(str,"# serial");
		str_len = 10; dump_send_state.additional.counter = SERIAL_BODY; break;
	case SERIAL_BODY:
		if (xlogger_send_serial(dump_send_state.additional.serialCounter, str)) dump_send_state.additional.counter = LED_STR;	// TODO: func
		else str_len = 65;
		break;
	//led
#ifdef USE_LED_STRIP
	case LED_STR:
		sprintf(str,"# led");
		str_len = 10; dump_send_state.additional.counter = LED_BODY; break;
	case LED_BODY:
		if (xlogger_send_led(str)) dump_send_state.additional.counter = COLOR_STR;
		else str_len = 25;
		break;
	case COLOR_STR:
		sprintf(str,"# color");
		str_len = 10; dump_send_state.additional.counter = COLOR_BODY; break;
	case COLOR_BODY:
		if (xlogger_send_color(str)) dump_send_state.additional.counter = MODE_COLOR_STR;
		else str_len = 25;
		break;
	case MODE_COLOR_STR:
		sprintf(str,"# mode_color");
		str_len = 12; dump_send_state.additional.counter = MODE_COLOR_BODY; break;
	case MODE_COLOR_BODY:
		if (xlogger_send_mode_color()) dump_send_state.additional.counter = AUX_STR;	// TODO: func
		else str_len = 25;
		break;
#else
	case LED_STR:
		dump_send_state.additional.counter = AUX_STR;
		break;
#endif
	//aux
	case AUX_STR:
		sprintf(str,"# aux");
		str_len = 5; dump_send_state.additional.counter = AUX_BODY; break;
	case AUX_BODY:
		if (xlogger_send_aux(str)) dump_send_state.additional.counter = ADJRANGE_STR;	// TODO: func
		else str_len = 40;
		break;
	//adjrange
	case ADJRANGE_STR:
		sprintf(str,"# adjrange");
		str_len = 15; dump_send_state.additional.counter = ADJRANGE_BODY; break;
	case ADJRANGE_BODY:
		if (xlogger_send_adjrange(str)) dump_send_state.additional.counter = RXRANGE_STR;	// TODO: func
		else str_len = 35;
		break;
	// adjrange
	case RXRANGE_STR:
		sprintf(str,"# rxrange");
		str_len = 15; dump_send_state.additional.counter = RXRANGE_BODY; break;
	case RXRANGE_BODY:
		if (xlogger_send_rxrange(str)) dump_send_state.additional.counter = COUNTER_ALL;	// TODO: func
		else str_len = 25;
		break;
	//finish
	case COUNTER_ALL:
		dump_send_state.additional.newData = 0;
		sendStr = 0;
		break;
	}
	if(sendStr && (str_len != 0))	xlogger_send_string(str, str_len, 0x07);
}


void xlogger_send_dump_commander() {

	if(dump_send_state.additional.newData) {
		xlogger_send_additional();
	}
	else if (dump_send_state.master.newData) {
		xlogger_send_dump_master();
	}
	else if (dump_send_state.profiles.newData) {
		xlogger_send_dump_profile();
	}
	else dump_send_state.need_send_dump = 0;

}

void xlogger_send_dump_reset() {
	dump_send_state.need_send_dump = 0;
	dump_send_state.settings_counter = 0;

	dump_send_state.master.newData = 0;
	dump_send_state.master.newDataHeader = 0;
	dump_send_state.master.newDataMaster = 0;

	dump_send_state.profiles.newDataProfiles = 0;
	dump_send_state.profiles.newDataRate = 0;
	dump_send_state.profiles.newDataHeader = 0;
	dump_send_state.profiles.newDataProfileNumber = 0;
	dump_send_state.profiles.newData = 0;
	dump_send_state.profiles.profile_number = getConfigProfile();

	dump_send_state.additional.newData = 0;
	dump_send_state.additional.counter = 0;
	dump_send_state.additional.motorMixerCounter = 0;
	dump_send_state.additional.servoMixerCounter = 0;
	dump_send_state.additional.servoCounter = 0;
	dump_send_state.additional.featureCounter = 0;
	dump_send_state.additional.beeperCounter = 0;
	dump_send_state.additional.serialCounter = 0;
	dump_send_state.additional.ledCounter = 0;
	dump_send_state.additional.colorCounter = 0;
	dump_send_state.additional.modeColor = 0;
	dump_send_state.additional.auxCounter = 0;
	dump_send_state.additional.adjRangeCounter = 0;
	dump_send_state.additional.rxRangeCounter = 0;


}

void xlogger_start_send_dump() {
	dump_send_state.need_send_dump = 1;

	dump_send_state.master.newData = 1;
	dump_send_state.master.newDataHeader = 1;
	dump_send_state.master.newDataMaster = 1;

	dump_send_state.profiles.newDataProfiles = 1;
	dump_send_state.profiles.newDataRate = 1;
	dump_send_state.profiles.newDataHeader = 1;
	dump_send_state.profiles.newDataProfileNumber = 1;
	dump_send_state.profiles.newData = 1;

	dump_send_state.additional.newData = 1;
}
/*---------------xlogger dump finctions-till there--------------*/

// xlogger sevice functions

void xlogger_send_tlm_gtmsg_pass(void) {
	// this function is called only if we want to passthrough gtmsg message
	// buffer is already filled with data

	xlogger_send_message(gtmsg_pass_blocks);
	gtmsg_pass_enable = 0;
}

void xlogger_send_debug(float val, uint8_t num){
	if (num < MAX_XLOGGER_DEBUG) {
		cur_Flags.Debug.newData = 1;
		cur_Flags.Debug.curTime = millis();
		cur_Flags.Debug.arr[num] = val;
	}
}

// other functions

void xlogger_set_payload_shot_time() {
#if defined(USE_TELEMETRY_XLOGGER)
		cur_Payload.photo.newData = 1;
		cur_Payload.photo.cx_photo_call = millis();
#endif
}

void xlogger_set_payload_answer_time(uint16_t passport_num) {
#if defined(USE_TELEMETRY_XLOGGER)
	cur_Payload.photo.cx_photo_answer = millis();
	cur_Payload.photo.newData = 1;
	cur_Payload.photo.cx_photo_passport_num =  passport_num;
#endif
}

void xlogger_send_flags(void) {
#if defined(USE_TELEMETRY_XLOGGER)
	cur_Flags.CX_flags.newData = 1;
	cur_Flags.CX_flags.curTime = millis();
	cur_Flags.INAV_flags.newData = 1;
	cur_Flags.INAV_flags.curTime = millis();
	cur_Flags.ARM.newData = 1;
	cur_Flags.ARM.newData = millis();
#endif
}

void xlogger_gtmsg_pass_send(uint8_t *buf, uint8_t start_byte,uint8_t stop_byte, uint8_t new_header) {
	// gtmsg message passthrough to the logger
	// 0<start_byte<stop_byte<255
	// SEND ONLY_DATA! message WILL have header and time mark.
	// data will be placed after time mark - from 9th byte
	// call this function with actual indexes in byte array! for(int i=start_byte;i<=stop_byte;i++)
#if defined(USE_TELEMETRY_XLOGGER)
	gtmsg_pass_blocks = (stop_byte-start_byte+1);
	if(gtmsg_pass_blocks % 16 > 0) gtmsg_pass_blocks=gtmsg_pass_blocks/16 + 1;
	else gtmsg_pass_blocks = gtmsg_pass_blocks/16;
	xloggerbuf[4] = new_header;
	xloggerbuf_clear(gtmsg_pass_blocks);
	uint32_fill(5, millis());
	uint8_t offset = 9;
	for(int i=start_byte;i<=stop_byte;i++) {
		xloggerbuf[i-start_byte+offset] = buf[i];
	}
	gtmsg_pass_enable = 1;

#endif
}

void xlogger_send_arm_disarm_result(uint8_t result) {
	// result = 1 - arm
	// result = 0 - disarm
#if defined(USE_TELEMETRY_XLOGGER)
	if((cur_Flags.ARM.armingFlags != armingFlags)) {
		cur_Flags.ARM.arming_result = result;
		cur_Flags.ARM.curTime = millis();
		cur_Flags.ARM.armingFlags = armingFlags;
		cur_Flags.ARM.newData = 1;
	}
#endif
}

void xlogger_launch(  uint8_t result  ){
	// 0 - нет данных (служебное значение)
	// 1 Ц успешно
	// 2 Ц ошибка миссии
	// 4 Ц ошибка режима

#if defined(USE_TELEMETRY_XLOGGER)
	if(cur_Flags.Launch.result != result) {


		cur_Flags.Launch.newData= 1;
		cur_Flags.Launch.result = result;
		cur_Flags.Launch.launch_time = millis();
		rtcGetDateTime(&cur_Flags.Launch.launch_time_UTC);
	}


#endif
}

void xlogger_launch_reset () {
#if defined(USE_TELEMETRY_XLOGGER)
	cur_Flags.Launch.newData= 0;
	cur_Flags.Launch.result = 0;
#endif
}

void xlogger_landed(){

#if defined(USE_TELEMETRY_XLOGGER)
	timeMs_t cur_time = millis();

	if(cur_Flags.Landed.flown_ms != cur_time) {
		cur_Flags.Landed.flown_ms = cur_time - cur_Flags.Launch.launch_time;
		cur_Flags.Landed.newData = 1;
	}
#endif
}

// data input functions
void xlogger_input_parse_message(void){
    // logger is alive
	if( xlogger_input_msgbuf[0] == 0x70 ){
		// reset time drop
		cur_xlogger_status.lastXLoggerHeartbeatTimeUs = micros();
		cur_xlogger_status.periodAfterLastXLoggerHeartbeatUs = 0;

		//set flags
		cur_xlogger_status.isXLoggerAlive = 1;

		if ((cur_Flags.UTCTime.timeStatus == 0) || ((cur_Flags.UTCTime.timeStatus == 1) && (cur_xlogger_status.needSendInit == 1))) {
			cur_Flags.UTCTime.timeStatus = rtcGetDateTime(&cur_Flags.UTCTime.dt);	// 0 if time is not known
			if (cur_Flags.UTCTime.timeStatus) {
				cur_Flags.UTCTime.newData = 1;
			}
		}
		if (cur_xlogger_status.needSendInit) {
			// FIXAR. this defines, what will be send first!
			xlogger_start_send_dump();
			xlogger_send_flags();
			cur_xlogger_status.needSendInitialInfoPocket = 1;
			cur_xlogger_status.needSendInit = 0;
		}



	}
}

uint8_t xlogger_input_clear(void){
	xlogger_input_state = 0;
	xlogger_input_char_count = 0;
	xlogger_input_c1 = 0;
	xlogger_input_c2 = 0;
	for(uint8_t i = 0; i<XLOGGERBUF_RX_LEN; i++) {
		xlogger_input_msgbuf[i] = 0;
	}
    return 0;
}

void xlogger_input_crc(uint8_t c){
	xlogger_input_c1 += c;
	xlogger_input_c2 += xlogger_input_c1;
}

uint8_t xlogger_input_parse_char( char c ){
   xlogger_input_char_count++;
   if( xlogger_input_state == 0 ){  //Header
       if( c != 0x67 ) return xlogger_input_clear();
       xlogger_input_state++;
   }else if( xlogger_input_state == 1){  //Crypter
       if( c != 0x41 ) return xlogger_input_clear();
       xlogger_input_state++;
   }else if( xlogger_input_state == 2){  // Source/Dest
       if( c != 0x00 ) return xlogger_input_clear();	// TODO: Fix source/dest: must be  if( c != 0 )
       xlogger_input_state++;
   }else if( xlogger_input_state == 3){  //Len
       if( c > 2 ) return xlogger_input_clear();
       xlogger_input_state++;
       xlogger_input_msglen = c * 16;
       xlogger_input_msgcurlen = 0;
   }else if( xlogger_input_state == 4){
       xlogger_input_msgbuf[xlogger_input_msgcurlen] = c;
       xlogger_input_msgcurlen++;
       if( xlogger_input_msgcurlen >= xlogger_input_msglen ){
           xlogger_input_state++;
       }
   }else if( xlogger_input_state == 5){
       if( c != xlogger_input_c1 ){
           //xlogger_input__crc_err();
           return xlogger_input_clear();
       }
       xlogger_input_state++;
       return 0; // Not need to calc crc!
   }else if( xlogger_input_state == 6){
       if( c != xlogger_input_c2 ){
           //xlogger_input__crc_err();
           return xlogger_input_clear();
       }
       xlogger_input_parse_message();
       xlogger_input_clear();
       return 1; // Not need to calc crc!
   }

   xlogger_input_crc(c);
   return 0;
}

// process functions
void initXLoggerTelemetry(void)
{
    portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_XLOGGER);
    xloggerPortSharing = determinePortSharing(portConfig, FUNCTION_TELEMETRY_XLOGGER);
    blocks_sent_last_time = 0;
    msg_type = 0;

    cur_PID.flight_thr.new = 0;
    cur_PID.poshold.new = 0;
    cur_PID.landing.new = 0;
    cur_PID.flight_azimuth.new = 0;
    cur_PID.flight_alt.new = 0;
    gpsSol.xlogger_flags.isnew = 0;
    cur_PID.inav_angle.newData = 0;
    cur_PID.inav_rate.newData = 0;
    cur_PID.new_poshold.new = 0;

    cur_Flags.CX_flags.newData = 0;
    cur_Flags.CX_flags.cx_point_hdg = 0;
    cur_Flags.CX_flags.cx_navstate = 0;
    cur_Flags.CX_flags.cx_polygon_state = 0;

    cur_Flags.INAV_flags.newData = 0;
    cur_Flags.INAV_flags.flightModeFlags = 0;
    cur_Flags.INAV_flags.failsafePhase = 0;
    cur_Flags.INAV_flags.hwHealthStatus = 0;
    cur_Flags.INAV_flags.receiver_status = 0;
    cur_Flags.INAV_flags.stateFlags = 0;

    cur_Payload.photo.newData = 0;
    cur_Payload.photo.cx_photo_answer = 0;
    cur_Payload.photo.cx_photo_call = 0;
    cur_Payload.photo.cx_photo_passport_num = 0;

    gtmsg_pass_enable = 0;

    cur_Flags.ARM.newData = 0;
    cur_Flags.ARM.armingFlags = 0;
    cur_Flags.ARM.arming_result = 0;
    cur_Flags.ARM.curTime = 0;

    xlogger_launch_reset();

    cur_Flags.Landed.newData = 0;
    cur_Flags.Landed.flown_ms = 0;

    dump_current_setting = 0;
    xlogger_send_dump_reset();

    cur_xlogger_status.isXLoggerAlive = 0;
    cur_xlogger_status.needSendInit = 1;	// we WANT to send it while initializing
    cur_xlogger_status.needSendInitialInfoPocket = 0;

    cur_Flags.UTCTime.newData = 0;
    cur_Flags.UTCTime.timeStatus = 0;
}



static bool processXLoggerIncomingTelemetry(void)
{
    while (serialRxBytesWaiting(xloggerPort)) {
        // Limit handling to one message per cycle
        char c = serialRead(xloggerPort);
        //gt_parse_char(c);
        xlogger_input_parse_char(c);
    }

    return false;
}

uint8_t checkAsync(uint8_t initial) {

	if (gtmsg_pass_enable) {
		// Ёто сообщение имеет высший приоритет, потому что флаг gtmsg_pass_enable означает, что буфер уже заполнен
		msg_type_buff = initial;
		return 100;
		}
	else if (cur_Flags.ARM.newData) {
		msg_type_buff = initial;
		return 0xB3;
	}
	else if (cur_Flags.INAV_flags.newData) {
		msg_type_buff = initial;
		return 0x30;
	}
	else if (cur_Flags.CX_flags.newData) {
		msg_type_buff = initial;
		return 0x31;
	}
	else if (cur_xlogger_status.needSendInitialInfoPocket){
		msg_type_buff = initial;
		return 0xB0;
	}
	else if (cur_Payload.photo.newData) {
		msg_type_buff = initial;
		return 0x32;
	}
	else if (cur_Flags.Launch.newData) {
		msg_type_buff = initial;
		return 0xB1;
	}
	else if (cur_Flags.Landed.newData) {
		msg_type_buff = initial;
		return 0xB2;
	}
	else if (cur_Flags.UTCTime.newData) {
		msg_type_buff = initial;
		return 0xB8;
	}
	else if (dump_send_state.need_send_dump) {
		msg_type_buff = initial;
		return 127;
	}

	else return initial;
}

void process_logger(timeUs_t currentTimeUs){
	//xloggerPort_freespace = serialTxBytesFree(xloggerPort);
	// isSerialTransmitBufferEmpty(xloggerPort)) {

   if( (last_logger_message_time + (blocks_sent_last_time * systemConfig()->cx_xlogger_min_period_us) < currentTimeUs) && isSerialTransmitBufferEmpty(xloggerPort) ){

    	last_logger_message_time = currentTimeUs;

    	msg_type = checkAsync(msg_type);

    	if( need_send_launch > 0 ){
    		need_send_launch = 0;
    		xlogger_send_launch_start();
    	} else {
    		switch (msg_type) {
    		case 0:
    			xlogger_send_0xD0();
    			msg_type = 1;
    			break;
    		case 1:
    			xlogger_send_0xD1();
    			msg_type = 2;
    			break;
    		case 2:
    			xlogger_send_0xD2();
    			msg_type = 3;
    			break;
    		case 3:
    			xlogger_send_0x62();
    			msg_type = 4;
    			break;
    		case 4:
#ifdef USE_GPS
			if (feature(FEATURE_GPS)) xlogger_send_0x63();
			else blocks_sent_last_time = 0;
#else
			blocks_sent_last_time = 0;
#endif
			msg_type = 5;
			break;
    		case 5:
    			xlogger_send_0x64();
    			msg_type = 6;
    			break;
    		case 6:
    			if (cur_PID.inav_rate.newData  ||  cur_PID.inav_angle.newData)	xlogger_send_0x65();
    			else blocks_sent_last_time = 0;
    			msg_type = 7;
    			break;
    		case 7:
    			if (cur_PID.flight_thr.new) xlogger_send_0x66();
    			else blocks_sent_last_time = 0;
    			msg_type = 8;
    			break;
    		case 8:
    			if (cur_PID.poshold.new) xlogger_send_0x67();
    			else blocks_sent_last_time = 0;
    			msg_type = 9;
    			break;
    		case 9:
    			if (cur_PID.landing.new) xlogger_send_0x68();
    			else blocks_sent_last_time = 0;
    			msg_type = 10;
    			break;
    		case 10:
    			if (cur_PID.flight_alt.new) xlogger_send_0x69();
    			else blocks_sent_last_time = 0;
    			msg_type = 11;
    			break;
    		case 11:
    			if (cur_PID.flight_azimuth.new) xlogger_send_0x6A();
    			else blocks_sent_last_time = 0;
    			msg_type = 0x6B;
    			break;
    		case 0x6B:
    			if (cur_Flags.Debug.newData) xlogger_send_0x6B();
    			else blocks_sent_last_time = 0;
    			msg_type = 0x6C;
    			break;
    		case 0x6C:
    			if (cur_PID.new_poshold.new) xlogger_send_0x6C();
    			else blocks_sent_last_time = 0;
    			msg_type = 0x6D;
    			break;
    		case 0x6D:
    			xlogger_send_0x6D();
    			msg_type = 0;
    			break;

    		case 0x30:
    			xlogger_send_0x30();
    			msg_type = msg_type_buff;
    			break;
    		case 0x31:
    			xlogger_send_0x31();
    			msg_type = msg_type_buff;
    			break;
    		case 0x32:
    			xlogger_send_0x32();
    			msg_type = msg_type_buff;
    			break;
    		case 0xB0:
    			xlogger_send_init();
    			msg_type = msg_type_buff;
    			break;
    		case 0xB1:
    			xlogger_send_launch_start();
    		    msg_type = msg_type_buff;
    		    break;
    		case 0xB2:
    		    xlogger_send_landed();
    		    msg_type = msg_type_buff;
    		    break;
    		case 0xB3:
    		    xlogger_send_0xB3();
    		    msg_type = msg_type_buff;
    		    break;
    		case 0xB8:
    			xlogger_send_UTC_time();
    		    msg_type = msg_type_buff;
    		    break;
    		case 100:	// 100 means telemetry passthrough message
    			xlogger_send_tlm_gtmsg_pass();
    			msg_type = msg_type_buff;
    			break;
    		case 127:
    		    xlogger_send_dump_commander();
    		    msg_type = msg_type_buff;
    		    break;
    		default:
    			msg_type = 0;
    		}
    	}
    }
}

void handleXLoggerTelemetry(timeUs_t currentTimeUs)
{
    if (!xloggerTelemetryEnabled) {
        return;
    }

    if (!xloggerPort) {
        return;
    }

    processXLoggerIncomingTelemetry();

//if( !is_logger_init ) xlogger_send_initial_info();


    //if(isXloggerOnline(currentTimeUs))
    bool stat = isXloggerOnline(currentTimeUs);

    process_logger(currentTimeUs);
}
