#pragma once
#include "common/axis.h"
#include "drivers/time.h"
#define MAX_XLOGGER_DEBUG 6

bool isXloggerOnline(timeUs_t currentTimeUs);	// xlogger status getter

void initXLoggerTelemetry(void);
void handleXLoggerTelemetry(timeUs_t currentTimeUs);
void checkXLoggerTelemetryState(void);

void freeXLoggerTelemetryPort(void);
void configureXLoggerTelemetryPort(void);

void xlogger_send_flags(void);
void xlogger_launch( uint8_t result );
void xlogger_launch_reset (void);
void xlogger_landed(void);

void xlogger_set_payload_shot_time(void);
void xlogger_set_payload_answer_time(uint16_t passport_num);
void xlogger_gtmsg_pass_send(uint8_t *buf, uint8_t start_byte,uint8_t stop_byte, uint8_t new_header);
void xlogger_send_arm_disarm_result(uint8_t result);
void xlogger_send_debug(float val, uint8_t num);

extern uint8_t xloggerbuf[255];

typedef struct tag_sens_xlogger {
	int16_t gyroADCRaw[XYZ_AXIS_COUNT];

} x_sens_state;

typedef struct tag_PID{

	struct {
		bool new;
		timeUs_t curTime;
		int32_t  az_to_point;
		int16_t  poshold_direction;
		uint32_t dist_to_origin;

		float cx_pr_p_cont;
		float cx_pr_i_cont;
		float cx_pr_d_cont;

		float cx_ph_p_cont;
		float cx_ph_i_cont;
		float cx_ph_d_cont;

		float vx_angle;
		float vy_angle;
	} poshold;


	struct {
		bool new;
		timeUs_t curTime;

		int32_t coord_az2pt;
		int16_t coord_azErr;
		float coord_d2orig;
		float speed_x_tgt;
		float speed_x_val;
		float speed_x_p_cont;
		float speed_x_i_cont;
		float speed_x_d_cont;
		float speed_x_pid_out;
		float speed_y_tgt;
		float speed_y_val;
		float speed_y_p_cont;
		float speed_y_i_cont;
		float speed_y_d_cont;
		float speed_y_pid_out;
		float coord_x_tgt;
		float coord_x_now;
		float coord_y_tgt;
		float coord_y_now;
	} new_poshold;

	struct {
		bool new;
		timeUs_t curTime;
		uint32_t 	need_alt_speed;
		float		cur_alts_err;
		float		cx_alt_spd_pid_p_cont;
		float		cx_alt_spd_pid_i_cont;
		float		cx_alt_spd_pid_d_cont;
		float		alts_res;
	} landing;

	struct {
		bool new;
		timeUs_t curTime;
		uint32_t	need_alt;
		float		alt_err_tho;
		float		cx_alt_t_p_cont;
		float		cx_alt_t_i_cont;
		float		cx_alt_t_d_cont;
		float		throttle_alt_res;
		uint16_t	need_as_in_cm;
		float		cur_speed_err;
		float		cx_as_pid_p_cont;
		float		cx_as_pid_i_cont;
		float		cx_as_pid_d_cont;
		float		throttle_spd_res;
		float 		throttle_res;
		float 		vbat_compensation_factor;

	} flight_thr;

	struct {
		bool new;
		timeUs_t curTime;
		float need_alt;
		float cur_alt_err;
		float cx_alt_pid_p_cont;
		float cx_alt_pid_i_cont;
		float cx_alt_pid_d_cont;
		float alt_res;
		float max_climb_angle;
		float max_dive_angle;
	} flight_alt;

	struct {
		bool new;
		timeUs_t curTime;
		float 	dist;
		float 	der_analog;
		float 	cx_linefollow_p_cont;
		float 	cx_linefollow_i_cont;
		float 	cx_linefollow_d_cont;
		float	nu_on_line;
		float 	need_course;
		float 	need_az;
		float 	cx_nav_pid_p_cont;
		float	cx_nav_pid_i_cont;
		float	cx_nav_pid_d_cont;
		int16_t	cur_az_err;
		float 	az_res;
	} flight_azimuth;

	float buff_p_cont;
	float buff_i_cont;
	float buff_d_cont;

	struct {
		bool newData;
		float Rate[XYZ_AXIS_COUNT];
		float Target[XYZ_AXIS_COUNT];
		float Error[XYZ_AXIS_COUNT];
		float Pterm[XYZ_AXIS_COUNT];
		float Iterm[XYZ_AXIS_COUNT];
		float Dterm[XYZ_AXIS_COUNT];
		float PIDres[XYZ_AXIS_COUNT];
	} inav_rate;

	struct {
		bool newData;
		// angle is attitude.raw[axis]
		float Target[XYZ_AXIS_COUNT];
		float Error[XYZ_AXIS_COUNT];

	} inav_angle;
} x_PID_state;

typedef struct tag_Flags{
	struct {
		bool newData;
		timeMs_t curTime;
		uint32_t flightModeFlags;
		uint32_t stateFlags;
		uint32_t hwHealthStatus;
		uint8_t failsafePhase;
		uint8_t receiver_status;
	}INAV_flags;
	struct {
		bool newData;
		timeMs_t curTime;
		uint8_t cx_polygon_state;
		uint8_t cx_navstate;
		uint16_t cx_point_hdg;
		uint32_t warnings;
		uint8_t manual_modes;

	}CX_flags;
	struct {
		bool newData;
		// result = 0 - disarm
		// result = 1 - arm
		uint8_t arming_result;
		timeMs_t curTime;
		uint32_t armingFlags;
	}ARM;
	struct {
		bool newData;
		uint8_t result;
		timeMs_t launch_time;
		dateTime_t launch_time_UTC;
	}Launch;
	struct {
		bool newData;
		timeMs_t landed_time;
		dateTime_t landed_time_UTC;
		timeMs_t flown_ms;
	}Landed;
	struct {
		bool newData;
		bool timeStatus; // 0 if time is not known
		dateTime_t dt;
	}UTCTime;

	struct {
		bool newData;
		timeMs_t curTime;
		float arr[MAX_XLOGGER_DEBUG];
	}Debug;
} x_Flags_state;

typedef struct tag_payload{
	struct {
		bool newData;
		timeMs_t cur_time;
		timeMs_t cx_photo_call;
		timeMs_t cx_photo_answer;
		uint16_t cx_photo_passport_num;
	}photo;
} x_Payload_state;

typedef enum {
	INIT_HEADER,
	HEADER_STR,

	MIXER_STR,
	MIXER_NAME,
	MIXER_RESET,
	MIXER_BODY,

	SMIXER_STR,
#ifdef USE_SERVOS
	SMIXER_RESET,
	SMIXER_BODY,

	SERVO_STR,
	SERVO_BODY,
#endif

	FEATURE_STR,
	FEATURE_BODY,

	BEEPER_STR,
#ifdef BEEPER
	BEEPER_BODY,
#endif

	MAP_STR,
	MAP_BODY,

	NAME_STR,
	NAME_BODY,

	SERIAL_STR,
	SERIAL_BODY,

	LED_STR,
#ifdef USE_LED_STRIP
	LED_BODY,

	COLOR_STR,
	COLOR_BODY,

	MODE_COLOR_STR,
	MODE_COLOR_BODY,
#endif

	AUX_STR,
	AUX_BODY,

	ADJRANGE_STR,
	ADJRANGE_BODY,

	RXRANGE_STR,
	RXRANGE_BODY,

	COUNTER_ALL
}add_count_e;

typedef struct dump_counts{
	// main flag
	bool need_send_dump;
	// counters
	uint32_t settings_counter;

	//structs
	struct {
		bool newData;
		add_count_e counter;
		uint8_t motorMixerCounter;
		uint8_t servoMixerCounter;
		uint8_t servoCounter;
		uint32_t featureCounter;
		uint16_t beeperCounter;
		uint8_t serialCounter;
		uint16_t ledCounter;
		uint16_t colorCounter;
		uint16_t modeColor;
		uint8_t auxCounter;
		uint8_t adjRangeCounter;
		uint8_t rxRangeCounter;
	}additional;
	struct {
		bool newDataProfiles;
		bool newDataRate;
		bool newDataHeader;
		bool newDataProfileNumber;
		bool newData;
		uint8_t profile_number;
	}profiles;
	struct {
		bool newDataMaster;
		bool newDataHeader;
		bool newData;
	}master;

} x_dump_counters;

typedef struct xlogger_st {

	bool needSendInit;	// main init flag

	bool needSendInitialInfoPocket;	// init pocket flag;
	bool isXLoggerAlive;
	bool wasXloggerAlive;
	timeMs_t periodAfterLastXLoggerHeartbeatUs;
	timeMs_t lastXLoggerHeartbeatTimeUs;
} xlogger_state;

extern x_PID_state cur_PID;
extern x_Flags_state cur_Flags;
extern x_sens_state cur_xlog_sens;
x_Payload_state cur_Payload;
x_dump_counters dump_send_state;
xlogger_state cur_xlogger_status;
