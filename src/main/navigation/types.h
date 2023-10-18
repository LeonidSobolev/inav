#ifndef _APTYPES_H_
#define _APTYPES_H_

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>

#include "platform.h"

typedef unsigned char bit;

typedef struct
{
  double  VX;
  double  VY;
  double  VZ;
} TGPSVelocity;

typedef struct
{
  double        lat;                    //deg
  double        lon;                    //deg
  double        hMSL;                   //m
  uint32_t      time;                   //ms
  double        vDOP;
  double        hDOP;
  uint8_t       fix;
  uint8_t       numSV;
  double        speed;                  //m/s
  double        heading;                //deg
  TGPSVelocity  velocity;
} TGPS;

typedef struct tag_SensorValue{
  float x, y, z; 
} SensorValue;

typedef float TAngle;

  

typedef union
{
  double d;
  unsigned char b [sizeof (double)];
} Double;


typedef struct tag_ShortInt{
  union
  {
    signed short int i;
    struct {unsigned char b1,b2;};
  };
} ShortInt;

typedef union
{
float f;
unsigned char a [sizeof (float)];
} bd_dloat;


typedef struct tagGpsPoint{
  double la, lo, h;
} TGPSPoint;

typedef union {
  TGPSPoint p;
  unsigned char b [ sizeof(TGPSPoint) ];
} TGPSPointBits;

#define AP_STATE_AP (0)
#define AP_STATE_MANUAL (1)

typedef struct tagPIDCoef{
  float rKi, rKp, rKd;
  float pKi, pKp, pKd;
  float asKi, asKp, asKd;
} TPID;
//
//typedef struct tagDebugImuStruct{
//  int16_t yaw;
//  int16_t gYaw;
//  int16_t speed;
//  int16_t ax, cacc;
//  int16_t mx, my, mz;
//} TDebugIMU;
  

typedef struct tagPID{
  float p, i, d;
} PID;

#define VTS_OFF (0)
#define VTS_VIDEO (1)
#define VTS_THERMO (2)

typedef struct tagPlaneCoord{
  Double roll, pitch, yaw;
  TGPSPointBits gps;
  Double roll_d, pitch_d;
  Double az;
  Double airspeed;
  Double groundspeed;
  Double pressure;
  double windSpeed, windAzimuth;
  double needAzimuth, needPitch, needRoll, needAS;
  double airspeedPitchCorrection;
  int state;
  double elevator, aileron, throttle, flaps, rudder;
  double holdedAltitude;
  bit armed;
  float parachute, buffer;
  bit camera_power;
  bit camera_shooting;
  uint8_t gpsFix;
  uint8_t gpsSatCount;
  double voltage, current;
  //Используется в случае если самолету нужна метка для чего-либо (кружить вокруг нее)
  TGPSPoint curOrigin;
  Double magAz;
  float trim_roll, trim_pitch;
  int photoNumber;
  TGPSPoint lastPhotoCoord;
  uint8_t curPhotoPage;
  uint8_t isAirStart;
  Double lastPhotoYaw,  lastPhotoRoll,  lastPhotoPitch; 
  uint16_t flightCount;
  uint8_t buzzerOn;
  uint8_t isEngineLocked; //Глобальная блокировка двигателя. Отключается катапультой, включается парашютом. 
  double energy; 
  uint8_t payloadSwitch;
  float temp;
  uint8_t hpgps;
  uint8_t hpgpsstate;
  uint32_t hpgps_flashAddr;
  uint32_t hpgps_flashStartAddr;
  uint8_t hpgps_sdProgress;
  uint32_t hpgps_lastErasedSubSector;
} TPlaneCoord;

#define HPGPS_OFF (0x00)
#define HPGPS_ONLINE (0x01)
#define HPGPS_OK (0xFF)

typedef struct tagTPass{
  uint16_t num;
  double la;
  double lo;
  double h;
  double roll;
  double pitch;
  double yaw; 
} TPass;

#pragma pack(push,1)
typedef struct tagTSmallPass{
  uint16_t num;
  int16_t az;
  double la;
  double lo;
  float h;
} TSmallPass;
#pragma pack(pop)

typedef struct tag_FutabaScalePoint{
  float chanNumber;
  float minIn, maxIn;
  float minOut, maxOut;
} FutabaScalePoint;

typedef struct tag_PWMScalePoint{
  float chanNumber;
  float min, center, max;
} PWMScalePoint;

#define PARAM_CLEARED (101)
#define ISPARAM_CLEARED (100)

typedef struct tagAPConst{
  //Общие константы
  float PL_MaxRoll; //
  float PL_MaxPitch; // 
  float PL_TakeoffAlt; // Высота переключения состояний из взлета в climb
  float PL_LandAlt; // Высота выброса парашюта
  
  //Константы модели
  float PL_TO_EngDelay; // Pflthrf gecrf ldbufntkz ghb dpktnt
  float PL_PAR_EngDelay;
  float PL_TO_Pitch;
  float PL_TO_Throttle;
  float CALIB_ParamWrEn;
  float PHOTO_HPGPS;
  float PHOTO_CamType;
  float DescendMinThrottle;
  float PL_CL_Throttle;  //Газ на наборе
  float MO_AS_MinCorr; // Время макс. надувания
  float MO_AS_MaxCorr; // Как сильно поддувать
  
  TPID pid;
  
  FutabaScalePoint a, e, t, r;  
  
  PWMScalePoint pwmal, pwmar, pwme, pwmt, pwmparachute, pwmr;
  float IsParamsValid;
  float PressureGround;
  float AltitudeGround;
  
  SensorValue aOffset, gOffset, mOffset;
  
  float C_PlaneMode;
  
  float Nav_Gain;
  float Nav_L;
  
  PID NavPitch;
  float NAV_Dest;
  float RollKalmanGain;
  float PitchKalmanGain;
  float YawKalmanGain;
  float AltKalmanGain;
  float AccFilterGain;
  
  float Sens_gscale; //gyro scale [1..4]
  float PID_MaxAil, PID_MaxElev, PID_MaxRudder;
  float PID_AilElevInfluence;
  float CALIB_AltSGain, PHOTO_J_Freq;
  
  float AS_cruise; // Какую возд. скорость выдерживать
  float Thro_min, Thro_max; // Макс. и минимальное значение газа регулятора
  float AS_pitchCorrI, AS_pitchCorrD; // Коэф сброса высоты при недостатке газа. Increase и Decrease
  
  float AS_filterGain; // Сейчас новое измерение возд. влияет так: result = result * (1-AS_filterGain) + ADCValue
  
  PID PID_AsGs;
  float MO_AscPitchLim;
  float MO_DesPitchLim;
  
  //Режим полета, 0 -- XTrack, 1 -- XTrackPID
  float ParamState; 
  //Калибровочный параметр, устанавливает режим калибровки чего-либо, см CalibControls.c
  float Calib_Mode;
  //Включение-выключение элевонов
  float C_IsElevons;
  //Угол наклона катапульты, минимальный
  float CatapultPitch;
  
  PID NavPitchAS;
  
  float ThrottleCruise;
  
  float ail_trim;
  float elev_trim;
  float rud_trim;
  
  float AS_cruise_land;
  float C_VoltageScale;
  float NAV_MaxPitchAsc;
  float C_IsEmul;
  float C_HasRudder;
  float MO_Thro_Inverse;
  float rud_RollScale;
  
  float mag_offsetX;
  float mag_offsetY;
  float mag_offsetZ;
  
  float mag_calib[9];
  
  float mag_msgOn;
  
  float mag_var;
  float PAR_MidPos;
  
  float verFW, numAP;
  float C_MinRad;
  float Calib_CurOffset;
  float Calib_CurScale;
  float Photo_StabDelay;
  float Calib_RPMScale;
} TAPConst;

typedef struct tag_ParamRange{
  float min;
  float max;
} TParamRange;

const char * TYPES_GetParamName( int paramid );
int TYPES_GetParamCompId( int paramid );

#define APCONST_PARAMCOUNT (150)

typedef union{
  TAPConst asStruct;
  float asParam[ sizeof(TAPConst) / sizeof( float ) ];
  uint8_t asByte[ sizeof(TAPConst) ];
} TAPConstBin;

#define APPERIPHSTATE_GPS (1)
#define APPERIPHSTATE_FUTABA (2)
#define APPERIPHSTATE_IMU (4)
#define APPERIPHSTATE_FLASH (8)
#define APPERIPHSTATE_PWM (16)
#define APPERIPHSTATE_WATCHDOG (32)
#define APPERIPHSTATE_MANUALCONTROL (64)

#define OVERRIDE_THROTTLE_OFF (0)
 

typedef struct tagAPState{
  uint8_t apstate;
  uint8_t mission_current;
  uint8_t crc;
} TAPState;

bit APState_CheckCRC( TAPState *s );
void APState_FillCRC( TAPState *s );

typedef struct tagTelemetry{
  int n;                          //4
  float r, p, y;                  //16
  double la, lo, h, v, satCount;  //56
  double hp;                      //
  double dv;                      //72
  unsigned char gpsStatus;        //73
  unsigned char satStatus;        //74
  unsigned char hh, mm, ss;       //77
  unsigned char msms;             // 78
  unsigned char photoNumber;      //79
} TTelemetry;

typedef union {
  TTelemetry t;
  unsigned char b [sizeof (TTelemetry)];
} TTelemetryBin;

//typedef struct tagMissionPoint{
//  uint8_t frame;
//  uint16_t command;
//  uint8_t current, autocontinue;
//  float param1, param2, param3, param4;
//  float la, lo, al;
//} TMissionPoint;
//
//typedef union {
//  TMissionPoint mp;
//  unsigned char b[ sizeof( TMissionPoint )];
//} TMissionPointBin;

#define MAXMISSIONPOINTSCOUNT (255)

typedef struct
{
	TGPSPoint points[ MAXMISSIONPOINTSCOUNT ];
	int points_photoDist[ MAXMISSIONPOINTSCOUNT ];
        int points_types[ MAXMISSIONPOINTSCOUNT ];
	int index;
	int size;
} TRoute;
 
typedef struct 
{
	TGPSPoint start;
	TGPSPoint end;
} TPair;

#define HARDWARE_STATE_AIRSTART         (0x0001)
#define HARDWARE_STATE_MANUAL           (0x0002)
#define HARDWARE_STATE_STABILIZE        (0x0004)
#define HARDWARE_STATE_GPSERROR         (0x0008)
#define HARDWARE_STATE_IMUERROR         (0x0010)
#define HARDWARE_STATE_FRAMERROR        (0x0020)
#define HARDWARE_STATE_KRLERROR         (0x0040)
#define HARDWARE_STATE_PHOTOERROR       (0x0080)
#define HARDWARE_STATE_I2CERROR         (0x0100)
#define HARDWARE_STATE_BMPERROR         (0x0200)
#define HARDWARE_STATE_IWDGOFF          (0x0400)
#define HARDWARE_STATE_PARAMERROR       (0x0800)
#define HARDWARE_STATE_MISSIONERROR     (0x1000)
#define HARDWARE_STATE_NOTREADY         (0x2000)
#define HARDWARE_STATE_GNSSSTORAGEERROR  (0x4000)
#define HARDWARE_STATE_EMULATORMODE     (0x8000)




typedef struct tag_PassRecord{
  TPass p;
  uint8_t isNeedToRec;
} PassRecord;


typedef uint16_t THardwareState;
  
//Общий ответ функций модулей
typedef uint8_t TError;
#define ERR_OK (0)
#define ERR_ERROR (1)
#define ERR_HARDERROR (2)
#define ERR_SOFTERROR (4)

typedef unsigned char (*TUartParseCharFunction)( unsigned char ); 
//typedef int (*mathFun)(int, int);

TParamRange TYPES_GetParamRange( int ix );
void UTILS_FloatToStr( float f, char * buf, int pos, int intSize, int floatSize );

void UTILS_Delay( int i );
float TYPES_GetPWMValue( float needPos, PWMScalePoint cp );

char* itoa(int value, char* result, int base);
 
double getAltitudeFromPressure( double pressure, double pressureGround );

uint32_t SetError( uint32_t errCode );
uint32_t ClearError( uint32_t errCode );

TPass Types_GetPass( int num,
                     double la,
                     double lo,
                     double h,
                     double roll,
                     double pitch,
                     double yaw
                       );



typedef struct tag_TFloatForfprintf{
  int intp;
  int fracp;
} TFFP;

#endif
