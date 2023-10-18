/*
 * This file is part of GTNav.
 * 24.08.18
 */
 
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_TELEMETRY) && defined(USE_TELEMETRY_GT)

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

#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/serial.h"

#include "navigation/navigation.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/gyro.h"
#include "sensors/pitotmeter.h"
#include "sensors/sensors.h"

#include "telemetry/gttel.h"
#include "telemetry/telemetry.h"

// // gt library uses unnames unions that's causes GCC to complain if -Wpedantic is used
// // until this is resolved in gt library - ignore -Wpedantic for gt code
// #pragma GCC diagnostic push
// #pragma GCC diagnostic ignored "-Wpedantic"
// #include "common/gt.h"
// #pragma GCC diagnostic pop

#define TELEMETRY_GT_PORT_MODE     MODE_RXTX
#define TELEMETRY_GT_MAXRATE       50
#define TELEMETRY_GT_DELAY         ((1000 * 1000) / TELEMETRY_GT_MAXRATE)

static serialPort_t *gtPort = NULL;
static serialPortConfig_t *portConfig;

static bool gtTelemetryEnabled =  false;
static portSharing_e gtPortSharing;

uint8_t gtbuf[23];

timeUs_t lastGTMessage;

void gtSendMessage(void);


void freeGTTelemetryPort(void)
{
    closeSerialPort(gtPort);
    gtPort = NULL;
    gtTelemetryEnabled = false;
}

void initGTTelemetry(void)
{
    portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_GT);  
    gtPortSharing = determinePortSharing(portConfig, FUNCTION_TELEMETRY_GT);
}

void configureGTTelemetryPort(void)
{
    if (!portConfig) {
      return;
    }

    baudRate_e baudRateIndex = portConfig->telemetry_baudrateIndex;
    if (baudRateIndex == BAUD_AUTO) {
        baudRateIndex = BAUD_57600;
    }

    gtPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_GT, NULL, NULL, baudRates[baudRateIndex], TELEMETRY_GT_PORT_MODE, SERIAL_NOT_INVERTED);
systemConfigMutable()->cx_payload_A = portConfig->identifier;


    if (!gtPort) {      
        return;
    }

    gtTelemetryEnabled = true;
}

void checkGTTelemetryState(void)
{
    bool newTelemetryEnabledValue = true;//telemetryDetermineEnabledState(gtPortSharing);
    
    if (newTelemetryEnabledValue == gtTelemetryEnabled) {
    }

    if (newTelemetryEnabledValue){      
        configureGTTelemetryPort();
    }else{
        freeGTTelemetryPort();
    }        
    
}


// void processGTTelemetry(timeUs_t currentTimeUs)
// {
    // // is executed @ TELEMETRY_GT_MAXRATE rate
    // if (gtStreamTrigger(MAV_DATA_STREAM_EXTENDED_STATUS)) {
        // gtSendSystemStatus();
    // }

    // if (gtStreamTrigger(MAV_DATA_STREAM_RC_CHANNELS)) {
        // gtSendRCChannelsAndRSSI();
    // }

// #ifdef USE_GPS
    // if (gtStreamTrigger(MAV_DATA_STREAM_POSITION)) {
        // gtSendPosition(currentTimeUs);
    // }
// #endif

    // if (gtStreamTrigger(MAV_DATA_STREAM_EXTRA1)) {
        // gtSendAttitude();
    // }

    // if (gtStreamTrigger(MAV_DATA_STREAM_EXTRA2)) {
        // gtSendHUDAndHeartbeat();
    // }
// }


void handleGTTelemetry(timeUs_t currentTimeUs)
{
    if (!gtTelemetryEnabled) return;
    if (!gtPort) return;
 
    
    if( lastGTMessage + 1000 > currentTimeUs ){
        lastGTMessage = currentTimeUs;
        gtSendMessage();
    }
}

void gtSendMessage(void){
    uint8_t spd = (int)(gpsSol.groundSpeed / 10.0f);
    int16_t h = (int)(getEstimatedActualPosition(Z));        
    int lat = gpsSol.llh.lat;
    int lon = gpsSol.llh.lon;
    uint8_t az_int = (int)( attitude.values.yaw * 255 / 360 );
    
  gtbuf[0] =0x67;
  gtbuf[1] = 0x41;  
    gtbuf[2] = 1;
    gtbuf[3] = 1;
    gtbuf[4] = 1;
    gtbuf[5] = 0;
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
    gtbuf[16] = spd;
    gtbuf[17] = 1;
    gtbuf[18] = az_int;
    gtbuf[19] = (int)(getBatteryVoltage() * 100) & 0xFF;
  gtbuf[20] = 0;
  gtbuf[21] = 0;

    for (int i = 0; i < 22; i++) {
        serialWrite(gtPort, gtbuf[i]);
    }
}

#endif
