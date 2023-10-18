/*
 * Author : VProkofiev
 * FlightGear emulator bridge protocol parser
 */

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>
#include <math.h>

#include "platform.h"
#include "build/build_config.h"
#include "sensors/barometer.h"


#include "build/debug.h"


#include "common/axis.h"
#include "common/typeconversion.h"
#include "common/gps_conversion.h"
#include "common/maths.h"
#include "common/utils.h"
#include "common/time.h"

#include "drivers/serial.h"
#include "drivers/time.h"

#include "fc/config.h"
#include "fc/runtime_config.h"

#include "io/serial.h"
#include "io/gps.h"
#include "io/gps_private.h"

#include "fc/rc_controls.h"
#include "flight/mixer.h"
#include "flight/servos.h"
#include "flight/imu.h"
#include "flight/payload.h"

uint8_t FG_IS_READY = 0;

uint8_t fge_aix = 0;
//la, lo : 4b; h, r, p, y : 2b
uint8_t fge_buf[22 + 38];
uint8_t ck1, ck2;
uint8_t mbuf[30];

uint8_t my_mode = 0;

#define MMODE_EMUL (1)
#define MMODE_LMP (2)
#define MMODE_FOF (3)

bool break_packet(void){
    fge_aix = 0;
    ck1 = 0;
    ck2 = 0;
    return false;
}

float b2f( uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3 ){
	float f;
	uint8_t b[4] = {b0, b1, b2, b3};
	memcpy(&f, &b, 4);
	return f;
}

void fill_lmp(void){
    float r = b2f( fge_buf[20-2], fge_buf[21-2], fge_buf[22-2], fge_buf[23-2] );
    float y = b2f( fge_buf[24-2], fge_buf[25-2], fge_buf[26-2], fge_buf[27-2] );
    float p = b2f( fge_buf[28-2], fge_buf[29-2], fge_buf[30-2], fge_buf[31-2] );

    if( systemConfig()->cx_emulator_mode == 1 ){
        attitude.values.roll = (int)(r*10);
        attitude.values.pitch = (int)(p*10);
        attitude.values.yaw = (int)(y*10);
        if (attitude.values.yaw < 0)
            attitude.values.yaw += 3600;
        if (attitude.values.yaw > 3600)
            attitude.values.yaw -= 3600;
    }
}


void fill_fof(void){
    int32_t h = fge_buf[0] + (fge_buf[1] << 8 )+ (fge_buf[2] << 16 )+ (fge_buf[3] << 24 );
    int32_t x = fge_buf[4] + (fge_buf[5] << 8 )+ (fge_buf[6] << 16 )+ (fge_buf[7] << 24 );
    int32_t y = fge_buf[8] + (fge_buf[9] << 8 )+ (fge_buf[10] << 16 )+ (fge_buf[11] << 24 );
    int32_t ang = fge_buf[12] + (fge_buf[13] << 8 )+ (fge_buf[14] << 16 )+ (fge_buf[15] << 24 );

//    if( systemConfig()->cx_emulator_mode == 1 ){
//        attitude.values.roll = (int)(r*10);
//        attitude.values.pitch = (int)(p*10);
//        attitude.values.yaw = (int)(y*10);
//        if (attitude.values.yaw < 0)
//            attitude.values.yaw += 3600;
//        if (attitude.values.yaw > 3600)
//            attitude.values.yaw -= 3600;
//    }
}

void fill_gps(void){
    uint32_t la = fge_buf[0] + (fge_buf[1] << 8 )+ (fge_buf[2] << 16 )+ (fge_buf[3] << 24 );
    uint32_t lo = fge_buf[4] + (fge_buf[5] << 8 )+ (fge_buf[6] << 16 )+ (fge_buf[7] << 24 );
    //int16_t h = fge_buf[8] + (fge_buf[9] << 8 );
    uint16_t r = fge_buf[10] + (fge_buf[11] << 8 );
    uint16_t p = fge_buf[12] + (fge_buf[13] << 8 );
    uint16_t y = fge_buf[14] + (fge_buf[15] << 8 );
    uint16_t spd = fge_buf[16] + (fge_buf[17] << 8 );
    int32_t h = fge_buf[18] + (fge_buf[19] << 8 )+ (fge_buf[20] << 16 )+ (fge_buf[21] << 24 );
    
    gpsSol.fixType = GPS_FIX_3D;
    gpsSol.llh.lon = lo;
    gpsSol.llh.lat = la;
    gpsSol.llh.alt = h;  //alt in cm
    //posControl.actualState.pos.v[2] = h;

    gpsSol.velNED[X] = spd;  // to cm/s
    gpsSol.velNED[Y] = 0;   // to cm/s
    gpsSol.velNED[Z] = 0;   // to cm/s
    gpsSol.groundSpeed = spd;    // to cm/s
    gpsSol.groundCourse = y;     // Heading 2D deg * 100000 rescaled to deg * 10
    gpsSol.numSat = 12;
    gpsSol.eph = 100;
    gpsSol.epv = 100;
    gpsSol.hdop = 100;
    gpsSol.flags.validVelNE = 1;
    gpsSol.flags.validVelD = 1;
    gpsSol.flags.validEPE = 1;


    gpsSol.time.year = 2018;
    gpsSol.time.month = 01;
    gpsSol.time.day = 2;
    gpsSol.time.hours = 3;
    gpsSol.time.minutes =4;
    gpsSol.time.seconds = 5;
    gpsSol.time.millis = 6;

    gpsSol.flags.validTime = 1;
    
    if( systemConfig()->cx_emulator_mode == 1 ){
        attitude.values.roll = r;
        attitude.values.pitch = p;
        attitude.values.yaw = y;
        if (attitude.values.yaw < 0)
            attitude.values.yaw += 3600;
        if (attitude.values.yaw > 3600)
            attitude.values.yaw -= 3600;        
        
        baro.BaroAlt = h;
    }
    if( systemConfig()->cx_emulator_mode == 2 ){
        attitude.values.yaw = y;
        if (attitude.values.yaw < 0)
            attitude.values.yaw += 3600;
        if (attitude.values.yaw > 3600)
            attitude.values.yaw -= 3600;

        baro.BaroAlt = h;
    }
    
    gpsSetState(GPS_RECEIVING_DATA);
    FG_IS_READY = 1;
}

bool push_byte(uint8_t c){
    if( fge_aix == 0 ){
        if( c == 0xAA ){
        	fge_aix++;
        	my_mode = MMODE_EMUL;
			return false;
        }
        if( c == 0xFF ){
        	fge_aix++;
        	my_mode = MMODE_LMP;
			return false;
        }
        if( c == 0x50 ){
        	fge_aix++;
        	my_mode = MMODE_FOF;
			return false;
        }


        return break_packet();
    }
    if( my_mode == MMODE_EMUL ){
		if( fge_aix == 1 ){
			if( c != 0x01 ){
				return break_packet();
			}
			fge_aix++;
			return false;
		}
		if( fge_aix == 24 ){
			if( c != ck1 ){
				gpsStats.errors++;
				return break_packet();
			}
			fge_aix++;
			return false;
		}
		if( fge_aix == 25 ){
			if( c != ck2 ){
				gpsStats.errors++;
				return break_packet();
			}
			fge_aix = 0;
			fill_gps();
			gpsStats.packetCount++;
			return true;
		}
		fge_buf[fge_aix-2] = c;
		ck1 += c;
		ck2 += ck1 + c;
		fge_aix++;
		return false;
	}

    if( my_mode == MMODE_LMP ){
		if( fge_aix == 1 ){
			if( c != 0x01 ){
				return break_packet();
			}
			fge_aix++;
			return false;
		}
		if( fge_aix == 59 ){
			fill_lmp();
			gpsStats.packetCount++;
			fge_aix = 0;
			return true;
		}



		fge_buf[fge_aix-2] = c;
		fge_aix++;
		return false;
	}


    if( my_mode == MMODE_FOF ){
		if( fge_aix == 1 ){
			if( c != 0x41 ){
				return break_packet();
			}
			fge_aix++;
			return false;
		}
		if( fge_aix == 18 ){
			fill_fof();
			gpsStats.packetCount++;
			fge_aix = 0;
			return true;
		}



		fge_buf[fge_aix-2] = c;
		fge_aix++;
		return false;
	}
}  

bool parse_input(void){
   bool res = false;
   while (serialRxBytesWaiting(gpsState.gpsPort)) {
     uint8_t newChar = serialRead(gpsState.gpsPort);
     res = res || push_byte(newChar);
     mbuf[2] = fge_aix;
   }
   return res;
}

uint8_t fg_state = 0;
extern uint8_t cx_nav_mode;
extern float pl_sum_dist;



void send_controls(void){
    uint16_t shifted_pitch = stab_pitch_value + 1500;

    mbuf[0] = 0xAA;


    mbuf[1] = 0x01 + systemConfig()->cx_emulator_mode;

	mbuf[3] = ( servo[4]& 0xFF);
    mbuf[4] = ((servo[4] >> 8) & 0xFF);
    mbuf[5] = (shifted_pitch & 0xFF);
    mbuf[6] = ((shifted_pitch >> 8) & 0xFF);
    
    if( systemConfig()->cx_emulator_mode == 1){
		mbuf[7] = (motor[0] & 0xFF);
		mbuf[8] = ((motor[0] >> 8) & 0xFF);
		mbuf[9] = (motor[1] & 0xFF);
		mbuf[10] = ((motor[1] >> 8) & 0xFF);
		mbuf[11] = (motor[2] & 0xFF);
		mbuf[12] = ((motor[2] >> 8) & 0xFF);
		mbuf[13] = (motor[3] & 0xFF);
		mbuf[14] = ((motor[3] >> 8) & 0xFF);
    } else {
		mbuf[7] = (attitude.values.roll & 0xFF);
		mbuf[8] = ((attitude.values.roll >> 8) & 0xFF);
		mbuf[9] = (attitude.values.pitch & 0xFF);
		mbuf[10] = ((attitude.values.pitch >> 8) & 0xFF);
		mbuf[11] = (rcCommand[THROTTLE] & 0xFF);
		mbuf[12] = ((rcCommand[THROTTLE] >> 8) & 0xFF);
    }

    mbuf[15] = (systemConfigMutable()->cx_debug & 0xFF);
    mbuf[16] = ((systemConfigMutable()->cx_debug  >> 8) & 0xFF);
    
    mbuf[17] = cx_nav_mode;
    
    uint32_t p1, p2, p3;
    
    //p1 = cx_nav_d2e;
    //p2 = cx_nav_d2s;
    //p3 = cx_nav_ds2e;
    p1 = (int)(pl_sum_dist);
    p2 = 1500 + (int)(payload[0] * 500.0);
    p3 = 1;
    
    mbuf[18] = (p1 & 0xFF);
    mbuf[19] = ((p1 >> 8) & 0xFF);    
    mbuf[20] = ((p1 >> 16) & 0xFF);    
    mbuf[21] = ((p1 >> 24) & 0xFF);  
    mbuf[22] = (p2 & 0xFF);
    mbuf[23] = ((p2 >> 8) & 0xFF);    
    mbuf[24] = ((p2 >> 16) & 0xFF);    
    mbuf[25] = ((p2 >> 24) & 0xFF);  
    mbuf[26] = (p3 & 0xFF);
    mbuf[27] = ((p3 >> 8) & 0xFF);    
    mbuf[28] = ((p3 >> 16) & 0xFF);    
    mbuf[29] = ((p3 >> 24) & 0xFF);      

    serialWriteBuf(gpsState.gpsPort, mbuf, 30);
}

bool gpsHandleFGEMUL(void)
{
    if( fg_state == 0){
        fg_state = 1;
        gpsSol.fixType = 2;
        mbuf[0] = 0xAA;
        mbuf[1] = 0x02;
    }
    send_controls();
    parse_input();
    return true;
}
