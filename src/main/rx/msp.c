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

#include "platform.h"

//#ifdef USE_RX_MSP

#include "build/build_config.h"

#include "common/utils.h"

#include "flight/failsafe.h"

#include "rx/rx.h"
#include "rx/msp.h"
#include "rx/sbus.h"


static uint16_t mspFrame[MAX_SUPPORTED_RC_CHANNEL_COUNT];
static bool rxMspFrameDone = false;

uint16_t rxMspReadRawRC(const rxRuntimeConfig_t *rxRuntimeConfigPtr, uint8_t chan)
{
    UNUSED(rxRuntimeConfigPtr);
	//check failsafeOff function from MSP
	if (chan == FailOffAuxChannel){
		if ((mspFrame[chan]< FailOffAuxStart)  ||  (FailOffAuxStop < mspFrame[chan])){
			failsafeSetOff(false);
		}else{
			failsafeSetOff(true);
			if (mspFrame[chan] == FailOffAuxStart+100)
				failsafeMSPSet(true);
			else	
				failsafeMSPSet(false);
		}
	}	
    return mspFrame[chan];
}

//emulate new rc packet on MSP
void rxMspWriteRawRC(uint16_t chan[8]){
	
	for (int i = 0; i < 8; i++) {
        mspFrame[i] = chan[i];
    }
	
	rxMspFrameDone = true;
}

void rxMspFrameReceive(uint16_t *frame, int channelCount)
{
    for (int i = 0; i < channelCount; i++) {
        mspFrame[i] = frame[i];
    }

    // Any channels not provided will be reset to zero
    for (int i = channelCount; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        mspFrame[i] = 0;
    }

    rxMspFrameDone = true;
}

uint8_t rxMspFrameStatus(rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxRuntimeConfig);

    if (!rxMspFrameDone) {
        return RX_FRAME_PENDING;
    }

    rxMspFrameDone = false;
    return RX_FRAME_COMPLETE;
}

void rxMspInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxConfig);

    rxRuntimeConfig->channelCount = MAX_SUPPORTED_RC_CHANNEL_COUNT;
    rxRuntimeConfig->rxRefreshRate = 20000;
    rxRuntimeConfig->rxSignalTimeout = DELAY_5_HZ;
    rxRuntimeConfig->rcReadRawFn = rxMspReadRawRC;
    rxRuntimeConfig->rcFrameStatusFn = rxMspFrameStatus;
    
    	//set initial values
    for (int i = 0; i < 4; i++) {
        mspFrame[i] = 1500;
    }
	mspFrame[2] = 1000;
    for (int i = 4; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        mspFrame[i] = 1000;
    }
    rxMspFrameDone = true;
}
//#endif
