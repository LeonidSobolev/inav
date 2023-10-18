/*
 * This file is part of cxnav.
 * Author: V.Prokofev
 * Date: 2018_06_02
 *
 * Payload stores and functions
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "platform.h"

#include "navigation/navigation.h"
#include "navigation/navigation_private.h"
#include "common/maths.h"
#include "common/time.h"
#include "flight/payload.h"
#include "fc/config.h"


// *** PUBLIC
float payload[MAX_SUPPORTED_PAYLOADS]; //Range [-1;1]

// *** STATIC
float pl_sum_dist = 0;
fpVector3_t pl_origin;

void payload_update_state( timeUs_t currentTimeUs ){
    // uint32_t dx = posControl.actualState.pos.V.X - pl_origin.V.X;
    // uint32_t dy = posControl.actualState.pos.V.Y - pl_origin.V.Y;
    // float distance_cm = sqrt(dx*dx + dy*dy);
    // if(distance_cm > 100){
      // pl_origin = posControl.actualState.pos;
      // pl_sum_dist += distance_cm;
      // payload[0] = systemConfig()->cx_payload_A / 100.0 * sinf( 2.0 * 3.14159265 * pl_sum_dist / 100.0 / systemConfig()->cx_payload_B ); 
    // }      
    //payload[0] = systemConfig()->cx_payload_A / 100.0 * sinf( 2.0 * 3.14159265 * currentTimeUs / 1000000000 / systemConfig()->cx_payload_B );
}
