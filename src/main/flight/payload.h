/*
 * This file is part of cxnav.
 * Author: V.Prokofev
 * Date: 2018_06_02
 */

#pragma once

#include "common/time.h"

#define MAX_SUPPORTED_PAYLOADS (1)

extern float payload[MAX_SUPPORTED_PAYLOADS];

void payload_update_state(  timeUs_t currentTimeUs  );
