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

#pragma once

#include "rx/rx.h"

void rxMspFrameReceive(uint16_t *frame, int channelCount);
void rxMspInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig);

void rxMspSetFailsafe(void);
uint8_t rxMspFrameStatus(rxRuntimeConfig_t *rxRuntimeConfig);
uint16_t rxMspReadRawRC(const rxRuntimeConfig_t *rxRuntimeConfigPtr, uint8_t chan);
void rxMspWriteRawRC(uint16_t chan[8]);
