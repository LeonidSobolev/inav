/*
 * This file is part of GTNav.
 * 24.08.18
 */

#pragma once

void initGTTelemetry(void);
void handleGTTelemetry(timeUs_t currentTimeUs);
void checkGTTelemetryState(void);

void freeGTTelemetryPort(void);
void configureGTTelemetryPort(void);
