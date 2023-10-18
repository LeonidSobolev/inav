#ifndef MTSSP_INTERFACE_H
#define MTSSP_INTERFACE_H

#include "../xSensXbus/xbusdef.h"
#include "stdint.h"
#include "../xSensXbus/xbusmessage.h"
#include "../drivers/accgyro/accgyro_MTi3.h"

#define DRDY_CONFIG_MEVENT_POS	3	//!	\brief Measurement pipe DataReady event enable: 0 = Disabled, 1 = Enabled
#define DRDY_CONFIG_NEVENT_POS	2	//!	\brief Notification pipe DataReady event enable: 0 = Disabled, 1 = Enabled
#define DRDY_CONFIG_OTYPE_POS	1	//!	\brief Output type DataReady pin: 0 = Push/pull, 1 = Open drain
#define DRDY_CONFIG_POL_POS		0	//!	\brief Polarity DataReady pin: 0 = Idle low, 1 = Idle high

#define MTData2_Type 0x36

void readProtocolInfo(busDevice_t *busDev, uint8_t *version, uint8_t *dataReadyConfig);
void configureProtocol(busDevice_t *busDev, uint8_t dataReadyConfig);
void readPipeStatus(busDevice_t *busDev, uint16_t *notificationMessageSize, uint16_t *measurementMessageSize);
uint8_t* readFromPipe(busDevice_t *busDev, uint16_t size, uint8_t pipe);
void sendXbusMessage(busDevice_t *busDev, XbusMessage const *xbusMessage);

#endif
