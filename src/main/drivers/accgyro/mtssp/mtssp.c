#include "mtssp.h"

/*!	\brief Constructs an MtsspInterface
 \param[in] driver The MtsspDriver for handling the communication with the device
 */

/*!	\brief Read MTSSP protocol info
 \param[out] version: The version byte
 \param[out] dataReadyConfig: The data ready configuration byte
 \sa configureProtocol
 */

extern uint8_t MTi3_dataIn[256];
extern uint8_t MTi3_dataOut[256];

void readProtocolInfo(busDevice_t *busDev, uint8_t *version, uint8_t *dataReadyConfig){
	uint8_t *pData = readPacket(busDev, XBUS_PROTOCOL_INFO, 2);
	*version = pData[0];
	*dataReadyConfig = pData[1];
}

/*!	\brief Write MTSSP protocol settings
 \param[in] dataReadyConfig The data ready configuration which must be set

 Bit 7:4	Reserved \n
 Bit 3	Measurement pipe DRDY event enable: 0 = disabled, 1 = enabled \n
 Bit 2	Notification pipe DRDY event enable: 0 = disabled, 1 = enabled \n
 Bit 1	Output type of DRDY pin: = 0 Push/pull, 1 = open drain \n
 Bit 0	Polarity of DRDY signal: 0 = Idle low, 1 = Idle high \n
 \sa readProtocolInfo
 */
void configureProtocol(busDevice_t *busDev, uint8_t dataReadyConfig){
	writePacket(busDev, XBUS_CONFIGURE_PROTOCOL, &dataReadyConfig, sizeof(dataReadyConfig));
}

/*!	\brief Read the pipe status
 \param[out] notificationMessageSize: The number of pending notification bytes
 \param[out] measurementMessageSize: The number of pending measurement bytes
 */
void readPipeStatus(busDevice_t *busDev, uint16_t *notificationMessageSize, uint16_t *measurementMessageSize){
	uint8_t *pData = readPacket(busDev, XBUS_PIPE_STATUS, 4);
	*notificationMessageSize = pData[0] | (pData[1] << 8);
	*measurementMessageSize = pData[2] | (pData[3] << 8);
}

/*!	\brief Read from notification or measurement data pipe
 \param[out] buffer Result buffer
 \param[in] size Number of bytes to read
 \param[in] pipe Pipe from which to read, XBUS_NOTIFICATION_PIPE or XBUS_MEASUREMENT_PIPE
 */
uint8_t* readFromPipe(busDevice_t *busDev, uint16_t size, uint8_t pipe) //uint8_t* buffer,
{
	if ((pipe == XBUS_NOTIFICATION_PIPE) || (pipe == XBUS_MEASUREMENT_PIPE))
		return readPacket(busDev, pipe, size);
}

/*! \brief Sends an xbus message to the motion tracker
 \param[in] xbusMessage Pointer to xbus message which should be send
 */
void sendXbusMessage(busDevice_t *busDev, XbusMessage const *xbusMessage){
	size_t rawLength = XbusMessage_createRawMessage(MTi3_dataOut, xbusMessage);
	writePacketRaw(busDev, rawLength);
}

