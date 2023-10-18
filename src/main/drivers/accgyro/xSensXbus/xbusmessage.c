#include "xbusmessage.h"
#include "xbusdef.h"

/*!	\brief Format a message into the raw mtssp format ready for transmission to a motion tracker.
 */
XbusMessage currentMessage = {.m_mid = 0, .m_length = 0, .m_data = 0};

size_t XbusMessage_createRawMessage(uint8_t *dest, XbusMessage const *message) //, enum XbusBusFormat format
{
	int n;
	uint8_t checksum;
	uint16_t length;
	uint8_t *dptr = dest;

	if (dest == 0){
		return (message->m_length < 255) ? message->m_length + 7 : message->m_length + 9;
	}

	*dptr++ = XBUS_CONTROL_PIPE;
	// Fill bytes required to allow MT to process data
	*dptr++ = 0;
	*dptr++ = 0;
	*dptr++ = 0;

	checksum = 0;
	checksum -= XBUS_MASTERDEVICE;

	*dptr = message->m_mid;
	checksum -= *dptr++;

	length = message->m_length;

	if (length < XBUS_EXTENDED_LENGTH){
		*dptr = length;
		checksum -= *dptr++;
	}else{
		*dptr = XBUS_EXTENDED_LENGTH;
		checksum -= *dptr++;
		*dptr = length >> 8;
		checksum -= *dptr++;
		*dptr = length & 0xFF;
		checksum -= *dptr++;
	}

	for(n = 0; n < message->m_length; n++){
		*dptr = message->m_data[n];
		checksum -= *dptr++;
	}

	*dptr++ = checksum;
	return dptr - dest;
}

XbusMessage const *setId(uint8_t id){
	currentMessage.m_length = 0;
	currentMessage.m_data = 0;
	currentMessage.m_mid = id;
	return &currentMessage;
}
