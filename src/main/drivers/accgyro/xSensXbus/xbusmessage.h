#ifndef XBUSMESSAGE_H
#define XBUSMESSAGE_H
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*!	\struct XbusMessage
 \brief Xbus message
 */
typedef struct {
	uint8_t m_mid;
	uint16_t m_length;
	uint8_t *m_data;

#ifdef __cplusplus
	XbusMessage(uint8_t mid, uint16_t length = 0, uint8_t* data = NULL)
		: m_mid(mid)
		, m_length(length)
		, m_data(data)
	{}
#endif
} XbusMessage;

/*!	\brief Low level bus format for transmitted Xbus messages
 */

size_t XbusMessage_createRawMessage(uint8_t *dest, XbusMessage const *message);

#ifdef __cplusplus
}
//}
#endif // extern "C"

#endif
