#ifndef XBUSDEF_H
#define XBUSDEF_H

/*! \brief Xbus message preamble byte. */
#define XBUS_PREAMBLE (0xFA)
/*! \brief Xbus message bus ID for master devices. */
#define XBUS_MASTERDEVICE (0xFF)
/*! \brief Xbus length byte for messages without payload. */
#define XBUS_NO_PAYLOAD (0x00)
/*! \brief Xbus length byte for message with an extended payload. */
#define XBUS_EXTENDED_LENGTH (0xFF)

/*! \brief Opcode to write to control pipe in I2C/SPI mode */
#define XBUS_PROTOCOL_INFO (0x01)
#define XBUS_CONFIGURE_PROTOCOL (0x02)
#define XBUS_CONTROL_PIPE (0x03)
#define XBUS_PIPE_STATUS (0x04)
#define XBUS_NOTIFICATION_PIPE (0x05)
#define XBUS_MEASUREMENT_PIPE (0x06)

#endif
