#pragma once

#include "../drivers/sensor.h"
#include "accgyro.h"
#include "stdbool.h"
#include "stdint.h"
#include "../drivers/accgyro/accgyro_mpu.h"

bool MTi3_AccDetect(accDev_t *acc);
bool MTi3_GyroDetect(gyroDev_t *gyro);

#pragma pack(push)
#pragma pack(1)
typedef struct {
	uint8_t type;
	uint8_t len;
	uint16_t id;
	uint8_t dataLen;
	uint16_t RawAccX;
	uint16_t RawAccY;
	uint16_t RawAccZ;
	uint16_t RawGyrX;
	uint16_t RawGyrY;
	uint16_t RawGyrZ;
	uint16_t RawMagX;
	uint16_t RawMagY;
	uint16_t RawMagZ;
	int16_t RawTemp;
} MTi3_RawData;
#pragma pack(pop)

bool MTi3_Request(busDevice_t *busDev, uint8_t rType);
uint8_t* readPacket(busDevice_t *busDev, uint8_t opcode, int dataLength); //uint8_t* dest
void writePacket(busDevice_t *busDev, uint8_t opcode, uint8_t const *data, int dataLength);
void writePacketRaw(busDevice_t *busDev, int dataLength); // uint8_t const* data,
static bool MTi3_UpdateSensorContex(busDevice_t *busDev, mpuContextData_t *ctx);
uint8_t* MTi3_ReadData(busDevice_t *busDev);
bool MTi3GyroReadScratchpad(gyroDev_t *gyro);
