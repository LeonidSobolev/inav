#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <../drivers/accgyro/xSensXbus/xbusdef.h>
/*
 #include "build/debug.h"

 #include "common/axis.h"
 #include "common/maths.h"

 #include "drivers/system.h"
 #include "drivers/time.h"
 #include "drivers/io.h"
 #include "drivers/exti.h"
 #include "drivers/bus.h"
 */

#include "../drivers/accgyro/accgyro_mpu.h"
#include "platform.h"
#include "../drivers/accgyro/accgyro.h"
#include "accgyro_MTi3.h"
//#include "gpio.h"
#include "drivers/gpio.h"
//#include "spi.h"
#include "drivers/bus_spi.h"
#include "../drivers/accgyro/xSensXbus/xbusmessageid.h"
#include "../drivers/accgyro/xSensXbus/xbusmessage.h"
#include "../drivers/accgyro/mtssp/mtssp.h"

#if defined(USE_IMU_MTi3)

#define TIMEOUT_MS 100

uint8_t MTi3_dataIn[256];
uint8_t MTi3_dataOut[256];

const uint8_t confData[] = {0xA0, 0x10, 0x00, 0x0A}; //RAW, 10Hz

static void MTi3_DevInit(gyroDev_t *gyro){
	busDevice_t *busDev = gyro->busDev;
	const gyroFilterAndRateConfig_t *config = mpuChooseGyroConfig(gyro->lpf, 1000000 / gyro->requestedSampleIntervalUs);
	gyro->sampleRateIntervalUs = 1000000 / config->gyroRateHz;
	gyroIntExtiInit(gyro);
	busSetSpeed(busDev, BUS_SPEED_INITIALIZATION);
	delay(2);
	XbusMessage *msg = setId(XMID_SetOutputConfig);
	msg->m_length = sizeof(confData);
	msg->m_data = confData;
	sendXbusMessage(busDev, msg);
	delay(10);

#ifdef USE_MPU_DATA_READY_SIGNAL
	configureProtocol(busDev, (1 << DRDY_CONFIG_MEVENT_POS) | (1 << DRDY_CONFIG_NEVENT_POS));
#endif

	if (MTi3_Request(busDev, XMID_Reset)){
		if (MTi3_Request(busDev, XMID_GotoMeasurement)){
			asm("NOP;");
		}
	}
	busSetSpeed(busDev, BUS_SPEED_FAST);

	if (!MTi3_ReadData(busDev))
		failureMode(FAILURE_GYRO_INIT_FAILED);
	/*if (((int8_t) gyro->gyroADCRaw[1]) == -1 && ((int8_t) gyro->gyroADCRaw[0]) == -1){
	 failureMode(FAILURE_GYRO_INIT_FAILED);
	 }*/
}

static void MTi3_setupAcc(accDev_t *acc){
	//acc->acc_1G = 512 * 4;
}

bool MTi3_AccDetect(accDev_t *acc){
	acc->busDev = busDeviceOpen(BUSTYPE_ANY, DEVHW_MTi3, acc->imuSensorToUse);
	if (acc->busDev == NULL){
		return false;
	}

	mpuContextData_t *ctx = busDeviceGetScratchpadMemory(acc->busDev);
	if (ctx->chipMagicNumber != 0x5432){
		return false;
	}

	acc->initFn = MTi3_setupAcc;
	acc->readFn = mpuAccReadScratchpad;
	acc->accAlign = acc->busDev->param;
	return true;
}

uint16_t notificationMessageSize = 0, measurementMessageSize = 0;

static bool MTi3_DevDetect(busDevice_t *busDev){
	busSetSpeed(busDev, BUS_SPEED_INITIALIZATION);
	if (MTi3_Request(busDev, XMID_Reset)){
		if (MTi3_Request(busDev, XMID_GotoConfig)){
			if (MTi3_Request(busDev, XMID_ReqDid)){
				return true;
			}
		}
	}
	return false;
}

bool MTi3_GyroDetect(gyroDev_t *gyro){
	gyro->busDev = busDeviceInit(BUSTYPE_ANY, DEVHW_MTi3, gyro->imuSensorToUse, OWNER_MPU);
	if (gyro->busDev == NULL){
		return false;
	}

	if (!MTi3_DevDetect(gyro->busDev)){
		busDeviceDeInit(gyro->busDev);
		return false;
	}

	// Magic number for ACC detection to indicate that we have detected ?????? gyro
	mpuContextData_t *ctx = busDeviceGetScratchpadMemory(gyro->busDev);
	ctx->chipMagicNumber = 0x5432;

	gyro->initFn = MTi3_DevInit;
	gyro->readFn = MTi3GyroReadScratchpad;
	gyro->intStatusFn = gyroCheckDataReady;
	gyro->temperatureFn = mpuTemperatureReadScratchpad;
	gyro->scale = 1.0f / 16.4f;     // 16.4 dps/lsb scalefactor
	gyro->gyroAlign = gyro->busDev->param;
	return true;
}

bool MTi3GyroReadScratchpad(gyroDev_t *gyro){
	busDevice_t *busDev = gyro->busDev;
	mpuContextData_t *ctx = busDeviceGetScratchpadMemory(busDev);
	if (MTi3_UpdateSensorContex(busDev, ctx)){
		gyro->gyroADCRaw[X] = (int16_t) ((ctx->gyroRaw[0] << 8) | ctx->gyroRaw[1]);
		gyro->gyroADCRaw[Y] = (int16_t) ((ctx->gyroRaw[2] << 8) | ctx->gyroRaw[3]);
		gyro->gyroADCRaw[Z] = (int16_t) ((ctx->gyroRaw[4] << 8) | ctx->gyroRaw[5]);
		return true;
	}
	return false;
}

void writePacket(busDevice_t *busDev, uint8_t opcode, uint8_t const *data, int dataLength){
	memset(&MTi3_dataOut[0], 0x00, 4);
	MTi3_dataOut[0] = opcode;
	memcpy(&MTi3_dataOut[4], (uint8_t*) data, dataLength);
	busTransfer(busDev, MTi3_dataIn, MTi3_dataOut, 4 + dataLength);
}

uint8_t* readPacket(busDevice_t *busDev, uint8_t opcode, int dataLength){
	memset(MTi3_dataOut, 0x00, 4);
	MTi3_dataOut[0] = opcode;
	busTransfer(busDev, MTi3_dataIn, MTi3_dataOut, 4 + dataLength);
	return &MTi3_dataIn[4];
}

void writePacketRaw(busDevice_t *busDev, int dataLength){
	busTransfer(busDev, MTi3_dataIn, MTi3_dataOut, dataLength);
}

bool MTi3_Request(busDevice_t *busDev, uint8_t rType){
	uint8_t attemptsRemaining = 5;
	writePacketRaw(busDev, XbusMessage_createRawMessage(MTi3_dataOut, setId(rType)));
	delay(2);
	do{
		readPipeStatus(busDev, &notificationMessageSize, &measurementMessageSize);

		if (notificationMessageSize && notificationMessageSize < sizeof(MTi3_dataIn)){
			if (*readFromPipe(busDev, notificationMessageSize, XBUS_NOTIFICATION_PIPE) == (rType + 1))
				return true;
		}

		if (!attemptsRemaining){
			return false;
		}else{
			delay(5);
		}
		notificationMessageSize = 0;
	}while (attemptsRemaining--);
}

uint8_t* MTi3_ReadData(busDevice_t *busDev){
	notificationMessageSize = 0;
	measurementMessageSize = 0;
	readPipeStatus(busDev, &notificationMessageSize, &measurementMessageSize);

	if (notificationMessageSize && notificationMessageSize < sizeof(MTi3_dataIn)){
		uint8_t *dOut = readFromPipe(busDev, notificationMessageSize, XBUS_NOTIFICATION_PIPE);
	}

	if (measurementMessageSize && measurementMessageSize < sizeof(MTi3_dataIn)){
		return readFromPipe(busDev, measurementMessageSize, XBUS_MEASUREMENT_PIPE);
	}
	return 0;
}

static bool MTi3_UpdateSensorContex(busDevice_t *busDev, mpuContextData_t *ctx){
	ctx->lastReadStatus = 0;
	MTi3_RawData *newData = (MTi3_RawData*) MTi3_ReadData(busDev);
	if (newData->type == MTData2_Type){
		if (newData->id == 0x10A0){
			*(uint16_t*) &ctx->accRaw[0] = newData->RawAccX;
			*(uint16_t*) &ctx->accRaw[2] = newData->RawAccY;
			*(uint16_t*) &ctx->accRaw[4] = newData->RawAccZ;

			*(uint16_t*) &ctx->gyroRaw[0] = newData->RawGyrX;
			*(uint16_t*) &ctx->gyroRaw[2] = newData->RawGyrY;
			*(uint16_t*) &ctx->gyroRaw[4] = newData->RawGyrZ;

			*(int16_t*) &ctx->tempRaw[0] = newData->RawTemp;

			ctx->lastReadStatus = true;
		}
	}
	return ctx->lastReadStatus;
}

#endif
