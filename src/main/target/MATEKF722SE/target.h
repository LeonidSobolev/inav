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

#define TARGET_BOARD_IDENTIFIER "MKF7"

#define USBD_PRODUCT_STRING  "MATEKF722"

#define LED0                    PA14  //Blue   SWCLK
#define LED1                    PA13  //Green  SWDIO

#define BEEPER                  PC13
#define BEEPER_INVERTED

// *************** Gyro & ACC **********************
#define USE_SPI
#define USE_SPI_DEVICE_1

#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define MPU6500_CS_PIN          PC15
#define MPU6500_SPI_BUS         BUS_SPI1

#define USE_EXTI
#define GYRO_INT_EXTI            PC3
#define USE_MPU_DATA_READY_SIGNAL

#define USE_GYRO
#define USE_GYRO_MPU6500
#define GYRO_MPU6500_ALIGN      CW90_DEG

#define USE_ACC
#define USE_ACC_MPU6500
#define ACC_MPU6500_ALIGN       CW90_DEG

// *************** I2C/Baro/Mag *********************
#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C1_SCL                PB8        // SCL pad
#define I2C1_SDA                PB9        // SDA pad

#define USE_BARO
#define BARO_I2C_BUS            BUS_I2C1
#define USE_BARO_BMP280
#define USE_BARO_MS5611
//#define USE_BARO_BMP085

#define USE_MAG
#define MAG_I2C_BUS             BUS_I2C1
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define USE_MAG_IST8310
#define USE_MAG_MAG3110

#define USE_MAG_IST8308
#define USE_MAG_LIS3MDL

// *************** SD Card **************************
//#define USE_SDCARD
//#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

// #define USE_SPI_DEVICE_3
// #define SPI3_SCK_PIN            PB3
// #define SPI3_MISO_PIN   	    PB4
// #define SPI3_MOSI_PIN   	    PB5

// #define SDCARD_SPI_INSTANCE     SPI3
// #define SDCARD_SPI_CS_PIN       PC1

// #define SDCARD_DMA_CHANNEL_TX               	DMA1_Stream7
// #define SDCARD_DMA_CHANNEL_TX_COMPLETE_FLAG 	DMA_FLAG_TCIF7
// #define SDCARD_DMA_CLK                      	RCC_AHB1Periph_DMA1
// #define SDCARD_DMA_CHANNEL                  	DMA_CHANNEL_0

// *************** OSD *****************************
// #define USE_SPI_DEVICE_2
// #define SPI2_SCK_PIN            PB13
// #define SPI2_MISO_PIN           PB14
// #define SPI2_MOSI_PIN           PB15

// #define USE_OSD
// #define USE_MAX7456
// #define MAX7456_SPI_BUS         BUS_SPI2
// #define MAX7456_CS_PIN          PB10

// *************** UART *****************************
#define USE_VCP
#define VBUS_SENSING_PIN        PB12
#define VBUS_SENSING_ENABLED

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART2
#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PA2

#define USE_UART3
#define UART3_RX_PIN            PC11
#define UART3_TX_PIN            PC10

#define USE_UART4
#define UART4_RX_PIN            PA1
#define UART4_TX_PIN            PA0

#define USE_UART5
#define UART5_RX_PIN            PD2
#define UART5_TX_PIN            PC12

#define SERIAL_PORT_COUNT       6

#define DEFAULT_RX_TYPE         RX_TYPE_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART2

// *************** ADC *****************************
#define USE_ADC
#define ADC_INSTANCE                ADC1
#define ADC1_DMA_STREAM             DMA2_Stream0
#define ADC_CHANNEL_1_PIN           PC2
#define ADC_CHANNEL_2_PIN           PC1
#define ADC_CHANNEL_3_PIN           PC0
#define VBAT_ADC_CHANNEL            ADC_CHN_1
#define CURRENT_METER_ADC_CHANNEL   ADC_CHN_2
#define RSSI_ADC_CHANNEL            ADC_CHN_3

#define DEFAULT_FEATURES        (FEATURE_TX_PROF_SEL | FEATURE_CURRENT_METER | FEATURE_TELEMETRY| FEATURE_VBAT  )

// #define USE_LED_STRIP
// #define WS2811_PIN                      PA8   //TIM2_CH1
// #define WS2811_DMA_HANDLER_IDENTIFER    DMA1_ST5_HANDLER
// #define WS2811_DMA_STREAM               DMA1_Stream5
// #define WS2811_DMA_CHANNEL              DMA_CHANNEL_3

#define USE_SPEKTRUM_BIND
#define BIND_PIN                PA3  //USART2_RX

#define PAYLOAD_EXTI PA4
#define PAYLOAD_SHOT PC12


#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT  9
#define MAX_PWM_OUTPUT_PORTS        7
#define USED_TIMERS                 (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(8))
