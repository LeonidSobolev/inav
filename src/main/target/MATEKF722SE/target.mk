F7X2RE_TARGETS += $(TARGET)
FEATURES       += SDCARD VCP

TARGET_SRC = \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/barometer/barometer_bmp085.c \
            drivers/barometer/barometer_bmp280.c \
            drivers/barometer/barometer_ms56xx.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/compass/compass_qmc5883l.c \
            drivers/compass/compass_ist8310.c \
            drivers/compass/compass_mag3110.c \
            drivers/light_ws2811strip.c \
            drivers/light_ws2811strip_hal.c \
            drivers/max7456.c

