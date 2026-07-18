#ifndef OPENWRT_ONE_H
#define OPENWRT_ONE_H

/*
 * OpenWrt One CN7 mikroBUS connector.
 *
 * BOARD mode uses the standard mikroBUS physical pin numbers 1..16.  AN is
 * an analog input and is deliberately unavailable to this digital-GPIO
 * backend.  BCM mode uses the native MT7981 GPIO line numbers.
 */
int pinToGpio_OPENWRT_ONE[64] =
{
   7, 6, 4, 5, 10, 12, 2, 25,
   22, 24, 23,
   -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
};

int pinTobcm_OPENWRT_ONE[64] =
{
   -1, -1, 2, -1, 4, 5, 6, 7,
   -1, -1, 10, -1, 12, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, 22, 23,
   24, 25, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
};

int physToGpio_OPENWRT_ONE[64] =
{
   -1,
   -1, -1,
   7, 6,
   4, 5,
   10, 12,
   -1, 2,
   25, 22,
   24, 23,
   -1, -1,
   -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1,
};

#define OPENWRT_ONE_I2C_DEV "/dev/i2c-0"
#define OPENWRT_ONE_SPI_DEV "/dev/spidev1.0"

#define OPENWRT_ONE_I2C_OFFSET -1
#define OPENWRT_ONE_SPI_OFFSET -1
#define OPENWRT_ONE_PWM_OFFSET -1

#endif
