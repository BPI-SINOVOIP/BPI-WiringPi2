#ifndef BPI_CANMV_K230D_ZERO_H
#define BPI_CANMV_K230D_ZERO_H

/*
 * BPI-CanMV-K230D Zero JP1 40-pin header.
 *
 * Values are the GPIO numbers in the official Banana Pi 40-pin table.  The
 * published row for physical pin 16 does not provide a GPIO number, so it is
 * deliberately unavailable instead of being inferred from a peripheral name.
 */
int pinToGpio_BPI_K230D_ZERO[64] =
{
   19, 20, 6, 2, -1, 10, 34, 5,
   12, 11, 14, 62, 16, 17, 15, 3,
   4, -1, -1, -1, -1, 25, 24, 60,
   23, 63, 61, 18, 22, 21, 33, 32,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
};

int pinTobcm_BPI_K230D_ZERO[64] =
{
   -1, -1, 12, 11, 5, 25, 24, 62,
   14, 17, 16, 15, 61, 60, 3, 4,
   18, 19, 20, 23, 22, 21, 2, -1,
   10, 34, 63, 6, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
};

int physToGpio_BPI_K230D_ZERO[64] =
{
   -1,
   -1, -1, 12, -1, 11, -1, 5,
   3, -1, 4, 19, 20, 6, -1, 2,
   -1, -1, 10, 16, -1, 17, 34, 15,
   14, -1, 62, 33, 32, 25, -1, 24,
   61, 60, -1, 23, 18, 63, 22, -1,
   21, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
};

#define K230D_ZERO_I2C_DEV "/dev/i2c-2"
#define K230D_ZERO_SPI_DEV "/dev/spidev0.0"

#define K230D_ZERO_I2C_OFFSET -1
#define K230D_ZERO_SPI_OFFSET -1
#define K230D_ZERO_PWM_OFFSET -1

#endif
