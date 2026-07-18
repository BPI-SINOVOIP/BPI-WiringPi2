#ifndef BPI_F4_H
#define BPI_F4_H

/*
 * BPI-F4 exposes GPIOs on several terminal blocks instead of one Raspberry
 * Pi-style header.  WiringPi logical pins 0..19 follow the GPIO-bearing rows
 * in the official 29-row F4 terminal table.  Physical mode uses that table's
 * row number (1..29), not a connector-local pin number.
 */
int pinToGpio_BPI_F4[64] =
{
   84, 85, 71, 70, 81, 80, 83, 82,
   60, 61, 69, 68, 72, 73, 74, 75,
   59, 58, 56, 57,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1,
};

int pinTobcm_BPI_F4[64] =
{
   84, 85, 71, 70, 81, 80, 83, 82,
   60, 61, 69, 68, 72, 73, 74, 75,
   59, 58, 56, 57,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1,
};

int physToGpio_BPI_F4[64] =
{
   -1,
   -1, -1, -1, -1, 84, 85, 71,
   70, -1, 81, 80, 83, 82, -1, 60,
   61, -1, 69, 68, -1, 72, 73, 74,
   75, -1, 59, 58, 56, 57,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1,
};

#define F4_I2C_DEV "/dev/i2c-0"
#define F4_SPI_DEV "/dev/spidev4.0"

#define F4_I2C_OFFSET -1
#define F4_SPI_OFFSET -1
#define F4_PWM_OFFSET -1

#endif
