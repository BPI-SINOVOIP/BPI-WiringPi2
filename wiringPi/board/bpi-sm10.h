#ifndef BPI_SM10_H
#define BPI_SM10_H

/*
 * BPI-SM10 / K3-CoM260 carrier J12 40-pin header.
 *
 * The line numbers below are the K3 global GPIO/pad numbers traced from the
 * official carrier schematic (J12 -> level shifter -> CoM260 SODIMM signal)
 * and the K3 pinctrl definitions.  Pin 18 is not connected to a GPIO in the
 * published schematic and remains unavailable.
 */
int pinToGpio_BPI_SM10[64] =
{
   118, 111, 62, 126, 63, -1, 61, 124,
   83, 82, 107, 70, 104, 105, 106, 121,
   120, -1, -1, -1, -1, 122, 125, 127,
   112, 60, 123, 119, 114, 113, 1, 0,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
};

int pinTobcm_BPI_SM10[64] =
{
   -1, -1, 83, 82, 124, 122, 125, 70,
   107, 105, 104, 106, 123, 127, 121, 120,
   119, 118, 111, 112, 114, 113, 61, 63,
   -1, 62, 60, 126, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
};

int physToGpio_BPI_SM10[64] =
{
   -1,
   -1, -1, 83, -1, 82, -1, 124,
   121, -1, 120, 118, 111, 62, -1, 126,
   63, -1, -1, 104, -1, 105, 61, 106,
   107, -1, 70, 1, 0, 122, -1, 125,
   123, 127, -1, 112, 119, 60, 114, -1,
   113, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
};

#define SM10_I2C_DEV "/dev/i2c-3"
#define SM10_SPI_DEV "/dev/spidev0.0"

#define SM10_I2C_OFFSET -1
#define SM10_SPI_OFFSET -1
#define SM10_PWM_OFFSET -1

#endif
