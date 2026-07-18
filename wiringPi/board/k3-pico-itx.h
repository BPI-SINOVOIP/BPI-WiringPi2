#ifndef K3_PICO_ITX_H
#define K3_PICO_ITX_H

/*
 * SpacemiT K3 Pico-ITX 26-pin, 3.3 V FPC connector.
 *
 * Only the eight application-CPU K3 GPIOs traced through the official
 * schematic are exposed. The RT24-owned signals on this connector and the
 * entire mixed-function 1.8 V 36-pin FPC connector deliberately remain
 * unavailable.
 */
int pinToGpio_K3_PICO_ITX[64] =
{
   21, 22, 28, 29, 31, 32, 33, 34,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
};

int pinTobcm_K3_PICO_ITX[64] =
{
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, 21, 22, -1,
   -1, -1, -1, -1, 28, 29, -1, 31,
   32, 33, 34, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
};

int physToGpio_K3_PICO_ITX[64] =
{
   -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
   21, 22, 28, 29, 31, 32, 33, 34,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1,
};

/* No application-CPU I2C or SPI bus is exposed by the supported 26-pin set. */
#define K3_PICO_ITX_I2C_DEV NULL
#define K3_PICO_ITX_SPI_DEV NULL

#define K3_PICO_ITX_I2C_OFFSET -1
#define K3_PICO_ITX_SPI_OFFSET -1
#define K3_PICO_ITX_PWM_OFFSET -1

#endif
