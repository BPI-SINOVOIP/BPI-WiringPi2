/*
 * wiringPi:
 *	Arduino look-a-like Wiring library for the Raspberry Pi
 *	Copyright (c) 2012-2017 Gordon Henderson
 *	Additional code for pwmSetClock by Chris Hall <chris@kchall.plus.com>
 *
 *	Thanks to code samples from Gert Jan van Loo and the
 *	BCM2835 ARM Peripherals manual, however it's missing
 *	the clock section /grr/mutter/
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as
 *    published by the Free Software Foundation, either version 3 of the
 *    License, or (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with wiringPi.
 *    If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

// Revisions:
//	19 Jul 2012:
//		Moved to the LGPL
//		Added an abstraction layer to the main routines to save a tiny
//		bit of run-time and make the clode a little cleaner (if a little
//		larger)
//		Added waitForInterrupt code
//		Added piHiPri code
//
//	 9 Jul 2012:
//		Added in support to use the /sys/class/gpio interface.
//	 2 Jul 2012:
//		Fixed a few more bugs to do with range-checking when in GPIO mode.
//	11 Jun 2012:
//		Fixed some typos.
//		Added c++ support for the .h file
//		Added a new function to allow for using my "pin" numbers, or native
//			GPIO pin numbers.
//		Removed my busy-loop delay and replaced it with a call to delayMicroseconds
//
//	02 May 2012:
//		Added in the 2 UART pins
//		Change maxPins to numPins to more accurately reflect purpose

#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>
#include <poll.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>

#include "softPwm.h"
#include "softTone.h"

#include "wiringPi.h"
#include "../version.h"
#include "bpi-gpio.h"

/* define in wiringPi.c */
extern int wiringPiMode ;
extern int wiringPiDebug ;
extern int *pinToGpio ;
extern int *physToGpio ;
extern int sysFds [] ;
extern volatile uint32_t *gpio ;
extern volatile uint32_t *pwm ;
extern volatile uint32_t *clk ;
extern volatile uint32_t *pads ;
#define BLOCK_SIZE              (4*1024)
extern void initialiseEpoch ();
#if 0
/* define in OLD wiringPi.h */
#define I2C_PIN		7
#define SPI_PIN		8
#define PULLUP		5
#define PULLDOWN	6
#define PULLOFF		7
#endif

//#define	D	printf("__%d__:(%s:%s)\n",__LINE__,__FILE__,__FUNCTION__);

//for M2 PL and PM group gpios
static volatile uint32_t *gpio_lm;
int *pinToGpio_BP ;
int *physToGpio_BP ;
int *pinTobcm_BP ;

// Add for Banana Pi
/* for mmap bananapi */
#define	MAX_PIN_NUM		      (0x40)  //64

//sunxi_gpio
#define SUNXI_GPIO_BASE       (0x01c20800)
#define SUNXI_GPIO_LM_BASE    (0x01f02c00)
#define SUN50IW9_GPIO_BASE    (0x0300B000)
#define MAP_SIZE	          (4096*2)
#define MAP_MASK	          (MAP_SIZE - 1)

//mt7623 gpio
#define MTK_GPIO_BASE_BP      (0x10005000)
#define MTK_GPIO_DIR          (0x00)
#define MTK_GPIO_PULLE        (0x150)
#define MTK_GPIO_PULLSEL      (0x280)
#define MTK_GPIO_DOUT         (0x500)
#define MTK_GPIO_DIN          (0x630)
#define MTK_GPIO_MODE         (0x760)
#define MTK_GPIO_MAP_SIZE     (8 * 1024)
#define MTK_GPIO_MODE_PINS_PER_REG   5
#define MTK_GPIO_FIELD_PINS_PER_REG  16

// MT7988/MT7986 pinctrl GPIO register layout from mainline/vendor DTS.
#define MTK_V2_GPIO_BASE_BP          (0x1001F000)
#define MTK_V2_GPIO_DIR              (0x00)
#define MTK_V2_GPIO_DOUT             (0x100)
#define MTK_V2_GPIO_DIN              (0x200)
#define MTK_V2_GPIO_MODE             (0x300)
#define MTK_V2_GPIO_MAP_SIZE         (4 * 1024)
#define MTK_V2_MODE_PINS_PER_REG     8
#define MTK_V2_MODE_BITS             4
#define MTK_V2_FIELD_PINS_PER_REG    32

// MT7622 pinctrl GPIO register layout from mainline pinctrl-mt7622.
#define MTK_MT7622_GPIO_BASE_BP      (0x10211000)
#define MTK_MT7622_GPIO_DIR          (0x00)
#define MTK_MT7622_GPIO_DOUT         (0x100)
#define MTK_MT7622_GPIO_DIN          (0x200)
#define MTK_MT7622_GPIO_MAP_SIZE     (4 * 1024)
#define MTK_MT7622_FIELD_PINS_PER_REG 32

//Amlogic Meson G12B/SM1 GPIO
#define MESON_GPIO_BASE_BP        (0xFF634000)
#define MESON_GPIO_AO_BASE_BP     (0xFF800000)
#define MESON_GPIO_PIN_BASE       410
#define MESON_GPIOH_PIN_START     (MESON_GPIO_PIN_BASE + 17)
#define MESON_GPIOH_PIN_END       (MESON_GPIO_PIN_BASE + 25)
#define MESON_GPIOA_PIN_START     (MESON_GPIO_PIN_BASE + 50)
#define MESON_GPIOA_PIN_END       (MESON_GPIO_PIN_BASE + 65)
#define MESON_GPIOX_PIN_START     (MESON_GPIO_PIN_BASE + 66)
#define MESON_GPIOX_PIN_MID       (MESON_GPIO_PIN_BASE + 81)
#define MESON_GPIOX_PIN_END       (MESON_GPIO_PIN_BASE + 85)
#define MESON_GPIOAO_PIN_START    (MESON_GPIO_PIN_BASE + 86)
#define MESON_GPIOAO_PIN_END      (MESON_GPIO_PIN_BASE + 97)

#define MESON_GPIOH_FSEL_REG_OFFSET    0x119
#define MESON_GPIOH_OUTP_REG_OFFSET    0x11A
#define MESON_GPIOH_INP_REG_OFFSET     0x11B
#define MESON_GPIOH_PUPD_REG_OFFSET    0x13D
#define MESON_GPIOH_PUEN_REG_OFFSET    0x14B
#define MESON_GPIOH_MUX_B_REG_OFFSET   0x1BB

#define MESON_GPIOA_FSEL_REG_OFFSET    0x120
#define MESON_GPIOA_OUTP_REG_OFFSET    0x121
#define MESON_GPIOA_INP_REG_OFFSET     0x122
#define MESON_GPIOA_PUPD_REG_OFFSET    0x13F
#define MESON_GPIOA_PUEN_REG_OFFSET    0x14D
#define MESON_GPIOA_MUX_D_REG_OFFSET   0x1BD
#define MESON_GPIOA_MUX_E_REG_OFFSET   0x1BE

#define MESON_GPIOX_FSEL_REG_OFFSET    0x116
#define MESON_GPIOX_OUTP_REG_OFFSET    0x117
#define MESON_GPIOX_INP_REG_OFFSET     0x118
#define MESON_GPIOX_PUPD_REG_OFFSET    0x13C
#define MESON_GPIOX_PUEN_REG_OFFSET    0x14A
#define MESON_GPIOX_MUX_3_REG_OFFSET   0x1B3
#define MESON_GPIOX_MUX_4_REG_OFFSET   0x1B4
#define MESON_GPIOX_MUX_5_REG_OFFSET   0x1B5

#define MESON_GPIOAO_FSEL_REG_OFFSET   0x109
#define MESON_GPIOAO_OUTP_REG_OFFSET   0x10D
#define MESON_GPIOAO_INP_REG_OFFSET    0x10A
#define MESON_GPIOAO_PUPD_REG_OFFSET   0x10B
#define MESON_GPIOAO_PUEN_REG_OFFSET   0x10C
#define MESON_GPIOAO_MUX_REG0_OFFSET   0x105
#define MESON_GPIOAO_MUX_REG1_OFFSET   0x106

//SpacemiT K1 GPIO
#define SPACEMIT_GPIO_BASE_BP          (0xD4019000)
#define SPACEMIT_PINCTRL_BASE_BP       (0xD401E000)
#define SPACEMIT_GPIO_PIN_BASE         0
#define SPACEMIT_GPIO_PIN_END          127

#define SPACEMIT_BANK012_OFFSET(x)     ((x) << 2)
#define SPACEMIT_BANK3_OFFSET          0x100

#define SPACEMIT_GPLR                  0x0
#define SPACEMIT_GPDR                  0xC
#define SPACEMIT_GPSR                  0x18
#define SPACEMIT_GPCR                  0x24
#define SPACEMIT_GSDR                  0x54
#define SPACEMIT_GCDR                  0x60

#define SPACEMIT_AF_SEL_OFFSET         0
#define SPACEMIT_AF_SEL_MASK           (7 << 0)
#define SPACEMIT_PULL_DIS              0
#define SPACEMIT_PULL_UP               6
#define SPACEMIT_PULL_DOWN             5
#define SPACEMIT_PULL_OFFSET           13
#define SPACEMIT_PULL_MASK             (7 << 13)

// Renesas RZ/V2N GPIO
#define RENESAS_GPIO_BASE_BP           (0x10410000)
#define RENESAS_GPIO_PIN_BASE          416
#define RENESAS_GPIO_PIN_END           511
#define RENESAS_GPIO_MAP_SIZE          (8 * 1024)

#define RENESAS_PINS_PER_PORT          8
#define RENESAS_EXTENDED_REG_OFFSET    0x10
#define RENESAS_PIN_OFFSET(pin)        ((pin) - RENESAS_GPIO_PIN_BASE)
#define RENESAS_PIN_ID_TO_PORT(n)      ((n) / RENESAS_PINS_PER_PORT)
#define RENESAS_PIN_ID_TO_PORT_OFFSET(n) \
  (RENESAS_PIN_ID_TO_PORT(n) + RENESAS_EXTENDED_REG_OFFSET)
#define RENESAS_PIN_ID_TO_PIN(n)       ((n) % RENESAS_PINS_PER_PORT)

#define RENESAS_P(n)                   (0x0000 + 0x10 + (n))
#define RENESAS_PM(n)                  (0x0100 + 0x20 + (n) * 2)
#define RENESAS_PMC(n)                 (0x0200 + 0x10 + (n))
#define RENESAS_PFC(n)                 (0x0400 + 0x40 + (n) * 4)
#define RENESAS_PIN(n)                 (0x0800 + 0x10 + (n))
#define RENESAS_PUPD(n)                (0x1C00 + (n) * 8)

#define RENESAS_PM_INPUT               0x1
#define RENESAS_PM_OUTPUT              0x2
#define RENESAS_PM_HIZ                 0x12

#define RENESAS_PULL_DIS               0x0
#define RENESAS_PULL_UP                0x3
#define RENESAS_PULL_DOWN              0x2

// Rockchip GPIO
#define ROCKCHIP_GPIO_BANKS            5
#define ROCKCHIP_GPIO_PIN_BASE         0
#define ROCKCHIP_GPIO_PIN_END          159
#define ROCKCHIP_GPIO_MAP_SIZE_RK3308  0x100
#define ROCKCHIP_GPIO_MAP_SIZE_RK3568  0x100
#define ROCKCHIP_GPIO_MAP_SIZE_RK3528  0x200
#define ROCKCHIP_GPIO_MAP_SIZE_RK3506  0x200
#define ROCKCHIP_GPIO_MAP_SIZE_RK3576  0x200
#define ROCKCHIP_GPIO_MAP_SIZE_RK3588  0x100

#define ROCKCHIP_GPIO_SWPORT_DR_V1     0x00
#define ROCKCHIP_GPIO_SWPORT_DDR_V1    0x04
#define ROCKCHIP_GPIO_EXT_PORT_V1      0x50
#define ROCKCHIP_GPIO_SWPORT_DR_V2     0x00
#define ROCKCHIP_GPIO_SWPORT_DDR_V2    0x08
#define ROCKCHIP_GPIO_EXT_PORT_V2      0x70

// Realtek RTD129x GPIO
#define REALTEK_GPIO_GROUPS            2
#define REALTEK_GPIO_MAP_SIZE          0x100
#define REALTEK_RTD129X_MISC_BASE      0x9801b100
#define REALTEK_RTD129X_ISO_BASE       0x98007100
#define REALTEK_RTD139X_ISO_BASE       0x98007100
#define REALTEK_RTD129X_MISC_PIN_BASE  0
#define REALTEK_RTD129X_MISC_PIN_END   100
#define REALTEK_RTD129X_ISO_PIN_BASE   101
#define REALTEK_RTD129X_ISO_PIN_END    135
#define REALTEK_RTD139X_ISO_PIN_BASE   0
#define REALTEK_RTD139X_ISO_PIN_END    56

// Synaptics VS680 DW APB GPIO
#define VS680_GPIO_BANKS               4
#define VS680_GPIO_MAP_SIZE            0x400
#define VS680_GPIO_SOC_PIN_BASE        0
#define VS680_GPIO_SOC_PIN_END         95
#define VS680_GPIO_SM_PIN_BASE         96
#define VS680_GPIO_SM_PIN_END          127
#define VS680_GPIO_SWPORT_DR           0x00
#define VS680_GPIO_SWPORT_DDR          0x04
#define VS680_GPIO_EXT_PORT            0x50

// Sunplus SP7021 GPIO
#define SP7021_GPIO_PIN_BASE           0
#define SP7021_GPIO_PIN_END            98
#define SP7021_GPIO_MAP_SIZE           0x1000
#define SP7021_GPIO_PAGE0_BASE         0x9C000000
#define SP7021_GPIO_PAGE2_BASE         0x9C003000
#define SP7021_GPIO_BASE0_OFFSET       0x300
#define SP7021_GPIO_BASE1_OFFSET       0x380
#define SP7021_GPIO_BASE2_OFFSET       0x2e4

#define SP7021_GPIO_GFR                0x00
#define SP7021_GPIO_CTL                0x00
#define SP7021_GPIO_OE                 0x20
#define SP7021_GPIO_OUT                0x40
#define SP7021_GPIO_IN                 0x60
#define SP7021_GPIO_IINV               0x00
#define SP7021_GPIO_OINV               0x20
#define SP7021_GPIO_OD                 0x40

#define SP7021_R16_ROF(r)              (((r) >> 4) << 2)
#define SP7021_R16_BOF(r)              ((r) & 0x0f)
#define SP7021_R32_ROF(r)              (((r) >> 5) << 2)
#define SP7021_R32_BOF(r)              ((r) & 0x1f)

// Sunplus SP7350 GPIO (BPI-F4)
#define SP7350_GPIO_PIN_BASE           0
#define SP7350_GPIO_PIN_END            105
#define SP7350_GPIO_MAP_SIZE           0x1000
#define SP7350_GPIO_PAGE_BASE          0xF8803000
#define SP7350_GPIO_FIRST_OFFSET       0x2e4
#define SP7350_GPIO_GPIOXT_OFFSET      0x380
#define SP7350_GPIO_FIRST              0x00
#define SP7350_GPIO_CTL                0x00
#define SP7350_GPIO_OE                 0x34
#define SP7350_GPIO_OUT                0x68
#define SP7350_GPIO_IN                 0x9c
#define SP7350_R16_ROF(r)              (((r) >> 4) << 2)
#define SP7350_R16_BOF(r)              ((r) & 0x0f)
#define SP7350_R32_ROF(r)              (((r) >> 5) << 2)
#define SP7350_R32_BOF(r)              ((r) & 0x1f)

struct realtek_gpio_group
{
  int pin_base;
  int pin_end;
  int map_index;
  const int *dir_offset;
  const int *dato_offset;
  const int *dati_offset;
};

//sunxi_pwm, only use ch0
#define SUNXI_PWM_BASE        (0x01c21400)
#define SUNXI_PWM_CH0_CTRL    (SUNXI_PWM_BASE)
#define SUNXI_PWM_CH0_PERIOD  (SUNXI_PWM_BASE + 0x04)

//each channel use the same offset bit
#define SUNXI_PWM_CH0_EN			(1 << 4)
#define SUNXI_PWM_CH0_ACT_STA		(1 << 5)
#define SUNXI_PWM_SCLK_CH0_GATING	(1 << 6)
#define SUNXI_PWM_CH0_MS_MODE		(1 << 7) //pulse mode
#define SUNXI_PWM_CH0_PUL_START		(1 << 8)

#define PWM_CLK_DIV_120 	0
#define PWM_CLK_DIV_180		1
#define PWM_CLK_DIV_240		2
#define PWM_CLK_DIV_360		3
#define PWM_CLK_DIV_480		4
#define PWM_CLK_DIV_12K		8
#define PWM_CLK_DIV_24K		9
#define PWM_CLK_DIV_36K		10
#define PWM_CLK_DIV_48K		11


//addr should 4K*n
//#define GPIO_BASE_BP		(SUNXI_GPIO_BASE)
#define GPIO_BASE_LM_BP		(0x01f02000)   
#define GPIO_BASE_BP        (0x01C20000)
#define GPIO_PWM_BP		    (0x01c21000)  //need 4k*n

#define GPIO_PADS_BP		(0x00100000)
#define CLOCK_BASE_BP		(0x00101000)
#define GPIO_TIMER_BP		(0x0000B000)

struct mtk_mt7622_pin_field_calc
{
  int s_pin;
  int e_pin;
  unsigned int s_addr;
  unsigned int x_addrs;
  unsigned int s_bit;
  unsigned int x_bits;
  int fixed;
};

static const struct mtk_mt7622_pin_field_calc mtk_mt7622_mode_ranges[] =
{
  {0, 0, 0x320, 0x10, 16, 4, 0},
  {1, 4, 0x3a0, 0x10, 16, 4, 0},
  {5, 5, 0x320, 0x10, 0, 4, 0},
  {6, 7, 0x300, 0x10, 4, 4, 1},
  {8, 9, 0x350, 0x10, 20, 4, 0},
  {10, 13, 0x300, 0x10, 8, 4, 1},
  {14, 15, 0x320, 0x10, 4, 4, 0},
  {16, 17, 0x320, 0x10, 20, 4, 0},
  {18, 21, 0x310, 0x10, 16, 4, 0},
  {22, 22, 0x380, 0x10, 16, 4, 0},
  {23, 24, 0x300, 0x10, 24, 4, 1},
  {25, 36, 0x300, 0x10, 12, 4, 1},
  {37, 50, 0x300, 0x10, 20, 4, 1},
  {51, 70, 0x330, 0x10, 4, 4, 0},
  {71, 72, 0x300, 0x10, 16, 4, 1},
  {73, 76, 0x310, 0x10, 0, 4, 0},
  {77, 77, 0x320, 0x10, 28, 4, 0},
  {78, 78, 0x320, 0x10, 12, 4, 0},
  {79, 82, 0x3a0, 0x10, 0, 4, 0},
  {83, 83, 0x350, 0x10, 28, 4, 0},
  {84, 84, 0x330, 0x10, 0, 4, 0},
  {85, 90, 0x360, 0x10, 4, 4, 0},
  {91, 94, 0x390, 0x10, 16, 4, 0},
  {95, 97, 0x380, 0x10, 20, 4, 0},
  {98, 101, 0x390, 0x10, 0, 4, 0},
  {102, 102, 0x360, 0x10, 0, 4, 0},
};

static const struct mtk_mt7622_pin_field_calc mtk_mt7622_pu_ranges[] =
{
  {0, 31, 0x930, 0x10, 0, 1, 0},
  {32, 50, 0xa30, 0x10, 0, 1, 0},
  {51, 70, 0x830, 0x10, 0, 1, 0},
  {71, 72, 0xb30, 0x10, 0, 1, 0},
  {73, 86, 0xb30, 0x10, 4, 1, 0},
  {87, 90, 0xc30, 0x10, 0, 1, 0},
  {91, 102, 0xb30, 0x10, 18, 1, 0},
};

static const struct mtk_mt7622_pin_field_calc mtk_mt7622_pd_ranges[] =
{
  {0, 31, 0x940, 0x10, 0, 1, 0},
  {32, 50, 0xa40, 0x10, 0, 1, 0},
  {51, 70, 0x840, 0x10, 0, 1, 0},
  {71, 72, 0xb40, 0x10, 0, 1, 0},
  {73, 86, 0xb40, 0x10, 4, 1, 0},
  {87, 90, 0xc40, 0x10, 0, 1, 0},
  {91, 102, 0xb40, 0x10, 18, 1, 0},
};

static int wiringPinMode = WPI_MODE_UNINITIALISED ;
static int bpi_found_mtk = 0 ;
static int bpi_found_mtk_v2 = 0 ;
static int bpi_found_mtk_mt7622 = 0 ;
static int bpi_found_sun50iw9 = 0 ;
static int bpi_found_meson = 0 ;
static int bpi_found_spacemit = 0 ;
static int bpi_found_renesas = 0 ;
static int bpi_found_rockchip = 0 ;
static int bpi_found_realtek = 0 ;
static int bpi_found_vs680 = 0 ;
static int bpi_found_sp7021 = 0 ;
static int bpi_found_sp7350 = 0 ;
static uint8_t *mtk_gpio_base = NULL ;
static uint8_t *mtk_v2_gpio_base = NULL ;
static uint8_t *mtk_mt7622_gpio_base = NULL ;
static volatile uint32_t *meson_gpio = NULL ;
static volatile uint32_t *meson_gpioao = NULL ;
static volatile uint32_t *spacemit_gpio = NULL ;
static volatile uint32_t *spacemit_pinctrl = NULL ;
static volatile uint32_t *renesas_gpio = NULL ;
static volatile uint32_t *rockchip_gpio[ROCKCHIP_GPIO_BANKS] = { NULL };
static volatile uint32_t *realtek_gpio[REALTEK_GPIO_GROUPS] = { NULL };
static volatile uint32_t *vs680_gpio[VS680_GPIO_BANKS] = { NULL };
static volatile uint32_t *sp7021_gpio_page0 = NULL ;
static volatile uint32_t *sp7021_gpio_page2 = NULL ;
static volatile uint32_t *sp7021_gpio_base0 = NULL ;
static volatile uint32_t *sp7021_gpio_base1 = NULL ;
static volatile uint32_t *sp7021_gpio_base2 = NULL ;
static volatile uint32_t *sp7350_gpio_page = NULL ;
static volatile uint32_t *sp7350_gpio_first = NULL ;
static volatile uint32_t *sp7350_gpio_gpioxt = NULL ;
static const off_t rockchip_gpio_base_rk3308[ROCKCHIP_GPIO_BANKS] = {
  0xff220000,
  0xff230000,
  0xff240000,
  0xff250000,
  0xff260000,
};
static const off_t rockchip_gpio_base_rk3568[ROCKCHIP_GPIO_BANKS] = {
  0xfdd60000,
  0xfe740000,
  0xfe750000,
  0xfe760000,
  0xfe770000,
};
static const off_t rockchip_gpio_base_rk3528[ROCKCHIP_GPIO_BANKS] = {
  0xff610000,
  0xffaf0000,
  0xffb00000,
  0xffb10000,
  0xffb20000,
};
static const off_t rockchip_gpio_base_rk3506[ROCKCHIP_GPIO_BANKS] = {
  0xff940000,
  0xff870000,
  0xff1c0000,
  0xff1d0000,
  0xff1e0000,
};
static const off_t rockchip_gpio_base_rk3576[ROCKCHIP_GPIO_BANKS] = {
  0x27320000,
  0x2ae10000,
  0x2ae20000,
  0x2ae30000,
  0x2ae40000,
};
static const off_t rockchip_gpio_base_rk3588[ROCKCHIP_GPIO_BANKS] = {
  0xfd8a0000,
  0xfec20000,
  0xfec30000,
  0xfec40000,
  0xfec50000,
};
static const off_t *rockchip_gpio_base = rockchip_gpio_base_rk3568;
static size_t rockchip_gpio_map_size = ROCKCHIP_GPIO_MAP_SIZE_RK3568;
static int rockchip_gpio_v2 = 1;
static int rockchip_gpio_swport_dr = ROCKCHIP_GPIO_SWPORT_DR_V2;
static int rockchip_gpio_swport_ddr = ROCKCHIP_GPIO_SWPORT_DDR_V2;
static int rockchip_gpio_ext_port = ROCKCHIP_GPIO_EXT_PORT_V2;
static const int realtek_rtd129x_misc_dir[4] = { 0x00, 0x04, 0x08, 0x0c };
static const int realtek_rtd129x_misc_dato[4] = { 0x10, 0x14, 0x18, 0x1c };
static const int realtek_rtd129x_misc_dati[4] = { 0x20, 0x24, 0x28, 0x2c };
static const int realtek_rtd129x_iso_dir[4] = { 0x00, 0x18, 0x00, 0x00 };
static const int realtek_rtd129x_iso_dato[4] = { 0x04, 0x1c, 0x00, 0x00 };
static const int realtek_rtd129x_iso_dati[4] = { 0x08, 0x20, 0x00, 0x00 };
static const struct realtek_gpio_group realtek_rtd129x_groups[REALTEK_GPIO_GROUPS] = {
  {
    REALTEK_RTD129X_MISC_PIN_BASE,
    REALTEK_RTD129X_MISC_PIN_END,
    0,
    realtek_rtd129x_misc_dir,
    realtek_rtd129x_misc_dato,
    realtek_rtd129x_misc_dati,
  },
  {
    REALTEK_RTD129X_ISO_PIN_BASE,
    REALTEK_RTD129X_ISO_PIN_END,
    1,
    realtek_rtd129x_iso_dir,
    realtek_rtd129x_iso_dato,
    realtek_rtd129x_iso_dati,
  },
};
static const off_t realtek_gpio_base_rtd129x[REALTEK_GPIO_GROUPS] = {
  REALTEK_RTD129X_MISC_BASE,
  REALTEK_RTD129X_ISO_BASE,
};
static const struct realtek_gpio_group realtek_rtd139x_groups[1] = {
  {
    REALTEK_RTD139X_ISO_PIN_BASE,
    REALTEK_RTD139X_ISO_PIN_END,
    0,
    realtek_rtd129x_iso_dir,
    realtek_rtd129x_iso_dato,
    realtek_rtd129x_iso_dati,
  },
};
static const off_t realtek_gpio_base_rtd139x[REALTEK_GPIO_GROUPS] = {
  REALTEK_RTD139X_ISO_BASE,
  0,
};
static const off_t vs680_gpio_base[VS680_GPIO_BANKS] = {
  0xf7e82400,
  0xf7e80800,
  0xf7e80c00,
  0xf7fc8000,
};
static const struct realtek_gpio_group *realtek_gpio_groups = realtek_rtd129x_groups;
static const off_t *realtek_gpio_base = realtek_gpio_base_rtd129x;
static int realtek_gpio_group_count = REALTEK_GPIO_GROUPS;
static size_t realtek_gpio_map_size = REALTEK_GPIO_MAP_SIZE;


static int syspin [64] =
{
  -1, -1, 2, 3, 4, 5, 6, 7,   //GPIO0,1 used to I2C
  8, 9, 10, 11, 12,13, 14, 15,
  16, 17, 18, 19, 20, 21, 22, 23,
  24, 25, 26, 27, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
} ;

static int edge [64] =
{
  -1, -1, -1, -1, 4, -1, -1, 7, 
  8, 9, 10, 11, -1,-1, 14, 15,
  -1, 17, -1, -1, -1, -1, 22, 23,
  24, 25, -1, 27, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
} ;

//static int version=0;
static int pwmmode=0;

static int bpi_wiringPiSetupRegOffset(int mode);

static uint32_t sunxi_gpio_phyaddr(int bank, uint32_t offset)
{
  if (bpi_found_sun50iw9)
    return SUN50IW9_GPIO_BASE + (bank * 36) + offset;

  if (bank >= 11)
    return SUNXI_GPIO_LM_BASE + ((bank - 11) * 36) + offset;

  return SUNXI_GPIO_BASE + (bank * 36) + offset;
}

/**
 *A20 Tools for Banana Pi 
 */

void sunxi_gpio_unexports(void)
{
  FILE *fd ;
  int i, pin;

  if (wiringPiDebug)
	printf("%s\n", __func__);

  wiringPiSetup();
  
  for (i = 0 ; i < 32 ; ++i) 
  {
    if ((i & PI_GPIO_MASK) == 0)    // On-board pin
    {
      if (wiringPiMode == WPI_MODE_PINS)
       	pin = pinToGpio_BP [i] ;
      else if (wiringPiMode == WPI_MODE_PHYS)
      	pin = physToGpio_BP [i] ;
      else if (wiringPiMode == WPI_MODE_GPIO)
      	pin= pinTobcm_BP[i];//need map A20 to bcm
      else 
	  	return;

	  if (wiringPiDebug)
	    printf("%s, i= %d, pin = %d\n", __func__, i, pin);

	  if (-1 == pin)  /*VCC or GND return directly*/
  	  {
  		//printf("%s, the pin:%d is invaild,please check it over!\n", __func__, pin);
  		continue;
  	  }
    }
  
    if ((fd = fopen ("/sys/class/gpio/unexport", "w")) == NULL)
    {
      fprintf (stderr, "Unable to open GPIO export interface\n") ;
      exit (1) ;
    }

	if (wiringPiDebug)
	    printf("%s, i= %d, pin = %d\n", __func__, i, pin);
	
    fprintf (fd, "%d\n", pin) ;
    fclose (fd) ;
  }
}

void sunxi_gpio_exports(void)
{
  int fd ;
  int i, l, first, pin;
  char fName [128] ;
  char buf [16] ;

  if (wiringPiDebug)
	printf("%s\n", __func__);

  wiringPiSetup();

  for(first = 0, i = 0; i < 32; i++)
  {
    if ((i & PI_GPIO_MASK) == 0)    // On-board pin
    {
      if (wiringPiMode == WPI_MODE_PINS)
       	pin = pinToGpio_BP [i] ;
      else if (wiringPiMode == WPI_MODE_PHYS)
      	pin = physToGpio_BP [i] ;
      else if (wiringPiMode == WPI_MODE_GPIO)
      	pin= pinTobcm_BP[i];//need map A20 to bcm
      else 
	  	return;

	  if (wiringPiDebug)
	    printf("%s, i= %d, pin = %d\n", __func__, i, pin);

	  if (-1 == pin)  /*VCC or GND return directly*/
  	  {
  		//printf("%s, the pin:%d is invaild,please check it over!\n", __func__, pin);
  		continue;
  	  }
    }

    // Try to read the direction
    sprintf (fName, "/sys/class/gpio/gpio%d/direction", pin) ;
    if ((fd = open (fName, O_RDONLY)) == -1)
      continue ;

    if (first == 0)
    {
      ++first ;
      printf("GPIO Pins exported:\n") ;
    }

    printf("%d(BP=%d): ", i, pin) ;

    if ((l = read (fd, buf, 16)) == 0)
      sprintf(buf, "%s", "?") ;
 
    buf [l] = 0 ;
    if ((buf [strlen (buf) - 1]) == '\n')
      buf [strlen (buf) - 1] = 0 ;

    printf("direction=%-3s  ", buf) ;

    close (fd) ;

    // Try to Read the value
    sprintf (fName, "/sys/class/gpio/gpio%d/value", pin) ;
    if ((fd = open (fName, O_RDONLY)) == -1)
    {
      printf ("No Value file (huh?)\n") ;
      continue ;
    }

    if ((l = read (fd, buf, 16)) == 0)
      sprintf (buf, "%s", "?") ;

    buf [l] = 0 ;
    if ((buf [strlen (buf) - 1]) == '\n')
      buf [strlen (buf) - 1] = 0 ;

    printf("value=%s  ", buf) ;

    // Read any edge trigger file
    sprintf (fName, "/sys/class/gpio/gpio%d/edge", pin) ;
    if ((fd = open (fName, O_RDONLY)) == -1)
    {
      printf ("\n") ;
      continue ;
    }

    if ((l = read (fd, buf, 16)) == 0)
      sprintf (buf, "%s", "?") ;

    buf [l] = 0 ;
    if ((buf [strlen (buf) - 1]) == '\n')
      buf [strlen (buf) - 1] = 0 ;

    printf("edge=%-8s\n", buf) ;

    close (fd) ;
	
  }
}


#if 0
/**
 * [readl read with an address]
 * @param  addr [address]
 * @return      [value]
 */
uint32_t readl(uint32_t addr)
{
  uint32_t val = 0;
  uint32_t mmap_base = (addr & ~MAP_MASK);
  uint32_t mmap_seek = ((addr - mmap_base) >> 2);

  val = *(gpio + mmap_seek);

  return val;
}

/**
 * [writel write with an address]
 * @param val  [value]
 * @param addr [address]
 */
void writel(uint32_t val, uint32_t addr)
{
  uint32_t mmap_base = (addr & ~MAP_MASK);
  uint32_t mmap_seek = ((addr - mmap_base) >> 2);

  *(gpio + mmap_seek) = val;
}
#endif

uint32_t sunxi_pwm_readl(uint32_t addr)
{
  uint32_t val = 0;
  uint32_t mmap_base = (addr & ~MAP_MASK);
  uint32_t mmap_seek = ((addr - mmap_base) >> 2);

  val = *(pwm + mmap_seek);

  return val;
}

void sunxi_pwm_writel(uint32_t val, uint32_t addr)
{
  uint32_t mmap_base = (addr & ~MAP_MASK);
  uint32_t mmap_seek = ((addr - mmap_base) >> 2);

  *(pwm + mmap_seek) = val;
}

uint32_t sunxi_gpio_readl(uint32_t addr, int bank)
{
  uint32_t val = 0;
  uint32_t mmap_base = (addr & ~MAP_MASK);
  uint32_t mmap_seek = ((addr - mmap_base) >> 2);

  /* DK, for PL and PM */
  if(!bpi_found_sun50iw9 && bank >= 11)
      val = *(gpio_lm+ mmap_seek);
  else
      val = *(gpio + mmap_seek);

  return val;
}

void sunxi_gpio_writel(uint32_t val, uint32_t addr, int bank)
{
  uint32_t mmap_base = (addr & ~MAP_MASK);
  uint32_t mmap_seek = ((addr - mmap_base) >> 2);

  if(!bpi_found_sun50iw9 && bank >= 11)
      *(gpio_lm+ mmap_seek) = val;
  else
      *(gpio + mmap_seek) = val;
}

void sunxi_pwm_set_enable(int en)
{
  int val = 0;
  uint32_t pwm_ch_addr=0;

  pwm_ch_addr = SUNXI_PWM_CH0_CTRL;
  
  val = sunxi_pwm_readl(pwm_ch_addr);
  if(en)
  {
	val |= (SUNXI_PWM_CH0_EN | SUNXI_PWM_SCLK_CH0_GATING);
  } 
  else 
  {
	val &= ~(SUNXI_PWM_CH0_EN | SUNXI_PWM_SCLK_CH0_GATING);
  }
  
  if (wiringPiDebug)
	printf(">>function%s,no:%d,enable? :0x%x\n",__func__, __LINE__, val);
  
  sunxi_pwm_writel(val, pwm_ch_addr);
  delay (1) ;
}

void sunxi_pwm_set_mode(int mode)
{
  int val = 0;
  uint32_t pwm_ch_addr=0;

  pwm_ch_addr = SUNXI_PWM_CH0_CTRL;
	 
  val = sunxi_pwm_readl(pwm_ch_addr);
  mode &= 1; //cover the mode to 0 or 1
  if(mode)
  { //pulse mode
    val |= ( SUNXI_PWM_CH0_MS_MODE|SUNXI_PWM_CH0_PUL_START);
    pwmmode=1;
  }
  else 
  {  //cycle mode
    val &= ~( SUNXI_PWM_CH0_MS_MODE);
    pwmmode=0;
  }
  
  val |= ( SUNXI_PWM_CH0_ACT_STA);
  
  if (wiringPiDebug)
	printf("%s, %d, mode = 0x%x\n",__func__, __LINE__, val);
  
  sunxi_pwm_writel(val, pwm_ch_addr);

  delay (1) ;
	
  val = sunxi_pwm_readl(pwm_ch_addr);
  
  if (wiringPiDebug)
    printf("%s after set, mode: %d, phyaddr:0x%x\n",__func__, val, pwm_ch_addr);
}

void sunxi_pwm_set_clk(int clk)
{
  int val = 0;
  uint32_t pwm_ch_addr=0;

  pwm_ch_addr = SUNXI_PWM_CH0_CTRL;
  
  val = sunxi_pwm_readl(pwm_ch_addr);

  //clear clk to 0
  val &= 0xfffffff0;
  val |= ((clk & 0xf) << 0);  //todo check wether clk is invalid or not
  sunxi_pwm_writel(val, pwm_ch_addr);
	 
  if (wiringPiDebug)
	printf(">>function%s,no:%d,clk? :0x%x\n",__func__, __LINE__, val);
	 
  delay (1) ;
}

/**
 * ch0 and ch1 set the same,16 bit period and 16 bit act
 */
uint32_t sunxi_pwm_get_period()
{
  uint32_t period_cys = 0;
  uint32_t pwm_ch_addr=0;

  pwm_ch_addr = SUNXI_PWM_CH0_PERIOD;

  period_cys = sunxi_pwm_readl(pwm_ch_addr);
  period_cys &= 0xffff0000;
  period_cys = period_cys >> 16;

  if (wiringPiDebug)
    printf(">>func:%s,no:%d, period/range: %d\n",__func__,__LINE__, period_cys);
  
  delay (1);
  return period_cys;
}

void sunxi_pwm_set_period(int period_cys)
{
  uint32_t val = 0;
  uint32_t pwm_ch_addr=0;

  pwm_ch_addr = SUNXI_PWM_CH0_PERIOD;

  if (wiringPiDebug)
    printf("%s before set, period/range: %d, phyaddr:0x%x\n",__func__, period_cys, pwm_ch_addr);

  period_cys &= 0xffff; //set max period to 2^16
  period_cys = period_cys << 16;
  val = sunxi_pwm_readl(pwm_ch_addr);
  val &=0x0000ffff;
  val |= period_cys;
  sunxi_pwm_writel(val, pwm_ch_addr);

  delay (10) ;
  
  val = sunxi_pwm_readl(pwm_ch_addr);//get ch1 period_cys
  val &= 0xffff0000;//get period_cys
  val = val >> 16;

  if (wiringPiDebug)
    printf("%s after set, period/range: %d, phyaddr:0x%x\n",__func__, val, pwm_ch_addr);
  
}


uint32_t sunxi_pwm_get_act(void)
{
  uint32_t period_act = 0;

  period_act = sunxi_pwm_readl(SUNXI_PWM_CH0_PERIOD);//get ch1 period_cys
  period_act &= 0xffff;//get period_act

  if (wiringPiDebug)
    printf(">>func:%s,no:%d,period/range:%d",__func__,__LINE__,period_act);
  delay (1) ;

  return period_act;
}

void sunxi_pwm_set_act(int act_cys)
{
  uint32_t per0 = 0;
  uint32_t pwm_ch_addr=0;
  
  //keep period the same, clear act_cys to 0 first
  if (wiringPiDebug)
    printf(">>func:%s no:%d\n",__func__,__LINE__);

  pwm_ch_addr = SUNXI_PWM_CH0_PERIOD;

  act_cys &= 0xffff;
  per0 = sunxi_pwm_readl(pwm_ch_addr);
  per0 &= 0xffff0000;
  per0 |= act_cys;
  sunxi_pwm_writel(per0,pwm_ch_addr);
  delay (10) ;

  per0 = sunxi_pwm_readl(pwm_ch_addr);
  per0 &= 0xffff;

  if (wiringPiDebug)
    printf("%s after set, act: %d, phyaddr:0x%x\n",__func__, per0, pwm_ch_addr);
  
}

void sunxi_pwm_clear_reg()
{
  sunxi_pwm_writel(0, SUNXI_PWM_CH0_CTRL); 
  sunxi_pwm_writel(0, SUNXI_PWM_CH0_PERIOD); 
}

void sunxi_pwm_set_all()
{	
  sunxi_pwm_clear_reg();
  
  //set default M:S to 1/2
  sunxi_pwm_set_period(1024);
  sunxi_pwm_set_act(512);
  sunxi_pwm_set_mode(PWM_MODE_MS);
  sunxi_pwm_set_clk(PWM_CLK_DIV_120);//default clk:24M/120
  sunxi_pwm_set_enable(1);
  delayMicroseconds (200);
}

int sunxi_get_pin_mode(int pin)
{
  uint32_t regval = 0;
  int bank = pin >> 5;
  int index = pin - (bank << 5);
  int offset = ((index - ((index >> 3) << 3)) << 2);
  uint32_t reval=0;
  uint32_t phyaddr=0;

  phyaddr = sunxi_gpio_phyaddr(bank, ((index >> 3) << 2));

  if (wiringPiDebug)
    printf("func:%s pin:%d,  bank:%d index:%d phyaddr:0x%x\n",__func__, pin , bank,index,phyaddr);

//  if(BP_PIN_MASK[bank][index] != -1)
  if(1)
  {
    regval = sunxi_gpio_readl(phyaddr, bank);
	
    if (wiringPiDebug)
      printf("read reg val: 0x%x offset:%d  return: %d\n",regval,offset,reval);

    //reval=regval &(reval+(7 << offset));
    reval=(regval>>offset)&7;

    if (wiringPiDebug)
      printf("read reg val: 0x%x offset:%d  return: %d\n",regval,offset,reval);

    return reval;
  }
  else 
  {
    printf("line:__%d___ %d pin (%d:%d) number error(\n",__LINE__,pin,bank,index);
    return reval;
  } 
}

void sunxi_set_pin_mode(int pin,int mode)
{
  uint32_t regval = 0;
  int bank = pin >> 5;
  int index = pin - (bank << 5);
  int offset = ((index - ((index >> 3) << 3)) << 2);
  uint32_t phyaddr=0;
  int reg_offset;

  phyaddr = sunxi_gpio_phyaddr(bank, ((index >> 3) << 2));

  if (wiringPiDebug)
    printf("func:%s pin:%d, MODE:%d bank:%d index:%d phyaddr:0x%x\n",__func__, pin , mode,bank,index,phyaddr);

//  if(BP_PIN_MASK[bank][index] != -1)
  if(1)
  {
    regval = sunxi_gpio_readl(phyaddr, bank);
	
    if (wiringPiDebug)
      printf("read reg val: 0x%x offset:%d\n",regval,offset);

    if(INPUT == mode)
    {
      regval &= ~(7 << offset);
      sunxi_gpio_writel(regval, phyaddr, bank);
      regval = sunxi_gpio_readl(phyaddr, bank);

      if (wiringPiDebug)
        printf("Input mode set over reg val: 0x%x\n",regval);
    }
    else if(OUTPUT == mode)
    {
      regval &= ~(7 << offset);
      regval |=  (1 << offset);
	  
      if (wiringPiDebug)
        printf("Out mode ready set val: 0x%x\n",regval);

      sunxi_gpio_writel(regval, phyaddr, bank);
      regval = sunxi_gpio_readl(phyaddr, bank);
	  
      if (wiringPiDebug)
        printf("Out mode set over reg val: 0x%x\n",regval);
    } 
    else if(PWM_OUTPUT == mode)
    {
      reg_offset = bpi_wiringPiSetupRegOffset(mode);
        if(reg_offset < 0){
	  printf("reg offset not defined\n");
	  return;
      }

      //set pin PWMx to pwm mode
      regval &= ~(7 << offset);
      regval |=  (reg_offset << offset);
	  
      if (wiringPiDebug)
        printf(">>>>>line:%d PWM mode ready to set val: 0x%x\n",__LINE__,regval);

      sunxi_gpio_writel(regval, phyaddr, bank);
      delayMicroseconds (200);
      regval = sunxi_gpio_readl(phyaddr, bank);
	  
      if (wiringPiDebug)
        printf("<<<<<PWM mode set over reg val: 0x%x\n",regval); 

	  //register configure
	  sunxi_pwm_set_all();
    }
	else if(I2C_PIN == mode)
    {
      reg_offset = bpi_wiringPiSetupRegOffset(mode);
        if(reg_offset < 0){
          printf("reg offset not defined\n");
          return;
      }

      //set pin to i2c mode
      regval &= ~(7 << offset);
      regval |=  (reg_offset << offset);

      sunxi_gpio_writel(regval, phyaddr, bank);
      delayMicroseconds (200);
      regval = sunxi_gpio_readl(phyaddr, bank);
	  
      if (wiringPiDebug)
        printf("<<<<<I2C mode set over reg val: 0x%x\n",regval); 
    }
	else if(SPI_PIN == mode)
    {
      reg_offset = bpi_wiringPiSetupRegOffset(mode);
        if(reg_offset < 0){
          printf("reg offset not defined\n");
          return;
      }

      //set pin to spi mode
      regval &= ~(7 << offset);
      regval |=  (reg_offset << offset);

      sunxi_gpio_writel(regval, phyaddr, bank);
      delayMicroseconds (200);
      regval = sunxi_gpio_readl(phyaddr, bank);
	  
      if (wiringPiDebug)
        printf("<<<<<SPI mode set over reg val: 0x%x\n",regval); 
    }
  }
  else 
  {
    printf("line:__%d___ %d pin (%d:%d) number error(\n",__LINE__,pin,bank,index);
  }

	return ;
}

void sunxi_digitalWrite(int pin, int value)
{ 
  uint32_t regval = 0;
  int bank = pin >> 5;
  int index = pin - (bank << 5);
  uint32_t phyaddr=0;

  phyaddr = sunxi_gpio_phyaddr(bank, 0x10);

  if (wiringPiDebug)
    printf("func:%s pin:%d, value:%d bank:%d index:%d phyaddr:0x%x\n",__func__, pin , value,bank,index,phyaddr);

//  if(BP_PIN_MASK[bank][index] != -1)
  if(1)
  {
    regval = sunxi_gpio_readl(phyaddr, bank);
	
    if (wiringPiDebug)
      printf("befor write reg val: 0x%x,index:%d\n",regval,index);

    if(0 == value)
    {
      regval &= ~(1 << index);
      sunxi_gpio_writel(regval, phyaddr, bank);
      regval = sunxi_gpio_readl(phyaddr, bank);
	  
      if (wiringPiDebug)
        printf("LOW val set over reg val: 0x%x\n",regval);
    }
    else
    {
      regval |= (1 << index);
      sunxi_gpio_writel(regval, phyaddr, bank);
      regval = sunxi_gpio_readl(phyaddr, bank);
	  
      if (wiringPiDebug)
        printf("HIGH val set over reg val: 0x%x\n",regval);
    }
  }
  else
  {
    printf("line:__%d___ %d pin (%d:%d) number error\n",__LINE__,pin,bank,index);
  }
	 
	 return ;
}

int sunxi_digitalRead(int pin)
{ 
  uint32_t regval = 0;
  int bank = pin >> 5;
  int index = pin - (bank << 5);
  uint32_t phyaddr=0;

  phyaddr = sunxi_gpio_phyaddr(bank, 0x10);

  if (wiringPiDebug)
    printf("func:%s pin:%d,bank:%d index:%d phyaddr:0x%x\n",__func__, pin,bank,index,phyaddr); 
  
//  if(BP_PIN_MASK[bank][index] != -1)
  if(1)
  {
    regval = sunxi_gpio_readl(phyaddr, bank);
    regval = regval >> index;
    regval &= 1;
	
    if (wiringPiDebug)
      printf("***** read reg val: 0x%x,bank:%d,index:%d,line:%d\n",regval,bank,index,__LINE__);
	
    return regval;
  }
  else
  {
    printf("line:__%d___ %d pin (%d:%d) number error(\n",__LINE__,pin,bank,index);
    return regval;
  } 
}

void sunxi_pullUpDnControl (int pin, int pud)
{
  uint32_t regval = 0;
  int bank = pin >> 5;
  int index = pin - (bank << 5);
  int sub = index >> 4;
  int sub_index = index - 16*sub;
  uint32_t phyaddr=0;

  phyaddr = sunxi_gpio_phyaddr(bank, 0x1c + sub*4);

  if (wiringPiDebug)
	printf("func:%s pin:%d,bank:%d index:%d sub:%d phyaddr:0x%x\n",__func__, pin,bank,index,sub,phyaddr); 
  
  if(1)
  {  //PI13~PI21 need check again
    regval = sunxi_gpio_readl(phyaddr, bank);
	
	if (wiringPiDebug)
	  printf("pullUpDn reg:0x%x, pud:0x%x sub_index:%d\n", regval, pud, sub_index);
	
	regval &= ~(3 << (sub_index << 1));
	regval |= (pud << (sub_index << 1));
	
	if (wiringPiDebug)
	  printf("pullUpDn val ready to set:0x%x\n", regval);
	
	sunxi_gpio_writel(regval, phyaddr, bank);
	regval = sunxi_gpio_readl(phyaddr, bank);
	
	if (wiringPiDebug)
	  printf("pullUpDn reg after set:0x%x  addr:0x%x\n", regval, phyaddr);
  }
  else 
  {
    printf("line:__%d___ %d pin (%d:%d) number error(\n",__LINE__,pin,bank,index);
  } 
  
  delay (1) ;	
  
  return ;
}

static volatile uint32_t *mtk_gpio_reg(unsigned int offset)
{
  return (volatile uint32_t *)(mtk_gpio_base + offset);
}

static int mtk_gpio_mapped(void)
{
  return mtk_gpio_base != NULL;
}

static unsigned int mtk_gpio_field_offset(unsigned int base, unsigned int pin)
{
  return base + (pin / MTK_GPIO_FIELD_PINS_PER_REG) * 0x10;
}

static unsigned int mtk_gpio_field_shift(unsigned int pin)
{
  return pin % MTK_GPIO_FIELD_PINS_PER_REG;
}

static unsigned int mtk_gpio_dir_offset(unsigned int pin, unsigned int *shift)
{
  if (pin <= 175)
  {
    *shift = pin % MTK_GPIO_FIELD_PINS_PER_REG;
    return MTK_GPIO_DIR + (pin / MTK_GPIO_FIELD_PINS_PER_REG) * 0x10;
  }

  *shift = (pin - 176) % MTK_GPIO_FIELD_PINS_PER_REG;
  return 0xc0 + ((pin - 176) / MTK_GPIO_FIELD_PINS_PER_REG) * 0x10;
}

static void mtk_gpio_update_bit(unsigned int offset, unsigned int shift, int value)
{
  uint32_t regval;
  volatile uint32_t *reg;

  if (!mtk_gpio_mapped())
    return;

  reg = mtk_gpio_reg(offset);
  regval = *reg;
  if (value)
    regval |= (1u << shift);
  else
    regval &= ~(1u << shift);
  *reg = regval;
}

static void mtk_set_pin_mode(int pin, int mode)
{
  uint32_t regval;
  unsigned int shift;
  volatile uint32_t *reg;

  if (!mtk_gpio_mapped())
    return;

  reg = mtk_gpio_reg(MTK_GPIO_MODE + (pin / MTK_GPIO_MODE_PINS_PER_REG) * 0x10);
  shift = (pin % MTK_GPIO_MODE_PINS_PER_REG) * 3;
  regval = *reg;
  regval &= ~(0x7u << shift);
  regval |= ((mode & 0x7u) << shift);
  *reg = regval;
}

static int mtk_get_pin_mode(int pin)
{
  uint32_t mode;
  unsigned int offset;
  unsigned int shift;

  if (!mtk_gpio_mapped())
    return 0;

  offset = MTK_GPIO_MODE + (pin / MTK_GPIO_MODE_PINS_PER_REG) * 0x10;
  shift = (pin % MTK_GPIO_MODE_PINS_PER_REG) * 3;
  mode = (*mtk_gpio_reg(offset) >> shift) & 0x7u;
  if (mode != 0)
    return mode;

  offset = mtk_gpio_dir_offset(pin, &shift);
  return ((*mtk_gpio_reg(offset) >> shift) & 0x1u) ? OUTPUT : INPUT;
}

static void mtk_set_pin_direction(int pin, int mode)
{
  unsigned int offset;
  unsigned int shift;

  if (!mtk_gpio_mapped())
    return;

  offset = mtk_gpio_dir_offset(pin, &shift);
  mtk_gpio_update_bit(offset, shift, mode == OUTPUT);
}

static int mtk_digitalRead(int pin)
{
  if (!mtk_gpio_mapped())
    return LOW;

  return ((*mtk_gpio_reg(mtk_gpio_field_offset(MTK_GPIO_DIN, pin)) >>
           mtk_gpio_field_shift(pin)) & 0x1u) ? HIGH : LOW;
}

static void mtk_digitalWrite(int pin, int value)
{
  if (!mtk_gpio_mapped())
    return;

  mtk_gpio_update_bit(mtk_gpio_field_offset(MTK_GPIO_DOUT, pin),
                      mtk_gpio_field_shift(pin), value == HIGH);
}

static void mtk_pullUpDnControl(int pin, int pud)
{
  unsigned int shift;

  if (!mtk_gpio_mapped())
    return;

  shift = mtk_gpio_field_shift(pin);
  if (pud == PUD_OFF)
  {
    mtk_gpio_update_bit(mtk_gpio_field_offset(MTK_GPIO_PULLE, pin), shift, 0);
    return;
  }

  mtk_gpio_update_bit(mtk_gpio_field_offset(MTK_GPIO_PULLSEL, pin), shift, pud == PUD_UP);
  mtk_gpio_update_bit(mtk_gpio_field_offset(MTK_GPIO_PULLE, pin), shift, 1);
}

static volatile uint32_t *mtk_v2_gpio_reg(unsigned int offset)
{
  return (volatile uint32_t *)(mtk_v2_gpio_base + offset);
}

static int mtk_v2_gpio_mapped(void)
{
  return mtk_v2_gpio_base != NULL;
}

static unsigned int mtk_v2_gpio_field_offset(unsigned int base, unsigned int pin)
{
  return base + (pin / MTK_V2_FIELD_PINS_PER_REG) * 0x10;
}

static unsigned int mtk_v2_gpio_field_shift(unsigned int pin)
{
  return pin % MTK_V2_FIELD_PINS_PER_REG;
}

static void mtk_v2_gpio_update_bit(unsigned int offset, unsigned int shift, int value)
{
  uint32_t regval;
  volatile uint32_t *reg;

  if (!mtk_v2_gpio_mapped())
    return;

  reg = mtk_v2_gpio_reg(offset);
  regval = *reg;
  if (value)
    regval |= (1u << shift);
  else
    regval &= ~(1u << shift);
  *reg = regval;
}

static void mtk_v2_set_pin_mode(int pin, int mode)
{
  uint32_t regval;
  unsigned int shift;
  volatile uint32_t *reg;

  if (!mtk_v2_gpio_mapped())
    return;

  reg = mtk_v2_gpio_reg(MTK_V2_GPIO_MODE + (pin / MTK_V2_MODE_PINS_PER_REG) * 0x10);
  shift = (pin % MTK_V2_MODE_PINS_PER_REG) * MTK_V2_MODE_BITS;
  regval = *reg;
  regval &= ~(0xfu << shift);
  regval |= ((mode & 0xfu) << shift);
  *reg = regval;
}

static int mtk_v2_get_pin_mode(int pin)
{
  uint32_t mode;
  unsigned int offset;
  unsigned int shift;

  if (!mtk_v2_gpio_mapped())
    return 0;

  offset = MTK_V2_GPIO_MODE + (pin / MTK_V2_MODE_PINS_PER_REG) * 0x10;
  shift = (pin % MTK_V2_MODE_PINS_PER_REG) * MTK_V2_MODE_BITS;
  mode = (*mtk_v2_gpio_reg(offset) >> shift) & 0xfu;
  if (mode != 0)
    return mode;

  offset = mtk_v2_gpio_field_offset(MTK_V2_GPIO_DIR, pin);
  shift = mtk_v2_gpio_field_shift(pin);
  return ((*mtk_v2_gpio_reg(offset) >> shift) & 0x1u) ? OUTPUT : INPUT;
}

static void mtk_v2_set_pin_direction(int pin, int mode)
{
  if (!mtk_v2_gpio_mapped())
    return;

  mtk_v2_gpio_update_bit(mtk_v2_gpio_field_offset(MTK_V2_GPIO_DIR, pin),
                         mtk_v2_gpio_field_shift(pin), mode == OUTPUT);
}

static int mtk_v2_digitalRead(int pin)
{
  if (!mtk_v2_gpio_mapped())
    return LOW;

  return ((*mtk_v2_gpio_reg(mtk_v2_gpio_field_offset(MTK_V2_GPIO_DIN, pin)) >>
           mtk_v2_gpio_field_shift(pin)) & 0x1u) ? HIGH : LOW;
}

static void mtk_v2_digitalWrite(int pin, int value)
{
  if (!mtk_v2_gpio_mapped())
    return;

  mtk_v2_gpio_update_bit(mtk_v2_gpio_field_offset(MTK_V2_GPIO_DOUT, pin),
                         mtk_v2_gpio_field_shift(pin), value == HIGH);
}

static void mtk_v2_pullUpDnControl(int pin, int pud)
{
  (void)pin;
  (void)pud;
}

static volatile uint32_t *mtk_mt7622_gpio_reg(unsigned int offset)
{
  return (volatile uint32_t *)(mtk_mt7622_gpio_base + offset);
}

static int mtk_mt7622_gpio_mapped(void)
{
  return mtk_mt7622_gpio_base != NULL;
}

static unsigned int mtk_mt7622_gpio_field_offset(unsigned int base, unsigned int pin)
{
  return base + (pin / MTK_MT7622_FIELD_PINS_PER_REG) * 0x10;
}

static unsigned int mtk_mt7622_gpio_field_shift(unsigned int pin)
{
  return pin % MTK_MT7622_FIELD_PINS_PER_REG;
}

static int mtk_mt7622_lookup_field(const struct mtk_mt7622_pin_field_calc *ranges,
                                   size_t count, int pin,
                                   unsigned int *offset,
                                   unsigned int *shift,
                                   unsigned int *mask)
{
  size_t i;
  unsigned int bits;

  for (i = 0; i < count; ++i)
  {
    if (pin < ranges[i].s_pin || pin > ranges[i].e_pin)
      continue;

    bits = ranges[i].fixed ? ranges[i].s_bit :
           ranges[i].s_bit + (unsigned int)(pin - ranges[i].s_pin) * ranges[i].x_bits;
    *offset = ranges[i].s_addr + ranges[i].x_addrs * (bits / 32);
    *shift = bits % 32;
    *mask = (1u << ranges[i].x_bits) - 1u;
    return 1;
  }

  return 0;
}

static void mtk_mt7622_gpio_update_bit(unsigned int offset, unsigned int shift, int value)
{
  uint32_t regval;
  volatile uint32_t *reg;

  if (!mtk_mt7622_gpio_mapped())
    return;

  reg = mtk_mt7622_gpio_reg(offset);
  regval = *reg;
  if (value)
    regval |= (1u << shift);
  else
    regval &= ~(1u << shift);
  *reg = regval;
}

static void mtk_mt7622_set_field(const struct mtk_mt7622_pin_field_calc *ranges,
                                 size_t count, int pin, int value)
{
  uint32_t regval;
  unsigned int offset;
  unsigned int shift;
  unsigned int mask;
  volatile uint32_t *reg;

  if (!mtk_mt7622_gpio_mapped())
    return;

  if (!mtk_mt7622_lookup_field(ranges, count, pin, &offset, &shift, &mask))
    return;

  reg = mtk_mt7622_gpio_reg(offset);
  regval = *reg;
  regval &= ~(mask << shift);
  regval |= ((unsigned int)value & mask) << shift;
  *reg = regval;
}

static int mtk_mt7622_get_field(const struct mtk_mt7622_pin_field_calc *ranges,
                                size_t count, int pin)
{
  unsigned int offset;
  unsigned int shift;
  unsigned int mask;

  if (!mtk_mt7622_gpio_mapped())
    return 0;

  if (!mtk_mt7622_lookup_field(ranges, count, pin, &offset, &shift, &mask))
    return 0;

  return (*mtk_mt7622_gpio_reg(offset) >> shift) & mask;
}

static void mtk_mt7622_set_pin_mode(int pin, int mode)
{
  mtk_mt7622_set_field(mtk_mt7622_mode_ranges,
                       sizeof(mtk_mt7622_mode_ranges) / sizeof(mtk_mt7622_mode_ranges[0]),
                       pin, mode);
}

static int mtk_mt7622_get_pin_mode(int pin)
{
  int mode;
  unsigned int offset;
  unsigned int shift;

  if (!mtk_mt7622_gpio_mapped())
    return 0;

  mode = mtk_mt7622_get_field(mtk_mt7622_mode_ranges,
                              sizeof(mtk_mt7622_mode_ranges) / sizeof(mtk_mt7622_mode_ranges[0]),
                              pin);
  if (mode != 0)
    return mode;

  offset = mtk_mt7622_gpio_field_offset(MTK_MT7622_GPIO_DIR, pin);
  shift = mtk_mt7622_gpio_field_shift(pin);
  return ((*mtk_mt7622_gpio_reg(offset) >> shift) & 0x1u) ? OUTPUT : INPUT;
}

static void mtk_mt7622_set_pin_direction(int pin, int mode)
{
  if (!mtk_mt7622_gpio_mapped())
    return;

  mtk_mt7622_gpio_update_bit(mtk_mt7622_gpio_field_offset(MTK_MT7622_GPIO_DIR, pin),
                             mtk_mt7622_gpio_field_shift(pin), mode == OUTPUT);
}

static int mtk_mt7622_digitalRead(int pin)
{
  if (!mtk_mt7622_gpio_mapped())
    return LOW;

  return ((*mtk_mt7622_gpio_reg(mtk_mt7622_gpio_field_offset(MTK_MT7622_GPIO_DIN, pin)) >>
           mtk_mt7622_gpio_field_shift(pin)) & 0x1u) ? HIGH : LOW;
}

static void mtk_mt7622_digitalWrite(int pin, int value)
{
  if (!mtk_mt7622_gpio_mapped())
    return;

  mtk_mt7622_gpio_update_bit(mtk_mt7622_gpio_field_offset(MTK_MT7622_GPIO_DOUT, pin),
                             mtk_mt7622_gpio_field_shift(pin), value == HIGH);
}

static void mtk_mt7622_pullUpDnControl(int pin, int pud)
{
  size_t pu_count = sizeof(mtk_mt7622_pu_ranges) / sizeof(mtk_mt7622_pu_ranges[0]);
  size_t pd_count = sizeof(mtk_mt7622_pd_ranges) / sizeof(mtk_mt7622_pd_ranges[0]);

  if (!mtk_mt7622_gpio_mapped())
    return;

  if (pud == PUD_UP)
  {
    mtk_mt7622_set_field(mtk_mt7622_pd_ranges, pd_count, pin, 0);
    mtk_mt7622_set_field(mtk_mt7622_pu_ranges, pu_count, pin, 1);
  }
  else if (pud == PUD_DOWN)
  {
    mtk_mt7622_set_field(mtk_mt7622_pu_ranges, pu_count, pin, 0);
    mtk_mt7622_set_field(mtk_mt7622_pd_ranges, pd_count, pin, 1);
  }
  else
  {
    mtk_mt7622_set_field(mtk_mt7622_pu_ranges, pu_count, pin, 0);
    mtk_mt7622_set_field(mtk_mt7622_pd_ranges, pd_count, pin, 0);
  }
}

static int meson_gpio_mapped(void)
{
  return meson_gpio != NULL && meson_gpioao != NULL;
}

static int meson_is_ao_pin(int pin)
{
  return pin >= MESON_GPIOAO_PIN_START && pin <= MESON_GPIOAO_PIN_END;
}

static volatile uint32_t *meson_gpio_regs(int pin)
{
  return meson_is_ao_pin(pin) ? meson_gpioao : meson_gpio;
}

static int meson_gpio_shift(int pin)
{
  if (pin >= MESON_GPIOH_PIN_START && pin <= MESON_GPIOH_PIN_END)
    return pin - MESON_GPIOH_PIN_START;
  if (pin >= MESON_GPIOA_PIN_START && pin <= MESON_GPIOA_PIN_END)
    return pin - MESON_GPIOA_PIN_START;
  if (pin >= MESON_GPIOX_PIN_START && pin <= MESON_GPIOX_PIN_END)
    return pin - MESON_GPIOX_PIN_START;
  if (pin >= MESON_GPIOAO_PIN_START && pin <= MESON_GPIOAO_PIN_END)
    return pin - MESON_GPIOAO_PIN_START;

  return -1;
}

static int meson_gpio_fsel_offset(int pin)
{
  if (pin >= MESON_GPIOH_PIN_START && pin <= MESON_GPIOH_PIN_END)
    return MESON_GPIOH_FSEL_REG_OFFSET;
  if (pin >= MESON_GPIOA_PIN_START && pin <= MESON_GPIOA_PIN_END)
    return MESON_GPIOA_FSEL_REG_OFFSET;
  if (pin >= MESON_GPIOX_PIN_START && pin <= MESON_GPIOX_PIN_END)
    return MESON_GPIOX_FSEL_REG_OFFSET;
  if (pin >= MESON_GPIOAO_PIN_START && pin <= MESON_GPIOAO_PIN_END)
    return MESON_GPIOAO_FSEL_REG_OFFSET;

  return -1;
}

static int meson_gpio_out_offset(int pin)
{
  if (pin >= MESON_GPIOH_PIN_START && pin <= MESON_GPIOH_PIN_END)
    return MESON_GPIOH_OUTP_REG_OFFSET;
  if (pin >= MESON_GPIOA_PIN_START && pin <= MESON_GPIOA_PIN_END)
    return MESON_GPIOA_OUTP_REG_OFFSET;
  if (pin >= MESON_GPIOX_PIN_START && pin <= MESON_GPIOX_PIN_END)
    return MESON_GPIOX_OUTP_REG_OFFSET;
  if (pin >= MESON_GPIOAO_PIN_START && pin <= MESON_GPIOAO_PIN_END)
    return MESON_GPIOAO_OUTP_REG_OFFSET;

  return -1;
}

static int meson_gpio_in_offset(int pin)
{
  if (pin >= MESON_GPIOH_PIN_START && pin <= MESON_GPIOH_PIN_END)
    return MESON_GPIOH_INP_REG_OFFSET;
  if (pin >= MESON_GPIOA_PIN_START && pin <= MESON_GPIOA_PIN_END)
    return MESON_GPIOA_INP_REG_OFFSET;
  if (pin >= MESON_GPIOX_PIN_START && pin <= MESON_GPIOX_PIN_END)
    return MESON_GPIOX_INP_REG_OFFSET;
  if (pin >= MESON_GPIOAO_PIN_START && pin <= MESON_GPIOAO_PIN_END)
    return MESON_GPIOAO_INP_REG_OFFSET;

  return -1;
}

static int meson_gpio_puen_offset(int pin)
{
  if (pin >= MESON_GPIOH_PIN_START && pin <= MESON_GPIOH_PIN_END)
    return MESON_GPIOH_PUEN_REG_OFFSET;
  if (pin >= MESON_GPIOA_PIN_START && pin <= MESON_GPIOA_PIN_END)
    return MESON_GPIOA_PUEN_REG_OFFSET;
  if (pin >= MESON_GPIOX_PIN_START && pin <= MESON_GPIOX_PIN_END)
    return MESON_GPIOX_PUEN_REG_OFFSET;
  if (pin >= MESON_GPIOAO_PIN_START && pin <= MESON_GPIOAO_PIN_END)
    return MESON_GPIOAO_PUEN_REG_OFFSET;

  return -1;
}

static int meson_gpio_pupd_offset(int pin)
{
  if (pin >= MESON_GPIOH_PIN_START && pin <= MESON_GPIOH_PIN_END)
    return MESON_GPIOH_PUPD_REG_OFFSET;
  if (pin >= MESON_GPIOA_PIN_START && pin <= MESON_GPIOA_PIN_END)
    return MESON_GPIOA_PUPD_REG_OFFSET;
  if (pin >= MESON_GPIOX_PIN_START && pin <= MESON_GPIOX_PIN_END)
    return MESON_GPIOX_PUPD_REG_OFFSET;
  if (pin >= MESON_GPIOAO_PIN_START && pin <= MESON_GPIOAO_PIN_END)
    return MESON_GPIOAO_PUPD_REG_OFFSET;

  return -1;
}

static int meson_gpio_mux_offset(int pin)
{
  if (pin >= MESON_GPIOH_PIN_START && pin <= MESON_GPIOH_PIN_END)
    return MESON_GPIOH_MUX_B_REG_OFFSET;
  if (pin >= MESON_GPIOA_PIN_START && pin <= MESON_GPIOA_PIN_START + 7)
    return MESON_GPIOA_MUX_D_REG_OFFSET;
  if (pin >= MESON_GPIOA_PIN_START + 8 && pin <= MESON_GPIOA_PIN_END)
    return MESON_GPIOA_MUX_E_REG_OFFSET;
  if (pin >= MESON_GPIOX_PIN_START && pin <= MESON_GPIOX_PIN_START + 7)
    return MESON_GPIOX_MUX_3_REG_OFFSET;
  if (pin >= MESON_GPIOX_PIN_START + 8 && pin <= MESON_GPIOX_PIN_MID)
    return MESON_GPIOX_MUX_4_REG_OFFSET;
  if (pin > MESON_GPIOX_PIN_MID && pin <= MESON_GPIOX_PIN_END)
    return MESON_GPIOX_MUX_5_REG_OFFSET;
  if (pin >= MESON_GPIOAO_PIN_START && pin <= MESON_GPIOAO_PIN_START + 7)
    return MESON_GPIOAO_MUX_REG0_OFFSET;
  if (pin >= MESON_GPIOAO_PIN_START + 8 && pin <= MESON_GPIOAO_PIN_END)
    return MESON_GPIOAO_MUX_REG1_OFFSET;

  return -1;
}

static void meson_update_reg(int pin, int offset, uint32_t clear, uint32_t set)
{
  uint32_t regval;
  volatile uint32_t *reg;

  if (!meson_gpio_mapped() || offset < 0)
    return;

  reg = meson_gpio_regs(pin) + offset;
  regval = *reg;
  regval &= ~clear;
  regval |= set;
  *reg = regval;
}

static void meson_set_pin_alt(int pin, int mode)
{
  int shift = meson_gpio_shift(pin);
  int mux = meson_gpio_mux_offset(pin);
  unsigned int mux_shift;
  uint32_t alt;

  if (!meson_gpio_mapped() || shift < 0 || mux < 0)
    return;

  alt = mode > 1 ? (uint32_t)(mode - 1) : (uint32_t)mode;
  mux_shift = (unsigned int)(shift & 0x7) * 4;
  meson_update_reg(pin, mux, 0xFu << mux_shift, (alt & 0xFu) << mux_shift);
}

static void meson_set_pin_mode(int pin, int mode)
{
  int shift = meson_gpio_shift(pin);
  int mux = meson_gpio_mux_offset(pin);
  int fsel = meson_gpio_fsel_offset(pin);
  unsigned int mux_shift;

  if (!meson_gpio_mapped() || shift < 0 || mux < 0 || fsel < 0)
    return;

  mux_shift = (unsigned int)(shift & 0x7) * 4;
  meson_update_reg(pin, mux, 0xFu << mux_shift, 0);
  if (mode == INPUT)
    meson_update_reg(pin, fsel, 0, 1u << shift);
  else if (mode == OUTPUT)
    meson_update_reg(pin, fsel, 1u << shift, 0);
}

static int meson_get_pin_mode(int pin)
{
  int shift = meson_gpio_shift(pin);
  int mux = meson_gpio_mux_offset(pin);
  int fsel = meson_gpio_fsel_offset(pin);
  unsigned int mux_shift;
  uint32_t mode;

  if (!meson_gpio_mapped() || shift < 0 || mux < 0 || fsel < 0)
    return 0;

  mux_shift = (unsigned int)(shift & 0x7) * 4;
  mode = (*(meson_gpio_regs(pin) + mux) >> mux_shift) & 0xFu;
  if (mode != 0)
    return (int)mode + 1;

  return (*(meson_gpio_regs(pin) + fsel) & (1u << shift)) ? INPUT : OUTPUT;
}

static void meson_pullUpDnControl(int pin, int pud)
{
  int shift = meson_gpio_shift(pin);
  int puen = meson_gpio_puen_offset(pin);
  int pupd = meson_gpio_pupd_offset(pin);

  if (!meson_gpio_mapped() || shift < 0 || puen < 0 || pupd < 0)
    return;

  if (pud == PUD_OFF)
  {
    meson_update_reg(pin, puen, 1u << shift, 0);
    return;
  }

  meson_update_reg(pin, pupd, 1u << shift, pud == PUD_UP ? 1u << shift : 0);
  meson_update_reg(pin, puen, 0, 1u << shift);
}

static int meson_digitalRead(int pin)
{
  int shift = meson_gpio_shift(pin);
  int offset = meson_gpio_in_offset(pin);

  if (!meson_gpio_mapped() || shift < 0 || offset < 0)
    return LOW;

  return (*(meson_gpio_regs(pin) + offset) & (1u << shift)) ? HIGH : LOW;
}

static void meson_digitalWrite(int pin, int value)
{
  int shift = meson_gpio_shift(pin);
  int offset = meson_gpio_out_offset(pin);

  if (!meson_gpio_mapped() || shift < 0 || offset < 0)
    return;

  meson_update_reg(pin, offset, value == LOW ? 1u << shift : 0, value == LOW ? 0 : 1u << shift);
}

static int spacemit_gpio_mapped(void)
{
  return spacemit_gpio != NULL && spacemit_pinctrl != NULL;
}

static int spacemit_is_pin(int pin)
{
  return pin >= SPACEMIT_GPIO_PIN_BASE && pin <= SPACEMIT_GPIO_PIN_END;
}

static int spacemit_mfpr_offset(int pin)
{
  if (pin < SPACEMIT_GPIO_PIN_BASE || pin > SPACEMIT_GPIO_PIN_END)
    return -1;
  if (pin <= 85)
    return (pin + 1) << 2;
  if (pin <= 92)
    return ((pin + 1) << 2) + 0x90;

  return ((pin + 1) << 2) + 0x4C;
}

static int spacemit_gpio_alt(int pin)
{
  if ((pin >= 70 && pin <= 73) || (pin >= 93 && pin <= 103))
    return 1;
  if (pin >= 104 && pin <= 109)
    return 4;

  return 0;
}

static int spacemit_bank_offset(int pin)
{
  int bank = pin >> 5;

  return bank == 3 ? SPACEMIT_BANK3_OFFSET : SPACEMIT_BANK012_OFFSET(bank);
}

static int spacemit_pin_shift(int pin)
{
  return pin & 0x1F;
}

static void spacemit_update_reg(volatile uint32_t *base, int offset, uint32_t clear, uint32_t set)
{
  volatile uint32_t *reg;
  uint32_t regval;

  if (!spacemit_gpio_mapped() || offset < 0)
    return;

  reg = base + (offset >> 2);
  regval = *reg;
  regval &= ~clear;
  regval |= set;
  *reg = regval;
}

static void spacemit_set_pin_alt(int pin, int mode)
{
  int mfpr = spacemit_mfpr_offset(pin);

  if (!spacemit_is_pin(pin) || mfpr < 0)
    return;

  spacemit_update_reg(spacemit_pinctrl, mfpr, SPACEMIT_AF_SEL_MASK,
                      (uint32_t)(mode & 0x7) << SPACEMIT_AF_SEL_OFFSET);
}

static void spacemit_set_pin_mode(int pin, int mode)
{
  int mfpr = spacemit_mfpr_offset(pin);
  int bank = spacemit_bank_offset(pin);
  int shift = spacemit_pin_shift(pin);
  int dir_offset;

  if (!spacemit_is_pin(pin) || mfpr < 0)
    return;

  spacemit_update_reg(spacemit_pinctrl, mfpr, SPACEMIT_AF_SEL_MASK,
                      (uint32_t)spacemit_gpio_alt(pin) << SPACEMIT_AF_SEL_OFFSET);

  if (mode == INPUT)
    dir_offset = bank + SPACEMIT_GCDR;
  else if (mode == OUTPUT)
    dir_offset = bank + SPACEMIT_GSDR;
  else
    return;

  spacemit_update_reg(spacemit_gpio, dir_offset, 0, 1u << shift);
}

static int spacemit_get_pin_mode(int pin)
{
  int mfpr = spacemit_mfpr_offset(pin);
  int bank = spacemit_bank_offset(pin);
  int shift = spacemit_pin_shift(pin);
  uint32_t af_sel;

  if (!spacemit_is_pin(pin) || mfpr < 0 || !spacemit_gpio_mapped())
    return 0;

  af_sel = (*(spacemit_pinctrl + (mfpr >> 2))) & SPACEMIT_AF_SEL_MASK;
  if (af_sel != (uint32_t)spacemit_gpio_alt(pin))
    return (int)af_sel + 2;

  return (*(spacemit_gpio + ((bank + SPACEMIT_GPDR) >> 2)) & (1u << shift)) ? OUTPUT : INPUT;
}

static void spacemit_pullUpDnControl(int pin, int pud)
{
  int mfpr = spacemit_mfpr_offset(pin);
  uint32_t pull = SPACEMIT_PULL_DIS;

  if (!spacemit_is_pin(pin) || mfpr < 0)
    return;

  if (pud == PUD_UP)
    pull = SPACEMIT_PULL_UP;
  else if (pud == PUD_DOWN)
    pull = SPACEMIT_PULL_DOWN;

  spacemit_update_reg(spacemit_pinctrl, mfpr, SPACEMIT_PULL_MASK,
                      (pull & 0x7) << SPACEMIT_PULL_OFFSET);
}

static int spacemit_digitalRead(int pin)
{
  int bank = spacemit_bank_offset(pin);
  int shift = spacemit_pin_shift(pin);

  if (!spacemit_is_pin(pin) || !spacemit_gpio_mapped())
    return LOW;

  return (*(spacemit_gpio + ((bank + SPACEMIT_GPLR) >> 2)) & (1u << shift)) ? HIGH : LOW;
}

static void spacemit_digitalWrite(int pin, int value)
{
  int bank = spacemit_bank_offset(pin);
  int shift = spacemit_pin_shift(pin);
  int offset;

  if (!spacemit_is_pin(pin) || !spacemit_gpio_mapped())
    return;

  offset = bank + (value == LOW ? SPACEMIT_GPCR : SPACEMIT_GPSR);
  spacemit_update_reg(spacemit_gpio, offset, 0, 1u << shift);
}

static int renesas_gpio_mapped(void)
{
  return renesas_gpio != NULL;
}

static int renesas_is_pin(int pin)
{
  return pin >= RENESAS_GPIO_PIN_BASE && pin <= RENESAS_GPIO_PIN_END;
}

static void renesas_update_reg(int offset, uint32_t clear, uint32_t set)
{
  volatile uint32_t *reg;
  uint32_t regval;

  if (!renesas_gpio_mapped() || offset < 0)
    return;

  reg = renesas_gpio + (offset >> 2);
  regval = *reg;
  regval &= ~clear;
  regval |= set;
  *reg = regval;
}

static void renesas_set_pin_alt(int pin, int mode)
{
  int offset, port, bit, pmc_phyaddr, pmc_shift;
  uint32_t pmc_mask;

  if (!renesas_is_pin(pin))
    return;

  offset = RENESAS_PIN_OFFSET(pin);
  port = RENESAS_PIN_ID_TO_PORT_OFFSET(offset);
  bit = RENESAS_PIN_ID_TO_PIN(offset);
  pmc_phyaddr = RENESAS_PMC(port);
  pmc_shift = (pmc_phyaddr % 4) * 8;
  pmc_mask = (1u << bit) << pmc_shift;

  renesas_update_reg(pmc_phyaddr, pmc_mask, pmc_mask);
  renesas_update_reg(RENESAS_PFC(port), 0xfu << (bit * 4),
                     (uint32_t)(mode & 0xf) << (bit * 4));
}

static void renesas_set_pin_mode(int pin, int mode)
{
  int offset, port, bit, pmc_phyaddr, pmc_shift, pm_phyaddr, pm_shift;
  uint32_t pmc_mask, pm_mask, pm_value;

  if (!renesas_is_pin(pin))
    return;

  offset = RENESAS_PIN_OFFSET(pin);
  port = RENESAS_PIN_ID_TO_PORT_OFFSET(offset);
  bit = RENESAS_PIN_ID_TO_PIN(offset);
  pmc_phyaddr = RENESAS_PMC(port);
  pmc_shift = (pmc_phyaddr % 4) * 8;
  pm_phyaddr = RENESAS_PM(port);
  pm_shift = (pm_phyaddr % 4) * 8;

  pmc_mask = (1u << bit) << pmc_shift;
  pm_mask = (0x3u << (bit * 2)) << pm_shift;

  if (mode == INPUT)
    pm_value = ((uint32_t)RENESAS_PM_INPUT << (bit * 2)) << pm_shift;
  else if (mode == OUTPUT)
    pm_value = ((uint32_t)RENESAS_PM_OUTPUT << (bit * 2)) << pm_shift;
  else
    return;

  renesas_update_reg(pmc_phyaddr, pmc_mask, 0);
  renesas_update_reg(pm_phyaddr, pm_mask, pm_value);
}

static int renesas_get_pin_mode(int pin)
{
  int offset, port, bit, pmc_phyaddr, pmc_shift, pm_phyaddr, pm_shift;
  uint32_t mode, gpiomode;

  if (!renesas_is_pin(pin) || !renesas_gpio_mapped())
    return 0;

  offset = RENESAS_PIN_OFFSET(pin);
  port = RENESAS_PIN_ID_TO_PORT_OFFSET(offset);
  bit = RENESAS_PIN_ID_TO_PIN(offset);
  pmc_phyaddr = RENESAS_PMC(port);
  pmc_shift = (pmc_phyaddr % 4) * 8;
  pm_phyaddr = RENESAS_PM(port);
  pm_shift = (pm_phyaddr % 4) * 8;

  mode = (*(renesas_gpio + (pmc_phyaddr >> 2)) >> pmc_shift) & (1u << bit);
  if (!mode)
  {
    gpiomode = *(renesas_gpio + (pm_phyaddr >> 2)) >> pm_shift;
    gpiomode = (gpiomode >> (bit * 2)) & 0x3;
    if (gpiomode == RENESAS_PM_OUTPUT)
      return OUTPUT;
    if (gpiomode == RENESAS_PM_INPUT)
      return INPUT;
    return RENESAS_PM_HIZ;
  }

  mode = *(renesas_gpio + (RENESAS_PFC(port) >> 2));
  mode = (mode >> (bit * 4)) & 0xf;
  return (int)mode + 2;
}

static void renesas_pullUpDnControl(int pin, int pud)
{
  int offset, port, port_offset, bit, pupd_phyaddr;
  uint32_t pull = RENESAS_PULL_DIS;

  if (!renesas_is_pin(pin))
    return;

  offset = RENESAS_PIN_OFFSET(pin);
  port = RENESAS_PIN_ID_TO_PORT_OFFSET(offset);
  port_offset = port + RENESAS_EXTENDED_REG_OFFSET;
  bit = RENESAS_PIN_ID_TO_PIN(offset);
  pupd_phyaddr = RENESAS_PUPD(port_offset);

  if (bit >= 4)
  {
    bit -= 4;
    pupd_phyaddr += 4;
  }

  if (pud == PUD_UP)
    pull = RENESAS_PULL_UP;
  else if (pud == PUD_DOWN)
    pull = RENESAS_PULL_DOWN;

  renesas_update_reg(pupd_phyaddr, 0x3u << (bit * 8), pull << (bit * 8));
}

static int renesas_digitalRead(int pin)
{
  int offset, port, bit, p_phyaddr, p_shift, pm_phyaddr, pm_shift;
  int pin_phyaddr, pin_shift;
  uint32_t gpiomode;

  if (!renesas_is_pin(pin) || !renesas_gpio_mapped())
    return LOW;

  offset = RENESAS_PIN_OFFSET(pin);
  port = RENESAS_PIN_ID_TO_PORT_OFFSET(offset);
  bit = RENESAS_PIN_ID_TO_PIN(offset);

  p_phyaddr = RENESAS_P(port);
  p_shift = (p_phyaddr % 4) * 8;
  pm_phyaddr = RENESAS_PM(port);
  pm_shift = (pm_phyaddr % 4) * 8;
  pin_phyaddr = RENESAS_PIN(port);
  pin_shift = (pin_phyaddr % 4) * 8;

  gpiomode = *(renesas_gpio + (pm_phyaddr >> 2)) >> pm_shift;
  gpiomode = (gpiomode >> (bit * 2)) & 0x3;

  if (gpiomode == RENESAS_PM_INPUT)
    return ((*(renesas_gpio + (pin_phyaddr >> 2)) >> pin_shift) & (1u << bit)) ? HIGH : LOW;
  if (gpiomode == RENESAS_PM_OUTPUT)
    return ((*(renesas_gpio + (p_phyaddr >> 2)) >> p_shift) & (1u << bit)) ? HIGH : LOW;

  return LOW;
}

static void renesas_digitalWrite(int pin, int value)
{
  int offset, port, bit, p_phyaddr, p_shift;
  uint32_t bit_mask;

  if (!renesas_is_pin(pin))
    return;

  offset = RENESAS_PIN_OFFSET(pin);
  port = RENESAS_PIN_ID_TO_PORT_OFFSET(offset);
  bit = RENESAS_PIN_ID_TO_PIN(offset);
  p_phyaddr = RENESAS_P(port);
  p_shift = (p_phyaddr % 4) * 8;
  bit_mask = (1u << bit) << p_shift;

  renesas_update_reg(p_phyaddr, bit_mask, value == LOW ? 0 : bit_mask);
}

static int rockchip_is_pin(int pin)
{
  return pin >= ROCKCHIP_GPIO_PIN_BASE && pin <= ROCKCHIP_GPIO_PIN_END;
}

static volatile uint32_t *rockchip_bank_regs(int bank)
{
  if (bank < 0 || bank >= ROCKCHIP_GPIO_BANKS)
    return NULL;

  return rockchip_gpio[bank];
}

static int rockchip_gpio_mapped(void)
{
  int i;

  for (i = 0; i < ROCKCHIP_GPIO_BANKS; ++i)
    if (rockchip_gpio[i] == NULL)
      return 0;

  return 1;
}

static uint32_t rockchip_read_reg(int bank, int offset)
{
  volatile uint32_t *regs = rockchip_bank_regs(bank);

  if (regs == NULL)
    return 0;

  if (rockchip_gpio_v2)
    return regs[offset >> 2] | (regs[(offset + 4) >> 2] << 16);

  return regs[offset >> 2];
}

static void rockchip_write_bit(int bank, int offset, int bit, int value)
{
  volatile uint32_t *regs = rockchip_bank_regs(bank);
  int half_bit;
  uint32_t data;

  if (regs == NULL)
    return;

  if (!rockchip_gpio_v2)
  {
    data = regs[offset >> 2];
    if (value)
      data |= (1u << bit);
    else
      data &= ~(1u << bit);
    regs[offset >> 2] = data;
    return;
  }

  half_bit = bit & 0xf;
  data = (value ? (1u << half_bit) : 0) | (1u << (half_bit + 16));
  regs[(offset + (bit >= 16 ? 4 : 0)) >> 2] = data;
}

static void rockchip_set_pin_alt(int pin, int mode)
{
  (void)pin;
  (void)mode;
}

static void rockchip_set_pin_mode(int pin, int mode)
{
  int bank, bit;

  if (!rockchip_is_pin(pin) || !rockchip_gpio_mapped())
    return;

  if (mode != INPUT && mode != OUTPUT)
    return;

  bank = pin >> 5;
  bit = pin & 0x1f;
  rockchip_write_bit(bank, rockchip_gpio_swport_ddr, bit, mode == OUTPUT);
}

static int rockchip_get_pin_mode(int pin)
{
  int bank, bit;

  if (!rockchip_is_pin(pin) || !rockchip_gpio_mapped())
    return INPUT;

  bank = pin >> 5;
  bit = pin & 0x1f;

  return (rockchip_read_reg(bank, rockchip_gpio_swport_ddr) & (1u << bit)) ? OUTPUT : INPUT;
}

static void rockchip_pullUpDnControl(int pin, int pud)
{
  (void)pin;
  (void)pud;
}

static int rockchip_digitalRead(int pin)
{
  int bank, bit;

  if (!rockchip_is_pin(pin) || !rockchip_gpio_mapped())
    return LOW;

  bank = pin >> 5;
  bit = pin & 0x1f;

  return (rockchip_read_reg(bank, rockchip_gpio_ext_port) & (1u << bit)) ? HIGH : LOW;
}

static void rockchip_digitalWrite(int pin, int value)
{
  int bank, bit;

  if (!rockchip_is_pin(pin) || !rockchip_gpio_mapped())
    return;

  bank = pin >> 5;
  bit = pin & 0x1f;
  rockchip_write_bit(bank, rockchip_gpio_swport_dr, bit, value != LOW);
}

static const struct realtek_gpio_group *realtek_group_for_pin(int pin, int *local_pin)
{
  int i;

  for (i = 0; i < realtek_gpio_group_count; ++i)
  {
    const struct realtek_gpio_group *group = &realtek_gpio_groups[i];

    if (pin < group->pin_base || pin > group->pin_end)
      continue;

    if (local_pin != NULL)
      *local_pin = pin - group->pin_base;
    return group;
  }

  return NULL;
}

static volatile uint32_t *realtek_group_regs(const struct realtek_gpio_group *group)
{
  if (group == NULL || group->map_index < 0 || group->map_index >= REALTEK_GPIO_GROUPS)
    return NULL;

  return realtek_gpio[group->map_index];
}

static int realtek_gpio_mapped(void)
{
  int i;

  for (i = 0; i < realtek_gpio_group_count; ++i)
    if (realtek_gpio[i] == NULL)
      return 0;

  return 1;
}

static uint32_t realtek_read_reg(const struct realtek_gpio_group *group, int offset)
{
  volatile uint32_t *regs = realtek_group_regs(group);

  if (regs == NULL)
    return 0;

  return regs[offset >> 2];
}

static void realtek_write_bit(const struct realtek_gpio_group *group, int offset, int bit, int value)
{
  volatile uint32_t *regs = realtek_group_regs(group);
  uint32_t data;

  if (regs == NULL)
    return;

  data = regs[offset >> 2];
  if (value)
    data |= (1u << bit);
  else
    data &= ~(1u << bit);
  regs[offset >> 2] = data;
}

static void realtek_set_pin_alt(int pin, int mode)
{
  (void)pin;
  (void)mode;
}

static void realtek_set_pin_mode(int pin, int mode)
{
  const struct realtek_gpio_group *group;
  int local_pin, index, bit;

  if (!realtek_gpio_mapped())
    return;

  group = realtek_group_for_pin(pin, &local_pin);
  if (group == NULL)
    return;

  if (mode != INPUT && mode != OUTPUT)
    return;

  index = local_pin >> 5;
  bit = local_pin & 0x1f;
  realtek_write_bit(group, group->dir_offset[index], bit, mode == OUTPUT);
}

static int realtek_get_pin_mode(int pin)
{
  const struct realtek_gpio_group *group;
  int local_pin, index, bit;

  if (!realtek_gpio_mapped())
    return INPUT;

  group = realtek_group_for_pin(pin, &local_pin);
  if (group == NULL)
    return INPUT;

  index = local_pin >> 5;
  bit = local_pin & 0x1f;
  return (realtek_read_reg(group, group->dir_offset[index]) & (1u << bit)) ? OUTPUT : INPUT;
}

static void realtek_pullUpDnControl(int pin, int pud)
{
  (void)pin;
  (void)pud;
}

static int realtek_digitalRead(int pin)
{
  const struct realtek_gpio_group *group;
  int local_pin, index, bit, offset;

  if (!realtek_gpio_mapped())
    return LOW;

  group = realtek_group_for_pin(pin, &local_pin);
  if (group == NULL)
    return LOW;

  index = local_pin >> 5;
  bit = local_pin & 0x1f;
  offset = realtek_get_pin_mode(pin) == OUTPUT ? group->dato_offset[index] : group->dati_offset[index];
  return (realtek_read_reg(group, offset) & (1u << bit)) ? HIGH : LOW;
}

static void realtek_digitalWrite(int pin, int value)
{
  const struct realtek_gpio_group *group;
  int local_pin, index, bit;

  if (!realtek_gpio_mapped())
    return;

  group = realtek_group_for_pin(pin, &local_pin);
  if (group == NULL)
    return;

  index = local_pin >> 5;
  bit = local_pin & 0x1f;
  realtek_write_bit(group, group->dato_offset[index], bit, value != LOW);
}

static int vs680_is_pin(int pin)
{
  return (pin >= VS680_GPIO_SOC_PIN_BASE && pin <= VS680_GPIO_SOC_PIN_END) ||
      (pin >= VS680_GPIO_SM_PIN_BASE && pin <= VS680_GPIO_SM_PIN_END);
}

static int vs680_pin_bank(int pin)
{
  if (pin >= VS680_GPIO_SOC_PIN_BASE && pin <= VS680_GPIO_SOC_PIN_END)
    return pin >> 5;

  if (pin >= VS680_GPIO_SM_PIN_BASE && pin <= VS680_GPIO_SM_PIN_END)
    return 3;

  return -1;
}

static int vs680_pin_bit(int pin)
{
  if (pin >= VS680_GPIO_SM_PIN_BASE)
    return pin - VS680_GPIO_SM_PIN_BASE;

  return pin & 0x1f;
}

static int vs680_gpio_mapped(void)
{
  int i;

  for (i = 0; i < VS680_GPIO_BANKS; ++i)
    if (vs680_gpio[i] == NULL)
      return 0;

  return 1;
}

static uint32_t vs680_read_reg(int bank, int offset)
{
  volatile uint32_t *regs;

  if (bank < 0 || bank >= VS680_GPIO_BANKS)
    return 0;

  regs = vs680_gpio[bank];
  if (regs == NULL)
    return 0;

  return regs[offset >> 2];
}

static void vs680_write_bit(int bank, int offset, int bit, int value)
{
  volatile uint32_t *regs;
  uint32_t data;

  if (bank < 0 || bank >= VS680_GPIO_BANKS)
    return;

  regs = vs680_gpio[bank];
  if (regs == NULL)
    return;

  data = regs[offset >> 2];
  if (value)
    data |= (1u << bit);
  else
    data &= ~(1u << bit);
  regs[offset >> 2] = data;
}

static void vs680_set_pin_alt(int pin, int mode)
{
  (void)pin;
  (void)mode;
}

static void vs680_set_pin_mode(int pin, int mode)
{
  int bank, bit;

  if (!vs680_is_pin(pin) || !vs680_gpio_mapped())
    return;

  if (mode != INPUT && mode != OUTPUT)
    return;

  bank = vs680_pin_bank(pin);
  bit = vs680_pin_bit(pin);
  vs680_write_bit(bank, VS680_GPIO_SWPORT_DDR, bit, mode == OUTPUT);
}

static int vs680_get_pin_mode(int pin)
{
  int bank, bit;

  if (!vs680_is_pin(pin) || !vs680_gpio_mapped())
    return INPUT;

  bank = vs680_pin_bank(pin);
  bit = vs680_pin_bit(pin);
  return (vs680_read_reg(bank, VS680_GPIO_SWPORT_DDR) & (1u << bit)) ? OUTPUT : INPUT;
}

static void vs680_pullUpDnControl(int pin, int pud)
{
  (void)pin;
  (void)pud;
}

static int vs680_digitalRead(int pin)
{
  int bank, bit;

  if (!vs680_is_pin(pin) || !vs680_gpio_mapped())
    return LOW;

  bank = vs680_pin_bank(pin);
  bit = vs680_pin_bit(pin);
  return (vs680_read_reg(bank, VS680_GPIO_EXT_PORT) & (1u << bit)) ? HIGH : LOW;
}

static void vs680_digitalWrite(int pin, int value)
{
  int bank, bit;

  if (!vs680_is_pin(pin) || !vs680_gpio_mapped())
    return;

  bank = vs680_pin_bank(pin);
  bit = vs680_pin_bit(pin);
  vs680_write_bit(bank, VS680_GPIO_SWPORT_DR, bit, value != LOW);
}

static int sp7021_gpio_mapped(void)
{
  return sp7021_gpio_base0 != NULL &&
      sp7021_gpio_base1 != NULL &&
      sp7021_gpio_base2 != NULL;
}

static int sp7021_is_pin(int pin)
{
  return pin >= SP7021_GPIO_PIN_BASE && pin <= SP7021_GPIO_PIN_END;
}

static volatile uint32_t *sp7021_reg(volatile uint32_t *base, unsigned int offset)
{
  return base + (offset >> 2);
}

static void sp7021_update_masked(volatile uint32_t *base, unsigned int offset,
                                 unsigned int bit, int value)
{
  uint32_t data;

  if (!sp7021_gpio_mapped())
    return;

  data = (1u << (bit + 16));
  if (value)
    data |= 1u << bit;
  *sp7021_reg(base, offset) = data;
}

static void sp7021_update_direct(volatile uint32_t *base, unsigned int offset,
                                 unsigned int bit, int value)
{
  volatile uint32_t *reg;
  uint32_t data;

  if (!sp7021_gpio_mapped())
    return;

  reg = sp7021_reg(base, offset);
  data = *reg;
  if (value)
    data |= 1u << bit;
  else
    data &= ~(1u << bit);
  *reg = data;
}

static void sp7021_claim_gpio(int pin)
{
  unsigned int bit16, bit32;

  if (!sp7021_is_pin(pin) || !sp7021_gpio_mapped())
    return;

  bit16 = SP7021_R16_BOF(pin);
  bit32 = SP7021_R32_BOF(pin);
  sp7021_update_direct(sp7021_gpio_base2, SP7021_GPIO_GFR + SP7021_R32_ROF(pin),
                       bit32, 1);
  sp7021_update_masked(sp7021_gpio_base0, SP7021_GPIO_CTL + SP7021_R16_ROF(pin),
                       bit16, 1);
}

static void sp7021_set_pin_alt(int pin, int mode)
{
  (void)mode;

  if (!sp7021_is_pin(pin))
    return;

  sp7021_claim_gpio(pin);
}

static void sp7021_set_pin_mode(int pin, int mode)
{
  unsigned int bit;

  if (!sp7021_is_pin(pin) || !sp7021_gpio_mapped())
    return;

  if (mode != INPUT && mode != OUTPUT)
    return;

  sp7021_claim_gpio(pin);
  bit = SP7021_R16_BOF(pin);
  sp7021_update_masked(sp7021_gpio_base0, SP7021_GPIO_OE + SP7021_R16_ROF(pin),
                       bit, mode == OUTPUT);
}

static int sp7021_get_pin_mode(int pin)
{
  unsigned int bit, offset;
  uint32_t data;

  if (!sp7021_is_pin(pin) || !sp7021_gpio_mapped())
    return INPUT;

  bit = SP7021_R16_BOF(pin);
  offset = SP7021_GPIO_OE + SP7021_R16_ROF(pin);
  data = *sp7021_reg(sp7021_gpio_base0, offset);
  return (data & (1u << bit)) ? OUTPUT : INPUT;
}

static void sp7021_pullUpDnControl(int pin, int pud)
{
  (void)pin;
  (void)pud;
}

static int sp7021_digitalRead(int pin)
{
  unsigned int bit, offset;

  if (!sp7021_is_pin(pin) || !sp7021_gpio_mapped())
    return LOW;

  bit = SP7021_R32_BOF(pin);
  offset = SP7021_GPIO_IN + SP7021_R32_ROF(pin);
  return (*sp7021_reg(sp7021_gpio_base0, offset) & (1u << bit)) ? HIGH : LOW;
}

static void sp7021_digitalWrite(int pin, int value)
{
  unsigned int bit, offset;

  if (!sp7021_is_pin(pin) || !sp7021_gpio_mapped())
    return;

  bit = SP7021_R16_BOF(pin);
  offset = SP7021_GPIO_OUT + SP7021_R16_ROF(pin);
  sp7021_update_masked(sp7021_gpio_base0, offset, bit, value != LOW);
}

static int sp7350_gpio_mapped(void)
{
  return sp7350_gpio_first != NULL && sp7350_gpio_gpioxt != NULL;
}

static int sp7350_is_pin(int pin)
{
  return pin >= SP7350_GPIO_PIN_BASE && pin <= SP7350_GPIO_PIN_END;
}

static volatile uint32_t *sp7350_reg(volatile uint32_t *base, unsigned int offset)
{
  return base + (offset >> 2);
}

static void sp7350_update_masked(volatile uint32_t *base, unsigned int offset,
                                 unsigned int bit, int value)
{
  uint32_t data;

  if (!sp7350_gpio_mapped())
    return;

  data = 1u << (bit + 16);
  if (value)
    data |= 1u << bit;
  *sp7350_reg(base, offset) = data;
}

static void sp7350_update_direct(volatile uint32_t *base, unsigned int offset,
                                 unsigned int bit, int value)
{
  volatile uint32_t *reg;
  uint32_t data;

  if (!sp7350_gpio_mapped())
    return;

  reg = sp7350_reg(base, offset);
  data = *reg;
  if (value)
    data |= 1u << bit;
  else
    data &= ~(1u << bit);
  *reg = data;
}

static void sp7350_claim_gpio(int pin)
{
  unsigned int bit16, bit32;

  if (!sp7350_is_pin(pin) || !sp7350_gpio_mapped())
    return;

  bit16 = SP7350_R16_BOF(pin);
  bit32 = SP7350_R32_BOF(pin);
  sp7350_update_direct(sp7350_gpio_first,
                       SP7350_GPIO_FIRST + SP7350_R32_ROF(pin), bit32, 1);
  sp7350_update_masked(sp7350_gpio_gpioxt,
                       SP7350_GPIO_CTL + SP7350_R16_ROF(pin), bit16, 1);
}

static void sp7350_set_pin_alt(int pin, int mode)
{
  (void)mode;

  if (!sp7350_is_pin(pin))
    return;

  sp7350_claim_gpio(pin);
}

static void sp7350_set_pin_mode(int pin, int mode)
{
  unsigned int bit;

  if (!sp7350_is_pin(pin) || !sp7350_gpio_mapped())
    return;
  if (mode != INPUT && mode != OUTPUT)
    return;

  sp7350_claim_gpio(pin);
  bit = SP7350_R16_BOF(pin);
  sp7350_update_masked(sp7350_gpio_gpioxt,
                       SP7350_GPIO_OE + SP7350_R16_ROF(pin),
                       bit, mode == OUTPUT);
}

static int sp7350_get_pin_mode(int pin)
{
  unsigned int bit, offset;
  uint32_t data;

  if (!sp7350_is_pin(pin) || !sp7350_gpio_mapped())
    return INPUT;

  bit = SP7350_R16_BOF(pin);
  offset = SP7350_GPIO_OE + SP7350_R16_ROF(pin);
  data = *sp7350_reg(sp7350_gpio_gpioxt, offset);
  return (data & (1u << bit)) ? OUTPUT : INPUT;
}

/* Pull configuration is intentionally deferred until F4 pad state is tested. */
static void sp7350_pullUpDnControl(int pin, int pud)
{
  (void)pin;
  (void)pud;
}

static int sp7350_digitalRead(int pin)
{
  unsigned int bit, offset;

  if (!sp7350_is_pin(pin) || !sp7350_gpio_mapped())
    return LOW;

  bit = SP7350_R32_BOF(pin);
  offset = SP7350_GPIO_IN + SP7350_R32_ROF(pin);
  return (*sp7350_reg(sp7350_gpio_gpioxt, offset) & (1u << bit)) ? HIGH : LOW;
}

static void sp7350_digitalWrite(int pin, int value)
{
  unsigned int bit, offset;

  if (!sp7350_is_pin(pin) || !sp7350_gpio_mapped())
    return;

  bit = SP7350_R16_BOF(pin);
  offset = SP7350_GPIO_OUT + SP7350_R16_ROF(pin);
  sp7350_update_masked(sp7350_gpio_gpioxt, offset, bit, value != LOW);
}

#ifdef BPI

int bpi_getAlt (int pin)
{
  int alt ;

  pin &= 63 ;

  if (wiringPiMode == WPI_MODE_PINS)
    pin = pinToGpio_BP [pin] ;
  else if (wiringPiMode == WPI_MODE_PHYS)
    pin = physToGpio_BP[pin] ;
  else if (wiringPiMode == WPI_MODE_GPIO)
    pin=pinTobcm_BP[pin];//need map A20 to bcm
  else return 0 ;
		
  if(-1 == pin) {
    if (wiringPiDebug)
      printf("[%s:L%d] the pin:%d is invaild,please check it over!\n", __func__,  __LINE__, pin);
    return -1;
  }
  if (bpi_found_mtk)
    return mtk_get_pin_mode(pin);
  if (bpi_found_mtk_v2)
    return mtk_v2_get_pin_mode(pin);
  if (bpi_found_mtk_mt7622)
    return mtk_mt7622_get_pin_mode(pin);
  if (bpi_found_meson)
    return meson_get_pin_mode(pin);
  if (bpi_found_spacemit)
    return spacemit_get_pin_mode(pin);
  if (bpi_found_renesas)
    return renesas_get_pin_mode(pin);
  if (bpi_found_rockchip)
    return rockchip_get_pin_mode(pin);
  if (bpi_found_realtek)
    return realtek_get_pin_mode(pin);
  if (bpi_found_vs680)
    return vs680_get_pin_mode(pin);
  if (bpi_found_sp7021)
    return sp7021_get_pin_mode(pin);
  if (bpi_found_sp7350)
    return sp7350_get_pin_mode(pin);

  alt=sunxi_get_pin_mode(pin);
  return alt ;
}


void bpi_pwmSetMode (int mode)
{
  if (bpi_found_mtk || bpi_found_mtk_v2 || bpi_found_mtk_mt7622 || bpi_found_sun50iw9 || bpi_found_meson || bpi_found_spacemit || bpi_found_renesas || bpi_found_rockchip || bpi_found_realtek || bpi_found_vs680 || bpi_found_sp7021 || bpi_found_sp7350)
    return;

  sunxi_pwm_set_mode(mode);
  sunxi_pwm_set_enable(1);
  return;
}

void bpi_pwmSetRange (unsigned int range)
{
  if (bpi_found_mtk || bpi_found_mtk_v2 || bpi_found_mtk_mt7622 || bpi_found_sun50iw9 || bpi_found_meson || bpi_found_spacemit || bpi_found_renesas || bpi_found_rockchip || bpi_found_realtek || bpi_found_vs680 || bpi_found_sp7021 || bpi_found_sp7350)
    return;

  sunxi_pwm_set_period(range);
  return;
}


void bpi_pwmSetClock (int divisor)
{
  if (bpi_found_mtk || bpi_found_mtk_v2 || bpi_found_mtk_mt7622 || bpi_found_sun50iw9 || bpi_found_meson || bpi_found_spacemit || bpi_found_renesas || bpi_found_rockchip || bpi_found_realtek || bpi_found_vs680 || bpi_found_sp7021 || bpi_found_sp7350)
    return;

  sunxi_pwm_set_clk(divisor);
  sunxi_pwm_set_enable(1);
  return;
}


void bpi_gpioClockSet (int pin, int freq)
{
  if(wiringPiDebug)
    printf("clock set pin:%d freq:%d\n", pin, freq);
  return;
}

/* bpi, set pin alt */
void sunxi_set_pin_alt(int pin, int mode)
{
  uint32_t regval = 0;
  int bank = pin >> 5;
  int index = pin - (bank << 5); 
  int offset = ((index - ((index >> 3) << 3)) << 2);
  uint32_t phyaddr=0;
                         
  /* for M2 PM and PL */
  if(bank >= 11)
    phyaddr = SUNXI_GPIO_LM_BASE + ((bank - 11) * 36) + ((index >> 3) << 2);
  else
    phyaddr = SUNXI_GPIO_BASE + (bank * 36) + ((index >> 3) << 2);
  if (wiringPiDebug)
    printf("func:%s pin:%d, MODE:%d bank:%d index:%d phyaddr:0x%x\n",__func__, pin , mode,bank,index,phyaddr);
  regval = sunxi_gpio_readl(phyaddr, bank);
  if (wiringPiDebug) 
    printf("read reg val: 0x%x offset:%d\n",regval,offset);
  regval &= ~(7 << offset);
  regval |=  ((mode & 0x7) << offset);
  if (wiringPiDebug) 
    printf("Out mode ready set val: 0x%x\n",regval);
  sunxi_gpio_writel(regval, phyaddr, bank);
  regval = sunxi_gpio_readl(phyaddr, bank);
  if (wiringPiDebug) 
    printf("Out mode set over reg val: 0x%x\n",regval);
  return;
}

void bpi_pinModeAlt (int pin, int mode)
{
  int origPin = pin ;

  if ((pin & PI_GPIO_MASK) == 0)    // On-board pin
  {
    if (wiringPiMode == WPI_MODE_PINS)
      pin = pinToGpio_BP [pin] ;
    else if (wiringPiMode == WPI_MODE_PHYS)
      pin = physToGpio_BP [pin] ;
    else if (wiringPiMode == WPI_MODE_GPIO)
      pin= pinTobcm_BP[pin];//need map A20 to bcm
    else
      return;

    if (-1 == pin)  /*VCC or GND return directly*/
    {
      //printf("[%s:L%d] the pin:%d is invaild,please check it over!\n", __func__,  __LINE__, pin);
      return;
    }
    if (wiringPiDebug)
      printf ("%s,%d,pin:%d,mode:%d\n", __func__, __LINE__,pin,mode) ;
    softPwmStop (origPin) ;
    softToneStop (origPin) ;
    if (bpi_found_mtk)
    {
      mtk_set_pin_mode(pin, mode);
      return;
    }
    if (bpi_found_mtk_v2)
    {
      mtk_v2_set_pin_mode(pin, mode);
      return;
    }
    if (bpi_found_mtk_mt7622)
    {
      mtk_mt7622_set_pin_mode(pin, mode);
      return;
    }
    if (bpi_found_meson)
    {
      meson_set_pin_alt(pin, mode);
      return;
    }
    if (bpi_found_spacemit)
    {
      spacemit_set_pin_alt(pin, mode);
      return;
    }
    if (bpi_found_renesas)
    {
      renesas_set_pin_alt(pin, mode);
      return;
    }
    if (bpi_found_rockchip)
    {
      rockchip_set_pin_alt(pin, mode);
      return;
    }
    if (bpi_found_realtek)
    {
      realtek_set_pin_alt(pin, mode);
      return;
    }
    if (bpi_found_vs680)
    {
      vs680_set_pin_alt(pin, mode);
      return;
    }
    if (bpi_found_sp7021)
    {
      sp7021_set_pin_alt(pin, mode);
      return;
    }
    if (bpi_found_sp7350)
    {
      sp7350_set_pin_alt(pin, mode);
      return;
    }
    sunxi_set_pin_alt(pin,mode);
  }
}


void bpi_pinMode (int pin, int mode)
{
  struct wiringPiNodeStruct *node = wiringPiNodes ;
  int origPin = pin ;
 
  if ((pin & PI_GPIO_MASK) == 0)    // On-board pin
  {
    if (wiringPiMode == WPI_MODE_PINS)
      pin = pinToGpio_BP [pin] ;
    else if (wiringPiMode == WPI_MODE_PHYS)
      pin = physToGpio_BP [pin] ;
    else if (wiringPiMode == WPI_MODE_GPIO)
      pin= pinTobcm_BP[pin];//need map A20 to bcm
    else 
      return;
    if (-1 == pin)  /*VCC or GND return directly*/
    {
      //printf("[%s:L%d] the pin:%d is invaild,please check it over!\n", __func__,  __LINE__, pin);
      return;
    }
    if (wiringPiDebug)
      printf ("%s,%d,pin:%d,mode:%d\n", __func__, __LINE__,pin,mode) ;
    softPwmStop (origPin) ;
    softToneStop (origPin) ;

    if (bpi_found_mtk)
    {
      if (mode == INPUT || mode == OUTPUT)
      {
        mtk_set_pin_mode(pin, 0);
        mtk_set_pin_direction(pin, mode);
      }
      else if (mode == PULLUP)
      {
        mtk_pullUpDnControl(pin, PUD_UP);
      }
      else if (mode == PULLDOWN)
      {
        mtk_pullUpDnControl(pin, PUD_DOWN);
      }
      else if (mode == PULLOFF)
      {
        mtk_pullUpDnControl(pin, PUD_OFF);
      }
      else
      {
        return;
      }
      wiringPinMode = mode;
      return;
    }

    if (bpi_found_mtk_v2)
    {
      if (mode == INPUT || mode == OUTPUT)
      {
        mtk_v2_set_pin_mode(pin, 0);
        mtk_v2_set_pin_direction(pin, mode);
      }
      else if (mode == PULLUP || mode == PULLDOWN || mode == PULLOFF)
      {
        mtk_v2_pullUpDnControl(pin, mode == PULLUP ? PUD_UP : (mode == PULLDOWN ? PUD_DOWN : PUD_OFF));
      }
      else
      {
        return;
      }
      wiringPinMode = mode;
      return;
    }

    if (bpi_found_mtk_mt7622)
    {
      if (mode == INPUT || mode == OUTPUT)
      {
        mtk_mt7622_set_pin_mode(pin, 0);
        mtk_mt7622_set_pin_direction(pin, mode);
      }
      else if (mode == PULLUP || mode == PULLDOWN || mode == PULLOFF)
      {
        mtk_mt7622_pullUpDnControl(pin, mode == PULLUP ? PUD_UP : (mode == PULLDOWN ? PUD_DOWN : PUD_OFF));
      }
      else
      {
        return;
      }
      wiringPinMode = mode;
      return;
    }

    if (bpi_found_meson)
    {
      if (mode == INPUT || mode == OUTPUT)
      {
        meson_set_pin_mode(pin, mode);
      }
      else if (mode == PULLUP)
      {
        meson_pullUpDnControl(pin, PUD_UP);
      }
      else if (mode == PULLDOWN)
      {
        meson_pullUpDnControl(pin, PUD_DOWN);
      }
      else if (mode == PULLOFF)
      {
        meson_pullUpDnControl(pin, PUD_OFF);
      }
      else
      {
        return;
      }
      wiringPinMode = mode;
      return;
    }

    if (bpi_found_spacemit)
    {
      if (mode == INPUT || mode == OUTPUT)
      {
        spacemit_set_pin_mode(pin, mode);
      }
      else if (mode == PULLUP)
      {
        spacemit_pullUpDnControl(pin, PUD_UP);
      }
      else if (mode == PULLDOWN)
      {
        spacemit_pullUpDnControl(pin, PUD_DOWN);
      }
      else if (mode == PULLOFF)
      {
        spacemit_pullUpDnControl(pin, PUD_OFF);
      }
      else
      {
        return;
      }
      wiringPinMode = mode;
      return;
    }

    if (bpi_found_renesas)
    {
      if (mode == INPUT || mode == OUTPUT)
      {
        renesas_set_pin_mode(pin, mode);
      }
      else if (mode == PULLUP)
      {
        renesas_pullUpDnControl(pin, PUD_UP);
      }
      else if (mode == PULLDOWN)
      {
        renesas_pullUpDnControl(pin, PUD_DOWN);
      }
      else if (mode == PULLOFF)
      {
        renesas_pullUpDnControl(pin, PUD_OFF);
      }
      else
      {
        return;
      }
      wiringPinMode = mode;
      return;
    }

    if (bpi_found_rockchip)
    {
      if (mode == INPUT || mode == OUTPUT)
      {
        rockchip_set_pin_mode(pin, mode);
      }
      else if (mode == PULLUP)
      {
        rockchip_pullUpDnControl(pin, PUD_UP);
      }
      else if (mode == PULLDOWN)
      {
        rockchip_pullUpDnControl(pin, PUD_DOWN);
      }
      else if (mode == PULLOFF)
      {
        rockchip_pullUpDnControl(pin, PUD_OFF);
      }
      else
      {
        return;
      }
      wiringPinMode = mode;
      return;
    }

    if (bpi_found_realtek)
    {
      if (mode == INPUT || mode == OUTPUT)
      {
        realtek_set_pin_mode(pin, mode);
      }
      else if (mode == PULLUP)
      {
        realtek_pullUpDnControl(pin, PUD_UP);
      }
      else if (mode == PULLDOWN)
      {
        realtek_pullUpDnControl(pin, PUD_DOWN);
      }
      else if (mode == PULLOFF)
      {
        realtek_pullUpDnControl(pin, PUD_OFF);
      }
      else
      {
        return;
      }
      wiringPinMode = mode;
      return;
    }

    if (bpi_found_vs680)
    {
      if (mode == INPUT || mode == OUTPUT)
      {
        vs680_set_pin_mode(pin, mode);
      }
      else if (mode == PULLUP)
      {
        vs680_pullUpDnControl(pin, PUD_UP);
      }
      else if (mode == PULLDOWN)
      {
        vs680_pullUpDnControl(pin, PUD_DOWN);
      }
      else if (mode == PULLOFF)
      {
        vs680_pullUpDnControl(pin, PUD_OFF);
      }
      else
      {
        return;
      }
      wiringPinMode = mode;
      return;
    }

    if (bpi_found_sp7021)
    {
      if (mode == INPUT || mode == OUTPUT)
      {
        sp7021_set_pin_mode(pin, mode);
      }
      else if (mode == PULLUP)
      {
        sp7021_pullUpDnControl(pin, PUD_UP);
      }
      else if (mode == PULLDOWN)
      {
        sp7021_pullUpDnControl(pin, PUD_DOWN);
      }
      else if (mode == PULLOFF)
      {
        sp7021_pullUpDnControl(pin, PUD_OFF);
      }
      else
      {
        return;
      }
      wiringPinMode = mode;
      return;
    }

    if (bpi_found_sp7350)
    {
      if (mode == INPUT || mode == OUTPUT)
      {
        sp7350_set_pin_mode(pin, mode);
      }
      else if (mode == PULLUP)
      {
        sp7350_pullUpDnControl(pin, PUD_UP);
      }
      else if (mode == PULLDOWN)
      {
        sp7350_pullUpDnControl(pin, PUD_DOWN);
      }
      else if (mode == PULLOFF)
      {
        sp7350_pullUpDnControl(pin, PUD_OFF);
      }
      else
      {
        return;
      }
      wiringPinMode = mode;
      return;
    }

    if (mode == INPUT)
    {
      sunxi_set_pin_mode(pin,INPUT);
      wiringPinMode = INPUT;
      return ;
    }
    else if (mode == OUTPUT)
    {
      sunxi_set_pin_mode(pin, OUTPUT);
      wiringPinMode = OUTPUT;
      return ;
    }
    else if (mode == PWM_OUTPUT)
    {
      if(pin != 6)
      {
        printf("the pin you choose does not support hardware PWM\n");
        printf("you can select PA6 for PWM pin\n");
        printf("or you can use it in softPwm mode\n");
        return ;
      }
      else
      {
        printf("you choose the hardware PWM:%d\n", 1);
      }
      sunxi_set_pin_mode(pin,PWM_OUTPUT);
      wiringPinMode = PWM_OUTPUT;
      return ;
    }
    else if (mode == I2C_PIN)
    {
      sunxi_set_pin_mode(pin, I2C_PIN);
      wiringPinMode = I2C_PIN;
    }
    else if (mode == SPI_PIN)
    {
      sunxi_set_pin_mode(pin, SPI_PIN);
      wiringPinMode = SPI_PIN;
    }
    else if (mode == PULLUP)
    {
      pullUpDnControl (origPin, 1);
      wiringPinMode = PULLUP;
      return ;
    }
    else if (mode == PULLDOWN)
    {
      pullUpDnControl (origPin, 2);
      wiringPinMode = PULLDOWN;
      return ;
    }
    else if (mode == PULLOFF)
    {
      pullUpDnControl (origPin, 0);
      wiringPinMode = PULLOFF;
      return ;
    }
    else
      return ;
  }
  else
  {
    if ((node = wiringPiFindNode (pin)) != NULL)
      node->pinMode (node, pin, mode) ;
    return ;
  }
}

void bpi_pullUpDnControl (int pin, int pud)
{
  struct wiringPiNodeStruct *node = wiringPiNodes ;
  if ((pin & PI_GPIO_MASK) == 0)		// On-Board Pin
  {
    /**/ if (wiringPiMode == WPI_MODE_PINS)
      pin = pinToGpio_BP [pin] ;
    else if (wiringPiMode == WPI_MODE_PHYS)
      pin = physToGpio_BP [pin] ;
    else if (wiringPiMode == WPI_MODE_GPIO)
     pin = pinTobcm_BP[pin];//need map A20 to bcm
    else 
     return ;

    if (wiringPiDebug)
      printf ("%s,%d,pin:%d\n", __func__, __LINE__,pin) ;
    if (-1 == pin)
    {
      printf("[%s:L%d] the pin:%d is invaild,please check it over!\n", __func__,  __LINE__, pin);
      return;
    }
    pud &= 3 ;
    if (bpi_found_mtk)
    {
      mtk_pullUpDnControl(pin, pud);
      return;
    }
    if (bpi_found_mtk_v2)
    {
      mtk_v2_pullUpDnControl(pin, pud);
      return;
    }
    if (bpi_found_mtk_mt7622)
    {
      mtk_mt7622_pullUpDnControl(pin, pud);
      return;
    }
    if (bpi_found_meson)
    {
      meson_pullUpDnControl(pin, pud);
      return;
    }
    if (bpi_found_spacemit)
    {
      spacemit_pullUpDnControl(pin, pud);
      return;
    }
    if (bpi_found_renesas)
    {
      renesas_pullUpDnControl(pin, pud);
      return;
    }
    if (bpi_found_rockchip)
    {
      rockchip_pullUpDnControl(pin, pud);
      return;
    }
    if (bpi_found_realtek)
    {
      realtek_pullUpDnControl(pin, pud);
      return;
    }
    if (bpi_found_vs680)
    {
      vs680_pullUpDnControl(pin, pud);
      return;
    }
    if (bpi_found_sp7021)
    {
      sp7021_pullUpDnControl(pin, pud);
      return;
    }
    if (bpi_found_sp7350)
    {
      sp7350_pullUpDnControl(pin, pud);
      return;
    }
    sunxi_pullUpDnControl(pin, pud);
    return;
  }
  else						// Extension module
  {
    if ((node = wiringPiFindNode (pin)) != NULL)
      node->pullUpDnControl (node, pin, pud) ;
    return ;
  }
}


int bpi_digitalRead (int pin)
{
  char c ;
  struct wiringPiNodeStruct *node = wiringPiNodes ;

  if ((pin & PI_GPIO_MASK) == 0)		// On-Board Pin
  {
    if (wiringPiMode == WPI_MODE_GPIO_SYS)	// Sys mode
    {
      if(pin==0)
      {
	if (wiringPiDebug)
          printf("%d %s,%d invalid pin,please check it over.\n",pin,__func__, __LINE__);
        return 0;
      }
      if(syspin[pin]==-1)
      {
        if (wiringPiDebug)
          printf("%d %s,%d invalid pin,please check it over.\n",pin,__func__, __LINE__);
        return 0;
      }
      if (sysFds [pin] == -1)
      {
        if (wiringPiDebug)
          printf ("pin %d sysFds -1.%s,%d\n", pin ,__func__, __LINE__) ;
        return LOW ;
      }
      if (wiringPiDebug)
        printf ("pin %d :%d.%s,%d\n", pin ,sysFds [pin],__func__, __LINE__) ;
      lseek  (sysFds [pin], 0L, SEEK_SET) ;
      read   (sysFds [pin], &c, 1) ;
      return (c == '0') ? LOW : HIGH ;
    }
    else if (wiringPiMode == WPI_MODE_PINS)
      pin = pinToGpio_BP [pin] ;
    else if (wiringPiMode == WPI_MODE_PHYS)
      pin = physToGpio_BP[pin] ;
    else if (wiringPiMode == WPI_MODE_GPIO)
      pin=pinTobcm_BP[pin];//need map A20 to bcm
    else 
      return LOW ;
    if(-1 == pin){
      if (wiringPiDebug)
        printf("[%s:L%d] the pin:%d is invaild,please check it over!\n", __func__,  __LINE__, pin);
      return LOW;
    }
    if (bpi_found_mtk)
      return mtk_digitalRead(pin);
    if (bpi_found_mtk_v2)
      return mtk_v2_digitalRead(pin);
    if (bpi_found_mtk_mt7622)
      return mtk_mt7622_digitalRead(pin);
    if (bpi_found_meson)
      return meson_digitalRead(pin);
    if (bpi_found_spacemit)
      return spacemit_digitalRead(pin);
    if (bpi_found_renesas)
      return renesas_digitalRead(pin);
    if (bpi_found_rockchip)
      return rockchip_digitalRead(pin);
    if (bpi_found_realtek)
      return realtek_digitalRead(pin);
    if (bpi_found_vs680)
      return vs680_digitalRead(pin);
    if (bpi_found_sp7021)
      return sp7021_digitalRead(pin);
    if (bpi_found_sp7350)
      return sp7350_digitalRead(pin);

    return sunxi_digitalRead(pin);
  }
  else
  {
    if ((node = wiringPiFindNode (pin)) == NULL)
      return LOW ;
    return node->digitalRead (node, pin) ;
  }
}

void bpi_digitalWrite (int pin, int value)
{
  struct wiringPiNodeStruct *node = wiringPiNodes ;
  if ((pin & PI_GPIO_MASK) == 0)    // On-Board Pin
  {
    /**/ if (wiringPiMode == WPI_MODE_GPIO_SYS) // Sys mode
    {
      if (wiringPiDebug)
      {
        if(pin==0)
        {
          printf("%d %s,%d invalid pin,please check it over.\n",pin,__func__, __LINE__);
          return;
        }
        if(syspin[pin]==-1)
        {
          printf("%d %s,%d invalid pin,please check it over.\n",pin,__func__, __LINE__);
          return;
        }
      }
      if (sysFds [pin] != -1)
      {
        if (wiringPiDebug)
        {
          printf ("pin %d sysFds -1.%s,%d\n", pin ,__func__, __LINE__) ;
          printf ("pin %d :%d.%s,%d\n", pin ,sysFds [pin],__func__, __LINE__) ;
        }
        if (value == LOW)
          write (sysFds [pin], "0\n", 2) ;
        else
          write (sysFds [pin], "1\n", 2) ;
      }
      return ;
    }
    else if (wiringPiMode == WPI_MODE_PINS)
      pin = pinToGpio_BP [pin] ;
    else if (wiringPiMode == WPI_MODE_PHYS)
      pin = physToGpio_BP [pin] ;
    else if (wiringPiMode == WPI_MODE_GPIO)
     pin=pinTobcm_BP[pin];//need map A20 to bcm
    else  return ;
    if(-1 == pin){
      printf("%d %s,%d %d invalid pin,please check it over.\n",pin,__func__, __LINE__,wiringPiMode);
      return ;
    }
    if (bpi_found_mtk)
    {
      mtk_digitalWrite(pin, value);
      return;
    }
    if (bpi_found_mtk_v2)
    {
      mtk_v2_digitalWrite(pin, value);
      return;
    }
    if (bpi_found_mtk_mt7622)
    {
      mtk_mt7622_digitalWrite(pin, value);
      return;
    }
    if (bpi_found_meson)
    {
      meson_digitalWrite(pin, value);
      return;
    }
    if (bpi_found_spacemit)
    {
      spacemit_digitalWrite(pin, value);
      return;
    }
    if (bpi_found_renesas)
    {
      renesas_digitalWrite(pin, value);
      return;
    }
    if (bpi_found_rockchip)
    {
      rockchip_digitalWrite(pin, value);
      return;
    }
    if (bpi_found_realtek)
    {
      realtek_digitalWrite(pin, value);
      return;
    }
    if (bpi_found_vs680)
    {
      vs680_digitalWrite(pin, value);
      return;
    }
    if (bpi_found_sp7021)
    {
      sp7021_digitalWrite(pin, value);
      return;
    }
    if (bpi_found_sp7350)
    {
      sp7350_digitalWrite(pin, value);
      return;
    }
    sunxi_digitalWrite(pin, value); 
  }
  else
  {
    if ((node = wiringPiFindNode (pin)) != NULL)
      node->digitalWrite (node, pin, value) ;
  }
}


void bpi_pwmWrite (int pin, int value)
{
  struct wiringPiNodeStruct *node = wiringPiNodes ;

  uint32_t a_val = 0;

  if (bpi_found_mtk || bpi_found_mtk_v2 || bpi_found_mtk_mt7622 || bpi_found_sun50iw9 || bpi_found_meson || bpi_found_spacemit || bpi_found_renesas || bpi_found_rockchip || bpi_found_realtek || bpi_found_vs680 || bpi_found_sp7021 || bpi_found_sp7350)
    return;

  if(pwmmode==1)//sycle
  {
    sunxi_pwm_set_mode(1);
  }
  else
  {
    //sunxi_pwm_set_mode(0);
  }
  if (pin < MAX_PIN_NUM)  // On-Board Pin needto fix me Jim
  {
    if (wiringPiMode == WPI_MODE_PINS)
      pin = pinToGpio_BP [pin] ;
    else if (wiringPiMode == WPI_MODE_PHYS){
      pin = physToGpio_BP[pin] ;
    } else if (wiringPiMode == WPI_MODE_GPIO)
      pin=pinTobcm_BP[pin];//need map A20 to bcm
    else
      return ;

    if(-1 == pin){
      printf("[%s:L%d] the pin:%d is invaild,please check it over!\n", __func__,  __LINE__, pin);
      return ;
    }
    if(pin != 6){
      printf("the pin(%d) you choose does not support hardware PWM\n", pin);
      printf("you can select PA6 for PWM pin\n");
      printf("or you can use it in softPwm mode\n");
      return ;
    }
    a_val = sunxi_pwm_get_period();
    if (wiringPiDebug)
      printf("==> no:%d period now is :%d,act_val to be set:%d\n",__LINE__,a_val, value);
    if((uint32_t)value > a_val){
      printf("val pwmWrite 0 <= X <= 1024\n");
      printf("Or you can set new range by yourself by pwmSetRange(range\n");
      return;
    }
    //if value changed chang it
    sunxi_pwm_set_enable(0);
    sunxi_pwm_set_act(value);
    sunxi_pwm_set_enable(1);
  }
  else 
  {
    printf("not on board :%s,%d\n", __func__, __LINE__) ;
    if ((node = wiringPiFindNode (pin)) != NULL)
    {
      if (wiringPiDebug)
        printf ("Jim find node%s,%d\n", __func__, __LINE__) ;
      node->digitalWrite (node, pin, value) ;
    }
  }
  if (wiringPiDebug)
    printf ("this fun is ok now %s,%d\n", __func__, __LINE__) ;
  return;
}

struct RegOffset
{
  int pwm_offset;
  int i2c_offset;
  int spi_offset;
};

struct BPIBoards
{
  const char *name;
  int gpioLayout;
  int model;
  int rev;
  int mem;
  int maker;
  int warranty;
  int *pinToGpio;
  int *physToGpio;
  int *pinTobcm;
  const char *i2c_dev;
  const char *spi_dev;
  struct RegOffset reg_offset;
} ;

/*
 * Board list
 *********************************************************************************
 */

struct BPIBoards bpiboard [] = 
{
  { "bpi-0",	      -1, 0, 1, 2, 5, 0, NULL, NULL, NULL, NULL, NULL, {-1, -1, -1} },
  { "bpi-1",	      -1, 1, 1, 2, 5, 0, NULL, NULL, NULL, NULL, NULL, {-1, -1, -1} },
  { "bpi-2",	      -1, 2, 1, 2, 5, 0, NULL, NULL, NULL, NULL, NULL, {-1, -1, -1} },
  { "bpi-3",	      -1, 3, 1, 2, 5, 0, NULL, NULL, NULL, NULL, NULL, {-1, -1, -1} },
  { "bpi-4",	      -1, 4, 1, 2, 5, 0, NULL, NULL, NULL, NULL, NULL, {-1, -1, -1} },
  { "bpi-5",	      -1, 5, 1, 2, 5, 0, NULL, NULL, NULL, NULL, NULL, {-1, -1, -1} },
  { "bpi-6",	      -1, 6, 1, 2, 5, 0, NULL, NULL, NULL, NULL, NULL, {-1, -1, -1} },
  { "bpi-7",	      -1, 7, 1, 2, 5, 0, NULL, NULL, NULL, NULL, NULL, {-1, -1, -1} },
  { "bpi-8",	      -1, 8, 1, 2, 5, 0, NULL, NULL, NULL, NULL, NULL, {-1, -1, -1} },
  { "bpi-9",	      -1, 9, 1, 2, 5, 0, NULL, NULL, NULL, NULL, NULL, {-1, -1, -1} },
  { "bpi-10",	      -1, 10, 1, 2, 5, 0, NULL, NULL, NULL, NULL, NULL, {-1, -1, -1} },
  { "bpi-11",	      -1, 11, 1, 2, 5, 0, NULL, NULL, NULL, NULL, NULL, {-1, -1, -1} },
  { "bpi-12",	      -1, 12, 1, 2, 5, 0, NULL, NULL, NULL, NULL, NULL, {-1, -1, -1} },
  { "bpi-13",	      -1, 13, 1, 2, 5, 0, NULL, NULL, NULL, NULL, NULL, {-1, -1, -1} },
  { "bpi-14",	      -1, 14, 1, 2, 5, 0, NULL, NULL, NULL, NULL, NULL, {-1, -1, -1} },
  { "bpi-15",	      -1, 15, 1, 2, 5, 0, NULL, NULL, NULL, NULL, NULL, {-1, -1, -1} },
  { "bpi-new",	      -1, 16, 1, 2, 5, 0, NULL, NULL, NULL, NULL, NULL, {-1, -1, -1} },
  { "bpi-x86",	      -1, 17, 1, 2, 5, 0, NULL, NULL, NULL, NULL, NULL, {-1, -1, -1} },
  { "bpi-rpi",	      -1, 18, 1, 2, 5, 0, NULL, NULL, NULL, NULL, NULL, {-1, -1, -1} },
  { "bpi-rpi2",	      -1, 19, 1, 2, 5, 0, NULL, NULL, NULL, NULL, NULL, {-1, -1, -1} },
  { "bpi-rpi3",	      -1, 20, 1, 2, 5, 0, NULL, NULL, NULL, NULL, NULL, {-1, -1, -1} },
  { "bpi-m1",	   10001, BPI_MODEL_M1, 1, 2, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M1P, physToGpio_BPI_M1P, pinTobcm_BPI_M1P, M1P_I2C_DEV, M1P_SPI_DEV, {M1P_PWM_OFFSET,M1P_I2C_OFFSET,M1P_SPI_OFFSET} },
  { "bpi-m1p",	   10001, BPI_MODEL_M1P, 1, 2, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M1P, physToGpio_BPI_M1P, pinTobcm_BPI_M1P, M1P_I2C_DEV, M1P_SPI_DEV, {M1P_PWM_OFFSET,M1P_I2C_OFFSET,M1P_SPI_OFFSET} },
  { "bpi-m1-plus", 10001, BPI_MODEL_M1P, 1, 2, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M1P, physToGpio_BPI_M1P, pinTobcm_BPI_M1P, M1P_I2C_DEV, M1P_SPI_DEV, {M1P_PWM_OFFSET,M1P_I2C_OFFSET,M1P_SPI_OFFSET} },
  { "bpi-pro",	   10001, BPI_MODEL_M1P, 1, 2, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M1P, physToGpio_BPI_M1P, pinTobcm_BPI_M1P, M1P_I2C_DEV, M1P_SPI_DEV, {M1P_PWM_OFFSET,M1P_I2C_OFFSET,M1P_SPI_OFFSET} },
  { "banana-pro", 10001, BPI_MODEL_M1P, 1, 2, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M1P, physToGpio_BPI_M1P, pinTobcm_BPI_M1P, M1P_I2C_DEV, M1P_SPI_DEV, {M1P_PWM_OFFSET,M1P_I2C_OFFSET,M1P_SPI_OFFSET} },
  { "bananapro",  10001, BPI_MODEL_M1P, 1, 2, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M1P, physToGpio_BPI_M1P, pinTobcm_BPI_M1P, M1P_I2C_DEV, M1P_SPI_DEV, {M1P_PWM_OFFSET,M1P_I2C_OFFSET,M1P_SPI_OFFSET} },
  { "bananapi-pro", 10001, BPI_MODEL_M1P, 1, 2, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M1P, physToGpio_BPI_M1P, pinTobcm_BPI_M1P, M1P_I2C_DEV, M1P_SPI_DEV, {M1P_PWM_OFFSET,M1P_I2C_OFFSET,M1P_SPI_OFFSET} },
  { "bananapipro", 10001, BPI_MODEL_M1P, 1, 2, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M1P, physToGpio_BPI_M1P, pinTobcm_BPI_M1P, M1P_I2C_DEV, M1P_SPI_DEV, {M1P_PWM_OFFSET,M1P_I2C_OFFSET,M1P_SPI_OFFSET} },
  { "bpi-r1",	   10001, BPI_MODEL_R1, 1, 2, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M1P, physToGpio_BPI_M1P, pinTobcm_BPI_M1P, M1P_I2C_DEV, M1P_SPI_DEV, {M1P_PWM_OFFSET,M1P_I2C_OFFSET,M1P_SPI_OFFSET} },
  { "bpi-m2",	   10101, BPI_MODEL_M2, 1, 2, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2, physToGpio_BPI_M2, pinTobcm_BPI_M2, M2_I2C_DEV, M2_SPI_DEV, {M2_PWM_OFFSET,M2_I2C_OFFSET,M2_SPI_OFFSET} },
  { "bpi-m3",	   10201, BPI_MODEL_M3, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M3, physToGpio_BPI_M3, pinTobcm_BPI_M3, M3_I2C_DEV, M3_SPI_DEV, {M3_PWM_OFFSET,M3_I2C_OFFSET,M3_SPI_OFFSET} },
  { "bpi-m2p",	   10301, BPI_MODEL_M2P, 1, 2, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2P, physToGpio_BPI_M2P, pinTobcm_BPI_M2P, M2P_I2C_DEV, M2P_SPI_DEV, {M2P_PWM_OFFSET,M2P_I2C_OFFSET,M2P_SPI_OFFSET} },
  { "bpi-m2-plus", 10301, BPI_MODEL_M2P, 1, 2, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2P, physToGpio_BPI_M2P, pinTobcm_BPI_M2P, M2P_I2C_DEV, M2P_SPI_DEV, {M2P_PWM_OFFSET,M2P_I2C_OFFSET,M2P_SPI_OFFSET} },
  { "bpi-m2-plus-h3", 10301, BPI_MODEL_M2P, 1, 2, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2P, physToGpio_BPI_M2P, pinTobcm_BPI_M2P, M2P_I2C_DEV, M2P_SPI_DEV, {M2P_PWM_OFFSET,M2P_I2C_OFFSET,M2P_SPI_OFFSET} },
  { "bpi-m2p-h3", 10301, BPI_MODEL_M2P, 1, 2, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2P, physToGpio_BPI_M2P, pinTobcm_BPI_M2P, M2P_I2C_DEV, M2P_SPI_DEV, {M2P_PWM_OFFSET,M2P_I2C_OFFSET,M2P_SPI_OFFSET} },
  { "bpi-m2p-480", 10301, BPI_MODEL_M2P, 1, 2, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2P, physToGpio_BPI_M2P, pinTobcm_BPI_M2P, M2P_I2C_DEV, M2P_SPI_DEV, {M2P_PWM_OFFSET,M2P_I2C_OFFSET,M2P_SPI_OFFSET} },
  { "bpi-m64",	   10401, BPI_MODEL_M64, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M64, physToGpio_BPI_M64, pinTobcm_BPI_M64, M64_I2C_DEV, M64_SPI_DEV, {M64_PWM_OFFSET,M64_I2C_OFFSET,M64_SPI_OFFSET} },
  { "bpi-m2u",	   10501, BPI_MODEL_M2U, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2U, physToGpio_BPI_M2U, pinTobcm_BPI_M2U, M2U_I2C_DEV, M2U_SPI_DEV, {M2U_PWM_OFFSET,M2U_I2C_OFFSET,M2U_SPI_OFFSET} },
  { "bpi-m2-ultra", 10501, BPI_MODEL_M2U, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2U, physToGpio_BPI_M2U, pinTobcm_BPI_M2U, M2U_I2C_DEV, M2U_SPI_DEV, {M2U_PWM_OFFSET,M2U_I2C_OFFSET,M2U_SPI_OFFSET} },
  { "bpi-m2m",	   10601, BPI_MODEL_M2M, 1, 1, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2M, physToGpio_BPI_M2M, pinTobcm_BPI_M2M, M2M_I2C_DEV, M2M_SPI_DEV, {M2M_PWM_OFFSET,M2M_I2C_OFFSET,M2M_SPI_OFFSET} },
  { "bpi-m2-magic", 10601, BPI_MODEL_M2M, 1, 1, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2M, physToGpio_BPI_M2M, pinTobcm_BPI_M2M, M2M_I2C_DEV, M2M_SPI_DEV, {M2M_PWM_OFFSET,M2M_I2C_OFFSET,M2M_SPI_OFFSET} },
  { "bpi-m2m-v1.1", 10601, BPI_MODEL_M2M_V11, 1, 1, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2M_V11, physToGpio_BPI_M2M_V11, pinTobcm_BPI_M2M_V11, M2M_V11_I2C_DEV, M2M_V11_SPI_DEV, {M2M_V11_PWM_OFFSET,M2M_V11_I2C_OFFSET,M2M_V11_SPI_OFFSET} },
  { "bpi-m2m-v11", 10601, BPI_MODEL_M2M_V11, 1, 1, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2M_V11, physToGpio_BPI_M2M_V11, pinTobcm_BPI_M2M_V11, M2M_V11_I2C_DEV, M2M_V11_SPI_DEV, {M2M_V11_PWM_OFFSET,M2M_V11_I2C_OFFSET,M2M_V11_SPI_OFFSET} },
  { "bpi-m2-magic-v1.1", 10601, BPI_MODEL_M2M_V11, 1, 1, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2M_V11, physToGpio_BPI_M2M_V11, pinTobcm_BPI_M2M_V11, M2M_V11_I2C_DEV, M2M_V11_SPI_DEV, {M2M_V11_PWM_OFFSET,M2M_V11_I2C_OFFSET,M2M_V11_SPI_OFFSET} },
  { "bpi-m2-magic-v11", 10601, BPI_MODEL_M2M_V11, 1, 1, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2M_V11, physToGpio_BPI_M2M_V11, pinTobcm_BPI_M2M_V11, M2M_V11_I2C_DEV, M2M_V11_SPI_DEV, {M2M_V11_PWM_OFFSET,M2M_V11_I2C_OFFSET,M2M_V11_SPI_OFFSET} },
  { "bpi-m2p_H2+", 10701, BPI_MODEL_M2P_H2P, 1, 2, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2P, physToGpio_BPI_M2P, pinTobcm_BPI_M2P, M2P_I2C_DEV, M2P_SPI_DEV, {M2P_PWM_OFFSET,M2P_I2C_OFFSET,M2P_SPI_OFFSET} },
  { "bpi-m2p-h2+", 10701, BPI_MODEL_M2P_H2P, 1, 2, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2P, physToGpio_BPI_M2P, pinTobcm_BPI_M2P, M2P_I2C_DEV, M2P_SPI_DEV, {M2P_PWM_OFFSET,M2P_I2C_OFFSET,M2P_SPI_OFFSET} },
  { "bpi-m2p-h2p", 10701, BPI_MODEL_M2P_H2P, 1, 2, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2P, physToGpio_BPI_M2P, pinTobcm_BPI_M2P, M2P_I2C_DEV, M2P_SPI_DEV, {M2P_PWM_OFFSET,M2P_I2C_OFFSET,M2P_SPI_OFFSET} },
  { "bpi-m2-plus-h2+", 10701, BPI_MODEL_M2P_H2P, 1, 2, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2P, physToGpio_BPI_M2P, pinTobcm_BPI_M2P, M2P_I2C_DEV, M2P_SPI_DEV, {M2P_PWM_OFFSET,M2P_I2C_OFFSET,M2P_SPI_OFFSET} },
  { "bpi-m2-plus-h2p", 10701, BPI_MODEL_M2P_H2P, 1, 2, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2P, physToGpio_BPI_M2P, pinTobcm_BPI_M2P, M2P_I2C_DEV, M2P_SPI_DEV, {M2P_PWM_OFFSET,M2P_I2C_OFFSET,M2P_SPI_OFFSET} },
  { "bpi-m2p_H5",  10801, BPI_MODEL_M2P_H5, 1, 2, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2P, physToGpio_BPI_M2P, pinTobcm_BPI_M2P, M2P_I2C_DEV, M2P_SPI_DEV, {M2P_PWM_OFFSET,M2P_I2C_OFFSET,M2P_SPI_OFFSET} },
  { "bpi-m2p-h5",  10801, BPI_MODEL_M2P_H5, 1, 2, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2P, physToGpio_BPI_M2P, pinTobcm_BPI_M2P, M2P_I2C_DEV, M2P_SPI_DEV, {M2P_PWM_OFFSET,M2P_I2C_OFFSET,M2P_SPI_OFFSET} },
  { "bpi-m2-plus-h5", 10801, BPI_MODEL_M2P_H5, 1, 2, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2P, physToGpio_BPI_M2P, pinTobcm_BPI_M2P, M2P_I2C_DEV, M2P_SPI_DEV, {M2P_PWM_OFFSET,M2P_I2C_OFFSET,M2P_SPI_OFFSET} },
  { "bpi-m2u_V40", 10901, BPI_MODEL_M2U_V40, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2U, physToGpio_BPI_M2U, pinTobcm_BPI_M2U, M2U_I2C_DEV, M2U_SPI_DEV, {M2U_PWM_OFFSET,M2U_I2C_OFFSET,M2U_SPI_OFFSET} },
  { "bpi-m2u-v40", 10901, BPI_MODEL_M2U_V40, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2U, physToGpio_BPI_M2U, pinTobcm_BPI_M2U, M2U_I2C_DEV, M2U_SPI_DEV, {M2U_PWM_OFFSET,M2U_I2C_OFFSET,M2U_SPI_OFFSET} },
  { "bpi-m2b", 10901, BPI_MODEL_M2U_V40, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2U, physToGpio_BPI_M2U, pinTobcm_BPI_M2U, M2U_I2C_DEV, M2U_SPI_DEV, {M2U_PWM_OFFSET,M2U_I2C_OFFSET,M2U_SPI_OFFSET} },
  { "bpi-m2-berry", 10901, BPI_MODEL_M2U_V40, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2U, physToGpio_BPI_M2U, pinTobcm_BPI_M2U, M2U_I2C_DEV, M2U_SPI_DEV, {M2U_PWM_OFFSET,M2U_I2C_OFFSET,M2U_SPI_OFFSET} },
  { "bpi-6204", 10901, BPI_MODEL_M2U_V40, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2U, physToGpio_BPI_M2U, pinTobcm_BPI_M2U, M2U_I2C_DEV, M2U_SPI_DEV, {M2U_PWM_OFFSET,M2U_I2C_OFFSET,M2U_SPI_OFFSET} },
  { "bpi-cs6204", 10901, BPI_MODEL_M2U_V40, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2U, physToGpio_BPI_M2U, pinTobcm_BPI_M2U, M2U_I2C_DEV, M2U_SPI_DEV, {M2U_PWM_OFFSET,M2U_I2C_OFFSET,M2U_SPI_OFFSET} },
  { "bpi-cs-6204", 10901, BPI_MODEL_M2U_V40, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2U, physToGpio_BPI_M2U, pinTobcm_BPI_M2U, M2U_I2C_DEV, M2U_SPI_DEV, {M2U_PWM_OFFSET,M2U_I2C_OFFSET,M2U_SPI_OFFSET} },
  { "bpi-6202", 10901, BPI_MODEL_M2U_V40, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2U, physToGpio_BPI_M2U, pinTobcm_BPI_M2U, M2U_I2C_DEV, M2U_SPI_DEV, {M2U_PWM_OFFSET,M2U_I2C_OFFSET,M2U_SPI_OFFSET} },
  { "bpi-cs6202", 10901, BPI_MODEL_M2U_V40, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2U, physToGpio_BPI_M2U, pinTobcm_BPI_M2U, M2U_I2C_DEV, M2U_SPI_DEV, {M2U_PWM_OFFSET,M2U_I2C_OFFSET,M2U_SPI_OFFSET} },
  { "bpi-cs-6202", 10901, BPI_MODEL_M2U_V40, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2U, physToGpio_BPI_M2U, pinTobcm_BPI_M2U, M2U_I2C_DEV, M2U_SPI_DEV, {M2U_PWM_OFFSET,M2U_I2C_OFFSET,M2U_SPI_OFFSET} },
  { "bpi-m2z",	   11001, BPI_MODEL_M2Z, 1, 1, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2P, physToGpio_BPI_M2P, pinTobcm_BPI_M2P, M2P_I2C_DEV, M2P_SPI_DEV, {M2P_PWM_OFFSET,M2P_I2C_OFFSET,M2P_SPI_OFFSET} },
  { "bpi-m2-zero", 11001, BPI_MODEL_M2Z, 1, 1, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2P, physToGpio_BPI_M2P, pinTobcm_BPI_M2P, M2P_I2C_DEV, M2P_SPI_DEV, {M2P_PWM_OFFSET,M2P_I2C_OFFSET,M2P_SPI_OFFSET} },
  { "bpi-p2z",	   11001, BPI_MODEL_M2Z, 1, 1, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2P, physToGpio_BPI_M2P, pinTobcm_BPI_M2P, M2P_I2C_DEV, M2P_SPI_DEV, {M2P_PWM_OFFSET,M2P_I2C_OFFSET,M2P_SPI_OFFSET} },
  { "bpi-p2-zero", 11001, BPI_MODEL_M2Z, 1, 1, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2P, physToGpio_BPI_M2P, pinTobcm_BPI_M2P, M2P_I2C_DEV, M2P_SPI_DEV, {M2P_PWM_OFFSET,M2P_I2C_OFFSET,M2P_SPI_OFFSET} },
  { "bpi-m4berry", 11201, BPI_MODEL_M4BERRY, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M4BERRY, physToGpio_BPI_M4BERRY, pinTobcm_BPI_M4BERRY, M4BERRY_I2C_DEV, M4BERRY_SPI_DEV, {M4BERRY_PWM_OFFSET,M4BERRY_I2C_OFFSET,M4BERRY_SPI_OFFSET} },
  { "bpi-m4-berry", 11201, BPI_MODEL_M4BERRY, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M4BERRY, physToGpio_BPI_M4BERRY, pinTobcm_BPI_M4BERRY, M4BERRY_I2C_DEV, M4BERRY_SPI_DEV, {M4BERRY_PWM_OFFSET,M4BERRY_I2C_OFFSET,M4BERRY_SPI_OFFSET} },
  { "bananapim4berry", 11201, BPI_MODEL_M4BERRY, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M4BERRY, physToGpio_BPI_M4BERRY, pinTobcm_BPI_M4BERRY, M4BERRY_I2C_DEV, M4BERRY_SPI_DEV, {M4BERRY_PWM_OFFSET,M4BERRY_I2C_OFFSET,M4BERRY_SPI_OFFSET} },
  { "bpi-m4zero", 11301, BPI_MODEL_M4ZERO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M4ZERO, physToGpio_BPI_M4ZERO, pinTobcm_BPI_M4ZERO, M4ZERO_I2C_DEV, M4ZERO_SPI_DEV, {M4ZERO_PWM_OFFSET,M4ZERO_I2C_OFFSET,M4ZERO_SPI_OFFSET} },
  { "bpi-m4-zero", 11301, BPI_MODEL_M4ZERO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M4ZERO, physToGpio_BPI_M4ZERO, pinTobcm_BPI_M4ZERO, M4ZERO_I2C_DEV, M4ZERO_SPI_DEV, {M4ZERO_PWM_OFFSET,M4ZERO_I2C_OFFSET,M4ZERO_SPI_OFFSET} },
  { "bananapim4zero", 11301, BPI_MODEL_M4ZERO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M4ZERO, physToGpio_BPI_M4ZERO, pinTobcm_BPI_M4ZERO, M4ZERO_I2C_DEV, M4ZERO_SPI_DEV, {M4ZERO_PWM_OFFSET,M4ZERO_I2C_OFFSET,M4ZERO_SPI_OFFSET} },
  { "bpi-m2s",     11401, BPI_MODEL_M2S, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2S, physToGpio_BPI_M2S, pinTobcm_BPI_M2S, M2S_I2C_DEV, M2S_SPI_DEV, {M2S_PWM_OFFSET,M2S_I2C_OFFSET,M2S_SPI_OFFSET} },
  { "bananapim2s", 11401, BPI_MODEL_M2S, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2S, physToGpio_BPI_M2S, pinTobcm_BPI_M2S, M2S_I2C_DEV, M2S_SPI_DEV, {M2S_PWM_OFFSET,M2S_I2C_OFFSET,M2S_SPI_OFFSET} },
  { "banana-pi-m2s", 11401, BPI_MODEL_M2S, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2S, physToGpio_BPI_M2S, pinTobcm_BPI_M2S, M2S_I2C_DEV, M2S_SPI_DEV, {M2S_PWM_OFFSET,M2S_I2C_OFFSET,M2S_SPI_OFFSET} },
  { "bananapi-m2s", 11401, BPI_MODEL_M2S, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2S, physToGpio_BPI_M2S, pinTobcm_BPI_M2S, M2S_I2C_DEV, M2S_SPI_DEV, {M2S_PWM_OFFSET,M2S_I2C_OFFSET,M2S_SPI_OFFSET} },
  { "bpi-cm4io",   11501, BPI_MODEL_CM4IO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_CM4IO, physToGpio_BPI_CM4IO, pinTobcm_BPI_CM4IO, CM4IO_I2C_DEV, CM4IO_SPI_DEV, {CM4IO_PWM_OFFSET,CM4IO_I2C_OFFSET,CM4IO_SPI_OFFSET} },
  { "bpi-cm4-io",  11501, BPI_MODEL_CM4IO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_CM4IO, physToGpio_BPI_CM4IO, pinTobcm_BPI_CM4IO, CM4IO_I2C_DEV, CM4IO_SPI_DEV, {CM4IO_PWM_OFFSET,CM4IO_I2C_OFFSET,CM4IO_SPI_OFFSET} },
  { "bananapicm4io", 11501, BPI_MODEL_CM4IO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_CM4IO, physToGpio_BPI_CM4IO, pinTobcm_BPI_CM4IO, CM4IO_I2C_DEV, CM4IO_SPI_DEV, {CM4IO_PWM_OFFSET,CM4IO_I2C_OFFSET,CM4IO_SPI_OFFSET} },
  { "banana-pi-cm4io", 11501, BPI_MODEL_CM4IO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_CM4IO, physToGpio_BPI_CM4IO, pinTobcm_BPI_CM4IO, CM4IO_I2C_DEV, CM4IO_SPI_DEV, {CM4IO_PWM_OFFSET,CM4IO_I2C_OFFSET,CM4IO_SPI_OFFSET} },
  { "bpi-cm4",     11501, BPI_MODEL_CM4IO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_CM4IO, physToGpio_BPI_CM4IO, pinTobcm_BPI_CM4IO, CM4IO_I2C_DEV, CM4IO_SPI_DEV, {CM4IO_PWM_OFFSET,CM4IO_I2C_OFFSET,CM4IO_SPI_OFFSET} },
  { "bananapicm4", 11501, BPI_MODEL_CM4IO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_CM4IO, physToGpio_BPI_CM4IO, pinTobcm_BPI_CM4IO, CM4IO_I2C_DEV, CM4IO_SPI_DEV, {CM4IO_PWM_OFFSET,CM4IO_I2C_OFFSET,CM4IO_SPI_OFFSET} },
  { "bpi-cm5pro",  12101, BPI_MODEL_CM5PRO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_CM5PRO, physToGpio_BPI_CM5PRO, pinTobcm_BPI_CM5PRO, CM5PRO_I2C_DEV, CM5PRO_SPI_DEV, {CM5PRO_PWM_OFFSET,CM5PRO_I2C_OFFSET,CM5PRO_SPI_OFFSET} },
  { "bpi-cm5-pro", 12101, BPI_MODEL_CM5PRO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_CM5PRO, physToGpio_BPI_CM5PRO, pinTobcm_BPI_CM5PRO, CM5PRO_I2C_DEV, CM5PRO_SPI_DEV, {CM5PRO_PWM_OFFSET,CM5PRO_I2C_OFFSET,CM5PRO_SPI_OFFSET} },
  { "bpi-cm5pro-io", 12101, BPI_MODEL_CM5PRO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_CM5PRO, physToGpio_BPI_CM5PRO, pinTobcm_BPI_CM5PRO, CM5PRO_I2C_DEV, CM5PRO_SPI_DEV, {CM5PRO_PWM_OFFSET,CM5PRO_I2C_OFFSET,CM5PRO_SPI_OFFSET} },
  { "bpi-cm5-pro-io", 12101, BPI_MODEL_CM5PRO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_CM5PRO, physToGpio_BPI_CM5PRO, pinTobcm_BPI_CM5PRO, CM5PRO_I2C_DEV, CM5PRO_SPI_DEV, {CM5PRO_PWM_OFFSET,CM5PRO_I2C_OFFSET,CM5PRO_SPI_OFFSET} },
  { "bananapicm5pro", 12101, BPI_MODEL_CM5PRO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_CM5PRO, physToGpio_BPI_CM5PRO, pinTobcm_BPI_CM5PRO, CM5PRO_I2C_DEV, CM5PRO_SPI_DEV, {CM5PRO_PWM_OFFSET,CM5PRO_I2C_OFFSET,CM5PRO_SPI_OFFSET} },
  { "bananapi-cm5pro", 12101, BPI_MODEL_CM5PRO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_CM5PRO, physToGpio_BPI_CM5PRO, pinTobcm_BPI_CM5PRO, CM5PRO_I2C_DEV, CM5PRO_SPI_DEV, {CM5PRO_PWM_OFFSET,CM5PRO_I2C_OFFSET,CM5PRO_SPI_OFFSET} },
  { "bananapi-cm5-pro", 12101, BPI_MODEL_CM5PRO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_CM5PRO, physToGpio_BPI_CM5PRO, pinTobcm_BPI_CM5PRO, CM5PRO_I2C_DEV, CM5PRO_SPI_DEV, {CM5PRO_PWM_OFFSET,CM5PRO_I2C_OFFSET,CM5PRO_SPI_OFFSET} },
  { "banana-pi-cm5-pro", 12101, BPI_MODEL_CM5PRO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_CM5PRO, physToGpio_BPI_CM5PRO, pinTobcm_BPI_CM5PRO, CM5PRO_I2C_DEV, CM5PRO_SPI_DEV, {CM5PRO_PWM_OFFSET,CM5PRO_I2C_OFFSET,CM5PRO_SPI_OFFSET} },
  { "bpi-m5",      11601, BPI_MODEL_M5, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M5, physToGpio_BPI_M5, pinTobcm_BPI_M5, M5_I2C_DEV, M5_SPI_DEV, {M5_PWM_OFFSET,M5_I2C_OFFSET,M5_SPI_OFFSET} },
  { "bananapim5",  11601, BPI_MODEL_M5, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M5, physToGpio_BPI_M5, pinTobcm_BPI_M5, M5_I2C_DEV, M5_SPI_DEV, {M5_PWM_OFFSET,M5_I2C_OFFSET,M5_SPI_OFFSET} },
  { "banana-pi-m5", 11601, BPI_MODEL_M5, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M5, physToGpio_BPI_M5, pinTobcm_BPI_M5, M5_I2C_DEV, M5_SPI_DEV, {M5_PWM_OFFSET,M5_I2C_OFFSET,M5_SPI_OFFSET} },
  { "bananapi-m5", 11601, BPI_MODEL_M5, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M5, physToGpio_BPI_M5, pinTobcm_BPI_M5, M5_I2C_DEV, M5_SPI_DEV, {M5_PWM_OFFSET,M5_I2C_OFFSET,M5_SPI_OFFSET} },
  { "bpi-m5pro",   12201, BPI_MODEL_M5PRO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M5PRO, physToGpio_BPI_M5PRO, pinTobcm_BPI_M5PRO, M5PRO_I2C_DEV, M5PRO_SPI_DEV, {M5PRO_PWM_OFFSET,M5PRO_I2C_OFFSET,M5PRO_SPI_OFFSET} },
  { "bpi-m5-pro",  12201, BPI_MODEL_M5PRO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M5PRO, physToGpio_BPI_M5PRO, pinTobcm_BPI_M5PRO, M5PRO_I2C_DEV, M5PRO_SPI_DEV, {M5PRO_PWM_OFFSET,M5PRO_I2C_OFFSET,M5PRO_SPI_OFFSET} },
  { "bananapim5pro", 12201, BPI_MODEL_M5PRO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M5PRO, physToGpio_BPI_M5PRO, pinTobcm_BPI_M5PRO, M5PRO_I2C_DEV, M5PRO_SPI_DEV, {M5PRO_PWM_OFFSET,M5PRO_I2C_OFFSET,M5PRO_SPI_OFFSET} },
  { "bananapi-m5pro", 12201, BPI_MODEL_M5PRO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M5PRO, physToGpio_BPI_M5PRO, pinTobcm_BPI_M5PRO, M5PRO_I2C_DEV, M5PRO_SPI_DEV, {M5PRO_PWM_OFFSET,M5PRO_I2C_OFFSET,M5PRO_SPI_OFFSET} },
  { "bananapi-m5-pro", 12201, BPI_MODEL_M5PRO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M5PRO, physToGpio_BPI_M5PRO, pinTobcm_BPI_M5PRO, M5PRO_I2C_DEV, M5PRO_SPI_DEV, {M5PRO_PWM_OFFSET,M5PRO_I2C_OFFSET,M5PRO_SPI_OFFSET} },
  { "banana-pi-m5-pro", 12201, BPI_MODEL_M5PRO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M5PRO, physToGpio_BPI_M5PRO, pinTobcm_BPI_M5PRO, M5PRO_I2C_DEV, M5PRO_SPI_DEV, {M5PRO_PWM_OFFSET,M5PRO_I2C_OFFSET,M5PRO_SPI_OFFSET} },
  { "bpi-m2pro",   11701, BPI_MODEL_M2PRO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M5, physToGpio_BPI_M5, pinTobcm_BPI_M5, M5_I2C_DEV, M5_SPI_DEV, {M5_PWM_OFFSET,M5_I2C_OFFSET,M5_SPI_OFFSET} },
  { "bpi-m2-pro",  11701, BPI_MODEL_M2PRO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M5, physToGpio_BPI_M5, pinTobcm_BPI_M5, M5_I2C_DEV, M5_SPI_DEV, {M5_PWM_OFFSET,M5_I2C_OFFSET,M5_SPI_OFFSET} },
  { "bananapim2pro", 11701, BPI_MODEL_M2PRO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M5, physToGpio_BPI_M5, pinTobcm_BPI_M5, M5_I2C_DEV, M5_SPI_DEV, {M5_PWM_OFFSET,M5_I2C_OFFSET,M5_SPI_OFFSET} },
  { "bananapi-m2pro", 11701, BPI_MODEL_M2PRO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M5, physToGpio_BPI_M5, pinTobcm_BPI_M5, M5_I2C_DEV, M5_SPI_DEV, {M5_PWM_OFFSET,M5_I2C_OFFSET,M5_SPI_OFFSET} },
  { "bananapi-m2-pro", 11701, BPI_MODEL_M2PRO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M5, physToGpio_BPI_M5, pinTobcm_BPI_M5, M5_I2C_DEV, M5_SPI_DEV, {M5_PWM_OFFSET,M5_I2C_OFFSET,M5_SPI_OFFSET} },
  { "banana-pi-m2pro", 11701, BPI_MODEL_M2PRO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M5, physToGpio_BPI_M5, pinTobcm_BPI_M5, M5_I2C_DEV, M5_SPI_DEV, {M5_PWM_OFFSET,M5_I2C_OFFSET,M5_SPI_OFFSET} },
  { "banana-pi-m2-pro", 11701, BPI_MODEL_M2PRO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M5, physToGpio_BPI_M5, pinTobcm_BPI_M5, M5_I2C_DEV, M5_SPI_DEV, {M5_PWM_OFFSET,M5_I2C_OFFSET,M5_SPI_OFFSET} },
  { "bpi-f3",      11801, BPI_MODEL_F3, 1, 4, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_F3, physToGpio_BPI_F3, pinTobcm_BPI_F3, F3_I2C_DEV, F3_SPI_DEV, {F3_PWM_OFFSET,F3_I2C_OFFSET,F3_SPI_OFFSET} },
  { "bananapif3",  11801, BPI_MODEL_F3, 1, 4, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_F3, physToGpio_BPI_F3, pinTobcm_BPI_F3, F3_I2C_DEV, F3_SPI_DEV, {F3_PWM_OFFSET,F3_I2C_OFFSET,F3_SPI_OFFSET} },
  { "banana-pi-f3", 11801, BPI_MODEL_F3, 1, 4, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_F3, physToGpio_BPI_F3, pinTobcm_BPI_F3, F3_I2C_DEV, F3_SPI_DEV, {F3_PWM_OFFSET,F3_I2C_OFFSET,F3_SPI_OFFSET} },
  { "bananapi-f3", 11801, BPI_MODEL_F3, 1, 4, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_F3, physToGpio_BPI_F3, pinTobcm_BPI_F3, F3_I2C_DEV, F3_SPI_DEV, {F3_PWM_OFFSET,F3_I2C_OFFSET,F3_SPI_OFFSET} },
  { "bpi-cm6",     13501, BPI_MODEL_CM6, 1, 4, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_CM6, physToGpio_BPI_CM6, pinTobcm_BPI_CM6, CM6_I2C_DEV, CM6_SPI_DEV, {CM6_PWM_OFFSET,CM6_I2C_OFFSET,CM6_SPI_OFFSET} },
  { "bananapicm6", 13501, BPI_MODEL_CM6, 1, 4, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_CM6, physToGpio_BPI_CM6, pinTobcm_BPI_CM6, CM6_I2C_DEV, CM6_SPI_DEV, {CM6_PWM_OFFSET,CM6_I2C_OFFSET,CM6_SPI_OFFSET} },
  { "bananapi-cm6", 13501, BPI_MODEL_CM6, 1, 4, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_CM6, physToGpio_BPI_CM6, pinTobcm_BPI_CM6, CM6_I2C_DEV, CM6_SPI_DEV, {CM6_PWM_OFFSET,CM6_I2C_OFFSET,CM6_SPI_OFFSET} },
  { "banana-pi-cm6", 13501, BPI_MODEL_CM6, 1, 4, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_CM6, physToGpio_BPI_CM6, pinTobcm_BPI_CM6, CM6_I2C_DEV, CM6_SPI_DEV, {CM6_PWM_OFFSET,CM6_I2C_OFFSET,CM6_SPI_OFFSET} },
  { "bpi-cm6-io",  13501, BPI_MODEL_CM6, 1, 4, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_CM6, physToGpio_BPI_CM6, pinTobcm_BPI_CM6, CM6_I2C_DEV, CM6_SPI_DEV, {CM6_PWM_OFFSET,CM6_I2C_OFFSET,CM6_SPI_OFFSET} },
  { "bpi-ai2n",    11901, BPI_MODEL_AI2N, 1, 5, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_AI2N, physToGpio_BPI_AI2N, pinTobcm_BPI_AI2N, AI2N_I2C_DEV, AI2N_SPI_DEV, {AI2N_PWM_OFFSET,AI2N_I2C_OFFSET,AI2N_SPI_OFFSET} },
  { "bpi-ai2-n",   11901, BPI_MODEL_AI2N, 1, 5, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_AI2N, physToGpio_BPI_AI2N, pinTobcm_BPI_AI2N, AI2N_I2C_DEV, AI2N_SPI_DEV, {AI2N_PWM_OFFSET,AI2N_I2C_OFFSET,AI2N_SPI_OFFSET} },
  { "bananapiai2n", 11901, BPI_MODEL_AI2N, 1, 5, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_AI2N, physToGpio_BPI_AI2N, pinTobcm_BPI_AI2N, AI2N_I2C_DEV, AI2N_SPI_DEV, {AI2N_PWM_OFFSET,AI2N_I2C_OFFSET,AI2N_SPI_OFFSET} },
  { "banana-pi-ai2n", 11901, BPI_MODEL_AI2N, 1, 5, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_AI2N, physToGpio_BPI_AI2N, pinTobcm_BPI_AI2N, AI2N_I2C_DEV, AI2N_SPI_DEV, {AI2N_PWM_OFFSET,AI2N_I2C_OFFSET,AI2N_SPI_OFFSET} },
  { "bananapi-ai2n", 11901, BPI_MODEL_AI2N, 1, 5, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_AI2N, physToGpio_BPI_AI2N, pinTobcm_BPI_AI2N, AI2N_I2C_DEV, AI2N_SPI_DEV, {AI2N_PWM_OFFSET,AI2N_I2C_OFFSET,AI2N_SPI_OFFSET} },
  { "bpi-r2pro",   12001, BPI_MODEL_R2PRO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_R2PRO, physToGpio_BPI_R2PRO, pinTobcm_BPI_R2PRO, R2PRO_I2C_DEV, R2PRO_SPI_DEV, {R2PRO_PWM_OFFSET,R2PRO_I2C_OFFSET,R2PRO_SPI_OFFSET} },
  { "bpi-r2-pro",  12001, BPI_MODEL_R2PRO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_R2PRO, physToGpio_BPI_R2PRO, pinTobcm_BPI_R2PRO, R2PRO_I2C_DEV, R2PRO_SPI_DEV, {R2PRO_PWM_OFFSET,R2PRO_I2C_OFFSET,R2PRO_SPI_OFFSET} },
  { "bananapir2pro", 12001, BPI_MODEL_R2PRO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_R2PRO, physToGpio_BPI_R2PRO, pinTobcm_BPI_R2PRO, R2PRO_I2C_DEV, R2PRO_SPI_DEV, {R2PRO_PWM_OFFSET,R2PRO_I2C_OFFSET,R2PRO_SPI_OFFSET} },
  { "bananapi-r2pro", 12001, BPI_MODEL_R2PRO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_R2PRO, physToGpio_BPI_R2PRO, pinTobcm_BPI_R2PRO, R2PRO_I2C_DEV, R2PRO_SPI_DEV, {R2PRO_PWM_OFFSET,R2PRO_I2C_OFFSET,R2PRO_SPI_OFFSET} },
  { "bananapi-r2-pro", 12001, BPI_MODEL_R2PRO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_R2PRO, physToGpio_BPI_R2PRO, pinTobcm_BPI_R2PRO, R2PRO_I2C_DEV, R2PRO_SPI_DEV, {R2PRO_PWM_OFFSET,R2PRO_I2C_OFFSET,R2PRO_SPI_OFFSET} },
  { "banana-pi-r2-pro", 12001, BPI_MODEL_R2PRO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_R2PRO, physToGpio_BPI_R2PRO, pinTobcm_BPI_R2PRO, R2PRO_I2C_DEV, R2PRO_SPI_DEV, {R2PRO_PWM_OFFSET,R2PRO_I2C_OFFSET,R2PRO_SPI_OFFSET} },
  { "bpi-m7",      12301, BPI_MODEL_M7, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M7, physToGpio_BPI_M7, pinTobcm_BPI_M7, M7_I2C_DEV, M7_SPI_DEV, {M7_PWM_OFFSET,M7_I2C_OFFSET,M7_SPI_OFFSET} },
  { "bananapim7",  12301, BPI_MODEL_M7, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M7, physToGpio_BPI_M7, pinTobcm_BPI_M7, M7_I2C_DEV, M7_SPI_DEV, {M7_PWM_OFFSET,M7_I2C_OFFSET,M7_SPI_OFFSET} },
  { "banana-pi-m7", 12301, BPI_MODEL_M7, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M7, physToGpio_BPI_M7, pinTobcm_BPI_M7, M7_I2C_DEV, M7_SPI_DEV, {M7_PWM_OFFSET,M7_I2C_OFFSET,M7_SPI_OFFSET} },
  { "bananapi-m7", 12301, BPI_MODEL_M7, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M7, physToGpio_BPI_M7, pinTobcm_BPI_M7, M7_I2C_DEV, M7_SPI_DEV, {M7_PWM_OFFSET,M7_I2C_OFFSET,M7_SPI_OFFSET} },
  { "bpi-w3",      12401, BPI_MODEL_W3, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_W3, physToGpio_BPI_W3, pinTobcm_BPI_W3, W3_I2C_DEV, W3_SPI_DEV, {W3_PWM_OFFSET,W3_I2C_OFFSET,W3_SPI_OFFSET} },
  { "bananapiw3",  12401, BPI_MODEL_W3, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_W3, physToGpio_BPI_W3, pinTobcm_BPI_W3, W3_I2C_DEV, W3_SPI_DEV, {W3_PWM_OFFSET,W3_I2C_OFFSET,W3_SPI_OFFSET} },
  { "banana-pi-w3", 12401, BPI_MODEL_W3, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_W3, physToGpio_BPI_W3, pinTobcm_BPI_W3, W3_I2C_DEV, W3_SPI_DEV, {W3_PWM_OFFSET,W3_I2C_OFFSET,W3_SPI_OFFSET} },
  { "bananapi-w3", 12401, BPI_MODEL_W3, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_W3, physToGpio_BPI_W3, pinTobcm_BPI_W3, W3_I2C_DEV, W3_SPI_DEV, {W3_PWM_OFFSET,W3_I2C_OFFSET,W3_SPI_OFFSET} },
  { "armsom-w3",   12401, BPI_MODEL_W3, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_W3, physToGpio_BPI_W3, pinTobcm_BPI_W3, W3_I2C_DEV, W3_SPI_DEV, {W3_PWM_OFFSET,W3_I2C_OFFSET,W3_SPI_OFFSET} },
  { "bpi-aim7",    12501, BPI_MODEL_AIM7, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M7, physToGpio_BPI_M7, pinTobcm_BPI_M7, M7_I2C_DEV, M7_SPI_DEV, {M7_PWM_OFFSET,M7_I2C_OFFSET,M7_SPI_OFFSET} },
  { "bananapiaim7", 12501, BPI_MODEL_AIM7, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M7, physToGpio_BPI_M7, pinTobcm_BPI_M7, M7_I2C_DEV, M7_SPI_DEV, {M7_PWM_OFFSET,M7_I2C_OFFSET,M7_SPI_OFFSET} },
  { "banana-pi-aim7", 12501, BPI_MODEL_AIM7, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M7, physToGpio_BPI_M7, pinTobcm_BPI_M7, M7_I2C_DEV, M7_SPI_DEV, {M7_PWM_OFFSET,M7_I2C_OFFSET,M7_SPI_OFFSET} },
  { "bananapi-aim7", 12501, BPI_MODEL_AIM7, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M7, physToGpio_BPI_M7, pinTobcm_BPI_M7, M7_I2C_DEV, M7_SPI_DEV, {M7_PWM_OFFSET,M7_I2C_OFFSET,M7_SPI_OFFSET} },
  { "armsom-aim7", 12501, BPI_MODEL_AIM7, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M7, physToGpio_BPI_M7, pinTobcm_BPI_M7, M7_I2C_DEV, M7_SPI_DEV, {M7_PWM_OFFSET,M7_I2C_OFFSET,M7_SPI_OFFSET} },
  { "armsom-aim7-io", 12501, BPI_MODEL_AIM7, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M7, physToGpio_BPI_M7, pinTobcm_BPI_M7, M7_I2C_DEV, M7_SPI_DEV, {M7_PWM_OFFSET,M7_I2C_OFFSET,M7_SPI_OFFSET} },
  { "bpi-m4super", 12601, BPI_MODEL_M4SUPER, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M4SUPER, physToGpio_BPI_M4SUPER, pinTobcm_BPI_M4SUPER, M4SUPER_I2C_DEV, M4SUPER_SPI_DEV, {M4SUPER_PWM_OFFSET,M4SUPER_I2C_OFFSET,M4SUPER_SPI_OFFSET} },
  { "bpi-m4-super", 12601, BPI_MODEL_M4SUPER, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M4SUPER, physToGpio_BPI_M4SUPER, pinTobcm_BPI_M4SUPER, M4SUPER_I2C_DEV, M4SUPER_SPI_DEV, {M4SUPER_PWM_OFFSET,M4SUPER_I2C_OFFSET,M4SUPER_SPI_OFFSET} },
  { "bananapim4super", 12601, BPI_MODEL_M4SUPER, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M4SUPER, physToGpio_BPI_M4SUPER, pinTobcm_BPI_M4SUPER, M4SUPER_I2C_DEV, M4SUPER_SPI_DEV, {M4SUPER_PWM_OFFSET,M4SUPER_I2C_OFFSET,M4SUPER_SPI_OFFSET} },
  { "banana-pi-m4-super", 12601, BPI_MODEL_M4SUPER, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M4SUPER, physToGpio_BPI_M4SUPER, pinTobcm_BPI_M4SUPER, M4SUPER_I2C_DEV, M4SUPER_SPI_DEV, {M4SUPER_PWM_OFFSET,M4SUPER_I2C_OFFSET,M4SUPER_SPI_OFFSET} },
  { "bananapi-m4super", 12601, BPI_MODEL_M4SUPER, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M4SUPER, physToGpio_BPI_M4SUPER, pinTobcm_BPI_M4SUPER, M4SUPER_I2C_DEV, M4SUPER_SPI_DEV, {M4SUPER_PWM_OFFSET,M4SUPER_I2C_OFFSET,M4SUPER_SPI_OFFSET} },
  { "bananapi-m4-super", 12601, BPI_MODEL_M4SUPER, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M4SUPER, physToGpio_BPI_M4SUPER, pinTobcm_BPI_M4SUPER, M4SUPER_I2C_DEV, M4SUPER_SPI_DEV, {M4SUPER_PWM_OFFSET,M4SUPER_I2C_OFFSET,M4SUPER_SPI_OFFSET} },
  { "armsom-sige3", 12601, BPI_MODEL_M4SUPER, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M4SUPER, physToGpio_BPI_M4SUPER, pinTobcm_BPI_M4SUPER, M4SUPER_I2C_DEV, M4SUPER_SPI_DEV, {M4SUPER_PWM_OFFSET,M4SUPER_I2C_OFFSET,M4SUPER_SPI_OFFSET} },
  { "bpi-m1super", 12701, BPI_MODEL_M1SUPER, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M1SUPER, physToGpio_BPI_M1SUPER, pinTobcm_BPI_M1SUPER, M1SUPER_I2C_DEV, M1SUPER_SPI_DEV, {M1SUPER_PWM_OFFSET,M1SUPER_I2C_OFFSET,M1SUPER_SPI_OFFSET} },
  { "bpi-m1-super", 12701, BPI_MODEL_M1SUPER, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M1SUPER, physToGpio_BPI_M1SUPER, pinTobcm_BPI_M1SUPER, M1SUPER_I2C_DEV, M1SUPER_SPI_DEV, {M1SUPER_PWM_OFFSET,M1SUPER_I2C_OFFSET,M1SUPER_SPI_OFFSET} },
  { "bpi-m1s", 12701, BPI_MODEL_M1SUPER, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M1SUPER, physToGpio_BPI_M1SUPER, pinTobcm_BPI_M1SUPER, M1SUPER_I2C_DEV, M1SUPER_SPI_DEV, {M1SUPER_PWM_OFFSET,M1SUPER_I2C_OFFSET,M1SUPER_SPI_OFFSET} },
  { "bananapim1super", 12701, BPI_MODEL_M1SUPER, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M1SUPER, physToGpio_BPI_M1SUPER, pinTobcm_BPI_M1SUPER, M1SUPER_I2C_DEV, M1SUPER_SPI_DEV, {M1SUPER_PWM_OFFSET,M1SUPER_I2C_OFFSET,M1SUPER_SPI_OFFSET} },
  { "banana-pi-m1-super", 12701, BPI_MODEL_M1SUPER, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M1SUPER, physToGpio_BPI_M1SUPER, pinTobcm_BPI_M1SUPER, M1SUPER_I2C_DEV, M1SUPER_SPI_DEV, {M1SUPER_PWM_OFFSET,M1SUPER_I2C_OFFSET,M1SUPER_SPI_OFFSET} },
  { "bananapi-m1super", 12701, BPI_MODEL_M1SUPER, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M1SUPER, physToGpio_BPI_M1SUPER, pinTobcm_BPI_M1SUPER, M1SUPER_I2C_DEV, M1SUPER_SPI_DEV, {M1SUPER_PWM_OFFSET,M1SUPER_I2C_OFFSET,M1SUPER_SPI_OFFSET} },
  { "bananapi-m1-super", 12701, BPI_MODEL_M1SUPER, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M1SUPER, physToGpio_BPI_M1SUPER, pinTobcm_BPI_M1SUPER, M1SUPER_I2C_DEV, M1SUPER_SPI_DEV, {M1SUPER_PWM_OFFSET,M1SUPER_I2C_OFFSET,M1SUPER_SPI_OFFSET} },
  { "bananapi-m1s", 12701, BPI_MODEL_M1SUPER, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M1SUPER, physToGpio_BPI_M1SUPER, pinTobcm_BPI_M1SUPER, M1SUPER_I2C_DEV, M1SUPER_SPI_DEV, {M1SUPER_PWM_OFFSET,M1SUPER_I2C_OFFSET,M1SUPER_SPI_OFFSET} },
  { "armsom-sige1", 12701, BPI_MODEL_M1SUPER, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M1SUPER, physToGpio_BPI_M1SUPER, pinTobcm_BPI_M1SUPER, M1SUPER_I2C_DEV, M1SUPER_SPI_DEV, {M1SUPER_PWM_OFFSET,M1SUPER_I2C_OFFSET,M1SUPER_SPI_OFFSET} },
  { "bpi-forge1", 12801, BPI_MODEL_FORGE1, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M1SUPER, physToGpio_BPI_M1SUPER, pinTobcm_BPI_M1SUPER, M1SUPER_I2C_DEV, M1SUPER_SPI_DEV, {M1SUPER_PWM_OFFSET,M1SUPER_I2C_OFFSET,M1SUPER_SPI_OFFSET} },
  { "bananapiforge1", 12801, BPI_MODEL_FORGE1, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M1SUPER, physToGpio_BPI_M1SUPER, pinTobcm_BPI_M1SUPER, M1SUPER_I2C_DEV, M1SUPER_SPI_DEV, {M1SUPER_PWM_OFFSET,M1SUPER_I2C_OFFSET,M1SUPER_SPI_OFFSET} },
  { "banana-pi-forge1", 12801, BPI_MODEL_FORGE1, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M1SUPER, physToGpio_BPI_M1SUPER, pinTobcm_BPI_M1SUPER, M1SUPER_I2C_DEV, M1SUPER_SPI_DEV, {M1SUPER_PWM_OFFSET,M1SUPER_I2C_OFFSET,M1SUPER_SPI_OFFSET} },
  { "bananapi-forge1", 12801, BPI_MODEL_FORGE1, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M1SUPER, physToGpio_BPI_M1SUPER, pinTobcm_BPI_M1SUPER, M1SUPER_I2C_DEV, M1SUPER_SPI_DEV, {M1SUPER_PWM_OFFSET,M1SUPER_I2C_OFFSET,M1SUPER_SPI_OFFSET} },
  { "armsom-forge1", 12801, BPI_MODEL_FORGE1, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M1SUPER, physToGpio_BPI_M1SUPER, pinTobcm_BPI_M1SUPER, M1SUPER_I2C_DEV, M1SUPER_SPI_DEV, {M1SUPER_PWM_OFFSET,M1SUPER_I2C_OFFSET,M1SUPER_SPI_OFFSET} },
  { "bpi-p2pro",   12901, BPI_MODEL_P2PRO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_P2PRO, physToGpio_BPI_P2PRO, pinTobcm_BPI_P2PRO, P2PRO_I2C_DEV, P2PRO_SPI_DEV, {P2PRO_PWM_OFFSET,P2PRO_I2C_OFFSET,P2PRO_SPI_OFFSET} },
  { "bpi-p2-pro",  12901, BPI_MODEL_P2PRO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_P2PRO, physToGpio_BPI_P2PRO, pinTobcm_BPI_P2PRO, P2PRO_I2C_DEV, P2PRO_SPI_DEV, {P2PRO_PWM_OFFSET,P2PRO_I2C_OFFSET,P2PRO_SPI_OFFSET} },
  { "bananapip2pro", 12901, BPI_MODEL_P2PRO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_P2PRO, physToGpio_BPI_P2PRO, pinTobcm_BPI_P2PRO, P2PRO_I2C_DEV, P2PRO_SPI_DEV, {P2PRO_PWM_OFFSET,P2PRO_I2C_OFFSET,P2PRO_SPI_OFFSET} },
  { "bananapi-p2pro", 12901, BPI_MODEL_P2PRO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_P2PRO, physToGpio_BPI_P2PRO, pinTobcm_BPI_P2PRO, P2PRO_I2C_DEV, P2PRO_SPI_DEV, {P2PRO_PWM_OFFSET,P2PRO_I2C_OFFSET,P2PRO_SPI_OFFSET} },
  { "bananapi-p2-pro", 12901, BPI_MODEL_P2PRO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_P2PRO, physToGpio_BPI_P2PRO, pinTobcm_BPI_P2PRO, P2PRO_I2C_DEV, P2PRO_SPI_DEV, {P2PRO_PWM_OFFSET,P2PRO_I2C_OFFSET,P2PRO_SPI_OFFSET} },
  { "banana-pi-p2-pro", 12901, BPI_MODEL_P2PRO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_P2PRO, physToGpio_BPI_P2PRO, pinTobcm_BPI_P2PRO, P2PRO_I2C_DEV, P2PRO_SPI_DEV, {P2PRO_PWM_OFFSET,P2PRO_I2C_OFFSET,P2PRO_SPI_OFFSET} },
  { "armsom-p2pro", 12901, BPI_MODEL_P2PRO, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_P2PRO, physToGpio_BPI_P2PRO, pinTobcm_BPI_P2PRO, P2PRO_I2C_DEV, P2PRO_SPI_DEV, {P2PRO_PWM_OFFSET,P2PRO_I2C_OFFSET,P2PRO_SPI_OFFSET} },
  { "bpi-w2",      13001, BPI_MODEL_W2, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_W2, physToGpio_BPI_W2, pinTobcm_BPI_W2, W2_I2C_DEV, W2_SPI_DEV, {W2_PWM_OFFSET,W2_I2C_OFFSET,W2_SPI_OFFSET} },
  { "bananapiw2",  13001, BPI_MODEL_W2, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_W2, physToGpio_BPI_W2, pinTobcm_BPI_W2, W2_I2C_DEV, W2_SPI_DEV, {W2_PWM_OFFSET,W2_I2C_OFFSET,W2_SPI_OFFSET} },
  { "bananapi-w2", 13001, BPI_MODEL_W2, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_W2, physToGpio_BPI_W2, pinTobcm_BPI_W2, W2_I2C_DEV, W2_SPI_DEV, {W2_PWM_OFFSET,W2_I2C_OFFSET,W2_SPI_OFFSET} },
  { "banana-pi-w2", 13001, BPI_MODEL_W2, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_W2, physToGpio_BPI_W2, pinTobcm_BPI_W2, W2_I2C_DEV, W2_SPI_DEV, {W2_PWM_OFFSET,W2_I2C_OFFSET,W2_SPI_OFFSET} },
  { "bpi-m4",      13101, BPI_MODEL_M4, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M4, physToGpio_BPI_M4, pinTobcm_BPI_M4, M4_I2C_DEV, M4_SPI_DEV, {M4_PWM_OFFSET,M4_I2C_OFFSET,M4_SPI_OFFSET} },
  { "bananapim4",  13101, BPI_MODEL_M4, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M4, physToGpio_BPI_M4, pinTobcm_BPI_M4, M4_I2C_DEV, M4_SPI_DEV, {M4_PWM_OFFSET,M4_I2C_OFFSET,M4_SPI_OFFSET} },
  { "bananapi-m4", 13101, BPI_MODEL_M4, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M4, physToGpio_BPI_M4, pinTobcm_BPI_M4, M4_I2C_DEV, M4_SPI_DEV, {M4_PWM_OFFSET,M4_I2C_OFFSET,M4_SPI_OFFSET} },
  { "banana-pi-m4", 13101, BPI_MODEL_M4, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M4, physToGpio_BPI_M4, pinTobcm_BPI_M4, M4_I2C_DEV, M4_SPI_DEV, {M4_PWM_OFFSET,M4_I2C_OFFSET,M4_SPI_OFFSET} },
  { "bpi-m6",      13201, BPI_MODEL_M6, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M6, physToGpio_BPI_M6, pinTobcm_BPI_M6, M6_I2C_DEV, M6_SPI_DEV, {M6_PWM_OFFSET,M6_I2C_OFFSET,M6_SPI_OFFSET} },
  { "bananapim6",  13201, BPI_MODEL_M6, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M6, physToGpio_BPI_M6, pinTobcm_BPI_M6, M6_I2C_DEV, M6_SPI_DEV, {M6_PWM_OFFSET,M6_I2C_OFFSET,M6_SPI_OFFSET} },
  { "bananapi-m6", 13201, BPI_MODEL_M6, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M6, physToGpio_BPI_M6, pinTobcm_BPI_M6, M6_I2C_DEV, M6_SPI_DEV, {M6_PWM_OFFSET,M6_I2C_OFFSET,M6_SPI_OFFSET} },
  { "banana-pi-m6", 13201, BPI_MODEL_M6, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M6, physToGpio_BPI_M6, pinTobcm_BPI_M6, M6_I2C_DEV, M6_SPI_DEV, {M6_PWM_OFFSET,M6_I2C_OFFSET,M6_SPI_OFFSET} },
  { "bpi-f2s",     13301, BPI_MODEL_F2S, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_F2S_F2P, physToGpio_BPI_F2S_F2P, pinTobcm_BPI_F2S_F2P, F2S_F2P_I2C_DEV, F2S_F2P_SPI_DEV, {F2S_F2P_PWM_OFFSET,F2S_F2P_I2C_OFFSET,F2S_F2P_SPI_OFFSET} },
  { "bananapif2s", 13301, BPI_MODEL_F2S, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_F2S_F2P, physToGpio_BPI_F2S_F2P, pinTobcm_BPI_F2S_F2P, F2S_F2P_I2C_DEV, F2S_F2P_SPI_DEV, {F2S_F2P_PWM_OFFSET,F2S_F2P_I2C_OFFSET,F2S_F2P_SPI_OFFSET} },
  { "bananapi-f2s", 13301, BPI_MODEL_F2S, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_F2S_F2P, physToGpio_BPI_F2S_F2P, pinTobcm_BPI_F2S_F2P, F2S_F2P_I2C_DEV, F2S_F2P_SPI_DEV, {F2S_F2P_PWM_OFFSET,F2S_F2P_I2C_OFFSET,F2S_F2P_SPI_OFFSET} },
  { "banana-pi-f2s", 13301, BPI_MODEL_F2S, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_F2S_F2P, physToGpio_BPI_F2S_F2P, pinTobcm_BPI_F2S_F2P, F2S_F2P_I2C_DEV, F2S_F2P_SPI_DEV, {F2S_F2P_PWM_OFFSET,F2S_F2P_I2C_OFFSET,F2S_F2P_SPI_OFFSET} },
  { "bpi-f2p",     13401, BPI_MODEL_F2P, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_F2S_F2P, physToGpio_BPI_F2S_F2P, pinTobcm_BPI_F2S_F2P, F2S_F2P_I2C_DEV, F2S_F2P_SPI_DEV, {F2S_F2P_PWM_OFFSET,F2S_F2P_I2C_OFFSET,F2S_F2P_SPI_OFFSET} },
  { "bananapif2p", 13401, BPI_MODEL_F2P, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_F2S_F2P, physToGpio_BPI_F2S_F2P, pinTobcm_BPI_F2S_F2P, F2S_F2P_I2C_DEV, F2S_F2P_SPI_DEV, {F2S_F2P_PWM_OFFSET,F2S_F2P_I2C_OFFSET,F2S_F2P_SPI_OFFSET} },
  { "bananapi-f2p", 13401, BPI_MODEL_F2P, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_F2S_F2P, physToGpio_BPI_F2S_F2P, pinTobcm_BPI_F2S_F2P, F2S_F2P_I2C_DEV, F2S_F2P_SPI_DEV, {F2S_F2P_PWM_OFFSET,F2S_F2P_I2C_OFFSET,F2S_F2P_SPI_OFFSET} },
  { "banana-pi-f2p", 13401, BPI_MODEL_F2P, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_F2S_F2P, physToGpio_BPI_F2S_F2P, pinTobcm_BPI_F2S_F2P, F2S_F2P_I2C_DEV, F2S_F2P_SPI_DEV, {F2S_F2P_PWM_OFFSET,F2S_F2P_I2C_OFFSET,F2S_F2P_SPI_OFFSET} },
  { "bpi-f4",      14101, BPI_MODEL_F4, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_F4, physToGpio_BPI_F4, pinTobcm_BPI_F4, F4_I2C_DEV, F4_SPI_DEV, {F4_PWM_OFFSET,F4_I2C_OFFSET,F4_SPI_OFFSET} },
  { "bananapif4",  14101, BPI_MODEL_F4, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_F4, physToGpio_BPI_F4, pinTobcm_BPI_F4, F4_I2C_DEV, F4_SPI_DEV, {F4_PWM_OFFSET,F4_I2C_OFFSET,F4_SPI_OFFSET} },
  { "bananapi-f4", 14101, BPI_MODEL_F4, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_F4, physToGpio_BPI_F4, pinTobcm_BPI_F4, F4_I2C_DEV, F4_SPI_DEV, {F4_PWM_OFFSET,F4_I2C_OFFSET,F4_SPI_OFFSET} },
  { "banana-pi-f4", 14101, BPI_MODEL_F4, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_F4, physToGpio_BPI_F4, pinTobcm_BPI_F4, F4_I2C_DEV, F4_SPI_DEV, {F4_PWM_OFFSET,F4_I2C_OFFSET,F4_SPI_OFFSET} },
  { "bpi-r4",      13601, BPI_MODEL_R4, 1, 4, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_R4, physToGpio_BPI_R4, pinTobcm_BPI_R4, R4_I2C_DEV, R4_SPI_DEV, {R4_PWM_OFFSET,R4_I2C_OFFSET,R4_SPI_OFFSET} },
  { "bananapir4",  13601, BPI_MODEL_R4, 1, 4, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_R4, physToGpio_BPI_R4, pinTobcm_BPI_R4, R4_I2C_DEV, R4_SPI_DEV, {R4_PWM_OFFSET,R4_I2C_OFFSET,R4_SPI_OFFSET} },
  { "bananapi-r4", 13601, BPI_MODEL_R4, 1, 4, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_R4, physToGpio_BPI_R4, pinTobcm_BPI_R4, R4_I2C_DEV, R4_SPI_DEV, {R4_PWM_OFFSET,R4_I2C_OFFSET,R4_SPI_OFFSET} },
  { "banana-pi-r4", 13601, BPI_MODEL_R4, 1, 4, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_R4, physToGpio_BPI_R4, pinTobcm_BPI_R4, R4_I2C_DEV, R4_SPI_DEV, {R4_PWM_OFFSET,R4_I2C_OFFSET,R4_SPI_OFFSET} },
  { "bpi-r3",      13701, BPI_MODEL_R3, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_R3, physToGpio_BPI_R3, pinTobcm_BPI_R3, R3_I2C_DEV, R3_SPI_DEV, {R3_PWM_OFFSET,R3_I2C_OFFSET,R3_SPI_OFFSET} },
  { "bananapir3",  13701, BPI_MODEL_R3, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_R3, physToGpio_BPI_R3, pinTobcm_BPI_R3, R3_I2C_DEV, R3_SPI_DEV, {R3_PWM_OFFSET,R3_I2C_OFFSET,R3_SPI_OFFSET} },
  { "bananapi-r3", 13701, BPI_MODEL_R3, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_R3, physToGpio_BPI_R3, pinTobcm_BPI_R3, R3_I2C_DEV, R3_SPI_DEV, {R3_PWM_OFFSET,R3_I2C_OFFSET,R3_SPI_OFFSET} },
  { "banana-pi-r3", 13701, BPI_MODEL_R3, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_R3, physToGpio_BPI_R3, pinTobcm_BPI_R3, R3_I2C_DEV, R3_SPI_DEV, {R3_PWM_OFFSET,R3_I2C_OFFSET,R3_SPI_OFFSET} },
  { "bpi-r64",     13801, BPI_MODEL_R64, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_R64, physToGpio_BPI_R64, pinTobcm_BPI_R64, R64_I2C_DEV, R64_SPI_DEV, {R64_PWM_OFFSET,R64_I2C_OFFSET,R64_SPI_OFFSET} },
  { "bananapir64", 13801, BPI_MODEL_R64, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_R64, physToGpio_BPI_R64, pinTobcm_BPI_R64, R64_I2C_DEV, R64_SPI_DEV, {R64_PWM_OFFSET,R64_I2C_OFFSET,R64_SPI_OFFSET} },
  { "bananapi-r64", 13801, BPI_MODEL_R64, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_R64, physToGpio_BPI_R64, pinTobcm_BPI_R64, R64_I2C_DEV, R64_SPI_DEV, {R64_PWM_OFFSET,R64_I2C_OFFSET,R64_SPI_OFFSET} },
  { "banana-pi-r64", 13801, BPI_MODEL_R64, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_R64, physToGpio_BPI_R64, pinTobcm_BPI_R64, R64_I2C_DEV, R64_SPI_DEV, {R64_PWM_OFFSET,R64_I2C_OFFSET,R64_SPI_OFFSET} },
  { "bpi-r4lite",  13901, BPI_MODEL_R4LITE, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_R4LITE, physToGpio_BPI_R4LITE, pinTobcm_BPI_R4LITE, R4LITE_I2C_DEV, R4LITE_SPI_DEV, {R4LITE_PWM_OFFSET,R4LITE_I2C_OFFSET,R4LITE_SPI_OFFSET} },
  { "bpi-r4-lite", 13901, BPI_MODEL_R4LITE, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_R4LITE, physToGpio_BPI_R4LITE, pinTobcm_BPI_R4LITE, R4LITE_I2C_DEV, R4LITE_SPI_DEV, {R4LITE_PWM_OFFSET,R4LITE_I2C_OFFSET,R4LITE_SPI_OFFSET} },
  { "bananapir4lite", 13901, BPI_MODEL_R4LITE, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_R4LITE, physToGpio_BPI_R4LITE, pinTobcm_BPI_R4LITE, R4LITE_I2C_DEV, R4LITE_SPI_DEV, {R4LITE_PWM_OFFSET,R4LITE_I2C_OFFSET,R4LITE_SPI_OFFSET} },
  { "bananapi-r4lite", 13901, BPI_MODEL_R4LITE, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_R4LITE, physToGpio_BPI_R4LITE, pinTobcm_BPI_R4LITE, R4LITE_I2C_DEV, R4LITE_SPI_DEV, {R4LITE_PWM_OFFSET,R4LITE_I2C_OFFSET,R4LITE_SPI_OFFSET} },
  { "bananapi-r4-lite", 13901, BPI_MODEL_R4LITE, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_R4LITE, physToGpio_BPI_R4LITE, pinTobcm_BPI_R4LITE, R4LITE_I2C_DEV, R4LITE_SPI_DEV, {R4LITE_PWM_OFFSET,R4LITE_I2C_OFFSET,R4LITE_SPI_OFFSET} },
  { "banana-pi-r4-lite", 13901, BPI_MODEL_R4LITE, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_R4LITE, physToGpio_BPI_R4LITE, pinTobcm_BPI_R4LITE, R4LITE_I2C_DEV, R4LITE_SPI_DEV, {R4LITE_PWM_OFFSET,R4LITE_I2C_OFFSET,R4LITE_SPI_OFFSET} },
  { "bpi-r4pro",  14001, BPI_MODEL_R4PRO, 1, 4, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_R4PRO, physToGpio_BPI_R4PRO, pinTobcm_BPI_R4PRO, R4PRO_I2C_DEV, R4PRO_SPI_DEV, {R4PRO_PWM_OFFSET,R4PRO_I2C_OFFSET,R4PRO_SPI_OFFSET} },
  { "bpi-r4-pro", 14001, BPI_MODEL_R4PRO, 1, 4, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_R4PRO, physToGpio_BPI_R4PRO, pinTobcm_BPI_R4PRO, R4PRO_I2C_DEV, R4PRO_SPI_DEV, {R4PRO_PWM_OFFSET,R4PRO_I2C_OFFSET,R4PRO_SPI_OFFSET} },
  { "bpi-r4-pro-4e", 14001, BPI_MODEL_R4PRO, 1, 4, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_R4PRO, physToGpio_BPI_R4PRO, pinTobcm_BPI_R4PRO, R4PRO_I2C_DEV, R4PRO_SPI_DEV, {R4PRO_PWM_OFFSET,R4PRO_I2C_OFFSET,R4PRO_SPI_OFFSET} },
  { "bpi-r4-pro-8x", 14001, BPI_MODEL_R4PRO, 1, 4, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_R4PRO, physToGpio_BPI_R4PRO, pinTobcm_BPI_R4PRO, R4PRO_I2C_DEV, R4PRO_SPI_DEV, {R4PRO_PWM_OFFSET,R4PRO_I2C_OFFSET,R4PRO_SPI_OFFSET} },
  { "bananapir4pro", 14001, BPI_MODEL_R4PRO, 1, 4, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_R4PRO, physToGpio_BPI_R4PRO, pinTobcm_BPI_R4PRO, R4PRO_I2C_DEV, R4PRO_SPI_DEV, {R4PRO_PWM_OFFSET,R4PRO_I2C_OFFSET,R4PRO_SPI_OFFSET} },
  { "bananapi-r4pro", 14001, BPI_MODEL_R4PRO, 1, 4, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_R4PRO, physToGpio_BPI_R4PRO, pinTobcm_BPI_R4PRO, R4PRO_I2C_DEV, R4PRO_SPI_DEV, {R4PRO_PWM_OFFSET,R4PRO_I2C_OFFSET,R4PRO_SPI_OFFSET} },
  { "bananapi-r4-pro", 14001, BPI_MODEL_R4PRO, 1, 4, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_R4PRO, physToGpio_BPI_R4PRO, pinTobcm_BPI_R4PRO, R4PRO_I2C_DEV, R4PRO_SPI_DEV, {R4PRO_PWM_OFFSET,R4PRO_I2C_OFFSET,R4PRO_SPI_OFFSET} },
  { "banana-pi-r4-pro", 14001, BPI_MODEL_R4PRO, 1, 4, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_R4PRO, physToGpio_BPI_R4PRO, pinTobcm_BPI_R4PRO, R4PRO_I2C_DEV, R4PRO_SPI_DEV, {R4PRO_PWM_OFFSET,R4PRO_I2C_OFFSET,R4PRO_SPI_OFFSET} },
  { "bpi-r2",	   11101, BPI_MODEL_R2, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_R2, physToGpio_BPI_R2, pinTobcm_BPI_R2, R2_I2C_DEV, R2_SPI_DEV, {R2_PWM_OFFSET,R2_I2C_OFFSET,R2_SPI_OFFSET} },
  { NULL,		0, 0, 1, 2, 5, 0, NULL, NULL, NULL, NULL, NULL, {-1, -1, -1} },
} ;

extern int bpi_found;

static struct BPIBoards *bpi_find_board_by_name(const char *hardware)
{
  struct BPIBoards *board;

  for (board = bpiboard ; board->name != NULL ; ++board)
    if (strcmp(board->name, hardware) == 0)
      return board;

  return NULL;
}

static int bpi_model_is_rk3576(int model)
{
  return model == BPI_MODEL_M5PRO || model == BPI_MODEL_CM5PRO;
}

static int bpi_model_is_rk3528(int model)
{
  return model == BPI_MODEL_M1SUPER;
}

static int bpi_model_is_rk3506(int model)
{
  return model == BPI_MODEL_FORGE1;
}

static int bpi_model_is_rk3308(int model)
{
  return model == BPI_MODEL_P2PRO;
}

static int bpi_model_is_rk3588(int model)
{
  return model == BPI_MODEL_M7 ||
      model == BPI_MODEL_W3 ||
      model == BPI_MODEL_AIM7;
}

static int bpi_model_is_rockchip(int model)
{
  return model == BPI_MODEL_R2PRO ||
      model == BPI_MODEL_M4SUPER ||
      bpi_model_is_rk3308(model) ||
      bpi_model_is_rk3506(model) ||
      bpi_model_is_rk3528(model) ||
      bpi_model_is_rk3576(model) ||
      bpi_model_is_rk3588(model);
}

static int bpi_model_is_realtek(int model)
{
  return model == BPI_MODEL_W2 ||
      model == BPI_MODEL_M4;
}

static int bpi_model_is_vs680(int model)
{
  return model == BPI_MODEL_M6;
}

static int bpi_model_is_sp7021(int model)
{
  return model == BPI_MODEL_F2S ||
      model == BPI_MODEL_F2P;
}

static void bpi_select_realtek_backend(int model)
{
  if (model == BPI_MODEL_M4)
  {
    realtek_gpio_base = realtek_gpio_base_rtd139x;
    realtek_gpio_groups = realtek_rtd139x_groups;
    realtek_gpio_group_count = 1;
    realtek_gpio_map_size = REALTEK_GPIO_MAP_SIZE;
    return;
  }

  realtek_gpio_base = realtek_gpio_base_rtd129x;
  realtek_gpio_groups = realtek_rtd129x_groups;
  realtek_gpio_group_count = REALTEK_GPIO_GROUPS;
  realtek_gpio_map_size = REALTEK_GPIO_MAP_SIZE;
}

static void rockchip_select_gpio_v1_regs(void)
{
  rockchip_gpio_v2 = 0;
  rockchip_gpio_swport_dr = ROCKCHIP_GPIO_SWPORT_DR_V1;
  rockchip_gpio_swport_ddr = ROCKCHIP_GPIO_SWPORT_DDR_V1;
  rockchip_gpio_ext_port = ROCKCHIP_GPIO_EXT_PORT_V1;
}

static void rockchip_select_gpio_v2_regs(void)
{
  rockchip_gpio_v2 = 1;
  rockchip_gpio_swport_dr = ROCKCHIP_GPIO_SWPORT_DR_V2;
  rockchip_gpio_swport_ddr = ROCKCHIP_GPIO_SWPORT_DDR_V2;
  rockchip_gpio_ext_port = ROCKCHIP_GPIO_EXT_PORT_V2;
}

static void bpi_select_rockchip_backend(int model)
{
  rockchip_select_gpio_v2_regs();

  if (bpi_model_is_rk3308(model))
  {
    rockchip_gpio_base = rockchip_gpio_base_rk3308;
    rockchip_gpio_map_size = ROCKCHIP_GPIO_MAP_SIZE_RK3308;
    rockchip_select_gpio_v1_regs();
    return;
  }

  if (bpi_model_is_rk3506(model))
  {
    rockchip_gpio_base = rockchip_gpio_base_rk3506;
    rockchip_gpio_map_size = ROCKCHIP_GPIO_MAP_SIZE_RK3506;
    return;
  }

  if (bpi_model_is_rk3528(model))
  {
    rockchip_gpio_base = rockchip_gpio_base_rk3528;
    rockchip_gpio_map_size = ROCKCHIP_GPIO_MAP_SIZE_RK3528;
    return;
  }

  if (bpi_model_is_rk3576(model))
  {
    rockchip_gpio_base = rockchip_gpio_base_rk3576;
    rockchip_gpio_map_size = ROCKCHIP_GPIO_MAP_SIZE_RK3576;
    return;
  }

  if (bpi_model_is_rk3588(model))
  {
    rockchip_gpio_base = rockchip_gpio_base_rk3588;
    rockchip_gpio_map_size = ROCKCHIP_GPIO_MAP_SIZE_RK3588;
    return;
  }

  rockchip_gpio_base = rockchip_gpio_base_rk3568;
  rockchip_gpio_map_size = ROCKCHIP_GPIO_MAP_SIZE_RK3568;
}

static struct BPIBoards *bpi_find_board_by_model_string(const char *hardware)
{
  if (strstr(hardware, "BananaPi M4 Berry") ||
      strstr(hardware, "Banana Pi BPI-M4 Berry") ||
      strstr(hardware, "BPI-M4Berry"))
    return bpi_find_board_by_name("bpi-m4berry");

  if (strstr(hardware, "Banana Pi BPI-W2") ||
      strstr(hardware, "BananaPi BPI-W2") ||
      strstr(hardware, "Banana Pi W2") ||
      strstr(hardware, "BananaPi W2") ||
      strstr(hardware, "BPI-W2") ||
      strstr(hardware, "rtd-1296-bananapi-w2") ||
      strstr(hardware, "Realtek_RTD1296"))
    return bpi_find_board_by_name("bpi-w2");

  if (strstr(hardware, "BananaPi BPI-M4-Zero") ||
      strstr(hardware, "Banana Pi BPI-M4-Zero") ||
      strstr(hardware, "BananaPi M4 Zero") ||
      strstr(hardware, "BPI-M4Zero"))
    return bpi_find_board_by_name("bpi-m4zero");

  if (strstr(hardware, "BananaPi M2S") ||
      strstr(hardware, "BananaPi BPI-M2S") ||
      strstr(hardware, "Banana Pi BPI-M2S") ||
      strstr(hardware, "Banana Pi M2S") ||
      strstr(hardware, "BPI-M2S"))
    return bpi_find_board_by_name("bpi-m2s");

  if (strstr(hardware, "Bananapi BPI-CM4") ||
      strstr(hardware, "BananaPi BPI-CM4") ||
      strstr(hardware, "Banana Pi BPI-CM4") ||
      strstr(hardware, "BananaPi BPI-CM4IO") ||
      strstr(hardware, "Banana Pi BPI-CM4IO") ||
      strstr(hardware, "BPI-CM4IO") ||
      strstr(hardware, "BPI-CM4"))
    return bpi_find_board_by_name("bpi-cm4io");

  if (strstr(hardware, "Banana Pi BPI-CM5 Pro") ||
      strstr(hardware, "BananaPi BPI-CM5 Pro") ||
      strstr(hardware, "Banana Pi CM5 Pro") ||
      strstr(hardware, "BananaPi CM5 Pro") ||
      strstr(hardware, "BPI-CM5 Pro") ||
      strstr(hardware, "ArmSoM CM5 IO") ||
      strstr(hardware, "armsom,cm5-io") ||
      strstr(hardware, "rk3576-armsom-cm5-io"))
    return bpi_find_board_by_name("bpi-cm5-pro");

  if (strstr(hardware, "Banana Pi BPI-M5 Pro") ||
      strstr(hardware, "BananaPi BPI-M5 Pro") ||
      strstr(hardware, "Banana Pi M5 Pro") ||
      strstr(hardware, "BananaPi M5 Pro") ||
      strstr(hardware, "BPI-M5 Pro") ||
      strstr(hardware, "rk3576-bananapi-m5-pro"))
    return bpi_find_board_by_name("bpi-m5-pro");

  if (strstr(hardware, "Banana Pi BPI-M7") ||
      strstr(hardware, "BananaPi BPI-M7") ||
      strstr(hardware, "Banana Pi M7") ||
      strstr(hardware, "BananaPi M7") ||
      strstr(hardware, "BPI-M7") ||
      strstr(hardware, "bananapi,m7") ||
      strstr(hardware, "rk3588-bananapi-m7"))
    return bpi_find_board_by_name("bpi-m7");

  if (strstr(hardware, "Banana Pi BPI-W3") ||
      strstr(hardware, "BananaPi BPI-W3") ||
      strstr(hardware, "Banana Pi W3") ||
      strstr(hardware, "BananaPi W3") ||
      strstr(hardware, "BPI-W3") ||
      strstr(hardware, "ArmSoM W3") ||
      strstr(hardware, "armsom w3") ||
      strstr(hardware, "bananapi,bpi-w3") ||
      strstr(hardware, "armsom,w3") ||
      strstr(hardware, "rk3588-bananapi-w3") ||
      strstr(hardware, "rk3588-armsom-w3"))
    return bpi_find_board_by_name("bpi-w3");

  if (strstr(hardware, "Banana Pi BPI-AIM7") ||
      strstr(hardware, "BananaPi BPI-AIM7") ||
      strstr(hardware, "Banana Pi AIM7") ||
      strstr(hardware, "BananaPi AIM7") ||
      strstr(hardware, "BPI-AIM7") ||
      strstr(hardware, "ArmSoM AIM7 IO") ||
      strstr(hardware, "ArmSoM AIM7") ||
      strstr(hardware, "armsom,aim7-io") ||
      strstr(hardware, "armsom,aim7") ||
      strstr(hardware, "rk3588-armsom-aim7-io"))
    return bpi_find_board_by_name("bpi-aim7");

  if (strstr(hardware, "Banana Pi BPI-M4 Super") ||
      strstr(hardware, "BananaPi BPI-M4 Super") ||
      strstr(hardware, "Banana Pi M4 Super") ||
      strstr(hardware, "BananaPi M4 Super") ||
      strstr(hardware, "BPI-M4 Super") ||
      strstr(hardware, "ArmSom Sige3") ||
      strstr(hardware, "ArmSoM Sige3") ||
      strstr(hardware, "armsom,sige3") ||
      strstr(hardware, "rk3568-armsom-sige3"))
    return bpi_find_board_by_name("bpi-m4-super");

  if (strstr(hardware, "Sinovoip_Bananapi_M4") ||
      strstr(hardware, "Banana Pi BPI-M4") ||
      strstr(hardware, "BananaPi BPI-M4") ||
      strstr(hardware, "Banana Pi M4") ||
      strstr(hardware, "BananaPi M4") ||
      strstr(hardware, "BPI-M4") ||
      strstr(hardware, "rtd-1395-bananapi-m4"))
    return bpi_find_board_by_name("bpi-m4");

  if (strstr(hardware, "Banana Pi BPI-M6") ||
      strstr(hardware, "BananaPi BPI-M6") ||
      strstr(hardware, "Banana Pi M6") ||
      strstr(hardware, "BananaPi M6") ||
      strstr(hardware, "BPI-M6") ||
      strstr(hardware, "Synaptics VS680 EVK") ||
      strstr(hardware, "vs680-a0-bananapi-m6"))
    return bpi_find_board_by_name("bpi-m6");

  if (strstr(hardware, "Banana Pi BPI-F2S") ||
      strstr(hardware, "BananaPi BPI-F2S") ||
      strstr(hardware, "Banana Pi F2S") ||
      strstr(hardware, "BananaPi F2S") ||
      strstr(hardware, "BPI-F2S") ||
      strstr(hardware, "SP7021/CA7/BPI-F2S") ||
      strstr(hardware, "sp7021-bpi-f2s"))
    return bpi_find_board_by_name("bpi-f2s");

  if (strstr(hardware, "Banana Pi BPI-F2P") ||
      strstr(hardware, "BananaPi BPI-F2P") ||
      strstr(hardware, "Banana Pi F2P") ||
      strstr(hardware, "BananaPi F2P") ||
      strstr(hardware, "BPI-F2P") ||
      strstr(hardware, "SP7021/CA7/BPI-F2P") ||
      strstr(hardware, "sp7021-bpi-f2p"))
    return bpi_find_board_by_name("bpi-f2p");

  if (strstr(hardware, "Banana Pi BPI-F4") ||
      strstr(hardware, "BananaPi BPI-F4") ||
      strstr(hardware, "Banana Pi F4") ||
      strstr(hardware, "BananaPi F4") ||
      strstr(hardware, "BPI-F4") ||
      strstr(hardware, "bananapi,bpi-f4") ||
      strstr(hardware, "sp7350-bpi-f4"))
    return bpi_find_board_by_name("bpi-f4");

  if (strstr(hardware, "Banana Pi BPI-M1 Super") ||
      strstr(hardware, "BananaPi BPI-M1 Super") ||
      strstr(hardware, "Banana Pi M1 Super") ||
      strstr(hardware, "BananaPi M1 Super") ||
      strstr(hardware, "BPI-M1 Super") ||
      strstr(hardware, "Banana Pi BPI-M1S") ||
      strstr(hardware, "BananaPi BPI-M1S") ||
      strstr(hardware, "Banana Pi M1S") ||
      strstr(hardware, "BananaPi M1S") ||
      strstr(hardware, "BPI-M1S") ||
      strstr(hardware, "ArmSom Sige1") ||
      strstr(hardware, "ArmSoM Sige1") ||
      strstr(hardware, "armsom,sige1") ||
      strstr(hardware, "rk3528-armsom-sige1"))
    return bpi_find_board_by_name("bpi-m1-super");

  if (strstr(hardware, "Banana Pi BPI-Forge1") ||
      strstr(hardware, "BananaPi BPI-Forge1") ||
      strstr(hardware, "Banana Pi Forge1") ||
      strstr(hardware, "BananaPi Forge1") ||
      strstr(hardware, "BPI-Forge1") ||
      strstr(hardware, "ArmSom Forge1") ||
      strstr(hardware, "ArmSoM Forge1") ||
      strstr(hardware, "armsom,forge1") ||
      strstr(hardware, "rockchip,rk3506J-armsom-forge1") ||
      strstr(hardware, "rockchip,rk3506j-armsom-forge1") ||
      strstr(hardware, "rk3506b-armsom-forge1"))
    return bpi_find_board_by_name("bpi-forge1");

  if (strstr(hardware, "Banana Pi BPI-P2 Pro") ||
      strstr(hardware, "BananaPi BPI-P2 Pro") ||
      strstr(hardware, "Banana Pi P2 Pro") ||
      strstr(hardware, "BananaPi P2 Pro") ||
      strstr(hardware, "BPI-P2 Pro") ||
      strstr(hardware, "ArmSom P2 Pro") ||
      strstr(hardware, "ArmSoM P2 Pro") ||
      strstr(hardware, "armsom,p2pro") ||
      strstr(hardware, "sinovoip,rk3308-bpi-p2pro") ||
      strstr(hardware, "rk3308-bpi-p2-pro"))
    return bpi_find_board_by_name("bpi-p2-pro");

  if (strstr(hardware, "Banana Pi BPI-M5") ||
      strstr(hardware, "BananaPi BPI-M5") ||
      strstr(hardware, "Banana Pi M5") ||
      strstr(hardware, "BananaPi M5") ||
      strstr(hardware, "BPI-M5"))
    return bpi_find_board_by_name("bpi-m5");

  if (strstr(hardware, "Banana Pi BPI-M2-PRO") ||
      strstr(hardware, "Banana Pi BPI-M2 Pro") ||
      strstr(hardware, "BananaPi BPI-M2-PRO") ||
      strstr(hardware, "BananaPi BPI-M2 Pro") ||
      strstr(hardware, "Banana Pi M2Pro") ||
      strstr(hardware, "Banana Pi M2 Pro") ||
      strstr(hardware, "BananaPi M2Pro") ||
      strstr(hardware, "BPI-M2-PRO") ||
      strstr(hardware, "BPI-M2-Pro") ||
      strstr(hardware, "BPI-M2 Pro"))
    return bpi_find_board_by_name("bpi-m2pro");

  if (strstr(hardware, "BananaPi BPI-CM6") ||
      strstr(hardware, "Banana Pi BPI-CM6") ||
      strstr(hardware, "BananaPi CM6") ||
      strstr(hardware, "Banana Pi CM6") ||
      strstr(hardware, "BPI-CM6"))
    return bpi_find_board_by_name("bpi-cm6");

  if (strstr(hardware, "BananaPi BPI-F3") ||
      strstr(hardware, "Banana Pi BPI-F3") ||
      strstr(hardware, "BananaPi F3") ||
      strstr(hardware, "Banana Pi F3") ||
      strstr(hardware, "BPI-F3") ||
      strstr(hardware, "k1-x deb1"))
    return bpi_find_board_by_name("bpi-f3");

  if (strstr(hardware, "BananaPi BPI-AI2N") ||
      strstr(hardware, "Banana Pi BPI-AI2N") ||
      strstr(hardware, "BananaPi AI2N") ||
      strstr(hardware, "Banana Pi AI2N") ||
      strstr(hardware, "BPI-AI2N"))
    return bpi_find_board_by_name("bpi-ai2n");

  if (strstr(hardware, "Bananapi BPI-R4 Pro") ||
      strstr(hardware, "BananaPi BPI-R4 Pro") ||
      strstr(hardware, "Banana Pi BPI-R4 Pro") ||
      strstr(hardware, "BananaPi R4 Pro") ||
      strstr(hardware, "Banana Pi R4 Pro") ||
      strstr(hardware, "BPI-R4-Pro") ||
      strstr(hardware, "BPI-R4 Pro") ||
      strstr(hardware, "BPI-R4_PRO") ||
      strstr(hardware, "bananapi,bpi-r4-pro") ||
      strstr(hardware, "bananapi,bpi-r4-pro-4e") ||
      strstr(hardware, "bananapi,bpi-r4-pro-8x") ||
      strstr(hardware, "mt7988a-bananapi-bpi-r4-pro") ||
      strstr(hardware, "mt7988a-bananapi_bpi-r4-pro"))
    return bpi_find_board_by_name("bpi-r4-pro");

  if (strstr(hardware, "Bananapi BPI-R4-LITE") ||
      strstr(hardware, "Bananapi BPI-R4 Lite") ||
      strstr(hardware, "BananaPi BPI-R4-LITE") ||
      strstr(hardware, "BananaPi BPI-R4 Lite") ||
      strstr(hardware, "Banana Pi BPI-R4 Lite") ||
      strstr(hardware, "BananaPi R4 Lite") ||
      strstr(hardware, "Banana Pi R4 Lite") ||
      strstr(hardware, "BPI-R4-LITE") ||
      strstr(hardware, "BPI-R4 Lite") ||
      strstr(hardware, "BPI-R4_Lite") ||
      strstr(hardware, "bananapi,bpi-r4-lite") ||
      strstr(hardware, "mt7987a-bananapi-bpi-r4-lite"))
    return bpi_find_board_by_name("bpi-r4-lite");

  if (strstr(hardware, "Bananapi BPI-R4") ||
      strstr(hardware, "BananaPi BPI-R4") ||
      strstr(hardware, "Banana Pi BPI-R4") ||
      strstr(hardware, "BananaPi R4") ||
      strstr(hardware, "Banana Pi R4") ||
      strstr(hardware, "BPI-R4") ||
      strstr(hardware, "mt7988a-bananapi-bpi-r4"))
    return bpi_find_board_by_name("bpi-r4");

  if (strstr(hardware, "BananaPi BPI-R3 Mini") ||
      strstr(hardware, "Banana Pi BPI-R3 Mini") ||
      strstr(hardware, "BananaPi R3 Mini") ||
      strstr(hardware, "Banana Pi R3 Mini") ||
      strstr(hardware, "BPI-R3 Mini") ||
      strstr(hardware, "mt7986a-bananapi-bpi-r3-mini"))
    return NULL;

  if (strstr(hardware, "Bananapi BPI-R3") ||
      strstr(hardware, "BananaPi BPI-R3") ||
      strstr(hardware, "Banana Pi BPI-R3") ||
      strstr(hardware, "BananaPi R3") ||
      strstr(hardware, "Banana Pi R3") ||
      strstr(hardware, "BPI-R3") ||
      strstr(hardware, "mt7986a-bananapi-bpi-r3"))
    return bpi_find_board_by_name("bpi-r3");

  if (strstr(hardware, "Bananapi BPI-R64") ||
      strstr(hardware, "BananaPi BPI-R64") ||
      strstr(hardware, "Banana Pi BPI-R64") ||
      strstr(hardware, "BananaPi R64") ||
      strstr(hardware, "Banana Pi R64") ||
      strstr(hardware, "BPI-R64") ||
      strstr(hardware, "bananapi,bpi-r64") ||
      strstr(hardware, "mt7622-bananapi-bpi-r64"))
    return bpi_find_board_by_name("bpi-r64");

  if (strstr(hardware, "Bananapi-R2 Pro") ||
      strstr(hardware, "BananaPi BPI-R2 Pro") ||
      strstr(hardware, "Banana Pi BPI-R2 Pro") ||
      strstr(hardware, "BananaPi R2 Pro") ||
      strstr(hardware, "Banana Pi R2 Pro") ||
      strstr(hardware, "BPI-R2 Pro") ||
      strstr(hardware, "rk3568-bpi-r2pro"))
    return bpi_find_board_by_name("bpi-r2-pro");

  return NULL;
}

static int bpi_set_layout_from_board(struct BPIBoards *board, int *gpioLayout)
{
  if (board == NULL)
    return 0;

  *gpioLayout = board->model;
  if (*gpioLayout >= BPI_MODEL_MIN) {
    bpi_found = 1;
    return 1;
  }

  return 0;
}

static int bpi_read_dt_string_file(const char *path, char *hardware, size_t hardware_size)
{
  FILE *bpiFd;
  size_t count;
  size_t i;

  if (hardware_size == 0)
    return 0;

  bpiFd = fopen(path, "r");
  if (bpiFd == NULL)
    return 0;

  count = fread(hardware, 1, hardware_size - 1, bpiFd);
  fclose(bpiFd);
  if (count == 0)
    return 0;

  hardware[count] = '\0';
  for (i = 0; i < count; ++i) {
    if (hardware[i] == '\0' || hardware[i] == '\n')
      hardware[i] = ' ';
  }

  return 1;
}

int bpi_piGpioLayout (void)
{
  FILE *bpiFd ;
  char buffer[1024];
  char hardware[1024];
  struct BPIBoards *board;
  static int  gpioLayout = -1 ;

  if (gpioLayout != -1)	// No point checking twice
    return gpioLayout ;

  bpi_found = 0; // -1: not init, 0: init but not found, 1: found
  bpi_found_mtk = 0;
  bpi_found_mtk_v2 = 0;
  bpi_found_mtk_mt7622 = 0;
  bpi_found_sun50iw9 = 0;
  bpi_found_meson = 0;
  bpi_found_spacemit = 0;
  bpi_found_renesas = 0;
  bpi_found_rockchip = 0;
  bpi_found_realtek = 0;
  bpi_found_vs680 = 0;
  bpi_found_sp7021 = 0;
  bpi_found_sp7350 = 0;
  if ((bpiFd = fopen("/var/lib/bananapi/board.sh", "r")) != NULL) {
    while(fgets(buffer, sizeof(buffer), bpiFd) != NULL) {
      if (sscanf(buffer, "BOARD=%1023s", hardware) != 1)
        continue;

      board = bpi_find_board_by_name(hardware);
      if (bpi_set_layout_from_board(board, &gpioLayout))
        break;
    }
    fclose(bpiFd);
  }

  if (bpi_found != 1 && bpi_read_dt_string_file("/proc/device-tree/compatible", hardware, sizeof(hardware))) {
    board = bpi_find_board_by_model_string(hardware);
    bpi_set_layout_from_board(board, &gpioLayout);
  }

  if (bpi_found != 1 && bpi_read_dt_string_file("/proc/device-tree/model", hardware, sizeof(hardware))) {
    board = bpi_find_board_by_model_string(hardware);
    bpi_set_layout_from_board(board, &gpioLayout);
  }

  //printf("BPI: name[%s] gpioLayout(%d)\n",board->name, gpioLayout);
  return gpioLayout ;
}

void bpi_piBoardId (int *model, int *rev, int *mem, int *maker, int *warranty)
{
  int bRev, bType, bMfg, bMem, bWarranty ;
  struct BPIBoards *board=bpiboard;
  static int  gpioLayout = -1 ;

  gpioLayout = piGpioLayout () ;
  //printf("BPI: gpioLayout(%d)\n", gpioLayout);
  if(gpioLayout >= BPI_MODEL_MIN) {
    for (board = bpiboard ; board->name != NULL ; ++board) {
      if (board->model == gpioLayout)
        break;
    }
    if (board->name == NULL)
      return;
    //printf("BPI: name[%s] gpioLayout(%d)\n",board->name, gpioLayout);
    bRev      = board->rev;
    bType     = board->model;
    bMfg      = board->maker;
    bMem      = board->mem;
    bWarranty = board->warranty;
    pinToGpio =  board->pinToGpio ;
    physToGpio = board->physToGpio ;
    pinToGpio_BP =  board->pinToGpio ;
    physToGpio_BP = board->physToGpio ;
    pinTobcm_BP = board->pinTobcm ;
    bpi_found_mtk = (board->model == BPI_MODEL_R2);
    bpi_found_mtk_v2 = (board->model == BPI_MODEL_R4 ||
                         board->model == BPI_MODEL_R3 ||
                         board->model == BPI_MODEL_R4LITE ||
                         board->model == BPI_MODEL_R4PRO);
    bpi_found_mtk_mt7622 = (board->model == BPI_MODEL_R64);
    bpi_found_sun50iw9 = (board->model == BPI_MODEL_M4BERRY || board->model == BPI_MODEL_M4ZERO);
    bpi_found_meson = (board->model == BPI_MODEL_M2S ||
                       board->model == BPI_MODEL_CM4IO ||
                       board->model == BPI_MODEL_M5 ||
                       board->model == BPI_MODEL_M2PRO);
    bpi_found_spacemit = (board->model == BPI_MODEL_F3 ||
                           board->model == BPI_MODEL_CM6);
    bpi_found_renesas = (board->model == BPI_MODEL_AI2N);
    bpi_found_rockchip = bpi_model_is_rockchip(board->model);
    if (bpi_found_rockchip)
      bpi_select_rockchip_backend(board->model);
    bpi_found_realtek = bpi_model_is_realtek(board->model);
    if (bpi_found_realtek)
      bpi_select_realtek_backend(board->model);
    bpi_found_vs680 = bpi_model_is_vs680(board->model);
    bpi_found_sp7021 = bpi_model_is_sp7021(board->model);
    bpi_found_sp7350 = (board->model == BPI_MODEL_F4);
    //printf("BPI: name[%s] bType(%d) model(%d)\n",board->name, bType, board->model);
    *model    = bType ;
    *rev      = bRev ;
    *mem      = bMem ;
    *maker    = bMfg  ;
    *warranty = bWarranty ;
    return; 
  }
}

int bpi_wiringPiSetup (void)
{
  int   fd ;
  int   model, rev, mem, maker, overVolted ;
  static int alreadyDoneThis = FALSE ;

  if (alreadyDoneThis)
    return 0 ;

  alreadyDoneThis = TRUE ;

  if (geteuid () != 0)
    (void)wiringPiFailure (WPI_FATAL, "wiringPiSetup: Must be root. (Did you forget sudo?)\n") ;

  if (wiringPiDebug)
    printf ("wiringPi: wiringPiSetup called\n") ;

  piBoardId (&model, &rev, &mem, &maker, &overVolted) ;

  // Open the master /dev/memory device
  if ((fd = open ("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC) ) < 0)
    return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: Unable to open /dev/mem: %s\n", strerror (errno)) ;

  if (bpi_found_mtk)
  {
    mtk_gpio_base = (uint8_t *)mmap(0, MTK_GPIO_MAP_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, MTK_GPIO_BASE_BP);
    close(fd);
    if (mtk_gpio_base == MAP_FAILED)
    {
      mtk_gpio_base = NULL;
      return wiringPiFailure (WPI_ALMOST,"wiringPiSetup: mmap (MTK GPIO) failed: %s\n", strerror (errno)) ;
    }
    initialiseEpoch () ;
    return 0 ;
  }

  if (bpi_found_mtk_v2)
  {
    mtk_v2_gpio_base = (uint8_t *)mmap(0, MTK_V2_GPIO_MAP_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, MTK_V2_GPIO_BASE_BP);
    close(fd);
    if (mtk_v2_gpio_base == MAP_FAILED)
    {
      mtk_v2_gpio_base = NULL;
      return wiringPiFailure (WPI_ALMOST,"wiringPiSetup: mmap (MTK GPIO v2) failed: %s\n", strerror (errno)) ;
    }
    initialiseEpoch () ;
    return 0 ;
  }

  if (bpi_found_mtk_mt7622)
  {
    mtk_mt7622_gpio_base = (uint8_t *)mmap(0, MTK_MT7622_GPIO_MAP_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, MTK_MT7622_GPIO_BASE_BP);
    close(fd);
    if (mtk_mt7622_gpio_base == MAP_FAILED)
    {
      mtk_mt7622_gpio_base = NULL;
      return wiringPiFailure (WPI_ALMOST,"wiringPiSetup: mmap (MTK MT7622 GPIO) failed: %s\n", strerror (errno)) ;
    }
    initialiseEpoch () ;
    return 0 ;
  }

  if (bpi_found_meson)
  {
    meson_gpio = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, MESON_GPIO_BASE_BP);
    meson_gpioao = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, MESON_GPIO_AO_BASE_BP);
    close(fd);
    if (meson_gpio == MAP_FAILED || meson_gpioao == MAP_FAILED)
    {
      meson_gpio = NULL;
      meson_gpioao = NULL;
      return wiringPiFailure (WPI_ALMOST,"wiringPiSetup: mmap (MESON GPIO) failed: %s\n", strerror (errno)) ;
    }

    initialiseEpoch () ;
    return 0 ;
  }

  if (bpi_found_spacemit)
  {
    spacemit_gpio = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, SPACEMIT_GPIO_BASE_BP);
    spacemit_pinctrl = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, SPACEMIT_PINCTRL_BASE_BP);
    close(fd);
    if (spacemit_gpio == MAP_FAILED || spacemit_pinctrl == MAP_FAILED)
    {
      spacemit_gpio = NULL;
      spacemit_pinctrl = NULL;
      return wiringPiFailure (WPI_ALMOST,"wiringPiSetup: mmap (SPACEMIT GPIO) failed: %s\n", strerror (errno)) ;
    }

    initialiseEpoch () ;
    return 0 ;
  }

  if (bpi_found_renesas)
  {
    renesas_gpio = (uint32_t *)mmap(0, RENESAS_GPIO_MAP_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, RENESAS_GPIO_BASE_BP);
    close(fd);
    if (renesas_gpio == MAP_FAILED)
    {
      renesas_gpio = NULL;
      return wiringPiFailure (WPI_ALMOST,"wiringPiSetup: mmap (RENESAS GPIO) failed: %s\n", strerror (errno)) ;
    }

    initialiseEpoch () ;
    return 0 ;
  }

  if (bpi_found_rockchip)
  {
    int i;

    for (i = 0; i < ROCKCHIP_GPIO_BANKS; ++i)
    {
      rockchip_gpio[i] = (uint32_t *)mmap(0, rockchip_gpio_map_size,
          PROT_READ|PROT_WRITE, MAP_SHARED, fd, rockchip_gpio_base[i]);
      if (rockchip_gpio[i] == MAP_FAILED)
      {
        int j;

        rockchip_gpio[i] = NULL;
        for (j = 0; j < i; ++j)
        {
          if (rockchip_gpio[j] != NULL)
          {
            munmap((void *)rockchip_gpio[j], rockchip_gpio_map_size);
            rockchip_gpio[j] = NULL;
          }
        }
        close(fd);
        return wiringPiFailure (WPI_ALMOST,"wiringPiSetup: mmap (ROCKCHIP GPIO) failed: %s\n", strerror (errno)) ;
      }
    }
    close(fd);
    initialiseEpoch () ;
    return 0 ;
  }

  if (bpi_found_realtek)
  {
    int i;

    for (i = 0; i < realtek_gpio_group_count; ++i)
    {
      realtek_gpio[i] = (uint32_t *)mmap(0, realtek_gpio_map_size,
          PROT_READ|PROT_WRITE, MAP_SHARED, fd, realtek_gpio_base[i]);
      if (realtek_gpio[i] == MAP_FAILED)
      {
        int j;

        realtek_gpio[i] = NULL;
        for (j = 0; j < i; ++j)
        {
          if (realtek_gpio[j] != NULL)
          {
            munmap((void *)realtek_gpio[j], realtek_gpio_map_size);
            realtek_gpio[j] = NULL;
          }
        }
        close(fd);
        return wiringPiFailure (WPI_ALMOST,"wiringPiSetup: mmap (REALTEK GPIO) failed: %s\n", strerror (errno)) ;
      }
    }
    close(fd);
    initialiseEpoch () ;
    return 0 ;
  }

  if (bpi_found_vs680)
  {
    int i;

    for (i = 0; i < VS680_GPIO_BANKS; ++i)
    {
      vs680_gpio[i] = (uint32_t *)mmap(0, VS680_GPIO_MAP_SIZE,
          PROT_READ|PROT_WRITE, MAP_SHARED, fd, vs680_gpio_base[i]);
      if (vs680_gpio[i] == MAP_FAILED)
      {
        int j;

        vs680_gpio[i] = NULL;
        for (j = 0; j < i; ++j)
        {
          if (vs680_gpio[j] != NULL)
          {
            munmap((void *)vs680_gpio[j], VS680_GPIO_MAP_SIZE);
            vs680_gpio[j] = NULL;
          }
        }
        close(fd);
        return wiringPiFailure (WPI_ALMOST,"wiringPiSetup: mmap (VS680 GPIO) failed: %s\n", strerror (errno)) ;
      }
    }
    close(fd);
    initialiseEpoch () ;
    return 0 ;
  }

  if (bpi_found_sp7021)
  {
    sp7021_gpio_page0 = (uint32_t *)mmap(0, SP7021_GPIO_MAP_SIZE,
        PROT_READ|PROT_WRITE, MAP_SHARED, fd, SP7021_GPIO_PAGE0_BASE);
    if (sp7021_gpio_page0 == MAP_FAILED)
    {
      sp7021_gpio_page0 = NULL;
      close(fd);
      return wiringPiFailure (WPI_ALMOST,"wiringPiSetup: mmap (SP7021 GPIO page0) failed: %s\n", strerror (errno)) ;
    }

    sp7021_gpio_page2 = (uint32_t *)mmap(0, SP7021_GPIO_MAP_SIZE,
        PROT_READ|PROT_WRITE, MAP_SHARED, fd, SP7021_GPIO_PAGE2_BASE);
    if (sp7021_gpio_page2 == MAP_FAILED)
    {
      sp7021_gpio_page2 = NULL;
      munmap((void *)sp7021_gpio_page0, SP7021_GPIO_MAP_SIZE);
      sp7021_gpio_page0 = NULL;
      close(fd);
      return wiringPiFailure (WPI_ALMOST,"wiringPiSetup: mmap (SP7021 GPIO page2) failed: %s\n", strerror (errno)) ;
    }

    sp7021_gpio_base0 = sp7021_gpio_page0 + (SP7021_GPIO_BASE0_OFFSET >> 2);
    sp7021_gpio_base1 = sp7021_gpio_page0 + (SP7021_GPIO_BASE1_OFFSET >> 2);
    sp7021_gpio_base2 = sp7021_gpio_page2 + (SP7021_GPIO_BASE2_OFFSET >> 2);
    close(fd);
    initialiseEpoch () ;
    return 0 ;
  }

  if (bpi_found_sp7350)
  {
    sp7350_gpio_page = (uint32_t *)mmap(0, SP7350_GPIO_MAP_SIZE,
        PROT_READ|PROT_WRITE, MAP_SHARED, fd, SP7350_GPIO_PAGE_BASE);
    if (sp7350_gpio_page == MAP_FAILED)
    {
      sp7350_gpio_page = NULL;
      close(fd);
      return wiringPiFailure (WPI_ALMOST,"wiringPiSetup: mmap (SP7350 GPIO) failed: %s\n", strerror (errno)) ;
    }

    sp7350_gpio_first = sp7350_gpio_page + (SP7350_GPIO_FIRST_OFFSET >> 2);
    sp7350_gpio_gpioxt = sp7350_gpio_page + (SP7350_GPIO_GPIOXT_OFFSET >> 2);
    close(fd);
    initialiseEpoch () ;
    return 0 ;
  }

  if (bpi_found_sun50iw9)
  {
    gpio_lm = NULL;
    gpio = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, SUN50IW9_GPIO_BASE);
    close(fd);
    if ((int32_t)gpio == -1)
      return wiringPiFailure (WPI_ALMOST,"wiringPiSetup: mmap (SUN50IW9 GPIO) failed: %s\n", strerror (errno)) ;

    initialiseEpoch () ;
    return 0 ;
  }

  gpio_lm = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, GPIO_BASE_LM_BP);

  gpio = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, GPIO_BASE_BP);
  if (((int32_t)gpio == -1) || ((int32_t)gpio == -1 ))
    return wiringPiFailure (WPI_ALMOST,"wiringPiSetup: mmap (GPIO) failed: %s\n", strerror (errno)) ;

  // PWM
  pwm = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, GPIO_PWM_BP) ;
  if ((int32_t)pwm == -1)
    return wiringPiFailure (WPI_ALMOST,"wiringPiSetup: mmap (PWM) failed: %s\n", strerror (errno)) ;
			 
  // Clock control (needed for PWM)
  clk = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, CLOCK_BASE_BP) ;
  if ((int32_t)clk == -1)
    return wiringPiFailure (WPI_ALMOST,"wiringPiSetup: mmap (CLOCK) failed: %s\n", strerror (errno)) ;
			 
  // The drive pads
  pads = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, GPIO_PADS_BP) ;
  if ((int32_t)pads == -1)
    return wiringPiFailure (WPI_ALMOST,"wiringPiSetup: mmap (PADS) failed: %s\n", strerror (errno)) ;

#ifdef	USE_TIMER
// The system timer
  timer = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, GPIO_TIMER_BP) ;
  if ((int32_t)timer == -1)
    return wiringPiFailure (WPI_ALMOST,"wiringPiSetup: mmap (TIMER) failed: %s\n", strerror (errno)) ;

// Set the timer to free-running, 1MHz.
//	0xF9 is 249, the timer divide is base clock / (divide+1)
//	so base clock is 250MHz / 250 = 1MHz.

  *(timer + TIMER_CONTROL) = 0x0000280 ;
  *(timer + TIMER_PRE_DIV) = 0x00000F9 ;
  timerIrqRaw = timer + TIMER_IRQ_RAW ;
#endif
  
  initialiseEpoch () ;

  return 0 ;
}

int bpi_wiringPiSetupI2C (int board_model, const char **device)
{
  struct BPIBoards *board;

  for (board = bpiboard ; board->name != NULL ; ++board) {
    if (board->model == board_model) {
      *device = board->i2c_dev; 
      return 0;
    }
  }

  return -1;
}

int bpi_wiringPiSetupSPI (int board_model, const char **device)
{
	struct BPIBoards *board;

	for (board = bpiboard ; board->name != NULL ; ++board) {
      if (board->model == board_model) {
        *device = board->spi_dev; 
	return 0;
      }
    }

	return -1;
}

static int bpi_wiringPiSetupRegOffset(int mode)
{
  struct BPIBoards *board;
  int board_model;
	
  board_model = piGpioLayout () ;
  for (board = bpiboard ; board->name != NULL ; ++board) {
    if (board->model == board_model) {
	  if(mode == PWM_OUTPUT)
	   return board->reg_offset.pwm_offset;
	  else if(mode == I2C_PIN)
	  	return board->reg_offset.i2c_offset;
	  else if(mode == SPI_PIN)
	  	return board->reg_offset.spi_offset;
    }
  }

  return -1;
}
#endif /* BPI */
