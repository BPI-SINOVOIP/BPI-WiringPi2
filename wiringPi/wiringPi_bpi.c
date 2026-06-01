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
#define MAP_SIZE	          (4096*2)
#define MAP_MASK	          (MAP_SIZE - 1)

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

static int wiringPinMode = WPI_MODE_UNINITIALISED ;


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



static int BP_PIN_MASK[14][32] =  //[BANK]  [INDEX]
{
  { 0, 1, 2, 3,-1,-1, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PA
  {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PB
  { 0, 1, 2, 3, 4,-1,-1, 7,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PC
  {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PD
  {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PE
  {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PF
  {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PG
  {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PH
  {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PI
  {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PJ
  {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PK
  {-1,-1, 2,-1, 4,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PL
  {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PM
  {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PN
};
//static int version=0;
static int pwmmode=0;

static int bpi_wiringPiSetupRegOffset(int mode);

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
  if(bank == 11)
      val = *(gpio_lm+ mmap_seek);
  else
      val = *(gpio + mmap_seek);

  return val;
}

void sunxi_gpio_writel(uint32_t val, uint32_t addr, int bank)
{
  uint32_t mmap_base = (addr & ~MAP_MASK);
  uint32_t mmap_seek = ((addr - mmap_base) >> 2);

  if(bank == 11)
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

  /* for M2 PM and PL */
  if(bank == 11)
    phyaddr = SUNXI_GPIO_LM_BASE + ((bank - 11) * 36) + ((index >> 3) << 2);
  else
  	phyaddr = SUNXI_GPIO_BASE + (bank * 36) + ((index >> 3) << 2);

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

  /* for M2 PM and PL */
  if(bank == 11)
    phyaddr = SUNXI_GPIO_LM_BASE + ((bank - 11) * 36) + ((index >> 3) << 2);
  else
    phyaddr = SUNXI_GPIO_BASE + (bank * 36) + ((index >> 3) << 2);

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

  /* for M2 PM and PL */
  if(bank == 11)
    phyaddr = SUNXI_GPIO_LM_BASE + ((bank - 11) * 36) + 0x10;
  else
     phyaddr = SUNXI_GPIO_BASE + (bank * 36) + 0x10;

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

  /* for M2 PM and PL */
  if(bank == 11)
    phyaddr = SUNXI_GPIO_LM_BASE + ((bank - 11) * 36) + 0x10;
  else
 	phyaddr = SUNXI_GPIO_BASE + (bank * 36) + 0x10;

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

  /* for M2 PM and PL */
  if(bank == 11)
    phyaddr = SUNXI_GPIO_LM_BASE + ((bank -11) * 36) + 0x1c + sub*4;
  else
 	phyaddr = SUNXI_GPIO_BASE + (bank * 36) + 0x1c + sub*4;

  if (wiringPiDebug)
	printf("func:%s pin:%d,bank:%d index:%d sub:%d phyaddr:0x%x\n",__func__, pin,bank,index,sub,phyaddr); 
  
  if(BP_PIN_MASK[bank][index] != -1)
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
  alt=sunxi_get_pin_mode(pin);
  return alt ;
}


void bpi_pwmSetMode (int mode)
{
  sunxi_pwm_set_mode(mode);
  sunxi_pwm_set_enable(1);
  return;
}

void bpi_pwmSetRange (unsigned int range)
{
  sunxi_pwm_set_period(range);
  return;
}


void bpi_pwmSetClock (int divisor)
{
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
  if(bank == 11)
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
        printf("the pin you choose is not surport hardware PWM\n");
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
      printf("the pin(%d) you choose is not surport hardware PWM\n", pin);
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
  { "bpi-r1",	   10001, BPI_MODEL_R1, 1, 2, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M1P, physToGpio_BPI_M1P, pinTobcm_BPI_M1P, M1P_I2C_DEV, M1P_SPI_DEV, {M1P_PWM_OFFSET,M1P_I2C_OFFSET,M1P_SPI_OFFSET} },
  { "bpi-m2",	   10101, BPI_MODEL_M2, 1, 2, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2, physToGpio_BPI_M2, pinTobcm_BPI_M2, M2_I2C_DEV, M2_SPI_DEV, {M2_PWM_OFFSET,M2_I2C_OFFSET,M2_SPI_OFFSET} },
  { "bpi-m3",	   10201, BPI_MODEL_M3, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M3, physToGpio_BPI_M3, pinTobcm_BPI_M3, M3_I2C_DEV, M3_SPI_DEV, {M3_PWM_OFFSET,M3_I2C_OFFSET,M3_SPI_OFFSET} },
  { "bpi-m2p",	   10301, BPI_MODEL_M2P, 1, 2, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2P, physToGpio_BPI_M2P, pinTobcm_BPI_M2P, M2P_I2C_DEV, M2P_SPI_DEV, {M2P_PWM_OFFSET,M2P_I2C_OFFSET,M2P_SPI_OFFSET} },
  { "bpi-m64",	   10401, BPI_MODEL_M64, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M64, physToGpio_BPI_M64, pinTobcm_BPI_M64, M64_I2C_DEV, M64_SPI_DEV, {M64_PWM_OFFSET,M64_I2C_OFFSET,M64_SPI_OFFSET} },
  { "bpi-m2u",	   10501, BPI_MODEL_M2U, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2U, physToGpio_BPI_M2U, pinTobcm_BPI_M2U, M2U_I2C_DEV, M2U_SPI_DEV, {M2U_PWM_OFFSET,M2U_I2C_OFFSET,M2U_SPI_OFFSET} },
  { "bpi-m2m",	   10601, BPI_MODEL_M2M, 1, 1, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2M, physToGpio_BPI_M2M, pinTobcm_BPI_M2M, M2M_I2C_DEV, M2M_SPI_DEV, {M2M_PWM_OFFSET,M2M_I2C_OFFSET,M2M_SPI_OFFSET} },
  { "bpi-m2p_H2+", 10701, BPI_MODEL_M2P_H2P, 1, 2, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2P, physToGpio_BPI_M2P, pinTobcm_BPI_M2P, M2P_I2C_DEV, M2P_SPI_DEV, {M2P_PWM_OFFSET,M2P_I2C_OFFSET,M2P_SPI_OFFSET} },
  { "bpi-m2p_H5",  10801, BPI_MODEL_M2P_H5, 1, 2, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2P, physToGpio_BPI_M2P, pinTobcm_BPI_M2P, M2P_I2C_DEV, M2P_SPI_DEV, {M2P_PWM_OFFSET,M2P_I2C_OFFSET,M2P_SPI_OFFSET} },
  { "bpi-m2u_V40", 10901, BPI_MODEL_M2U_V40, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2U, physToGpio_BPI_M2U, pinTobcm_BPI_M2U, M2U_I2C_DEV, M2U_SPI_DEV, {M2U_PWM_OFFSET,M2U_I2C_OFFSET,M2U_SPI_OFFSET} },
  { "bpi-m2z",	   11001, BPI_MODEL_M2Z, 1, 1, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_M2P, physToGpio_BPI_M2P, pinTobcm_BPI_M2P, M2P_I2C_DEV, M2P_SPI_DEV, {M2P_PWM_OFFSET,M2P_I2C_OFFSET,M2P_SPI_OFFSET} },
  { "bpi-r2",	   11101, BPI_MODEL_R2, 1, 3, BPI_MAKER_SINOVOIP, 0, pinToGpio_BPI_R2, physToGpio_BPI_R2, pinTobcm_BPI_R2, R2_I2C_DEV, R2_SPI_DEV, {R2_PWM_OFFSET,R2_I2C_OFFSET,R2_SPI_OFFSET} },  // !!!!! mem field is copy from M3, don't know correct num for R2
  { NULL,		0, 0, 1, 2, 5, 0, NULL, NULL, NULL, NULL, NULL, {-1, -1, -1} },
} ;

extern int bpi_found;

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
  if ((bpiFd = fopen("/var/lib/bananapi/board.sh", "r")) == NULL) {
    return -1;
  }
  while(!feof(bpiFd)) {
    fgets(buffer, sizeof(buffer), bpiFd);
    sscanf(buffer, "BOARD=%s", hardware);
    //printf("BPI: buffer[%s] hardware[%s]\n",buffer, hardware);
// Search for board:
    for (board = bpiboard ; board->name != NULL ; ++board) {
      //printf("BPI: name[%s] hardware[%s]\n",board->name, hardware);
      if (strcmp (board->name, hardware) == 0) {
        //gpioLayout = board->gpioLayout;
        gpioLayout = board->model; // BPI: use model to replace gpioLayout
        //printf("BPI: name[%s] gpioLayout(%d)\n",board->name, gpioLayout);
        if(gpioLayout >= BPI_MODEL_MIN) {
          bpi_found = 1;
          break;
        }
      }
    }
    if(bpi_found == 1) {
      break;
    }
  }
  fclose(bpiFd);
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
