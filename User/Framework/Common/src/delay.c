/*******************************************************************************/
 /**
  ******************************************************************************
  * @file    Delay.c
  * @author  Reason Chen
  * @version V1.0
  * @date    26/2/2014
  * @brief   Simple delay func
  ******************************************************************************
  * Note: By Default, the system clock is 72MHz, and Cortex-M3 timer clock source
  *       is the 1/8 of the HCLK.
  */

/* Includes ------------------------------------------------------------------*/
#include "Delay.h"
#include "stm32f10x_conf.h"

#ifdef CLOCK_HSE
#define SYS_CLK_NORMALIZE_RATIO         9
#elif defined CLOCK_HSI
#define SYS_CLK_NORMALIZE_RATIO         8
#endif

void uDelay(u16 nus)
{
#if (defined STM32F10X_HD)
    volatile u32 temp;

    SysTick->LOAD = SYS_CLK_NORMALIZE_RATIO * nus;
    SysTick->VAL = 0x00;
    SysTick->CTRL = 0x01;
    do
    {
        temp = SysTick->CTRL;
    }while((temp&0x01)&&(!(temp&(1<<16))));

    SysTick->CTRL = 0x00;
    SysTick->VAL = 0x00;
#else
    nus *= 6;
    while(nus--);
#endif
}


void mDelay(u16 nms)
{
#if (defined STM32F10X_HD)
    volatile u32 temp;

    SysTick->LOAD = SYS_CLK_NORMALIZE_RATIO * 1000 * nms;
    SysTick->VAL = 0x00;
    SysTick->CTRL = 0x01;
    do
    {
        temp = SysTick->CTRL;
    }while((temp&0x01)&&(!(temp&(1<<16))));

    SysTick->CTRL = 0x00;
    SysTick->VAL = 0x00;
#else
  while(nms--)
  {
    uDelay(1000);
  }
#endif
}

void sDelay(u8 ns)
{
    while(ns--){
        mDelay(1000);
    }
}
/******************************************************************************/



