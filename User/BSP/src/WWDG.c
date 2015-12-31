/**
  ******************************************************************************
  * @file    WWDG.c
  * @author  Reason Chen
  * @version V1.0
  * @date    5-May-2015
  * @brief   WWDG driver
  ******************************************************************************
  */

#include "WWDG.h"
#include <stdio.h>

void WWDG_Feed(void)
{
    WWDG_SetCounter(127);
    printf(".");
}

void WWDG_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable WWDG clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = WWDG_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);

    /* WWDG clock counter = (PCLK1(32MHz)/4096)/8 = 976.5625 Hz (~1024 us)  */
    WWDG_SetPrescaler(WWDG_Prescaler_8);

    /* Set Window value to 80; WWDG counter should be refreshed only when the counter
    is below 80 (and greater than 64) otherwise a reset will be generated */
    WWDG_SetWindowValue(80);

    plat_int_reg_cb(STM32F10x_INT_WWDG, (void*)WWDG_Feed);

    /* enable WWDG interrupt occurs while counter count down to 0x40 */
    WWDG_EnableIT();

    WWDG_ClearFlag();

    /* Enable WWDG and set counter value to 127, WWDG timeout = ~1024 us * 64 = 65.536 ms
    In this case the refresh window is: ~1024 us * (80-64) = 16.384ms
    */
    WWDG_Enable(127);
}