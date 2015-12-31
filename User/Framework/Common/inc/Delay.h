/*******************************************************************************/
 /**
  ******************************************************************************
  * @file    Delay.H
  * @author  lschen@miramems.com
  * @version V0.1.0
  * @date    26/2/2014
  * @brief   Simple delay functions.
  ******************************************************************************/

#ifndef __DELAY_H__
#define __DELAY_H__

#include "stm32f10x.h"

/********************************************************************************/
/**
  * @brief: inaccurate us delay
  * @param: us to delay
  * @retval : None
**/
/********************************************************************************/
void uDelay(u16 nus);
/********************************************************************************/
/**
  * @brief: inaccurate ms delay
  * @param: ms to delay
  * @retval : None
  * @Note: please use sDelay() if nms > 1800ms
**/
/********************************************************************************/
void mDelay(u16 nms);

/* Second level delay */
void sDelay(u8 ns);

#endif /* !__DELAY_H__ */
