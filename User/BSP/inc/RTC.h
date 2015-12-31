/**
  ******************************************************************************
  * @file    RTC.h
  * @author  Reason Chen
  * @version V1.0
  * @date    5-May-2015
  * @brief   RealTime Clock
  ******************************************************************************
  */

#ifndef __RTC_H__
#define __RTC_H__

#include "stm32f10x_conf.h"

#define RTC_ALR_WAKE_BIT                0
#define RTC_ALR_PLAN_BIT                1
#define RTC_ALR_WAKE_MASK               (1<<RTC_ALR_WAKE_BIT)
#define RTC_ALR_PLAN_MASK               (1<<RTC_ALR_PLAN_BIT)

#define RTC_WAKE_ALR_SEC                3

s8 RTC_Init(void);
void Time_Adjust(u8 HH, u8 MM, u8 SS);
void Time_Display(void);
void RTC_AlarmSet(u32 val);
s8 RTC_AlarmGet(void);
u8 RTC_LoadAlarmCfg(void);
void RTC_AlarmCfgWindowStateSet(void);
u8 RTC_AlarmCfgWindowStateGet(void);
void RTC_AlarmCfgWindowReset(void);

#endif /* !__RTC_H__ */
