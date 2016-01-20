/******************** (C) COPYRIGHT 2007 EJ ********************
* File Name          : PwrManagement.c
* Author             : Reason Chen
* Version            : V1.0
* Date               : 06/12/2014
* Description        : Power Management Policy
*******************************************************************************/
#include <stdlib.h>
#include <string.h>
#include "boardcfg.h"
#include "PwrManagement.h"
#include "delay.h"
#include "IrDA.h"
#include "SweepRobot.h"
#include "CtrlPanel.h"
#include "Buzzer.h"
#include "PWM.h"
#include "Measurement.h"

u8 gSystemIdleCnt = PM_SYS_SB_SEC;

void PM_SysTryToStandby(void)
{
    Msg_t   Msg;

    gSystemIdleCnt--;
    if(gSystemIdleCnt==0){
        Msg.expire = 0;
        Msg.prio = MSG_PRIO_HIGHEST;
        Msg.type = MSG_TYPE_PM;
        Msg.MsgCB = NULL;
        Msg.Data.PMEvt = PM_MODE_STANDBY;
        SweepRobot_SendMsg(&Msg);
    }
}

s8 PM_SysTryToResume(void)
{
    s16  AliveCnt = 50;                                                           //5ms
    u8   i = 0, flag = 0;

    while(AliveCnt--){
        for(i = 0; i < IRDA_LIGHT_NUM; i++){
            if(0 != (PulsTimeS[i]+PulsTimeE[i])){
                flag = 1;
                break;
            }
        }
        if(flag){
            break;
        }
        if(PM_WAKEUP_PIN_SIGN()){
            return 1;
        }
        uDelay(500);
    }
    if(AliveCnt < 0){
        return 0;
    }

    AliveCnt = 150;
    while(AliveCnt--){
        if( IrDA_FrameBuf==REMOTE_CMD_WAKEUP ){
            return 1;
        }
        if(PM_WAKEUP_PIN_SIGN()){
            return 1;
        }
        mDelay(1);
    }
    return 0;
}

void PM_TryToRusumeWork(void)
{
    Msg_t   Msg;
    u8      flag = 0;

    flag = RTC_LoadAlarmCfg();
    if(flag & RTC_ALR_PLAN_MASK){
        Msg.expire = 0;
        Msg.prio = MSG_PRIO_HIGHEST;
        Msg.type = MSG_TYPE_PM;
        Msg.MsgCB = NULL;
        Msg.Data.PMEvt = PM_MODE_RESUME;
        SweepRobot_SendMsg(&Msg);
    }
}

void RCC_Config(void)
{
    RCC_DeInit();

#ifdef CLOCK_HSE
    RCC_HSEConfig(RCC_HSE_ON);
    while(RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET);
#elif defined CLOCK_HSI
    RCC_HSICmd(ENABLE);
    while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);
#endif

    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    FLASH_SetLatency(FLASH_Latency_2);

    RCC_HCLKConfig(RCC_SYSCLK_Div1);

    RCC_PCLK2Config(RCC_HCLK_Div1);

    RCC_PCLK1Config(RCC_HCLK_Div2);

#ifdef CLOCK_HSE
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
#elif defined CLOCK_HSI
    RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_16);
#endif

    RCC_PLLCmd(ENABLE);

    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
}

void PM_Init(void)
{
    GPIO_InitTypeDef    GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(POWER_SUPPLY_CTRL_GPIO_PERIPH_ID, ENABLE);

    GPIO_InitStructure.GPIO_Pin = POWER_SUPPLY_CTRL_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(POWER_SUPPLY_CTRL_GPIO, &GPIO_InitStructure);
    PM_POWER_SUPPLY_CTRL(PM_OFF);

    /* LED pre-inited */
    RCC_APB2PeriphClockCmd(CTRL_LED_CTRL_GPIO_PERIPH_ID, ENABLE);
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = CTRL_LED_R_CTRL_PIN;
    GPIO_Init(CTRL_LED_R_CTRL_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = CTRL_LED_G_CTRL_PIN;
    GPIO_Init(CTRL_LED_G_CTRL_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = CTRL_LED_B_CTRL_PIN;
    GPIO_Init(CTRL_LED_B_CTRL_GPIO, &GPIO_InitStructure);
    /* LED Off on startup */
    GPIO_ResetBits(CTRL_LED_R_CTRL_GPIO, CTRL_LED_R_CTRL_PIN);
    GPIO_ResetBits(CTRL_LED_G_CTRL_GPIO, CTRL_LED_G_CTRL_PIN);
    GPIO_ResetBits(CTRL_LED_B_CTRL_GPIO, CTRL_LED_B_CTRL_PIN);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    /* Enable Wake up pin */
    PWR_WakeUpPinCmd(ENABLE);

    /* Powered on by default */
    PM_POWER_SUPPLY_CTRL(PM_ON);
    uDelay(250);

    /* Change SYSCLK to running clock */
    RCC_Config();

    SystemCoreClockUpdate();

    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
}

void PM_DeInit(void)
{

}

s8 PM_EnterPwrMode(enum PM_Mode mode)
{
    switch(mode)
    {
        case PM_MODE_RESUME:
            break;
        case PM_MODE_SLEEP:
        case PM_MODE_STOP:
        case PM_MODE_STANDBY:
            /* Prepare for enterring to low power state */
            CtrlPanel_LEDCtrl(CTRL_PANEL_LED_GREEN, 0);
//            WWDG_DeInit();
            RCC_DeInit();
            PM_POWER_SUPPLY_CTRL(PM_OFF);
            uDelay(1000);
            /* Enter to low power state */
            PWR_EnterSTANDBYMode();
            while(1);
            break;
    }
    return 0;
}
