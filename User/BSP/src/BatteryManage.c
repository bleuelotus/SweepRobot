/**
  ******************************************************************************
  * @file    Battery.c
  * @author  Reason Chen
  * @version V1.0
  * @date    5-May-2015
  * @brief   Battery management driver
  ******************************************************************************
  */

#include <stdlib.h>
#include "delay.h"
#include "Measurement.h"
#include "BatteryManage.h"
#include "PWM.h"
#include "MsgQueue.h"
#include "SweepRobot.h"
#include "CtrlPanel.h"
#include "IrDA.h"

//#define DEBUG_BM

#define ADC_BAT_SAMPLE_AVE_CNT              5

#ifdef CLOCK_HSE
#define BM_CHARGE_POWER_MAX                 360             /* duty cycle */
#elif defined CLOCK_HSI
#define BM_CHARGE_POWER_MAX                 320             /* duty cycle */
#endif

#define IS_CHARGE_CONNECTED()               GPIO_ReadInputDataBit(BM_CHARGE_SW_STATUS_GPIO, BM_CHARGE_SW_STATUS_PIN)

enum ADC_BAT_CHAN{
    ADC_BAT_VOL,
    ADC_BAT_CUR,
#ifdef REVISION_1_2
    ADC_VREFIN,
#endif
    ADC_BAT_CHANNEL_NUM,
};

#ifdef REVISION_1_2
static u16 BM_CHARGE_CUR_50MA               = 0;
static u16 BM_CHARGE_CUR_100MA              = 0;
static u16 BM_CHARGE_CUR_600MA              = 0;
static u16 BM_CHARGE_CUR_1000MA             = 0;

static u16 BAT_LEVEL_LOW                    = 0;
static u16 BAT_LEVEL_HIGH                   = 0;
static u16 BAT_LEVEL_FULL                   = 0;
static u16 BAT_CHARGE_LEVEL_FULL            = 0;
#else
#define BM_CHARGE_CUR_50MA                  0x1F
#define BM_CHARGE_CUR_100MA                 0x3E
#define BM_CHARGE_CUR_600MA                 0x175
#define BM_CHARGE_CUR_1000MA                0x26C

enum BatteryLevelLSB {
                                                            /* SampleV * 6 = BATV */
    BAT_LEVEL_LOW                           = 0x0A81,       /* 2.16V  ---  13.0V */
    BAT_LEVEL_HIGH                          = 0x0AFD,       /* 2.27V  ---  13.6V */
    BAT_LEVEL_FULL                          = 0x0CEE,       /* 2.67V   --- 16.0V */
    BAT_CHARGE_LEVEL_FULL                   = 0x0D2C,       /* 2.72V  ---  16.3V */
};
#endif

BatteryCond_t gBM_Cond = { 100,   BAT_STATE_UNKNOWN,    BAT_STATE_UNKNOWN };
static u16 TempADC[ADC_BAT_CHANNEL_NUM][ADC_BAT_SAMPLE_AVE_CNT] = {0};
static u32 ADC_BatLSB[ADC_BAT_CHANNEL_NUM] = {0};
static u8 BM_StateInited = 0;
static u8 BM_ChargeFlag = 0;
static Msg_t Msg;

static u16 ChargeCnt = 0;
static s8 LedBrightnessSch = 0, LedBrightnessDir = 1;
static u8 NotifyCnt = 0;
#define BM_EVT_NOTIFY_PERIOD                30                                  //30 * 100ms = 3s

u8 BM_ChargeExit(void);
u8 BM_ChargeProc(void);
void BM_ChargeStart(void);
void BM_ChargeStop(void);

void BM_ConditionUpdate(void)
{
    u8      i = 0;

    for(i = 0; i < ADC_BAT_SAMPLE_AVE_CNT-1; i++){
        TempADC[ADC_BAT_VOL][i] = TempADC[ADC_BAT_VOL][i+1];
        TempADC[ADC_BAT_CUR][i] = TempADC[ADC_BAT_CUR][i+1];
        TempADC[ADC_VREFIN][i] = TempADC[ADC_VREFIN][i+1];
    }

    TempADC[ADC_BAT_VOL][ADC_BAT_SAMPLE_AVE_CNT-1] = ADCConvertedLSB[MEAS_CHAN_BAT_VOL-1];
    TempADC[ADC_BAT_CUR][ADC_BAT_SAMPLE_AVE_CNT-1] = ADCConvertedLSB[MEAS_CHAN_BAT_CHARGE_CUR-1];
    TempADC[ADC_VREFIN][ADC_BAT_SAMPLE_AVE_CNT-1] = ADCConvertedLSB[MEAS_CHAN_VREFIN-1];

    if(BM_StateInited < ADC_BAT_SAMPLE_AVE_CNT){
        BM_StateInited++;
        return;
    }

    ADC_BatLSB[ADC_BAT_VOL] = 0;
    ADC_BatLSB[ADC_BAT_CUR] = 0;
    ADC_BatLSB[ADC_VREFIN] = 0;
    for(i = 0; i < ADC_BAT_SAMPLE_AVE_CNT; i++){
        ADC_BatLSB[ADC_BAT_VOL] += TempADC[ADC_BAT_VOL][i];
        ADC_BatLSB[ADC_BAT_CUR] += TempADC[ADC_BAT_CUR][i];
        ADC_BatLSB[ADC_VREFIN] += TempADC[ADC_VREFIN][i];
    }

    ADC_BatLSB[ADC_BAT_VOL] /= ADC_BAT_SAMPLE_AVE_CNT;
    ADC_BatLSB[ADC_BAT_CUR] /= ADC_BAT_SAMPLE_AVE_CNT;
    ADC_BatLSB[ADC_VREFIN] /= ADC_BAT_SAMPLE_AVE_CNT;

    BM_CHARGE_CUR_50MA               = ADC_BatLSB[ADC_VREFIN]/48;
    BM_CHARGE_CUR_100MA              = ADC_BatLSB[ADC_VREFIN]/24;
    BM_CHARGE_CUR_600MA              = ADC_BatLSB[ADC_VREFIN]/4;
    BM_CHARGE_CUR_1000MA             = ADC_BatLSB[ADC_VREFIN]*5/12;

    BAT_LEVEL_LOW                    = ADC_BatLSB[ADC_VREFIN]*9/5;
    BAT_LEVEL_HIGH                   = ADC_BatLSB[ADC_VREFIN]*17/9;
    BAT_LEVEL_FULL                   = ADC_BatLSB[ADC_VREFIN]*20/9;
    BAT_CHARGE_LEVEL_FULL            = ADC_BatLSB[ADC_VREFIN]*9/4;

    /* Update battery state */
    if(IS_CHARGE_CONNECTED()){
        switch(gBM_Cond.LastState){
            case BAT_STATE_DISCHARGING:
                /* To solve the IrDA stop problem */
                IrDA_DisableLights();
                gBM_Cond.state = BAT_STATE_WAIT_FOR_CHARGE;
                /* Send power link message */
                Msg.expire = 0;
                Msg.type = MSG_TYPE_BM;
                Msg.prio = MSG_PRIO_LOWEST;
                Msg.MsgCB = BM_ChargeStart;
                Msg.Data.BatEvt = BM_EVT_POWER_LINK;
                SweepRobot_SendMsg(&Msg);
                break;
            case BAT_STATE_WAIT_FOR_CHARGE:
                if(BM_ChargeFlag){
                    gBM_Cond.state = BAT_STATE_CHARGING;
                }
                break;
            case BAT_STATE_CHARGING:
                if(BM_ChargeProc()){
                    BM_ChargeExit();
                    gBM_Cond.state = BAT_STATE_CHARGE_COMPLETE;
                    /* Send complete charge message */
                    Msg.expire = 0;
                    Msg.type = MSG_TYPE_BM;
                    Msg.prio = MSG_PRIO_LOWEST;
                    Msg.MsgCB = BM_ChargeStop;
                    Msg.Data.BatEvt = BM_EVT_CHARGE_COMPLETE;
                    SweepRobot_SendMsg(&Msg);
                }
                /* To solve the IrDA stop problem */
                if(!IrDA_State()){
                    IrDA_ReEnableLights();
                }

#if (defined DEBUG_LOG && defined DEBUG_BM)
#ifndef REVISION_1_2
                printf("Charge Lvl: %d, %d.\r\n", gBM_Cond.level, ChargeCnt);
#else
                printf("Charge Vol: %3.2f, %d.\r\n", (float)ADC_BatLSB[ADC_BAT_VOL]/(float)ADC_BatLSB[ADC_VREFIN]*1.2f*6.f, ChargeCnt);
#endif
#endif
                break;
            case BAT_STATE_CHARGE_COMPLETE:
                if(!BM_ChargeFlag && gBM_Cond.level < BM_BAT_FULL_LVL){
                    gBM_Cond.state = BAT_STATE_WAIT_FOR_CHARGE;
                    /* Send charge request message */
                    Msg.expire = 0;
                    Msg.type = MSG_TYPE_BM;
                    Msg.prio = MSG_PRIO_LOWEST;
                    Msg.MsgCB = BM_ChargeStart;
                    Msg.Data.BatEvt = BM_EVT_POWER_LINK;
                    SweepRobot_SendMsg(&Msg);
                }
                break;
            case BAT_STATE_UNKNOWN:
                gBM_Cond.state = BAT_STATE_DISCHARGING;
                break;
        }
    }
    else{
        switch(gBM_Cond.LastState){
            case BAT_STATE_DISCHARGING:
                if( (gBM_Cond.level < BM_BAT_WARNNING_LVL) && (!BM_ChargeFlag) ){

                    if((++NotifyCnt) > BM_EVT_NOTIFY_PERIOD){
                        NotifyCnt = 0;
                        /* Send low battery condition message */
                        Msg.expire = 0;
                        Msg.type = MSG_TYPE_BM;
                        Msg.prio = MSG_PRIO_LOWEST;
                        Msg.MsgCB = NULL;
                        if(gBM_Cond.level < BM_BAT_CRITICAL_LVL){
                            Msg.Data.BatEvt = BM_EVT_CRITICAL_LEVEL;
                        }
                        else{
                            Msg.Data.BatEvt = BM_EVT_LOW_LEVEL;
                        }
                        SweepRobot_SendMsg(&Msg);
                    }
                }
                else{
                    NotifyCnt = 0;
                }
                break;
            case BAT_STATE_WAIT_FOR_CHARGE:
                gBM_Cond.state = BAT_STATE_DISCHARGING;
                /* To solve the IrDA stop problem */
                if(!IrDA_State()){
                    IrDA_ReEnableLights();
                }
                /* Power loss event */
                Msg.expire = 0;
                Msg.type = MSG_TYPE_BM;
                Msg.prio = MSG_PRIO_LOWEST;
                Msg.MsgCB = BM_ChargeStop;
                Msg.Data.BatEvt = BM_EVT_POWER_LOSS;
                SweepRobot_SendMsg(&Msg);
                break;
            case BAT_STATE_CHARGING:
                BM_ChargeExit();
                gBM_Cond.state = BAT_STATE_DISCHARGING;
                /* Send power link loss message */
                Msg.expire = 0;
                Msg.type = MSG_TYPE_BM;
                Msg.prio = MSG_PRIO_LOWEST;
                Msg.MsgCB = BM_ChargeStop;
                Msg.Data.BatEvt = BM_EVT_POWER_LOSS;
                SweepRobot_SendMsg(&Msg);
                break;
            case BAT_STATE_CHARGE_COMPLETE:
                gBM_Cond.state = BAT_STATE_DISCHARGING;
                /* Send power link loss message */
                Msg.expire = 0;
                Msg.type = MSG_TYPE_BM;
                Msg.prio = MSG_PRIO_LOWEST;
                Msg.MsgCB = NULL;
                Msg.Data.BatEvt = BM_EVT_POWER_LOSS;
                SweepRobot_SendMsg(&Msg);
                break;
            case BAT_STATE_UNKNOWN:
                gBM_Cond.state = BAT_STATE_DISCHARGING;
                break;
        }
    }

    if(ADC_BatLSB[ADC_BAT_VOL] > BAT_LEVEL_FULL){
        gBM_Cond.level = 100;
    }
    else if(ADC_BatLSB[ADC_BAT_VOL] > BAT_LEVEL_LOW){
        if(gBM_Cond.state!=BAT_STATE_CHARGING){
            gBM_Cond.level = (u8)( (float)(ADC_BatLSB[ADC_BAT_VOL] - BAT_LEVEL_LOW) / (float)(BAT_LEVEL_FULL - BAT_LEVEL_LOW) * 100.f );
        }
        else{
            gBM_Cond.level = (u8)( (float)(ADC_BatLSB[ADC_BAT_VOL] - BAT_LEVEL_LOW) / (float)(BAT_CHARGE_LEVEL_FULL - BAT_LEVEL_LOW) * 100.f );
        }
    }
    else{
        /* Shouldn't be here */
        gBM_Cond.level = 0;
    }

    gBM_Cond.LastState = gBM_Cond.state;

#if (defined DEBUG_LOG && defined DEBUG_BM)
#ifdef REVISION_1_2
    printf("Bat Vol: %2.1f\r\n", (((float)ADC_BatLSB[ADC_BAT_VOL]*1.2f*6.f)/(float)ADC_BatLSB[ADC_VREFIN]));
#else
    printf("Bat Vol: %2.1f\r\n", (((float)ADC_BatLSB[ADC_BAT_VOL]*3.3f*6.f)/4096.f));
#endif
#endif
}

s8 BM_ChargePowerDec(void)
{
    u16  DutyCycle = 0;

    PWM_DutyCycleGet(PWM_CHAN_CHARGE, &DutyCycle);

    if(DutyCycle > 1){
        DutyCycle -= 1;
    }
    return PWM_DutyCycleSet(PWM_CHAN_CHARGE, DutyCycle);
}

s8 BM_ChargePowerInc(void)
{
    u16  DutyCycle = 0;

    PWM_DutyCycleGet(PWM_CHAN_CHARGE, &DutyCycle);

    if(DutyCycle < BM_CHARGE_POWER_MAX){
        DutyCycle += 1;
    }
    return PWM_DutyCycleSet(PWM_CHAN_CHARGE, DutyCycle);
}

void BM_ChargeStart(void)
{
#ifdef DEBUG_LOG
    printf("Start charging.\r\n");
#endif
    BM_ChargeFlag = 1;
}

void BM_ChargeStop(void)
{
#ifdef DEBUG_LOG
    printf("Stop charging %d.\r\n", gBM_Cond.level);
#endif
    BM_ChargeFlag = 0;
}

u8 BM_ChargeExit(void)
{
    ChargeCnt = 0;
    return PWM_DutyCycleSet(PWM_CHAN_CHARGE, 0);
}

u8 BM_ChargeProc(void)
{
    CtrlPanel_LEDCtrl(CTRL_PANEL_LED_BLUE, LedBrightnessSch);
    if((LedBrightnessDir > 0) && (LedBrightnessSch==10)){
        LedBrightnessDir = -1;
    }
    else if((LedBrightnessDir < 0) && (LedBrightnessSch==0)){
        LedBrightnessDir = 1;
    }
    LedBrightnessSch += LedBrightnessDir;

    if (ADC_BatLSB[ADC_BAT_VOL] <= BAT_LEVEL_HIGH){
        if (ADC_BatLSB[ADC_BAT_CUR] <= (BM_CHARGE_CUR_100MA)){
            BM_ChargePowerInc();
        }
        else if (ADC_BatLSB[ADC_BAT_CUR] > (BM_CHARGE_CUR_100MA)){
            BM_ChargePowerDec();
        }
    }
    else if (ADC_BatLSB[ADC_BAT_VOL] < BAT_CHARGE_LEVEL_FULL){
        if (ADC_BatLSB[ADC_BAT_CUR] <= (BM_CHARGE_CUR_600MA)){
            BM_ChargePowerInc();
        }
        else if (ADC_BatLSB[ADC_BAT_CUR] > (BM_CHARGE_CUR_600MA)){
            BM_ChargePowerDec();
        }
    }
    else{
        BM_ChargePowerDec();
        if (ADC_BatLSB[ADC_BAT_CUR] <= BM_CHARGE_CUR_50MA){
            if ((ChargeCnt++) >= 100){
                return 1;
            }
        }
    }
    return 0;
}

void BM_Init(void)
{
    GPIO_InitTypeDef            GPIO_InitStructure;
    NVIC_InitTypeDef            NVIC_InitStructure;
    TIM_TimeBaseInitTypeDef     TIM_TimeBaseStructure;

    /* Enable GPIO clock */
    RCC_APB2PeriphClockCmd(BM_CHARGE_SW_STATUS_GPIO_PERIPH_ID, ENABLE);

    GPIO_InitStructure.GPIO_Pin = BM_CHARGE_SW_STATUS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BM_CHARGE_SW_STATUS_GPIO, &GPIO_InitStructure);

    PWM_ChanInit(PWM_CHAN_CHARGE, 1, 0);

    NVIC_InitStructure.NVIC_IRQChannel = BAT_MONITOR_TIM_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = BAT_MONITOR_TIM_IRQ_PP;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = BAT_MONITOR_TIM_IRQ_SP;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    RCC_APB1PeriphClockCmd(BAT_MONITOR_TIM_PERIPH_ID , ENABLE);
    TIM_DeInit(BAT_MONITOR_TIM);
    TIM_TimeBaseStructure.TIM_Period = 10000-1;                                 // 100ms
#ifdef CLOCK_HSE
    TIM_TimeBaseStructure.TIM_Prescaler = 720-1;
#elif defined CLOCK_HSI
    TIM_TimeBaseStructure.TIM_Prescaler = 640-1;
#endif
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(BAT_MONITOR_TIM, &TIM_TimeBaseStructure);
    TIM_ClearFlag(BAT_MONITOR_TIM, TIM_FLAG_Update);
    TIM_ITConfig(BAT_MONITOR_TIM, TIM_IT_Update, ENABLE);

    plat_int_reg_cb(BAT_MONITOR_TIM_INT_IDX, (void*)BM_ConditionUpdate);
    TIM_Cmd(BAT_MONITOR_TIM, ENABLE);
}

#ifdef USE_SWRB_TEST
static u8 BM_TestChargeProc(void)
{
    CtrlPanel_LEDCtrl(CTRL_PANEL_LED_BLUE, LedBrightnessSch);
    if((LedBrightnessDir > 0) && (LedBrightnessSch==10)){
        LedBrightnessDir = -1;
    }
    else if((LedBrightnessDir < 0) && (LedBrightnessSch==0)){
        LedBrightnessDir = 1;
    }
    LedBrightnessSch += LedBrightnessDir;

    if (ADC_BatLSB[ADC_BAT_VOL] <= BAT_LEVEL_HIGH){
        if (ADC_BatLSB[ADC_BAT_CUR] <= (BM_CHARGE_CUR_100MA)){
            BM_ChargePowerInc();
        }
        else if (ADC_BatLSB[ADC_BAT_CUR] > (BM_CHARGE_CUR_100MA)){
            BM_ChargePowerDec();
        }
    }
    else if (ADC_BatLSB[ADC_BAT_VOL] < BAT_CHARGE_LEVEL_FULL){
        if (ADC_BatLSB[ADC_BAT_CUR] <= (BM_CHARGE_CUR_600MA)){
            BM_ChargePowerInc();
        }
        else if (ADC_BatLSB[ADC_BAT_CUR] > (BM_CHARGE_CUR_600MA)){
            BM_ChargePowerDec();
        }
    }
    else{
        BM_ChargePowerDec();
    }
    return 0;
}

void BM_TestConditionUpdate(void)
{
    u8      i = 0;

    for(i = 0; i < ADC_BAT_SAMPLE_AVE_CNT-1; i++){
        TempADC[ADC_BAT_VOL][i] = TempADC[ADC_BAT_VOL][i+1];
        TempADC[ADC_BAT_CUR][i] = TempADC[ADC_BAT_CUR][i+1];
        TempADC[ADC_VREFIN][i] = TempADC[ADC_VREFIN][i+1];
    }

    TempADC[ADC_BAT_VOL][ADC_BAT_SAMPLE_AVE_CNT-1] = ADCConvertedLSB[MEAS_CHAN_BAT_VOL-1];
    TempADC[ADC_BAT_CUR][ADC_BAT_SAMPLE_AVE_CNT-1] = ADCConvertedLSB[MEAS_CHAN_BAT_CHARGE_CUR-1];
    TempADC[ADC_VREFIN][ADC_BAT_SAMPLE_AVE_CNT-1] = ADCConvertedLSB[MEAS_CHAN_VREFIN-1];

    if(BM_StateInited < ADC_BAT_SAMPLE_AVE_CNT){
        BM_StateInited++;
        return;
    }

    ADC_BatLSB[ADC_BAT_VOL] = 0;
    ADC_BatLSB[ADC_BAT_CUR] = 0;
    ADC_BatLSB[ADC_VREFIN] = 0;
    for(i = 0; i < ADC_BAT_SAMPLE_AVE_CNT; i++){
        ADC_BatLSB[ADC_BAT_VOL] += TempADC[ADC_BAT_VOL][i];
        ADC_BatLSB[ADC_BAT_CUR] += TempADC[ADC_BAT_CUR][i];
        ADC_BatLSB[ADC_VREFIN] += TempADC[ADC_VREFIN][i];
    }

    ADC_BatLSB[ADC_BAT_VOL] /= ADC_BAT_SAMPLE_AVE_CNT;
    ADC_BatLSB[ADC_BAT_CUR] /= ADC_BAT_SAMPLE_AVE_CNT;
    ADC_BatLSB[ADC_VREFIN] /= ADC_BAT_SAMPLE_AVE_CNT;

    BM_CHARGE_CUR_50MA               = ADC_BatLSB[ADC_VREFIN]/48;
    BM_CHARGE_CUR_100MA              = ADC_BatLSB[ADC_VREFIN]/24;
    BM_CHARGE_CUR_600MA              = ADC_BatLSB[ADC_VREFIN]/4;
    BM_CHARGE_CUR_1000MA             = ADC_BatLSB[ADC_VREFIN]*5/12;

    BAT_LEVEL_LOW                    = ADC_BatLSB[ADC_VREFIN]*9/5;
    BAT_LEVEL_HIGH                   = ADC_BatLSB[ADC_VREFIN]*17/9;
    BAT_LEVEL_FULL                   = ADC_BatLSB[ADC_VREFIN]*20/9;
    BAT_CHARGE_LEVEL_FULL            = ADC_BatLSB[ADC_VREFIN]*9/4;
    
    if(IS_CHARGE_CONNECTED()){
        BM_TestChargeProc();
    }else{
        PWM_DutyCycleSet(PWM_CHAN_CHARGE, 0);
    }
}
#endif
