/******************** (C) COPYRIGHT 2007 EJE ********************
* File Name          : SweepRobot.c
* Author             : Reason Chen
* Version            : V1.0
* Date               : 5-May-2015
* Description        : Sweeping Robot demo
*******************************************************************************/
#include <stdlib.h>
#include "math.h"
#include "string.h"
#include "boardcfg.h"
#include "stm32f10x_conf.h"
#include "SweepRobot.h"
#include "SweepRobotTest.h"
#include "MotorCtrl.h"
#include "delay.h"
#include "BatteryManage.h"
#include "PWM.h"
#include "PwrManagement.h"
#include "MotionCtrl.h"
#include "Measurement.h"
#include "CtrlPanel.h"
#include "RTC.h"
#include "Buzzer.h"
#include "Usart.h"
#include "WWDG.h"

#define DEBUG_HOME

enum RobotState     gRobotState;
enum RobotWorkMode  gRobotMode, gRobotModeLast;
static s8 RobotPlanFlag = 0;

#define ROBOT_MAIN_MSG_Q_SIZE               10
#define STARTUP_DELAY_TIME                  30000                               // 3s

struct RobotHomingState_s {

    u8      Pos;
    s16     Angle;
};

static MsgQueue_t *MainMsgQ = NULL;
static const s16 HomingSigSrc2AngleMap[] = { 180, 90, 45, 315, 270 };
enum _RobotHomingStage gHomingStage = ROBOT_HOMING_STAGE_UNKNOWN;
static enum _RobotHomingStage LastHomingStage = ROBOT_HOMING_STAGE_UNKNOWN;
static u8 gRobotStartupSeqNum = 0;
static const u32 STARTUP_SEQ_DELAY_TIME[5] = {10000, 2500, 2500, 2500, 2500};

void SweepRobot_PMMsgProc(enum PM_Mode mode);
void SweepRobot_BMMsgProc(enum BatteryEvt evt);
void SweepRobot_CtrlMsgProc(u8 CtrlCode);
void SweepRobot_PwrStationMsgProc(PwrStationSigData_t *PwrSig);
void SweepRobot_MotionMsgProc(enum MotionEvt evt);
void SweepRobot_AutoModeProc(void);

s8 SweepRobot_Init(void)
{
    s8      err = 0;

    /* Power management init */
    PM_Init();
    /* Enable WWDG */
//    WWDG_Init();
#ifdef DEBUG_LOG
    UART4_Config();
#endif
    IrDA_Init();

    /* Real time clock init */
    RobotPlanFlag = RTC_Init();
    if(RobotPlanFlag & RTC_ALR_PLAN_MASK){
    }
    else if(RobotPlanFlag & RTC_ALR_WAKE_MASK){
        if(!PM_SysTryToResume()){
            PM_EnterPwrMode(PM_MODE_STANDBY);
        }
    }
    /* Message Queue for processing robot state */
    MainMsgQ = MsgQueue_Create(ROBOT_MAIN_MSG_Q_SIZE);
    if(MainMsgQ == NULL){
        err = -1;
        goto SWEEPROBOT_INIT_FAIL;
    }
    /* Physical state measurement */
    Meas_Init();
    /* Start measurement */
    Meas_Start();
    /* PWM resource init */
    PWM_ControllerInit();
    /* PWM controller start */
    PWM_ControllerStart();
    /* Battery management init */
    BM_Init();
    /* Buzzer init */
    Buzzer_Init();
    /* Motor controller init */
    err = MotorCtrl_Init();
    if(err)
        goto SWEEPROBOT_INIT_FAIL;
    /* Robot motion control */
    MotionCtrl_Init();
    /* CtrlPanel config */
    CtrlPanel_Init();

    gRobotState = ROBOT_STATE_IDLE;
#ifdef DEBUG_LOG
    printf("Robot init OK !\r\n");
#endif
    return err;
SWEEPROBOT_INIT_FAIL:
#ifdef DEBUG_LOG
    printf("Robot init failed !\r\n");
#endif
    return err;
}

void SweepRobot_Start(void)
{
    u8  i = 0;

    if(RobotPlanFlag & RTC_ALR_WAKE_MASK){
        RobotPlanFlag &= ~RTC_ALR_WAKE_MASK;
        if(!PM_WAKEUP_PIN_SIGN()){
          Buzzer_Play(BUZZER_ONE_PULS, BUZZER_SND_VERY_LONG);
        }
    }

    if(RobotPlanFlag & RTC_ALR_PLAN_MASK){
        RobotPlanFlag &= ~RTC_ALR_PLAN_MASK;
        SweepRobot_AutoModeProc();
    }

    while(1){
        /* Message loop */
        for(i = 0; i < ROBOT_MAIN_MSG_Q_SIZE; i++){
            if(IS_UART4_GET_DATA_FINISH()){
                USART_CmdProc();
            }
            if(!MainMsgQ->Msg.expire){
                if(MainMsgQ->Msg.prio == MSG_PRIO_HIGHEST){
                    if(gRobotState != ROBOT_STATE_TEST){
                        switch(MainMsgQ->Msg.type){
                            case MSG_TYPE_PM:
    #ifdef DEBUG_LOG
                                printf("PM msg %d.\r\n", MainMsgQ->Msg.Data.PMEvt);
    #endif
                                SweepRobot_PMMsgProc(MainMsgQ->Msg.Data.PMEvt);
                                break;
                            case MSG_TYPE_BM:
    #ifdef DEBUG_LOG
                                printf("BM msg %d.\r\n", MainMsgQ->Msg.Data.BatEvt);
    #endif
                                SweepRobot_BMMsgProc(MainMsgQ->Msg.Data.BatEvt);
                                break;
                            case MSG_TYPE_CTRL:
                                PM_ResetSysIdleState();
    #ifdef DEBUG_LOG
                                printf("CTRL msg code:0x%X.\r\n", MainMsgQ->Msg.Data.ByteVal);
    #endif
                                SweepRobot_CtrlMsgProc(MainMsgQ->Msg.Data.ByteVal);
                                break;
                            case MSG_TYPE_PWR_STATION:
    #ifdef DEBUG_LOG
                                printf("PWR_STATION msg Pos:%d, Sig:0x%X.\r\n", MainMsgQ->Msg.Data.PSSigDat.src, MainMsgQ->Msg.Data.PSSigDat.sig);
    #endif
                                SweepRobot_PwrStationMsgProc(&MainMsgQ->Msg.Data.PSSigDat);
                                break;
                            case MSG_TYPE_MOTION:
                                PM_ResetSysIdleState();
    #ifdef DEBUG_LOG
                                printf("MOTION msg 0x%X.\r\n", MainMsgQ->Msg.Data.MEvt);
    #endif                            
                                SweepRobot_MotionMsgProc(MainMsgQ->Msg.Data.MEvt);
                                break;
                            case MSG_TYPE_TEST_CTRL:
                                PM_ResetSysIdleState();
                                SweepRobotTest_StartCtrlMsgPorc(&MainMsgQ->Msg.Data.TestCtrlDat);
                                break;
                        }
                    }else{
                        switch(MainMsgQ->Msg.type){
                            case MSG_TYPE_PM:
                                SweepRobot_PMMsgProc(MainMsgQ->Msg.Data.PMEvt);
                                break;
                            case MSG_TYPE_CTRL:
                                PM_ResetSysTestState();
                                SweepRobotTest_IrDARxCodeProc(&MainMsgQ->Msg.Data.PSSigDat);
                                break;
                            case MSG_TYPE_PWR_STATION:
                                SweepRobotTest_IrDARxCodeProc(&MainMsgQ->Msg.Data.PSSigDat);
                                break;
                            case MSG_TYPE_TEST_CTRL:
                                PM_ResetSysTestState();
                                SweepRobotTest_CtrlMsgProc(&MainMsgQ->Msg.Data.TestCtrlDat);
                                break;
                        }
                    }
                    if(MainMsgQ->Msg.MsgCB!=NULL){
                        MainMsgQ->Msg.MsgCB();
                    }
                    MainMsgQ->Msg.expire = 1;
                }
                else{
                    MainMsgQ->Msg.prio--;
                }
            }
            MainMsgQ = MainMsgQ->Next;
        }
    }
}

void SweepRobot_StartupInit(void)
{
    gRobotState = ROBOT_STATE_STARTUP;
    gRobotStartupSeqNum++;

    plat_int_reg_cb(MOTION_MONITOR_TIM_INT_IDX, (void*)SweepRobot_StartupInit);

    if(1==gRobotStartupSeqNum){
        if(WHEEL_FLOAT_SIGN_ALL
         ||ASH_TRAY_INSTALL_SIGN
          ){
            goto STARTUP_FAIL_ON_WF_AT;
        }
        MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_FAN, MOTOR_FAN_CHAN_STARTUP_SPEED);
    }
    else if(2==gRobotStartupSeqNum){
#if defined REVISION_1_2
        _ADCRefVal = ADCConvertedLSB[MEAS_CHAN_VREFIN-1];
#endif
        /* chcek FAN current */
        if(ADCConvertedLSB[MEAS_CHAN_FAN_CUR-1] > FAN_CUR_THRESHOLD){
            goto STARTUP_FAIL_ON_FAN_OC;
        }
        MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_LBRUSH, MOTOR_LBRUSH_CHAN_STARTUP_SPEED);
    }
    else if(3==gRobotStartupSeqNum){
#if defined REVISION_1_2
        _ADCRefVal = ADCConvertedLSB[MEAS_CHAN_VREFIN-1];
#endif
        /* chcek Lbrush current */
        if(ADCConvertedLSB[MEAS_CHAN_BRUSH_CUR_LEFT-1] > LBRUSH_CUR_THRESHOLD){
            goto STARTUP_FAIL_ON_LB_OC;
        }
        MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_RBRUSH, MOTOR_RBRUSH_CHAN_STARTUP_SPEED);
    }
    else if(4==gRobotStartupSeqNum){
#if defined REVISION_1_2
        _ADCRefVal = ADCConvertedLSB[MEAS_CHAN_VREFIN-1];
#endif
        /* chcek Rbrush current */
        if(ADCConvertedLSB[MEAS_CHAN_BRUSH_CUR_RIGHT-1] > RBRUSH_CUR_THRESHOLD){
            goto STARTUP_FAIL_ON_RB_OC;
        }
        MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_MBRUSH, MOTOR_MBRUSH_CHAN_STARTUP_SPEED);
    }
    else{
#if defined REVISION_1_2
        _ADCRefVal = ADCConvertedLSB[MEAS_CHAN_VREFIN-1];
#endif
        /* chcek Mbrush current */
        if(ADCConvertedLSB[MEAS_CHAN_BRUSH_CUR_MIDDLE-1] > MBRUSH_CUR_THRESHOLD){
            goto STARTUP_FAIL_ON_MB_OC;
        }

        switch(gRobotMode){
            case ROBOT_WORK_MODE_AUTO:
                plat_int_reg_cb(MOTION_MONITOR_TIM_INT_IDX, (void*)MotionCtrl_AutoMotionInit);
                break;
            case ROBOT_WORK_MODE_EDGE:
                plat_int_reg_cb(MOTION_MONITOR_TIM_INT_IDX, (void*)MotionCtrl_EdgeMotionInit);
                break;
            case ROBOT_WORK_MODE_SPOT:
                plat_int_reg_cb(MOTION_MONITOR_TIM_INT_IDX, (void*)MotionCtrl_SpotMotionInit);
                break;
        }
        gRobotStartupSeqNum = 0;
    }

    TIM_SetCounter(MOTION_MONITOR_TIM, 0);
    TIM_ITConfig(MOTION_MONITOR_TIM, TIM_IT_Update, DISABLE);
    TIM_SetAutoreload(MOTION_MONITOR_TIM, STARTUP_SEQ_DELAY_TIME[gRobotStartupSeqNum]);
    TIM_ClearFlag(MOTION_MONITOR_TIM, TIM_FLAG_Update);
    TIM_ITConfig(MOTION_MONITOR_TIM, TIM_IT_Update, ENABLE);
    TIM_Cmd(MOTION_MONITOR_TIM, ENABLE);
    return;

STARTUP_FAIL_ON_MB_OC:
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_MBRUSH, 0);
STARTUP_FAIL_ON_RB_OC:
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_RBRUSH, 0);
STARTUP_FAIL_ON_LB_OC:
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_LBRUSH, 0);
STARTUP_FAIL_ON_FAN_OC:
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_FAN, 0);
    TIM_Cmd(MOTION_MONITOR_TIM, DISABLE);
    TIM_SetCounter(MOTION_MONITOR_TIM, 0);
    plat_int_dereg_cb(MOTION_MONITOR_TIM_INT_IDX);
STARTUP_FAIL_ON_WF_AT:
    gRobotStartupSeqNum = 0;
    Buzzer_Play(BUZZER_TWO_PULS, BUZZER_SND_NORMAL);
    CtrlPanel_LEDCtrl(CTRL_PANEL_LED_RED, CTRL_PANEL_LED_BR_LVL);
    gRobotState = ROBOT_STATE_IDLE;
}

void SweepRobot_StartupComplete(void)
{
    TIM_Cmd(MOTION_MONITOR_TIM, DISABLE);
    TIM_SetCounter(MOTION_MONITOR_TIM, 0);

    plat_int_dereg_cb(MOTION_MONITOR_TIM_INT_IDX);
}

void SweepRobot_StartupAbort(void)
{
    gRobotStartupSeqNum = 0;

    TIM_Cmd(MOTION_MONITOR_TIM, DISABLE);
    TIM_SetCounter(MOTION_MONITOR_TIM, 0);

    plat_int_dereg_cb(MOTION_MONITOR_TIM_INT_IDX);

    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_FAN,    0);
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_MBRUSH, 0);
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_LBRUSH, 0);
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_RBRUSH, 0);

    gRobotState = ROBOT_STATE_IDLE;
}

void SweepRobot_Stop()
{
    MotionCtrl_Stop();

    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_FAN,    0);
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_MBRUSH, 0);
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_LBRUSH, 0);
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_RBRUSH, 0);

    gRobotState = ROBOT_STATE_IDLE;
}

void SweepRobot_AutoModeProc(void)
{
    if(gRobotState != ROBOT_STATE_RUNNING){
        if(gRobotState == ROBOT_STATE_STARTUP){
            SweepRobot_StartupAbort();
        }
        else if(gRobotState == ROBOT_STATE_HOME){
            MotionCtrl_DishomingMotionInit();
        }
        else{
            gRobotMode = ROBOT_WORK_MODE_AUTO;
            SweepRobot_StartupInit();
        }
    }
    else{
        if(gRobotMode!=ROBOT_WORK_MODE_DISHOMING){
            SweepRobot_Stop();
        }
    }
}

void SweepRobot_SpotModeProc(void)
{
    if((gRobotState == ROBOT_STATE_HOME) || (gRobotState == ROBOT_STATE_STARTUP)){
        return;
    }

    if(gRobotState != ROBOT_STATE_RUNNING){
        gRobotMode = ROBOT_WORK_MODE_SPOT;
        SweepRobot_StartupInit();
    }
    else{
        if(gRobotMode==ROBOT_WORK_MODE_SPOT){
            return;
        }
        MotionCtrl_Stop();

        MotionCtrl_SpotMotionInit();
    }
}

void SweepRobot_EdgeModeProc(void)
{
    if((gRobotState == ROBOT_STATE_HOME) || (gRobotState == ROBOT_STATE_STARTUP)){
        return;
    }

    if(gRobotState != ROBOT_STATE_RUNNING){
        gRobotMode = ROBOT_WORK_MODE_EDGE;
        SweepRobot_StartupInit();
    }
    else{
        if(gRobotMode==ROBOT_WORK_MODE_EDGE){
            return;
        }
        MotionCtrl_Stop();

        MotionCtrl_EdgeMotionInit();
    }
}

void SweepRobot_ManualModeProc(enum MotionCtrlManualAct act)
{
    if((gRobotState == ROBOT_STATE_HOME) || (gRobotState == ROBOT_STATE_STARTUP)){
        return;
    }

    if(gRobotState != ROBOT_STATE_RUNNING){
        MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_FAN,    MOTOR_FAN_CHAN_STARTUP_SPEED);
        MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_MBRUSH, MOTOR_MBRUSH_CHAN_STARTUP_SPEED);
        MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_LBRUSH, MOTOR_LBRUSH_CHAN_STARTUP_SPEED);
        MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_RBRUSH, MOTOR_RBRUSH_CHAN_STARTUP_SPEED);
    }
    else{
        MotionCtrl_Stop();
    }
    MotionCtrl_ManualCtrlProc(act);
}

void SweepRobot_HomingInit(void)
{
    if((gRobotState == ROBOT_STATE_HOME) || (gRobotState == ROBOT_STATE_STARTUP) || (gRobotState == ROBOT_STATE_RUNNING && gRobotMode == ROBOT_WORK_MODE_HOMING)){
        return;
    }

    if(gRobotState != ROBOT_STATE_RUNNING){
        MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_MBRUSH, 60);
        mDelay(50);
        MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_LBRUSH, 30);
        MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_RBRUSH, 30);
    }
    else{
        MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_FAN,    0);
        MotionCtrl_Stop();
    }
    MotionCtrl_HomingMotionInit();

    gHomingStage = ROBOT_HOMING_STAGE_UNKNOWN;
    LastHomingStage = ROBOT_HOMING_STAGE_UNKNOWN;
}

void SweepRobot_IdleStateSync(void)
{
    gRobotState = ROBOT_STATE_IDLE;

    if(gRobotMode==ROBOT_WORK_MODE_DISHOMING){
        SweepRobot_AutoModeProc();
    }
    else{
        MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_FAN,    0);
        MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_MBRUSH, 0);
        MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_LBRUSH, 0);
        MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_RBRUSH, 0);
    }
}

void SweepRobot_HomingSuccess(void)
{
    gRobotState = ROBOT_STATE_HOME;

    MotionCtrl_Stop();

    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_FAN,    0);
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_MBRUSH, 0);
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_LBRUSH, 0);
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_RBRUSH, 0);

    gHomingStage = ROBOT_HOMING_STAGE_UNKNOWN;
    LastHomingStage = ROBOT_HOMING_STAGE_UNKNOWN;
}

void SweepRobot_PMMsgProc(enum PM_Mode mode)
{
    switch(mode){

        case PM_MODE_RESUME:
            if(gRobotState==ROBOT_STATE_IDLE || gRobotState==ROBOT_STATE_HOME){
                SweepRobot_AutoModeProc();
            }
            break;
        case PM_MODE_STANDBY:
            if(gRobotState==ROBOT_STATE_IDLE||ROBOT_STATE_TEST){
                PM_EnterPwrMode(mode);
            }else{
                PM_ResetSysIdleState();
            }
            break;
        default:
            break;
    }
}

void SweepRobot_BMMsgProc(enum BatteryEvt evt)
{
    switch(evt){
        case BM_EVT_POWER_LOSS:
#ifdef DEBUG_LOG
            printf("Robot disconnect from power station.\r\n");
#endif
            if(gRobotState == ROBOT_STATE_HOME){
                gRobotState = ROBOT_STATE_IDLE;
//                CtrlPanel_LEDCtrl(CTRL_PANEL_LED_GREEN, CTRL_PANEL_LED_BR_LVL);
                /* attemp to clam to power station again */
                SweepRobot_HomingInit();
            }
            break;
        case BM_EVT_POWER_LINK:
#ifdef DEBUG_LOG
            printf("Robot connect to power station.\r\n");
#endif
            if(gRobotState == ROBOT_STATE_RUNNING && gRobotMode == ROBOT_WORK_MODE_HOMING){
                SweepRobot_HomingSuccess();
            }
            gRobotState = ROBOT_STATE_HOME;
            gRobotMode = ROBOT_WORK_MODE_HOMING;
            break;
        case BM_EVT_LOW_LEVEL:
            /* Low battery condition, try to home and get charged */
#ifdef DEBUG_LOG
            printf("Robot low battery condition.\r\n");
#endif
            if(gRobotState==ROBOT_STATE_RUNNING && gRobotMode != ROBOT_WORK_MODE_HOMING && gRobotMode != ROBOT_WORK_MODE_DISHOMING && gRobotMode != ROBOT_WORK_MODE_MANUAL){
                CtrlPanel_LEDCtrl(CTRL_PANEL_LED_RED, CTRL_PANEL_LED_BR_LVL);
                SweepRobot_HomingInit();
            }
            break;
        case BM_EVT_CRITICAL_LEVEL:
            /* Critical battery condition, failed to get charged then stop running */
#ifdef DEBUG_LOG
            printf("Robot critical battery condition.\r\n");
#endif
            if(gRobotState==ROBOT_STATE_RUNNING && gRobotMode != ROBOT_WORK_MODE_MANUAL){
                SweepRobot_Stop();
                CtrlPanel_LEDCtrl(CTRL_PANEL_LED_RED, CTRL_PANEL_LED_BR_LVL);
            }
            break;
        case BM_EVT_CHARGE_COMPLETE:
            CtrlPanel_LEDCtrl(CTRL_PANEL_LED_GREEN, CTRL_PANEL_LED_BR_LVL);
#ifdef DEBUG_LOG
            printf("Robot finish charging.\r\n");
#endif
            break;
    }
}

void SweepRobot_CtrlMsgProc(u8 CtrlCode)
{
    switch(CtrlCode){
        case REMOTE_CMD_RUN_STOP:
            SweepRobot_AutoModeProc();
            break;
        case REMOTE_CMD_SPOT:
            SweepRobot_SpotModeProc();
            break;
        case REMOTE_CMD_MODE_1:
            /* work mode switch */
            SweepRobot_EdgeModeProc();
            break;
        case REMOTE_CMD_MODE_2:
            /* work mode switch */
            break;
        case REMOTE_CMD_MODE_3:
            /* work mode switch */
            break;
        case REMOTE_CMD_MODE_4:
            /* work mode switch */
            break;
        case REMOTE_CMD_UP:
            SweepRobot_ManualModeProc(MANUAL_ACT_UP);
            break;
        case REMOTE_CMD_DOWN:
            SweepRobot_ManualModeProc(MANUAL_ACT_DOWN);
            break;
        case REMOTE_CMD_LEFT:
            SweepRobot_ManualModeProc(MANUAL_ACT_LEFT);
            break;
        case REMOTE_CMD_RIGHT:
            SweepRobot_ManualModeProc(MANUAL_ACT_RIGHT);
            break;
        case REMOTE_CMD_CHARGE:
            SweepRobot_HomingInit();
            break;
        case REMOTE_CMD_PLAN_MODE:
            RTC_AlarmCfgWindowStateSet();
            Buzzer_Play(BUZZER_ONE_PULS, BUZZER_SND_VERY_LONG);
            break;
        case REMOTE_CMD_PLAN_STATE:
            if(RTC_AlarmGet()){
                Buzzer_Play(BUZZER_ONE_PULS, BUZZER_SND_NORMAL);
            }
            break;
        case REMOTE_CMD_PLAN_CANCEL:
            if(RTC_AlarmCfgWindowStateGet()){
                RTC_AlarmCfgWindowReset();
                RTC_AlarmSet(0);
                Buzzer_Play(BUZZER_TWO_PULS, BUZZER_SND_NORMAL);
            }
            break;
        default:
            if(RTC_AlarmCfgWindowStateGet()){
                RTC_AlarmCfgWindowReset();
                if(CtrlCode >= REMOTE_CMD_PLAN_23P5H && CtrlCode <= REMOTE_CMD_PLAN_0P5H){
                    RTC_AlarmSet((REMOTE_CMD_PLAN_0P5H-CtrlCode+1)*1800);
                    Buzzer_Play(BUZZER_ONE_PULS, BUZZER_SND_LONG);
                }
            }
            break;
    }
}

void SweepRobot_MotionMsgProc(enum MotionEvt evt)
{
    switch(evt){
        case MOTION_EVT_PATH_FAULT:
            if(gRobotState == ROBOT_STATE_RUNNING){
                if(gRobotMode == ROBOT_WORK_MODE_MANUAL){
                    MotionCtrl_PathFaultProc(1);
                }
                else{
                    MotionCtrl_PathFaultProc(0);
                }
            }
            break;
        case MOTION_EVT_EXCEPTION:
#ifdef DEBUG_LOG
            printf("Exception state.\r\n");
#endif
            CtrlPanel_LEDCtrl(CTRL_PANEL_LED_RED, CTRL_PANEL_LED_BR_LVL);
            Buzzer_Play(BUZZER_TWO_PULS, BUZZER_SND_NORMAL);
            if(MotionCtrl_ExceptionHandle()){
                SweepRobot_Stop();
            }
            break;
        case MOTION_EVT_TRAPPED:
            MotionCtrl_TrapProc();
            break;
        case MOTION_EVT_STOP:
            SweepRobot_Stop();
            break;
        case MOTION_EVT_IDLE_SYNC:
            SweepRobot_IdleStateSync();
            break;
    }
}

#ifdef PWS_HW_VERSION_1_0
#define IS_PWR_STATION_BACKOFF_SIG(sig)     ((sig==PWR_STATION_BACKOFF_SIG_L)||(sig==PWR_STATION_BACKOFF_SIG_R))
#define IS_PWR_STATION_SIG_LONG(sig)        ((sig==PWR_STATION_HOME_SIG_LL)||(sig==PWR_STATION_HOME_SIG_RL))
#define IS_PWR_STATION_SIG_CENTER(sig)      (sig==PWR_STATION_HOME_SIG_CENTER)
#if 0
#define IS_PWR_STATION_SIG_SHORT(sig)       ((sig==PWR_STATION_HOME_SIG_LS)||(sig==PWR_STATION_HOME_SIG_RS))
#define IS_PWR_STATION_SIG_LEFT(sig)        ((sig==PWR_STATION_HOME_SIG_LL)||(sig==PWR_STATION_HOME_SIG_LS))
#define IS_PWR_STATION_SIG_RIGHT(sig)       ((sig==PWR_STATION_HOME_SIG_RL)||(sig==PWR_STATION_HOME_SIG_RS))
#else
#define IS_PWR_STATION_SIG_SHORT(sig)       ((sig==PWR_STATION_BACKOFF_SIG_R)||(sig==PWR_STATION_BACKOFF_SIG_L))
#define IS_PWR_STATION_SIG_LEFT(sig)        ((sig==PWR_STATION_HOME_SIG_LL)||(sig==PWR_STATION_BACKOFF_SIG_L))
#define IS_PWR_STATION_SIG_RIGHT(sig)       ((sig==PWR_STATION_HOME_SIG_RL)||(sig==PWR_STATION_BACKOFF_SIG_R))
#endif
#else /* Power Station HW Revision 1.1 */
#define IS_PWR_STATION_BACKOFF_SIG(sig)     ((sig==PWR_STATION_BACKOFF_SIG_L)||(sig==PWR_STATION_BACKOFF_SIG_R))
#define IS_PWR_STATION_SIG_LONG(sig)        ((sig==PWR_STATION_HOME_SIG_LL)||(sig==PWR_STATION_HOME_SIG_RL))
#define IS_PWR_STATION_SIG_SHORT(sig)       ((sig==PWR_STATION_HOME_SIG_LS)||(sig==PWR_STATION_HOME_SIG_RS))
#define IS_PWR_STATION_SIG_CENTER(sig)      (sig==PWR_STATION_HOME_SIG_CENTER)
#define IS_PWR_STATION_SIG_LEFT(sig)        ((sig==PWR_STATION_HOME_SIG_LL)||(sig==PWR_STATION_HOME_SIG_LS))
#define IS_PWR_STATION_SIG_RIGHT(sig)       ((sig==PWR_STATION_HOME_SIG_RL)||(sig==PWR_STATION_HOME_SIG_RS))
#endif

void SweepRobot_PwrStationMsgProc(PwrStationSigData_t *PwrSig)
{
    static PwrStationSigData_t RobotHomingData[10];
    static u8 i, j, k, l, HomingDataCnt = 0, RepeatCodeFound = 0, HomingStateConfirmCnt = 0;
    static struct RobotHomingState_s HomingState, LastHomingState = {0};

    if(gRobotState != ROBOT_STATE_RUNNING){
        return;
    }

    if(gRobotMode == ROBOT_WORK_MODE_HOMING){
        /* Ignore backoff signal */
        if(IS_PWR_STATION_BACKOFF_SIG(PwrSig->sig)){
            return;
        }
        for(i = 0; i < HomingDataCnt; i++){
            if(RobotHomingData[i].sig == PwrSig->sig && RobotHomingData[i].src == PwrSig->src){
                break;
            }
        }

        if(i < HomingDataCnt){
            /* all received signal in one loop */
            l = HomingDataCnt-i;
            if(l >= 2){
                /* find repeative code */
                for(j = 0; j < l; j++){
                    if(RepeatCodeFound)
                        break;
                    for(k = j+1; k < l; k++){
                        if(RobotHomingData[j].sig == RobotHomingData[k].sig){
                            HomingState.Pos = RobotHomingData[j].sig;
                            HomingState.Angle = (HomingSigSrc2AngleMap[RobotHomingData[j].src] + HomingSigSrc2AngleMap[RobotHomingData[k].src]);
                            HomingState.Angle = (HomingState.Angle == 360) ? 0 : (HomingState.Angle / 2);
                            RepeatCodeFound = 1;
                            break;
                        }
                    }
                }
                /* priority */
                if(RepeatCodeFound){
                    /* Aleady get foucs */
                    if(IS_PWR_STATION_SIG_LONG(HomingState.Pos)){
                        /* Last postion is stage2(short distance area) */
                        if(LastHomingStage==ROBOT_HOMING_STAGE_UNKNOWN||LastHomingStage==ROBOT_HOMING_STAGE2){
                            for(i = 0; i < l; i++){
                                if(IS_PWR_STATION_SIG_SHORT(RobotHomingData[i].sig)){
                                    /* found short signal */
                                    break;
                                }
                            }
                            if(i>=l){
                                for(i = 0; i < l; i++){
                                    if(IS_PWR_STATION_SIG_CENTER(RobotHomingData[i].sig)){
                                        /* found center signal */
                                        break;
                                    }
                                }
                            }
                            if(i<l){
                                HomingState.Pos = RobotHomingData[i].sig;
                                HomingState.Angle = HomingSigSrc2AngleMap[RobotHomingData[i].src];
                            }
                        }
                        /* come from stage1 or stage3, stage ok */
                        else {
                            for(i = 0; i < l; i++){
                                if(IS_PWR_STATION_SIG_CENTER(RobotHomingData[i].sig)){
                                    /* found center signal */
                                    break;
                                }
                            }
                            if(i>=l){
                                for(i = 0; i < l; i++){
                                    if(IS_PWR_STATION_SIG_SHORT(RobotHomingData[i].sig)){
                                        /* found short signal */
                                        break;
                                    }
                                }
                            }
                            if(i<l){
                                HomingState.Pos = RobotHomingData[i].sig;
                                HomingState.Angle = HomingSigSrc2AngleMap[RobotHomingData[i].src];
                            }
                        }
                    }
                }
                else{
                    if(LastHomingStage==ROBOT_HOMING_STAGE_UNKNOWN||LastHomingStage==ROBOT_HOMING_STAGE2){
                        for(i = 0; i < l; i++){
                            if(IS_PWR_STATION_SIG_SHORT(RobotHomingData[i].sig)){
                                /* found short signal */
                                break;
                            }
                        }
                        if(i>=l){
                            for(i = 0; i < l; i++){
                                if(IS_PWR_STATION_SIG_CENTER(RobotHomingData[i].sig)){
                                    /* found center signal */
                                    break;
                                }
                            }
                        }
                        if(i<l){
                            HomingState.Pos = RobotHomingData[i].sig;
                            HomingState.Angle = HomingSigSrc2AngleMap[RobotHomingData[i].src];
                        }
                        else{
                            HomingState.Pos = RobotHomingData[0].sig;
                            HomingState.Angle = HomingSigSrc2AngleMap[RobotHomingData[0].src];
                        }
                    }
                    /* come from stage1 or stage3, stage ok */
                    else {
                        for(i = 0; i < l; i++){
                            if(IS_PWR_STATION_SIG_CENTER(RobotHomingData[i].sig)){
                                /* found center signal */
                                break;
                            }
                        }
                        if(i>=l){
                            for(i = 0; i < l; i++){
                                if(IS_PWR_STATION_SIG_SHORT(RobotHomingData[i].sig)){
                                    /* found short signal */
                                    break;
                                }
                            }
                        }
                        if(i<l){
                            HomingState.Pos = RobotHomingData[i].sig;
                            HomingState.Angle = HomingSigSrc2AngleMap[RobotHomingData[i].src];
                        }
                        else{
                            HomingState.Pos = RobotHomingData[0].sig;
                            HomingState.Angle = HomingSigSrc2AngleMap[RobotHomingData[0].src];
                        }
                    }
                }
            }
            else {
                HomingState.Pos = RobotHomingData[HomingDataCnt-1].sig;
                HomingState.Angle = HomingSigSrc2AngleMap[RobotHomingData[HomingDataCnt-1].src];
            }

            HomingState.Angle = (HomingState.Angle > 180) ? (HomingState.Angle - 360) : HomingState.Angle;
#ifdef DEBUG_LOG
            printf("Pos: 0x%X, Ang: %d\r\n", HomingState.Pos, HomingState.Angle);
#endif
            HomingDataCnt = 0;
            RepeatCodeFound = 0;
        }

        if(HomingDataCnt >= 5){
            HomingDataCnt = 0;
        }

        RobotHomingData[HomingDataCnt].sig = PwrSig->sig;
        RobotHomingData[HomingDataCnt].src = PwrSig->src;
        HomingDataCnt++;

//        if(LastHomingState.Pos!=HomingState.Pos){
//            HomingStateConfirmCnt++;
//            if(HomingStateConfirmCnt>5){
//                LastHomingState.Pos = HomingState.Pos;
//                LastHomingState.Angle = HomingState.Angle;
//            }
//            else{
//                HomingState.Pos = LastHomingState.Pos;
//                HomingState.Angle = LastHomingState.Angle;
//            }
//        }

        /* plan path to power station */
        if(IS_PWR_STATION_SIG_LONG(HomingState.Pos)){
            gHomingStage = ROBOT_HOMING_STAGE1;
            if(IS_PWR_STATION_SIG_LEFT(HomingState.Pos)){
                if(HomingState.Angle < -90){
                    MotionCtrl_MoveDirTune(0, 1, WHEEL_HOMING_SPEED, 1);
                }
                else if(HomingState.Angle < 90){
                    MotionCtrl_MoveDirTune(WHEEL_HOMING_SPEED, 1, WHEEL_HOMING_SPEED/5, 1);
                }
                else if(HomingState.Angle == 90){
                    MotionCtrl_MoveDirTune(4*WHEEL_HOMING_SPEED/5, 1, WHEEL_HOMING_SPEED, 1);
                }
                else{
                    MotionCtrl_MoveDirTune(0, 1, WHEEL_HOMING_SPEED, 1);
                }
            }
            else {
                if(HomingState.Angle > 90){
                    MotionCtrl_MoveDirTune(WHEEL_HOMING_SPEED, 1, 0, 1);
                }
                else if(HomingState.Angle > -90){
                    MotionCtrl_MoveDirTune(WHEEL_HOMING_SPEED/5, 1, WHEEL_HOMING_SPEED, 1);
                }
                else if(HomingState.Angle == -90){
                    MotionCtrl_MoveDirTune(WHEEL_HOMING_SPEED, 1, 4*WHEEL_HOMING_SPEED/5, 1);
                }
                else{
                    MotionCtrl_MoveDirTune(WHEEL_HOMING_SPEED, 1, 0, 1);
                }
            }
        }
        else if(IS_PWR_STATION_SIG_SHORT(HomingState.Pos)){
            gHomingStage = ROBOT_HOMING_STAGE2;
            if(IS_PWR_STATION_SIG_LEFT(HomingState.Pos)){
                if(HomingState.Angle <= 0 && HomingState.Angle >= -135){
                    MotionCtrl_MoveDirTune(0, 1, WHEEL_HOMING_SPEED, 1);
                }
                else if(HomingState.Angle > 0 && HomingState.Angle < 180){
                    MotionCtrl_MoveDirTune(WHEEL_HOMING_SPEED, 1, 0, 1);
                }
                else {
                    MotionCtrl_MoveDirTune(WHEEL_HOMING_SPEED, 1, WHEEL_HOMING_SPEED, 1);
                }
            }
            else{
                if(HomingState.Angle <= 0 && HomingState.Angle >= -135){
                    MotionCtrl_MoveDirTune(0, 1, WHEEL_HOMING_SPEED, 1);
                }
                else if(HomingState.Angle > 0 && HomingState.Angle < 180){
                    MotionCtrl_MoveDirTune(WHEEL_HOMING_SPEED, 1, 0, 1);
                }
                else {
                    MotionCtrl_MoveDirTune(WHEEL_HOMING_SPEED, 1, WHEEL_HOMING_SPEED, 1);
                }
            }
        }
        /* Center area */
        else if(PWR_STATION_HOME_SIG_CENTER==HomingState.Pos){
            gHomingStage = ROBOT_HOMING_STAGE3;
            if(HomingState.Angle > 45){
                MotionCtrl_MoveDirTune(WHEEL_HOMING_SPEED/10, 0, WHEEL_HOMING_SPEED/10, 1);
            }
            else if(HomingState.Angle > 0){
                MotionCtrl_MoveDirTune(WHEEL_HOMING_SPEED/5, 1, 3*WHEEL_HOMING_SPEED/10, 1);
            }
            else if(HomingState.Angle < -45){
                MotionCtrl_MoveDirTune(WHEEL_HOMING_SPEED/10, 1, WHEEL_HOMING_SPEED/10, 0);
            }
            else if(HomingState.Angle < 0){
                MotionCtrl_MoveDirTune(3*WHEEL_HOMING_SPEED/10, 1, WHEEL_HOMING_SPEED/5, 1);
            }
            else{
                gHomingStage = ROBOT_HOMING_STAGE_OK;
                MotionCtrl_MoveDirTune(3*WHEEL_HOMING_SPEED/5, 1, 3*WHEEL_HOMING_SPEED/5, 1);
            }
        }
#ifdef DEBUG_HOME
        if(gHomingStage!=LastHomingStage) {
            if(gHomingStage==ROBOT_HOMING_STAGE_OK){
                CtrlPanel_LEDCtrl(CTRL_PANEL_LED_RED, CTRL_PANEL_LED_BR_LVL);
            }
            else{
                CtrlPanel_LEDCtrl(CTRL_PANEL_LED_BLUE, CTRL_PANEL_LED_BR_LVL);
            }
        }
#endif
        LastHomingStage = gHomingStage;
    }
    else {
        if( IS_MOTION_PROC_FINISH() && IS_PWR_STATION_BACKOFF_SIG(PwrSig->sig) ){
            if( (PwrSig->src==IRDA_RECV_POS_L || PwrSig->src==IRDA_RECV_POS_FL) ){
                if(gRobotMode == ROBOT_WORK_MODE_MANUAL){
                    MotionCtrl_ChargeStationAvoid(1, WHEEL_TURN_30_CNT, 1);
                }
                else{
                    MotionCtrl_ChargeStationAvoid(1, WHEEL_TURN_30_CNT, 0);
                }
            }
            else if( (PwrSig->src==IRDA_RECV_POS_R || PwrSig->src==IRDA_RECV_POS_FR) ){
                if(gRobotMode == ROBOT_WORK_MODE_MANUAL){
                    MotionCtrl_ChargeStationAvoid(0, WHEEL_TURN_30_CNT, 1);
                }
                else{
                    MotionCtrl_ChargeStationAvoid(0, WHEEL_TURN_30_CNT, 0);
                }
            }
        }
    }
}

s8 SweepRobot_SendMsg(Msg_t *Msg)
{
    if(MsgQueue_InQueue(MainMsgQ, ROBOT_MAIN_MSG_Q_SIZE, Msg)){
#ifdef DEBUG_LOG
        printf("Msg inqueue failed !\r\n");
#endif
        return -1;
    }
    return 0;
}
