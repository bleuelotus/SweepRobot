/******************** (C) COPYRIGHT 2007 EJE ********************
* File Name          : SweepRobotTest.c
* Author             : Meredith Rowe
* Version            : V1.0
* Date               : 15-Nov-2015
* Description        : Sweeping Robot Test
*******************************************************************************/
#include "SweepRobotTest.h"
#include "stm32f10x_it.h"

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

#include <stdlib.h>
#include <string.h>

#define SWRB_TEST_SN_YEAR_REG       BKP_DR8
#define SWRB_TEST_SN_MONTH_REG      BKP_DR9
#define SWRB_TEST_SN_DATE_REG       BKP_DR10
#define SWRB_TEST_SN_SNUM_REG       BKP_DR11

#define SWRB_TEST_SN_FLASH_PAGE         0x00
#define SWRB_TEST_SN_YEAR_FLASH_ADDR    0x00
#define SWRB_TEST_SN_MONTH_FLASH_ADDR   0x02
#define SWRB_TEST_SN_DATE_FLASH_ADDR    0x04
#define SWRB_TEST_SN_SNUM_FLASH_ADDR    0x06

#define KEY_SIGN            GPIO_ReadInputDataBit(CTRL_BTN_ALL_IN_ONE_GPIO, CTRL_BTN_ALL_IN_ONE_PIN)
#define CHARGE_24V_SIGN     GPIO_ReadInputDataBit(BM_CHARGE_SW_STATUS_GPIO, BM_CHARGE_SW_STATUS_PIN)

#define ADC_BAT_SAMPLE_AVE_CNT              5
#define ADC_BAT_CHANNEL_NUM                 3

#define STDIO_UART_RX_CMD_BUF_LEN        20
#define STDIO_UART_RX_CMD_ACT_BUF_LEN    20

#define IS_CHARGE_CONNECTED()               GPIO_ReadInputDataBit(BM_CHARGE_SW_STATUS_GPIO, BM_CHARGE_SW_STATUS_PIN)

enum MotorSelect{
    MOTOR_SELECT_L,
    MOTOR_SELECT_R,
};

enum MotorDir{
    MOTOR_CTRL_DIR_BACKWARD,
    MOTOR_CTRL_DIR_FORWARD,
};

enum MEAS_IFRD_BOTTOM_CHAN{
    MEAS_IFRD_BOTTOM_CHAN_FRONT,
    MEAS_IFRD_BOTTOM_CHAN_SIDE,
};

static uint16_t aSwrbTestData[SWRB_TEST_DATA_BOUND] = { 0 };

static u8 gSwrbTestIrDARxCode[IRDA_LIGHT_NUM] = {0x00};

static s8 LedBrightnessSch = 0, LedBrightnessDir = 1;

static u16 TempADC[ADC_BAT_CHANNEL_NUM][ADC_BAT_SAMPLE_AVE_CNT] = {0};
static u8 BM_StateInited = 0;
static u32 ADC_BatLSB[ADC_BAT_CHANNEL_NUM] = {0};

enum ADC_BAT_CHAN{
    ADC_BAT_VOL,
    ADC_BAT_CUR,
    ADC_BAT_INTVOL,
};
#define BM_CHARGE_CUR_50MA                  0.025f
#define BM_CHARGE_CUR_100MA                 0.05f
#define BM_CHARGE_CUR_600MA                 0.3f
#define BM_CHARGE_CUR_1000MA                0.5f
static float ADC2Value_BatLSB[ADC_BAT_CHANNEL_NUM] = {0.f};

u16 UsartRxState = 0;
static char UsartRxCmdBuf[STDIO_UART_RX_CMD_BUF_LEN];
static char UsartRxCmdActBuf[STDIO_UART_RX_CMD_ACT_BUF_LEN];
static u16 UsartRxCmdLen, UsartRxCmdActLen;
static int UsartRxCmdParaBuf;

void USART_TestCtrlCmdSend(enum USARTTestCtrlCmd cmd, enum USARTTestCtrlCmdAct cmd_act, int cmd_para);

extern void BM_ConditionUpdate(void);
extern void BM_TestConditionUpdate(void);
extern s8 BM_ChargePowerInc(void);
extern s8 BM_ChargePowerDec(void);

extern void WheelCntMach_TestStart(void);
extern void WheelCntMach_TestStop(void);
extern void IFRD_TestPathDetectStart(void);
extern void IFRD_TestPathDetectStop(void);
extern u16 MotionCtrl_ChanSpeedGet(u8 Wheel_Idx);


/* TEST BM process */
void BM_ChargeTestStart(USARTTestCtrlData_t *testCtrlData)
{
    TIM_ClearFlag(BAT_MONITOR_TIM, TIM_IT_Update);
    plat_int_reg_cb(BAT_MONITOR_TIM_INT_IDX, (void*)BM_TestConditionUpdate);
    TIM_Cmd(BAT_MONITOR_TIM, ENABLE);
    if(testCtrlData->Cmd_Para){
        PWM_DutyCycleSet(PWM_CHAN_CHARGE, testCtrlData->Cmd_Para);
    }
}

void BM_ChargeTestStop(void)
{
    PWM_DutyCycleSet(PWM_CHAN_CHARGE, 0);
    TIM_Cmd(BAT_MONITOR_TIM, DISABLE);
    TIM_ClearFlag(BAT_MONITOR_TIM, TIM_IT_Update);
    CtrlPanel_LEDCtrl(CTRL_PANEL_LED_BLUE, CTRL_PANEL_LED_BR_LVL);
}

/* TEST USART process */
void UART4_ISR(void)
{
    u8 ch;
    
    if(USART_GetITStatus(STDIO_UART, USART_IT_RXNE) != RESET){
        ch = USART_ReceiveData(STDIO_UART);
        if( !(UsartRxState & (1<<USART_RX_STATE_REC_FINISH_POS) ) ){
            if( UsartRxState & (1<<USART_RX_STATE_CR_REC_POS) ){
                if(ch != '\n')
                    UsartRxState = 0;
                else{
                    UsartRxState |= (1<<USART_RX_STATE_REC_FINISH_POS);
                }
            }else{
                if(ch == '\r')
                    UsartRxState |= (1<<USART_RX_STATE_CR_REC_POS);
                else{
                    if( !(UsartRxState & (1<<USART_RX_STATE_CMD_PARA_REC_EN_POS) ) ){
                        if( ch == '='){
                            UsartRxState |= (1<<USART_RX_STATE_CMD_PARA_REC_EN_POS);
                            UsartRxState &= USART_RX_STATE_CNT_CLR_MASK;
                            return;
                        }
                        if( !(UsartRxState & (1<<USART_RX_STATE_CMD_ACT_REC_EN_POS) ) ){
                            if( !(UsartRxState & (1<<USART_RX_STATE_DASH_REC_POS)) ){
                                if( ch == '-'){
                                    UsartRxState |= (1<<USART_RX_STATE_DASH_REC_POS);
                                    return;
                                }
                                if( (('a' <= ch) && ('z' >= ch)) || (('A' <= ch) && ('Z' >= ch)) || ('_' == ch) ){
                                    UsartRxCmdBuf[UsartRxState&USART_RX_STATE_CNT_MASK] = ch;
                                    UsartRxState++;
                                    UsartRxCmdLen = UsartRxState&USART_RX_STATE_CNT_MASK;
                                    if( (UsartRxState&USART_RX_STATE_CNT_MASK) >  STDIO_UART_RX_CMD_BUF_LEN){
                                        UsartRxState = 0;
                                        return;
                                    }
                                }else{
                                    UsartRxState = 0;
                                    return;
                                }
                            }else{
                                if( ch == '>'){
                                    UsartRxState |= (1<<USART_RX_STATE_CMD_ACT_REC_EN_POS);
                                    UsartRxState &= USART_RX_STATE_CNT_CLR_MASK;
                                }else{
                                    UsartRxState = 0;
                                    return;
                                }
                            }
                        }else{
                            if( (('a' <= ch) && ('z' >= ch)) || (('A' <= ch) && ('Z' >= ch)) || ('_' == ch) ){
                                UsartRxCmdActBuf[UsartRxState&USART_RX_STATE_CNT_MASK] = ch;
                                UsartRxState++;
                                UsartRxCmdActLen = UsartRxState&USART_RX_STATE_CNT_MASK;
                                if( (UsartRxState&USART_RX_STATE_CNT_MASK) >  STDIO_UART_RX_CMD_BUF_LEN){
                                    UsartRxState = 0;
                                    return;
                                }
                            }else{
                                UsartRxState = 0;
                                return;
                            }
                        }
                    }else{
                        if( ('0' <= ch) && ('9' >= ch) ){
                            if(UsartRxState&USART_RX_STATE_CNT_MASK){
                                UsartRxCmdParaBuf *= 10;
                            }
                            UsartRxCmdParaBuf += (ch - '0');
                            UsartRxState++;
                        }else{
                            UsartRxState = 0;
                            return;
                        }
                    }
                }
            }
        }
    }
}

s8 USART_ArrayTostring(char *src_array, char* *dest_str, u16 cmd_len)
{
    *dest_str = (char *)malloc(sizeof(char)*(cmd_len+1) );
    
    if(NULL==(*dest_str) )
        return -1;
    
    memset(*dest_str, 0, sizeof(char)*(cmd_len+1));
    strncpy(*dest_str, src_array, cmd_len);
    
    return 0;
}

static void USART_CmdProcFinishProc(void)
{
    UsartRxState = 0;
    UsartRxCmdParaBuf = 0;
}

void USART_CmdProc(void)
{
    enum USARTTestCtrlCmd cmd;
    enum USARTTestCtrlCmdAct cmd_act;
    
    char *UsartRxCmdStrBuf = NULL;
    char *UsartRxCmdActStrBuf = NULL;
    
    if( USART_ArrayTostring(UsartRxCmdBuf, &UsartRxCmdStrBuf, UsartRxCmdLen) )
        return;
#ifdef __USE_FULL_INS
    if(       !(strcmp(UsartRxCmdStrBuf, "TEST")) ){
        cmd = TEST_CTRL_CMD_TEST;
    }else if( !(strcmp(UsartRxCmdStrBuf, "MANUL")) ){
        cmd = TEST_CTRL_CMD_MANUL;
    }else if( !(strcmp(UsartRxCmdStrBuf, "WHEEL")) ){
        cmd = TEST_CTRL_CMD_WHEEL;
    }else if( !(strcmp(UsartRxCmdStrBuf, "LWHEEL")) ){
        cmd = TEST_CTRL_CMD_LWHEEL;
    }else if( !(strcmp(UsartRxCmdStrBuf, "RWHEEL")) ){
        cmd = TEST_CTRL_CMD_RWHEEL;
    }else if( !(strcmp(UsartRxCmdStrBuf, "BRUSH")) ){
        cmd = TEST_CTRL_CMD_BRUSH;
    }else if( !(strcmp(UsartRxCmdStrBuf, "LBRUSH")) ){
        cmd = TEST_CTRL_CMD_LBRUSH;
    }else if( !(strcmp(UsartRxCmdStrBuf, "RBRUSH")) ){
        cmd = TEST_CTRL_CMD_RBRUSH;
    }else if( !(strcmp(UsartRxCmdStrBuf, "MBRUSH")) ){
        cmd = TEST_CTRL_CMD_MBRUSH;
    }else if( !(strcmp(UsartRxCmdStrBuf, "FAN")) ){
        cmd = TEST_CTRL_CMD_FAN;
    }else if( !(strcmp(UsartRxCmdStrBuf, "SENSOR")) ){
        cmd = TEST_CTRL_CMD_SENSOR;
    }else if( !(strcmp(UsartRxCmdStrBuf, "COLLISION")) ){
        cmd = TEST_CTRL_CMD_COLLISION;
    }else if( !(strcmp(UsartRxCmdStrBuf, "WHEEL_FLOAT")) ){
        cmd = TEST_CTRL_CMD_WHEEL_FLOAT;
    }else if( !(strcmp(UsartRxCmdStrBuf, "ASH_TRAY")) ){
        cmd = TEST_CTRL_CMD_ASH_TRAY;
    }else if( !(strcmp(UsartRxCmdStrBuf, "RGB_LED")) ){
        cmd = TEST_CTRL_CMD_RGB_LED;
    }else if( !(strcmp(UsartRxCmdStrBuf, "KEY")) ){
        cmd = TEST_CTRL_CMD_KEY;
    }else if( !(strcmp(UsartRxCmdStrBuf, "IRDA")) ){
        cmd = TEST_CTRL_CMD_IRDA;
    }else if( !(strcmp(UsartRxCmdStrBuf, "BUZZER")) ){
        cmd = TEST_CTRL_CMD_BUZZER;
    }else if( !(strcmp(UsartRxCmdStrBuf, "CHARGE")) ){
        cmd = TEST_CTRL_CMD_CHARGE;
    }else if( !(strcmp(UsartRxCmdStrBuf, "POWER_STATION")) ){
        cmd = TEST_CTRL_CMD_POWER_STATION;
    }else if( !(strcmp(UsartRxCmdStrBuf, "SN_READ")) ){
        cmd = TEST_CTRL_CMD_SN_READ;
    }else if( !(strcmp(UsartRxCmdStrBuf, "SN_WRITE")) ){
        cmd = TEST_CTRL_CMD_SN_WRITE;
    }else{
        free(UsartRxCmdStrBuf);
        UsartRxCmdStrBuf = NULL;
        USART_CmdProcFinishProc();
        return;
    }
#else
    if(       !(strcmp(UsartRxCmdStrBuf, "T")) ){
        cmd = TEST_CTRL_CMD_TEST;
    }else if( !(strcmp(UsartRxCmdStrBuf, "MNL")) ){
        cmd = TEST_CTRL_CMD_MANUL;
    }else if( !(strcmp(UsartRxCmdStrBuf, "WHL")) ){
        cmd = TEST_CTRL_CMD_WHEEL;
    }else if( !(strcmp(UsartRxCmdStrBuf, "LW")) ){
        cmd = TEST_CTRL_CMD_LWHEEL;
    }else if( !(strcmp(UsartRxCmdStrBuf, "RW")) ){
        cmd = TEST_CTRL_CMD_RWHEEL;
    }else if( !(strcmp(UsartRxCmdStrBuf, "BRS")) ){
        cmd = TEST_CTRL_CMD_BRUSH;
    }else if( !(strcmp(UsartRxCmdStrBuf, "LB")) ){
        cmd = TEST_CTRL_CMD_LBRUSH;
    }else if( !(strcmp(UsartRxCmdStrBuf, "RB")) ){
        cmd = TEST_CTRL_CMD_RBRUSH;
    }else if( !(strcmp(UsartRxCmdStrBuf, "MB")) ){
        cmd = TEST_CTRL_CMD_MBRUSH;
    }else if( !(strcmp(UsartRxCmdStrBuf, "FAN")) ){
        cmd = TEST_CTRL_CMD_FAN;
    }else if( !(strcmp(UsartRxCmdStrBuf, "SNSR")) ){
        cmd = TEST_CTRL_CMD_SENSOR;
    }else if( !(strcmp(UsartRxCmdStrBuf, "CLSN")) ){
        cmd = TEST_CTRL_CMD_COLLISION;
    }else if( !(strcmp(UsartRxCmdStrBuf, "WF")) ){
        cmd = TEST_CTRL_CMD_WHEEL_FLOAT;
    }else if( !(strcmp(UsartRxCmdStrBuf, "AT")) ){
        cmd = TEST_CTRL_CMD_ASH_TRAY;
    }else if( !(strcmp(UsartRxCmdStrBuf, "RGB")) ){
        cmd = TEST_CTRL_CMD_RGB_LED;
    }else if( !(strcmp(UsartRxCmdStrBuf, "KEY")) ){
        cmd = TEST_CTRL_CMD_KEY;
    }else if( !(strcmp(UsartRxCmdStrBuf, "IRDA")) ){
        cmd = TEST_CTRL_CMD_IRDA;
    }else if( !(strcmp(UsartRxCmdStrBuf, "BZR")) ){
        cmd = TEST_CTRL_CMD_BUZZER;
    }else if( !(strcmp(UsartRxCmdStrBuf, "CRG")) ){
        cmd = TEST_CTRL_CMD_CHARGE;
    }else if( !(strcmp(UsartRxCmdStrBuf, "PS")) ){
        cmd = TEST_CTRL_CMD_POWER_STATION;
    }else if( !(strcmp(UsartRxCmdStrBuf, "SNR")) ){
        cmd = TEST_CTRL_CMD_SN_READ;
    }else if( !(strcmp(UsartRxCmdStrBuf, "SNW")) ){
        cmd = TEST_CTRL_CMD_SN_WRITE;
    }else{
        free(UsartRxCmdStrBuf);
        UsartRxCmdStrBuf = NULL;
        USART_CmdProcFinishProc();
        return;
    }
#endif
    free(UsartRxCmdStrBuf);
    UsartRxCmdStrBuf = NULL;
    
    if( USART_ArrayTostring(UsartRxCmdActBuf, &UsartRxCmdActStrBuf, UsartRxCmdActLen) )
        return;
#ifdef __USE_FULL_INS
    if(       !(strcmp(UsartRxCmdActStrBuf, "ON")) ){
        cmd_act = TEST_CTRL_CMD_ACT_ON;
    }else if( !(strcmp(UsartRxCmdActStrBuf, "OFF")) ){
        cmd_act = TEST_CTRL_CMD_ACT_OFF;
    }else if( !(strcmp(UsartRxCmdActStrBuf, "WRITE")) ){
        cmd_act = TEST_CTRL_CMD_ACT_WRITE;
    }else if( !(strcmp(UsartRxCmdActStrBuf, "READ")) ){
        cmd_act = TEST_CTRL_CMD_ACT_READ;
    }else if( !(strcmp(UsartRxCmdActStrBuf, "DIR")) ){
        cmd_act = TEST_CTRL_CMD_ACT_DIR;
    }else if( !(strcmp(UsartRxCmdActStrBuf, "SPEED")) ){
        cmd_act = TEST_CTRL_CMD_ACT_SPEED;
    }else if( !(strcmp(UsartRxCmdActStrBuf, "IFRD_LED")) ){
        cmd_act = TEST_CTRL_CMD_ACT_IFRD_LED;
    }else if( !(strcmp(UsartRxCmdActStrBuf, "B_SWITCH")) ){
        cmd_act = TEST_CTRL_CMD_ACT_B_SWITCH;
    }else if( !(strcmp(UsartRxCmdActStrBuf, "START")) ){
        cmd_act = TEST_CTRL_CMD_ACT_START;
    }else if( !(strcmp(UsartRxCmdActStrBuf, "STOP")) ){
        cmd_act = TEST_CTRL_CMD_ACT_STOP;
    }else if( !(strcmp(UsartRxCmdActStrBuf, "YEAR")) ){
        cmd_act = TEST_CTRL_CMD_ACT_YEAR;
    }else if( !(strcmp(UsartRxCmdActStrBuf, "MONTH")) ){
        cmd_act = TEST_CTRL_CMD_ACT_MONTH;
    }else if( !(strcmp(UsartRxCmdActStrBuf, "DATE")) ){
        cmd_act = TEST_CTRL_CMD_ACT_DATE;
    }else if( !(strcmp(UsartRxCmdActStrBuf, "SN")) ){
        cmd_act = TEST_CTRL_CMD_ACT_SN;
    }else if( !(strcmp(UsartRxCmdActStrBuf, "ALL")) ){
        cmd_act = TEST_CTRL_CMD_ACT_ALL;
    }else if( !(strcmp(UsartRxCmdActStrBuf, "ERASE")) ){
        cmd_act = TEST_CTRL_CMD_ACT_ERASE;
    }else{
        free(UsartRxCmdActStrBuf);
        UsartRxCmdActStrBuf = NULL;
        USART_CmdProcFinishProc();
        return;
    }
#else
    if(       !(strcmp(UsartRxCmdActStrBuf, "ON")) ){
        cmd_act = TEST_CTRL_CMD_ACT_ON;
    }else if( !(strcmp(UsartRxCmdActStrBuf, "OFF")) ){
        cmd_act = TEST_CTRL_CMD_ACT_OFF;
    }else if( !(strcmp(UsartRxCmdActStrBuf, "WR")) ){
        cmd_act = TEST_CTRL_CMD_ACT_WRITE;
    }else if( !(strcmp(UsartRxCmdActStrBuf, "RD")) ){
        cmd_act = TEST_CTRL_CMD_ACT_READ;
    }else if( !(strcmp(UsartRxCmdActStrBuf, "DIR")) ){
        cmd_act = TEST_CTRL_CMD_ACT_DIR;
    }else if( !(strcmp(UsartRxCmdActStrBuf, "SPD")) ){
        cmd_act = TEST_CTRL_CMD_ACT_SPEED;
    }else if( !(strcmp(UsartRxCmdActStrBuf, "IFRD")) ){
        cmd_act = TEST_CTRL_CMD_ACT_IFRD_LED;
    }else if( !(strcmp(UsartRxCmdActStrBuf, "BSWC")) ){
        cmd_act = TEST_CTRL_CMD_ACT_B_SWITCH;
    }else if( !(strcmp(UsartRxCmdActStrBuf, "STRT")) ){
        cmd_act = TEST_CTRL_CMD_ACT_START;
    }else if( !(strcmp(UsartRxCmdActStrBuf, "STOP")) ){
        cmd_act = TEST_CTRL_CMD_ACT_STOP;
    }else if( !(strcmp(UsartRxCmdActStrBuf, "YEAR")) ){
        cmd_act = TEST_CTRL_CMD_ACT_YEAR;
    }else if( !(strcmp(UsartRxCmdActStrBuf, "MNTH")) ){
        cmd_act = TEST_CTRL_CMD_ACT_MONTH;
    }else if( !(strcmp(UsartRxCmdActStrBuf, "DATE")) ){
        cmd_act = TEST_CTRL_CMD_ACT_DATE;
    }else if( !(strcmp(UsartRxCmdActStrBuf, "SN")) ){
        cmd_act = TEST_CTRL_CMD_ACT_SN;
    }else if( !(strcmp(UsartRxCmdActStrBuf, "ALL")) ){
        cmd_act = TEST_CTRL_CMD_ACT_ALL;
    }else if( !(strcmp(UsartRxCmdActStrBuf, "ERS")) ){
        cmd_act = TEST_CTRL_CMD_ACT_ERASE;
    }else{
        free(UsartRxCmdActStrBuf);
        UsartRxCmdActStrBuf = NULL;
        USART_CmdProcFinishProc();
        return;
    }
#endif
    free(UsartRxCmdActStrBuf);
    UsartRxCmdActStrBuf = NULL;

    USART_TestCtrlCmdSend(cmd, cmd_act, UsartRxCmdParaBuf);
    
    USART_CmdProcFinishProc();
}

void USART_TestCtrlCmdSend(enum USARTTestCtrlCmd cmd, enum USARTTestCtrlCmdAct cmd_act, int cmd_para)
{
    Msg_t   Msg;

    Msg.expire = 0;
    Msg.prio = MSG_PRIO_HIGHEST;
    Msg.type = MSG_TYPE_TEST_CTRL;
    Msg.MsgCB = NULL;
    Msg.Data.TestCtrlDat.Cmd = cmd;
    Msg.Data.TestCtrlDat.Cmd_Act = cmd_act;
    Msg.Data.TestCtrlDat.Cmd_Para = cmd_para;
    SweepRobot_SendMsg(&Msg);
}

/* TEST IrDA process */
static void SWRB_IrDATestCodeTxSend(u8 code)
{
    u8 i;

    IFRD_TX_ENABLE();
    uDelay(5000);
    IFRD_TX_DISABLE();
    uDelay(1000);

    for(i=0;i<8;i++){
        if(code & 0x800){
            IFRD_TX_ENABLE();
            uDelay(3000);
            IFRD_TX_DISABLE();
            uDelay(1500);
        }else{
            IFRD_TX_ENABLE();
            uDelay(1500);
            IFRD_TX_DISABLE();
            uDelay(3000);
        }
        code<<=1;
    }

    IFRD_TX_ENABLE();
    uDelay(1000);
    IFRD_TX_DISABLE();
}

static void SweepRobotTest_CtrlMsgTestOnProc(void)
{
    gRobotState = ROBOT_STATE_TEST;
    TIM_Cmd(BAT_MONITOR_TIM, DISABLE);
    TIM_ClearFlag(BAT_MONITOR_TIM, TIM_IT_Update);
    plat_int_reg_cb(BAT_MONITOR_TIM_INT_IDX, (void*)BM_TestConditionUpdate);
    WheelCntMach_TestStart();
    IFRD_TestPathDetectStart();
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_FAN,    0);
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_MBRUSH, 0);
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_LBRUSH, 0);
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_RBRUSH, 0);
    CtrlPanel_LEDCtrl(CTRL_PANEL_LED_BLUE, CTRL_PANEL_LED_BR_LVL);
        
}

static void SweepRobotTest_CtrlMsgTestOffProc(void)
{
    gRobotState = ROBOT_STATE_IDLE;
    BM_ChargeTestStop();
    plat_int_reg_cb(BAT_MONITOR_TIM_INT_IDX, (void*)BM_ConditionUpdate);
    TIM_Cmd(BAT_MONITOR_TIM, ENABLE);
    WheelCntMach_TestStop();
    IFRD_TestPathDetectStop();
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_FAN,    0);
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_MBRUSH, 0);
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_LBRUSH, 0);
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_RBRUSH, 0);
    CtrlPanel_LEDCtrl(CTRL_PANEL_LED_GREEN, CTRL_PANEL_LED_BR_LVL);
}

static void SweepRobotTest_CtrlMsgManulReadProc(void)
{
    int i;

    aSwrbTestData[SWRB_TEST_DATA_WHEEL_L_SPEED_POS] = MotionCtrl_ChanSpeedGet(MOTOR_SELECT_L);
    aSwrbTestData[SWRB_TEST_DATA_WHEEL_R_SPEED_POS] = MotionCtrl_ChanSpeedGet(MOTOR_SELECT_R);
    aSwrbTestData[SWRB_TEST_DATA_BRUSH_L_CUR_POS] = ADCConvertedLSB[MEAS_CHAN_BRUSH_CUR_LEFT-1];
    aSwrbTestData[SWRB_TEST_DATA_BRUSH_R_CUR_POS] = ADCConvertedLSB[MEAS_CHAN_BRUSH_CUR_RIGHT-1];

    aSwrbTestData[SWRB_TEST_DATA_BRUSH_M_CUR_POS] = ADCConvertedLSB[MEAS_CHAN_BRUSH_CUR_MIDDLE-1];
    aSwrbTestData[SWRB_TEST_DATA_FAN_CUR_POS] = ADCConvertedLSB[MEAS_CHAN_FAN_CUR-1];
    aSwrbTestData[SWRB_TEST_DATA_IFRD_FL_POS] = ADCConvertedLSB[MEAS_CHAN_IFRD_FRONT_RX_L-1];
    aSwrbTestData[SWRB_TEST_DATA_IFRD_FR_POS] = ADCConvertedLSB[MEAS_CHAN_IFRD_FRONT_RX_R-1];

    aSwrbTestData[SWRB_TEST_DATA_IFRD_L_POS] = ADCConvertedLSB[MEAS_CHAN_IFRD_SIDE_RX_L-1];
    aSwrbTestData[SWRB_TEST_DATA_IFRD_R_POS] = ADCConvertedLSB[MEAS_CHAN_IFRD_SIDE_RX_R-1];
    aSwrbTestData[SWRB_TEST_DATA_IFRD_B_FL_POS] = ADCConvertedLSB[MEAS_CHAN_IFRD_BOTTOM_RX_L-1];
    aSwrbTestData[SWRB_TEST_DATA_IFRD_B_FR_POS] = ADCConvertedLSB[MEAS_CHAN_IFRD_BOTTOM_RX_R-1];

    aSwrbTestData[SWRB_TEST_DATA_IFRD_B_SL_POS] = ADCConvertedLSB[MEAS_CHAN_IFRD_BOTTOM_RX_L-1];
    aSwrbTestData[SWRB_TEST_DATA_IFRD_B_SR_POS] = ADCConvertedLSB[MEAS_CHAN_IFRD_BOTTOM_RX_R-1];
    aSwrbTestData[SWRB_TEST_DATA_COLLISION_L_POS] = COLLISION_SIGN_LEFT;
    aSwrbTestData[SWRB_TEST_DATA_COLLISION_FL_POS] = COLLISION_SIGN_FL;

    aSwrbTestData[SWRB_TEST_DATA_COLLISION_R_POS] = COLLISION_SIGN_RIGHT;
    aSwrbTestData[SWRB_TEST_DATA_COLLISION_FR_POS] = COLLISION_SIGN_FR;
    aSwrbTestData[SWRB_TEST_DATA_WHEEL_FLOAT_L_POS] = LWHEEL_FLOAT_SIGN;
    aSwrbTestData[SWRB_TEST_DATA_WHEEL_FLOAT_R_POS] = RWHEEL_FLOAT_SIGN;

    aSwrbTestData[SWRB_TEST_DATA_ASH_TRAY_INS_POS] = ASH_TRAY_INSTALL_SIGN;
    aSwrbTestData[SWRB_TEST_DATA_ASH_TRAY_LVL_POS] = ADCConvertedLSB[MEAS_CHAN_ASH_TRAY_LVL-1];
    aSwrbTestData[SWRB_TEST_DATA_UNIWHEEL_POS] = ADCConvertedLSB[MEAS_CHAN_UNIVERSAL_WHEEL_SIG-1];
    aSwrbTestData[SWRB_TEST_DATA_KEY_POS] = KEY_SIGN;

    aSwrbTestData[SWRB_TEST_DATA_IRDA_B_RxCODE_POS] = gSwrbTestIrDARxCode[IRDA_BACK_LIGHT];
    aSwrbTestData[SWRB_TEST_DATA_IRDA_L_RxCODE_POS] = gSwrbTestIrDARxCode[IRDA_LEFT_LIGHT];
    aSwrbTestData[SWRB_TEST_DATA_IRDA_FL_RxCODE_POS] = gSwrbTestIrDARxCode[IRDA_FRONT_L_LIGHT];
    aSwrbTestData[SWRB_TEST_DATA_IRDA_FR_RxCODE_POS] = gSwrbTestIrDARxCode[IRDA_FRONT_R_LIGHT];

    aSwrbTestData[SWRB_TEST_DATA_IRDA_R_RxCODE_POS] = gSwrbTestIrDARxCode[IRDA_RIGHT_LIGHT];
    aSwrbTestData[SWRB_TEST_DATA_CHARGE_CUR_POS] = ADCConvertedLSB[MEAS_CHAN_BAT_CHARGE_CUR-1];
    aSwrbTestData[SWRB_TEST_DATA_CHARGE_VOL_POS] = ADCConvertedLSB[MEAS_CHAN_BAT_VOL-1];
    aSwrbTestData[SWRB_TEST_DATA_CHARGE_24V_POS] = CHARGE_24V_SIGN;

    aSwrbTestData[SWRB_TEST_DATA_INTERNAL_REFVOL_POS] = ADCConvertedLSB[MEAS_CHAN_VREFIN-1];

    for(i=SWRB_TEST_DATA_WHEEL_L_SPEED_POS;i<SWRB_TEST_DATA_BOUND;i++){
        if(SWRB_TEST_DATA_IRDA_B_RxCODE_POS <= i && SWRB_TEST_DATA_IRDA_R_RxCODE_POS >= i){
            printf("%X,", aSwrbTestData[i]);
        }else{
            printf("%d,", aSwrbTestData[i]);
        }
        
        aSwrbTestData[i] = 0;
    }
    
    for(i=IRDA_BACK_LIGHT;i<IRDA_LIGHT_NUM;i++)
        gSwrbTestIrDARxCode[i] = 0;

    printf("\r\n");
}

static void SweepRobotTest_CtrlMsgWheelDirProc(USARTTestCtrlData_t *TestCtrlDat)
{
    switch(TestCtrlDat->Cmd_Para){
        case MOTOR_CTRL_DIR_BACKWARD:
            MotorCtrl_ChanDirSet(MOTOR_CTRL_CHAN_LWHEEL, MOTOR_CTRL_DIR_BACKWARD);
            MotorCtrl_ChanDirSet(MOTOR_CTRL_CHAN_RWHEEL, MOTOR_CTRL_DIR_BACKWARD);
            break;
        case MOTOR_CTRL_DIR_FORWARD:
            MotorCtrl_ChanDirSet(MOTOR_CTRL_CHAN_LWHEEL, MOTOR_CTRL_DIR_FORWARD);
            MotorCtrl_ChanDirSet(MOTOR_CTRL_CHAN_RWHEEL, MOTOR_CTRL_DIR_FORWARD);
            break;
    }
}

static void SweepRobotTest_CtrlMsgWheelOnProc(USARTTestCtrlData_t *TestCtrlDat)
{
    switch(TestCtrlDat->Cmd_Para){
        case MOTOR_SELECT_L:
            MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_LWHEEL, MOTOR_LWHEEL_CHAN_STARTUP_SPEED);
            break;
        case MOTOR_SELECT_R:
            MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_RWHEEL, MOTOR_RWHEEL_CHAN_STARTUP_SPEED);
            break;
    }
}

static void SweepRobotTest_CtrlMsgWheelOffProc(USARTTestCtrlData_t *TestCtrlDat)
{
    switch(TestCtrlDat->Cmd_Para){
        case MOTOR_SELECT_L:
            MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_LWHEEL, 0);
            break;
        case MOTOR_SELECT_R:
            MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_RWHEEL, 0);
            break;
    }
}

static void SweepRobotTest_CtrlMsgWheelReadProc(USARTTestCtrlData_t *TestCtrlDat)
{
    switch(TestCtrlDat->Cmd_Para){
        case MOTOR_SELECT_L:
            printf("%d\r\n", MotionCtrl_ChanSpeedGet(MOTOR_SELECT_L) );
            break;
        case MOTOR_SELECT_R:
            printf("%d\r\n", MotionCtrl_ChanSpeedGet(MOTOR_SELECT_R) );
            break;
    }
}

static void SweepRobotTest_CtrlMsgSingleWheelDirProc(enum MotorCtrlChannel motor_ctrl_chan, USARTTestCtrlData_t *TestCtrlDat)
{
    switch(TestCtrlDat->Cmd_Para){
        case MOTOR_CTRL_DIR_BACKWARD:
            MotorCtrl_ChanDirSet(motor_ctrl_chan, MOTOR_CTRL_DIR_BACKWARD);
            break;
        case MOTOR_CTRL_DIR_FORWARD:
            MotorCtrl_ChanDirSet(motor_ctrl_chan, MOTOR_CTRL_DIR_FORWARD);
            break;
    }
}

static void SweepRobotTest_CtrlMsgSingleWheelSpeedProc(enum MotorCtrlChannel motor_ctrl_chan, USARTTestCtrlData_t *TestCtrlDat)
{
    MotorCtrl_ChanSpeedLevelSet(motor_ctrl_chan, TestCtrlDat->Cmd_Para);
}

static void SweepRobotTest_CtrlMsgSingleWheelReadProc(enum MotorSelect motor_select, USARTTestCtrlData_t *TestCtrlDat)
{
    printf("%d\r\n", MotionCtrl_ChanSpeedGet(motor_select) );
}

static void SweepRobotTest_CtrlMsgBrushOnProc(USARTTestCtrlData_t *TestCtrlDat)
{
    switch(TestCtrlDat->Cmd_Para){
        case 0:
            MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_LBRUSH, MOTOR_LBRUSH_CHAN_STARTUP_SPEED);
            break;
        case 1:
            MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_RBRUSH, MOTOR_RBRUSH_CHAN_STARTUP_SPEED);
            break;
        case 2:
            MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_MBRUSH, MOTOR_MBRUSH_CHAN_STARTUP_SPEED);
            break;
    }
}

static void SweepRobotTest_CtrlMsgBrushOffProc(USARTTestCtrlData_t *TestCtrlDat)
{
    switch(TestCtrlDat->Cmd_Para){
        case 0:
            MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_LBRUSH, 0);
            break;
        case 1:
            MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_RBRUSH, 0);
            break;
        case 2:
            MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_MBRUSH, 0);
            break;
    }
}

static void SweepRobotTest_CtrlMsgBrushReadProc(USARTTestCtrlData_t *TestCtrlDat)
{
    switch(TestCtrlDat->Cmd_Para){
        case 0:
            printf("%d\r\n", ADCConvertedLSB[MEAS_CHAN_BRUSH_CUR_LEFT-1]);
            break;
        case 1:
            printf("%d\r\n", ADCConvertedLSB[MEAS_CHAN_BRUSH_CUR_RIGHT-1]);
            break;
        case 2:
            printf("%d\r\n", ADCConvertedLSB[MEAS_CHAN_BRUSH_CUR_MIDDLE-1]);
            break;
    }
}

static void SweepRobotTest_CtrlMsgRgbLedOnProc(USARTTestCtrlData_t *TestCtrlDat)
{
    switch(TestCtrlDat->Cmd_Para){
        case CTRL_PANEL_LED_RED:
            CtrlPanel_LEDCtrl(CTRL_PANEL_LED_RED, CTRL_PANEL_LED_BR_LVL);
            break;
        case CTRL_PANEL_LED_GREEN:
            CtrlPanel_LEDCtrl(CTRL_PANEL_LED_GREEN, CTRL_PANEL_LED_BR_LVL);
            break;
        case CTRL_PANEL_LED_BLUE:
            CtrlPanel_LEDCtrl(CTRL_PANEL_LED_BLUE, CTRL_PANEL_LED_BR_LVL);
            break;
        case CTRL_PANEL_LED_RGB:
            CtrlPanel_LEDCtrl(CTRL_PANEL_LED_RGB, CTRL_PANEL_LED_BR_LVL);
            break;
    }
}

static void SweepRobotTest_CtrlMsgRgbLedOffProc(USARTTestCtrlData_t *TestCtrlDat)
{
    switch(TestCtrlDat->Cmd_Para){
        case CTRL_PANEL_LED_RED:
            CtrlPanel_LEDCtrl(CTRL_PANEL_LED_RED, 0);
            break;
        case CTRL_PANEL_LED_GREEN:
            CtrlPanel_LEDCtrl(CTRL_PANEL_LED_GREEN, 0);
            break;
        case CTRL_PANEL_LED_BLUE:
            CtrlPanel_LEDCtrl(CTRL_PANEL_LED_BLUE, 0);
            break;
        case CTRL_PANEL_LED_RGB:
            CtrlPanel_LEDCtrl(CTRL_PANEL_LED_RGB, 0);
            break;
    }
}

static void SweepRobotTest_CtrlMsgIrDAOnProc(void)
{
    TIM_SetCounter(BAT_MONITOR_TIM, 0);
    TIM_Cmd(BAT_MONITOR_TIM, ENABLE);
}

static void SweepRobotTest_CtrlMsgIrDAOffProc(void)
{
    PWM_DutyCycleSet(PWM_CHAN_CHARGE, 0);
    TIM_Cmd(BAT_MONITOR_TIM, DISABLE);
    TIM_SetCounter(BAT_MONITOR_TIM, 0);
}

static void SweepRobotTest_CtrlMsgIrDAReadProc(void)
{
    enum IRAD_Light i;

    for(i=IRDA_BACK_LIGHT;i<IRDA_LIGHT_NUM;i++)
        printf("%d,",gSwrbTestIrDARxCode[i]);
    printf("\r\n");
}

static void SweepRobotTest_CtrlMsgIrDAWriteProc(u8 code)
{
    SWRB_IrDATestCodeTxSend(code);
}

static void SweepRobotTest_CtrlMsgIrDAEraseProc(void)
{
    enum IRAD_Light i;

    for(i=IRDA_BACK_LIGHT;i<IRDA_LIGHT_NUM;i++)
        gSwrbTestIrDARxCode[i] = 0;
}

void SweepRobotTest_IrDARxCodeProc(PwrStationSigData_t *PwrSig)
{
    gSwrbTestIrDARxCode[PwrSig->src] = PwrSig->sig;
}

static void SweepRobotTest_CtrlMsgBuzzerOnProc(USARTTestCtrlData_t *TestCtrlDat)
{
    switch(TestCtrlDat->Cmd_Para){
        case BUZZER_ONE_PULS:
            Buzzer_Play(BUZZER_ONE_PULS, BUZZER_SND_SHORT);
            break;
        case BUZZER_TWO_PULS:
            Buzzer_Play(BUZZER_TWO_PULS, BUZZER_SND_SHORT);
            break;
        case BUZZER_TRI_PULS:
            Buzzer_Play(BUZZER_TRI_PULS, BUZZER_SND_SHORT);
            break;
        case BUZZER_CONSECUTIVE_PULS:
            Buzzer_Play(BUZZER_CONSECUTIVE_PULS, BUZZER_SND_SHORT);
            break;
    }
}

static void SweepRobotTest_CtrlMsgChargeReadProc(USARTTestCtrlData_t *TestCtrlDat)
{
    switch(TestCtrlDat->Cmd_Para){
        case 0:
            printf("%d\r\n", ADCConvertedLSB[MEAS_CHAN_BAT_VOL-1]);
            break;
        case 1:
            printf("%d\r\n", ADCConvertedLSB[MEAS_CHAN_BAT_CHARGE_CUR-1]);
            break;
        case 2:
            printf("%d\r\n", CHARGE_24V_SIGN);
            break;
        default:break;
    }
}

static void SweepRobotTest_CtrlMsgCollisionReadProc(USARTTestCtrlData_t *TestCtrlDat)
{
    switch(TestCtrlDat->Cmd_Para){
        case 0:
            printf("%d\r\n", COLLISION_SIGN_LEFT);
            break;
        case 1:
            printf("%d\r\n", COLLISION_SIGN_FL);
            break;
        case 2:
            printf("%d\r\n", COLLISION_SIGN_RIGHT);
            break;
        case 3:
            printf("%d\r\n", COLLISION_SIGN_FR);
            break;
        default:break;
    }
}

static void SweepRobotTest_CtrlMsgWheelFloatReadProc(USARTTestCtrlData_t *TestCtrlDat)
{
    switch(TestCtrlDat->Cmd_Para){
        case 0:
            printf("%d\r\n", LWHEEL_FLOAT_SIGN);
            break;
        case 1:
            printf("%d\r\n", RWHEEL_FLOAT_SIGN);
            break;
        default:break;
    }
}

static void SweepRobotTest_CtrlMsgAshTrayReadProc(USARTTestCtrlData_t *TestCtrlDat)
{
    switch(TestCtrlDat->Cmd_Para){
        case 0:
            printf("%d\r\n", ASH_TRAY_INSTALL_SIGN);
            break;
        case 1:
            printf("%d\r\n", ADCConvertedLSB[MEAS_CHAN_ASH_TRAY_LVL-1]);
            break;
        default:break;
    }
}

static void SweepRobotTest_CtrlMsgSensorReadProc(USARTTestCtrlData_t *TestCtrlDat)
{
    switch(TestCtrlDat->Cmd_Para){
        case MEAS_CHAN_IFRD_FRONT_RX_L:
            printf("%d\r\n", ADCConvertedLSB[MEAS_CHAN_IFRD_FRONT_RX_L-1]);
            break;
        case MEAS_CHAN_IFRD_FRONT_RX_R:
            printf("%d\r\n", ADCConvertedLSB[MEAS_CHAN_IFRD_FRONT_RX_R-1]);
            break;
        case MEAS_CHAN_IFRD_SIDE_RX_L:
            printf("%d\r\n", ADCConvertedLSB[MEAS_CHAN_IFRD_SIDE_RX_L-1]);
            break;
        case MEAS_CHAN_IFRD_SIDE_RX_R:
            printf("%d\r\n", ADCConvertedLSB[MEAS_CHAN_IFRD_SIDE_RX_R-1]);
            break;
        case MEAS_CHAN_IFRD_BOTTOM_RX_L:
            printf("%d\r\n", ADCConvertedLSB[MEAS_CHAN_IFRD_BOTTOM_RX_L-1]);
            break;
        case MEAS_CHAN_IFRD_BOTTOM_RX_R:
            printf("%d\r\n", ADCConvertedLSB[MEAS_CHAN_IFRD_BOTTOM_RX_R-1]);
            break;
        case MEAS_CHAN_UNIVERSAL_WHEEL_SIG:
            printf("%d\r\n", ADCConvertedLSB[MEAS_CHAN_UNIVERSAL_WHEEL_SIG-1]);
            break;
        case MEAS_CHAN_BRUSH_CUR_LEFT:
            printf("%d\r\n", ADCConvertedLSB[MEAS_CHAN_BRUSH_CUR_LEFT-1]);
            break;
        case MEAS_CHAN_BRUSH_CUR_RIGHT:
            printf("%d\r\n", ADCConvertedLSB[MEAS_CHAN_BRUSH_CUR_RIGHT-1]);
            break;
        case MEAS_CHAN_BRUSH_CUR_MIDDLE:
            printf("%d\r\n", ADCConvertedLSB[MEAS_CHAN_BRUSH_CUR_MIDDLE-1]);
            break;
        case MEAS_CHAN_FAN_CUR:
            printf("%d\r\n", ADCConvertedLSB[MEAS_CHAN_FAN_CUR-1]);
            break;
        case MEAS_CHAN_BAT_CHARGE_CUR:
            printf("%d\r\n", ADCConvertedLSB[MEAS_CHAN_BAT_CHARGE_CUR-1]);
            break;
        case MEAS_CHAN_BAT_VOL:
            printf("%d\r\n", ADCConvertedLSB[MEAS_CHAN_BAT_VOL-1]);
            break;
        case MEAS_CHAN_ASH_TRAY_LVL:
            printf("%d\r\n", ADCConvertedLSB[MEAS_CHAN_ASH_TRAY_LVL-1]);
            break;
        case MEAS_CHAN_VREFIN:
            printf("%d\r\n", ADCConvertedLSB[MEAS_CHAN_VREFIN-1]);
            break;
    }
}

/* XXX: READ and WRITE BkpRegister to save Serial number */
//static void SweepRobotTest_CtrlMsgSNReadBkpRegProc(uint16_t bkp_reg)
//{
//    printf("%d\r\n", BKP_ReadBackupRegister(bkp_reg) );
//}

//static void SweepRobotTest_CtrlMsgSNReadAllBkpRegProc(void)
//{
//    printf("%d%d%d%d\r\n", BKP_ReadBackupRegister(SWRB_TEST_SN_YEAR_REG), BKP_ReadBackupRegister(SWRB_TEST_SN_MONTH_REG),\
//                           BKP_ReadBackupRegister(SWRB_TEST_SN_DATE_REG), BKP_ReadBackupRegister(SWRB_TEST_SN_SNUM_REG) );
//}

//static void SweepRobotTest_CtrlMsgSNWriteBkpRegProc(uint16_t bkp_reg, USARTTestCtrlData_t *TestCtrlDat)
//{
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
//    PWR_BackupAccessCmd(ENABLE);
//    BKP_WriteBackupRegister(bkp_reg, TestCtrlDat->Cmd_Para);
//}

static void SweepRobotTest_CtrlMsgSNReadFlashProc(u32 addr)
{
    printf("%d\r\n", UserMEM_HalfWordRead(addr) );
}

static void SweepRobotTest_CtrlMsgSNReadAllFlashProc(void)
{
    printf("SerialNumber:%d%d%d%03d\r\n", UserMEM_HalfWordRead(SWRB_TEST_SN_YEAR_FLASH_ADDR), UserMEM_HalfWordRead(SWRB_TEST_SN_MONTH_FLASH_ADDR),\
                           UserMEM_HalfWordRead(SWRB_TEST_SN_DATE_FLASH_ADDR), UserMEM_HalfWordRead(SWRB_TEST_SN_SNUM_FLASH_ADDR) );
}

static void SweepRobotTest_CtrlMsgSNWriteFlashProc(u32 addr, USARTTestCtrlData_t *TestCtrlDat)
{
    UserMEM_Init();
    UserMEM_HalfWordWrite(addr, TestCtrlDat->Cmd_Para);
    UserMEM_DeInit();
}

static void SweepRobotTest_CtrlMsgSNEraseFlashPageProc(u32 addr)
{
    UserMEM_Init();
    UserMEM_EraseByte(addr);
    UserMEM_DeInit();
}

/* SWRB TEST On Msg Proc */
void SweepRobotTest_StartCtrlMsgPorc(USARTTestCtrlData_t *TestCtrlDat)
{
    switch(TestCtrlDat->Cmd){
        case TEST_CTRL_CMD_TEST:
            switch(TestCtrlDat->Cmd_Act){
                case TEST_CTRL_CMD_ACT_ON:
                    SweepRobotTest_CtrlMsgTestOnProc();
                    break;
                default:break;
            }
            break;
        case TEST_CTRL_CMD_SN_READ:
            switch(TestCtrlDat->Cmd_Act){
                case TEST_CTRL_CMD_ACT_ALL:
//                    SweepRobotTest_CtrlMsgSNReadAllBkpRegProc();
                    SweepRobotTest_CtrlMsgSNReadAllFlashProc();
                    break;
                default:break;
            }
            break;
        case TEST_CTRL_CMD_SENSOR:
            switch(TestCtrlDat->Cmd_Act){
                case TEST_CTRL_CMD_ACT_READ:
                    SweepRobotTest_CtrlMsgSensorReadProc(TestCtrlDat);
                    break;
            }
            break;
        default:break;
    }
}

/* SWRB TEST CTRL Msg Proc */
void SweepRobotTest_CtrlMsgProc(USARTTestCtrlData_t *TestCtrlDat)
{
    switch(TestCtrlDat->Cmd){
        case TEST_CTRL_CMD_TEST:
            switch(TestCtrlDat->Cmd_Act){
                case TEST_CTRL_CMD_ACT_ON:
                    SweepRobotTest_CtrlMsgTestOnProc();
                    break;
                case TEST_CTRL_CMD_ACT_OFF:
                    SweepRobotTest_CtrlMsgTestOffProc();
                    break;
            }
            break;
        case TEST_CTRL_CMD_MANUL:
            switch(TestCtrlDat->Cmd_Act){
                case TEST_CTRL_CMD_ACT_READ:
                    SweepRobotTest_CtrlMsgManulReadProc();
                    break;
            }
            break;
        case TEST_CTRL_CMD_WHEEL:
            switch(TestCtrlDat->Cmd_Act){
                case TEST_CTRL_CMD_ACT_DIR:
                    SweepRobotTest_CtrlMsgWheelDirProc(TestCtrlDat);
                    break;
                case TEST_CTRL_CMD_ACT_ON:
                    SweepRobotTest_CtrlMsgWheelOnProc(TestCtrlDat);
                    break;
                case TEST_CTRL_CMD_ACT_OFF:
                    SweepRobotTest_CtrlMsgWheelOffProc(TestCtrlDat);
                    break;
                case TEST_CTRL_CMD_ACT_READ:
                    SweepRobotTest_CtrlMsgWheelReadProc(TestCtrlDat);
                    break;
                default:break;
            }
            break;
        case TEST_CTRL_CMD_LWHEEL:
            switch(TestCtrlDat->Cmd_Act){
                case TEST_CTRL_CMD_ACT_DIR:
                    SweepRobotTest_CtrlMsgSingleWheelDirProc(MOTOR_CTRL_CHAN_LWHEEL, TestCtrlDat);
                    break;
                case TEST_CTRL_CMD_ACT_SPEED:
                    SweepRobotTest_CtrlMsgSingleWheelSpeedProc(MOTOR_CTRL_CHAN_LWHEEL, TestCtrlDat);
                    break;
                case TEST_CTRL_CMD_ACT_READ:
                    SweepRobotTest_CtrlMsgSingleWheelReadProc(MOTOR_SELECT_L, TestCtrlDat);
                    break;
                default:break;
            }
            break;
        case TEST_CTRL_CMD_RWHEEL:
            switch(TestCtrlDat->Cmd_Act){
                case TEST_CTRL_CMD_ACT_DIR:
                    SweepRobotTest_CtrlMsgSingleWheelDirProc(MOTOR_CTRL_CHAN_RWHEEL, TestCtrlDat);
                    break;
                case TEST_CTRL_CMD_ACT_SPEED:
                    SweepRobotTest_CtrlMsgSingleWheelSpeedProc(MOTOR_CTRL_CHAN_RWHEEL, TestCtrlDat);
                    break;
                case TEST_CTRL_CMD_ACT_READ:
                    SweepRobotTest_CtrlMsgSingleWheelReadProc(MOTOR_SELECT_R, TestCtrlDat);
                    break;
            }
            break;
        case TEST_CTRL_CMD_BRUSH:
            switch(TestCtrlDat->Cmd_Act){
                case TEST_CTRL_CMD_ACT_ON:
                    SweepRobotTest_CtrlMsgBrushOnProc(TestCtrlDat);
                    break;
                case TEST_CTRL_CMD_ACT_OFF:
                    SweepRobotTest_CtrlMsgBrushOffProc(TestCtrlDat);
                    break;
                case TEST_CTRL_CMD_ACT_READ:
                    SweepRobotTest_CtrlMsgBrushReadProc(TestCtrlDat);
                    break;
            }
            break;
        case TEST_CTRL_CMD_LBRUSH:
            switch(TestCtrlDat->Cmd_Act){
                case TEST_CTRL_CMD_ACT_ON:
                    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_LBRUSH, MOTOR_LBRUSH_CHAN_STARTUP_SPEED);
                    break;
                case TEST_CTRL_CMD_ACT_OFF:
                    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_LBRUSH, 0);
                    break;
                case TEST_CTRL_CMD_ACT_SPEED:
                    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_LBRUSH, TestCtrlDat->Cmd_Para);
                    break;
                case TEST_CTRL_CMD_ACT_READ:
                    printf("%d\r\n", ADCConvertedLSB[MEAS_CHAN_BRUSH_CUR_LEFT-1]);
                    break;
            }
            break;
        case TEST_CTRL_CMD_RBRUSH:
            switch(TestCtrlDat->Cmd_Act){
                case TEST_CTRL_CMD_ACT_ON:
                    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_RBRUSH, MOTOR_RBRUSH_CHAN_STARTUP_SPEED);
                    break;
                case TEST_CTRL_CMD_ACT_OFF:
                    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_RBRUSH, 0);
                    break;
                case TEST_CTRL_CMD_ACT_SPEED:
                    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_RBRUSH, TestCtrlDat->Cmd_Para);
                    break;
                case TEST_CTRL_CMD_ACT_READ:
                    printf("%d\r\n", ADCConvertedLSB[MEAS_CHAN_BRUSH_CUR_RIGHT-1]);
                    break;
            }
            break;
        case TEST_CTRL_CMD_MBRUSH:
            switch(TestCtrlDat->Cmd_Act){
                case TEST_CTRL_CMD_ACT_ON:
                    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_MBRUSH, MOTOR_MBRUSH_CHAN_STARTUP_SPEED);
                    break;
                case TEST_CTRL_CMD_ACT_OFF:
                    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_MBRUSH, 0);
                    break;
                case TEST_CTRL_CMD_ACT_SPEED:
                    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_MBRUSH, TestCtrlDat->Cmd_Para);
                    break;
                case TEST_CTRL_CMD_ACT_READ:
                    printf("%d\r\n", ADCConvertedLSB[MEAS_CHAN_BRUSH_CUR_MIDDLE-1]);
                    break;
            }
            break;
        case TEST_CTRL_CMD_FAN:
            switch(TestCtrlDat->Cmd_Act){
                case TEST_CTRL_CMD_ACT_ON:
                    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_FAN, MOTOR_FAN_CHAN_STARTUP_SPEED);
                    break;
                case TEST_CTRL_CMD_ACT_OFF:
                    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_FAN, 0);
                    break;
                case TEST_CTRL_CMD_ACT_SPEED:
                    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_FAN, TestCtrlDat->Cmd_Para);
                    break;
                case TEST_CTRL_CMD_ACT_READ:
                    printf("%d\r\n", ADCConvertedLSB[MEAS_CHAN_FAN_CUR-1]);
                    break;
            }
            break;
        case TEST_CTRL_CMD_RGB_LED:
            switch(TestCtrlDat->Cmd_Act){
                case TEST_CTRL_CMD_ACT_ON:
                    SweepRobotTest_CtrlMsgRgbLedOnProc(TestCtrlDat);
                    break;
                case TEST_CTRL_CMD_ACT_OFF:
                    SweepRobotTest_CtrlMsgRgbLedOffProc(TestCtrlDat);
                    break;
            }
            break;
        case TEST_CTRL_CMD_KEY:
            switch(TestCtrlDat->Cmd_Act){
                case TEST_CTRL_CMD_ACT_READ:
                    printf("%d\r\n", KEY_SIGN);
                    break;
            }
            break;
        case TEST_CTRL_CMD_IRDA:
            switch(TestCtrlDat->Cmd_Act){
                case TEST_CTRL_CMD_ACT_ON:
                    SweepRobotTest_CtrlMsgIrDAOnProc();
                    break;
                case TEST_CTRL_CMD_ACT_OFF:
                    SweepRobotTest_CtrlMsgIrDAOffProc();
                    break;
                case TEST_CTRL_CMD_ACT_READ:
                    SweepRobotTest_CtrlMsgIrDAReadProc();
                    break;
                case TEST_CTRL_CMD_ACT_WRITE:
                    SweepRobotTest_CtrlMsgIrDAWriteProc(TestCtrlDat->Cmd_Para);
                    break;
                case TEST_CTRL_CMD_ACT_ERASE:
                    SweepRobotTest_CtrlMsgIrDAEraseProc();
                    break;
                default:break;
            }
            break;
        case TEST_CTRL_CMD_BUZZER:
            switch(TestCtrlDat->Cmd_Act){
                case TEST_CTRL_CMD_ACT_ON:
                    SweepRobotTest_CtrlMsgBuzzerOnProc(TestCtrlDat);
                    break;
                case TEST_CTRL_CMD_ACT_OFF:
                    Buzzer_Stop();
                    break;
            }
            break;
        case TEST_CTRL_CMD_CHARGE:
            switch(TestCtrlDat->Cmd_Act){
                case TEST_CTRL_CMD_ACT_ON:
                    BM_ChargeTestStart(TestCtrlDat);
                    break;
                case TEST_CTRL_CMD_ACT_OFF:
                    BM_ChargeTestStop();
                    break;
                case TEST_CTRL_CMD_ACT_READ:
                    SweepRobotTest_CtrlMsgChargeReadProc(TestCtrlDat);
                    break;
            }
            break;
        case TEST_CTRL_CMD_COLLISION:
            switch(TestCtrlDat->Cmd_Act){
                case TEST_CTRL_CMD_ACT_READ:
                    SweepRobotTest_CtrlMsgCollisionReadProc(TestCtrlDat);
                    break;
            }
            break;
        case TEST_CTRL_CMD_WHEEL_FLOAT:
            switch(TestCtrlDat->Cmd_Act){
                case TEST_CTRL_CMD_ACT_READ:
                    SweepRobotTest_CtrlMsgWheelFloatReadProc(TestCtrlDat);
                    break;
            }
            break;
        case TEST_CTRL_CMD_ASH_TRAY:
            switch(TestCtrlDat->Cmd_Act){
                case TEST_CTRL_CMD_ACT_READ:
                    SweepRobotTest_CtrlMsgAshTrayReadProc(TestCtrlDat);
                    break;
            }
            break;
        case TEST_CTRL_CMD_POWER_STATION:
            switch(TestCtrlDat->Cmd_Act){
                case TEST_CTRL_CMD_ACT_ON:
                    BM_ChargeTestStart(TestCtrlDat);
                    break;
                case TEST_CTRL_CMD_ACT_OFF:
                    BM_ChargeTestStop();
                    break;
                case TEST_CTRL_CMD_ACT_READ:
                    break;
            }
            break;
        case TEST_CTRL_CMD_SENSOR:
            switch(TestCtrlDat->Cmd_Act){
                case TEST_CTRL_CMD_ACT_IFRD_LED:
                    if(TestCtrlDat->Cmd_Para)
                        IFRD_TX_ENABLE();
                    else
                        IFRD_TX_DISABLE();
                    break;
                case TEST_CTRL_CMD_ACT_B_SWITCH:
                    if(TestCtrlDat->Cmd_Para){
//                        Meas_IFRDBottomRxSwitch(MEAS_IFRD_BOTTOM_CHAN_SIDE);
                        AD_CHAN_TDM_SW_ON();
                        uDelay(100);
                    }else{
//                        Meas_IFRDBottomRxSwitch(MEAS_IFRD_BOTTOM_CHAN_FRONT);
                        AD_CHAN_TDM_SW_OFF();
                        uDelay(100);
                    }
                    break;
                case TEST_CTRL_CMD_ACT_READ:
                    SweepRobotTest_CtrlMsgSensorReadProc(TestCtrlDat);
                    break;
            }
            break;
        case TEST_CTRL_CMD_SN_READ:
            switch(TestCtrlDat->Cmd_Act){
                case TEST_CTRL_CMD_ACT_YEAR:
//                    SweepRobotTest_CtrlMsgSNReadBkpRegProc(SWRB_TEST_SN_YEAR_REG);
                    SweepRobotTest_CtrlMsgSNReadFlashProc(SWRB_TEST_SN_YEAR_FLASH_ADDR);
                    break;
                case TEST_CTRL_CMD_ACT_MONTH:
//                    SweepRobotTest_CtrlMsgSNReadBkpRegProc(SWRB_TEST_SN_MONTH_REG);
                    SweepRobotTest_CtrlMsgSNReadFlashProc(SWRB_TEST_SN_MONTH_FLASH_ADDR);
                    break;
                case TEST_CTRL_CMD_ACT_DATE:
//                    SweepRobotTest_CtrlMsgSNReadBkpRegProc(SWRB_TEST_SN_DATE_REG);
                    SweepRobotTest_CtrlMsgSNReadFlashProc(SWRB_TEST_SN_DATE_FLASH_ADDR);
                    break;
                case TEST_CTRL_CMD_ACT_SN:
//                    SweepRobotTest_CtrlMsgSNReadBkpRegProc(SWRB_TEST_SN_SNUM_REG);
                    SweepRobotTest_CtrlMsgSNReadFlashProc(SWRB_TEST_SN_SNUM_FLASH_ADDR);
                    break;
                case TEST_CTRL_CMD_ACT_ALL:
//                    SweepRobotTest_CtrlMsgSNReadAllBkpRegProc();
                    SweepRobotTest_CtrlMsgSNReadAllFlashProc();
                    break;
                default:break;
            }
            break;
        case TEST_CTRL_CMD_SN_WRITE:
            switch(TestCtrlDat->Cmd_Act){
                case TEST_CTRL_CMD_ACT_YEAR:
//                    SweepRobotTest_CtrlMsgSNWriteBkpRegProc(SWRB_TEST_SN_YEAR_REG, TestCtrlDat);
                    SweepRobotTest_CtrlMsgSNWriteFlashProc(SWRB_TEST_SN_YEAR_FLASH_ADDR, TestCtrlDat);
                    break;
                case TEST_CTRL_CMD_ACT_MONTH:
//                    SweepRobotTest_CtrlMsgSNWriteBkpRegProc(SWRB_TEST_SN_MONTH_REG, TestCtrlDat);
                    SweepRobotTest_CtrlMsgSNWriteFlashProc(SWRB_TEST_SN_MONTH_FLASH_ADDR, TestCtrlDat);
                    break;
                case TEST_CTRL_CMD_ACT_DATE:
//                    SweepRobotTest_CtrlMsgSNWriteBkpRegProc(SWRB_TEST_SN_DATE_REG, TestCtrlDat);
                    SweepRobotTest_CtrlMsgSNWriteFlashProc(SWRB_TEST_SN_DATE_FLASH_ADDR, TestCtrlDat);
                    break;
                case TEST_CTRL_CMD_ACT_SN:
//                    SweepRobotTest_CtrlMsgSNWriteBkpRegProc(SWRB_TEST_SN_SNUM_REG, TestCtrlDat);
                    SweepRobotTest_CtrlMsgSNWriteFlashProc(SWRB_TEST_SN_SNUM_FLASH_ADDR, TestCtrlDat);
                    break;
                case TEST_CTRL_CMD_ACT_ERASE:
                    SweepRobotTest_CtrlMsgSNEraseFlashPageProc(SWRB_TEST_SN_FLASH_PAGE);
                    break;
                default:break;
            }
            break;
    }
}


