#ifndef __SWEEPROBOT_TEST_H
#define __SWEEPROBOT_TEST_H

#include "stm32f10x.h"
#include "IrDA.h"

enum SWRB_TEST_DATA_POS{
    
    SWRB_TEST_DATA_WHEEL_L_SPEED_POS,
    SWRB_TEST_DATA_WHEEL_R_SPEED_POS,
    
    SWRB_TEST_DATA_BRUSH_L_CUR_POS,
    SWRB_TEST_DATA_BRUSH_R_CUR_POS,
    SWRB_TEST_DATA_BRUSH_M_CUR_POS,
    
    SWRB_TEST_DATA_FAN_CUR_POS,
    
    SWRB_TEST_DATA_IFRD_FL_POS,
    SWRB_TEST_DATA_IFRD_FR_POS,
    SWRB_TEST_DATA_IFRD_L_POS,
    SWRB_TEST_DATA_IFRD_R_POS,
    SWRB_TEST_DATA_IFRD_B_FL_POS,
    SWRB_TEST_DATA_IFRD_B_FR_POS,
    SWRB_TEST_DATA_IFRD_B_SL_POS,
    SWRB_TEST_DATA_IFRD_B_SR_POS,
    
    SWRB_TEST_DATA_COLLISION_L_POS,
    SWRB_TEST_DATA_COLLISION_FL_POS,
    SWRB_TEST_DATA_COLLISION_R_POS,
    SWRB_TEST_DATA_COLLISION_FR_POS,
    
    SWRB_TEST_DATA_WHEEL_FLOAT_L_POS,
    SWRB_TEST_DATA_WHEEL_FLOAT_R_POS,
    
    SWRB_TEST_DATA_ASH_TRAY_INS_POS,
    SWRB_TEST_DATA_ASH_TRAY_LVL_POS,
    
    SWRB_TEST_DATA_UNIWHEEL_POS,
    
    SWRB_TEST_DATA_KEY_POS,
    
    SWRB_TEST_DATA_IRDA_B_RxCODE_POS,
    SWRB_TEST_DATA_IRDA_L_RxCODE_POS,
    SWRB_TEST_DATA_IRDA_FL_RxCODE_POS,
    SWRB_TEST_DATA_IRDA_FR_RxCODE_POS,
    SWRB_TEST_DATA_IRDA_R_RxCODE_POS,
    
    SWRB_TEST_DATA_BUZZER_OK_POS,
    
    SWRB_TEST_DATA_RGB_LED_OK_POS,
    
    SWRB_TEST_DATA_CHARGE_CUR_POS,
    SWRB_TEST_DATA_CHARGE_VOL_POS,
    SWRB_TEST_DATA_CHARGE_24V_POS,
    
    SWRB_TEST_DATA_INTERNAL_REFVOL_POS,
    
    SWRB_TEST_DATA_BOUND,
};

enum USARTTestCtrlCmd {
    
    TEST_CTRL_CMD_TEST,
    
    TEST_CTRL_CMD_MANUL,
    
    TEST_CTRL_CMD_WHEEL,
    TEST_CTRL_CMD_LWHEEL,
    TEST_CTRL_CMD_RWHEEL,
    TEST_CTRL_CMD_BRUSH,
    TEST_CTRL_CMD_LBRUSH,
    TEST_CTRL_CMD_RBRUSH,
    TEST_CTRL_CMD_MBRUSH,
    TEST_CTRL_CMD_FAN,
    
    TEST_CTRL_CMD_SENSOR,
    TEST_CTRL_CMD_COLLISION,
    TEST_CTRL_CMD_WHEEL_FLOAT,
    TEST_CTRL_CMD_ASH_TRAY,
    
    TEST_CTRL_CMD_RGB_LED,
    TEST_CTRL_CMD_KEY,
    TEST_CTRL_CMD_IRDA,
    TEST_CTRL_CMD_BUZZER,
    
    TEST_CTRL_CMD_CHARGE,
    
    TEST_CTRL_CMD_POWER_STATION,
    
    TEST_CTRL_CMD_SN_READ,
    TEST_CTRL_CMD_SN_WRITE,
};

enum USARTTestCtrlCmdAct {
    
    TEST_CTRL_CMD_ACT_ON,
    TEST_CTRL_CMD_ACT_OFF,
    TEST_CTRL_CMD_ACT_START,
    TEST_CTRL_CMD_ACT_STOP,
    TEST_CTRL_CMD_ACT_READ,
    TEST_CTRL_CMD_ACT_WRITE,
    TEST_CTRL_CMD_ACT_DIR,
    TEST_CTRL_CMD_ACT_SPEED,
    TEST_CTRL_CMD_ACT_IFRD_LED,
    TEST_CTRL_CMD_ACT_B_SWITCH,
    TEST_CTRL_CMD_ACT_YEAR,
    TEST_CTRL_CMD_ACT_MONTH,
    TEST_CTRL_CMD_ACT_DATE,
    TEST_CTRL_CMD_ACT_SN,
    TEST_CTRL_CMD_ACT_ALL,
    TEST_CTRL_CMD_ACT_ERASE,
};

typedef struct USARTTestCtrlData{

    enum USARTTestCtrlCmd       Cmd;
    enum USARTTestCtrlCmdAct    Cmd_Act;
    int                         Cmd_Para;
} USARTTestCtrlData_t;

extern u16 UsartRxState;

#define STDIO_UART          UART4

#define USART_RX_STATE_REC_FINISH_POS            15
#define USART_RX_STATE_CR_REC_POS                14
#define USART_RX_STATE_DASH_REC_POS              13
#define USART_RX_STATE_CMD_ACT_REC_EN_POS        12
#define USART_RX_STATE_CMD_PARA_REC_EN_POS       11
#define USART_RX_STATE_CNT_MASK                  0x07FF
#define USART_RX_STATE_CNT_CLR_MASK              0xF800
#define IS_UART4_GET_DATA_FINISH()              (UsartRxState&(1<<USART_RX_STATE_REC_FINISH_POS)?1:0)

void UART4_ISR(void);
void USART_CmdProc(void);

void SweepRobotTest_IrDARxCodeProc(PwrStationSigData_t *PwrSig);
void SweepRobotTest_StartCtrlMsgPorc(USARTTestCtrlData_t *TestCtrlDat);
void SweepRobotTest_CtrlMsgProc(USARTTestCtrlData_t *TestCtrlDat);

#endif