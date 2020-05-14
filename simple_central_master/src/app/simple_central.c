/******************************************************************************

 @file  simple_central.c

 @brief This file contains the Simple BLE Central sample application for use
        with the CC2650 Bluetooth Low Energy Protocol Stack.

 Group: CMCU, SCS
 Target Device: CC2640R2

 ******************************************************************************
 
 Copyright (c) 2013-2017, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: simplelink_cc2640r2_sdk_1_30_00_25
 Release Date: 2017-03-02 20:08:31
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Semaphore.h>

#include "bcomdef.h"

#include "hci_tl.h"
#include "linkdb.h"
#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "central.h"
#include "gapbondmgr.h"
#include "simple_gatt_profile.h"

#include "osal_snv.h"
#include "icall_apimsg.h"

#include "math.h"

#include "util.h"
#include "board_key.h"
#ifdef USE_CORE_SDK
#include <ti/display/Display.h>
#else // !USE_CORE_SDK
  #include <ti/mw/display/Display.h>
#endif // USE_CORE_SDK
#include "board_key.h"
#include "board.h"

#include "simple_central.h"

#include "ble_user_config.h"

#if defined( USE_FPGA ) || defined( DEBUG_SW_TRACE )
#include <driverlib/ioc.h>
#endif // USE_FPGA | DEBUG_SW_TRACE

#include "icall_api.h"

#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTCC26XX.h>
#include <ti/drivers/pin/PINCC26xx.h>
#include <driverlib/aon_batmon.h>
#include <driverlib/aux_wuc.h>

//power management
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/devices/DeviceFamily.h>

#include DeviceFamily_constructPath(inc/hw_fcfg1.h)
#include DeviceFamily_constructPath(driverlib/aux_adc.h)

#define TEST
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Simple BLE Central Task Events
#define SBC_START_DISCOVERY_EVT               0x0001
#define SBC_PAIRING_STATE_EVT                 0x0002
#define SBC_PASSCODE_NEEDED_EVT               0x0004
#define SBC_RSSI_READ_EVT                     0x0008
#define SBC_KEY_CHANGE_EVT                    0x0010
#define SBC_STATE_CHANGE_EVT                  0x0020
#ifdef TEST
#define SBC_SCAN_EVT                          0x0040
#define SBC_SCAN_STARTED_EVT                  0x0080
#endif //TEST


#define read_test                             Event_Id_10
#define Parking_init                          Event_Id_11
#define Echo                                  Event_Id_12
#define SafeNotify                            Event_Id_13
#define Position_return                       Event_Id_14

// Simple BLE Central Task Events
#define SBC_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define SBC_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30

#define SBC_ALL_EVENTS                        (SBC_ICALL_EVT           | \
                                               SBC_QUEUE_EVT           | \
                                               SBC_SCAN_EVT            | \
                                               SBC_SCAN_STARTED_EVT    | \
                                               SBC_START_DISCOVERY_EVT | \
                                               read_test               | \
                                               Parking_init            | \
                                               Echo                    | \
                                               Position_return         | \
                                               SafeNotify)

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  20

// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 4000

// Discovery mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

// TRUE to use high scan duty cycle when creating link
#define DEFAULT_LINK_HIGH_DUTY_CYCLE          FALSE

// TRUE to use white list when creating link
#define DEFAULT_LINK_WHITE_LIST               FALSE

// Default RSSI polling period in ms
#define DEFAULT_RSSI_PERIOD                   1000

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         GAPCENTRALROLE_PARAM_UPDATE_REQ_AUTO_ACCEPT

// Minimum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL      400

// Maximum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL      800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_UPDATE_SLAVE_LATENCY          0

// Supervision timeout value (units of 10ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_CONN_TIMEOUT           600

// Default passcode
#define DEFAULT_PASSCODE                      19655

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_DISPLAY_ONLY

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           1000

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          TRUE

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

// Type of Display to open
#if !defined(Display_DISABLE_ALL)
  #ifdef USE_CORE_SDK
    #if defined(BOARD_DISPLAY_USE_LCD) && (BOARD_DISPLAY_USE_LCD!=0)
      #define SBC_DISPLAY_TYPE Display_Type_LCD
    #elif defined (BOARD_DISPLAY_USE_UART) && (BOARD_DISPLAY_USE_UART!=0)
      #define SBC_DISPLAY_TYPE Display_Type_UART
    #else // !BOARD_DISPLAY_USE_LCD && !BOARD_DISPLAY_USE_UART
      #define SBC_DISPLAY_TYPE 0 // Option not supported
    #endif // BOARD_DISPLAY_USE_LCD && BOARD_DISPLAY_USE_UART
  #else // !USE_CORE_SDK
    #if !defined(BOARD_DISPLAY_EXCLUDE_LCD)
      #define SBC_DISPLAY_TYPE Display_Type_LCD
    #elif !defined (BOARD_DISPLAY_EXCLUDE_UART)
      #define SBC_DISPLAY_TYPE Display_Type_UART
    #else // BOARD_DISPLAY_EXCLUDE_LCD && BOARD_DISPLAY_EXCLUDE_UART
      #define SBC_DISPLAY_TYPE 0 // Option not supported
    #endif // BOARD_DISPLAY_EXCLUDE_LCD
  #endif // USE_CORE_SDK
#else // Display_DISABLE_ALL
  #define SBC_DISPLAY_TYPE 0 // No Display
#endif // Display_DISABLE_ALL

// Task configuration
#define SBC_TASK_PRIORITY                     1

#ifndef SBC_TASK_STACK_SIZE
#define SBC_TASK_STACK_SIZE                   864
#endif

// Application states
enum
{
  BLE_STATE_IDLE,
  BLE_STATE_CONNECTING,
  BLE_STATE_CONNECTED,
  BLE_STATE_DISCONNECTING
};

// Discovery states
enum
{
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_MTU,                 // Exchange ATT MTU size
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_CHAR                 // Characteristic discovery
};

// Key states for connections
typedef enum {
  GATT_RW,                 // Perform GATT Read/Write
  RSSI,                    // Toggle RSSI updates
  CONN_UPDATE,             // Send Connection Parameter Update
  DISCONNECT               // Disconnect
} keyPressConnOpt_t;

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr; // event header
  uint8_t *pData;  // event data
} sbcEvt_t;

// RSSI read data structure
typedef struct
{
  uint16_t period;      // how often to read RSSI
  uint16_t connHandle;  // connection handle
  Clock_Struct *pClock; // pointer to clock struct
} readRssi_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

#ifdef TEST

#define SCANNING_STARTED 1
#define SCANNING_STOPPED 0

#define SCAN_BNODE_RECORD_SIZE 4
#define SCAN_SIZE              100

static PIN_State  sbpPins;
static PIN_Handle hSbpPins;

#define RED_LED     IOID_1
#define GREEN_LED   IOID_0

PIN_Config gpioPinTable[] = {
  RED_LED     | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  GREEN_LED   | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  PIN_TERMINATE
};

static uint8_t scanning_status = SCANNING_STARTED;
//static uint16_t scan_num = 0;

typedef struct
{
  uint16_t  count;
  int16_t   totalRSSI;
  int8_t    averageRSSI;
  uint8_t   MacAddr[B_ADDR_LEN];
  int8      scanrssi[20];
  uint8   *scandata[18];
}bNode_data_t;

//static bNode_data_t scan_bNode_record[SCAN_BNODE_RECORD_SIZE] = {0};
//
//static uint8 *device_data = {0};
//static uint8_t scan_Mac[B_ADDR_LEN] = {0};

//static int8 scanrssi[20];
#endif //TEST

//UART
static UART_Handle UART_handle;
static UART_Params UART_params;
Semaphore_Struct semUARTRxStruct;
Semaphore_Handle uartSemHandle;

#define UartTimeout 100000 //unit in 10 us

//Mac Address
uint8 ownAddress[B_ADDR_LEN]={0};

/*************************************NB-IoT*************************************************************/
//static uint8_t  NB_Dead = 0;

//static uint8_t  NB_Config[9] ={0x24,0x04,0xD2,0x0D,0x72,0x24,0x39,0x09,0x7E};//預設雲淶 13.114.36.57 port2430
//static uint8_t  NB_Config[9] ={0x24,0x04,0xD2,0x77,0x17,0x11,0x73,0x1F,0x99};//預設華方 119.23.17.115 port8089
//static uint8_t  NB_Config[9] ={0x24,0x04,0xD2,0x8C,0x7C,0x20,0x6F,0x1E,0x61};//140.124.32.111,7777,65

static uint8_t  Read_Uart[200]={0x00};

//static uint8_t  NB_Reset[8]="AT+NRB\r\n";

//static uint8_t  CFUN_Close[11]="AT+CFUN=0\r\n";
static uint8_t  CFUN_Open [11]="AT+CFUN=1\r\n";

static uint8_t  Write_IMEI[11]="AT+CGSN=1\r\n";
static uint8_t  volatile IMEI[15]={0x00};

static uint8_t  Write_IMSI[9]="AT+CIMI\r\n";
static uint8_t  IMSI[15]={0x00};
//static uint8_t IMEIdata[15]={0x00};
//static uint8_t IMSI1[8]={0x00};

static uint8_t  CGDCONT_FET[27]={0X41,0X54,0X2B,0X43,0X47,0X44,0X43,0X4F,0X4E,0X54,0X3D,0X31,0X2C,0X22,0X49,0X50,0X22,0X2C,0X22,0X6E,0X62,0X69,0X6F,0X74,0X22,0X0D,0X0A};//nbiot 遠傳
static uint8_t  CGDCONT_CHT[34]={0X41,0X54,0X2B,0X43,0X47,0X44,0X43,0X4F,0X4E,0X54,0X3D,0X31,0X2C,0X22,0X49,0X50,0X22,0X2C,0X22,0X69,0X6E,0X74,0X65,0X72,0X6E,0X65,0X74,0X2E,0X69,0X6F,0X74,0X22,0X0D,0X0A}; //internet.iot 中華電信
static uint8_t  CGDCONT_CT [26]={0X41,0X54,0X2B,0X43,0X47,0X44,0X43,0X4F,0X4E,0X54,0X3D,0X31,0X2C,0X22,0X49,0X50,0X22,0X2C,0X22,0x63,0x74,0x6E,0x62,0X22,0X0D,0X0A};//ctnb 中國電信

//static uint8_t  Cops_Close[11]="AT+Cops=2\r\n";
static uint8_t  Cops_FET  [21]={0x41,0x54,0x2B,0x43,0x6F,0x70,0x73,0x3D,0x31,0x2C,0x32,0x2C,0x22,0x34,0x36,0x36,0x30,0x31,0x22,0X0D,0X0A};
static uint8_t  Cops_CHT  [21]={0x41,0x54,0x2B,0x43,0x6F,0x70,0x73,0x3D,0x31,0x2C,0x32,0x2C,0x22,0x34,0x36,0x36,0x39,0x32,0x22,0X0D,0X0A};
static uint8_t  Cops_CT   [21]={0x41,0x54,0x2B,0x43,0x6F,0x70,0x73,0x3D,0x31,0x2C,0x32,0x2C,0x22,0x34,0x36,0x30,0x31,0x31,0x22,0X0D,0X0A};

static uint8_t  Lock_Band[17]="AT+NBAND=5,8,28\r\n";

//static uint8_t  CGATT_Close[12]={"AT+CGATT=0\r\n"};
static uint8_t  CGATT_Open[12]={"AT+CGATT=1\r\n"};
static uint8_t  Write_CGATT[11]={"AT+CGATT?\r\n"};
static uint8_t  GATT_Error = 0;


static uint8_t  Write_CSQ[8]="AT+CSQ\r\n";
static uint8_t  CSQ[2]={0x00};
//static uint8_t  CSQ1[2]={0x00};
//static uint8_t  CSQdata[1]={0x00};

static uint8_t  REG_Open[12]="AT+CEREG=1\r\n";
static uint8_t  Write_REG[11]="AT+CEREG?\r\n";
static uint8_t  REG         = 0x00;

//static uint8_t  Write_NUESTATS[13]="AT+NUESTATS\r\n";

uint8_t  NB_IoT_RSSI[5]={0x00};
uint8_t  NB_IoT_Cell_ID[8]={0x00};
uint8_t  NB_IoT_PCI[3]={0x00};
uint8_t  NB_IoT_SNR[2]={0x00};

uint8_t  Socket_Number = 0x30;

static uint8_t  NSMI[11]={"AT+NSMI=1\r\n"}; //啟動Echo

//static uint8_t  PSM_Mode_Config[32]={"AT+CPSMS=1,,,00101000,00000101\r\n"};
//static uint8_t  PSM_Mode_Open[12]={"AT+NPSMR=1\r\n"};
//static uint8_t  IDle_Mode_Open[12]={"AT+CSCON=1\r\n"};

//static uint8_t  Time__NSORF[15]="AT+NSORF=3,14\r\n";

//static uint8_t  Write_NSORF[15]="AT+NSORF=1,14\r\n";
//static uint8_t  Read_NSORF[50]={0x00}; //42

//static uint8_t  NBIOT_Data     [172]={0x00}; 
//static uint8_t  NBIOT_Data_Read[35] ={0x00}; //30
//static uint16_t NBIoT_Data_Count    =0x0000;
//static uint8_t  NBIOT_Judge_Data[58]={0x00}; 
//static uint8_t  NBIOT_Wave_Data[247]={0x00};


//static uint8_t  NSOCR_UDP[27]={"AT+NSOCR=DGRAM,17,"};

//static uint8_t  NSOCR_Judge[27]="AT+NSOCR=DGRAM,17,05678,1\r\n";
//static uint8_t  NSOCR_Wave[27]="AT+NSOCR=DGRAM,17,09012,1\r\n";


//static uint8_t  NSOST_UDP[40]={0x00};
//static uint8_t  NSOST_Judge[32]="AT+NSOST=2,13.114.36.57,2403,12,"; //前面指令設定 port:2403 //參數用
//static uint8_t  NSOST_Wave[33]="AT+NSOST=3,13.114.36.57,2499,106,"; //前面指令設定 port:2499 //波型用
//static uint8_t  NSOST_Judge[34]="AT+NSOST=1,140.124.32.111,7777,65,";

//static uint8_t  NSOCL_UDP  [12]="AT+NSOCL=1\r\n";
//static uint8_t  NSOCL_Judge[12]="AT+NSOCL=2\r\n";
//static uint8_t  NSOCL_Wave [12]="AT+NSOCL=3\r\n";

//static uint8_t  Time_Correction[49]="AT+NSOST=1,13.114.36.57,1101,8,6461746574696D65\r\n";
/*
//static uint8_t close_ECHO = false;
//static uint8_t Write_CFUN_Close[11]="AT+CFUN=0\r\n";
//static uint8_t Write_CFUN_Open[11]="AT+CFUN=1\r\n";

static uint8_t CGDCONT_FET[27]={0X41,0X54,0X2B,0X43,0X47,0X44,0X43,0X4F,0X4E,0X54,0X3D,0X31,0X2C,0X22,0X49,0X50,0X22,0X2C,0X22,0X6E,0X62,0X69,0X6F,0X74,0X22,0X0D,0X0A};//nbiot 遠傳
static uint8_t CGDCONT_CHT[34]={0X41,0X54,0X2B,0X43,0X47,0X44,0X43,0X4F,0X4E,0X54,0X3D,0X31,0X2C,0X22,0X49,0X50,0X22,0X2C,0X22,0X69,0X6E,0X74,0X65,0X72,0X6E,0X65,0X74,0X2E,0X69,0X6F,0X74,0X22, 0X0D,0X0A}; //internet.iot 中華

static uint8_t Write_CIMI[9]="AT+CIMI\r\n";
static uint8_t  Read_CIMI[25]={0x00};
static uint8_t  CIMI[15]={0x00};

static uint8_t Write_CSQ[8]="AT+CSQ\r\n";

static uint8_t Write_REG[11]="AT+CEREG?\r\n";

static uint8_t CGATT[12]="AT+CGATT=1\r\n";
static uint8_t Write_CGATT[11]="AT+CGATT?\r\n";

static uint8_t NSMI[11]="AT+NSMI=1\r\n";  //Sending message indications is enabled

//static uint8_t Write_NSORF[14]="AT+NSORF=1,4\r\n"; //Read Echo





//static uint16_t NBIoT_Data_Count=0x00; 
*/
//static uint8_t Read_Uart[50]={0x00};
static uint8_t NBIOT_Data1[78]={0x00};
static uint8_t NBIOT_Data2[86]={0x00};
static uint8_t NSOCR[26]="AT+NSOCR=DGRAM,17,9216,1\r\n";
static uint8_t NSOST_UDP1[34]="AT+NSOST=1,140.124.32.111,5678,21,";
static uint8_t NSOST_UDP2[34]="AT+NSOST=1,140.124.32.111,5678,25,";
//static uint8_t  NB_Config[9] ={0x24,0x04,0xD2,0x8C,0x7C,0x20,0x6F,0x16,0x2E};//140.124.32.111,5678,22
//static uint8_t  NSOST_UDP1[40]={0x00};
//static uint8_t  NSOST_UDP2[40]={0x00};

static uint8_t NBIOT_Data_Read[30]={0x00};
static uint8_t volatile NB_flag = 0;

static uint8_t NBIot_Init_Ok = 0;
//static uint8_t NB_Control_flag= 1;
static uint8_t NBIoT_Data_Count = 0x0000;
uint8_t  CFUN_OK_Flag     = 0;
uint8_t  IMEI_Ok_Flag     = 0;
uint8_t  IMSI_Ok_Flag     = 0;
uint8_t CIMI_Ok_Flag = 0;
uint8_t APN_Ok_Flag = 0;
uint8_t CSQ_Ok_Flag = 0;
uint8_t REG_Ok_Flag = 0;
uint8_t GATT_Ok_Flag = 0;

/*********************************Batt*********************************************/
uint8_t batVol_H;
uint8_t batVol_L;
uint32_t batVol;
float batVol_LL;
float batVol_HH;
float Batt;
int BattA;
int BattD;

static uint8_t volatile Safe_flag = 0;
static uint8_t volatile Position_flag = 0;
static uint8_t position[1] ={0x00};

static uint8_t advertData[19] = {0x00};
static uint8_t routedata[13] = {0x00};
static uint8_t routedata2[26] = {0x00};
static uint8_t gatewaydata[8] = {0x00};
static uint8_t gatewaydata2[16] = {0x00};
static uint8_t bicycle_number = 0;

static uint8_t slave_advdata[31] = {0x00};
static uint8_t safedata[18] = {0x00};
static uint8_t safedata2[36] = {0x00};
static uint8_t master_safedata[7] = {0x00};
static uint8_t master_safedata2[14] = {0x00}; 

/***********SVM******************/
float D     =0;
float D_sum =0;

static int sigma = 4;
const float Base =0.030223879;
float Alpha_Y[200]={0,0.2822,0.6553,2.2585,2.7771,3,0.7886,0.2715,3,3,0.8374,0.034,0,0.9453,1.0676,0.2791,1.0365,0.6944,1.5401,2.3072,1.2942,0,1.0907,0,0.3678,0.5093,0,0.4481,0,0.2618,1.2173,3,1.6961,1.3476,2.8382,1.003,0.605,0,0.4008,0.471,0.5295,0.0896,0.7523,0.5598,0.7955,0.3416,0,0,0.4823,0.7993,0.7862,0.5729,0.7427,0.9305,0.4164,0.8203,0.7397,0,0.8622,0.9313,0.7862,1.2751,0.1532,0,1.0738,0.5128,0.6214,1.108,0.4181,0,0.1189,0.4821,0.7006,0.1519,0.1133,0.3554,1.5704,0.104,0.7847,0.8276,0.553,0.4371,0.6896,0.3882,0.3975,0.6397,1.6863,0.1436,0.6064,0,1.1972,0,0.1999,0.734,0.1057,0.164,0.6726,0.508,0.3482,0.8731,-0.7077,-0.5748,-0.5315,-0.3215,-0.4703,-1.0137,-0.7728,0,0,-1.1725,-0.4352,-0.9569,-0.4663,-2.3624,-0.2866,-0.3841,-0.4874,-2.7173,-1.1107,-0.5996,-0.7791,0,-0.0247,0,-0.7715,-0.0944,-0.2614,0,0,-0.0383,-0.0255,-0.3509,-0.7731,-0.7271,-0.3064,-0.9414,-0.5174,-0.0683,-0.8068,-1.1074,-0.195,-0.2923,-0.8853,-0.703,-0.7973,-0.3342,-0.8232,-0.7547,-0.9549,-0.9983,-0.8457,-0.1587,0,-1.7926,-0.5639,-0.0967,-0.5565,0,-3,0,-0.3428,-1.4312,0,-0.3347,0,-0.0304,0,-0.1297,-2.2855,-1.2393,-0.0641,-0.5628,-1.0517,0,0,-0.5165,-0.3425,-0.5723,-0.5619,-0.5096,-1.3977,-3,-2.8868,-1.9301,-0.7242,-1.7591,-3,-0.4036,-0.7207,-1.4372,-0.2782,-0.7087,-0.843,-3,-0.4871,-1.3174,-1.1763,0,-0.3777,-1.839};

int Trainingdata1[200]={87,83,82,84,81,85,83,83,92,93,88,88,89,86,87,84,81,89,83,87,90,88,78,77,80,76,87,89,80,88,78,77,76,76,88,76,88,76,77,83,72,68,69,68,69,68,71,68,68,70,52,59,62,54,62,51,62,62,61,65,74,76,73,69,68,65,69,64,71,70,83,70,69,85,73,69,81,70,86,71,89,78,77,74,69,81,90,81,77,86,87,85,85,86,85,83,83,85,84,91,93,95,85,92,88,93,86,86,93,79,78,77,81,77,81,72,78,72,70,70,76,88,83,83,83,84,77,89,88,82,80,85,82,82,74,82,80,83,82,82,80,88,73,77,80,71,73,84,83,76,75,77,75,88,87,86,86,86,78,87,84,82,87,88,88,85,86,87,84,90,92,93,91,88,88,91,89,91,94,92,82,83,91,83,90,93,84,89,85,87,92,94,97,92,93,90,98,92,93,90};
int Trainingdata2[200]={92,92,91,88,92,94,92,91,91,92,93,94,91,93,90,91,92,90,94,94,88,87,85,86,86,84,88,88,88,86,89,91,87,90,87,89,86,85,91,88,83,80,97,84,81,92,84,85,91,94,78,77,80,77,80,83,90,95,96,92,74,82,79,83,81,84,85,84,87,81,69,68,71,70,68,73,65,70,70,66,67,59,61,71,66,67,67,62,68,68,58,57,48,58,50,57,48,57,56,46,86,86,84,86,85,93,91,84,91,87,83,82,92,91,87,83,89,86,93,78,73,77,77,79,75,83,78,78,78,80,81,77,75,92,77,76,88,77,87,88,79,81,79,79,83,78,79,78,70,67,89,79,81,85,83,77,84,85,93,84,73,61,71,68,72,69,72,65,68,65,77,82,65,77,73,75,76,72,79,73,87,93,88,96,88,92,91,98,97,97,94,87,93,94,94,87,92,91,90,94};
int Trainingdata3[200]={89,87,78,90,91,80,91,82,82,84,86,86,82,85,93,81,77,79,84,88,76,74,75,71,72,71,76,70,76,70,79,78,80,89,81,76,75,72,73,77,70,70,61,72,61,68,71,71,63,68,67,59,64,67,59,61,72,66,66,55,65,71,70,71,76,77,69,81,71,68,69,78,75,69,75,70,78,71,74,68,72,77,75,69,75,69,77,73,80,71,88,82,79,81,81,82,82,88,90,77,84,92,94,91,87,80,94,92,91,91,86,76,83,74,86,81,70,78,86,83,90,93,90,93,95,92,86,88,87,85,80,80,84,83,82,76,85,84,78,83,83,93,77,86,93,78,77,87,82,91,83,82,83,81,84,87,83,83,80,83,87,87,87,87,85,86,85,85,79,80,90,89,85,82,92,80,94,92,82,85,80,82,82,92,93,86,90,87,83,83,86,88,87,82,87,81,81,89,90,90};
int Trainingdata4[200]={56,51,51,65,66,66,48,50,69,65,58,57,56,69,58,56,66,57,70,71,68,74,66,75,78,70,78,75,77,74,77,86,85,81,82,84,77,74,75,83,76,85,78,86,88,78,77,78,78,85,83,84,80,93,86,84,83,86,87,85,79,88,88,81,81,79,91,90,80,88,87,94,86,86,94,84,91,87,88,94,86,91,84,83,95,83,94,90,90,87,89,85,85,85,86,84,84,81,84,86,79,73,81,82,78,80,82,82,75,73,93,93,88,88,91,90,88,84,85,87,86,87,87,86,83,84,87,87,87,90,91,88,78,81,83,80,72,85,90,75,68,87,66,66,90,67,86,68,67,77,95,85,87,86,94,89,85,92,84,85,87,91,87,87,86,89,87,90,93,87,87,89,97,88,88,89,88,88,91,85,64,67,67,66,69,68,64,68,64,67,70,65,89,66,82,75,71,80,86,73};

static float kernel[200]={0};
static int volatile Testdata[4]={57,65,63,80};
static uint8_t Testdata1[4]={0x00};



enum
{
  Init_State,      //0x00
  Outside_State,   //0x01
  Inside_State,    //0x02
};

static uint8_t License_State=0;
/*************************************************************************************/
static int8 volatile testdata[4];
//float testdata[4] = {1.05E-12,9.63E-13,2.53E-12,8.82E-13};
//int testdata1[4] = {0,0,0,0};

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Clock object used to signal timeout
static Clock_Struct startDiscClock;

static Clock_Struct periodicClock;

static Clock_Struct myClock_read_test;
static Clock_Struct myClock_init;
static Clock_Struct myClock_Echo;
static Clock_Struct myClock_safe;
static Clock_Struct myClock_return;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task configuration
Task_Struct sbcTask;
Char sbcTaskStack[SBC_TASK_STACK_SIZE];

// GAP GATT Attributes
static const uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Simple BLE Central";

// Number of scan results and scan result index
static uint8_t scanRes = 0;
static int8_t scanIdx = -1;

// Scan result list
static gapDevRec_t devList[DEFAULT_MAX_SCAN_RES];

// Scanning state
static bool scanningStarted = FALSE;

// Connection handle of current connection
static uint16_t connHandle = GAP_CONNHANDLE_INIT;

// Application state
static uint8_t state = BLE_STATE_IDLE;

// Discovery state
static uint8_t discState = BLE_DISC_STATE_IDLE;

// Discovered service start and end handle
static uint16_t svcStartHdl = 0;
static uint16_t svcEndHdl = 0;

// Discovered characteristic handle
static uint16_t charHdl = 0;

// Value to write
static uint8_t charVal = 0;

// Value read/write toggle
static bool doWrite = FALSE;

// GATT read/write procedure state
static bool procedureInProgress = FALSE;

// Maximum PDU size (default = 27 octets)
static uint16 maxPduSize;

// Array of RSSI read structures
static readRssi_t readRssi[MAX_NUM_BLE_CONNS];

// Key option state.
static keyPressConnOpt_t keyPressConnOpt = DISCONNECT;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

//自定義函數
static void NBIoT_Init(void);
static void NBIoT_Data(void);
static void NBIoT_Safe(void);
static void Voltage(void);
static void SVM();
// 讀mac//
static void Read_Mac(uint8 *Mac_Address);

static void SimpleBLECentral_init(void);
static void SimpleBLECentral_taskFxn(UArg a0, UArg a1);

static void SimpleBLECentral_processGATTMsg(gattMsgEvent_t *pMsg);
static void SimpleBLECentral_handleKeys(uint8_t shift, uint8_t keys);
static void SimpleBLECentral_processStackMsg(ICall_Hdr *pMsg);
static void SimpleBLECentral_processAppMsg(sbcEvt_t *pMsg);
static void SimpleBLECentral_processRoleEvent(gapCentralRoleEvent_t *pEvent);
static void SimpleBLECentral_processGATTDiscEvent(gattMsgEvent_t *pMsg);
static void SimpleBLECentral_startDiscovery(void);
static bool SimpleBLECentral_findSvcUuid(uint16_t uuid, uint8_t *pData,
                                         uint8_t dataLen);
static void SimpleBLECentral_addDeviceInfo(uint8_t *pAddr, uint8_t addrType);
static void SimpleBLECentral_processPairState(uint8_t state, uint8_t status);
static void SimpleBLECentral_processPasscode(uint16_t connectionHandle,
                                             uint8_t uiOutputs);

static void SimpleBLECentral_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg);
static bStatus_t SimpleBLECentral_StartRssi(uint16_t connHandle, uint16_t period);
static bStatus_t SimpleBLECentral_CancelRssi(uint16_t connHandle);
static readRssi_t *SimpleBLECentral_RssiAlloc(uint16_t connHandle);
static readRssi_t *SimpleBLECentral_RssiFind(uint16_t connHandle);
static void SimpleBLECentral_RssiFree(uint16_t connHandle);

static uint8_t SimpleBLECentral_eventCB(gapCentralRoleEvent_t *pEvent);
static void SimpleBLECentral_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs);
static void SimpleBLECentral_pairStateCB(uint16_t connHandle, uint8_t state,
                                         uint8_t status);

void SimpleBLECentral_startDiscHandler(UArg a0);
void SimpleBLECentral_keyChangeHandler(uint8 keys);
void SimpleBLECentral_readRssiHandler(UArg a0);

static uint8_t SimpleBLECentral_enqueueMsg(uint8_t event, uint8_t status,
                                           uint8_t *pData);

static void UARTreadCallback(UART_Handle handle, void *buf, size_t count);

#ifdef FPGA_AUTO_CONNECT
static void SimpleBLECentral_startGapDiscovery(void);
static void SimpleBLECentral_connectToFirstDevice(void);
#endif // FPGA_AUTO_CONNECT

/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapCentralRoleCB_t SimpleBLECentral_roleCB =
{
  SimpleBLECentral_eventCB     // Event callback
};

// Bond Manager Callbacks
static gapBondCBs_t SimpleBLECentral_bondCB =
{
  (pfnPasscodeCB_t)SimpleBLECentral_passcodeCB, // Passcode callback
  SimpleBLECentral_pairStateCB                  // Pairing state callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

void Hex_to_Ascii(uint8_t hex, uint8_t *data)
{
  uint8_t byte;

  // High byte
  byte = (hex & 0xF0) >> 4;
  data[0] = (byte > 9) ? (byte + 0x37) : (byte + 0x30);

  // Low byte
  byte = (hex & 0x0F);
  data[1] = (byte > 9) ? (byte + 0x37) : (byte + 0x30);
}

 void Hex_to_Dec(uint8_t hex, uint8_t *data)
 {
   uint8_t byte;
   
   //High byte
   byte = hex >> 4;
   data[0] = byte;
   
   //Low byte
   byte = (hex & 0X0F);
   data[1] = byte; 
   
   data[2]=data[0]*10+data[1];
   data[3]=data[0]+0X30;
   data[4]=data[1]+0X30;
 }
////PIN
//static void buttonHwiFxn(PIN_Handle hPin, PIN_Id pinId)
//{
//  // set event in SBP task to process outside of hwi context
//  
//  Event_post(syncEvent, SBP_PERIODIC_EVT); // Add
//  
//}

#ifdef FPGA_AUTO_CONNECT
/*********************************************************************
 * @fn      SimpleBLECentral_startGapDiscovery
 *
 * @brief   Start discovering devices
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleBLECentral_startGapDiscovery(void)
{
  // Start discovery
  if ((state != BLE_STATE_CONNECTED) && (!scanningStarted))
  {
    scanningStarted = TRUE;
    scanRes = 0;

    Display_print0(dispHandle, 2, 0, "Discovering...");
    Display_clearLines(dispHandle, 3, 4);

    GAPCentralRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
                                  DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                  DEFAULT_DISCOVERY_WHITE_LIST);
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_connectToFirstDevice
 *
 * @brief   Connect to first device in list of discovered devices
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleBLECentral_connectToFirstDevice(void)
{
  uint8_t addrType;
  uint8_t *peerAddr;

  scanIdx = 0;

  if (state == BLE_STATE_IDLE)
  {
    // connect to current device in scan result
    peerAddr = devList[scanIdx].addr;
    addrType = devList[scanIdx].addrType;

    state = BLE_STATE_CONNECTING;

    GAPCentralRole_EstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                 DEFAULT_LINK_WHITE_LIST,
                                 addrType, peerAddr);

    Display_print0(dispHandle, 2, 0, "Connecting");
    Display_print0(dispHandle, 3, 0, Util_convertBdAddr2Str(peerAddr));
    Display_clearLine(dispHandle, 4);
  }
}
#endif // FPGA_AUTO_CONNECT

/*********************************************************************
 * @fn      SimpleBLEPeripheral_createTask
 *
 * @brief   Task creation function for the Simple BLE Peripheral.
 *
 * @param   none
 *
 * @return  none
 */
void SimpleBLECentral_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sbcTaskStack;
  taskParams.stackSize = SBC_TASK_STACK_SIZE;
  taskParams.priority = SBC_TASK_PRIORITY;

  Task_construct(&sbcTask, SimpleBLECentral_taskFxn, &taskParams, NULL);
}

/****************************PIN*/
static PIN_Config SBP_configTable[] =
{
//  Board_LED_R | PIN_GPIO_OUTPUT_EN  | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
//  Board_LED_G | PIN_GPIO_OUTPUT_EN  | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
//  Board_LED_B | PIN_GPIO_OUTPUT_EN  | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
//   
//  LoRa_WakeUp | PIN_GPIO_OUTPUT_EN  | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
//  LoRa_Sleep  | PIN_GPIO_OUTPUT_EN  | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
//  LoRa_Set    | PIN_GPIO_OUTPUT_EN  | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  
  //Reset | PIN_GPIO_OUTPUT_EN  | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,  
  //AMR_Reset   | PIN_GPIO_OUTPUT_EN  | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  
  //IR_SW       | PIN_GPIO_OUTPUT_EN  | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  
  NBIoT_RI    | PIN_GPIO_OUTPUT_DIS | PIN_INPUT_EN | PIN_NOPULL, 
  AMR_DRDY    | PIN_GPIO_OUTPUT_DIS | PIN_INPUT_EN | PIN_NOPULL,  
  Solar       | PIN_GPIO_OUTPUT_DIS | PIN_INPUT_EN | PIN_NOPULL,
  //IR_Read     | PIN_GPIO_OUTPUT_DIS | PIN_INPUT_EN | PIN_NOPULL,  
  
  PIN_TERMINATE //PIN END
};
static PIN_State sbpPins;
static PIN_Handle hSbpPins;

/*********************************************************************
 * @fn      SimpleBLECentral_Init
 *
 * @brief   Initialization function for the Simple BLE Central App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleBLECentral_init(void)
{
  
  Read_Mac(ownAddress);
  
  //UART
  Semaphore_Params semParams;
  Semaphore_Params_init(&semParams);
  Semaphore_construct(&semUARTRxStruct, 0, &semParams);
  uartSemHandle = Semaphore_handle(&semUARTRxStruct);

  UART_init();
  UART_Params_init(&UART_params);
  UART_params.baudRate = 9600;
  UART_params.readMode = UART_MODE_CALLBACK;
  UART_params.readCallback   = UARTreadCallback;
  UART_params.writeTimeout   = UartTimeout;
  UART_handle = UART_open(Board_UART0, &UART_params); 
  //UART_readCancel(UART_handle);
  UARTCC26XX_control(UART_handle, UARTCC26XX_CMD_RETURN_PARTIAL_ENABLE, NULL);
  
  //PIN
  // Open pin structure for use
  hSbpPins = PIN_open(&sbpPins, SBP_configTable);
  // Register ISR
  //PIN_registerIntCb(hSbpPins, buttonHwiFxn);
  // Configure interrupt
  PIN_setConfig(hSbpPins, PIN_BM_IRQ, Board_KEY_UP | PIN_IRQ_NEGEDGE);
  // Enable wakeup
  PIN_setConfig(hSbpPins, PINCC26XX_BM_WAKEUP, Board_KEY_UP|PINCC26XX_WAKEUP_NEGEDGE);
  
  uint8_t i;

  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

#if defined( USE_FPGA )
  // configure RF Core SMI Data Link
  IOCPortConfigureSet(IOID_12, IOC_PORT_RFC_GPO0, IOC_STD_OUTPUT);
  IOCPortConfigureSet(IOID_11, IOC_PORT_RFC_GPI0, IOC_STD_INPUT);

  // configure RF Core SMI Command Link
  IOCPortConfigureSet(IOID_10, IOC_IOCFG0_PORT_ID_RFC_SMI_CL_OUT, IOC_STD_OUTPUT);
  IOCPortConfigureSet(IOID_9, IOC_IOCFG0_PORT_ID_RFC_SMI_CL_IN, IOC_STD_INPUT);

  // configure RF Core tracer IO
  IOCPortConfigureSet(IOID_8, IOC_PORT_RFC_TRC, IOC_STD_OUTPUT);
#else // !USE_FPGA
  #if defined( DEBUG_SW_TRACE )
    // configure RF Core tracer IO
    IOCPortConfigureSet(IOID_8, IOC_PORT_RFC_TRC, IOC_STD_OUTPUT | IOC_CURRENT_4MA | IOC_SLEW_ENABLE);
  #endif // DEBUG_SW_TRACE
#endif // USE_FPGA

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Setup discovery delay as a one-shot timer
  Util_constructClock(&startDiscClock, SimpleBLECentral_startDiscHandler,
                      DEFAULT_SVC_DISCOVERY_DELAY, 0, false, 0);

  Board_initKeys(SimpleBLECentral_keyChangeHandler);

  // Open Display.
  dispHandle = Display_open(SBC_DISPLAY_TYPE, NULL);

  // Initialize internal data
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    readRssi[i].connHandle = GAP_CONNHANDLE_ALL;
    readRssi[i].pClock = NULL;
  }

  // Setup Central Profile
  {
    uint8_t scanRes = DEFAULT_MAX_SCAN_RES;

    GAPCentralRole_SetParameter(GAPCENTRALROLE_MAX_SCAN_RES, sizeof(uint8_t),
                                &scanRes);
  }

  // Setup GAP
  GAP_SetParamValue(TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION);
  GAP_SetParamValue(TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION);
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN,
                   (void *)attDeviceName);

  // Setup the GAP Bond Manager
  {
    uint32_t passkey = DEFAULT_PASSCODE;
    uint8_t pairMode = DEFAULT_PAIRING_MODE;
    uint8_t mitm = DEFAULT_MITM_MODE;
    uint8_t ioCap = DEFAULT_IO_CAPABILITIES;
    uint8_t bonding = DEFAULT_BONDING_MODE;

    GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t),
                            &passkey);
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
  }

  // Initialize GATT Client
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd(selfEntity);

  // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);         // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES); // GATT attributes

  // Start the Device
  VOID GAPCentralRole_StartDevice(&SimpleBLECentral_roleCB);

  // Register with bond manager after starting device
  GAPBondMgr_Register(&SimpleBLECentral_bondCB);

  // Register with GAP for HCI/Host messages (for RSSI)
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  Display_print0(dispHandle, 0, 0, "BLE Central");
  
  Util_constructClock(&myClock_read_test, SimpleBLECentral_startDiscHandler,1000,1000,false, read_test ); //主程式0.1S
  Util_constructClock(&myClock_init,      SimpleBLECentral_startDiscHandler,1000,1000,false,Parking_init); //初始事件 1S  
//Util_constructClock(&myClock_Echo,      SimpleBLECentral_startDiscHandler,1000,1000,false,Echo); //初始事件 1S  
//Util_constructClock(&myClock_safe,      SimpleBLECentral_startDiscHandler,1800000,1800000,false,SafeNotify );//報平安
  Util_constructClock(&myClock_return, SimpleBLECentral_startDiscHandler,1000,1000,false, Position_return );

Util_startClock(&myClock_init); 
}

/*********************************************************************
 * @fn      SimpleBLECentral_taskFxn
 *
 * @brief   Application task entry point for the Simple BLE Central.
 *
 * @param   none
 *
 * @return  events not processed
 */
static void SimpleBLECentral_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  SimpleBLECentral_init();

#ifdef TEST
  //uint8_t index = 0;
  
  hSbpPins=PIN_open(&sbpPins, gpioPinTable);
  //PIN_setOutputValue(hSbpPins, RED_LED, 0);
  PIN_setOutputValue(hSbpPins, GREEN_LED, 0);

  GAPCentralRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
                                DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                DEFAULT_DISCOVERY_WHITE_LIST);
  
  Testdata1[0] =0x58;
  Testdata1[1] =0x62;
  Testdata1[2] =0x56;
  Testdata1[3] =0x80;
  SVM();
#endif //TEST
  
  // Application main loop
  for (;;)
  {
    uint32_t events;

    events = Event_pend(syncEvent, Event_Id_NONE, SBC_ALL_EVENTS,
                        ICALL_TIMEOUT_FOREVER);

    if (events)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          // Process inter-task message
          SimpleBLECentral_processStackMsg((ICall_Hdr *)pMsg);
        }

        if (pMsg)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message
      if (events & SBC_QUEUE_EVT)
      {
        while (!Queue_empty(appMsgQueue))
        {
          sbcEvt_t *pMsg = (sbcEvt_t *)Util_dequeueMsg(appMsgQueue);
          if (pMsg)
          {
            // Process message
            SimpleBLECentral_processAppMsg(pMsg);

            // Free the space from the message
            ICall_free(pMsg);
          }
        }
      }
      
      if (events & Parking_init)
      {
         events &= ~Parking_init;
        
         //Voltage();    
         NBIoT_Init();       //NBIOT初始化
         PIN_setOutputValue(hSbpPins, Board_LED_B ,1);
        
        if(NBIot_Init_Ok == 1)
        {                   
//          PIN_setOutputValue(hSbpPins, Board_LED_G ,0);
//          PIN_setOutputValue(hSbpPins, Board_LED_R ,0);    
//          PIN_setOutputValue(hSbpPins, Board_LED_B ,0);            
//          PIN_setOutputValue(hSbpPins, Board_LED_W ,0);   
          
          Util_stopClock(&myClock_init);
          Util_startClock(&myClock_read_test); 
        }  
      } 
      
       if (events & read_test)
      {
         events &= ~read_test;                

           GAPCentralRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
                         DEFAULT_DISCOVERY_ACTIVE_SCAN,
                         DEFAULT_DISCOVERY_WHITE_LIST); 

         Voltage();
         //Time();
         
          
       if(NB_flag == 1)
       {
//         for(int x=0; x<4; x++)
//         {
//           Testdata1[x]=routedata[x+8];
//           //Testdata[4] = 0x57;    ///Rssi=87
//         }
         memcpy(&Testdata1[0], &routedata[8], 4);
         
          char a[4]={0x00}; 
         if(memcmp(a,Testdata1,4)!=0)
           {
             SVM();
           }
         
         //SVM();
         if(License_State != 0)
         {
           NBIoT_Data();
         
         
           UART_read(UART_handle,Read_Uart, sizeof(Read_Uart));
           UART_write(UART_handle,Write_CSQ,sizeof(Write_CSQ)); 
           Semaphore_pend(uartSemHandle, UartTimeout); 
           memset(Read_Uart , 0 , sizeof(Read_Uart));
           
           UART_read(UART_handle,NBIOT_Data_Read, sizeof(NBIOT_Data_Read));  
           UART_write(UART_handle,NBIOT_Data1,sizeof(NBIOT_Data1));
           Semaphore_pend(uartSemHandle, UartTimeout);
           NB_flag = 0;
           License_State = 0;

//           Util_stopClock(&myClock_read_test);
//           Util_startClock(&myClock_init);        
         }
         
       }
       else if (Safe_flag == 1)
       {
         
         NBIoT_Safe();
         
         UART_read(UART_handle,NBIOT_Data_Read, sizeof(NBIOT_Data_Read));  
         UART_write(UART_handle,NBIOT_Data2,sizeof(NBIOT_Data2));
         Semaphore_pend(uartSemHandle, UartTimeout);
         Safe_flag = 0;
       }
       
         else 
         {
           memset(advertData , 0 , sizeof(advertData));
           memset(routedata , 0 , sizeof(routedata));
           memset(slave_advdata, 0, sizeof(slave_advdata));
           memset(safedata, 0, sizeof(safedata));
//            UART_read(UART_handle,NBIOT_Data_Read, sizeof(NBIOT_Data_Read));  
//            UART_write(UART_handle,NBIOT_Data,sizeof(NBIOT_Data));
//            Semaphore_pend(uartSemHandle, UartTimeout);  
//           Util_stopClock(&myClock_read_test);
//           Util_startClock(&myClock_init);
           Util_startClock(&myClock_return);
         }    
         }
 
#ifdef SCAN
      if(events & SBC_SCAN_EVT)
      {
        events &= ~SBC_SCAN_EVT;

        //PIN_setOutputValue(hSbpPins, RED_LED, 1);

        for(index=0; index < SCAN_BNODE_RECORD_SIZE; index++)
        {
          if(scan_bNode_record[index].count != 0)
          {
            if(memcmp(scan_bNode_record[index].MacAddr,scan_Mac,6) == 0)
            {
                scan_bNode_record[index].count++;
                for(int x=0; x < 18; x++)
                {
                  if(scan_bNode_record[index].scandata[x] == 0)
                  {
                    scan_bNode_record[index].scandata[x] = device_data;
                    break;
                  }
                }
                
                break;
            }
          }
        }
        
        
        if(index == SCAN_BNODE_RECORD_SIZE)
        {
          for(index=0; index < SCAN_BNODE_RECORD_SIZE; index++)
          { 
            if(scan_bNode_record[index].count == 0)
            {
              memcpy(scan_bNode_record[index].MacAddr, scan_Mac, 6);
              scan_bNode_record[index].count++;
              for(int x=0; x < 18; x++)
                {
                  if(scan_bNode_record[index].scandata[x] == 0)
                  {
                    scan_bNode_record[index].scandata[x] = device_data;
                    break;
                  }
                }
              break;
            } 
          }
        }
        
        if(scanning_status == SCANNING_STARTED)
        {
          scanning_status = SCANNING_STARTED;
          GAPCentralRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
                                        DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                        DEFAULT_DISCOVERY_WHITE_LIST);
        }
      }
      else if(events & SBC_SCAN_STARTED_EVT)
      {
        events &= ~SBC_SCAN_STARTED_EVT;
        if(scanning_status == SCANNING_STARTED)
        {
          scanning_status = SCANNING_STARTED;
          GAPCentralRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
                                        DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                        DEFAULT_DISCOVERY_WHITE_LIST);
        }
      }
#endif //SCAN

      if (events & SBC_START_DISCOVERY_EVT)
      {
        SimpleBLECentral_startDiscovery();
      }
      
//      if (events & Position_return)
//      {
//        events &= ~Position_return;
//        
//        if(Position_flag == 1)
//        {
//          UART_read(UART_handle,Read_Uart, sizeof(Read_Uart));  
//          UART_write(UART_handle,position,sizeof(position));
//          
//          Position_flag = 0;
//        }
//      }
    }
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_processStackMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void SimpleBLECentral_processStackMsg(ICall_Hdr *pMsg)
{
  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      SimpleBLECentral_processRoleEvent((gapCentralRoleEvent_t *)pMsg);
      break;

    case GATT_MSG_EVENT:
      SimpleBLECentral_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
      {
        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            SimpleBLECentral_processCmdCompleteEvt((hciEvt_CmdComplete_t *)pMsg);
            break;

          case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
            AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
            break;
      
          default:
            break;
        }
      }
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_processAppMsg
 *
 * @brief   Central application event processing function.
 *
 * @param   pMsg - pointer to event structure
 *
 * @return  none
 */
static void SimpleBLECentral_processAppMsg(sbcEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case SBC_STATE_CHANGE_EVT:
      SimpleBLECentral_processStackMsg((ICall_Hdr *)pMsg->pData);

      // Free the stack message
      ICall_freeMsg(pMsg->pData);
      break;

    case SBC_KEY_CHANGE_EVT:
      SimpleBLECentral_handleKeys(0, pMsg->hdr.state);
      break;

    case SBC_RSSI_READ_EVT:
      {
        readRssi_t *pRssi = (readRssi_t *)pMsg->pData;

        // If link is up and RSSI reads active
        if (pRssi->connHandle != GAP_CONNHANDLE_ALL &&
            linkDB_Up(pRssi->connHandle))
        {
          // Restart timer
          Util_restartClock(pRssi->pClock, pRssi->period);

          // Read RSSI
          VOID HCI_ReadRssiCmd(pRssi->connHandle);
        }
      }
      break;

    // Pairing event
    case SBC_PAIRING_STATE_EVT:
      {
        SimpleBLECentral_processPairState(pMsg->hdr.state, *pMsg->pData);

        ICall_free(pMsg->pData);
        break;
      }

    // Passcode event
    case SBC_PASSCODE_NEEDED_EVT:
      {
        SimpleBLECentral_processPasscode(connHandle, *pMsg->pData);

        ICall_free(pMsg->pData);
        break;
      }

    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_processRoleEvent
 *
 * @brief   Central role event processing function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void SimpleBLECentral_processRoleEvent(gapCentralRoleEvent_t *pEvent)
{
  switch (pEvent->gap.opcode)
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
      {
        maxPduSize = pEvent->initDone.dataPktLen;

        Display_print0(dispHandle, 1, 0, Util_convertBdAddr2Str(pEvent->initDone.devAddr));
        Display_print0(dispHandle, 2, 0, "Initialized");

        // Prompt user to begin scanning.
        Display_print0(dispHandle, 5, 0, "Discover ->");

#ifdef FPGA_AUTO_CONNECT
        SimpleBLECentral_startGapDiscovery();
#endif // FPGA_AUTO_CONNECT
      }
      break;

    case GAP_DEVICE_INFO_EVENT:
      {
        // if filtering device discovery results based on service UUID
        /*if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
        {
          if (SimpleBLECentral_findSvcUuid(SIMPLEPROFILE_SERV_UUID,
                                           pEvent->deviceInfo.pEvtData,
                                           pEvent->deviceInfo.dataLen))
          {*/
            SimpleBLECentral_addDeviceInfo(pEvent->deviceInfo.addr,
                                           pEvent->deviceInfo.addrType);
            
            if(pEvent->deviceInfo.pEvtData[2] == '$')
            {
              memcpy(advertData,pEvent->deviceInfo.pEvtData,pEvent->deviceInfo.dataLen); 
              memcpy(&routedata[0], &advertData[2], 13);
              NB_flag         = 1;
              
              GAPCentralRole_CancelDiscovery();
            }
            
            if(pEvent->deviceInfo.pEvtData[2] == '@')
            {
              memcpy(slave_advdata,pEvent->deviceInfo.pEvtData,pEvent->deviceInfo.dataLen); 
              memcpy(&safedata[0], &slave_advdata[2], 18);
              Safe_flag         = 1;
              GAPCentralRole_CancelDiscovery();
            }
            
//#ifdef TEST
//          if(pEvent->deviceInfo.addr[5] == 0x0C &&
//             pEvent->deviceInfo.addr[4] == 0x61)
//          {  
//            PIN_setOutputValue(hSbpPins, RED_LED, 0);
//            
//            scan_num++;
//            dev_rssi = pEvent->deviceInfo.rssi;
//            memcpy(scan_Mac, pEvent->deviceInfo.addr, 6);
//
//            Event_post(syncEvent, SBC_SCAN_EVT);
//
//            GAPCentralRole_CancelDiscovery();
//            //scanning_status = SCANNING_STOPPED;
//          }
//#endif //TEST
         // }
      //  }
      }
      break;

    case GAP_DEVICE_DISCOVERY_EVENT:
      {
        // discovery complete
        scanningStarted = FALSE;

#ifdef TEST      
        if(scanning_status == SCANNING_STARTED)
        {
          Event_post(syncEvent, SBC_SCAN_STARTED_EVT);
        }
#endif //TEST

        // if not filtering device discovery results based on service UUID
        if (DEFAULT_DEV_DISC_BY_SVC_UUID == FALSE)
        {
          // Copy results
          scanRes = pEvent->discCmpl.numDevs;
          memcpy(devList, pEvent->discCmpl.pDevList,
                 (sizeof(gapDevRec_t) * scanRes));
        }

        Display_print1(dispHandle, 2, 0, "Devices Found %d", scanRes);

        if (scanRes > 0)
        {
#ifndef FPGA_AUTO_CONNECT
          Display_print0(dispHandle, 3, 0, "<- To Select");
        

        // Initialize scan index.
        scanIdx = -1;

        // Prompt user that re-performing scanning at this state is possible.
        Display_print0(dispHandle, 5, 0, "Discover ->");

#else // FPGA_AUTO_CONNECT
          SimpleBLECentral_connectToFirstDevice();
        
#endif // FPGA_AUTO_CONNECT
        }
      }
      break;

    case GAP_LINK_ESTABLISHED_EVENT:
      {
        if (pEvent->gap.hdr.status == SUCCESS)
        {
          state = BLE_STATE_CONNECTED;
          connHandle = pEvent->linkCmpl.connectionHandle;
          procedureInProgress = TRUE;

          // If service discovery not performed initiate service discovery
          if (charHdl == 0)
          {
            Util_startClock(&startDiscClock);
          }

          Display_print0(dispHandle, 2, 0, "Connected");
          Display_print0(dispHandle, 3, 0, Util_convertBdAddr2Str(pEvent->linkCmpl.devAddr));

          // Display the initial options for a Right key press.
          SimpleBLECentral_handleKeys(0, KEY_LEFT);
        }
        else
        {
          state = BLE_STATE_IDLE;
          connHandle = GAP_CONNHANDLE_INIT;
          discState = BLE_DISC_STATE_IDLE;

          Display_print0(dispHandle, 2, 0, "Connect Failed");
          Display_print1(dispHandle, 3, 0, "Reason: %d", pEvent->gap.hdr.status);
        }
      }
      break;

    case GAP_LINK_TERMINATED_EVENT:
      {
        state = BLE_STATE_IDLE;
        connHandle = GAP_CONNHANDLE_INIT;
        discState = BLE_DISC_STATE_IDLE;
        charHdl = 0;
        procedureInProgress = FALSE;
        keyPressConnOpt = DISCONNECT;
        scanIdx = -1;

        // Cancel RSSI reads
        SimpleBLECentral_CancelRssi(pEvent->linkTerminate.connectionHandle);

        Display_print0(dispHandle, 2, 0, "Disconnected");
        Display_print1(dispHandle, 3, 0, "Reason: %d", pEvent->linkTerminate.reason);
        Display_clearLine(dispHandle, 4);
        Display_clearLine(dispHandle, 6);

        // Prompt user to begin scanning.
        Display_print0(dispHandle, 5, 0, "Discover ->");
      }
      break;

    case GAP_LINK_PARAM_UPDATE_EVENT:
      {
        Display_print1(dispHandle, 2, 0, "Param Update: %d", pEvent->linkUpdate.status);
      }
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_handleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void SimpleBLECentral_handleKeys(uint8_t shift, uint8_t keys)
{
  (void)shift;  // Intentionally unreferenced parameter

  if (keys & KEY_LEFT)
  {
    // If not connected
    if (state == BLE_STATE_IDLE)
    {
      // If not currently scanning
      if (!scanningStarted)
      {
        // Increment index of current result.
        scanIdx++;

        // If there are no scanned devices
        if (scanIdx >= scanRes)
        {
          // Prompt the user to begin scanning again.
          scanIdx = -1;
          Display_print0(dispHandle, 2, 0, "");
          Display_print0(dispHandle, 3, 0, "");
          Display_print0(dispHandle, 5, 0, "Discover ->");
        }
        else
        {
          // Display the indexed scanned device.
          Display_print1(dispHandle, 2, 0, "Device %d", (scanIdx + 1));
          Display_print0(dispHandle, 3, 0, Util_convertBdAddr2Str(devList[scanIdx].addr));
          Display_print0(dispHandle, 5, 0, "Connect ->");
          Display_print0(dispHandle, 6, 0, "<- Next Option");
        }
      }
    }
    else if (state == BLE_STATE_CONNECTED)
    {
      keyPressConnOpt = (keyPressConnOpt == DISCONNECT) ? GATT_RW :
                                          (keyPressConnOpt_t) (keyPressConnOpt + 1);

      switch (keyPressConnOpt)
      {
        case GATT_RW:
          Display_print0(dispHandle, 5, 0, "GATT Read/Write ->");
          break;

        case RSSI:
          Display_print0(dispHandle, 5, 0, "Toggle Read RSSI ->");
          break;

        case CONN_UPDATE:
          Display_print0(dispHandle, 5, 0, "Connection Update ->");
          break;

        case DISCONNECT:
          Display_print0(dispHandle, 5, 0, "Disconnect ->");
          break;

        default:
          break;
      }

      Display_print0(dispHandle, 6, 0, "<- Next Option");
    }

    return;
  }

  if (keys & KEY_RIGHT)
  {
    if (state == BLE_STATE_IDLE)
    {
      if (scanIdx == -1)
      {
        if (!scanningStarted)
        {
          scanningStarted = TRUE;
          scanRes = 0;

          Display_print0(dispHandle, 2, 0, "Discovering...");
          Display_print0(dispHandle, 3, 0, "");
          Display_print0(dispHandle, 4, 0, "");
          Display_print0(dispHandle, 5, 0, "");
          Display_print0(dispHandle, 6, 0, "");

          GAPCentralRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
                                        DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                        DEFAULT_DISCOVERY_WHITE_LIST);
        }
      }
      // Connect if there is a scan result
      else
      {
        // connect to current device in scan result
        uint8_t *peerAddr = devList[scanIdx].addr;
        uint8_t addrType = devList[scanIdx].addrType;

        state = BLE_STATE_CONNECTING;

        GAPCentralRole_EstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                     DEFAULT_LINK_WHITE_LIST,
                                     addrType, peerAddr);

        Display_print0(dispHandle, 2, 0, "Connecting");
        Display_print0(dispHandle, 3, 0, Util_convertBdAddr2Str(peerAddr));
        Display_clearLine(dispHandle, 4);

        // Forget the scan results.
        scanRes = 0;
        scanIdx = -1;
      }
    }
    else if (state == BLE_STATE_CONNECTED)
    {
      switch (keyPressConnOpt)
      {
        case GATT_RW:
          if (charHdl != 0 &&
              procedureInProgress == FALSE)
          {
            uint8_t status;

            // Do a read or write as long as no other read or write is in progress
            if (doWrite)
            {
              // Do a write
              attWriteReq_t req;

              req.pValue = GATT_bm_alloc(connHandle, ATT_WRITE_REQ, 1, NULL);
              if ( req.pValue != NULL )
              {
                req.handle = charHdl;
                req.len = 1;
                req.pValue[0] = charVal;
                req.sig = 0;
                req.cmd = 0;

                status = GATT_WriteCharValue(connHandle, &req, selfEntity);
                if ( status != SUCCESS )
                {
                  GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ);
                }
              }
              else
              {
                status = bleMemAllocError;
              }
            }
            else
            {
              // Do a read
              attReadReq_t req;

              req.handle = charHdl;
              status = GATT_ReadCharValue(connHandle, &req, selfEntity);
            }

            if (status == SUCCESS)
            {
              procedureInProgress = TRUE;
              doWrite = !doWrite;
            }
          }
          break;

        case RSSI:
          // Start or cancel RSSI polling
          if (SimpleBLECentral_RssiFind(connHandle) == NULL)
          {
            SimpleBLECentral_StartRssi(connHandle, DEFAULT_RSSI_PERIOD);
          }
          else
          {
            SimpleBLECentral_CancelRssi(connHandle);

            Display_print0(dispHandle, 4, 0, "RSSI Cancelled");
          }
          break;

        case CONN_UPDATE:
           // Connection update
          GAPCentralRole_UpdateLink(connHandle,
                                    DEFAULT_UPDATE_MIN_CONN_INTERVAL,
                                    DEFAULT_UPDATE_MAX_CONN_INTERVAL,
                                    DEFAULT_UPDATE_SLAVE_LATENCY,
                                    DEFAULT_UPDATE_CONN_TIMEOUT);
          break;

        case DISCONNECT:
          state = BLE_STATE_DISCONNECTING;

          GAPCentralRole_TerminateLink(connHandle);

          Display_print0(dispHandle, 2, 0, "Disconnecting");
          Display_print0(dispHandle, 3, 0, "");
          Display_print0(dispHandle, 4, 0, "");
          Display_print0(dispHandle, 5, 0, "");

          keyPressConnOpt = GATT_RW;
          break;

        default:
          break;
      }
    }

    return;
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  none
 */
static void SimpleBLECentral_processGATTMsg(gattMsgEvent_t *pMsg)
{
  if (state == BLE_STATE_CONNECTED)
  {
    // See if GATT server was unable to transmit an ATT response
    if (pMsg->hdr.status == blePending)
    {
      // No HCI buffer was available. App can try to retransmit the response
      // on the next connection event. Drop it for now.
      Display_print1(dispHandle, 4, 0, "ATT Rsp dropped %d", pMsg->method);
    }
    else if ((pMsg->method == ATT_READ_RSP)   ||
             ((pMsg->method == ATT_ERROR_RSP) &&
              (pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ)))
    {
      if (pMsg->method == ATT_ERROR_RSP)
      {
        Display_print1(dispHandle, 4, 0, "Read Error %d", pMsg->msg.errorRsp.errCode);
      }
      else
      {
        // After a successful read, display the read value
        Display_print1(dispHandle, 4, 0, "Read rsp: %d", pMsg->msg.readRsp.pValue[0]);
      }

      procedureInProgress = FALSE;
    }
    else if ((pMsg->method == ATT_WRITE_RSP)  ||
             ((pMsg->method == ATT_ERROR_RSP) &&
              (pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ)))
    {
      if (pMsg->method == ATT_ERROR_RSP)
      {
        Display_print1(dispHandle, 4, 0, "Write Error %d", pMsg->msg.errorRsp.errCode);
      }
      else
      {
        // After a successful write, display the value that was written and
        // increment value
        Display_print1(dispHandle, 4, 0, "Write sent: %d", charVal++);
      }

      procedureInProgress = FALSE;

    }
    else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
    {
      // ATT request-response or indication-confirmation flow control is
      // violated. All subsequent ATT requests or indications will be dropped.
      // The app is informed in case it wants to drop the connection.

      // Display the opcode of the message that caused the violation.
      Display_print1(dispHandle, 4, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
    }
    else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
    {
      // MTU size updated
      Display_print1(dispHandle, 4, 0, "MTU Size: %d", pMsg->msg.mtuEvt.MTU);
    }
    else if (discState != BLE_DISC_STATE_IDLE)
    {
      SimpleBLECentral_processGATTDiscEvent(pMsg);
    }
  } // else - in case a GATT message came after a connection has dropped, ignore it.

  // Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);
}

/*********************************************************************
 * @fn      SimpleBLECentral_processCmdCompleteEvt
 *
 * @brief   Process an incoming OSAL HCI Command Complete Event.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void SimpleBLECentral_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg)
{
  switch (pMsg->cmdOpcode)
  {
    case HCI_READ_RSSI:
      {
        //int8 rssi = (int8)pMsg->pReturnParam[3];

        Display_print1(dispHandle, 4, 0, "RSSI -dB: %d", (uint32_t)(-rssi));
      }
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_StartRssi
 *
 * @brief   Start periodic RSSI reads on a link.
 *
 * @param   connHandle - connection handle of link
 * @param   period - RSSI read period in ms
 *
 * @return  SUCCESS: Terminate started
 *          bleIncorrectMode: No link
 *          bleNoResources: No resources
 */
static bStatus_t SimpleBLECentral_StartRssi(uint16_t connHandle, uint16_t period)
{
  readRssi_t *pRssi;

  // Verify link is up
  if (!linkDB_Up(connHandle))
  {
    return bleIncorrectMode;
  }

  // If already allocated
  if ((pRssi = SimpleBLECentral_RssiFind(connHandle)) != NULL)
  {
    // Stop timer
    Util_stopClock(pRssi->pClock);

    pRssi->period = period;
  }
  // Allocate structure
  else if ((pRssi = SimpleBLECentral_RssiAlloc(connHandle)) != NULL)
  {
    pRssi->period = period;
  }
  // Allocate failed
  else
  {
    return bleNoResources;
  }

  // Start timer
  Util_restartClock(pRssi->pClock, period);

  return SUCCESS;
}

/*********************************************************************
 * @fn      SimpleBLECentral_CancelRssi
 *
 * @brief   Cancel periodic RSSI reads on a link.
 *
 * @param   connHandle - connection handle of link
 *
 * @return  SUCCESS: Operation successful
 *          bleIncorrectMode: No link
 */
static bStatus_t SimpleBLECentral_CancelRssi(uint16_t connHandle)
{
  readRssi_t *pRssi;

  if ((pRssi = SimpleBLECentral_RssiFind(connHandle)) != NULL)
  {
    // Stop timer
    Util_stopClock(pRssi->pClock);

    // Free RSSI structure
    SimpleBLECentral_RssiFree(connHandle);

    return SUCCESS;
  }

  // Not found
  return bleIncorrectMode;
}

/*********************************************************************
 * @fn      gapCentralRole_RssiAlloc
 *
 * @brief   Allocate an RSSI structure.
 *
 * @param   connHandle - Connection handle
 *
 * @return  pointer to structure or NULL if allocation failed.
 */
static readRssi_t *SimpleBLECentral_RssiAlloc(uint16_t connHandle)
{
  uint8_t i;

  // Find free RSSI structure
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (readRssi[i].connHandle == GAP_CONNHANDLE_ALL)
    {
      readRssi_t *pRssi = &readRssi[i];

      pRssi->pClock = (Clock_Struct *)ICall_malloc(sizeof(Clock_Struct));
      if (pRssi->pClock)
      {
        Util_constructClock(pRssi->pClock, SimpleBLECentral_readRssiHandler,
                            0, 0, false, i);
        pRssi->connHandle = connHandle;

        return pRssi;
      }
    }
  }

  // No free structure found
  return NULL;
}

/*********************************************************************
 * @fn      gapCentralRole_RssiFind
 *
 * @brief   Find an RSSI structure.
 *
 * @param   connHandle - Connection handle
 *
 * @return  pointer to structure or NULL if not found.
 */
static readRssi_t *SimpleBLECentral_RssiFind(uint16_t connHandle)
{
  uint8_t i;

  // Find free RSSI structure
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (readRssi[i].connHandle == connHandle)
    {
      return &readRssi[i];
    }
  }

  // Not found
  return NULL;
}

/*********************************************************************
 * @fn      gapCentralRole_RssiFree
 *
 * @brief   Free an RSSI structure.
 *
 * @param   connHandle - Connection handle
 *
 * @return  none
 */
static void SimpleBLECentral_RssiFree(uint16_t connHandle)
{
  uint8_t i;

  // Find RSSI structure
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (readRssi[i].connHandle == connHandle)
    {
      readRssi_t *pRssi = &readRssi[i];
      if (pRssi->pClock)
      {
        Clock_destruct(pRssi->pClock);

        // Free clock struct
        ICall_free(pRssi->pClock);
        pRssi->pClock = NULL;
      }

      pRssi->connHandle = GAP_CONNHANDLE_ALL;
      break;
    }
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void SimpleBLECentral_processPairState(uint8_t state, uint8_t status)
{
  if (state == GAPBOND_PAIRING_STATE_STARTED)
  {
    Display_print0(dispHandle, 2, 0, "Pairing started");
  }
  else if (state == GAPBOND_PAIRING_STATE_COMPLETE)
  {
    if (status == SUCCESS)
    {
      Display_print0(dispHandle, 2, 0, "Pairing success");
    }
    else
    {
      Display_print1(dispHandle, 2, 0, "Pairing fail: %d", status);
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_BONDED)
  {
    if (status == SUCCESS)
    {
      Display_print0(dispHandle, 2, 0, "Bonding success");
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_BOND_SAVED)
  {
    if (status == SUCCESS)
    {
      Display_print0(dispHandle, 2, 0, "Bond save success");
    }
    else
    {
      Display_print1(dispHandle, 2, 0, "Bond save failed: %d", status);
    }
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
static void SimpleBLECentral_processPasscode(uint16_t connectionHandle,
                                             uint8_t uiOutputs)
{
  uint32_t  passcode;

  // Create random passcode
  passcode = Util_GetTRNG();
  passcode %= 1000000;

  // Display passcode to user
  if (uiOutputs != 0)
  {
    Display_print1(dispHandle, 4, 0, "Passcode: %d", passcode);
  }

  // Send passcode response
  GAPBondMgr_PasscodeRsp(connectionHandle, SUCCESS, passcode);
}

/*********************************************************************
 * @fn      SimpleBLECentral_startDiscovery
 *
 * @brief   Start service discovery.
 *
 * @return  none
 */
static void SimpleBLECentral_startDiscovery(void)
{
  attExchangeMTUReq_t req;

  // Initialize cached handles
  svcStartHdl = svcEndHdl = charHdl = 0;

  discState = BLE_DISC_STATE_MTU;

  // Discover GATT Server's Rx MTU size
  req.clientRxMTU = maxPduSize - L2CAP_HDR_SIZE;

  // ATT MTU size should be set to the minimum of the Client Rx MTU
  // and Server Rx MTU values
  VOID GATT_ExchangeMTU(connHandle, &req, selfEntity);
}

/*********************************************************************
 * @fn      SimpleBLECentral_processGATTDiscEvent
 *
 * @brief   Process GATT discovery event
 *
 * @return  none
 */
static void SimpleBLECentral_processGATTDiscEvent(gattMsgEvent_t *pMsg)
{
  if (discState == BLE_DISC_STATE_MTU)
  {
    // MTU size response received, discover simple BLE service
    if (pMsg->method == ATT_EXCHANGE_MTU_RSP)
    {
      uint8_t uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(SIMPLEPROFILE_SERV_UUID),
                                         HI_UINT16(SIMPLEPROFILE_SERV_UUID) };

      // Just in case we're using the default MTU size (23 octets)
      Display_print1(dispHandle, 4, 0, "MTU Size: %d", ATT_MTU_SIZE);

      discState = BLE_DISC_STATE_SVC;

      // Discovery simple BLE service
      VOID GATT_DiscPrimaryServiceByUUID(connHandle, uuid, ATT_BT_UUID_SIZE,
                                         selfEntity);
    }
  }
  else if (discState == BLE_DISC_STATE_SVC)
  {
    // Service found, store handles
    if (pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
        pMsg->msg.findByTypeValueRsp.numInfo > 0)
    {
      svcStartHdl = ATT_ATTR_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
      svcEndHdl = ATT_GRP_END_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
    }

    // If procedure complete
    if (((pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP) &&
         (pMsg->hdr.status == bleProcedureComplete))  ||
        (pMsg->method == ATT_ERROR_RSP))
    {
      if (svcStartHdl != 0)
      {
        attReadByTypeReq_t req;

        // Discover characteristic
        discState = BLE_DISC_STATE_CHAR;

        req.startHandle = svcStartHdl;
        req.endHandle = svcEndHdl;
        req.type.len = ATT_BT_UUID_SIZE;
        req.type.uuid[0] = LO_UINT16(SIMPLEPROFILE_CHAR1_UUID);
        req.type.uuid[1] = HI_UINT16(SIMPLEPROFILE_CHAR1_UUID);

        VOID GATT_ReadUsingCharUUID(connHandle, &req, selfEntity);
      }
    }
  }
  else if (discState == BLE_DISC_STATE_CHAR)
  {
    // Characteristic found, store handle
    if ((pMsg->method == ATT_READ_BY_TYPE_RSP) &&
        (pMsg->msg.readByTypeRsp.numPairs > 0))
    {
      charHdl = BUILD_UINT16(pMsg->msg.readByTypeRsp.pDataList[0],
                             pMsg->msg.readByTypeRsp.pDataList[1]);

      Display_print0(dispHandle, 2, 0, "Simple Svc Found");
      procedureInProgress = FALSE;
    }

    discState = BLE_DISC_STATE_IDLE;
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_findSvcUuid
 *
 * @brief   Find a given UUID in an advertiser's service UUID list.
 *
 * @return  TRUE if service UUID found
 */
static bool SimpleBLECentral_findSvcUuid(uint16_t uuid, uint8_t *pData,
                                         uint8_t dataLen)
{
  uint8_t adLen;
  uint8_t adType;
  uint8_t *pEnd;

  pEnd = pData + dataLen - 1;

  // While end of data not reached
  while (pData < pEnd)
  {
    // Get length of next AD item
    adLen = *pData++;
    if (adLen > 0)
    {
      adType = *pData;

      // If AD type is for 16-bit service UUID
      if ((adType == GAP_ADTYPE_16BIT_MORE) ||
          (adType == GAP_ADTYPE_16BIT_COMPLETE))
      {
        pData++;
        adLen--;

        // For each UUID in list
        while (adLen >= 2 && pData < pEnd)
        {
          // Check for match
          if ((pData[0] == LO_UINT16(uuid)) && (pData[1] == HI_UINT16(uuid)))
          {
            // Match found
            return TRUE;
          }

          // Go to next
          pData += 2;
          adLen -= 2;
        }

        // Handle possible erroneous extra byte in UUID list
        if (adLen == 1)
        {
          pData++;
        }
      }
      else
      {
        // Go to next item
        pData += adLen;
      }
    }
  }

  // Match not found
  return FALSE;
}

/*********************************************************************
 * @fn      SimpleBLECentral_addDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void SimpleBLECentral_addDeviceInfo(uint8_t *pAddr, uint8_t addrType)
{
  uint8_t i;

  // If result count not at max
  if (scanRes < DEFAULT_MAX_SCAN_RES)
  {
    // Check if device is already in scan results
    for (i = 0; i < scanRes; i++)
    {
      if (memcmp(pAddr, devList[i].addr , B_ADDR_LEN) == 0)
      {
        return;
      }
    }

    // Add addr to scan result list
    memcpy(devList[scanRes].addr, pAddr, B_ADDR_LEN);
    devList[scanRes].addrType = addrType;

    // Increment scan result count
    scanRes++;
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_eventCB
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  TRUE if safe to deallocate event message, FALSE otherwise.
 */
static uint8_t SimpleBLECentral_eventCB(gapCentralRoleEvent_t *pEvent)
{
  // Forward the role event to the application
  if (SimpleBLECentral_enqueueMsg(SBC_STATE_CHANGE_EVT,
                                  SUCCESS, (uint8_t *)pEvent))
  {
    // App will process and free the event
    return FALSE;
  }

  // Caller should free the event
  return TRUE;
}

/*********************************************************************
 * @fn      SimpleBLECentral_pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void SimpleBLECentral_pairStateCB(uint16_t connHandle, uint8_t state,
                                         uint8_t status)
{
  uint8_t *pData;

  // Allocate space for the event data.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = status;

    // Queue the event.
    SimpleBLECentral_enqueueMsg(SBC_PAIRING_STATE_EVT, state, pData);
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_passcodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void SimpleBLECentral_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs)
{
  uint8_t *pData;

  // Allocate space for the passcode event.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = uiOutputs;

    // Enqueue the event.
    SimpleBLECentral_enqueueMsg(SBC_PASSCODE_NEEDED_EVT, 0, pData);
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_startDiscHandler
 *
 * @brief   Clock handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void SimpleBLECentral_startDiscHandler(UArg a0)
{
  Event_post(syncEvent, a0);
}

/*********************************************************************
 * @fn      SimpleBLECentral_keyChangeHandler
 *
 * @brief   Key event handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void SimpleBLECentral_keyChangeHandler(uint8 keys)
{
  SimpleBLECentral_enqueueMsg(SBC_KEY_CHANGE_EVT, keys, NULL);
}

/*********************************************************************
 * @fn      SimpleBLECentral_readRssiHandler
 *
 * @brief   Read RSSI handler function
 *
 * @param   a0 - read RSSI index
 *
 * @return  none
 */
void SimpleBLECentral_readRssiHandler(UArg a0)
{
  SimpleBLECentral_enqueueMsg(SBC_RSSI_READ_EVT, SUCCESS,
                              (uint8_t *)&readRssi[a0]);
}

/*********************************************************************
 * @fn      SimpleBLECentral_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 * @param   pData - message data pointer.
 *
 * @return  TRUE or FALSE
 */
static uint8_t SimpleBLECentral_enqueueMsg(uint8_t event, uint8_t state,
                                           uint8_t *pData)
{
  sbcEvt_t *pMsg = ICall_malloc(sizeof(sbcEvt_t));

  // Create dynamic pointer to message.
  if (pMsg)
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;
    pMsg->pData = pData;

    // Enqueue the message.
    return Util_enqueueMsg(appMsgQueue, syncEvent, (uint8_t *)pMsg);
  }

  return FALSE;
}


/***********************************************************************/
static void Read_Mac(uint8 *Mac_Address)        
{    
  uint32_t Own_Mac0 = HWREG(FCFG1_BASE + FCFG1_O_MAC_BLE_0);  
  uint32_t Own_Mac1 = HWREG(FCFG1_BASE + FCFG1_O_MAC_BLE_1);  
    
  Mac_Address[5] = Own_Mac0;  
  Mac_Address[4] = Own_Mac0 >> 8;  
  Mac_Address[3] = Own_Mac0 >> 16;  
  Mac_Address[2] = Own_Mac0 >> 24;  
  Mac_Address[1] = Own_Mac1;  
  Mac_Address[0] = Own_Mac1 >> 8;  
}    

/*********************************************************************
 * @fn      UARTreadCallback
 *
 * @brief   UART read Callback.
 *
 * @param   handle - Uart handle.
 *          *buf   - optional
 *          count  - optional
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */

static void UARTreadCallback(UART_Handle handle, void *buf, size_t count)
{
  Semaphore_post(uartSemHandle);
}

//static void Time()
//{
//  userTime = UTC_getClock();
//  UTC_convertUTCTime(&Timeconfig, userTime);
//}

/********************************
*@fn NBIOT
*
*
*********************************/

static void NBIoT_Init()
{
  /*****************************/
  //NBIoT 
  if(CFUN_OK_Flag == 0)
  {
   UART_read(UART_handle,Read_Uart,sizeof(Read_Uart));
   UART_write(UART_handle,CFUN_Open,sizeof(CFUN_Open)); 
   Semaphore_pend(uartSemHandle, 600000); //6S
   CFUN_OK_Flag = 1;
   //memset(Read_Uart , 0 , sizeof(Read_Uart));
  }
  
  else if(IMEI_Ok_Flag == 0)
  {
    UART_read(UART_handle,Read_Uart, sizeof(Read_Uart));
    UART_write(UART_handle,Write_IMEI,sizeof(Write_IMEI));
    Semaphore_pend(uartSemHandle, UartTimeout);
    
    for(int x=0;x<15;x++)
    {
      IMEI[x]=Read_Uart[x+8];
    }  
      IMEI_Ok_Flag =1 ;
    //memset(Read_Uart , 0 , sizeof(Read_Uart));
  }
  
  else if(IMSI_Ok_Flag == 0)
  {
    UART_read(UART_handle,Read_Uart,sizeof(Read_Uart));
    UART_write(UART_handle,Write_IMSI,sizeof(Write_IMSI)); 
    Semaphore_pend(uartSemHandle, UartTimeout);
    
    for(int x=0;x<15;x++)
    {
      IMSI[x]=Read_Uart[x+2];
    }
    
    if(IMSI[0] == '4')
    {
      IMSI_Ok_Flag =1 ;
      //memset(Read_Uart , 0 , sizeof(Read_Uart));
    }  
  }

  else if(APN_Ok_Flag == 0)
  {
    
    if(IMSI[2] == '0' && IMSI[3] == '1' && IMSI[4] == '1')
    { 
      UART_write(UART_handle, CGDCONT_CT,sizeof(CGDCONT_CT)); //APN : ctnb
      Task_sleep(100000);
      UART_write(UART_handle,Cops_CT,sizeof(Cops_CT));        //Oper : 46011   
      
      APN_Ok_Flag =1;  
      //memset(Read_Uart , 0 , sizeof(Read_Uart));
    }    
    
    else if(IMSI[2] == '6' && IMSI[3] == '0' && IMSI[4] == '1')
    {
      UART_write(UART_handle, CGDCONT_FET,sizeof(CGDCONT_FET)); //APN : nbiot
      Task_sleep(100000);
      UART_write(UART_handle,Cops_FET,sizeof(Cops_FET));        //Oper : 46601
      
      APN_Ok_Flag =1;  
      //memset(Read_Uart , 0 , sizeof(Read_Uart));
    }    
    
    else if(IMSI[2] == '6' && IMSI[3] == '9' && IMSI[4] == '2')
    {
      UART_write(UART_handle, CGDCONT_CHT,sizeof(CGDCONT_CHT)); //APN : internet.iot
      Task_sleep(100000);
      UART_write(UART_handle,Cops_CHT,sizeof(Cops_CHT));        //Oper : 46692
    
      APN_Ok_Flag =1;
      //memset(Read_Uart , 0 , sizeof(Read_Uart));
    }
    
//    else if(IMSI[2] == '6' && IMSI[3] == '9' && IMSI[4] == '7')
//    {
//      UART_write(UART_handle, CGDCONT_TM,sizeof(CGDCONT_TM)); //APN : twm.nbiot
//      Task_sleep(100000);
//      UART_write(UART_handle,Cops_TM,sizeof(Cops_TM));        //Oper : 46697
//      
//      APN_Ok_Flag =1;     
//    }

    UART_write(UART_handle,REG_Open,sizeof(REG_Open));
    Task_sleep(100000);   
    UART_write(UART_handle, CGATT_Open,sizeof(CGATT_Open)); //GATT=1
    Task_sleep(100000);
    UART_write(UART_handle,Lock_Band,sizeof(Lock_Band));
    Task_sleep(100000);
    UART_read(UART_handle,Read_Uart, sizeof(Read_Uart));    
    Semaphore_pend(uartSemHandle, UartTimeout);     
  }
  
   else if(CSQ_Ok_Flag == 0 )
  {
    UART_read(UART_handle,Read_Uart, sizeof(Read_Uart));
    UART_write(UART_handle,Write_CSQ,sizeof(Write_CSQ)); 
    Semaphore_pend(uartSemHandle, UartTimeout);         
    
    for(int x=0;x<2;x++)
    {
      CSQ[x]=Read_Uart[x+7];
    }
    
    if(CSQ[1] !='9')
    { 
      CSQ_Ok_Flag =1; 
      //memset(Read_Uart , 0 , sizeof(Read_Uart));    
    } 
    
  }
  else if(GATT_Ok_Flag == 0)
  {
    UART_read(UART_handle,Read_Uart, sizeof(Read_Uart));
    UART_write(UART_handle,Write_CGATT,sizeof(Write_CGATT)); 
    Semaphore_pend(uartSemHandle, UartTimeout); 
    
    if(Read_Uart[9] == '1' )
    {
      GATT_Ok_Flag = 1;
      GATT_Error   = 0; 
      //memset(Read_Uart , 0 , sizeof(Read_Uart));
    }  

    else if(Read_Uart[9] == '0')
    {
      GATT_Error ++;
    }

  }
   
  else if(REG_Ok_Flag == 0)
  {
    UART_read(UART_handle,Read_Uart, sizeof(Read_Uart));
    UART_write(UART_handle,Write_REG,sizeof(Write_REG)); 
    Semaphore_pend(uartSemHandle, UartTimeout); 
    
    REG=Read_Uart[11];
    
    if(REG == '1')
    {
      REG_Ok_Flag = 1;
      //memset(Read_Uart , 0 , sizeof(Read_Uart));
    }
    
  }
  
  
  else if(NBIot_Init_Ok ==0 )
  {
    UART_read(UART_handle,Read_Uart, sizeof(Read_Uart));
    UART_write(UART_handle, NSOCR,sizeof(NSOCR)); //UDP channel 1
    Task_sleep(10000);
    UART_read(UART_handle,Read_Uart, sizeof(Read_Uart));
    UART_write(UART_handle, NSMI,sizeof(NSMI));
    Task_sleep(10000);
//    UART_write(UART_handle, PSM_Mode_Config,sizeof(PSM_Mode_Config));    
//    Task_sleep(10000);    
//    UART_write(UART_handle, PSM_Mode_Open,sizeof(PSM_Mode_Open));    
//    Task_sleep(10000);        
//    UART_write(UART_handle, IDle_Mode_Open,sizeof(IDle_Mode_Open));    
//    Task_sleep(10000);          
    //UART_read(UART_handle,Read_Uart, sizeof(Read_Uart));
    Semaphore_pend(uartSemHandle, UartTimeout);
    
    NBIot_Init_Ok = 1;
    
  }
   //memset(Read_Uart , 0 , sizeof(Read_Uart));
  /*
//  if(CFUN_Open_Flag == 0)
//  {
//    UART_read(UART_handle,Read_Uart,sizeof(Read_Uart));
//    UART_write(UART_handle,Write_CFUN_Open,sizeof(Write_CFUN_Open)); 
//    Semaphore_pend(uartSemHandle, 600000);
//    
//    CFUN_Open_Flag = 1;
//    memset(Read_Uart , 0 , sizeof(Read_Uart));
//  }
  
  if(CIMI_Ok_Flag == 0)
  {
    UART_read(UART_handle,Read_Uart,sizeof(Read_Uart));
    UART_write(UART_handle,Write_CIMI,sizeof(Write_CIMI)); 
    Semaphore_pend(uartSemHandle, UartTimeout);
    for(int x=0;x<15;x++)
    {
      CIMI[x]=Read_Uart[x+2];
    }
  
  if(CIMI[0] == '4' || CIMI[0] == '8')
  {
    CIMI_Ok_Flag =1 ;
    memset(Read_Uart , 0 , sizeof(Read_Uart));
  }
  }
  else if(CIMI_Ok_Flag == 1 && CSQ_Ok_Flag == 0 )
  {
    UART_read(UART_handle,Read_Uart, sizeof(Read_Uart));
    UART_write(UART_handle,Write_CSQ,sizeof(Write_CSQ)); 
    Semaphore_pend(uartSemHandle, UartTimeout);
    
    if(Read_Uart[8] !='9')
    {
      CSQ_Ok_Flag = 1;
      memset(Read_Uart , 0 , sizeof(Read_Uart));
    }

  }

  else if(CSQ_Ok_Flag == 1 && CEREG_Ok_Flag == 0)
  {
    UART_read(UART_handle,Read_Uart, sizeof(Read_Uart));
    UART_write(UART_handle,Write_REG,sizeof(Write_REG)); 
    Semaphore_pend(uartSemHandle, UartTimeout); 

  if(Read_Uart[11] == '1')
  {
    CEREG_Ok_Flag = 1;
    memset(Read_Uart , 0 , sizeof(Read_Uart));
  }
  
  }
  
  else if(CEREG_Ok_Flag == 1 && APN_Ok_Flag == 0)
  {
    if(CIMI[2] == '6' && CIMI[3] == '9' && CIMI[4] == '2')
    {
      UART_write(UART_handle, CGDCONT_CHT,sizeof(CGDCONT_CHT)); //APN

      APN_Ok_Flag =1;
    }
    else if(CIMI[2] == '6' && CIMI[3] == '0' && CIMI[4] == '1')
    {
      UART_write(UART_handle, CGDCONT_FET,sizeof(CGDCONT_FET)); //APN

      APN_Ok_Flag =1;
    }
    UART_write(UART_handle, CGATT,sizeof(CGATT)); //GATT=1
  }

  else if(APN_Ok_Flag == 1 && CGATT_Ok_Flag == 0)
  {
    UART_read(UART_handle,Read_Uart, sizeof(Read_Uart));
    UART_write(UART_handle,Write_CGATT,sizeof(Write_CGATT)); 
    Semaphore_pend(uartSemHandle, UartTimeout); 

  if(Read_Uart[10] == '1' )
  {
    CGATT_Ok_Flag =1;
  } 

  }

  else if(CGATT_Ok_Flag == 1 && NBIot_Init_Ok ==0 )
  {
    UART_write(UART_handle, NSOCR,sizeof(NSOCR)); //UDP channel 1
    //UART_write(UART_handle, NSOCR_Wave,sizeof(NSOCR_Wave)); //UDP channel 2
    UART_write(UART_handle, NSMI,sizeof(NSMI));

    NBIot_Init_Ok = 1;
  }*/
  }


static void NBIoT_Data()
{
  NBIoT_Data_Count++;
  
  //uint8_t Count=13;
  
//  NSOST_UDP1[0] ='A';
//  NSOST_UDP1[1] ='T';
//  NSOST_UDP1[2] ='+';
//  NSOST_UDP1[3] ='N';
//  NSOST_UDP1[4] ='S';
//  NSOST_UDP1[5] ='O';
//  NSOST_UDP1[6] ='S';
//  NSOST_UDP1[7] ='T';
//  NSOST_UDP1[8] ='=';
//  NSOST_UDP1[9] ='1';
//  NSOST_UDP1[10]=',';
//  NSOST_UDP1[11]='0';
//  NSOST_UDP1[12]='x';
//  NSOST_UDP1[13]=NB_Config[3]>>4;
//  NSOST_UDP1[14]=NB_Config[3] & 0X0F;
//  NSOST_UDP1[15]='.';
//  NSOST_UDP1[16]='0';
//  NSOST_UDP1[17]='x';
//  NSOST_UDP1[18]=NB_Config[4]>>4;
//  NSOST_UDP1[19]=NB_Config[4] & 0X0F;
//  NSOST_UDP1[20]='.';
//  NSOST_UDP1[21]='0';
//  NSOST_UDP1[22]='x';
//  NSOST_UDP1[23]=NB_Config[5]>>4;
//  NSOST_UDP1[24]=NB_Config[5] & 0X0F;
//  NSOST_UDP1[25]='.';
//  NSOST_UDP1[26]='0';
//  NSOST_UDP1[27]='x';
//  NSOST_UDP1[28]=NB_Config[6]>>4;
//  NSOST_UDP1[29]=NB_Config[6] & 0X0F;
//  NSOST_UDP1[30]=',';
//  NSOST_UDP1[31]=((((NB_Config[7]<<8) | NB_Config[8])%100000)/10000)  +0x30;
//  NSOST_UDP1[32]=((((NB_Config[7]<<8) | NB_Config[8])%10000)/1000)+0x30;
//  NSOST_UDP1[33]=((((NB_Config[7]<<8) | NB_Config[8])%1000)/100)+0x30;
//  NSOST_UDP1[34]=((((NB_Config[7]<<8) | NB_Config[8])%100)/10)+0x30;
//  NSOST_UDP1[35]=((((NB_Config[7]<<8) | NB_Config[8])%10)/1)+0x30;
//  NSOST_UDP1[36]=',';
//  NSOST_UDP1[37]='2';
//  NSOST_UDP1[38]='1';
//  NSOST_UDP1[39]=',';
  
  gatewaydata[0] = License_State;//內外結果
  memcpy(&gatewaydata[1], &ownAddress[0], 6);
  gatewaydata[7] = bicycle_number;
  /*
  routedata[0]=0x31;
  routedata[1]=0x31;
  routedata[2]=0x31;
  routedata[3]=0x31;
  routedata[4]=0x31;
  routedata[5]=0x31;
  routedata[6]=0x31;
  routedata[7]=0x31;
  routedata[8]=0x31;
  routedata[9]=0x31;
  routedata[10]=0x31;
  routedata[11]=0x31;
  routedata[12]=0x31;
*/
  for(int x=0;x<sizeof(routedata);x++)
  {
    Hex_to_Ascii(routedata[x],&routedata2[x*2]);
  }

  for(int x=0;x<sizeof(gatewaydata);x++)
  {
    Hex_to_Ascii(gatewaydata[x],&gatewaydata2[x*2]);
  }
  
  for (int x=0;x<(sizeof(NSOST_UDP1)+sizeof(routedata2)+sizeof(gatewaydata2));x++)
  {
    if(x < sizeof(NSOST_UDP1))
    {
      NBIOT_Data1[x]=NSOST_UDP1[x];
    }
    else if(sizeof(NSOST_UDP1)<=x  && x<(sizeof(NSOST_UDP1)+sizeof(routedata2)))
    {
      NBIOT_Data1[x]=routedata2[x-sizeof(NSOST_UDP1)];
    }
    else if(sizeof(NSOST_UDP1)+sizeof(routedata2)<=x  && x<(sizeof(NSOST_UDP1)+sizeof(routedata2)+sizeof(gatewaydata2)))
    {
      NBIOT_Data1[x]=gatewaydata2[x-(sizeof(NSOST_UDP1)+sizeof(routedata2))];
    }
    
    NBIOT_Data1[76]=0X0D;
    NBIOT_Data1[77]=0X0A;
  }
}

/*************************************************************************
*  Slave Safe Notify
*
**************************************************************************/
static void NBIoT_Safe()
{
  NBIoT_Data_Count++;
  
  memcpy(&master_safedata[0], &ownAddress[0], 6);
  master_safedata[6] = BattD;
  
  for(int x=0;x<sizeof(safedata);x++)
  {
    Hex_to_Ascii(safedata[x],&safedata2[x*2]);
  }
  
  for(int x=0;x<sizeof(master_safedata);x++)
  {
    Hex_to_Ascii(master_safedata[x],&master_safedata2[x*2]);
  }
  
  for (int x=0;x<(sizeof(NSOST_UDP2)+sizeof(safedata2)+sizeof(master_safedata2));x++)
  {
    if(x < sizeof(NSOST_UDP2))
    {
      NBIOT_Data2[x]=NSOST_UDP2[x];
    }
    else if(sizeof(NSOST_UDP2)<=x  && x<(sizeof(NSOST_UDP2)+sizeof(safedata2)))
    {
      NBIOT_Data2[x]=safedata2[x-sizeof(NSOST_UDP2)];
    }
    else if(sizeof(NSOST_UDP2)+sizeof(safedata2)<=x  && x<(sizeof(NSOST_UDP2)+sizeof(safedata2)+sizeof(master_safedata2)))
    {
      NBIOT_Data2[x]=master_safedata2[x-(sizeof(NSOST_UDP2)+sizeof(safedata2))];
    }
    
    NBIOT_Data2[84]=0X0D;
    NBIOT_Data2[85]=0X0A;
  }
}
/***************************************************************
* master voltage
*
*
**************************************************************/
static void Voltage()
{
//  Init_Solar_Energy();
//  AUXADCGenManualTrigger();
//  Solar_energy = AUXADCReadFifo();//讀到的值 最大4096
//  AUXADCFlushFifo();//清空flash
//  AUXADCDisable();//關閉ADC
//
//  Solar_energy=((Solar_energy/4096)*4.3); //電壓轉換
//
//  Solar_energyA= (int)Solar_energy*10;
//  Solar_energyD=8*((Solar_energyA-(Solar_energyA%10))/5)+(Solar_energyA%10);


  //Batt
  batVol=AONBatMonBatteryVoltageGet();

  batVol_H=batVol>>8;
  batVol_HH=batVol_H;
  batVol_L=batVol;
  batVol_LL=batVol_L;
  Batt= batVol_LL/256+batVol_HH;
  BattA=(int)Batt*10;
  BattD=8*((BattA-(BattA%10))/5)+(BattA%10);

  //Temp
  //Temp=AONBatMonTemperatureGetDegC();
}

/**********************************************************
*@fn      SVM
*
*
***********************************************************/
static void SVM()
{
  for(int x=0; x<200; x++)
  {
    kernel[x] = ((Trainingdata1[x]-Testdata1[0])*(Trainingdata1[x]-Testdata1[0])+(Trainingdata2[x]-Testdata1[1])*(Trainingdata2[x]-Testdata1[1])+(Trainingdata3[x]-Testdata1[2])*(Trainingdata3[x]-Testdata1[2])+(Trainingdata4[x]-Testdata1[3])*(Trainingdata4[x]-Testdata1[3]));
    kernel[x] = ((-1*kernel[x])/(2*sigma*sigma));
    kernel[x] = exp(kernel[x]);
  }
  
  for(int x=0; x<200; x++)
  {
    D=D+(Alpha_Y[x]*kernel[x]);
  }
  
  D_sum=D+Base;
  
  if(D_sum > 0)
  {
    License_State = Inside_State;
    bicycle_number = bicycle_number + 1;
    Position_flag = 1;
    position[1]=0x69; //'i'nside
  }
  else if(D_sum < 0)
  {
    License_State = Outside_State;
    bicycle_number = bicycle_number - 1;
    Position_flag = 1;
    position[1]=0x6F; //'o'utside
  }
}
/*********************************************************************
*********************************************************************/
