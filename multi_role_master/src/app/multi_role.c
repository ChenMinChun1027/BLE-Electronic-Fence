/******************************************************************************

@file  multi_role.c

@brief This file contains the multi_role sample application for use
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
#include "devinfoservice.h"
#include "simple_gatt_profile.h"
#include "multi.h"
#include "gapbondmgr.h"
#include "osal_snv.h"
#include "icall_apimsg.h"
#include "icall.h"
#include "util.h"
#include "ll_common.h"

#include "board_key.h"
#ifdef USE_CORE_SDK
  #include <ti/display/Display.h>
#else /* !USE_CORE_SDK */
  #include <ti/mw/display/Display.h>
#endif /* USE_CORE_SDK */
#include "board.h"

#include "two_btn_menu.h"
#include "multi_role_menu.h"
#include "multi_role.h"

#include <ti/mw/lcd/LCDDogm1286.h>

#if defined( USE_FPGA ) || defined( DEBUG_SW_TRACE )
#include <driverlib/ioc.h>
#endif // USE_FPGA | DEBUG_SW_TRACE

#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <driverlib/aon_batmon.h>
#include <driverlib/aux_wuc.h>
#include "shutdn_button.h"
#include <xdc/runtime/System.h>
#ifdef DEVICE_FAMILY
    #undef DEVICE_FAMILY_PATH
    #define DEVICE_FAMILY_PATH(x) <ti/devices/DEVICE_FAMILY/x>
    #include DEVICE_FAMILY_PATH(inc/hw_prcm.h)
    #include DEVICE_FAMILY_PATH(driverlib/sys_ctrl.h)
#else
    #error "You must define DEVICE_FAMILY at the project level as one of cc26x0, cc26x0r2, cc13x0, etc."
#endif 


#include "icall_api.h"
#include "icall_apimsg.h"

//#include "oad_target.h"
//#include "oad.h"

#ifdef I2c_Acceleration
#include <ti/drivers/i2c/I2CCC26XX.h>
#include <ti/drivers/I2C.h>
#include "bsp_i2c.h"
#include "math.h"

#define LIS2DH_WHO_AM_I     0x0F
#define LIS2DH_CTRL_REG1    0x20
#define LIS2DH_CTRL_REG4    0x23

int16 Acceleration_x,Acceleration_y,Acceleration_z;
uint16 Acceler_x,Acceler_y,Acceler_z,Acceler_xyz;
uint32 Accxyz;


#endif //I2c_Acceleration
uint8_t advopen  = true;
uint8_t advclose = false;
int led_count=0;
int adv_count=0;
uint8 b[10]={0};

#include "shutdn_button.h"
#define KEY_BTN0              0x0001

#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTCC26XX.h>
#include "math.h"
/*********************************************************************
* CONSTANTS
*/

// Maximum number of scan responses
// this can only be set to 15 because that is the maximum
// amount of item actions the menu module supports
#define DEFAULT_MAX_SCAN_RES                  30

// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Connection parameters
#define DEFAULT_CONN_INT_MIN                  200
#define DEFAULT_CONN_INT_MAX                  200
#define DEFAULT_CONN_TIMEOUT                  1000
#define DEFAULT_CONN_LATENCY                  0

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           1000

// Scan parameters
#define DEFAULT_SCAN_DURATION                 4000
#define DEFAULT_SCAN_WIND                     50
#define DEFAULT_SCAN_INT                      50

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

// TRUE to use high scan duty cycle when creating link
#define DEFAULT_LINK_HIGH_DUTY_CYCLE          FALSE

// TRUE to use white list when creating link
#define DEFAULT_LINK_WHITE_LIST               FALSE

// Type of Display to open
#if !defined(Display_DISABLE_ALL)
  #ifdef USE_CORE_SDK
    #if defined(BOARD_DISPLAY_USE_LCD) && (BOARD_DISPLAY_USE_LCD!=0)
      #define MR_DISPLAY_TYPE Display_Type_LCD
    #elif defined (BOARD_DISPLAY_USE_UART) && (BOARD_DISPLAY_USE_UART!=0)
      #define MR_DISPLAY_TYPE Display_Type_UART
    #else // !BOARD_DISPLAY_USE_LCD && !BOARD_DISPLAY_USE_UART
      #define MR_DISPLAY_TYPE 0 // Option not supported
    #endif // BOARD_DISPLAY_USE_LCD && BOARD_DISPLAY_USE_UART
  #else // !USE_CORE_SDK
    #if !defined(BOARD_DISPLAY_EXCLUDE_LCD)
      #define MR_DISPLAY_TYPE Display_Type_LCD
    #elif !defined (BOARD_DISPLAY_EXCLUDE_UART)
      #define MR_DISPLAY_TYPE Display_Type_UART
    #else // BOARD_DISPLAY_EXCLUDE_LCD && BOARD_DISPLAY_EXCLUDE_UART
      #define MR_DISPLAY_TYPE 0 // Option not supported
    #endif // !BOARD_DISPLAY_EXCLUDE_LCD || !BOARD_DISPLAY_EXCLUDE_UART
  #endif // USE_CORE_SDK
#else // BOARD_DISPLAY_USE_LCD && BOARD_DISPLAY_USE_UART
  #define MR_DISPLAY_TYPE 0 // No Display
#endif // Display_DISABLE_ALL

// Task configuration
#define MR_TASK_PRIORITY                     1
#ifndef MR_TASK_STACK_SIZE
#define MR_TASK_STACK_SIZE                   610
#endif

// Internal Events for RTOS application
#define MR_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define MR_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30
#define MR_STATE_CHANGE_EVT                  Event_Id_00
#define MR_CHAR_CHANGE_EVT                   Event_Id_01
#define MR_CONN_EVT_END_EVT                  Event_Id_02
#define MR_KEY_CHANGE_EVT                    Event_Id_03
#define MR_PAIRING_STATE_EVT                 Event_Id_04
#define MR_PASSCODE_NEEDED_EVT               Event_Id_05
#define MR_PERIODIC_EVT                      Event_Id_06

#define SBP_PERIODIC_EVT                     Event_Id_07

#define SBC_SCAN_EVT                         Event_Id_08
#define SBC_SCAN_STARTED_EVT                 Event_Id_09


#define read_test                            Event_Id_10
#define Init_Evt                             Event_Id_11
#define Echo                                 Event_Id_12
#define SafeNotify                           Event_Id_13
#define Position_return                      Event_Id_14

#define MR_ALL_EVENTS                        (MR_ICALL_EVT           | \
                                             MR_QUEUE_EVT            | \
                                             MR_STATE_CHANGE_EVT     | \
                                             MR_CHAR_CHANGE_EVT      | \
                                             MR_CONN_EVT_END_EVT     | \
                                             MR_KEY_CHANGE_EVT       | \
                                             MR_PAIRING_STATE_EVT    | \
                                             MR_PERIODIC_EVT         | \
                                             MR_PASSCODE_NEEDED_EVT  | \
                                             SBP_PERIODIC_EVT        | \
                                             SBC_SCAN_EVT            | \
                                             SBC_SCAN_STARTED_EVT    | \
                                             read_test               | \
                                             Init_Evt                | \
                                             Echo                    | \
                                             SafeNotify              | \
                                             Position_return)

// Discovery states
typedef enum {
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_MTU,                 // Exchange ATT MTU size
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_CHAR                 // Characteristic discovery
} discState_t;

// Row numbers
#define MR_ROW_DEV_ADDR      (TBM_ROW_APP)
#define MR_ROW_CONN_STATUS   (TBM_ROW_APP + 1)
#define MR_ROW_ADV           (TBM_ROW_APP + 2)
#define MR_ROW_SECURITY      (TBM_ROW_APP + 3)
#define MR_ROW_STATUS1       (TBM_ROW_APP + 4)
#define MR_ROW_STATUS2       (TBM_ROW_APP + 5)

// address string length is an ascii character for each digit +
// an initial 0x + an ending null character
#define B_STR_ADDR_LEN       ((B_ADDR_LEN*2) + 3)

// How often to perform periodic event (in msec)
#define MR_PERIODIC_EVT_PERIOD               5000

/*********************************************************************
* TYPEDEFS
*/

// App event passed from profiles.
typedef struct
{
  uint16_t event;  // event type
  uint8_t *pData;  // event data pointer
} mrEvt_t;

// pairing callback event
typedef struct
{
  uint16 connectionHandle; // connection Handle
  uint8_t state;             // state returned from GAPBondMgr
  uint8_t status;            // status of state
} gapPairStateEvent_t;

// discovery information
typedef struct
{
  discState_t discState;   // discovery state
  uint16_t svcStartHdl;    // service start handle
  uint16_t svcEndHdl;      // service end handle
  uint16_t charHdl;        // characteristic handle
} discInfo_t;

// device discovery information with room for string address
typedef struct
{
  uint8_t eventType;                // Indicates advertising event type used by the advertiser: @ref GAP_ADVERTISEMENT_REPORT_TYPE_DEFINES
  uint8_t addrType;                 // Address Type: @ref ADDRTYPE_DEFINES
  uint8_t addr[B_ADDR_LEN];         // Device's Address
  uint8_t strAddr[B_STR_ADDR_LEN];  // Device Address as String
} mrDevRec_t;

// entry to map index to connection handle and store address string for menu module
typedef struct
{
  uint16_t connHandle;              // connection handle of an active connection
  uint8_t strAddr[B_STR_ADDR_LEN];  // memory location for menu module to store address string
} connHandleMapEntry_t;

/*********************************************************************
* GLOBAL VARIABLES
*/

// Display Interface
Display_Handle dispHandle = NULL;

/*********************************************************************
* LOCAL VARIABLES
*/

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


//static int8 scanrssi[20];

static uint8_t ownAddress[B_ADDR_LEN];

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

#if 0
static SPI_Handle SbpSpiHandle; 
static SPI_Params SbpSpiParams;
SPI_Transaction spiTransaction;

#define TXBUF_LEN 1
#define RXBUF_LEN 1
static uint8_t txBuf=0x0F;
static uint8_t rxBuf;
static uint8_t volatile tx = 0;
static uint8_t volatile rx = 0;
#endif

//static PIN_Handle gpioPinHandle;
//static PIN_State gpioPinState;
//static uint8_t i=0;

////UART
static UART_Handle UART_handle;
static UART_Params UART_params;
Semaphore_Struct semUARTRxStruct;
Semaphore_Handle uartSemHandle;
#define UartTimeout 100000 //unit in 10 us

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

//static uint8_t Read_Uart[50]={0x00};
static uint8_t NBIOT_Data1[84]={0x00};
static uint8_t NBIOT_Data2[92]={0x00};
static uint8_t NSOCR[26]="AT+NSOCR=DGRAM,17,2135,1\r\n";
//static uint8_t NSOST[34]="AT+NSOST=1,140.124.32.111,6666,22,";
static uint8_t  NB_Config[9] ={0x24,0x04,0xD2,0x8C,0x7C,0x20,0x6F,0x1A,0x0A};//140.124.32.111,6666,22
static uint8_t  NSOST_UDP1[40]={0x00};
static uint8_t  NSOST_UDP2[40]={0x00};

static uint8_t NBIOT_Data_Read[30]={0x00};

struct flag
{
  uint8_t NB_flag;
  uint8_t NBIot_Init_Ok;
  uint8_t  CFUN_OK_Flag;
  uint8_t  IMEI_Ok_Flag;
  uint8_t  IMSI_Ok_Flag;
  uint8_t APN_Ok_Flag;
  uint8_t CSQ_Ok_Flag;
  uint8_t REG_Ok_Flag;
  uint8_t GATT_Ok_Flag;
}flag = {0, 0, 0, 0, 0, 0, 0, 0, 0};

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

static uint8_t advertData1[21] = {0x00};
static uint8_t routedata[13] = {0x00};
static uint8_t routedata2[26] = {0x00};
static uint8_t gatewaydata[8] = {0x00};
static uint8_t gatewaydata2[16] = {0x00};
static uint8_t bicycle_number = 0;

static uint8_t slave_advdata[26] = {0x00};
static uint8_t safedata[18] = {0x00};
static uint8_t safedata2[36] = {0x00};
static uint8_t master_safedata[7] = {0x00};
static uint8_t master_safedata2[14] = {0x00}; 

//static int Testdata1[4]={0x00};
/***********SVM******************/
//float D     =0;
float D_sum =0;

static int sigma = 4;
float Base =0.030223879;
//float Alpha_Y[200]={0,0.2822,0.6553,2.2585,2.7771,3,0.7886,0.2715,3,3,0.8374,0.034,0,0.9453,1.0676,0.2791,1.0365,0.6944,1.5401,2.3072,1.2942,0,1.0907,0,0.3678,0.5093,0,0.4481,0,0.2618,1.2173,3,1.6961,1.3476,2.8382,1.003,0.605,0,0.4008,0.471,0.5295,0.0896,0.7523,0.5598,0.7955,0.3416,0,0,0.4823,0.7993,0.7862,0.5729,0.7427,0.9305,0.4164,0.8203,0.7397,0,0.8622,0.9313,0.7862,1.2751,0.1532,0,1.0738,0.5128,0.6214,1.108,0.4181,0,0.1189,0.4821,0.7006,0.1519,0.1133,0.3554,1.5704,0.104,0.7847,0.8276,0.553,0.4371,0.6896,0.3882,0.3975,0.6397,1.6863,0.1436,0.6064,0,1.1972,0,0.1999,0.734,0.1057,0.164,0.6726,0.508,0.3482,0.8731,-0.7077,-0.5748,-0.5315,-0.3215,-0.4703,-1.0137,-0.7728,0,0,-1.1725,-0.4352,-0.9569,-0.4663,-2.3624,-0.2866,-0.3841,-0.4874,-2.7173,-1.1107,-0.5996,-0.7791,0,-0.0247,0,-0.7715,-0.0944,-0.2614,0,0,-0.0383,-0.0255,-0.3509,-0.7731,-0.7271,-0.3064,-0.9414,-0.5174,-0.0683,-0.8068,-1.1074,-0.195,-0.2923,-0.8853,-0.703,-0.7973,-0.3342,-0.8232,-0.7547,-0.9549,-0.9983,-0.8457,-0.1587,0,-1.7926,-0.5639,-0.0967,-0.5565,0,-3,0,-0.3428,-1.4312,0,-0.3347,0,-0.0304,0,-0.1297,-2.2855,-1.2393,-0.0641,-0.5628,-1.0517,0,0,-0.5165,-0.3425,-0.5723,-0.5619,-0.5096,-1.3977,-3,-2.8868,-1.9301,-0.7242,-1.7591,-3,-0.4036,-0.7207,-1.4372,-0.2782,-0.7087,-0.843,-3,-0.4871,-1.3174,-1.1763,0,-0.3777,-1.839};
//
//int Trainingdata1[200]={87,83,82,84,81,85,83,83,92,93,88,88,89,86,87,84,81,89,83,87,90,88,78,77,80,76,87,89,80,88,78,77,76,76,88,76,88,76,77,83,72,68,69,68,69,68,71,68,68,70,52,59,62,54,62,51,62,62,61,65,74,76,73,69,68,65,69,64,71,70,83,70,69,85,73,69,81,70,86,71,89,78,77,74,69,81,90,81,77,86,87,85,85,86,85,83,83,85,84,91,93,95,85,92,88,93,86,86,93,79,78,77,81,77,81,72,78,72,70,70,76,88,83,83,83,84,77,89,88,82,80,85,82,82,74,82,80,83,82,82,80,88,73,77,80,71,73,84,83,76,75,77,75,88,87,86,86,86,78,87,84,82,87,88,88,85,86,87,84,90,92,93,91,88,88,91,89,91,94,92,82,83,91,83,90,93,84,89,85,87,92,94,97,92,93,90,98,92,93,90};
//int Trainingdata2[200]={92,92,91,88,92,94,92,91,91,92,93,94,91,93,90,91,92,90,94,94,88,87,85,86,86,84,88,88,88,86,89,91,87,90,87,89,86,85,91,88,83,80,97,84,81,92,84,85,91,94,78,77,80,77,80,83,90,95,96,92,74,82,79,83,81,84,85,84,87,81,69,68,71,70,68,73,65,70,70,66,67,59,61,71,66,67,67,62,68,68,58,57,48,58,50,57,48,57,56,46,86,86,84,86,85,93,91,84,91,87,83,82,92,91,87,83,89,86,93,78,73,77,77,79,75,83,78,78,78,80,81,77,75,92,77,76,88,77,87,88,79,81,79,79,83,78,79,78,70,67,89,79,81,85,83,77,84,85,93,84,73,61,71,68,72,69,72,65,68,65,77,82,65,77,73,75,76,72,79,73,87,93,88,96,88,92,91,98,97,97,94,87,93,94,94,87,92,91,90,94};
//int Trainingdata3[200]={89,87,78,90,91,80,91,82,82,84,86,86,82,85,93,81,77,79,84,88,76,74,75,71,72,71,76,70,76,70,79,78,80,89,81,76,75,72,73,77,70,70,61,72,61,68,71,71,63,68,67,59,64,67,59,61,72,66,66,55,65,71,70,71,76,77,69,81,71,68,69,78,75,69,75,70,78,71,74,68,72,77,75,69,75,69,77,73,80,71,88,82,79,81,81,82,82,88,90,77,84,92,94,91,87,80,94,92,91,91,86,76,83,74,86,81,70,78,86,83,90,93,90,93,95,92,86,88,87,85,80,80,84,83,82,76,85,84,78,83,83,93,77,86,93,78,77,87,82,91,83,82,83,81,84,87,83,83,80,83,87,87,87,87,85,86,85,85,79,80,90,89,85,82,92,80,94,92,82,85,80,82,82,92,93,86,90,87,83,83,86,88,87,82,87,81,81,89,90,90};
//int Trainingdata4[200]={56,51,51,65,66,66,48,50,69,65,58,57,56,69,58,56,66,57,70,71,68,74,66,75,78,70,78,75,77,74,77,86,85,81,82,84,77,74,75,83,76,85,78,86,88,78,77,78,78,85,83,84,80,93,86,84,83,86,87,85,79,88,88,81,81,79,91,90,80,88,87,94,86,86,94,84,91,87,88,94,86,91,84,83,95,83,94,90,90,87,89,85,85,85,86,84,84,81,84,86,79,73,81,82,78,80,82,82,75,73,93,93,88,88,91,90,88,84,85,87,86,87,87,86,83,84,87,87,87,90,91,88,78,81,83,80,72,85,90,75,68,87,66,66,90,67,86,68,67,77,95,85,87,86,94,89,85,92,84,85,87,91,87,87,86,89,87,90,93,87,87,89,97,88,88,89,88,88,91,85,64,67,67,66,69,68,64,68,64,67,70,65,89,66,82,75,71,80,86,73};

//static float k_num[200]={0};
static int volatile Testdata[4]={57,65,63,80};
static uint8_t Testdata1[4]={0x00};


enum
{
  Init_State,      //0x00
  Outside_State,   //0x01
  Inside_State,    //0x02
};

static uint8_t License_State=Init_State;
/*************************************************************************************/
/*********************************************************************
* LOCAL VARIABLES
*/
// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Clock instances for internal periodic events.
static Clock_Struct periodicClock;
static Clock_Struct myClock_init;
static Clock_Struct myClock_read_test;
static Clock_Struct myClock_safe;
static Clock_Struct myClock_return;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task configuration
Task_Struct mrTask;
Char mrTaskStack[MR_TASK_STACK_SIZE];

static uint8_t scanRspData[] =
{
  // complete name
  0x0D,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'E', 'L', 'P', '-', 'B', '4', '0', '-', '0', '9', 'F', '7',



  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

/*
// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8_t advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16(SIMPLEPROFILE_SERV_UUID),
  HI_UINT16(SIMPLEPROFILE_SERV_UUID)
};
*/

//static uint8_t advertData[] = {0x02, 0x01, 0x06, 0x03, 0x02, 0xF0, 0xFF, 0x0C,
//                               0xFF, 0x40, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
//                               0x00, 0x00, 0x00, 0x00, 0x00, 0xAB, 0xAB, 0xAB,
//                               0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x30};

static uint8_t Position_advertData[] = {0x01, 0xFF, 0x00, 0x02, 0x01, 0x06, 0x03, 0x02, 0xF0};
// pointer to allocate the connection handle map
static connHandleMapEntry_t *connHandleMap;

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Multi Role";

// Number of scan results and scan result index
static uint8_t scanRes = 0;
//static int8_t scanIdx = -1;

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;

// Pointer to per connection discovery info
discInfo_t *discInfo;

// Maximim PDU size (default = 27 octets)
static uint16 maxPduSize;

// Scanning started flag
static bool scanningStarted = FALSE;

// Connecting flag
static uint8_t connecting = FALSE;

// Scan result list
static mrDevRec_t devList[DEFAULT_MAX_SCAN_RES];

// Value to write
static uint8_t charVal = 0;

// Value read/write toggle
static bool doWrite = FALSE;

// Dummy parameters to use for connection updates
gapRole_updateConnParams_t updateParams =
{
  .connHandle = INVALID_CONNHANDLE,
  .minConnInterval = 80,
  .maxConnInterval = 150,
  .slaveLatency = 0,
  .timeoutMultiplier = 200
};

// Connection index for mapping connection handles
static uint16_t connIndex = INVALID_CONNHANDLE;

// Maximum number of connected devices
static uint8_t maxNumBleConns = MAX_NUM_BLE_CONNS;

/*********************************************************************
* LOCAL FUNCTIONS
*/
static void multi_role_init( void );
static void multi_role_taskFxn(UArg a0, UArg a1);
static uint8_t multi_role_processStackMsg(ICall_Hdr *pMsg);
static uint8_t multi_role_processGATTMsg(gattMsgEvent_t *pMsg);
static void multi_role_processAppMsg(mrEvt_t *pMsg);
static void multi_role_processCharValueChangeEvt(uint8_t paramID);
static void multi_role_processRoleEvent(gapMultiRoleEvent_t *pEvent);
static void multi_role_sendAttRsp(void);
static void multi_role_freeAttRsp(uint8_t status);
static void multi_role_charValueChangeCB(uint8_t paramID);
static uint8_t multi_role_enqueueMsg(uint16_t event, uint8_t *pData);
static void multi_role_startDiscovery(uint16_t connHandle);
static void multi_role_processGATTDiscEvent(gattMsgEvent_t *pMsg);
static void multi_role_handleKeys(uint8_t keys);
static uint8_t multi_role_eventCB(gapMultiRoleEvent_t *pEvent);
static void multi_role_paramUpdateDecisionCB(gapUpdateLinkParamReq_t *pReq,
                                             gapUpdateLinkParamReqReply_t *pRsp);
static void multi_role_sendAttRsp(void);
static void multi_role_addDeviceInfo(uint8_t *pAddr, uint8_t addrType);
static void multi_role_freeAttRsp(uint8_t status);
static uint16_t multi_role_mapConnHandleToIndex(uint16_t connHandle);
static void multi_role_keyChangeHandler(uint8_t keysPressed);
static uint8_t multi_role_addMappingEntry(uint16_t connHandle, uint8_t *addr);
static void multi_role_processPasscode(gapPasskeyNeededEvent_t *pData);
static void multi_role_processPairState(gapPairStateEvent_t* pairingEvent);
static void multi_role_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                  uint8_t uiInputs, uint8_t uiOutputs, uint32_t numComparison);
static void multi_role_pairStateCB(uint16_t connHandle, uint8_t state,
                                   uint8_t status);
static void multi_role_performPeriodicTask(void);
static void multi_role_clockHandler(UArg arg);

//自定義函數
static void NBIoT_Init(void);
static void NBIoT_Data(void);
static void NBIoT_Safe(void);
static void Voltage(void);
static void Position_judge(void);
// 讀mac//
static void Read_Mac(uint8 *Mac_Address);

static void UARTreadCallback(UART_Handle handle, void *buf, size_t count);
/*********************************************************************
 * EXTERN FUNCTIONS
*/
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
* PROFILE CALLBACKS
*/

// GAP Role Callbacks
static gapRolesCBs_t multi_role_gapRoleCBs =
{
  multi_role_eventCB,                   // Events to be handled by the app are passed through the GAP Role here
  multi_role_paramUpdateDecisionCB      // Callback for application to decide whether to accept a param update
};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t multi_role_simpleProfileCBs =
{
  multi_role_charValueChangeCB // Characteristic value change callback
};

// GAP Bond Manager Callbacks
static gapBondCBs_t multi_role_BondMgrCBs =
{
  (pfnPasscodeCB_t)multi_role_passcodeCB, // Passcode callback
  multi_role_pairStateCB                  // Pairing state callback
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/

static void Blink(const int LED, uint8_t blink_count)
{
  uint8_t LED_ON  = 10;     //time unit in microseconds
  uint8_t LED_OFF = 200;    //time unit in miniseconds
  
  uint8_t blink_counter;
  
  for (blink_counter = 0; blink_counter < blink_count; blink_counter++)
  {
    //PIN_setOutputValue(gpioPinHandle,CC2640R2DK_5XD_PIN_LED1 , CC2640R2DK_5XD_PIN_LED_ON);
    Task_sleep(LED_ON/10);
    Task_sleep(LED_OFF*100);
  }
  //PIN_setOutputValue(gpioPinHandle,CC2640R2DK_5XD_PIN_LED1 , CC2640R2DK_5XD_PIN_LED_OFF);
  
}

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
/*********************************************************************
* @fn      multi_role_createTask
*
* @brief   Task creation function for multi_role.
*
* @param   None.
*
* @return  None.
*/
void multi_role_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = mrTaskStack;
  taskParams.stackSize = MR_TASK_STACK_SIZE;
  taskParams.priority = MR_TASK_PRIORITY;

  Task_construct(&mrTask, multi_role_taskFxn, &taskParams, NULL);
}

/*********************************************************************
* @fn      multi_role_init
*
* @brief   Called during initialization and contains application
*          specific initialization (ie. hardware initialization/setup,
*          table initialization, power up notification, etc), and
*          profile initialization/setup.
*
* @param   None.
*
* @return  None.
*/
static void multi_role_init(void)
{
  //Read Macaddress
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
  
  //uint8_t i;

  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Create one-shot clocks for internal periodic events.  
  Util_constructClock(&periodicClock, multi_role_clockHandler,
                      MR_PERIODIC_EVT_PERIOD, 0, false, MR_PERIODIC_EVT);
  
  Util_constructClock(&myClock_init, multi_role_clockHandler,
                      1000, 1000, false, Init_Evt); 
  Util_constructClock(&myClock_read_test, multi_role_clockHandler,
                      0,0,false, read_test ); 
  Util_constructClock(&myClock_return, multi_role_clockHandler,
                      1000,1000,false, Position_return );
  
  
  // Init keys and LCD
  Board_initKeys(multi_role_handleKeys);
  
  // Open Display.
  dispHandle = Display_open(MR_DISPLAY_TYPE, NULL);

  /**************Init Menu*****************************/
  // Disable all menus except mrMenuScan and mrMenuAdvertise
  tbm_setItemStatus(&mrMenuMain, TBM_ITEM_0 | TBM_ITEM_5,
                    TBM_ITEM_1 | TBM_ITEM_2 | TBM_ITEM_3 | TBM_ITEM_4 );

  // Disable all items of submenus
  tbm_setItemStatus(&mrMenuConnect, TBM_ITEM_NONE, TBM_ITEM_ALL);
  tbm_setItemStatus(&mrMenuGattRw, TBM_ITEM_NONE, TBM_ITEM_ALL);
  tbm_setItemStatus(&mrMenuConnUpdate, TBM_ITEM_NONE, TBM_ITEM_ALL);
  tbm_setItemStatus(&mrMenuDisconnect, TBM_ITEM_NONE, TBM_ITEM_ALL);
  
  // Init two button menu
  tbm_initTwoBtnMenu(dispHandle, &mrMenuMain, 1, NULL);
  
  // Setup the GAP
  {
    // Set advertising interval the same for all scenarios
    uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_CONN_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_CONN_ADV_INT_MAX, advInt);

    // Set scan duration
    GAP_SetParamValue(TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION);

    // Scan interval and window the same for all scenarios
    GAP_SetParamValue(TGAP_CONN_SCAN_INT, DEFAULT_SCAN_INT);
    GAP_SetParamValue(TGAP_CONN_SCAN_WIND, DEFAULT_SCAN_WIND);
    GAP_SetParamValue(TGAP_CONN_HIGH_SCAN_INT, DEFAULT_SCAN_INT);
    GAP_SetParamValue(TGAP_CONN_HIGH_SCAN_WIND, DEFAULT_SCAN_WIND);
    GAP_SetParamValue(TGAP_GEN_DISC_SCAN_INT, DEFAULT_SCAN_INT);
    GAP_SetParamValue(TGAP_GEN_DISC_SCAN_WIND, DEFAULT_SCAN_WIND);
    GAP_SetParamValue(TGAP_LIM_DISC_SCAN_INT, DEFAULT_SCAN_INT);
    GAP_SetParamValue(TGAP_LIM_DISC_SCAN_WIND, DEFAULT_SCAN_WIND);
    GAP_SetParamValue(TGAP_CONN_EST_SCAN_INT, DEFAULT_SCAN_INT);
    GAP_SetParamValue(TGAP_CONN_EST_SCAN_WIND, DEFAULT_SCAN_WIND);

    // Set connection parameters
    GAP_SetParamValue(TGAP_CONN_EST_INT_MIN, DEFAULT_CONN_INT_MIN);
    GAP_SetParamValue(TGAP_CONN_EST_INT_MAX, DEFAULT_CONN_INT_MAX);
    GAP_SetParamValue(TGAP_CONN_EST_SUPERV_TIMEOUT, DEFAULT_CONN_TIMEOUT);
    GAP_SetParamValue(TGAP_CONN_EST_LATENCY, DEFAULT_CONN_LATENCY);

    // Register to receive GAP and HCI messages
    GAP_RegisterForMsgs(selfEntity);
  }

  // Setup the GAP Role Profile
  {
    /*--------PERIPHERAL-------------*/
    uint8_t initialAdvertEnable = TRUE;
    uint16_t advertOffTime = 0;
    uint8_t channelMap = GAP_ADVCHAN_ALL;

    // device starts advertising upon initialization
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &initialAdvertEnable, NULL);
    GAPRole_SetParameter(GAPROLE_ADV_CHANNEL_MAP, sizeof(uint8_t),
                         &channelMap, NULL);                                          //更改廣播通道[37,38,39,ALL]

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                         &advertOffTime, NULL);

    // Set scan response data
    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData),
                         scanRspData, NULL);

    // Set advertising data
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(Position_advertData), Position_advertData, NULL);

    // set max amount of scan responses
    uint8_t scanRes = DEFAULT_MAX_SCAN_RES;

    // Set the max amount of scan responses
    GAPRole_SetParameter(GAPROLE_MAX_SCAN_RES, sizeof(uint8_t),
                         &scanRes, NULL);

    // Start the GAPRole and negotiate max number of connections
    VOID GAPRole_StartDevice(&multi_role_gapRoleCBs, &maxNumBleConns);

    // Allocate memory for index to connection handle map
    if (connHandleMap = ICall_malloc(sizeof(connHandleMapEntry_t) * maxNumBleConns))
    {
      // Init index to connection handle map
      for (uint8_t i = 0; i < maxNumBleConns; i++)
      {
        connHandleMap[i].connHandle = INVALID_CONNHANDLE;
      }
    }

    // Allocate memory for per connection discovery information
    if (discInfo = ICall_malloc(sizeof(discInfo_t) * maxNumBleConns))
    {
      // Init index to connection handle map to 0's
      for (uint8_t i = 0; i < maxNumBleConns; i++)
      {
        discInfo[i].charHdl = 0;
        discInfo[i].discState = BLE_DISC_STATE_IDLE;
        discInfo[i].svcEndHdl = 0;
        discInfo[i].svcStartHdl = 0;
      }
    }
  }

  // GATT
  {
    // Set the GAP Characteristics
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

    // Initialize GATT Server Services
    GGS_AddService(GATT_ALL_SERVICES);           // GAP
    GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
    DevInfo_AddService();                        // Device Information Service
    SimpleProfile_AddService(GATT_ALL_SERVICES); // Simple GATT Profile

    // Setup Profile Characteristic Values
    {
      uint8_t charValue1 = 1;
      uint8_t charValue2 = 2;
      uint8_t charValue3 = 3;
      uint8_t charValue4 = 4;
      uint8_t charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5 };

      SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1, sizeof(uint8_t),
                                 &charValue1);
      SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2, sizeof(uint8_t),
                                 &charValue2);
      SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR3, sizeof(uint8_t),
                                 &charValue3);
      SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint8_t),
                                 &charValue4);
      SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN,
                                 charValue5);
    }

    // Register callback with SimpleGATTprofile
    SimpleProfile_RegisterAppCBs(&multi_role_simpleProfileCBs);

    /*-----------------CLIENT------------------*/
    // Initialize GATT Client
    VOID GATT_InitClient();

    // Register for GATT local events and ATT Responses pending for transmission
    GATT_RegisterForMsgs(selfEntity);

    // Register to receive incoming ATT Indications/Notifications
    GATT_RegisterForInd(selfEntity);
  }

  // Setup the GAP Bond Manager
  {
    uint8_t pairMode = GAPBOND_PAIRING_MODE_INITIATE;
    uint8_t mitm = TRUE;
    uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8_t bonding = TRUE;

    // Set pairing mode
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);

    // Set authentication requirements
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);

    // Set I/O capabilities
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);

    // Sst bonding requirements
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);

    // Register and start Bond Manager
    VOID GAPBondMgr_Register(&multi_role_BondMgrCBs);
  }

#if !defined (USE_LL_CONN_PARAM_UPDATE)
  // Get the currently set local supported LE features
  // The HCI will generate an HCI event that will get received in the main
  // loop
  HCI_LE_ReadLocalSupportedFeaturesCmd();
#endif // !defined (USE_LL_CONN_PARAM_UPDATE)  

  //PIN_setOutputValue(gpioPinHandle,CC2640R2DK_5XD_PIN_LED1 , 0);
  //Util_startClock(&myClock_init);
}

/*********************************************************************
* @fn      multi_role_taskFxn
*
* @brief   Application task entry point for the multi_role.
*
* @param   a0, a1 - not used.
*
* @return  None.
*/

static void multi_role_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  multi_role_init();

  //uint8_t index = 0;
  
  
  
  hSbpPins=PIN_open(&sbpPins, gpioPinTable);
  //PIN_setOutputValue(hSbpPins, RED_LED, 0);
  PIN_setOutputValue(hSbpPins, GREEN_LED, 1);
  
  GAPRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
                                DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                DEFAULT_DISCOVERY_WHITE_LIST);
  
  Position_judge();

  
  // Application main loop
  for (;;)
  {
    uint32_t events;
    
    // Waits for an event to be posted associated with the calling thread.
    // Note that an event associated with a thread is posted when a
    // message is queued to the message receive queue of the thread
    events = Event_pend(syncEvent, Event_Id_NONE, MR_ALL_EVENTS,
                        ICALL_TIMEOUT_FOREVER);
    
    if (events)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;
      
      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8_t safeToDealloc = TRUE;
        
        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;
          
          // Check for BLE stack events first
          if (pEvt->signature == 0xffff)
          {
            if (pEvt->event_flag & MR_CONN_EVT_END_EVT)
            {
              // Try to retransmit pending ATT Response (if any)
              multi_role_sendAttRsp();
            }
          }
          else
          {
            // Process inter-task message
            safeToDealloc = multi_role_processStackMsg((ICall_Hdr *)pMsg);
          }
        }
        
        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }
      
      // If RTOS queue is not empty, process app message.
      if (events & MR_QUEUE_EVT)
      {
        while (!Queue_empty(appMsgQueue))
        {
          mrEvt_t *pMsg = (mrEvt_t *)Util_dequeueMsg(appMsgQueue);
          if (pMsg)
          {
            // Process message.
            multi_role_processAppMsg(pMsg);
            
            // Free the space from the message.
            ICall_free(pMsg);
          }
        }
      }
      
      if (events & MR_PERIODIC_EVT)
      {
        Util_startClock(&periodicClock);
        
        // Perform periodic application task
        multi_role_performPeriodicTask();
      }      
      
      if (events & Init_Evt)
      {
         events &= ~Init_Evt;
         
         // Perform periodic application task
        multi_role_performPeriodicTask();
        
         //Voltage();    
         //NBIoT_Init();       //NBIOT初始化
         GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),&advclose,NULL);
        
        if(flag.NBIot_Init_Ok == 1)
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

           GAPRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
                         DEFAULT_DISCOVERY_ACTIVE_SCAN,
                         DEFAULT_DISCOVERY_WHITE_LIST); 

         Voltage();
         //Time();
         
         Testdata1[0] = 0x56;
         Testdata1[1] = 0x62;
         Testdata1[2] = 0x65;
         Testdata1[3] = 0x80;
          
       if(flag.NB_flag ==1)
       {
//         for(int x=0; x<4; x++)
//         {
//           Testdata1[x]=routedata[x+7];
//         }
//         
//          char a[4]={0x00}; 
//         if(memcmp(a,Testdata1,4)!=0)
//           {
//             Position_judge();
//           }
         
         //Position_judge();
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
           flag.NB_flag = 0;

           
           Util_stopClock(&myClock_read_test);
           Util_startClock(&myClock_init);
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
           memset(advertData1 , 0 , sizeof(advertData1));
           memset(routedata , 0 , sizeof(routedata));
           memset(slave_advdata, 0, sizeof(slave_advdata));
           memset(safedata, 0, sizeof(safedata));
//            UART_read(UART_handle,NBIOT_Data_Read, sizeof(NBIOT_Data_Read));  
//            UART_write(UART_handle,NBIOT_Data,sizeof(NBIOT_Data));
//            Semaphore_pend(uartSemHandle, UartTimeout);  
           //Util_stopClock(&myClock_read_test);
           Util_startClock(&myClock_return);
         }    
         }
      
      if (events & Position_return)
      {
        events &= ~Position_return;
        
        if(Position_flag == 1)
        {
          Position_advertData[2] = License_State;
          GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(Position_advertData), Position_advertData, NULL);
          GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),&advopen,NULL);
          Position_flag = 0;
          Util_stopClock(&myClock_return);
          Util_startClock(&myClock_init);
        }
      }
    }
  }
}


/*********************************************************************
* @fn      multi_role_processStackMsg
*
* @brief   Process an incoming stack message.
*
* @param   pMsg - message to process
*
* @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
*/
static uint8_t multi_role_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = multi_role_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
      {
        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            // Process HCI Command Complete Event
            {
  #if !defined (USE_LL_CONN_PARAM_UPDATE)
                
              // This code will disable the use of the LL_CONNECTION_PARAM_REQ
              // control procedure (for connection parameter updates, the 
              // L2CAP Connection Parameter Update procedure will be used 
              // instead). To re-enable the LL_CONNECTION_PARAM_REQ control
              // procedures, define the symbol USE_LL_CONN_PARAM_UPDATE                
              
              // Parse Command Complete Event for opcode and status
              hciEvt_CmdComplete_t* command_complete = (hciEvt_CmdComplete_t*) pMsg;
              uint8_t   pktStatus = command_complete->pReturnParam[0]; 
              
              //find which command this command complete is for
              switch (command_complete->cmdOpcode)
              {
                case HCI_LE_READ_LOCAL_SUPPORTED_FEATURES:
                  {
                    if (pktStatus == SUCCESS)
                    {
                      uint8_t featSet[8];
                      
                      // get current feature set from received event (bits 1-9 of
                      // the returned data
                      memcpy( featSet, &command_complete->pReturnParam[1], 8 );                          
                      
                      // Clear bit 1 of byte 0 of feature set to disable LL
                      // Connection Parameter Updates
                      CLR_FEATURE_FLAG( featSet[0], LL_FEATURE_CONN_PARAMS_REQ );
                        
                      // Update controller with modified features
                      HCI_EXT_SetLocalSupportedFeaturesCmd( featSet );
                    }
                  }
                  break;
                
                default:
                  //do nothing
                  break;
              }   
  #endif // !defined (USE_LL_CONN_PARAM_UPDATE)
                  
            }          
            break;

          case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
            AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
            break;

          default:
            break;
        }
      }
      break;

    case GAP_MSG_EVENT:
      multi_role_processRoleEvent((gapMultiRoleEvent_t *)pMsg);
      break;

    default:
      // Do nothing
      break;
  }

  return (safeToDealloc);
}

/*********************************************************************
* @fn      multi_role_processGATTMsg
*
* @brief   Process GATT messages and events.
*
* @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
*/
static uint8_t multi_role_processGATTMsg(gattMsgEvent_t *pMsg)
{
  // See if GATT server was unable to transmit an ATT response
  if (pMsg->hdr.status == blePending)
  {
    // No HCI buffer was available. Let's try to retransmit the response
    // on the next connection event.
    if (HCI_EXT_ConnEventNoticeCmd(pMsg->connHandle, selfEntity,
                                   MR_CONN_EVT_END_EVT) == SUCCESS)
    {
      // First free any pending response
      multi_role_freeAttRsp(FAILURE);

      // Hold on to the response message for retransmission
      pAttRsp = pMsg;

      // Don't free the response message yet
      return (FALSE);
    }
  }
  else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.

    // Display the opcode of the message that caused the violation.
    Display_print1(dispHandle, MR_ROW_STATUS1, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
    Display_print1(dispHandle, MR_ROW_STATUS1, 0, "MTU Size: %d", pMsg->msg.mtuEvt.MTU);
  }

  // Messages from GATT server
  if (linkDB_NumActive() > 0)
  {
    // Find index from connection handle
    connIndex = multi_role_mapConnHandleToIndex(pMsg->connHandle);
    if ((pMsg->method == ATT_READ_RSP)   ||
        ((pMsg->method == ATT_ERROR_RSP) &&
         (pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ)))
    {
      if (pMsg->method == ATT_ERROR_RSP)
      {
        Display_print1(dispHandle, MR_ROW_STATUS2, 0, "Read Error %d", pMsg->msg.errorRsp.errCode);
      }
      else
      {
        // After a successful read, display the read value
        Display_print1(dispHandle, MR_ROW_STATUS2, 0, "Read rsp: %d", pMsg->msg.readRsp.pValue[0]);
      }

    }
    else if ((pMsg->method == ATT_WRITE_RSP)  ||
             ((pMsg->method == ATT_ERROR_RSP) &&
              (pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ)))
    {

      if (pMsg->method == ATT_ERROR_RSP == ATT_ERROR_RSP)
      {
        Display_print1(dispHandle, MR_ROW_STATUS2, 0, "Write Error %d", pMsg->msg.errorRsp.errCode);
      }
      else
      {
        // After a succesful write, display the value that was written and
        // increment value
        Display_print1(dispHandle, MR_ROW_STATUS2, 0, "Write sent: %d", charVal++);
      }
    }
    else if (discInfo[connIndex].discState != BLE_DISC_STATE_IDLE)
    {
      multi_role_processGATTDiscEvent(pMsg);
    }
  } // Else - in case a GATT message came after a connection has dropped, ignore it.

  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
* @fn      multi_role_sendAttRsp
*
* @brief   Send a pending ATT response message.
*
* @param   none
*
* @return  none
*/
static void multi_role_sendAttRsp(void)
{
  // See if there's a pending ATT Response to be transmitted
  if (pAttRsp != NULL)
  {
    uint8_t status;

    // Increment retransmission count
    rspTxRetry++;

    // Try to retransmit ATT response till either we're successful or
    // the ATT Client times out (after 30s) and drops the connection.
    status = GATT_SendRsp(pAttRsp->connHandle, pAttRsp->method, &(pAttRsp->msg));
    if ((status != blePending) && (status != MSG_BUFFER_NOT_AVAIL))
    {
      // Disable connection event end notice
      HCI_EXT_ConnEventNoticeCmd(pAttRsp->connHandle, selfEntity, 0);

      // We're done with the response message
      multi_role_freeAttRsp(status);
    }
    else
    {
      // Continue retrying
      Display_print1(dispHandle, MR_ROW_STATUS2, 0, "Rsp send retry:", rspTxRetry);
    }
  }
}

/*********************************************************************
* @fn      multi_role_freeAttRsp
*
* @brief   Free ATT response message.
*
* @param   status - response transmit status
*
* @return  none
*/
static void multi_role_freeAttRsp(uint8_t status)
{
  // See if there's a pending ATT response message
  if (pAttRsp != NULL)
  {
    // See if the response was sent out successfully
    if (status == SUCCESS)
    {
      Display_print1(dispHandle, MR_ROW_STATUS2, 0, "Rsp sent, retry: %d", rspTxRetry);
    }
    else
    {
      // Free response payload
      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);

      Display_print1(dispHandle, MR_ROW_STATUS2, 0, "Rsp retry failed: %d", rspTxRetry);
    }

    // Free response message
    ICall_freeMsg(pAttRsp);

    // Reset our globals
    pAttRsp = NULL;
    rspTxRetry = 0;
  }
}

/*********************************************************************
* @fn      multi_role_processAppMsg
*
* @brief   Process an incoming callback from a profile.
*
* @param   pMsg - message to process
*
* @return  None.
*/
static void multi_role_processAppMsg(mrEvt_t *pMsg)
{
  switch (pMsg->event)
  {
  case MR_STATE_CHANGE_EVT:
    multi_role_processStackMsg((ICall_Hdr *)pMsg->pData);
    // Free the stack message
    ICall_freeMsg(pMsg->pData);
    break;

  case MR_CHAR_CHANGE_EVT:
    multi_role_processCharValueChangeEvt(*(pMsg->pData));
    // Free the app data
    ICall_free(pMsg->pData);
    break;

  case MR_KEY_CHANGE_EVT:
    multi_role_handleKeys(*(pMsg->pData));
    // Free the app data
    ICall_free(pMsg->pData);
    break;

  case MR_PAIRING_STATE_EVT:
    multi_role_processPairState((gapPairStateEvent_t*)pMsg->pData);
    // Free the app data
    ICall_free(pMsg->pData);
    break;

  case MR_PASSCODE_NEEDED_EVT:
    multi_role_processPasscode((gapPasskeyNeededEvent_t*)pMsg->pData);
    // Free the app data
    ICall_free(pMsg->pData);
    break;

  default:
    // Do nothing.
    break;
  }
}

/*********************************************************************
* @fn      multi_role_eventCB
*
* @brief   Multi GAPRole event callback function.
*
* @param   pEvent - pointer to event structure
*
* @return  TRUE if safe to deallocate event message, FALSE otherwise.
*/
static uint8_t multi_role_eventCB(gapMultiRoleEvent_t *pEvent)
{
  // Forward the role event to the application
  if (multi_role_enqueueMsg(MR_STATE_CHANGE_EVT, (uint8_t *)pEvent))
  {
    // App will process and free the event
    return FALSE;
  }

  // Caller should free the event
  return TRUE;
}

/*********************************************************************
* @fn      multi_role_paramUpdateDecisionCB
*
* @brief   Callback for application to decide whether or not to accept
*          a parameter update request and, if accepted, what parameters
*          to use
*
* @param   pReq - pointer to param update request
* @param   pRsp - pointer to param update response
*
* @return  none
*/
static void multi_role_paramUpdateDecisionCB(gapUpdateLinkParamReq_t *pReq,
                                             gapUpdateLinkParamReqReply_t *pRsp)
{
  // Make some decision based on desired parameters. Here is an example
  // where only parameter update requests with 0 slave latency are accepted
  if (pReq->connLatency == 0)
  {
    // Accept and respond with remote's desired parameters
    pRsp->accepted = TRUE;
    pRsp->connLatency = pReq->connLatency;
    pRsp->connTimeout = pReq->connTimeout;
    pRsp->intervalMax = pReq->intervalMax;
    pRsp->intervalMin = pReq->intervalMin;
  }

  // Don't accept param update requests with slave latency other than 0
  else
  {
    pRsp->accepted = FALSE;
  }
}

/*********************************************************************
* @fn      multi_role_processRoleEvent
*
* @brief   Multi role event processing function.
*
* @param   pEvent - pointer to event structure
*
* @return  none
*/
static void multi_role_processRoleEvent(gapMultiRoleEvent_t *pEvent)
{
  //uint8 bNode_parrent[9]={0x02, 0x01, 0x06, 0x1A, 0xFF, 0x4C, 0x00, 0x02, 0x15};
  switch (pEvent->gap.opcode)
  {
    // GAPRole started
    case GAP_DEVICE_INIT_DONE_EVENT:
    {
      // Store max pdu size
      maxPduSize = pEvent->initDone.dataPktLen;

      Display_print0(dispHandle, MR_ROW_DEV_ADDR, 0, Util_convertBdAddr2Str(pEvent->initDone.devAddr));
      Display_print0(dispHandle, MR_ROW_CONN_STATUS, 0, "Connected to 0");
      Display_print0(dispHandle, MR_ROW_STATUS1, 0, "Initialized");

      // Set device info characteristic
      DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, pEvent->initDone.devAddr);
    }
    break;

    // Advertising started
    case GAP_MAKE_DISCOVERABLE_DONE_EVENT:
    {
      Display_print0(dispHandle, MR_ROW_ADV, 0, "Advertising");
    }
    break;

    // Advertising ended
    case GAP_END_DISCOVERABLE_DONE_EVENT:
    {
      // Display advertising info depending on whether there are any connections
      if (linkDB_NumActive() < maxNumBleConns)
      {
        Display_print0(dispHandle, MR_ROW_ADV, 0, "Ready to Advertise");
      }
      else
      {
        Display_print0(dispHandle, MR_ROW_ADV, 0, "Can't Adv : Max conns reached");
      }
    }
    break;

    // A discovered device report
    case GAP_DEVICE_INFO_EVENT:
    {
      if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
        {
          //if (SimpleBLECentral_findSvcUuid(SIMPLEPROFILE_SERV_UUID,
                                           //pEvent->deviceInfo.pEvtData,
                                           //pEvent->deviceInfo.dataLen))
          
          
          
          multi_role_addDeviceInfo(pEvent->deviceInfo.addr,
                                   pEvent->deviceInfo.addrType);

          if(pEvent->deviceInfo.pEvtData[2] == '$')
            {
              memcpy(advertData1,pEvent->deviceInfo.pEvtData,pEvent->deviceInfo.dataLen); 
              memcpy(&routedata[0], &advertData1[2], 13);
              flag.NB_flag         = 1;
              
              GAPRole_CancelDiscovery();
            }
            
            if(pEvent->deviceInfo.pEvtData[2] == '@')
            {
              memcpy(slave_advdata,pEvent->deviceInfo.pEvtData,pEvent->deviceInfo.dataLen); 
              memcpy(&safedata[0], &slave_advdata[2], 18);
              Safe_flag         = 1;
              GAPRole_CancelDiscovery();
            }

        }
    }
    break;

    // End of discovery report
    case GAP_DEVICE_DISCOVERY_EVENT:
    {
      uint8_t i;
      
      // Discovery complete
      scanningStarted = FALSE;

      // If devices were found
      if (pEvent->discCmpl.numDevs > 0)
      {
        // Update menu
        tbm_setItemStatus(&mrMenuMain, TBM_ITEM_1, TBM_ITEM_NONE);
        // Loop through discovered devices to store in static device list
        for (i = 0; i < pEvent->discCmpl.numDevs; i++)
        {
          // Store address type
          devList[i].addrType = pEvent->discCmpl.pDevList[i].addrType;

          // Store event type (adv / scan response)
          devList[i].eventType = pEvent->discCmpl.pDevList[i].eventType;

          // Store address
          memcpy(devList[i].addr, pEvent->discCmpl.pDevList[i].addr, B_ADDR_LEN);

          // Convert address to string
          uint8_t *pAddr = (uint8_t*)Util_convertBdAddr2Str(devList[i].addr);

          // Copy converted string to static device list
          memcpy(devList[i].strAddr, pAddr, B_STR_ADDR_LEN);

          // Set the action description in the connect submenu
          TBM_SET_ACTION_DESC(&mrMenuConnect, i, devList[i].strAddr);
          tbm_setItemStatus(&mrMenuConnect, (1 << i) , TBM_ITEM_NONE);
        }

        // Disable any non-active scan results
        for (; i < (DEFAULT_MAX_SCAN_RES - 1); i++)
        {
          tbm_setItemStatus(&mrMenuConnect, TBM_ITEM_NONE, (1 << i));
        }
      }

      Display_print1(dispHandle, MR_ROW_STATUS1, 0, "Devices Found %d", pEvent->discCmpl.numDevs);
    }
    break;

    // Connection has been established
    case GAP_LINK_ESTABLISHED_EVENT:
    {
      // If succesfully established
      if (pEvent->gap.hdr.status == SUCCESS)
      {
        Display_print0(dispHandle, MR_ROW_STATUS1, 0, "Connected!");
        Display_print1(dispHandle, MR_ROW_CONN_STATUS, 0, "Connected to %d", linkDB_NumActive());

        // Clear connecting flag
        connecting = FALSE;

        // Add index-to-connHandle mapping entry and update menus
        uint8_t index = multi_role_addMappingEntry(pEvent->linkCmpl.connectionHandle, pEvent->linkCmpl.devAddr);

        //turn off advertising if no available links
        if (linkDB_NumActive() >= maxNumBleConns)
        {
          uint8_t advertEnabled = FALSE;
          GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &advertEnabled, NULL);
          Display_print0(dispHandle, MR_ROW_ADV, 0, "Can't adv: no links");
        }

        // Print last connected device
        Display_print0(dispHandle, MR_ROW_STATUS2, 0, (char*)connHandleMap[index].strAddr);

        // Return to main menu
        tbm_goTo(&mrMenuMain);

        // Start service discovery
        multi_role_startDiscovery(pEvent->linkCmpl.connectionHandle);

        // Start periodic clock if this is the first connection
        if (linkDB_NumActive() == 1)
        {
          Util_startClock(&periodicClock);
        }
      }
      // If the connection was not successfully established
      else
      {
        Display_print0(dispHandle, MR_ROW_STATUS1, 0, "Connect Failed");
        Display_print1(dispHandle, MR_ROW_STATUS2, 0, "Reason: %d", pEvent->gap.hdr.status);
      }
    }
    break;

    // Connection has been terminated
    case GAP_LINK_TERMINATED_EVENT:
    {
      // read current num active so that this doesn't change before this event is processed
      uint8_t currentNumActive = linkDB_NumActive();
      
      // Find index from connection handle
      connIndex = multi_role_mapConnHandleToIndex(pEvent->linkTerminate.connectionHandle);

      // Check to prevent buffer overrun
      if (connIndex < maxNumBleConns)
      {
        // Clear screen, reset discovery info, and return to main menu
        connHandleMap[connIndex].connHandle = INVALID_CONNHANDLE;
        
        // Reset discovery info
        discInfo[connIndex].discState = BLE_DISC_STATE_IDLE;
        discInfo[connIndex].charHdl = 0;        
       
        // Disable connection item from connectable menus
        tbm_setItemStatus(&mrMenuGattRw, TBM_ITEM_NONE, (1 << connIndex));
        tbm_setItemStatus(&mrMenuConnUpdate, TBM_ITEM_NONE, (1 << connIndex));
        tbm_setItemStatus(&mrMenuDisconnect, TBM_ITEM_NONE, (1 << connIndex));  
       
        // If there aren't any active connections
        if (currentNumActive == 0)
        {
          // Disable connectable menus 
          tbm_setItemStatus(&mrMenuMain, TBM_ITEM_NONE, 
                            TBM_ITEM_2 | TBM_ITEM_3 | TBM_ITEM_4);
          
          // Stop periodic clock
          Util_stopClock(&periodicClock);
        }
        
        // Clear screen
        Display_print1(dispHandle, MR_ROW_CONN_STATUS, 0, "Connected to %d", linkDB_NumActive());
        Display_print0(dispHandle, MR_ROW_STATUS1, 0, "Disconnected!");  
        
        // If it is possible to advertise again
        if (currentNumActive == (maxNumBleConns-1)) 
        {
          Display_print0(dispHandle, MR_ROW_ADV, 0, "Ready to Advertise");
          Display_print0(dispHandle, MR_ROW_STATUS2, 0, "Ready to Scan");
        }      
      }
    }
    break;

    // A parameter update has occurred
    case GAP_LINK_PARAM_UPDATE_EVENT:
    {
      Display_print1(dispHandle, MR_ROW_STATUS1, 0, "Param Update %d", pEvent->linkUpdate.status);
    }
    break;

  default:
    break;
  }
}

/*********************************************************************
* @fn      multi_role_charValueChangeCB
*
* @brief   Callback from Simple Profile indicating a characteristic
*          value change.
*
* @param   paramID - parameter ID of the value that was changed.
*
* @return  None.
*/
static void multi_role_charValueChangeCB(uint8_t paramID)
{
  uint8_t *pData;

  // Allocate space for the event data.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = paramID;

    // Queue the event.
    multi_role_enqueueMsg(MR_CHAR_CHANGE_EVT, pData);
  }
}

/*********************************************************************
* @fn      multi_role_processCharValueChangeEvt
*
* @brief   Process a pending Simple Profile characteristic value change
*          event.
*
* @param   paramID - parameter ID of the value that was changed.
*
* @return  None.
*/
static void multi_role_processCharValueChangeEvt(uint8_t paramID)
{
  uint8_t newValue;
  uint8_t newValue3;

  // Print new value depending on which characteristic was updated
  switch(paramID)
  {
  case SIMPLEPROFILE_CHAR1:
    // Get new value
    SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR1, &newValue);

    Display_print1(dispHandle, MR_ROW_STATUS2, 0, "Char 1: %d", (uint16_t)newValue);
    break;

  case SIMPLEPROFILE_CHAR3:
    // Get new value
    SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR3, &newValue3);
    
   // if( newValue3[0] == 0x21)
   {
//      writedata[0] = 1;                  //flash
//      
//      writedata[1] = newValue3[1];       //car_id
//      routedata[1] = writedata[1];
//      osal_snv_write(0x80, 2, &writedata[0]);
    }
    break;

  default:
    // Should not reach here!
    break;
  }
}

/*********************************************************************
* @fn      multi_role_enqueueMsg
*
* @brief   Creates a message and puts the message in RTOS queue.
*
* @param   event - message event.
* @param   pData - pointer to data to be queued
*
* @return  None.
*/
static uint8_t multi_role_enqueueMsg(uint16_t event, uint8_t *pData)
{
  // Allocate space for the message
  mrEvt_t *pMsg = ICall_malloc(sizeof(mrEvt_t));

  // If sucessfully allocated
  if (pMsg)
  {
    // Fill up message
    pMsg->event = event;
    pMsg->pData = pData;

    // Enqueue the message.
    return Util_enqueueMsg(appMsgQueue, syncEvent, (uint8_t *)pMsg);
  }

  return FALSE;
}

/*********************************************************************
* @fn      multi_role_keyChangeHandler
*
* @brief   Key event handler function
*
* @param   a0 - ignored
*
* @return  none
*/

void multi_role_keyChangeHandler(uint8_t keys)
{
  uint8_t *pData;

  // Allocate space for the event data.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    // Store the key data
    *pData = keys;

    // Queue the event.
    multi_role_enqueueMsg(MR_KEY_CHANGE_EVT, pData);
  }
}

/*********************************************************************
* @fn      multi_role_handleKeys
*
* @brief   Handles all key events for this device.
*
* @param   keys - bit field for key events. Valid entries:
*                 HAL_KEY_SW_2
*                 HAL_KEY_SW_1
*
* @return  none
*/
static void multi_role_handleKeys(uint8_t keys)
{
  if( keys & KEY_BTN0)
  {
    Blink(CC2640R2DK_5XD_PIN_LED1, 5);
    Power_shutdown(NULL, 0); 
  }
  else{
    if( SysCtrlResetSourceGet() == RSTSRC_WAKEUP_FROM_SHUTDOWN )
    {
      Util_startClock(&myClock_init);
      //Blink(GREEN_LED, 2);
    }
  }
  /*
  if (PIN_getInputValue(Board_BTN0) == 0)
  {  
      Blink(CC2640R2DK_5XD_PIN_LED1, 5);
      //PINCC26XX_setWakeup(ButtonTable);
      Power_shutdown(NULL, 0);    
  }
  else{
    if( SysCtrlResetSourceGet() == RSTSRC_WAKEUP_FROM_SHUTDOWN ) 
      {
         //Util_startClock(&ledperiodicClock);
         //Blink(GREEN_LED, 2);
      }
  }
  */
  
  /*
  if (keys & KEY_LEFT)
  {
    // Check if the key is still pressed
    if (PIN_getInputValue(Board_BUTTON0) == 0)
    {
      tbm_buttonLeft();
    }
  }
  else if (keys & KEY_RIGHT)
  {
    // Check if the key is still pressed
    if (PIN_getInputValue(Board_BUTTON1) == 0)
    {
      tbm_buttonRight();
    }
  }
*/
}


/*********************************************************************
* @fn      multi_role_startDiscovery
*
* @brief   Start service discovery.
*
* @param   connHandle - connection handle
*
* @return  none
*/
static void multi_role_startDiscovery(uint16_t connHandle)
{
  // Exchange MTU request
  attExchangeMTUReq_t req;

  // Map connection handle to index
  connIndex = multi_role_mapConnHandleToIndex(connHandle);

  // Check to prevent buffer overrun
  if (connIndex < maxNumBleConns)
  {
    // Update discovery state of this connection
    discInfo[connIndex].discState= BLE_DISC_STATE_MTU;

    // Initialize cached handles
    discInfo[connIndex].svcStartHdl = discInfo[connIndex].svcEndHdl = 0;
  }

  // Discover GATT Server's Rx MTU size
  req.clientRxMTU = maxPduSize - L2CAP_HDR_SIZE;

  // ATT MTU size should be set to the minimum of the Client Rx MTU
  // and Server Rx MTU values
  VOID GATT_ExchangeMTU(connHandle, &req, selfEntity);
}

/*********************************************************************
* @fn      multi_role_processGATTDiscEvent
*
* @brief   Process GATT discovery event
*
* @param   pMsg - pointer to discovery event stack message
*
* @return  none
*/
static void multi_role_processGATTDiscEvent(gattMsgEvent_t *pMsg)
{
  // Map connection handle to index
  connIndex = multi_role_mapConnHandleToIndex(pMsg->connHandle);
  // Check to prevent buffer overrun
  if (connIndex < maxNumBleConns)
  {
    //MTU update
    if (pMsg->method == ATT_MTU_UPDATED_EVENT)
    {
      // MTU size updated
      Display_print1(dispHandle, MR_ROW_STATUS1, 0, "MTU Size: %d", pMsg->msg.mtuEvt.MTU);
    }
    // If we've updated the MTU size
    else if (discInfo[connIndex].discState == BLE_DISC_STATE_MTU)
    {
      // MTU size response received, discover simple BLE service
      if (pMsg->method == ATT_EXCHANGE_MTU_RSP)
      {
        uint8_t uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(SIMPLEPROFILE_SERV_UUID),
        HI_UINT16(SIMPLEPROFILE_SERV_UUID) };

        // Advance state
        discInfo[connIndex].discState= BLE_DISC_STATE_SVC;

        // Discovery of simple BLE service
        VOID GATT_DiscPrimaryServiceByUUID(pMsg->connHandle, uuid, ATT_BT_UUID_SIZE,
                                           selfEntity);
      }
    }
    // If we're performing service discovery
    else if (discInfo[connIndex].discState == BLE_DISC_STATE_SVC)
    {
      // Service found, store handles
      if (pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
          pMsg->msg.findByTypeValueRsp.numInfo > 0)
      {
        discInfo[connIndex].svcStartHdl = ATT_ATTR_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
        discInfo[connIndex].svcEndHdl = ATT_GRP_END_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
      }

      // If procedure is complete
      if (((pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP) &&
           (pMsg->hdr.status == bleProcedureComplete))  ||
          (pMsg->method == ATT_ERROR_RSP))
      {
        // If we've discovered the service
        if (discInfo[connIndex].svcStartHdl != 0)
        {
          attReadByTypeReq_t req;

          // Discover characteristic
          discInfo[connIndex].discState = BLE_DISC_STATE_CHAR;
          req.startHandle = discInfo[connIndex].svcStartHdl;
          req.endHandle = discInfo[connIndex].svcEndHdl;
          req.type.len = ATT_BT_UUID_SIZE;
          req.type.uuid[0] = LO_UINT16(SIMPLEPROFILE_CHAR1_UUID);
          req.type.uuid[1] = HI_UINT16(SIMPLEPROFILE_CHAR1_UUID);

          // Send characteristic discovery request
          VOID GATT_ReadUsingCharUUID(pMsg->connHandle, &req, selfEntity);
        }
      }
    }
    // If we're discovering characteristics
    else if (discInfo[connIndex].discState == BLE_DISC_STATE_CHAR)
    {
      // Characteristic found
      if ((pMsg->method == ATT_READ_BY_TYPE_RSP) &&
          (pMsg->msg.readByTypeRsp.numPairs > 0))
      {
        // Store handle
        discInfo[connIndex].charHdl = BUILD_UINT16(pMsg->msg.readByTypeRsp.pDataList[0],
                                                   pMsg->msg.readByTypeRsp.pDataList[1]);

        Display_print0(dispHandle, MR_ROW_STATUS1, 0, "Simple Svc Found");
      }
    }
  }
}

/*********************************************************************
* @fn      multi_role_mapConnHandleToIndex
*
* @brief   Translates connection handle to index
*
* @param   connHandle - the connection handle
*
* @return  index or INVALID_CONNHANDLE if connHandle isn't found
*/
static uint16_t multi_role_mapConnHandleToIndex(uint16_t connHandle)
{
  uint16_t index;
  // Loop through connection
  for (index = 0; index < maxNumBleConns; index ++)
  {
    // If matching connection handle found
    if (connHandleMap[index].connHandle == connHandle)
    {
      return index;
    }
  }
  // Not found if we got here
  return INVALID_CONNHANDLE;
}

/************************************************************************
* @fn      multi_role_pairStateCB
*
* @param   connHandle - the connection handle
*
* @param   state - pairing state
*
* @param   status - status of pairing state
*
* @return  none
*/
static void multi_role_pairStateCB(uint16_t connHandle, uint8_t state,
                                   uint8_t status)
{
  gapPairStateEvent_t *pData;

  // Allocate space for the passcode event.
  if ((pData = ICall_malloc(sizeof(gapPairStateEvent_t))))
  {
    pData->connectionHandle = connHandle;
    pData->state = state;
    pData->status = status;

    // Enqueue the event.
    multi_role_enqueueMsg(MR_PAIRING_STATE_EVT, (uint8_t *) pData);
  }
}

/*********************************************************************
* @fn      multi_role_passcodeCB
*
* @brief   Passcode callback.
*
* @param   deviceAddr - pointer to device address
*
* @param   connHandle - the connection handle
*
* @param   uiInputs - pairing User Interface Inputs
*
* @param   uiOutputs - pairing User Interface Outputs
*
* @param   numComparison - numeric Comparison 20 bits
*
* @return  none
*/
static void multi_role_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                  uint8_t uiInputs, uint8_t uiOutputs, uint32_t numComparison)
{
  gapPasskeyNeededEvent_t *pData;

  // Allocate space for the passcode event.
  if ((pData = ICall_malloc(sizeof(gapPasskeyNeededEvent_t))))
  {
    memcpy(pData->deviceAddr, deviceAddr, B_ADDR_LEN);
    pData->connectionHandle = connHandle;
    pData->uiInputs = uiInputs;
    pData->uiOutputs = uiOutputs;
    pData->numComparison = numComparison;

    // Enqueue the event.
    multi_role_enqueueMsg(MR_PASSCODE_NEEDED_EVT, (uint8_t *) pData);
  }
}

/*********************************************************************
* @fn      multi_role_processPairState
*
* @brief   Process the new paring state.
*
* @param   pairingEvent - pairing event received from the stack
*
* @return  none
*/
static void multi_role_processPairState(gapPairStateEvent_t* pairingEvent)
{
  // If we've started pairing
  if (pairingEvent->state == GAPBOND_PAIRING_STATE_STARTED)
  {
    Display_print1(dispHandle, MR_ROW_SECURITY, 0,"connHandle %d pairing", pairingEvent->connectionHandle);
  }
  // If pairing is finished
  else if (pairingEvent->state == GAPBOND_PAIRING_STATE_COMPLETE)
  {
    if (pairingEvent->status == SUCCESS)
    {
      Display_print1(dispHandle, MR_ROW_SECURITY, 0,"connHandle %d paired", pairingEvent->connectionHandle);
    }
    else
    {
      Display_print2(dispHandle, MR_ROW_SECURITY, 0, "pairing failed: %d", pairingEvent->connectionHandle, pairingEvent->status);
    }
  }
  // If a bond has happened
  else if (pairingEvent->state == GAPBOND_PAIRING_STATE_BONDED)
  {
    if (pairingEvent->status == SUCCESS)
    {
      Display_print1(dispHandle, MR_ROW_SECURITY, 0, "Cxn %d bonding success", pairingEvent->connectionHandle);
    }
  }
  // If a bond has been saved
  else if (pairingEvent->state == GAPBOND_PAIRING_STATE_BOND_SAVED)
  {
    if (pairingEvent->status == SUCCESS)
    {
      Display_print1(dispHandle, MR_ROW_SECURITY, 0, "Cxn %d bond save success", pairingEvent->connectionHandle);
    }
    else
    {
      Display_print2(dispHandle, MR_ROW_SECURITY, 0, "Cxn %d bond save failed: %d", pairingEvent->connectionHandle, pairingEvent->status);
    }
  }
}

/*********************************************************************
* @fn      multi_role_processPasscode
*
* @brief   Process the Passcode request.
*
* @return  none
*/
static void multi_role_processPasscode(gapPasskeyNeededEvent_t *pData)
{
  // Use static passcode
  uint32_t passcode = 123456;
  Display_print1(dispHandle, MR_ROW_SECURITY, 0, "Passcode: %d", passcode);
  // Send passcode to GAPBondMgr
  GAPBondMgr_PasscodeRsp(pData->connectionHandle, SUCCESS, passcode);
}

/*********************************************************************
 * @fn      multi_role_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 */
static void multi_role_clockHandler(UArg arg)
{
  // Wake up the application.
  Event_post(syncEvent, arg);
  //Event_post(syncEvent, SBP_PERIODIC_EVT);
  
}

/*********************************************************************
 * @brief   Perform a periodic application task to demonstrate notification
 *          capabilities of simpleGATTProfile. This function gets called
 *          every five seconds (MR_PERIODIC_EVT_PERIOD). In this example,
 *          the value of the third characteristic in the SimpleGATTProfile
 *          service is retrieved from the profile, and then copied into the
 *          value of the the fourth characteristic.
 *
 */
static void multi_role_performPeriodicTask(void)
{
  uint8_t valueToCopy;

  // Call to retrieve the value of the third characteristic in the profile
  if (SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR3, &valueToCopy) == SUCCESS)
  {
    // Call to set that value of the fourth characteristic in the profile.
    // Note that if notifications of the fourth characteristic have been
    // enabled by a GATT client device, then a notification will be sent
    // every time this function is called. Also note that this will 
    // send a notification to each connected device.
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint8_t),
                               &valueToCopy);
  }
}

/*********************************************************************
* @fn      multi_role_addMappingEntry
*
* @brief   add a new connection to the index-to-connHandle map
*
* @param   connHandle - the connection handle
*
* @param   addr - pointer to device address
*
* @return  index of connection handle
*/
static uint8_t multi_role_addMappingEntry(uint16_t connHandle, uint8_t *addr)
{
  uint16_t index;
  // Loop though connections
  for (index = 0; index < maxNumBleConns; index++)
  {
    // If there is an open connection
    if (connHandleMap[index].connHandle == INVALID_CONNHANDLE)
    {
      // Store mapping
      connHandleMap[index].connHandle = connHandle;

      // Convert address to string
      uint8_t *pAddr = (uint8_t *) Util_convertBdAddr2Str(addr);

      // Copy converted string to persistent connection handle list
      memcpy(connHandleMap[index].strAddr, pAddr, B_STR_ADDR_LEN);
      
      // Enable items in submenus
      tbm_setItemStatus(&mrMenuGattRw, (1 << index), TBM_ITEM_NONE);
      tbm_setItemStatus(&mrMenuConnUpdate, (1 << index), TBM_ITEM_NONE);
      tbm_setItemStatus(&mrMenuDisconnect, (1 << index), TBM_ITEM_NONE);
     
      // Add device address as a string to action description of connectable menus
      TBM_SET_ACTION_DESC(&mrMenuGattRw, index, connHandleMap[index].strAddr);
      TBM_SET_ACTION_DESC(&mrMenuConnUpdate, index, connHandleMap[index].strAddr);
      TBM_SET_ACTION_DESC(&mrMenuDisconnect, index, connHandleMap[index].strAddr);

      // Enable connectable menus if they are disabled
      if (!(TBM_IS_ITEM_ACTIVE(&mrMenuMain, TBM_ITEM_2)))
      {
        tbm_setItemStatus(&mrMenuMain, TBM_ITEM_2 | TBM_ITEM_3 | TBM_ITEM_4,
                          TBM_ITEM_NONE);
      }

      return index;
    }
  }
  // No room if we get here
  return bleNoResources;
}

/*********************************************************************
* @fn      mr_doScan
*
* @brief   Respond to user input to start scanning
*
* @param   index - not used
*
* @return  TRUE since there is no callback to use this value
*/
bool mr_doScan(uint8_t index)
{
  (void) index;

  // If we can connect to another device
  if (linkDB_NumActive() < maxNumBleConns)
  {
    // If we're not already scanning
    if (!scanningStarted)
    {
      // Set scannin started flag
      scanningStarted = TRUE;

      // Start scanning
      GAPRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
                             DEFAULT_DISCOVERY_ACTIVE_SCAN, DEFAULT_DISCOVERY_WHITE_LIST);

      Display_print0(dispHandle, MR_ROW_STATUS1, 0, "Discovering...");
    }
    // We're already scanning...so cancel
    else
    {
      // Cancel scanning
      GAPRole_CancelDiscovery();
      Display_print0(dispHandle, MR_ROW_STATUS1, 0, "Discovery Cancelled");

      // Clear scanning started flag
      scanningStarted = FALSE;
    }
  }
  return TRUE;
}

/*********************************************************************
* @fn      mr_doConnect
*
* @brief   Respond to user input to form a connection
*
* @param   index - index as selected from the mrMenuConnect
*
* @return  TRUE since there is no callback to use this value
*/
bool mr_doConnect(uint8_t index)
{
  // If already connecting...cancel
  if (connecting == TRUE)
  {
    // Cancel connection request
    GAPRole_TerminateConnection(GAP_CONNHANDLE_INIT);
    Display_print0(dispHandle, MR_ROW_STATUS1, 0, "Connecting Cancelled");

    // Clear connecting flag
    connecting = FALSE;
  }
  // If attempting to connect
  else
  {
    // Connect to current device in scan result
    GAPRole_EstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,
                          DEFAULT_LINK_WHITE_LIST,
                          devList[index].addrType, devList[index].addr);

    // Set connecting state flag
    connecting = TRUE;
    Display_print0(dispHandle, MR_ROW_STATUS1, 0, "Connecting to:");
    Display_print0(dispHandle, MR_ROW_STATUS2, 0, (char*)devList[index].strAddr);
  }

  return TRUE;
}

/*********************************************************************
* @fn      mr_doGattRw
*
* @brief   Respond to user input to do a GATT read or write
*
* @param   index - index as selected from the mrMenuGattRw
*
* @return  TRUE since there is no callback to use this value
*/
bool mr_doGattRw(uint8_t index)
{
  bStatus_t status = FAILURE;
  // If characteristic has been discovered
  if (discInfo[index].charHdl != 0)
  {
    // Do a read / write as long as no other read or write is in progress
    if (doWrite)
    {
      // Do a write
      attWriteReq_t req;

      // Allocate GATT write request
      req.pValue = GATT_bm_alloc(connHandleMap[index].connHandle, ATT_WRITE_REQ, 1, NULL);
      // If successfully allocated
      if (req.pValue != NULL)
      {
        // Fill up request
        req.handle = discInfo[index].charHdl;
        req.len = 1;
        req.pValue[0] = charVal;
        req.sig = 0;
        req.cmd = 0;

        // Send GATT write to controller
        status = GATT_WriteCharValue(connHandleMap[index].connHandle, &req, selfEntity);

        // If not sucessfully sent
        if ( status != SUCCESS )
        {
          // Free write request as the controller will not
          GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ);
        }
      }
    }
    // Do a read
    else
    {
      // Create read request...place in CSTACK
      attReadReq_t req;

      // Fill up read request
      req.handle = discInfo[index].charHdl;

      // Send read request. no need to free if unsuccessful since the request
      // is only placed in CSTACK; not allocated
      status = GATT_ReadCharValue(connHandleMap[index].connHandle, &req, selfEntity);
    }

    // If succesfully queued in controller
    if (status == SUCCESS)
    {
      // Toggle read / write
      doWrite = !doWrite;
    }
  }

  return TRUE;
}

/*********************************************************************
* @fn      mr_doConnUpdate
*
* @brief   Respond to user input to do a connection update
*
* @param   index - index as selected from the mrMenuConnUpdate
*
* @return  TRUE since there is no callback to use this value
*/
bool mr_doConnUpdate(uint8_t index)
{
  bStatus_t status = FAILURE;
  // Fill in connection handle in dummy params
  updateParams.connHandle = connHandleMap[index].connHandle;

  // Send connection parameter update
  status = gapRole_connUpdate( GAPROLE_NO_ACTION, &updateParams);

  // If successfully sent to controller
  if (status == SUCCESS)
  {
    Display_print0(dispHandle, MR_ROW_STATUS1, 0, "Updating");
  }
  // If there is already an ongoing update
  else if (status == blePending)
  {
    Display_print0(dispHandle, MR_ROW_STATUS1, 0, "Already Updating");
  }

  return TRUE;
}

/*********************************************************************
* @fn      mr_doDisconnect
*
* @brief   Respond to user input to terminate a connection
*
* @param   index - index as selected from the mrMenuConnUpdate
*
* @return  TRUE since there is no callback to use this value
*/
bool mr_doDisconnect(uint8_t index)
{
  // Disconnect
  GAPRole_TerminateConnection(connHandleMap[index].connHandle);
  Display_print0(dispHandle, MR_ROW_STATUS1, 0, "Disconnecting");

  return TRUE;
}

/* Actions for Menu: Init - Advertise */
bool mr_doAdvertise(uint8_t index)
{
  (void) index;
  uint8_t adv;
  uint8_t adv_status;

  // Get current advertising status
  GAPRole_GetParameter(GAPROLE_ADVERT_ENABLED, &adv_status, NULL);

  // If we're currently advertising
  if (adv_status)
  {
    // Turn off advertising
    adv = FALSE;
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &adv, NULL);
  }
  // If we're not currently advertising
  else
  {
    // Turn on advertising
    adv = TRUE;
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &adv, NULL);
  }

  return TRUE;
}

/*********************************************************************
 * @fn      SimpleBLECentral_addDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void multi_role_addDeviceInfo(uint8_t *pAddr, uint8_t addrType)
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

/*********************************************************************/
/*
*    @fn        Read Macaddress
*
*    @brief     Type Transformation
*/
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

//****************************************************************************** 
char *Addr2Str(uint8 *Addr)  
{  
  uint8         i;  
  char          Hex[] = "0123456789ABCDEF";  
  static char   Mac_Str[B_ADDR_LEN*2];  
  char          *MacStr = Mac_Str;  
  
  for(i = B_ADDR_LEN; i > 0; i--)  
  {  
    *MacStr++ = Hex[*Addr >> 4];  
    *MacStr++ = Hex[*Addr++ & 0x0F];  
  }  
  
  return Mac_Str;  
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

/********************************
*@fn NBIOT
*
*
*********************************/

static void NBIoT_Init()
{
  /*****************************/
  //NBIoT 
  if(flag.CFUN_OK_Flag == 0)
  {
   UART_read(UART_handle,Read_Uart,sizeof(Read_Uart));
   UART_write(UART_handle,CFUN_Open,sizeof(CFUN_Open)); 
   Semaphore_pend(uartSemHandle, 600000); //6S
   flag.CFUN_OK_Flag = 1;
   //memset(Read_Uart , 0 , sizeof(Read_Uart));
  }
  
  else if(flag.IMEI_Ok_Flag == 0)
  {
    UART_read(UART_handle,Read_Uart, sizeof(Read_Uart));
    UART_write(UART_handle,Write_IMEI,sizeof(Write_IMEI));
    Semaphore_pend(uartSemHandle, UartTimeout);
    
    for(int x=0;x<15;x++)
    {
      IMEI[x]=Read_Uart[x+8];
    }  
      flag.IMEI_Ok_Flag =1 ;
    //memset(Read_Uart , 0 , sizeof(Read_Uart));
  }
  
  else if(flag.IMSI_Ok_Flag == 0)
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
      flag.IMSI_Ok_Flag =1 ;
      //memset(Read_Uart , 0 , sizeof(Read_Uart));
    }  
  }

  else if(flag.APN_Ok_Flag == 0)
  {
    
    if(IMSI[2] == '0' && IMSI[3] == '1' && IMSI[4] == '1')
    { 
      UART_write(UART_handle, CGDCONT_CT,sizeof(CGDCONT_CT)); //APN : ctnb
      Task_sleep(100000);
      UART_write(UART_handle,Cops_CT,sizeof(Cops_CT));        //Oper : 46011   
      
      flag.APN_Ok_Flag =1;  
      //memset(Read_Uart , 0 , sizeof(Read_Uart));
    }    
    
    else if(IMSI[2] == '6' && IMSI[3] == '0' && IMSI[4] == '1')
    {
      UART_write(UART_handle, CGDCONT_FET,sizeof(CGDCONT_FET)); //APN : nbiot
      Task_sleep(100000);
      UART_write(UART_handle,Cops_FET,sizeof(Cops_FET));        //Oper : 46601
      
      flag.APN_Ok_Flag =1;  
      //memset(Read_Uart , 0 , sizeof(Read_Uart));
    }    
    
    else if(IMSI[2] == '6' && IMSI[3] == '9' && IMSI[4] == '2')
    {
      UART_write(UART_handle, CGDCONT_CHT,sizeof(CGDCONT_CHT)); //APN : internet.iot
      Task_sleep(100000);
      UART_write(UART_handle,Cops_CHT,sizeof(Cops_CHT));        //Oper : 46692
    
      flag.APN_Ok_Flag =1;
      //memset(Read_Uart , 0 , sizeof(Read_Uart));
    }
    
//    else if(IMSI[2] == '6' && IMSI[3] == '9' && IMSI[4] == '7')
//    {
//      UART_write(UART_handle, CGDCONT_TM,sizeof(CGDCONT_TM)); //APN : twm.nbiot
//      Task_sleep(100000);
//      UART_write(UART_handle,Cops_TM,sizeof(Cops_TM));        //Oper : 46697
//      
//      flag.APN_Ok_Flag =1;     
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
  
   else if(flag.CSQ_Ok_Flag == 0 )
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
      flag.CSQ_Ok_Flag =1; 
      //memset(Read_Uart , 0 , sizeof(Read_Uart));    
    } 
    
  }
  else if(flag.GATT_Ok_Flag == 0)
  {
    UART_read(UART_handle,Read_Uart, sizeof(Read_Uart));
    UART_write(UART_handle,Write_CGATT,sizeof(Write_CGATT)); 
    Semaphore_pend(uartSemHandle, UartTimeout); 
    
    if(Read_Uart[9] == '1' )
    {
      flag.GATT_Ok_Flag = 1;
      GATT_Error   = 0; 
      //memset(Read_Uart , 0 , sizeof(Read_Uart));
    }  

    else if(Read_Uart[9] == '0')
    {
      GATT_Error ++;
    }

  }
   
  else if(flag.REG_Ok_Flag == 0)
  {
    UART_read(UART_handle,Read_Uart, sizeof(Read_Uart));
    UART_write(UART_handle,Write_REG,sizeof(Write_REG)); 
    Semaphore_pend(uartSemHandle, UartTimeout); 
    
    REG=Read_Uart[11];
    
    if(REG == '1')
    {
      flag.REG_Ok_Flag = 1;
      //memset(Read_Uart , 0 , sizeof(Read_Uart));
    }
    
  }
  
  
  else if(flag.NBIot_Init_Ok ==0 )
  {
    UART_write(UART_handle, NSOCR,sizeof(NSOCR)); //UDP channel 1
    Task_sleep(10000);
    UART_write(UART_handle, NSMI,sizeof(NSMI));
    Task_sleep(10000);
//    UART_write(UART_handle, PSM_Mode_Config,sizeof(PSM_Mode_Config));    
//    Task_sleep(10000);    
//    UART_write(UART_handle, PSM_Mode_Open,sizeof(PSM_Mode_Open));    
//    Task_sleep(10000);        
//    UART_write(UART_handle, IDle_Mode_Open,sizeof(IDle_Mode_Open));    
//    Task_sleep(10000);          
    UART_read(UART_handle,Read_Uart, sizeof(Read_Uart));
    Semaphore_pend(uartSemHandle, UartTimeout);
    
    flag.NBIot_Init_Ok = 1;
  }
   //memset(Read_Uart , 0 , sizeof(Read_Uart));
  }


static void NBIoT_Data()
{
  //uint8_t Count=13;
  
  NSOST_UDP1[0] ='A';
  NSOST_UDP1[1] ='T';
  NSOST_UDP1[2] ='+';
  NSOST_UDP1[3] ='N';
  NSOST_UDP1[4] ='S';
  NSOST_UDP1[5] ='O';
  NSOST_UDP1[6] ='S';
  NSOST_UDP1[7] ='T';
  NSOST_UDP1[8] ='=';
  NSOST_UDP1[9] ='1';
  NSOST_UDP1[10]=',';
  NSOST_UDP1[11]='0';
  NSOST_UDP1[12]='x';
  NSOST_UDP1[13]=NB_Config[3]>>4;
  NSOST_UDP1[14]=NB_Config[3] & 0X0F;
  NSOST_UDP1[15]='.';
  NSOST_UDP1[16]='0';
  NSOST_UDP1[17]='x';
  NSOST_UDP1[18]=NB_Config[4]>>4;
  NSOST_UDP1[19]=NB_Config[4] & 0X0F;
  NSOST_UDP1[20]='.';
  NSOST_UDP1[21]='0';
  NSOST_UDP1[22]='x';
  NSOST_UDP1[23]=NB_Config[5]>>4;
  NSOST_UDP1[24]=NB_Config[5] & 0X0F;
  NSOST_UDP1[25]='.';
  NSOST_UDP1[26]='0';
  NSOST_UDP1[27]='x';
  NSOST_UDP1[28]=NB_Config[6]>>4;
  NSOST_UDP1[29]=NB_Config[6] & 0X0F;
  NSOST_UDP1[30]=',';
  NSOST_UDP1[31]=((((NB_Config[7]<<8) | NB_Config[8])%100000)/10000)  +0x30;
  NSOST_UDP1[32]=((((NB_Config[7]<<8) | NB_Config[8])%10000)/1000)+0x30;
  NSOST_UDP1[33]=((((NB_Config[7]<<8) | NB_Config[8])%1000)/100)+0x30;
  NSOST_UDP1[34]=((((NB_Config[7]<<8) | NB_Config[8])%100)/10)+0x30;
  NSOST_UDP1[35]=((((NB_Config[7]<<8) | NB_Config[8])%10)/1)+0x30;
  NSOST_UDP1[36]=',';
  NSOST_UDP1[37]='2';
  NSOST_UDP1[38]='1';
  NSOST_UDP1[39]=',';
  
  gatewaydata[0] = License_State;//內外結果
  memcpy(&gatewaydata[1], &ownAddress[0], 6);
  gatewaydata[7] = bicycle_number;

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
    
    NBIOT_Data1[82]=0X0D;
    NBIOT_Data1[83]=0X0A;
  }
}

/*************************************************************************
*  Slave Safe Notify
*
**************************************************************************/
static void NBIoT_Safe()
{
  //uint8_t Count=13;
  
  NSOST_UDP2[0] ='A';
  NSOST_UDP2[1] ='T';
  NSOST_UDP2[2] ='+';
  NSOST_UDP2[3] ='N';
  NSOST_UDP2[4] ='S';
  NSOST_UDP2[5] ='O';
  NSOST_UDP2[6] ='S';
  NSOST_UDP2[7] ='T';
  NSOST_UDP2[8] ='=';
  NSOST_UDP2[9] ='1';
  NSOST_UDP2[10]=',';
  NSOST_UDP2[11]='0';
  NSOST_UDP2[12]='x';
  NSOST_UDP2[13]=NB_Config[3]>>4;
  NSOST_UDP2[14]=NB_Config[3] & 0X0F;
  NSOST_UDP2[15]='.';
  NSOST_UDP2[16]='0';
  NSOST_UDP2[17]='x';
  NSOST_UDP2[18]=NB_Config[4]>>4;
  NSOST_UDP2[19]=NB_Config[4] & 0X0F;
  NSOST_UDP2[20]='.';
  NSOST_UDP2[21]='0';
  NSOST_UDP2[22]='x';
  NSOST_UDP2[23]=NB_Config[5]>>4;
  NSOST_UDP2[24]=NB_Config[5] & 0X0F;
  NSOST_UDP2[25]='.';
  NSOST_UDP2[26]='0';
  NSOST_UDP2[27]='x';
  NSOST_UDP2[28]=NB_Config[6]>>4;
  NSOST_UDP2[29]=NB_Config[6] & 0X0F;
  NSOST_UDP2[30]=',';
  NSOST_UDP2[31]=((((NB_Config[7]<<8) | NB_Config[8])%100000)/10000)  +0x30;
  NSOST_UDP2[32]=((((NB_Config[7]<<8) | NB_Config[8])%10000)/1000)+0x30;
  NSOST_UDP2[33]=((((NB_Config[7]<<8) | NB_Config[8])%1000)/100)+0x30;
  NSOST_UDP2[34]=((((NB_Config[7]<<8) | NB_Config[8])%100)/10)+0x30;
  NSOST_UDP2[35]=((((NB_Config[7]<<8) | NB_Config[8])%10)/1)+0x30;
  NSOST_UDP2[36]=',';
  NSOST_UDP2[37]='2';
  NSOST_UDP2[38]='5';
  NSOST_UDP2[39]=',';
  
  //gatewaydata[0] = License_State;//內外結果 
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
    
    NBIOT_Data2[90]=0X0D;
    NBIOT_Data2[91]=0X0A;
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
static void Position_judge()
{
  float D     =0;
  //float D_sum =0;

//  int sigma = 4;
//  const float Base =0.030223879;
  float Alpha_Y[200]={0,0.2822,0.6553,2.2585,2.7771,3,0.7886,0.2715,3,3,0.8374,0.034,0,0.9453,1.0676,0.2791,1.0365,0.6944,1.5401,2.3072,1.2942,0,1.0907,0,0.3678,0.5093,0,0.4481,0,0.2618,1.2173,3,1.6961,1.3476,2.8382,1.003,0.605,0,0.4008,0.471,0.5295,0.0896,0.7523,0.5598,0.7955,0.3416,0,0,0.4823,0.7993,0.7862,0.5729,0.7427,0.9305,0.4164,0.8203,0.7397,0,0.8622,0.9313,0.7862,1.2751,0.1532,0,1.0738,0.5128,0.6214,1.108,0.4181,0,0.1189,0.4821,0.7006,0.1519,0.1133,0.3554,1.5704,0.104,0.7847,0.8276,0.553,0.4371,0.6896,0.3882,0.3975,0.6397,1.6863,0.1436,0.6064,0,1.1972,0,0.1999,0.734,0.1057,0.164,0.6726,0.508,0.3482,0.8731,-0.7077,-0.5748,-0.5315,-0.3215,-0.4703,-1.0137,-0.7728,0,0,-1.1725,-0.4352,-0.9569,-0.4663,-2.3624,-0.2866,-0.3841,-0.4874,-2.7173,-1.1107,-0.5996,-0.7791,0,-0.0247,0,-0.7715,-0.0944,-0.2614,0,0,-0.0383,-0.0255,-0.3509,-0.7731,-0.7271,-0.3064,-0.9414,-0.5174,-0.0683,-0.8068,-1.1074,-0.195,-0.2923,-0.8853,-0.703,-0.7973,-0.3342,-0.8232,-0.7547,-0.9549,-0.9983,-0.8457,-0.1587,0,-1.7926,-0.5639,-0.0967,-0.5565,0,-3,0,-0.3428,-1.4312,0,-0.3347,0,-0.0304,0,-0.1297,-2.2855,-1.2393,-0.0641,-0.5628,-1.0517,0,0,-0.5165,-0.3425,-0.5723,-0.5619,-0.5096,-1.3977,-3,-2.8868,-1.9301,-0.7242,-1.7591,-3,-0.4036,-0.7207,-1.4372,-0.2782,-0.7087,-0.843,-3,-0.4871,-1.3174,-1.1763,0,-0.3777,-1.839};

  int Trainingdata1[200]={87,83,82,84,81,85,83,83,92,93,88,88,89,86,87,84,81,89,83,87,90,88,78,77,80,76,87,89,80,88,78,77,76,76,88,76,88,76,77,83,72,68,69,68,69,68,71,68,68,70,52,59,62,54,62,51,62,62,61,65,74,76,73,69,68,65,69,64,71,70,83,70,69,85,73,69,81,70,86,71,89,78,77,74,69,81,90,81,77,86,87,85,85,86,85,83,83,85,84,91,93,95,85,92,88,93,86,86,93,79,78,77,81,77,81,72,78,72,70,70,76,88,83,83,83,84,77,89,88,82,80,85,82,82,74,82,80,83,82,82,80,88,73,77,80,71,73,84,83,76,75,77,75,88,87,86,86,86,78,87,84,82,87,88,88,85,86,87,84,90,92,93,91,88,88,91,89,91,94,92,82,83,91,83,90,93,84,89,85,87,92,94,97,92,93,90,98,92,93,90};
  int Trainingdata2[200]={92,92,91,88,92,94,92,91,91,92,93,94,91,93,90,91,92,90,94,94,88,87,85,86,86,84,88,88,88,86,89,91,87,90,87,89,86,85,91,88,83,80,97,84,81,92,84,85,91,94,78,77,80,77,80,83,90,95,96,92,74,82,79,83,81,84,85,84,87,81,69,68,71,70,68,73,65,70,70,66,67,59,61,71,66,67,67,62,68,68,58,57,48,58,50,57,48,57,56,46,86,86,84,86,85,93,91,84,91,87,83,82,92,91,87,83,89,86,93,78,73,77,77,79,75,83,78,78,78,80,81,77,75,92,77,76,88,77,87,88,79,81,79,79,83,78,79,78,70,67,89,79,81,85,83,77,84,85,93,84,73,61,71,68,72,69,72,65,68,65,77,82,65,77,73,75,76,72,79,73,87,93,88,96,88,92,91,98,97,97,94,87,93,94,94,87,92,91,90,94};
  int Trainingdata3[200]={89,87,78,90,91,80,91,82,82,84,86,86,82,85,93,81,77,79,84,88,76,74,75,71,72,71,76,70,76,70,79,78,80,89,81,76,75,72,73,77,70,70,61,72,61,68,71,71,63,68,67,59,64,67,59,61,72,66,66,55,65,71,70,71,76,77,69,81,71,68,69,78,75,69,75,70,78,71,74,68,72,77,75,69,75,69,77,73,80,71,88,82,79,81,81,82,82,88,90,77,84,92,94,91,87,80,94,92,91,91,86,76,83,74,86,81,70,78,86,83,90,93,90,93,95,92,86,88,87,85,80,80,84,83,82,76,85,84,78,83,83,93,77,86,93,78,77,87,82,91,83,82,83,81,84,87,83,83,80,83,87,87,87,87,85,86,85,85,79,80,90,89,85,82,92,80,94,92,82,85,80,82,82,92,93,86,90,87,83,83,86,88,87,82,87,81,81,89,90,90};
  int Trainingdata4[200]={56,51,51,65,66,66,48,50,69,65,58,57,56,69,58,56,66,57,70,71,68,74,66,75,78,70,78,75,77,74,77,86,85,81,82,84,77,74,75,83,76,85,78,86,88,78,77,78,78,85,83,84,80,93,86,84,83,86,87,85,79,88,88,81,81,79,91,90,80,88,87,94,86,86,94,84,91,87,88,94,86,91,84,83,95,83,94,90,90,87,89,85,85,85,86,84,84,81,84,86,79,73,81,82,78,80,82,82,75,73,93,93,88,88,91,90,88,84,85,87,86,87,87,86,83,84,87,87,87,90,91,88,78,81,83,80,72,85,90,75,68,87,66,66,90,67,86,68,67,77,95,85,87,86,94,89,85,92,84,85,87,91,87,87,86,89,87,90,93,87,87,89,97,88,88,89,88,88,91,85,64,67,67,66,69,68,64,68,64,67,70,65,89,66,82,75,71,80,86,73};

  float k_num[200] = {0};

  for(int x=0; x<200; x++)
  {

    k_num[x] = ((Trainingdata1[x]-Testdata1[0])*(Trainingdata1[x]-Testdata1[0])+(Trainingdata2[x]-Testdata1[1])*(Trainingdata2[x]-Testdata1[1])+(Trainingdata3[x]-Testdata1[2])*(Trainingdata3[x]-Testdata1[2])+(Trainingdata4[x]-Testdata1[3])*(Trainingdata4[x]-Testdata1[3]));
    k_num[x] = ((-1*k_num[x])/(2*sigma*sigma));
    k_num[x] = exp(k_num[x]);
  }
  
  for(int x=0; x<200; x++)
  {
    D=D+(Alpha_Y[x]*k_num[x]);
  }
  
  D_sum=D+Base;
  
  if(D_sum > 0)
  {
    License_State = Inside_State;
    bicycle_number = bicycle_number + 1;
    Position_flag = 1;
  }
  else if(D_sum < 0)
  {
    License_State = Outside_State;
    bicycle_number = bicycle_number - 1;
    Position_flag = 1;
  }
}
/*********************************************************************
*********************************************************************/
