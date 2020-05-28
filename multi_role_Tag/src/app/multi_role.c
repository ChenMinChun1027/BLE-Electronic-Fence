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

#include <ti/drivers/ADCBuf.h>
#include <driverlib/aon_batmon.h>
#include <driverlib/aux_wuc.h>

#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
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

uint8_t advclose = false;
#endif //I2c_Acceleration
uint8_t advopen  = true;
int led_count=0;
int adv_count=0;
int RSSI_count = 0;
int RLED=0;
uint8 b[10]={0};

#include "shutdn_button.h"
#define KEY_BTN0              0x0001
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
#define SBP_LED_EVT                          Event_Id_08
#define SBP_MOVE_EVT                         Event_Id_09
#define SBC_SCAN_EVT                         Event_Id_10
#define SBC_SCAN_STARTED_EVT                 Event_Id_11
#define SBP_BOARDCAST_EVT                    Event_Id_12
#define reset_EVT                            Event_Id_13
#define Tagsafe_event                        Event_Id_14

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
                                             SBP_LED_EVT             | \
                                             SBP_MOVE_EVT            | \
                                             SBC_SCAN_EVT            | \
                                             SBC_SCAN_STARTED_EVT    | \
                                             SBP_BOARDCAST_EVT       | \
                                             Tagsafe_event           | \
                                             reset_EVT  )

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

/*********************************************************************
* LOCAL VARIABLES
*/

#define SCANNING_STARTED 1
#define SCANNING_STOPPED 0

#define SCAN_BNODE_RECORD_SIZE 4
#define SCAN_SIZE              100

static uint8_t scanning_status = SCANNING_STARTED;
static uint16_t scan_num = 0;

typedef struct
{
  uint16_t  count;
  int16_t   totalRSSI;
  int8_t    averageRSSI;
  uint8_t   MacAddr[B_ADDR_LEN];
  int8      scanrssi[50];
}bNode_data_t;

static bNode_data_t scan_bNode_record[SCAN_BNODE_RECORD_SIZE] = {0};

static int8  dev_rssi     = 0;
static uint8_t scan_Mac[B_ADDR_LEN] = {0};

static int8 rssidata[4];
//static uint8_t writedata[2] = {0x00};
//static int flashdata = 0;
static uint8_t routedata[12] = {0x00};
static uint8_t newValue3[2] = {0x00};
static uint8_t position_data[10] = {0x00}; 

static uint8_t Lock_state = 0x00;
enum{
  Locked,
  Unlock,
};

/*********************************Batt*********************************************/
uint8_t batVol_H;
uint8_t batVol_L;
uint32_t batVol;
float batVol_LL;
float batVol_HH;
float Batt;
int BattA;
int BattD;

static uint8_t ownAddress[B_ADDR_LEN];

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Clock instances for internal periodic events.
static Clock_Struct periodicClock;
static Clock_Struct accelerClock;
static Clock_Struct ledperiodicClock;
static Clock_Struct moveClock;
//static Clock_Struct boardcastClock;
static Clock_Struct scanClock;
static Clock_Struct resetClock;
static Clock_Struct safeClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task configuration
Task_Struct mrTask;
Char mrTaskStack[MR_TASK_STACK_SIZE];

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

//static I2C_Handle I2CHandle;
//static I2C_Params I2CParams;
//Semaphore_Struct mutex;
//
//#define TXBUF_LEN 1
//#define RXBUF_LEN 1
//static uint8_t txBuf=0x0F;
//static uint8_t rxBuf=0x00;
//static uint8_t volatile tx = 0;
//static uint8_t volatile rx = 0;

//static PIN_State sbpPins;
//static PIN_Handle hSbpPins;

static PIN_Handle gpioPinHandle;
static PIN_State gpioPinState;
static uint8_t i=0;

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

#if 0
static bool spi_open(uint32_t bitRate) {
	SPI_Params spiParams;
    /*  Configure SPI as master */
    SPI_Params_init(&spiParams);//!< ??初始化
    spiParams.bitRate = bitRate;
    spiParams.mode = SPI_MASTER;
    spiParams.transferMode = SPI_MODE_BLOCKING;

    /* Attempt to open SPI. */
    SbpSpiHandle = SPI_open(Board_SPI0, &SbpSpiParams);//!< 打?Board_SPI0

    return SbpSpiHandle != NULL;
}

static bool spi_write(const uint8_t *buf, size_t len) {
    SPI_Transaction masterTransaction;

    masterTransaction.count  = len;
    masterTransaction.txBuf  = (void*)buf;//!< ?要??的?据存放的地址??*txBuf
    masterTransaction.arg    = NULL;
    masterTransaction.rxBuf  = NULL;

    return SPI_transfer(SbpSpiHandle, &masterTransaction);//!< ?用SPI_transfer()?入?据
}

static bool spi_read(uint8_t *buf, size_t len) {
    SPI_Transaction masterTransaction;

    masterTransaction.count = len;
    masterTransaction.rxBuf = buf;//!< ?要接收到的?据存入buf所指的地址中
    masterTransaction.txBuf = NULL;
    masterTransaction.arg = NULL;

    return SPI_transfer(SbpSpiHandle, &masterTransaction);//!< ?用SPI_transfer()?出?据
}


static bool spi_WriteRead(uint8_t *txBuf, uint8_t wlen, uint8_t *rxBuf, uint8_t rlen)//SPI寫/讀函式
{
  SPI_Transaction masterTransaction;
  bool success;
  masterTransaction.writecount  = wlen; //所要寫的資料長度，位元組單位
  masterTransaction.txBuf       = txBuf;//所要被寫資料的地址
  masterTransaction.readcount   = rlen;
  masterTransaction.rxBuf       = rxBuf;//接收資料所存放的地址
  //success = SPI_transfer(SbpSpiHandle, &masterTransaction);
  return SPI_transfer(SbpSpiHandle, &masterTransaction);
}
#endif

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

static uint8_t advertData[] = {0x0E, 0xFF, 0x24, 0x01, 0x00, 0x00, 0x00, 0x00,
                               0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 
                               0x01, 0x06, 0x03, 0x02, 0xF0, 0xFF}; //advertData[4] = 0x01 ->車牌RSSI

static uint8_t tagsafe_advertdata[] = {0x13, 0xFF, 0x40, 0x03, 0x00, 0x00, 0x00, 
                                       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                                       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 
                                       0x01, 0x06, 0x03, 0x02, 0xF0, 0xFF}; //advertData[4] = 0x03 ->車牌SAFE
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


static void Read_Mac(uint8 *Mac_Address);
char *Addr2Str(uint8 *Addr);
static void Voltage();
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
#ifdef I2c_Acceleration
  bspI2cInit();
#endif //I2c_Acceleration
  
  //flash讀入
  //osal_snv_read(0x81, 2, &writedata);
//  flashdata = writedata[0];
  
  Read_Mac(ownAddress);

  attDeviceName[0] ='E';
  attDeviceName[1] ='L';
  attDeviceName[2] ='P';
  attDeviceName[3] ='-';
  attDeviceName[4] ='B';
  attDeviceName[5] ='4';
  attDeviceName[6] ='0';
  attDeviceName[7] ='-';
  Hex_to_Ascii(ownAddress[4],&attDeviceName[8]);
  Hex_to_Ascii(ownAddress[5],&attDeviceName[10]);
  //Hex_to_Ascii(routedata[1],&attDeviceName[12]);

  
    //SPI_init ();   // Initialize the SPI driver
    //bspSpiOpen();
//  SPI_Params_init (&SbpSpiParams);   // Initialize SPI parameters 
//  //SbpSpiParams.dataSize = 8 ;
//  SbpSpiParams.bitRate = 4000000;
//    SbpSpiParams.mode         = SPI_MASTER;
//    SbpSpiParams.transferMode = SPI_MODE_BLOCKING;
//  SbpSpiHandle = SPI_open(Board_SPI0, &SbpSpiParams);

  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Create one-shot clocks for internal periodic events.  
//  Util_constructClock(&periodicClock, multi_role_clockHandler,
//                      MR_PERIODIC_EVT_PERIOD, 1000, false, MR_PERIODIC_EVT);
  
  Util_constructClock(&resetClock, multi_role_clockHandler,
                      1000, 1000, false, SBP_PERIODIC_EVT);
  
//  Util_constructClock(&resetClock, multi_role_clockHandler,
//                      1000, 1000, false, reset_EVT);
  
//  Util_constructClock(&ledperiodicClock, multi_role_clockHandler,
//                      2000, 2000, true, SBP_LED_EVT);  
  
  Util_constructClock(&moveClock, multi_role_clockHandler,
                      500, 500, false, SBP_MOVE_EVT); 
  
//  Util_constructClock(&boardcastClock, multi_role_clockHandler,
//                      500, 0, false, SBP_BOARDCAST_EVT);
  
  Util_constructClock(&scanClock, multi_role_clockHandler,
                      50, 50, false, SBC_SCAN_EVT);
  
  Util_constructClock(&safeClock, multi_role_clockHandler,
                      1000, 1800000, false, Tagsafe_event);
  
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
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData, NULL);

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
  
  //routedata[0] = 0x40;
  //routedata[1] = writedata[]
  
  //PIN_setOutputValue(hSbpPins, Board_LED_G,1); //設定綠燈亮  
  PIN_setOutputValue(gpioPinHandle,CC2640R2_LAUNCHXL_PIN_GLED , 0);
  PIN_setOutputValue(gpioPinHandle,CC2640R2_LAUNCHXL_PIN_RLED , 1);
  //Util_startClock(&moveClock);
  //Util_startClock(&safeClock);
  
  //Util_startClock(&accelerClock);
  //GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),&advopen,NULL);
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
  
  uint8_t index = 0;

//  GAPRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
//                                DEFAULT_DISCOVERY_ACTIVE_SCAN,
//                                DEFAULT_DISCOVERY_WHITE_LIST);
  
#ifdef I2c_Acceleration
    
    uint8 pBuf[2] = {0x0f,0};
    uint8 rBuf[2] = {0};
    pBuf[0] = LIS2DH_WHO_AM_I ;
    
    bspI2cWriteRead(pBuf,1,rBuf,1);
    
    pBuf[0] = LIS2DH_CTRL_REG1;
    pBuf[1] = 0x67; //0x67
    bspI2cWrite(pBuf,2);
    
    pBuf[0] = LIS2DH_CTRL_REG4;
    pBuf[1] = 0x10;
    bspI2cWrite(pBuf,2);
#endif //I2c_Acceleration
    
    // Fill in transmitBuffer
    //spiTransaction.writecount = 1;
    //spiTransaction.txBuf = txBuf;
    //spiTransaction.rxBuf = rxBuf;
    //rx = SPI_transfer(SbpSpiHandle, &spiTransaction);
    //bspSpiOpen();
  
  // Application main loop
  for (;;)
  {
    uint32_t events;
    
    // Waits for an event to be posted associated with the calling thread.
    // Note that an event associated with a thread is posted when a
    // message is queued to the message receive queue of the thread
    events = Event_pend(syncEvent, Event_Id_NONE, MR_ALL_EVENTS,
                        ICALL_TIMEOUT_FOREVER);
    
    

//              GAPRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
//                                DEFAULT_DISCOVERY_ACTIVE_SCAN,
//                                DEFAULT_DISCOVERY_WHITE_LIST);
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
      
//      if (events & MR_PERIODIC_EVT)
//      {
//        Util_startClock(&periodicClock);
//        
//        // Perform periodic application task
//        multi_role_performPeriodicTask();
//      }      
      
      if (events & SBP_PERIODIC_EVT)
      {
       events &= ~SBP_PERIODIC_EVT;
       
       //RED_LED13
       RLED++;
       PIN_setOutputValue(gpioPinHandle,CC2640R2_LAUNCHXL_PIN_RLED , RLED%2);
       
       
       adv_count++;
        if(adv_count == 5)
        {  
          memcpy(&tagsafe_advertdata[4], ownAddress, 6);
          tagsafe_advertdata[10] = BattD;

          GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData),tagsafe_advertdata,NULL);
          GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),&advopen,NULL);
          //Util_startClock(&moveClock);
          Util_stopClock(&resetClock);
        }
                
#ifdef I2c_Acceleration
        Acceleration_x=0;
        Acceleration_y=0;
        Acceleration_z=0;
        Acceler_x=0;
        Acceler_y=0;
        Acceler_z=0;
        uint8 first_address[6]={0x28,0x29,0x2a,0x2b,0x2c,0x2d};
        uint8 Read_Acceleration[6]={0x00};
        for (int x=0; x<6; x++){
        bspI2cWriteRead(&first_address[x],1,&Read_Acceleration[x],1);
        }
        Acceleration_x = (uint16)Read_Acceleration[1] << 8 | (uint16)Read_Acceleration[0]  ;
        Acceleration_x=Acceleration_x >>4;
        Acceleration_y = (uint16)Read_Acceleration[3] << 8 | (uint16)Read_Acceleration[2] ;
        Acceleration_y=Acceleration_y>>4;
        Acceleration_z = (uint16)Read_Acceleration[5] << 8 | (uint16)Read_Acceleration[4]; 
        Acceleration_z=Acceleration_z>>4;
        Acceleration_z=abs(Acceleration_z);
        
        Acceler_x = Acceleration_x/8;
        Acceler_y = Acceleration_y/8;
        Acceler_z = Acceleration_z/8;
        Accxyz = Acceler_x*Acceler_x+Acceler_y*Acceler_y+Acceler_z*Acceler_z;
        Acceler_xyz = sqrt(Accxyz);
   
        if(Acceler_xyz > 65520)
        {
          led_count = 0;
          //Util_startClock(&moveClock);
          GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),&advclose,NULL);
          PIN_setOutputValue(gpioPinHandle,CC2640R2DK_5XD_PIN_LED1 , 0);
          
          memset(scan_bNode_record,0,sizeof(scan_bNode_record));
          //memset(rssidata,0,sizeof(rssidata));
          memset(routedata,0,sizeof(routedata));
          
          for(int x=0; x<4; x++)
          {
            rssidata[x]=0;
          }
          
          Util_stopClock(&moveClock);
        } 
        else
        {
          //Util_stopClock(&accelerClock);
          Util_startClock(&moveClock);
          //GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(routedata),routedata,NULL);
//          GAPRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
//                                DEFAULT_DISCOVERY_ACTIVE_SCAN,
//                                DEFAULT_DISCOVERY_WHITE_LIST);
        }
#endif //I2c_Acceleration  
      }
   
//      if (events && SBP_LED_EVT)
//      {
//        events &= ~SBP_LED_EVT;
//        PIN_setOutputValue(gpioPinHandle,CC2640R2DK_5XD_PIN_LED1 , i); 
//        i = !i ;
//      }
      
      if (events & SBP_MOVE_EVT)
      {
        events &= ~SBP_MOVE_EVT;
        
          led_count++;
          RSSI_count++;
          //PIN_setOutputValue(hSbpPins, Board_LED_G, led_count%2); //設定綠燈亮  
        
          PIN_setOutputValue(gpioPinHandle,CC2640R2_LAUNCHXL_PIN_RLED , 0);
          PIN_setOutputValue(gpioPinHandle,CC2640R2_LAUNCHXL_PIN_GLED , i);
          i=!i;

          Voltage();
          GAPRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
                                DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                DEFAULT_DISCOVERY_WHITE_LIST);
          
         memcpy(&advertData[4], &ownAddress[0], 6);
         
         if(led_count == 80)
         {
            led_count = 0;
            adv_count = 0;
            RSSI_count = 0;
            PIN_setOutputValue(gpioPinHandle,CC2640R2_LAUNCHXL_PIN_GLED , 0);
            memset(scan_bNode_record,0,SCAN_BNODE_RECORD_SIZE * sizeof(bNode_data_t));
            memset(routedata,0,sizeof(routedata));
          for(int x=0; x<4; x++)
          {
            rssidata[x]=0;
          }
           Util_stopClock(&moveClock);
           Util_stopClock(&scanClock);
           Util_startClock(&resetClock);
           //Util_startClock(&safeClock);
         }
      }
      
//      if(events & Tagsafe_event)
//      {
//        events &= ~Tagsafe_event;    
//      }
      
       if(events & SBC_SCAN_EVT)
      {
        events &= ~SBC_SCAN_EVT;

       // PIN_setOutputValue(gpioPinHandle, CC2640R2DK_5XD_PIN_LED1, 1);
        
        for(index=0; index < SCAN_BNODE_RECORD_SIZE; index++)
        {
          if(scan_bNode_record[index].count != 0)
          {
            if(memcmp(scan_bNode_record[index].MacAddr,scan_Mac,6) == 0)
            {
                scan_bNode_record[index].count++;
                for(int x=0; x < 50; x++)
                {
                  if(scan_bNode_record[index].scanrssi[x] == 0)
                  {
                    scan_bNode_record[index].scanrssi[x] = dev_rssi;
                   /* for(int y=0; y<4; y++)
          {
            testingdata[y] = scan_bNode_record[index].scanrssi[x];
          }*/
                    break;
                  }
                  
                for(int y=0; y<4; y++){
                   rssidata[y] = scan_bNode_record[y].scanrssi[0];
                }
                  
                }
                scan_bNode_record[index].totalRSSI += dev_rssi;
                scan_bNode_record[index].averageRSSI =
                          scan_bNode_record[index].totalRSSI/scan_bNode_record[index].count;
               
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
              for(int x=0; x < 50; x++)
                {
                  if(scan_bNode_record[index].scanrssi[x] == 0)
                  {
                    scan_bNode_record[index].scanrssi[x] = dev_rssi;
                    break;
                  }

                for(int y=0; y<4; y++){
                  rssidata[y] = scan_bNode_record[y].scanrssi[0];
                }
                }
              scan_bNode_record[index].totalRSSI += dev_rssi;
              scan_bNode_record[index].averageRSSI =
                        scan_bNode_record[index].totalRSSI/scan_bNode_record[index].count;
              break;
            } 
          }
        }

//        if(scanning_status == SCANNING_STARTED)
//        {
//          scanning_status = SCANNING_STARTED;
//          GAPRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
//                                        DEFAULT_DISCOVERY_ACTIVE_SCAN,
//                                        DEFAULT_DISCOVERY_WHITE_LIST);
//        }
        
       
        for(int x=0; x<4; x++)
        {
          routedata[x+8] = abs(rssidata[x]);
        }

        char a[4] ={0x00}; 
        
        if( memcmp(a,&routedata[8], 4) != 0)
        {
          if(RSSI_count >= 55)
          {
            memcpy(&advertData[10], &routedata[8], 4);
            advertData[14] = BattD;
            GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData),advertData,NULL);
            GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),&advopen,NULL);
            RSSI_count = 0;
          }
          
        }      
      }
//      else if(events & SBC_SCAN_STARTED_EVT)
//      {
//        events &= ~SBC_SCAN_STARTED_EVT;
//        if(scanning_status == SCANNING_STARTED)
//        {
//          scanning_status = SCANNING_STARTED;
//          GAPRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
//                                        DEFAULT_DISCOVERY_ACTIVE_SCAN,
//                                        DEFAULT_DISCOVERY_WHITE_LIST);
//          Util_startClock(&boardcastClock);
//          Util_startClock(&accelerClock);
//        }
//      }
      
      
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
  uint8 bNode_parrent[9]={0x02, 0x01, 0x06, 0x1A, 0xFF, 0x4C, 0x00, 0x02, 0x15};
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

          if( /*0 == memcmp(bNode_parrent, pEvent->deviceInfo.pEvtData, sizeof(bNode_parrent))*/
             pEvent->deviceInfo.addr[5] == 0x0C &&
             pEvent->deviceInfo.addr[4] == 0x61
             /*(
             pEvent->deviceInfo.addr[1] == 0x15 &&
             pEvent->deviceInfo.addr[0] == 0x0E)  ||
             (
             pEvent->deviceInfo.addr[1] == 0x8E &&
             pEvent->deviceInfo.addr[0] == 0x0F ) ||
             (
             pEvent->deviceInfo.addr[1] == 0xA9 &&
             pEvent->deviceInfo.addr[0] == 0x0B ) ||
             (
             pEvent->deviceInfo.addr[1] == 0xB3 &&
             pEvent->deviceInfo.addr[0] == 0x02 )  */
               )
          {
            
            scan_num++;
            dev_rssi = pEvent->deviceInfo.rssi;
            memcpy(scan_Mac, pEvent->deviceInfo.addr, 6);
            
            Util_startClock(&scanClock);

            //Event_post(syncEvent, SBC_SCAN_EVT);

            GAPRole_CancelDiscovery();
            //scanning_status = SCANNING_STOPPED;
          }
          
          if( pEvent->deviceInfo.pEvtData[2] == 0x23 )
          {
            memcpy(position_data,pEvent->deviceInfo.pEvtData,pEvent->deviceInfo.dataLen);
            
            if(position_data[3] == 'o')
            {
              Lock_state = Unlock;
            }
            else if(position_data[3] == 'i')
            {
              Lock_state = Locked;
            }
            
            
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
    
    if( newValue3[0] == 0x2B)  //開鎖 "+"
    {
      Lock_state = Unlock;                  
    }
    else if (newValue3[0] == 0x2D) //上鎖 "-"
    {
      Util_startClock(&scanClock);
      //Lock_data = 0;
    }
    else if (newValue3[0] == 0x21)
    {
      Util_startClock(&moveClock);
    }
    else if (newValue3[0] == 0x22)
    {
      Util_stopClock(&moveClock);
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
    Blink(CC2640R2_LAUNCHXL_PIN_GLED, 5);
    Power_shutdown(NULL, 0); 
  }
  else{
    if( SysCtrlResetSourceGet() == RSTSRC_WAKEUP_FROM_SHUTDOWN )
    {
      Util_startClock(&moveClock);
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


/********************************************************/
/*
* @fn             Voltage
* @brief         Power conversion
*
*/
/********************************************************/
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
//  Solar_energyA=(int)Solar_energy*10;
//  Solar_energyD=8*((Solar_energyA-(Solar_energyA%10))/5)+(Solar_energyA%10);
//  //routeData[27]=Solar_energyD;


  //Batt
  batVol=AONBatMonBatteryVoltageGet();

  batVol_H=batVol>>8;
  batVol_HH=batVol_H;

  batVol_L=batVol;
  batVol_LL=batVol_L;

  Batt= batVol_LL/256+batVol_HH;
  BattA=Batt*10;
  BattD=8*((BattA-(BattA%10))/5)+(BattA%10);

}

/*********************************************************************
*********************************************************************/
