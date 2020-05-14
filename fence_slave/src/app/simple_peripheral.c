/******************************************************************************

 @file       simple_peripheral.c

 @brief This file contains the Simple Peripheral sample application for use
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
 Release Name: simplelink_cc2640r2_sdk_1_40_00_45
 Release Date: 2017-07-20 17:16:59
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include <string.h>
#include <math.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

/* driver header files */
#include <ti/drivers/ADCBuf.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTCC26XX.h>
#include <ti/drivers/pin/PINCC26xx.h>
#include <ti/display/Display.h>
#include <driverlib/aon_batmon.h>
#include <driverlib/aux_wuc.h>

//power management
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/devices/DeviceFamily.h>

#include DeviceFamily_constructPath(inc/hw_fcfg1.h)
#include DeviceFamily_constructPath(driverlib/aux_adc.h)

#include "hal_board.h"
#include "hci_tl.h"
#include "linkdb.h"
#include "gapgattserver.h"
#include "gapbondmgr.h"
#include "gatt.h"
#include "gattservapp.h"
#include "util.h"
#include "devinfoservice.h"
#include "simple_gatt_profile.h"
#include "ll_common.h"

#if defined(FEATURE_OAD) || defined(IMAGE_INVALIDATE)
#include "oad_target.h"
#include "oad.h"
#endif //FEATURE_OAD || IMAGE_INVALIDATE


#ifdef USE_RCOSC
#include "rcosc_calibration.h"
#endif //USE_RCOSC

#include "board_key.h"
#include "board.h"

#include "icall_api.h"
#include "icall_apimsg.h"

#include "simple_peripheral.h"
#include "peripheral.h"

//I2C
#include "bsp_i2c.h"

//2592
#include <driverlib/ioc.h>

//clock
#include "rtc_clock.h"

//Flash
#include "osal_snv.h"

//#include "CC2640R2DK_5XD.h"

/*********************************************************************
 * CONSTANTS
 */

// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160  //160

// General discoverable mode: advertise indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

#ifndef FEATURE_OAD
// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80 //160

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800 //240
#else //!FEATURE_OAD
// Minimum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     8 //160

// Maximum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     8 //240
#endif // FEATURE_OAD

// Slave latency to use for automatic parameter update request
#define DEFAULT_DESIRED_SLAVE_LATENCY         0 //5

// Supervision timeout value (units of 10ms, 1000=10s) for automatic parameter
// update request
#define DEFAULT_DESIRED_CONN_TIMEOUT          100  //600

// After the connection is formed, the peripheral waits until the central
// device asks for its preferred connection parameters
#define DEFAULT_ENABLE_UPDATE_REQUEST         GAPROLE_LINK_PARAM_UPDATE_INITIATE_BOTH_PARAMS

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         0 //6

// How often to perform periodic event (in msec)
#define SBP_PERIODIC_EVT_PERIOD               1000 //5000 

// Application specific event ID for HCI Connection Event End Events
#define SBP_HCI_CONN_EVT_END_EVT              0x0001
 
#ifdef FEATURE_OAD
// The size of an OAD packet.
#define OAD_PACKET_SIZE                       ((OAD_BLOCK_SIZE) + 2)
#endif // FEATURE_OAD

// Task configuration
#define SBP_TASK_PRIORITY                     1

#ifndef SBP_TASK_STACK_SIZE
#define SBP_TASK_STACK_SIZE                   644
#endif

#define SBP_STATE_CHANGE_EVT                  0x0001
#define SBP_CHAR_CHANGE_EVT                   0x0002
#define SBP_BTN_EVT                           0x0004

// Internal Events for RTOS application
#define SBP_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define SBP_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30
#define SBP_PERIODIC_EVT                      Event_Id_00
//ADD
#define init_event                            Event_Id_04
#define safe_event                            Event_Id_05
#define led_event                             Event_Id_06

// Bitwise OR of all events to pend on
#ifdef FEATURE_OAD
#define SBP_QUEUE_PING_EVT                    Event_Id_01

#define SBP_ALL_EVENTS                        (SBP_ICALL_EVT        | \
                                               SBP_QUEUE_EVT        | \
                                               SBP_PERIODIC_EVT     | \
                                               SBP_QUEUE_PING_EVT   | \
                                               SBP_BTN_EVT          | \
                                               init_event           | \
                                               safe_event           | \
                                               led_event  )
#else
#define SBP_ALL_EVENTS                        (SBP_ICALL_EVT        | \
                                               SBP_QUEUE_EVT        | \
                                               SBP_PERIODIC_EVT     | \
                                               SBP_BTN_EVT          | \
                                               init_event           | \
                                               safe_event           | \
                                               led_event  )
#endif /* FEATURE_OAD */

#define UartTimeout 100000 //unit in 10 us
/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr;  // event header.
  uint8_t *pData;  // event data
} sbpEvt_t;

#ifdef test

typedef struct
{
  uint8_t position[4];
  uint8_t RSSIrecord[100];
  int16_t totalRSSI;
  int8_t average;
  uint8_t count;
  int8_t MaxRS;
  int8_ MinRS;
}adv_node;

adv_node BikeRecord[50] ;

#endif 
/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
//Display_Handle dispHandle = NULL;

/*********************************************************************
 * LOCAL VARIABLES              本地變數
 */
//RTCTime times;
  /*
  RTCTimeStruct TimeConfig={
    .year   =2018,
    .month  =11,
    .day    =15,
    .hour   =15,
    .minutes=29,
    .seconds=0,
  };*/
  RTCTimeStruct TimeConfig;
  //uint32 userTime;
  RTCTime userTime;

//電量
int ADC_Result=0;
float Solar_energy;
int Solar_energyA;
int Solar_energyD;
uint8 Solar_Wave_Array[100]={0};
uint8_t batVol_H;
uint8_t batVol_L;
uint32_t batVol;
uint8_t Temp;
float batVol_LL;
float batVol_HH;
float Batt;
int BattA;
int BattD;

//Notify
uint16 notify_Handle; 

//Value
uint8 b[10]={0};
int led_count = 0;
static uint8_t i=0;
//Data
static int flashdata=0;
//static uint8_t routeData[31]={0X00}; 
static uint8_t newValue3[10]={0X00};
static uint8_t ioswritedata[10]={0X00};
//static uint8_t test2data[100]={0X00};
//uint8_t test1=0X43;
static uint8_t test2time[24]={0X00};


//廣播 adv
uint8_t advopen = TRUE;
uint8_t advclose = FALSE;
uint8_t timet[7]={0}; 

//UART
UART_Handle UART_handle;
static UART_Params UART_params;
Semaphore_Struct semUARTRxStruct;
Semaphore_Handle uartSemHandle;

static uint8_t ownAddress[B_ADDR_LEN];
static void Read_Mac(uint8 *Mac_Address); 

static PIN_State sbpPins; 
static PIN_Handle hSbpPins;
static uint8_t LED_value = 0;

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;
//static ICall_Semaphore sem;//old

// Clock instances for internal periodic events.
static Clock_Struct periodicClock;
static Clock_Struct PeriodicClock;

static Clock_Struct myClock_safeClock;
static Clock_Struct myClock_initClock;
static Clock_Struct myClock_ledClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

#if defined(FEATURE_OAD)
// Event data from OAD profile.
static Queue_Struct oadQ;
static Queue_Handle hOadQ;
#endif //FEATURE_OAD

// Task configuration
Task_Struct sbpTask;
Char sbpTaskStack[SBP_TASK_STACK_SIZE];

// Scan response data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
// complete name
  0x11,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE, 
  'F',
  'e',
  'n',
  'c',
  'e',
  '-',
  'S',
  'l',
  'a',
  'v',
  'e',
  
  // connection interval range
   0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),   // 100ms
  HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
  LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),   // 1s
  HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8_t advertData[] = {0x13, 0xFF, 0x40, 0x02, 0x00, 0x00, 0x00, 0x00,
                               0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                               0x00, 0x00, 0x00, 0x00, 0x02, 0x01, 0x06, 0x03, 
                               0x02, 0xF0};  //advertData[4] = 0x02 ->道釘
/*
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
#if !defined(FEATURE_OAD) || defined(FEATURE_OAD_ONCHIP)
  0x03,   // length of this data
#else //OAD for external flash
  0x05,  // lenght of this data
#endif //FEATURE_OAD
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
#ifdef FEATURE_OAD
  LO_UINT16(OAD_SERVICE_UUID),
  HI_UINT16(OAD_SERVICE_UUID),
#endif //FEATURE_OAD
#ifndef FEATURE_OAD_ONCHIP
  LO_UINT16(SIMPLEPROFILE_SERV_UUID),
  HI_UINT16(SIMPLEPROFILE_SERV_UUID)
#endif //FEATURE_OAD_ONCHIP
};*/

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Fence_Slave";

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;

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
void Dec_to_Hex(uint8_t dec, uint8_t *data)
{
  uint8_t byte;
  //High byte
  byte = (dec/10)+0X30;
  data[0] = byte;
  
  //Low byte
  byte = (dec%10)+0X30;
  data[1] = byte;
}
/*
void Hex_to_Dexinhex(uint8_t hex, uint *data)
{
  uint8_t byte;
  
  //High byte
  byte = 
}*/
//void Hex_to_Hex(uint8_t hex, uint8_t *data)
//{
//  uint8_t byte;
//  
//  //High byte
//  byte = hex >> 4;
//  data[0] = byte;
//  
//  //Low byte
//  byte = (hex & 0X0F);
//  data[1] = byte;
//  
//  data[2] = data[0]*10+data[1];
//  data[3] = (data[2]/10)*16 + (data[2]%10);
//}


/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void buttonHwiFxn(PIN_Handle hPin, PIN_Id pinId);

static void SimpleBLEPeripheral_init( void );
static void SimpleBLEPeripheral_taskFxn(UArg a0, UArg a1);

static uint8_t SimpleBLEPeripheral_processStackMsg(ICall_Hdr *pMsg);
static uint8_t SimpleBLEPeripheral_processGATTMsg(gattMsgEvent_t *pMsg);
static void SimpleBLEPeripheral_processAppMsg(sbpEvt_t *pMsg);
static void SimpleBLEPeripheral_processStateChangeEvt(gaprole_States_t newState);
static void SimpleBLEPeripheral_processCharValueChangeEvt(uint8_t paramID);
static void SimpleBLEPeripheral_performPeriodicTask(void);
static void SimpleBLEPeripheral_clockHandler(UArg arg);

static void SimpleBLEPeripheral_sendAttRsp(void);
static void SimpleBLEPeripheral_freeAttRsp(uint8_t status);

static void SimpleBLEPeripheral_stateChangeCB(gaprole_States_t newState);
#ifndef FEATURE_OAD_ONCHIP
static void SimpleBLEPeripheral_charValueChangeCB(uint8_t paramID);
#endif //!FEATURE_OAD_ONCHIP
static uint8_t SimpleBLEPeripheral_enqueueMsg(uint8_t event, uint8_t state,
                                              uint8_t *pData);
#ifdef FEATURE_OAD
void SimpleBLEPeripheral_processOadWriteCB(uint8_t event, uint16_t connHandle,
                                           uint8_t *pData);
#endif //FEATURE_OAD

//LOCAL FUNCTIONS

static void UARTreadCallback(UART_Handle handle, void *buf, size_t count);

static void Init_Solar_Energy(void)
{
    //intialisation of ADC

    // Enable clock for ADC digital and analog interface (not currently enabled in driver)
    AUXWUCClockEnable(AUX_WUC_ADI_CLOCK|AUX_WUC_SOC_CLOCK);   // was : AUX_WUC_MODCLKEN0_SOC_M|AUX_WUC_MODCLKEN0_AUX_ADI4_M);
    
    // Connect AUX IO7 (DIO23, but also DP2 on XDS110) as analog input.
    AUXADCSelectInput(ADC_COMPB_IN_AUXIO4);
     
    // Set up ADC range
    // AUXADC_REF_FIXED = nominally 4.3 V
    AUXADCEnableSync(AUXADC_REF_FIXED,  AUXADC_SAMPLE_TIME_2P7_US, AUXADC_TRIGGER_MANUAL);    
        
}
static void Time();
static void Voltage();
/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Peripheral GAPRole Callbacks
static gapRolesCBs_t SimpleBLEPeripheral_gapRoleCBs =
{
  SimpleBLEPeripheral_stateChangeCB     // GAPRole State Change Callbacks
};

// GAP Bond Manager Callbacks
// These are set to NULL since they are not needed. The application
// is set up to only perform justworks pairing.
static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs =
{
  NULL,//(pfnPasscodeCB_t) SimpleBLEPeripheral_passcodeCB, // Passcode callback
  NULL//SimpleBLEPeripheral_pairStateCB                   // Pairing / Bonding state Callback
};

// Simple GATT Profile Callbacks
#ifndef FEATURE_OAD_ONCHIP
static simpleProfileCBs_t SimpleBLEPeripheral_simpleProfileCBs =
{
  SimpleBLEPeripheral_charValueChangeCB // Characteristic value change callback
};
#endif //!FEATURE_OAD_ONCHIP

#ifdef FEATURE_OAD
static oadTargetCBs_t simpleBLEPeripheral_oadCBs =
{
  SimpleBLEPeripheral_processOadWriteCB // Write Callback.
};
#endif //FEATURE_OAD

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

//PIN
static void buttonHwiFxn(PIN_Handle hPin, PIN_Id pinId)
{

  SimpleBLEPeripheral_enqueueMsg(SBP_BTN_EVT, 0, NULL);

  // set event in SBP task to process outside of hwi context

  Event_post(syncEvent, SBP_PERIODIC_EVT); // Add

}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_createTask
 *
 * @brief   Task creation function for the Simple Peripheral.
 *
 * @param   None.
 *
 * @return  None.
 */
void SimpleBLEPeripheral_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sbpTaskStack;
  taskParams.stackSize = SBP_TASK_STACK_SIZE;
  taskParams.priority = SBP_TASK_PRIORITY;

  Task_construct(&sbpTask, SimpleBLEPeripheral_taskFxn, &taskParams, NULL);
}

/****************************PIN*/
PIN_Config SBP_configTable[] =
{

  CC2640R2_LAUNCHXL_PIN_RLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
//  CC2640R2_LAUNCHXL_PIN_BTN2 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,

  Solar      | PIN_GPIO_OUTPUT_DIS | PIN_INPUT_EN    | PIN_NOPULL,
  Battery    | PIN_GPIO_OUTPUT_DIS | PIN_INPUT_EN    | PIN_NOPULL,                     //IOID_10
//  NBIoT_RI   | PIN_GPIO_OUTPUT_DIS | PIN_INPUT_EN    | PIN_NOPULL,
//
//  LoRa_WakeUp| PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
//  LoRa_Sleep | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
//  LoRa_Set   | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,

  PIN_TERMINATE //PIN END
};
static PIN_State sbpPins;
static PIN_Handle hSbpPins;
/*********************************************************************
 * @fn      SimpleBLEPeripheral_init
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
static void SimpleBLEPeripheral_init(void)
{
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
 //UARTCC26XX_control(UART_handle, UARTCC26XX_CMD_RETURN_PARTIAL_ENABLE, NULL);

  //PIN
  // Open pin structure for use
  hSbpPins = PIN_open(&sbpPins, SBP_configTable);
  // Register ISR
  PIN_registerIntCb(hSbpPins, buttonHwiFxn);
  // Configure interrupt
  //PIN_setConfig(hSbpPins, PIN_BM_IRQ, Board_KEY_UP | PIN_IRQ_NEGEDGE);
  PIN_setConfig(hSbpPins, PIN_BM_IRQ, CC2640R2_LAUNCHXL_PIN_BTN2 | PIN_IRQ_NEGEDGE);
  // Enable wakeup
  //PIN_setConfig(hSbpPins, PINCC26XX_BM_WAKEUP, Board_KEY_UP|PINCC26XX_WAKEUP_NEGEDGE); 
  PIN_setConfig(hSbpPins, PINCC26XX_BM_WAKEUP, CC2640R2_LAUNCHXL_PIN_BTN2|PINCC26XX_WAKEUP_NEGEDGE);

  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  
  ICall_registerApp(&selfEntity, &syncEvent);

#ifdef USE_RCOSC
  RCOSC_enableCorrection();
#endif // USE_RCOSC

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

  // Map RFC_GPO0 to DIO13
  IOCPortConfigureSet(IOID_13, IOC_PORT_RFC_GPO0,
  IOC_IOMODE_NORMAL);
  // Map RFC_GPO1 to DIO7
  IOCPortConfigureSet(IOID_7, IOC_PORT_RFC_GPO1,
  IOC_IOMODE_NORMAL); 
   
  //db 調整 需在ICall_registerApp後
  //傳輸功率
  HCI_EXT_SetRxGainCmd(HCI_EXT_RX_GAIN_HIGH);
  HCI_EXT_SetTxPowerCmd( HCI_EXT_TX_POWER_5_DBM);
  
  RTC_init(); 
 
  //flash讀入
  osal_snv_read(0x81, 10, &ioswritedata[0]);
  //flashdata = ioswritedata[0];
  Read_Mac(ownAddress);
  
  
          TimeConfig.year     = 2018;
          TimeConfig.month    = test2time[2];
          TimeConfig.day      = test2time[7];
          TimeConfig.hour     = test2time[12];
          TimeConfig.minutes  = test2time[17];
          TimeConfig.seconds  = test2time[22];
  userTime = RTC_convertRTCSecs(&TimeConfig);
  RTC_setClock(userTime);
  
  //RTC_setClock(times);
  Time();

  //廣播名稱設定
  attDeviceName[0] ='F';
  attDeviceName[1] ='e';
  attDeviceName[2] ='n';
  attDeviceName[3] ='c';  
  attDeviceName[4] ='e';
  attDeviceName[5] ='-';
  attDeviceName[6] ='S';
  attDeviceName[7] ='l';
  attDeviceName[8] ='a';
  attDeviceName[9] ='v';
  attDeviceName[10] ='e';
  attDeviceName[11] ='-';
  Hex_to_Ascii(ownAddress[4],&attDeviceName[12]);
  Hex_to_Ascii(ownAddress[5],&attDeviceName[14]);

  

  // Hard code the BD Address till CC2650 board gets its own IEEE address
  //HCI_EXT_SetBDADDRCmd(bdAddress);

  // Set device's Sleep Clock Accuracy
  //HCI_EXT_SetSCACmd(40);
				    
  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);
  
  // Set GAP Parameters: After a connection was established, delay in seconds
  // before sending when GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE,...)
  // uses GAPROLE_LINK_PARAM_UPDATE_INITIATE_BOTH_PARAMS or
  // GAPROLE_LINK_PARAM_UPDATE_INITIATE_APP_PARAMS
  // For current defaults, this has no effect.
  GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);
  
  // Setup the Peripheral GAPRole Profile. For more information see the User's
  // Guide:
  {
    // Device starts advertising upon initialization of GAP
    uint8_t initialAdvertEnable = TRUE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until re-enabled by the application
    //uint16_t advertOffTime = 0;

    uint8_t enableUpdateRequest = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16_t desiredMinInterval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16_t desiredMaxInterval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16_t desiredSlaveLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16_t desiredConnTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the Peripheral GAPRole Parameters
    // Set the GAP Role Parameters
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &initialAdvertEnable);//開關
    //GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                         //&advertOffTime);//若為0設定為廣30.72
    
    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData),scanRspData);//廣播封包2

    GAPRole_SetParameter(GAPROLE_ADVERT_DATA,   sizeof(advertData), advertData);//廣播封包1  (原先註解)

    GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8_t),
                         &enableUpdateRequest);
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMinInterval);
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMaxInterval);
    GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t),
                         &desiredSlaveLatency);
    GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t),
                         &desiredConnTimeout);
  }

   // Set the Device Name characteristic in the GAP GATT Service
  // For more information, see the section in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-docs-latest/html/ble-stack/gaprole.html
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);
  
  
  // Set GAP Parameters to set the advertising interval
  // For more information, see the GAP section of the User's Guide:
  // http://software-dl.ti.com/lprf/sdg-latest/html
  {
    // Use the same interval for general and limited advertising.
    // Note that only general advertising will occur based on the above configuration
    uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
  }

  // Setup the GAP Bond Manager. For more information see the section in the
  // User's Guide:
  // http://software-dl.ti.com/lprf/sdg-latest/html/
  {
    // Don't send a pairing request after connecting; the peer device must
    // initiate pairing
    //uint32_t passkey = 0; // passkey "000000"//migrate
    uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    // Use authenticated pairing: require passcode.
    uint8_t mitm = TRUE;
    //uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8_t ioCap = GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT;
    // Request bonding (storing long-term keys for re-encryption upon subsequent
    // connections without repairing)
    uint8_t bonding = TRUE;

    //GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t),
    //                        &passkey);//migrate
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
  }

  // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);           // GAP GATT Service
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT Service
  DevInfo_AddService();                        // Device Information Service

#ifndef FEATURE_OAD_ONCHIP
  SimpleProfile_AddService(GATT_ALL_SERVICES); // Simple GATT Profile
#endif //!FEATURE_OAD_ONCHIP

#ifdef FEATURE_OAD
  VOID OAD_addService();                 // OAD Profile
  OAD_register((oadTargetCBs_t *)&simpleBLEPeripheral_oadCBs);
  hOadQ = Util_constructQueue(&oadQ);
#endif //FEATURE_OAD

#ifdef IMAGE_INVALIDATE
  Reset_addService();
#endif //IMAGE_INVALIDATE


#ifndef FEATURE_OAD_ONCHIP
  // Setup the SimpleProfile Characteristic Values
  {
    uint8_t charValue1 = 1;
    uint8_t charValue2 = 2;
    uint8_t charValue3 = 3;
    uint8_t charValue4 = 4;
    uint8_t charValue5[SIMPLEPROFILE_CHAR5_LEN] = {1, 2, 3, 4, 5};
    uint8_t charValue6[SIMPLEPROFILE_CHAR6_LEN] = {0};

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
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR6, SIMPLEPROFILE_CHAR6_LEN,
                               charValue6);
  }

  // Register callback with SimpleGATTprofile
  SimpleProfile_RegisterAppCBs(&SimpleBLEPeripheral_simpleProfileCBs);
#endif //!FEATURE_OAD_ONCHIP

  // Start the Device
  VOID GAPRole_StartDevice(&SimpleBLEPeripheral_gapRoleCBs);

  // Start Bond Manager
  VOID GAPBondMgr_Register(&simpleBLEPeripheral_BondMgrCBs);

  // Register with GAP for HCI/Host messages
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  //This should be included only if 4.2 length extension feature is enable....
  //HCI_LE_ReadMaxDataLenCmd();

#if !defined (USE_LL_CONN_PARAM_UPDATE)
  // Get the currently set local supported LE features
  // The HCI will generate an HCI event that will get received in the main
  // loop
  HCI_LE_ReadLocalSupportedFeaturesCmd();
#endif // !defined (USE_LL_CONN_PARAM_UPDATE)

  //初始LED燈設定    
  PIN_setOutputValue(hSbpPins, Board_LED_R,0);  //設定紅燈亮  
  PIN_setOutputValue(hSbpPins, Board_LED_G,0); //設定綠燈亮  
  PIN_setOutputValue(hSbpPins, Board_LED_B,0); //設定藍燈亮  
  
  Util_constructClock(&myClock_initClock,SimpleBLEPeripheral_clockHandler,1000,1000,false,init_event);
  Util_constructClock(&myClock_safeClock,SimpleBLEPeripheral_clockHandler,1000,1800000,false,safe_event);
  Util_constructClock(&myClock_ledClock,SimpleBLEPeripheral_clockHandler,1000,1000,false,led_event);
  //第一時間(由Util_startClock呼叫的計時時間),第二時間(自己跑這行的計時時間) ,FALSE(不跑，僅用Util_startClock()跑),最後為要跳的EVENT
  // 若為true， 則第一值為第一次觸發的時間，第二值為之後觸發的時間 //為false，Util_startClock觸發才能動作，第一值為第一次的觸發時間，第二值為之後觸發的時間，若為 0只觸發一次，若非0需使用stop才能停止。 

  Util_startClock(&myClock_initClock);
  //Util_startClock(&myClock_ledClock);
  GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),&advopen);

}
  
/*********************************************************************
 * @fn      SimpleBLEPeripheral_taskFxn
 *
 * @brief   Application task entry point for the Simple Peripheral.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  SimpleBLEPeripheral_init();
  bspI2cInit(); 

  // Application main loop
  for (;;)
  {
    uint32_t events;

    // Waits for an event to be posted associated with the calling thread.
    // Note that an event associated with a thread is posted when a
    // message is queued to the message receive queue of the thread
    events = Event_pend(syncEvent, Event_Id_NONE, SBP_ALL_EVENTS,
                        ICALL_TIMEOUT_FOREVER);
  Time();
  
    if (events)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      // Fetch any available messages that might have been sent from the stack
      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8 safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

          // Check for BLE stack events first
          if (pEvt->signature == 0xffff)
          {
            // The GATT server might have returned a blePending as it was trying
            // to process an ATT Response. Now that we finished with this
            // connection event, let's try sending any remaining ATT Responses
            // on the next connection event.
            if (pEvt->event_flag & SBP_HCI_CONN_EVT_END_EVT)
            {
              // Try to retransmit pending ATT Response (if any)
              SimpleBLEPeripheral_sendAttRsp();
            }
          }
          else
          {
            // Process inter-task message
            safeToDealloc = SimpleBLEPeripheral_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }
    
      // If RTOS queue is not empty, process app message.
      if (events & SBP_QUEUE_EVT)
      {
        while (!Queue_empty(appMsgQueue))
        {
          sbpEvt_t *pMsg = (sbpEvt_t *)Util_dequeueMsg(appMsgQueue);
          if (pMsg)
          {
            // Process message.
            SimpleBLEPeripheral_processAppMsg(pMsg);

            // Free the space from the message.
            ICall_free(pMsg);
          }
        }
      }
      
      if (events & SBP_PERIODIC_EVT)
      {
          Util_startClock(&periodicClock);

          // Perform periodic application task
          SimpleBLEPeripheral_performPeriodicTask();
      }

      if (events & init_event)
      {
        events &= ~init_event;
    
        i=!i;
        //PIN_setOutputValue(hSbpPins, Board_LED_G,i); //設定綠燈亮
        PIN_setOutputValue(hSbpPins, Board_LED_R,i);
        PIN_setOutputValue(hSbpPins, Board_LED_B,i);
        
        if(flashdata ==1)
        {
          //PIN_setOutputValue(hSbpPins, Board_LED_G,0);
          PIN_setOutputValue(hSbpPins, Board_LED_R,0);
          PIN_setOutputValue(hSbpPins, Board_LED_B,0);
          Util_stopClock(&myClock_initClock);
          Util_startClock(&myClock_safeClock);
          //Util_startClock(&myClock_ledClock);
        }
        
      }
      if (events & safe_event)
      {
        events &= ~safe_event;
        
        Voltage();
        
        for(int x=0; x<6; x++)
        {
          advertData[x+4] = ownAddress[x];
        }
        advertData[10]=BattD;
        memcpy(&advertData[11], &ioswritedata[1], 9);
        
        GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData),advertData);
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),&advopen);
        
        Util_startClock(&myClock_ledClock);
        
      }
      
      if(events & led_event)
      {
        events &= ~led_event;
        
        led_count++;
        i=!i;
        PIN_setOutputValue(hSbpPins, Board_LED_G,i);
       
        if(led_count >=30)
        {
          led_count = 0;
          PIN_setOutputValue(hSbpPins, Board_LED_G,0);
          Util_stopClock(&myClock_ledClock);
        }
        
      }
      
      
} 
}
#ifdef FEATURE_OAD
      if (events & SBP_QUEUE_PING_EVT)
      {
        while (!Queue_empty(hOadQ))
        {
          oadTargetWrite_t *oadWriteEvt = Queue_get(hOadQ);

          // Identify new image.
          if (oadWriteEvt->event == OAD_WRITE_IDENTIFY_REQ)
          {
            OAD_imgIdentifyWrite(oadWriteEvt->connHandle, oadWriteEvt->pData);
          }
          // Write a next block request.
          else if (oadWriteEvt->event == OAD_WRITE_BLOCK_REQ)
          {
            OAD_imgBlockWrite(oadWriteEvt->connHandle, oadWriteEvt->pData);
          }

          // Free buffer.
          ICall_free(oadWriteEvt);
        }
      }
#endif //FEATURE_OAD    
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimpleBLEPeripheral_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = SimpleBLEPeripheral_processGATTMsg((gattMsgEvent_t *)pMsg);
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
              // The L2CAP Connection Parameter Update procedure is used to
              // support a delta between the minimum and maximum connection
              // intervals required by some iOS devices.

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

                      // Get current feature set from received event (bits 1-9
                      // of the returned data
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

      default:
        // do nothing
        break;

    }

  return (safeToDealloc);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimpleBLEPeripheral_processGATTMsg(gattMsgEvent_t *pMsg)
{
  // See if GATT server was unable to transmit an ATT response
  if (pMsg->hdr.status == blePending)
  {
    // No HCI buffer was available. Let's try to retransmit the response
    // on the next connection event.
    if (HCI_EXT_ConnEventNoticeCmd(pMsg->connHandle, selfEntity,
                                   SBP_HCI_CONN_EVT_END_EVT) == SUCCESS)
    {
      // First free any pending response
      SimpleBLEPeripheral_freeAttRsp(FAILURE);

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
  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
  }

  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_sendAttRsp
 *
 * @brief   Send a pending ATT response message.
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleBLEPeripheral_sendAttRsp(void)
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
      SimpleBLEPeripheral_freeAttRsp(status);
    }
    else
    {
      // Continue retrying
    }
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_freeAttRsp
 *
 * @brief   Free ATT response message.
 *
 * @param   status - response transmit status
 *
 * @return  none
 */
static void SimpleBLEPeripheral_freeAttRsp(uint8_t status)
{
  // See if there's a pending ATT response message
  if (pAttRsp != NULL)
  {
    // See if the response was sent out successfully
    if (status == SUCCESS)
    {
    }
    else
    {
      // Free response payload
      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);
    }

    // Free response message
    ICall_freeMsg(pAttRsp);

    // Reset our globals
    pAttRsp = NULL;
    rspTxRetry = 0;
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_processAppMsg(sbpEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case SBP_STATE_CHANGE_EVT:
      {
        SimpleBLEPeripheral_processStateChangeEvt((gaprole_States_t)pMsg->
                                                hdr.state);
      }
      break;

    case SBP_CHAR_CHANGE_EVT:
      {
        SimpleBLEPeripheral_processCharValueChangeEvt(pMsg->hdr.state);
      }
      break;
      
    case SBP_BTN_EVT:
      //toggle red LED
      if (LED_value)
      {
        PIN_setOutputValue(hSbpPins, CC2640R2_LAUNCHXL_PIN_RLED , LED_value--);
      }
      else
      {
        PIN_setOutputValue(hSbpPins, CC2640R2_LAUNCHXL_PIN_RLED, LED_value++);
      }
      break;
      
    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_stateChangeCB
 *
 * @brief   Callback from GAP Role indicating a role state change.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_stateChangeCB(gaprole_States_t newState)
{
  SimpleBLEPeripheral_enqueueMsg(SBP_STATE_CHANGE_EVT, newState, 0);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processStateChangeEvt
 *
 * @brief   Process a pending GAP Role state change event.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_processStateChangeEvt(gaprole_States_t newState)
{
#ifdef PLUS_BROADCASTER
  static bool firstConnFlag = false;
#endif // PLUS_BROADCASTER

  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        //uint8_t ownAddress[B_ADDR_LEN];
        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

        //GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
      }
      break;

    case GAPROLE_ADVERTISING:
      break;

#ifdef PLUS_BROADCASTER
    // After a connection is dropped, a device in PLUS_BROADCASTER will continue
    // sending non-connectable advertisements and shall send this change of
    // state to the application.  These are then disabled here so that sending
    // connectable advertisements can resume.
    case GAPROLE_ADVERTISING_NONCONN:
      {
        uint8_t advertEnabled = FALSE;

        // Disable non-connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                           &advertEnabled);

        advertEnabled = TRUE;

        // Enabled connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                             &advertEnabled);

        // Reset flag for next connection.
        firstConnFlag = false;

        SimpleBLEPeripheral_freeAttRsp(bleNotConnected);
      }
      break;
#endif //PLUS_BROADCASTER

    case GAPROLE_CONNECTED:
      {
        linkDBInfo_t linkInfo;
        uint8_t numActive = 0;

        Util_startClock(&periodicClock);

        numActive = linkDB_NumActive();

        // Use numActive to determine the connection handle of the last
        // connection
        if ( linkDB_GetInfo( numActive - 1, &linkInfo ) == SUCCESS )
        {
        }
        else
        {
          uint8_t peerAddress[B_ADDR_LEN];

          GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);
        }

        #ifdef PLUS_BROADCASTER
          // Only turn advertising on for this state when we first connect
          // otherwise, when we go from connected_advertising back to this state
          // we will be turning advertising back on.
          if (firstConnFlag == false)
          {
            uint8_t advertEnabled = FALSE; // Turn on Advertising

            // Disable connectable advertising.
            GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);

            // Set to true for non-connectable advertising.
            advertEnabled = TRUE;

            // Enable non-connectable advertising.
            GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);
            firstConnFlag = true;
          }
        #endif // PLUS_BROADCASTER
      }
      break;

    case GAPROLE_CONNECTED_ADV:
      break;

    case GAPROLE_WAITING:
      Util_stopClock(&periodicClock);
      SimpleBLEPeripheral_freeAttRsp(bleNotConnected);
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      SimpleBLEPeripheral_freeAttRsp(bleNotConnected);

      #ifdef PLUS_BROADCASTER
        // Reset flag for next connection.
        firstConnFlag = false;
      #endif // PLUS_BROADCASTER
      break;

    case GAPROLE_ERROR:
      break;

    default:
      break;
  }

}

#ifndef FEATURE_OAD_ONCHIP
/*********************************************************************
 * @fn      SimpleBLEPeripheral_charValueChangeCB
 *
 * @brief   Callback from Simple Profile indicating a characteristic
 *          value change.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_charValueChangeCB(uint8_t paramID)
{
  SimpleBLEPeripheral_enqueueMsg(SBP_CHAR_CHANGE_EVT, paramID, 0);
}
#endif //!FEATURE_OAD_ONCHIP

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processCharValueChangeEvt
 *
 * @brief   Process a pending Simple Profile characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_processCharValueChangeEvt(uint8_t paramID)
{
#ifndef FEATURE_OAD_ONCHIP
  uint8_t newValue;

  switch(paramID)
  {
    case SIMPLEPROFILE_CHAR1:
      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR1, &newValue);

      break;

    case SIMPLEPROFILE_CHAR3:
      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR3, &newValue3);
      
      //重啟
      if(newValue3[0]=='r' && newValue3[1]=='e' && newValue3[2]=='s'  &&  newValue3[3]=='e'  && newValue3[4]=='t')  //0x72 0x65 0x73 0x65 0x74
      {
      HAL_SYSTEM_RESET();
      }
      
      //設定參數
        if( newValue3[0]=='!' ) //假經緯度 //0x21 
      {
        ioswritedata[0] = 1;
//        ioswritedata[1] = newValue3[1];
//        ioswritedata[2] = newValue3[2];
//        ioswritedata[3] = newValue3[3];
//        ioswritedata[4] = newValue3[4];
//        ioswritedata[5] = newValue3[5];
//        ioswritedata[6] = newValue3[6];
//        ioswritedata[7] = newValue3[7];
//        ioswritedata[8] = newValue3[8];
//        ioswritedata[9] = newValue3[9];
        
        memcpy(&ioswritedata[1], &newValue3[1], 9); 
        osal_snv_write(0x81, 10, &ioswritedata[0]);
        flashdata = ioswritedata[0];
        /*
          Ioswritedata[0] =            1 ; //flash
          
          memcpy(&Ioswritedata[1],&newValue3[1],19);
          osal_snv_write(0x80, 20, &Ioswritedata[0]);
          Hex_to_Dec(Ioswritedata[2],&test2time[0]);
          Hex_to_Dec(Ioswritedata[3],&test2time[5]);
          Hex_to_Dec(Ioswritedata[4],&test2time[10]);
          Hex_to_Dec(Ioswritedata[5],&test2time[15]);
          Hex_to_Dec(Ioswritedata[6],&test2time[20]);
          
          TimeConfig.year     = 2018;
          TimeConfig.month    = test2time[2];
          TimeConfig.day      = test2time[7];
          TimeConfig.hour     = test2time[12];
          TimeConfig.minutes  = test2time[17];
          TimeConfig.seconds  = test2time[22];
          userTime = RTC_convertRTCSecs(&TimeConfig);
          RTC_setClock(userTime);
          
          flashdata = Ioswritedata[0];
          test2data[15]=test2time[14];
          test2data[16]=test2time[15];
          test2data[17]=test2time[19];
          test2data[18]=test2time[20]; 
          Hex_to_Ascii(Ioswritedata[10],&test2data[19]);
          Hex_to_Ascii(Ioswritedata[11],&test2data[21]);
          Hex_to_Ascii(Ioswritedata[14],&test2data[23]);
          Hex_to_Ascii(Ioswritedata[15],&test2data[25]);
          Hex_to_Ascii(Ioswritedata[16],&test2data[27]);
          test2data[29] = Ioswritedata[17];
          Hex_to_Ascii(Ioswritedata[18],&test2data[30]);
          Hex_to_Ascii(Ioswritedata[19],&test2data[32]);
          memcpy(&routeData[8],&Ioswritedata[5],4);
          memcpy(&routeData[12],&Ioswritedata[14],6);
*/
       }                       

      break;
      
      case SIMPLEPROFILE_CHAR6:

      GAPRole_GetParameter(GAPROLE_CONNHANDLE,&notify_Handle);
      
      break;
      
      
    default:
      // should not reach here!
      break;
  }
#endif //!FEATURE_OAD_ONCHIP
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets called
 *          every five seconds (SBP_PERIODIC_EVT_PERIOD). In this example,
 *          the value of the third characteristic in the SimpleGATTProfile
 *          service is retrieved from the profile, and then copied into the
 *          value of the the fourth characteristic.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_performPeriodicTask(void)
{
#ifndef FEATURE_OAD_ONCHIP

  uint8_t valueToCopy;

  // Call to retrieve the value of the third characteristic in the profile
  if (SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR3, &valueToCopy) == SUCCESS)
  {
    // Call to set that value of the fourth characteristic in the profile.
    // Note that if notifications of the fourth characteristic have been
    // enabled by a GATT client device, then a notification will be sent
    // every time this function is called.
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint8_t),
                               &valueToCopy);
  }
#endif //!FEATURE_OAD_ONCHIP
}

#ifdef FEATURE_OAD
/*********************************************************************
 * @fn      SimpleBLEPeripheral_processOadWriteCB
 *
 * @brief   Process a write request to the OAD profile.
 *
 * @param   event      - event type:
 *                       OAD_WRITE_IDENTIFY_REQ
 *                       OAD_WRITE_BLOCK_REQ
 * @param   connHandle - the connection Handle this request is from.
 * @param   pData      - pointer to data for processing and/or storing.
 *
 * @return  None.
 */
void SimpleBLEPeripheral_processOadWriteCB(uint8_t event, uint16_t connHandle,
                                           uint8_t *pData)
{
  oadTargetWrite_t *oadWriteEvt = ICall_malloc( sizeof(oadTargetWrite_t) + \
                                             sizeof(uint8_t) * OAD_PACKET_SIZE);

  if ( oadWriteEvt != NULL )
  {
    oadWriteEvt->event = event;
    oadWriteEvt->connHandle = connHandle;

    oadWriteEvt->pData = (uint8_t *)(&oadWriteEvt->pData + 1);
    memcpy(oadWriteEvt->pData, pData, OAD_PACKET_SIZE);

    Queue_put(hOadQ, (Queue_Elem *)oadWriteEvt);

    // Post the application's event.  For OAD, no event flag is used.
    Event_post(syncEvent, SBP_QUEUE_PING_EVT);
  }
  else
  {
    // Fail silently.
  }
}
#endif //FEATURE_OAD

/*********************************************************************
 * @fn      SimpleBLEPeripheral_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_clockHandler(UArg arg)
{
  // Wake up the application.
  Event_post(syncEvent, arg);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 * @param   pData - message data pointer.
 *
 * @return  TRUE or FALSE
 */
static uint8_t SimpleBLEPeripheral_enqueueMsg(uint8_t event, uint8_t state,
                                           uint8_t *pData)
{
  sbpEvt_t *pMsg = ICall_malloc(sizeof(sbpEvt_t));

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

/*********************************************************************/
/*
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
/*********************************************************************/


static void Time()
{
  userTime = RTC_getClock();
  RTC_convertRTCTime(&TimeConfig, userTime);

//  routeData[8] = TimeConfig.minutes;
//  routeData[9] = TimeConfig.seconds;
//  Dec_to_Hex(TimeConfig.minutes,&test3[0]);
//  Dec_to_Hex(TimeConfig.seconds,&test3[2]);
//  test2data[15]=test3[0];
//  test2data[16]=test3[1];
//  test2data[17]=test3[2];
//  test2data[18]=test3[3];
  //Hex_to_Ascii(TimeConfig.minutes,&test2data[15]);
  //Hex_to_Ascii(TimeConfig.seconds,&test2data[17]);
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
  Init_Solar_Energy();
  AUXADCGenManualTrigger();
  Solar_energy = AUXADCReadFifo();//讀到的值 最大4096
  AUXADCFlushFifo();//清空flash
  AUXADCDisable();//關閉ADC

  Solar_energy=((Solar_energy/4096)*4.3); //電壓轉換

  Solar_energyA=(int)Solar_energy*10;
  Solar_energyD=8*((Solar_energyA-(Solar_energyA%10))/5)+(Solar_energyA%10);
  //routeData[27]=Solar_energyD;


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

/******************************************************************************
********************************************************************************/
